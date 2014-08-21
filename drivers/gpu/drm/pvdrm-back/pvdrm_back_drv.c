/*
  Copyright (C) 2014 Yusuke Suzuki <utatane.tea@gmail.com>

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <linux/atomic.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>
#include <xen/grant_table.h>

#include <xen/interface/memory.h>
#include <xen/interface/grant_table.h>
#include <xen/interface/io/protocols.h>

#include <asm/xen/hypervisor.h>

#include "drmP.h"

#include "../pvdrm/pvdrm_slot.h"

typedef struct {
	spinlock_t req_lock;
	wait_queue_head_t req;
} pvdrm_back_core_t;

static pvdrm_back_core_t* pvdrm_back_core;

struct pvdrm_back_device {
	struct xenbus_device* xbdev;
	struct task_struct* thread;
	struct file* filp;

	grant_ref_t ref;
	struct pvdrm_mapped* mapped;
};

static uint64_t pvdrm_back_count(struct pvdrm_back_device* info)
{
	return atomic_read(&info->mapped->count);
}

static struct pvdrm_slot* claim_slot(struct pvdrm_back_device* info)
{
	uint32_t id;
	atomic_dec(&info->mapped->count);
	id = info->mapped->ring[info->mapped->get++ % PVDRM_SLOT_NR];
	return &info->mapped->slot[id];
}

static int process_slot(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	int ret;
	int ioctl_ret = 0;
        struct drm_file* file_priv = NULL;
        struct drm_device* dev = NULL;
	mm_segment_t fs;

	ret = 0;
	file_priv = info->filp->private_data;
	dev = file_priv->minor->dev;

	fs = get_fs();
	set_fs(get_ds());
	/* Processing slot. */
	switch (slot->code) {
	case PVDRM_IOCTL_NOUVEAU_GETPARAM:
		/* TODO:(Yusuke Suzuki) Instead of this, register ioctl from each drivers or call drm_ioctl. */
		ioctl_ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GETPARAM, (unsigned long)pvdrm_slot_payload(slot));
		break;
	}
	set_fs(fs);

	slot->ret = ioctl_ret;

	/* Emit fence. */
	pvdrm_fence_emit(&slot->__fence, 42);
	return ret;
}

static int thread_main(void *arg)
{

	int ret;
	struct pvdrm_back_device* info;

	ret = 0;
	info = arg;

	/* Kick state. */
	printk(KERN_INFO "Starting PVDRM backend thread.\n");
	xenbus_switch_state(info->xbdev, XenbusStateConnected);
	printk(KERN_INFO "PVDRM backend thread connected.\n");

	{
		void* addr = NULL;

		ret = xenbus_scanf(XBT_NIL, info->xbdev->otherend, "counter-ref", "%u", &info->ref);
		if (ret < 0) {
			xenbus_dev_fatal(info->xbdev, ret, "reading counter-ref");
			return ret;
                }
		printk(KERN_INFO "PVDRM: mapping %u.\n", info->ref);

		ret = xenbus_map_ring_valloc(info->xbdev, info->ref, &addr);
		if (ret) {
			xenbus_dev_fatal(info->xbdev, ret, "mapping counter-ref");
			return ret;
		}
		info->mapped = addr;
		printk(KERN_INFO "PVDRM: sizeof mapped id is %u.\n", pvdrm_slot_id(info->mapped, &info->mapped->slot[31]));
	}

        /* Open DRM file. */
	{
		mm_segment_t fs;
		fs = get_fs();
		set_fs(get_ds());
		/* FIXME: Currently we use this path directly. */
		info->filp = filp_open("/dev/dri/card0", O_RDWR, 0);
		set_fs(fs);

		printk(KERN_INFO "PVDRM: Opened drm device.\n");
        }

	printk(KERN_INFO "PVDRM: Start main loop.\n");
	while (true) {
		while (!kthread_should_stop() && !pvdrm_back_count(info)) {
			/* Sleep. */
			ktime_t time;
			__set_current_state(TASK_INTERRUPTIBLE);
			time = ktime_set(0, 200);  /* This value derived from Paradice [ASPLOS '14]. */
			schedule_hrtimeout(&time, HRTIMER_MODE_REL);
		}

		if (kthread_should_stop()) {
			break;
		}

		if (pvdrm_back_count(info)) {
			struct pvdrm_slot* slot = claim_slot(info);
			process_slot(info, slot);
		}
	}
	printk(KERN_INFO "PVDRM: End main loop.\n");

	return 0;
}

static int pvdrm_back_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	int ret;
	struct pvdrm_back_device* info;

	printk(KERN_INFO "Proving PVDRM backend driver.\n");

	info = kzalloc(sizeof(struct pvdrm_back_device), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}
	info->xbdev = xbdev;
	dev_set_drvdata(&xbdev->dev, info);

	ret = xenbus_switch_state(xbdev, XenbusStateInitWait);
	if (ret) {
		printk(KERN_INFO "PVDRM failed");
		return ret;
	}
	return 0;
}

static int pvdrm_back_remove(struct xenbus_device *xbdev)
{
	struct pvdrm_back_device* info = NULL;

	printk(KERN_INFO "Removing PVDRM backend driver.\n");

	info = dev_get_drvdata(&xbdev->dev);
	kfree(info);

	return 0;
}

static void frontend_changed(struct xenbus_device *xbdev, enum xenbus_state frontend_state)
{
	struct pvdrm_back_device* info;
	int ret = 0;

	printk(KERN_INFO "Frontend changed PVDRM backend driver to state %s.\n", xenbus_strstate(frontend_state));

	info = dev_get_drvdata(&xbdev->dev);

	printk(KERN_INFO "%s", xenbus_strstate(frontend_state));

	switch (frontend_state) {
	case XenbusStateInitialising:
		if (xbdev->state == XenbusStateClosed) {
			printk(KERN_INFO "PVDRM xenbus is closed...\n");
			xenbus_switch_state(xbdev, XenbusStateInitWait);
		}
		break;

	case XenbusStateInitialised:
		break;

	case XenbusStateConnected:
		if (xbdev->state == XenbusStateConnected) {
			break;
		}

		/* OK, connect it. */
		/* TODO: Implement it */
		info->thread = kthread_run(thread_main, (void*)info, "pvdrm-back");
		break;

	case XenbusStateClosing:
		xenbus_switch_state(xbdev, XenbusStateClosing);
		break;

	case XenbusStateClosed:
		/* TODO: Implement it */
		xenbus_switch_state(xbdev, XenbusStateClosed);
		if (xenbus_dev_is_online(xbdev)) {
			break;
		}
		if (info->thread) {
			kthread_stop(info->thread);
			info->thread = NULL;
		}

		/* fall through if not online */
	case XenbusStateUnknown:
		/* implies xen_blkif_disconnect() via xen_blkbk_remove() */
		device_unregister(&xbdev->dev);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret) {
		xenbus_dev_fatal(xbdev, ret, "saw state %d at frontend", frontend_state);
	}
}

static const struct xenbus_device_id pvdrm_back_ids[] = {
	{ "vdrm" },
	{ "" }
};

static DEFINE_XENBUS_DRIVER(pvdrm_back, ,
	.probe = pvdrm_back_probe,
	.remove = pvdrm_back_remove,
	.otherend_changed = frontend_changed
);

static int __init pvdrm_back_init(void)
{
	/* int i, threads; */

	if (!xen_domain())
		return -ENODEV;

	pvdrm_back_core = kzalloc(sizeof(pvdrm_back_core_t), GFP_KERNEL);
	if (!pvdrm_back_core) {
		BUG();
		return -ENOMEM;
	}

#if 0
	threads = num_online_cpus();
	for (i = 0; i < threads; ++i) {
	}
#endif

	spin_lock_init(&pvdrm_back_core->req_lock);
	init_waitqueue_head(&pvdrm_back_core->req);

	printk(KERN_INFO "Initialising PVDRM backend driver.\n");

	return xenbus_register_backend(&pvdrm_back_driver);
}
module_init(pvdrm_back_init);

static void __exit pvdrm_back_exit(void)
{
	xenbus_unregister_driver(&pvdrm_back_driver);
}
module_exit(pvdrm_back_exit);

MODULE_AUTHOR("Yusuke Suzuki");
MODULE_DESCRIPTION("PVDRM backend driver");
MODULE_LICENSE("GPL");
/* vim: set sw=8 ts=8 et tw=80 : */
