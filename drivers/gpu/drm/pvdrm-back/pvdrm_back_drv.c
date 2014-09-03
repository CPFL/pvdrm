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
#include <linux/delay.h>
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
	uint32_t pos;
	atomic_dec(&info->mapped->count);
	pos = ((uint32_t)atomic_add_return(1, &info->mapped->get)) % PVDRM_SLOT_NR;
	id = info->mapped->ring[pos];
	return &info->mapped->slot[id];
}

struct copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
};

static int transfer(struct copier* copier, struct pvdrm_slot* slot, uint8_t* addr)
{
	if (slot->u.transfer.nr_buffers) {
		/* FIXME: Check buffer size doesn't exceed for the security issues. */
		const size_t size = slot->u.transfer.nr_buffers * sizeof(struct drm_nouveau_gem_pushbuf_bo);
		memcpy(copier->buffers, addr, size);
		addr += size;
		copier->buffers += slot->u.transfer.nr_buffers;
	}

	if (slot->u.transfer.nr_relocs) {
		const size_t size = slot->u.transfer.nr_relocs * sizeof(struct drm_nouveau_gem_pushbuf_reloc);
		memcpy(copier->relocs, addr, size);
		addr += size;
		copier->relocs += slot->u.transfer.nr_relocs;
	}

	if (slot->u.transfer.nr_push) {
		const size_t size = slot->u.transfer.nr_push * sizeof(struct drm_nouveau_gem_pushbuf_push);
		memcpy(copier->push, addr, size);
		addr += size;
		copier->push += slot->u.transfer.nr_push;
	}

	return 0;
}

static int process_pushbuf(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	/* buffers, reloc, push is user space pointers of the guest domain. */
	int ret = 0;
	struct drm_nouveau_gem_pushbuf* req = pvdrm_slot_payload(slot);
	struct drm_nouveau_gem_pushbuf_bo* buffers = NULL;
	struct drm_nouveau_gem_pushbuf_reloc* relocs = NULL;
	struct drm_nouveau_gem_pushbuf_push* push = NULL;
	void* addr = NULL;

	printk(KERN_INFO "PVDRM: pushbuf with ref %d\n", slot->u.transfer.ref);
	if (slot->u.transfer.ref < 0 || req->nr_push == 0) {
		/* OK, there's no buffers. */
		printk(KERN_INFO "PVDRM: pushbuf with no buffers...\n");
		/* FIXME: Check parameter is valid. */
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));
		goto destroy_data;
	}

	if (req->nr_buffers && req->buffers) {
		if (req->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS) {
			return -EINVAL;
		}
		buffers = kmalloc(sizeof(struct drm_nouveau_gem_pushbuf_bo) * req->nr_buffers, GFP_KERNEL);
		if (!buffers) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	if (req->nr_relocs && req->relocs) {
		if (req->nr_relocs > NOUVEAU_GEM_MAX_RELOCS) {
			ret = -EINVAL;
			goto destroy_data;
		}
		relocs = kmalloc(sizeof(struct drm_nouveau_gem_pushbuf_reloc) * req->nr_relocs, GFP_KERNEL);
		if (!relocs) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	if (req->nr_push && req->push) {
		if (req->nr_push > NOUVEAU_GEM_MAX_PUSH) {
			ret = -EINVAL;
			goto destroy_data;
		}
		push = kmalloc(sizeof(struct drm_nouveau_gem_pushbuf_push) * req->nr_push, GFP_KERNEL);
		if (!push) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	ret = xenbus_map_ring_valloc(info->xbdev, slot->u.transfer.ref, &addr);
	if (ret) {
		goto destroy_data;
	}

	/* Copying data to the host side. */
	{
		int next = 0;
		struct copier copier = {
			.buffers = buffers,
			.relocs = relocs,
			.push = push,
		};
		printk(KERN_INFO "PVDRM: Copying pushbufs...\n");
		do {
			printk(KERN_INFO "PVDRM: Copy! pushbuf...\n");
			ret = transfer(&copier, slot, addr);
			if (ret) {
				goto unmap_ref;
			}
			next = slot->u.transfer.next;
			if (next > 0) {
				pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
				pvdrm_fence_wait(&slot->__fence, 1, false);
			}
		} while (next > 0);
		printk(KERN_INFO "PVDRM: Copying pushbuf... Done.\n");
	}

	req->buffers = (unsigned long)buffers;
	req->relocs = (unsigned long)relocs;
	req->push = (unsigned long)push;

	ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));

unmap_ref:
	if (addr) {
		xenbus_unmap_ring_vfree(info->xbdev, addr);
	}

destroy_data:
	if (buffers) {
		kfree(buffers);
	}
	if (relocs) {
		kfree(relocs);
	}
	if (push) {
		kfree(push);
	}

	return ret;
}

static int process_mmap(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	/* Call f_op->mmap operation directly. */
	int i;
	int ret = 0;
	struct drm_pvdrm_gem_mmap* req = pvdrm_slot_payload(slot);
	struct vm_area_struct* vma = NULL;
	struct vm_fault vmf = { 0 };
	pte_t* ptes = NULL;
	struct vm_struct* area = NULL;
	void* addr = NULL;
	uint64_t size = 0;
	grant_ref_t head;
	uint32_t pages = 0;

	size = (req->vm_end - req->vm_start);
	pages = size / PAGE_SIZE;

	if (pages > PVDRM_MMAP_MAX_PAGES_PER_ONE_CALL) {
		return -EINVAL;
	}

	area = alloc_vm_area(size, &ptes);
	if (!area) {
		BUG();
	}
	addr = area->addr;
	printk(KERN_INFO "PVDRM:allocated area addresss size%lu 0x%llx, %u, %lu, 0x%llx | 0x%llx.\n", size, area->addr, area->nr_pages, area->size, info->mapped);

	vma = kmalloc(sizeof(*vma), GFP_KERNEL);
	if (!vma) {
		BUG();
	}

	vma->vm_mm = current->active_mm;
	vma->vm_start = (unsigned long)addr;
	vma->vm_end = ((unsigned long)addr) + size;
	vma->vm_flags = req->flags;
	vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	/* vma->vm_page_prot = vm_get_page_prot(vm_flags); */
	vma->vm_pgoff = req->map_handle;
	vma->vm_file = info->filp;
	/* get_file(info->filp); */
	ret = info->filp->f_op->mmap(info->filp, vma);
	if (ret) {
		BUG();
	}

	vmf.flags = req->flags;
	vmf.pgoff = req->map_handle;
	vmf.virtual_address = addr;
	do {
		ret = vma->vm_ops->fault(vma, &vmf);
		if (ret & VM_FAULT_ERROR) {
			BUG();
		}
	} while (ret & VM_FAULT_RETRY);
	printk(KERN_INFO "PVDRM: fault with %d.\n", ret);

	if (ret & VM_FAULT_NOPAGE) {
		/* page is installed. */
	} else if (ret & VM_FAULT_LOCKED) {
		/* shoudl install page. */
	}
	printk(KERN_INFO "PVDRM: mmap is done with 0x%u / 0x%llx / 0x%llx\n", ret, (unsigned long)vmf.virtual_address, page_to_phys(pte_page(ptes[0])));
	msleep(10000);

	for (i = 0; i < pages; ++i) {
		int ref = xenbus_grant_ring(info->xbdev, pfn_to_mfn(page_to_pfn(pte_page(ptes[i]))));
		printk(KERN_INFO "PVDRM: mmap is done with 0x%u / 0x%llx / 0x%llx\n", ref, page_to_pfn(pte_page(ptes[i])), pfn_to_mfn(page_to_pfn(pte_page(ptes[i]))));
		msleep(10000);
		if (ref < 0) {
			/* FIXME: bug... */
			xenbus_dev_fatal(info->xbdev, ref, "granting ring page");
			BUG();
		}
		slot->u.references[i] = ref;
	}
	msleep(10000);

	return pages;
}

static int process_slot(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	int ret;
        struct drm_file* file_priv = NULL;
        struct drm_device* dev = NULL;
	mm_segment_t fs;

	ret = 0;
	file_priv = info->filp->private_data;
	dev = file_priv->minor->dev;

	fs = get_fs();
	set_fs(get_ds());
	printk(KERN_INFO "PVDRM: processing slot %d\n", slot->code);
	msleep(10000);

	/* Processing slot. */
	/* FIXME: Need to check in the host side. */
	switch (slot->code) {
	case PVDRM_IOCTL_NOUVEAU_GETPARAM:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GETPARAM, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, (unsigned long)pvdrm_slot_payload(slot));
		printk(KERN_INFO "PVDRM: allocate channel id %d\n", ((struct drm_nouveau_channel_alloc*)(pvdrm_slot_payload(slot)))->channel);
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_CHANNEL_FREE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_INFO:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_INFO, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_NEW:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_NEW, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF:
		ret = process_pushbuf(info, slot);
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_PREP, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_FINI, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_NOUVEAU_GEM_FREE: {
			/* FIXME: Need to investigate more... */
			struct drm_pvdrm_gem_free* req = pvdrm_slot_payload(slot);
			struct drm_gem_object* obj = drm_gem_object_lookup(dev, file_priv, req->handle);
			if (!obj) {
				ret = -EINVAL;
				break;
			}
			drm_gem_object_handle_free(obj);
			drm_gem_object_free(&obj->refcount);
			kfree(obj);
			ret = 0;
		}
		break;

	case PVDRM_GEM_NOUVEAU_GEM_CLOSE:
		ret = drm_ioctl(info->filp, DRM_IOCTL_GEM_CLOSE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_NOUVEAU_GEM_MMAP:
		/* FIXME: Need to check... */
		ret = process_mmap(info, slot);
		break;


	default:
		printk(KERN_INFO "PVDRM: unhandled slot %d\n", slot->code);
		break;
	}
	set_fs(fs);

	slot->ret = ret;

	/* Emit fence. */
	pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
	printk(KERN_INFO "PVDRM: slot %d is done\n", slot->code);
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

        /* Close DRM file. */
	if (info->filp) {
		mm_segment_t fs;
		fs = get_fs();
		set_fs(get_ds());
		filp_close(info->filp, NULL);
		set_fs(fs);
		info->filp = NULL;
	}

	return 0;
}

static int pvdrm_back_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	int ret;
	struct pvdrm_back_device* info;

	printk(KERN_INFO "Proving PVDRM backend driver %d.\n", xen_pv_domain());

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

		/* OK, connected. */
		info->thread = kthread_run(thread_main, (void*)info, "pvdrm-back");
		break;

	case XenbusStateClosing:
		xenbus_switch_state(xbdev, XenbusStateClosing);
		break;

	case XenbusStateClosed:
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
