/*
  Copyright (C) 2014 Yusuke Suzuki <yusuke.suzuki@sslab.ics.keio.ac.jp>

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

#include <linux/console.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>
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

#include <drmP.h>

#include <common/pvdrm_log.h>

#include "pvdrm_cast.h"
#include "pvdrm_drm.h"
#include "pvdrm_gem.h"
#include "pvdrm_host_table.h"
#include "pvdrm_irq.h"
#include "pvdrm_vblank.h"
#include "pvdrm_nouveau_abi16.h"

int drm_xenbus_init(struct drm_driver *driver, struct xenbus_device *xbdev);

MODULE_PARM_DESC(cache, "Cache the GART memory TTMs in the frontend driver.");
int pvdrm_cache = 0;
module_param_named(cache, pvdrm_cache, int, 0400);

static const struct vm_operations_struct pvdrm_gem_vm_ops = {
	.fault = pvdrm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations pvdrm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = pvdrm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
};

#if !defined(DRM_RENDER_ALLOW)
#define DRM_RENDER_ALLOW 0
#endif

struct drm_ioctl_desc pvdrm_nouveau_ioctls[] = {
	DRM_IOCTL_DEF_DRV(NOUVEAU_GETPARAM, pvdrm_nouveau_abi16_ioctl_getparam, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_SETPARAM, pvdrm_nouveau_abi16_ioctl_setparam, DRM_UNLOCKED|DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_ALLOC, pvdrm_nouveau_abi16_ioctl_channel_alloc, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_FREE, pvdrm_nouveau_abi16_ioctl_channel_free, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GROBJ_ALLOC, pvdrm_nouveau_abi16_ioctl_grobj_alloc, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_NOTIFIEROBJ_ALLOC, pvdrm_nouveau_abi16_ioctl_notifierobj_alloc, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GPUOBJ_FREE, pvdrm_nouveau_abi16_ioctl_gpuobj_free, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_NEW, pvdrm_nouveau_gem_ioctl_new, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_PUSHBUF, pvdrm_nouveau_gem_ioctl_pushbuf, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_PREP, pvdrm_nouveau_gem_ioctl_cpu_prep, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_FINI, pvdrm_nouveau_gem_ioctl_cpu_fini, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_INFO, pvdrm_nouveau_gem_ioctl_info, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
};

static struct drm_driver pvdrm_drm_driver = {
	.driver_features = DRIVER_HAVE_IRQ | DRIVER_GEM,  /* DRIVER_HAVE_IRQ | DRIVER_MODESET | DRIVER_GEM  | DRIVER_PRIME, */
	.load       = pvdrm_drm_load,
	.unload     = pvdrm_drm_unload,

	.open       = pvdrm_drm_open,
	.preclose   = pvdrm_drm_preclose,
	.postclose  = pvdrm_drm_postclose,

	.gem_free_object  = pvdrm_gem_object_free,
	.gem_open_object  = pvdrm_gem_object_open,
	.gem_close_object = pvdrm_gem_object_close,
	.gem_vm_ops = &pvdrm_gem_vm_ops,

	.fops   = &pvdrm_fops,
	.irq_handler = pvdrm_irq_handler,

#if 0
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = pvdrm_gem_prime_export,
	.gem_prime_import = pvdrm_gem_prime_import,
#endif

	.get_vblank_counter = pvdrm_vblank_get_counter,
	.enable_vblank = pvdrm_vblank_enable,
	.disable_vblank = pvdrm_vblank_disable,

	/* .name       = DRIVER_NAME, */
	.desc       = DRIVER_DESC,
	.date       = DRIVER_DATE,
	.major      = DRIVER_MAJOR,
	.minor      = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

/* Called after backend is connected. */
/* FIXME: There's no memory free. */
int pvdrm_drm_init(struct pvdrm_device* pvdrm, struct drm_device *dev)
{
	pvdrm_slots_init(pvdrm);
	pvdrm->gem_cache = pvdrm_cache_new(pvdrm);
	pvdrm->gem_cache_enabled = !!pvdrm_cache;
	pvdrm->hosts_cache = kmem_cache_create("pvdrm_host_table", sizeof(struct pvdrm_host_table_entry), 0, 0, NULL);
	PVDRM_INFO("init with cache %d.\n", pvdrm->gem_cache_enabled);
	return 0;
}

int pvdrm_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct pvdrm_device *pvdrm = NULL;
	int ret = 0;

	pvdrm = kzalloc(sizeof(struct pvdrm_device), GFP_KERNEL);
	if (!pvdrm) {
		return -ENOMEM;
        }

	/* Configure it. */
	dev->dev_private = (void*)pvdrm;
	pvdrm->dev = dev;

	if (drm_ht_create(&pvdrm->mh2obj, 16)) {
		goto out;
	}
	spin_lock_init(&pvdrm->mh2obj_lock);

	idr_init(&pvdrm->channels_idr);
	spin_lock_init(&pvdrm->channels_lock);
	pvdrm->wq = alloc_ordered_workqueue("pvdrm-front", 0);
	if (!pvdrm->wq) {
		BUG();
	}

	PVDRM_INFO("loaded.\n");
out:
	if (ret)
		pvdrm_drm_unload(dev);

	return ret;
}

int pvdrm_drm_unload(struct drm_device *dev)
{
	struct pvdrm_device *pvdrm = NULL;
	int ret = 0;

	pvdrm = drm_device_to_pvdrm(dev);
	if (pvdrm) {
		pvdrm_slots_release(pvdrm);
		kfree(pvdrm);
		dev->dev_private = NULL;
	}

	return ret;
}

static int pvdrm_nouveau_global_call(struct drm_device* dev, int code, void *data, size_t size)
{
	struct pvdrm_device* pvdrm;
	int ret;
	pvdrm = drm_device_to_pvdrm(dev);
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm, PVDRM_FILE_GLOBAL_HANDLE);
		ret = pvdrm_slot_call(pvdrm, slot, code, data, size);
		pvdrm_slot_free(pvdrm, slot);
	}
	return ret;
}

int pvdrm_drm_open(struct drm_device* dev, struct drm_file* file)
{
	int ret = 0;
	struct drm_pvdrm_file_open req = { 0 };
	struct pvdrm_fpriv* fpriv = NULL;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);

	ret = pvdrm_nouveau_global_call(dev, PVDRM_FILE_OPEN, &req, sizeof(struct drm_pvdrm_file_open));
	if (ret) {
		goto fail_hypercall;
	}

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv) {
		ret = -ENOMEM;
		goto fail_hypercall;
	}

	fpriv->host = req.file;
	fpriv->file = file;
	fpriv->hosts = pvdrm_host_table_new(pvdrm);

	file->driver_priv = fpriv;

	return ret;

fail_hypercall:
	return ret;
}

void pvdrm_drm_preclose(struct drm_device *dev, struct drm_file *file)
{
}

void pvdrm_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);
	struct drm_pvdrm_file_close req = {
		.file = fpriv->host,
	};
	pvdrm_nouveau_global_call(dev, PVDRM_FILE_CLOSE, &req, sizeof(struct drm_pvdrm_file_close));
	kfree(fpriv);
}

static int pvdrm_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	/* In this phase, we can swich ioctl implementation to nouveau or other drivers. */
	int ret = 0;

	PVDRM_INFO("Proving PVDRM frontend driver.\n");

        /* Setup pvdrm driver for "nouveau" */
	pvdrm_drm_driver.ioctls = pvdrm_nouveau_ioctls;
	pvdrm_drm_driver.num_ioctls = ARRAY_SIZE(pvdrm_nouveau_ioctls);
        pvdrm_drm_driver.name = "nouveau";

	ret = drm_xenbus_init(&pvdrm_drm_driver, xbdev);
	if (ret) {
		BUG();
		return ret;
	}

	PVDRM_INFO("Initialised PVDRM frontend driver.\n");

	xenbus_switch_state(xbdev, XenbusStateInitialised);

	return ret;
}

static int pvdrm_remove(struct xenbus_device *xbdev)
{
	return 0;
}

static int pvdrm_connect(struct xenbus_device *xbdev)
{
	struct pvdrm_device* pvdrm;
	int ret;

	ret = 0;

	PVDRM_INFO("CONNECTED.\n");

	pvdrm = xbdev_to_pvdrm(xbdev);
	pvdrm_drm_init(pvdrm, xbdev_to_drm_device(xbdev));

	PVDRM_INFO("setting counter-ref.\n");
        {
		struct xenbus_transaction xbt;
again:
		ret = xenbus_transaction_start(&xbt);
		if (ret) {
			xenbus_dev_fatal(xbdev, ret, "starting transaction");
			return ret;
		}

		ret = xenbus_printf(xbt, xbdev->nodename, "counter-ref", "%u", pvdrm->slots->ref);
		if (ret) {
			xenbus_dev_fatal(xbdev, ret, "writing counter-ref");
			return ret;
		}

		ret = xenbus_transaction_end(xbt, 0);
		if (ret) {
			if (ret == -EAGAIN) {
				goto again;
			}
			xenbus_dev_fatal(xbdev, ret, "completing transaction");
			return ret;
		}
	}

	xenbus_switch_state(xbdev, XenbusStateConnected);
	PVDRM_INFO("setting is done.\n");

	return 0;
}

static void backend_changed(struct xenbus_device *xbdev, enum xenbus_state backend_state)
{
	PVDRM_INFO("Backend PVDRM driver state changed %s(%s).\n", xenbus_strstate(backend_state), xenbus_strstate(xbdev->state));

	switch (backend_state) {
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateUnknown:
	case XenbusStateClosed:
		break;

	case XenbusStateInitWait:
		pvdrm_connect(xbdev);
		break;

	case XenbusStateConnected:
		/* Connected & Connected. */
		PVDRM_INFO("Ok, now connecting.\n");
		break;

	case XenbusStateClosing:
		xenbus_frontend_closed(xbdev);
		break;
	}
}

static const struct xenbus_device_id pvdrm_ids[] = {
	{ "vdrm" },
	{ "" }
};

static DEFINE_XENBUS_DRIVER(pvdrm, "pvdrm-front",
	.probe = pvdrm_probe,
	.remove = pvdrm_remove,
	/* .resume = pvdrm_resume, */
	.otherend_changed = backend_changed,
);

static int __init pvdrm_init(void)
{
	if (!xen_domain())
		return -ENODEV;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
	if (!xen_has_pv_devices())
		return -ENODEV;
#else
	if (xen_hvm_domain() && !xen_platform_pci_unplug)
		return -ENODEV;
#endif

	PVDRM_INFO("Initialising PVDRM driver.\n");

	return xenbus_register_frontend(&pvdrm_driver);
}
module_init(pvdrm_init);

static void __exit pvdrm_exit(void)
{
	xenbus_unregister_driver(&pvdrm_driver);
}
module_exit(pvdrm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
