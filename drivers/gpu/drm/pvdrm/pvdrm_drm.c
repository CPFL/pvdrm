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

#include <linux/console.h>
#include <linux/module.h>

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
#include "drm.h"
#include "drm_crtc_helper.h"

#include "pvdrm_drm.h"
#include "pvdrm_gem.h"
#include "pvdrm_irq.h"
#include "pvdrm_load.h"
#include "pvdrm_ttm.h"
#include "pvdrm_vblank.h"

#if 0
static struct drm_ioctl_desc nouveau_ioctls[] = {
	DRM_IOCTL_DEF_DRV(NOUVEAU_GETPARAM, nouveau_abi16_ioctl_getparam, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_SETPARAM, nouveau_abi16_ioctl_setparam, DRM_UNLOCKED|DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_ALLOC, nouveau_abi16_ioctl_channel_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_FREE, nouveau_abi16_ioctl_channel_free, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GROBJ_ALLOC, nouveau_abi16_ioctl_grobj_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_NOTIFIEROBJ_ALLOC, nouveau_abi16_ioctl_notifierobj_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GPUOBJ_FREE, nouveau_abi16_ioctl_gpuobj_free, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_NEW, nouveau_gem_ioctl_new, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_PUSHBUF, nouveau_gem_ioctl_pushbuf, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_PREP, nouveau_gem_ioctl_cpu_prep, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_FINI, nouveau_gem_ioctl_cpu_fini, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_INFO, nouveau_gem_ioctl_info, DRM_UNLOCKED|DRM_AUTH),
};

static struct drm_driver driver = {
	.driver_features =
		DRIVER_USE_AGP | DRIVER_PCI_DMA | DRIVER_SG |
		DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED | DRIVER_GEM |
		DRIVER_MODESET | DRIVER_PRIME,
	.load = nouveau_load,
	.firstopen = nouveau_firstopen,
	.lastclose = nouveau_lastclose,
	.unload = nouveau_unload,
	.open = nouveau_open,
	.preclose = nouveau_preclose,
	.postclose = nouveau_postclose,
#if defined(CONFIG_DRM_NOUVEAU_DEBUG)
	.debugfs_init = nouveau_debugfs_init,
	.debugfs_cleanup = nouveau_debugfs_takedown,
#endif
	.irq_preinstall = nouveau_irq_preinstall,
	.irq_postinstall = nouveau_irq_postinstall,
	.irq_uninstall = nouveau_irq_uninstall,
	.irq_handler = nouveau_irq_handler,
	.get_vblank_counter = drm_vblank_count,
	.enable_vblank = nouveau_vblank_enable,
	.disable_vblank = nouveau_vblank_disable,
	.ioctls = nouveau_ioctls,
	.fops = &pvdrm_driver_fops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = nouveau_gem_prime_export,
	.gem_prime_import = nouveau_gem_prime_import,

	.gem_init_object = nouveau_gem_object_new,
	.gem_free_object = nouveau_gem_object_del,
	.gem_open_object = nouveau_gem_object_open,
	.gem_close_object = nouveau_gem_object_close,
};

#endif

static struct drm_ioctl_desc pvdrm_ioctls[] = {
};

static const struct file_operations pvdrm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = pvdrm_ttm_mmap,
	.poll = drm_poll,
	.fasync = drm_fasync,
	.read = drm_read,
	.llseek = noop_llseek,
};

static struct drm_driver pvdrm_drm_driver = {
	.driver_features	= DRIVER_HAVE_IRQ | DRIVER_MODESET | DRIVER_GEM,
	.load       = pvdrm_load,
	.unload     = pvdrm_unload,

	.open       = pvdrm_open,
	.preclose   = pvdrm_preclose,
	.postclose  = pvdrm_postclose,

	.gem_init_object  = pvdrm_gem_object_init,
	.gem_free_object  = pvdrm_gem_object_free,
	.gem_open_object  = pvdrm_gem_object_open,
	.gem_close_object = pvdrm_gem_object_close,
	.fops   = &pvdrm_fops,
	.ioctls = pvdrm_ioctls,
	.irq_handler = pvdrm_irq_handler,

	.get_vblank_counter = pvdrm_vblank_get_counter,
	.enable_vblank = pvdrm_vblank_enable,
	.disable_vblank = pvdrm_vblank_disable,

	.name       = DRIVER_NAME,
	.desc       = DRIVER_DESC,
	.date       = DRIVER_DATE,
	.major      = DRIVER_MAJOR,
	.minor      = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int __devinit pvdrm_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	return drm_xenbus_init(&pvdrm_drm_driver, xbdev);
}

static void pvdrm_remove(struct xenbus_device *xbdev)
{
	drm_xenbus_exit(&pvdrm_drm_driver, xbdev);
}

static void pvdrm_changed(struct xenbus_device *xbdev, enum xenbus_state backend_state)
{
}

static const struct xenbus_device_id pvdrm_ids[] = {
	{ "vdrm" },
	{ "" }
};

static DEFINE_XENBUS_DRIVER(pvdrm, "pvdrm",
	.probe = pvdrm_probe,
	.remove = pvdrm_remove,
	/* .resume = pvdrm_resume, */
	.otherend_changed = pvdrm_changed,
	/* .is_ready = pvdrm_is_ready, */
);

static int __init pvdrm_init(void)
{
	int ret = 0;

	if (!xen_domain())
		return -ENODEV;

	if (xen_hvm_domain() && !xen_platform_pci_unplug)
		return -ENODEV;

	ret = xenbus_register_frontend(&pvdrm_driver);
	if (ret)
                return -ENODEV;
	return 0;
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
/* vim: set sw=8 ts=8 et tw=80 : */
