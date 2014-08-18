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
#include <linux/device.h>
#include <linux/module.h>
#include <drm/nouveau_drm.h>

#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"

/* Include nouveau's abi16 header directly. */
#include "../nouveau/nouveau_abi16.h"

#include "pvdrm_gem.h"
#include "pvdrm_slot.h"
#include "pvdrm_nouveau_abi16.h"

#define DEFINE_IOCTL_STUB(__code, __req)\
	struct pvdrm_device* pvdrm;\
	int ret;\
	\
	pvdrm = dev->dev_private;\
	{\
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);\
		slot->code = (__code);\
		memcpy(pvdrm_slot_payload(slot), data, sizeof(__req));\
		ret = pvdrm_slot_request(pvdrm, slot);\
		memcpy(data, pvdrm_slot_payload(slot), sizeof(__req));\
		pvdrm_slot_free(pvdrm, slot);\
	}\
	\
	return ret\

int pvdrm_nouveau_abi16_ioctl(struct drm_device *dev, int code, void *data, size_t size)
{
	struct pvdrm_device* pvdrm;
	int ret;
	pvdrm = dev->dev_private;
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);
		slot->code = (code);
		memcpy(pvdrm_slot_payload(slot), data, size);
		ret = pvdrm_slot_request(pvdrm, slot);
		memcpy(data, pvdrm_slot_payload(slot), size);
		pvdrm_slot_free(pvdrm, slot);
	}
	return ret;
}

int pvdrm_nouveau_abi16_ioctl_getparam(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GETPARAM, data, sizeof(struct drm_nouveau_getparam));
}

int pvdrm_nouveau_abi16_ioctl_setparam(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return -EINVAL;
}

int pvdrm_nouveau_abi16_ioctl_channel_alloc(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, data, sizeof(struct drm_nouveau_channel_alloc));
}

int pvdrm_nouveau_abi16_ioctl_channel_free(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE, data, sizeof(struct drm_nouveau_channel_free));
}

int pvdrm_nouveau_abi16_ioctl_grobj_alloc(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GROBJ_ALLOC, data, sizeof(struct drm_nouveau_grobj_alloc));
}

int pvdrm_nouveau_abi16_ioctl_notifierobj_alloc(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_NOTIFIEROBJ_ALLOC, data, sizeof(struct drm_nouveau_notifierobj_alloc));
}

int pvdrm_nouveau_abi16_ioctl_gpuobj_free(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GPUOBJ_FREE, data, sizeof(struct drm_nouveau_gpuobj_free));
}

int pvdrm_nouveau_gem_ioctl_new(struct drm_device *dev, void *data, struct drm_file *file_priv) {
	struct drm_pvdrm_gem_object* result = NULL;
	return pvdrm_gem_object_new(dev, file_priv, data, &result);
}

int pvdrm_nouveau_gem_ioctl_pushbuf(struct drm_device *dev, void *data, struct drm_file *file_priv) {
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF, data, sizeof(struct drm_nouveau_gem_pushbuf));
}

int pvdrm_nouveau_gem_ioctl_cpu_prep(struct drm_device *dev, void *data, struct drm_file *file_priv) {
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP, data, sizeof(struct drm_nouveau_gem_cpu_prep));
}

int pvdrm_nouveau_gem_ioctl_cpu_fini(struct drm_device *dev, void *data, struct drm_file *file_priv) {
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI, data, sizeof(struct drm_nouveau_gem_cpu_fini));
}

int pvdrm_nouveau_gem_ioctl_info(struct drm_device *dev, void *data, struct drm_file *file_priv) {
	return pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GEM_INFO, data, sizeof(struct drm_nouveau_gem_info));
}

struct drm_ioctl_desc pvdrm_nouveau_ioctls[] = {
	DRM_IOCTL_DEF_DRV(NOUVEAU_GETPARAM, pvdrm_nouveau_abi16_ioctl_getparam, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_SETPARAM, pvdrm_nouveau_abi16_ioctl_setparam, DRM_UNLOCKED|DRM_AUTH|DRM_MASTER|DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_ALLOC, pvdrm_nouveau_abi16_ioctl_channel_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_CHANNEL_FREE, pvdrm_nouveau_abi16_ioctl_channel_free, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GROBJ_ALLOC, pvdrm_nouveau_abi16_ioctl_grobj_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_NOTIFIEROBJ_ALLOC, pvdrm_nouveau_abi16_ioctl_notifierobj_alloc, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GPUOBJ_FREE, pvdrm_nouveau_abi16_ioctl_gpuobj_free, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_NEW, pvdrm_nouveau_gem_ioctl_new, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_PUSHBUF, pvdrm_nouveau_gem_ioctl_pushbuf, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_PREP, pvdrm_nouveau_gem_ioctl_cpu_prep, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_CPU_FINI, pvdrm_nouveau_gem_ioctl_cpu_fini, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(NOUVEAU_GEM_INFO, pvdrm_nouveau_gem_ioctl_info, DRM_UNLOCKED|DRM_AUTH),
};

/* vim: set sw=8 ts=8 et tw=80 : */
