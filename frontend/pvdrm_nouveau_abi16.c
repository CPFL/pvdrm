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
#include <drm/nouveau_drm.h>

#include <drmP.h>
#include <drm_crtc_helper.h>

#include <common/pvdrm_log.h>
#include <common/pvdrm_slot.h>

#include "pvdrm_cast.h"
#include "pvdrm_channel.h"
#include "pvdrm_drm.h"
#include "pvdrm_gem.h"
#include "pvdrm_pushbuf.h"
#include "pvdrm_nouveau_abi16.h"

int pvdrm_nouveau_abi16_ioctl(struct drm_file* file, int code, void* data, size_t size)
{
	struct pvdrm_device* pvdrm;
	int ret;
	BUG_ON(!(drm_file_to_fpriv(file)));
	pvdrm = drm_file_to_pvdrm(file);
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm, drm_file_to_fpriv(file)->host);
		ret = pvdrm_slot_call(pvdrm, slot, code, data, size);
		pvdrm_slot_free(pvdrm, slot);
	}
	return ret;
}

int pvdrm_nouveau_abi16_ioctl_getparam(struct drm_device *dev, void *data, struct drm_file *file)
{
	return pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GETPARAM, data, sizeof(struct drm_nouveau_getparam));
}

int pvdrm_nouveau_abi16_ioctl_setparam(struct drm_device *dev, void *data, struct drm_file *file)
{
	return -EINVAL;
}

int pvdrm_nouveau_abi16_ioctl_channel_alloc(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct pvdrm_channel* channel = NULL;
	return pvdrm_channel_alloc(dev, file, data, &channel);
}

int pvdrm_nouveau_abi16_ioctl_channel_free(struct drm_device *dev, void *data, struct drm_file *file)
{
	/* FIXME: Not implemented yet. */
	return pvdrm_channel_free(dev, file, data);
}

int pvdrm_nouveau_abi16_ioctl_grobj_alloc(struct drm_device *dev, void *data, struct drm_file *file)
{
	int ret = 0;
	struct pvdrm_channel* chan = NULL;
#if 0
	struct drm_pvdrm_gem_object* obj = NULL;
#endif
	struct pvdrm_device* pvdrm;
	struct drm_nouveau_grobj_alloc* req_out = data;

	pvdrm = drm_device_to_pvdrm(dev);

        chan = pvdrm_channel_lookup(dev, req_out->channel);
        if (!chan) {
		PVDRM_ERROR("GROBJ channel %u.\n", req_out->channel);
		ret = -EINVAL;
		goto free_chan;
        }
	PVDRM_INFO("GROBJ guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->host;
#if 0
	obj = pvdrm_gem_object_lookup(dev, file, req_out->handle);
	if (!obj) {
		ret = -EINVAL;
		goto free_chan;
	}
	req_out->handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);
#endif

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GROBJ_ALLOC, data, sizeof(struct drm_nouveau_grobj_alloc));
#if 0
free_obj:
	drm_gem_object_unreference(&obj->base);
#endif
free_chan:
	pvdrm_channel_unreference(chan);
	return ret;
}

int pvdrm_nouveau_abi16_ioctl_notifierobj_alloc(struct drm_device *dev, void *data, struct drm_file *file)
{
	return pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_NOTIFIEROBJ_ALLOC, data, sizeof(struct drm_nouveau_notifierobj_alloc));
}

int pvdrm_nouveau_abi16_ioctl_gpuobj_free(struct drm_device *dev, void *data, struct drm_file *file)
{
	int ret = 0;
	struct drm_nouveau_gpuobj_free* req_out = data;
	struct pvdrm_channel* chan = NULL;
#if 0
	struct drm_pvdrm_gem_object* obj;
#endif
	struct pvdrm_device* pvdrm;

	pvdrm = drm_device_to_pvdrm(dev);

	chan = pvdrm_channel_lookup(dev, req_out->channel);
        if (!chan) {
		PVDRM_ERROR("Freeing channel %u.\n", req_out->channel);
		ret = -EINVAL;
		goto free_chan;
        }
	PVDRM_DEBUG("Freeing guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->host;

#if 0
	obj = pvdrm_gem_object_lookup(dev, file, req_out->handle);
	if (!obj) {
		ret = -EINVAL;
		goto free_chan;
	}
	req_out->handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);
#endif

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GPUOBJ_FREE, data, sizeof(struct drm_nouveau_gpuobj_free));
#if 0
free_obj:
	drm_gem_object_unreference(&obj->base);
#endif
free_chan:
	pvdrm_channel_unreference(chan);
	return ret;
}

int pvdrm_nouveau_gem_ioctl_new(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_pvdrm_gem_object* result = NULL;
	return pvdrm_gem_object_new(dev, file, data, &result);
}

struct pushbuf_copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	uint32_t nr_buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	uint32_t nr_relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
	uint32_t nr_push;
};

int pvdrm_nouveau_gem_ioctl_pushbuf(struct drm_device *dev, void *data, struct drm_file *file)
{
	return pvdrm_pushbuf(dev, file, data);
}

int pvdrm_nouveau_gem_ioctl_cpu_prep(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_nouveau_gem_cpu_prep* req = data;
	uint32_t handle = req->handle;
	struct drm_pvdrm_gem_object* obj;
	int ret;

	obj = pvdrm_gem_object_lookup(dev, file, handle);
	if (!obj) {
		return -EINVAL;
	}
	req->handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP, data, sizeof(struct drm_nouveau_gem_cpu_prep));

	req->handle = handle;
	drm_gem_object_unreference(&obj->base);
	return ret;
}

int pvdrm_nouveau_gem_ioctl_cpu_fini(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_nouveau_gem_cpu_fini* req = data;
	uint32_t handle = req->handle;
	struct drm_pvdrm_gem_object* obj;
	int ret;

	obj = pvdrm_gem_object_lookup(dev, file, handle);
	if (!obj) {
		return -EINVAL;
	}
	req->handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI, data, sizeof(struct drm_nouveau_gem_cpu_fini));

	req->handle = handle;
	drm_gem_object_unreference(&obj->base);
	return ret;
}

int pvdrm_nouveau_gem_ioctl_info(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_nouveau_gem_info* req = data;
	uint32_t handle = req->handle;
	struct drm_pvdrm_gem_object* obj;
	int ret;

	obj = pvdrm_gem_object_lookup(dev, file, handle);
	if (!obj) {
		return -EINVAL;
	}
	req->handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_INFO, data, sizeof(struct drm_nouveau_gem_info));
	if (obj->hash.key == -1) {
		pvdrm_gem_register_host_info(dev, file, obj, req);
	}

	req->handle = handle;
	drm_gem_object_unreference(&obj->base);
	return ret;
}
