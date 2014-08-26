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

#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_gem.h"
#include "pvdrm_slot.h"
#include "pvdrm_nouveau_abi16.h"

int pvdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int ret = 0;
#if 0
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(vma->vm_private_data);
	struct drm_device *dev = obj->base.dev;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	pgoff_t page_offset;
	unsigned long pfn;
	bool write = !!(vmf->flags & FAULT_FLAG_WRITE);

	/* Allocate pfn, pin it and pass it as memory window. */
	page_offset = ((unsigned long)vmf->virtual_address - vma->vm_start) >> PAGE_SHIFT;

	/* FIXME: Implement it. */
#endif
	return ret;
}

int pvdrm_gem_object_init(struct drm_gem_object *obj)
{
	return 0;
}

void pvdrm_gem_object_free(struct drm_gem_object *gem)
{
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(gem);
	struct drm_device *dev = obj->base.dev;
	struct drm_pvdrm_gem_free req = {
		.handle = obj->host,
	};
	int ret = 0;
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_GEM_NOUVEAU_GEM_FREE, &req, sizeof(struct drm_pvdrm_gem_free));
	drm_gem_object_release(&obj->base);
	kfree(obj);
}

int pvdrm_gem_object_open(struct drm_gem_object *gem, struct drm_file *file)
{
	return 0;
}

void pvdrm_gem_object_close(struct drm_gem_object *gem, struct drm_file *file)
{
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(gem);
	struct drm_device *dev = obj->base.dev;
	struct drm_gem_close req = {
		.handle = obj->host,
	};
	int ret = 0;
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_GEM_NOUVEAU_GEM_CLOSE, &req, sizeof(struct drm_gem_close));
}

struct drm_pvdrm_gem_object* pvdrm_gem_alloc_object(struct drm_device* dev, struct drm_file *file, uint32_t host, uint32_t size)
{
	int ret;
	struct drm_pvdrm_gem_object *obj;

	obj = kzalloc(sizeof(struct drm_pvdrm_gem_object), GFP_KERNEL);
	if (!obj) {
		goto free;
	}

	if (drm_gem_object_init(dev, &obj->base, size) != 0) {
		goto free;
	}

	if (dev->driver->gem_init_object != NULL &&
	    dev->driver->gem_init_object(&obj->base) != 0) {
		goto fput;
	}

	/* Store host information. */
	obj->handle = (uint32_t)-1;
	obj->host = host;

	/* FIXME: These code are moved from pvdrm_gem_object_new. */
	ret = drm_gem_handle_create(file, &obj->base, &obj->handle);
	if (ret) {
		pvdrm_gem_object_free(&obj->base);
		return NULL;
	}

	/* Drop reference from allocate - handle holds it now */
	drm_gem_object_unreference(&obj->base);

	return obj;
fput:
	/* Object_init mangles the global counters - readjust them. */
	fput(obj->base.filp);
free:
	kfree(obj);
	return NULL;
}

int pvdrm_gem_object_new(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_gem_new *req_out, struct drm_pvdrm_gem_object** result)
{
	struct drm_pvdrm_gem_object *obj;
	int ret;

	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_GEM_NEW, req_out, sizeof(struct drm_nouveau_gem_new));
	if (ret) {
		return ret;
	}

	obj = pvdrm_gem_alloc_object(dev, file, req_out->info.handle, req_out->info.size);
	if (obj == NULL) {
		return -ENOMEM;
	}

	/* Adjust gem information for guest environment. */
	req_out->info.handle = obj->handle;

	*result = obj;
	return 0;
}

struct drm_pvdrm_gem_object* pvdrm_gem_object_lookup(struct drm_device *dev, struct drm_file *file, uint32_t handle)
{
	return (struct drm_pvdrm_gem_object*)drm_gem_object_lookup(dev, file, handle);
}

/* vim: set sw=8 ts=8 et tw=80 : */
