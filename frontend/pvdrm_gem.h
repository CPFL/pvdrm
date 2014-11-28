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
#ifndef PVDRM_GEM_H_
#define PVDRM_GEM_H_

#include <drmP.h>
#include <common/pvdrm_nouveau_drm.h>

struct pvdrm_fpriv;

struct drm_pvdrm_gem_object {
	struct drm_gem_object base;
	struct drm_hash_item hash;
	uint32_t global;
	uint32_t domain;
	uint64_t map_handle;
	bool cacheable;

	/* grant page mappings. */
	struct page** pages;
	grant_handle_t* handles;

	struct page* backing;  /* Backing store for VRAM mapping. */
};

uint32_t pvdrm_gem_host(struct pvdrm_fpriv* fpriv, struct drm_pvdrm_gem_object* obj);
int pvdrm_gem_refcount(const struct drm_pvdrm_gem_object* obj);
int pvdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
int pvdrm_gem_object_init(struct drm_gem_object *obj);
void pvdrm_gem_object_free(struct drm_gem_object *gobj);
int pvdrm_gem_object_open(struct drm_gem_object *obj, struct drm_file *file);
void pvdrm_gem_object_close(struct drm_gem_object *obj, struct drm_file *file);
int pvdrm_gem_object_new(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_gem_new *req_out, struct drm_pvdrm_gem_object** result);
struct drm_pvdrm_gem_object* pvdrm_gem_object_lookup(struct drm_device *dev, struct drm_file *file, uint32_t handle);
struct drm_pvdrm_gem_object* pvdrm_gem_alloc_object(struct drm_device *dev, struct drm_file *file, uint32_t host, uint32_t size, uint32_t* handle);
void pvdrm_gem_register_host_info(struct drm_device* dev, struct drm_file *file, struct drm_pvdrm_gem_object* obj, struct drm_nouveau_gem_info* info);

int pvdrm_gem_mmap(struct file *filp, struct vm_area_struct *vma);

#endif  /* PVDRM_GEM_H_ */
