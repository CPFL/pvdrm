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

#include "drmP.h"
#include "drm.h"

#include "pvdrm_cast.h"
#include "pvdrm_drm.h"
#include "pvdrm_ttm.h"

static int pvdrm_ttm_mem_global_init(struct drm_global_reference* ref)
{
	return ttm_mem_global_init(ref->object);
}

static void pvdrm_ttm_mem_global_release(struct drm_global_reference* ref)
{
	ttm_mem_global_release(ref->object);
}

int pvdrm_ttm_global_init(struct pvdrm_device* pvdrm)
{
	struct drm_global_reference* global_ref;
	struct pvdrm_ttm* ttm;
	int ret = 0;


	ttm = kzalloc(sizeof(struct pvdrm_ttm), GFP_KERNEL);
	if (!ttm) {
		return -ENOMEM;
	}
	pvdrm->ttm = ttm;

	// init mem_global_ref
	global_ref = &ttm->mem_global_ref;
	global_ref->global_type = DRM_GLOBAL_TTM_MEM;
	global_ref->size = sizeof(struct ttm_mem_global);
	global_ref->init = &pvdrm_ttm_mem_global_init;
	global_ref->release = &pvdrm_ttm_mem_global_release;
	ret = drm_global_item_ref(global_ref);
	if (ret) {
		BUG();
		return ret;
	}

	// init bo_global_ref
	ttm->bo_global_ref.mem_glob = ttm->mem_global_ref.object;
	global_ref = &ttm->bo_global_ref.ref;
	global_ref->global_type = DRM_GLOBAL_TTM_BO;
	global_ref->size = sizeof(struct ttm_bo_global);
	global_ref->init = &ttm_bo_global_init;
	global_ref->release = &ttm_bo_global_release;
	ret = drm_global_item_ref(global_ref);
	if (ret) {
		BUG();
		return ret;
	}

	return ret;
}

void pvdrm_ttm_global_release(struct pvdrm_device* pvdrm)
{
	if (pvdrm->ttm->mem_global_ref.release == NULL) {
		return;
	}

	drm_global_item_unref(&pvdrm->ttm->bo_global_ref.ref);
	drm_global_item_unref(&pvdrm->ttm->mem_global_ref);
	pvdrm->ttm->mem_global_ref.release = NULL;
	kfree(pvdrm->ttm);
	pvdrm->ttm = NULL;
}

int
pvdrm_ttm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* FIXME: We should implement ttm shadow memory on the guest side. */
	struct drm_file* file_priv = filp->private_data;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(file_priv->minor->dev);

	if (unlikely(vma->vm_pgoff < DRM_FILE_PAGE_OFFSET)) {
		return drm_mmap(filp, vma);
	}

	return ttm_bo_mmap(filp, vma, &pvdrm->ttm->bdev);
}
