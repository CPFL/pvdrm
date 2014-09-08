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

#include <linux/types.h>
#include <linux/delay.h>

#include <xen/xen.h>
#include <xen/page.h>
#include <xen/xenbus.h>
#include <xen/xenbus_dev.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/balloon.h>
#include <asm/xen/hypervisor.h>

#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_gem.h"
#include "pvdrm_log.h"
#include "pvdrm_slot.h"
#include "pvdrm_nouveau_abi16.h"

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
	struct pvdrm_device* pvdrm = NULL;

	PVDRM_DEBUG("PVDRM: freeing GEM %llx.\n", (unsigned long long)obj->host);

	pvdrm = drm_device_to_pvdrm(dev);
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_GEM_NOUVEAU_GEM_FREE, &req, sizeof(struct drm_pvdrm_gem_free));

	/* FIXME: mmap list should be freed. */

	drm_gem_object_release(&obj->base);
	if (obj->backing) {
		free_pages(obj->backing, get_order(obj->base.size));
		obj->backing = 0;
	}
	if (obj->hash.key != -1) {
		spin_lock(&pvdrm->mh2obj_lock);
		drm_ht_remove_item(&pvdrm->mh2obj, &obj->hash);
		spin_unlock(&pvdrm->mh2obj_lock);
	}
	kfree(obj);
}

int pvdrm_gem_object_open(struct drm_gem_object *gem, struct drm_file *file)
{
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(gem);
	PVDRM_DEBUG("PVDRM: opening GEM %llx.\n", (unsigned long long)obj->host);
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

	PVDRM_DEBUG("PVDRM: closing GEM %llx.\n", (unsigned long long)obj->host);
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_GEM_NOUVEAU_GEM_CLOSE, &req, sizeof(struct drm_gem_close));
}

struct drm_pvdrm_gem_object* pvdrm_gem_alloc_object(struct drm_device* dev, struct drm_file *file, uint32_t host, uint32_t size)
{
	int ret = 0;
	struct drm_pvdrm_gem_object *obj;

	obj = kzalloc(sizeof(struct drm_pvdrm_gem_object), GFP_KERNEL);
	if (!obj) {
		goto free;
	}

	/* FIXME: drm_gem_private_object_init is preferable. */
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
	obj->hash.key = -1;

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

void pvdrm_gem_register_host_info(struct drm_device* dev, struct drm_file *file, struct drm_pvdrm_gem_object* obj, struct drm_nouveau_gem_info* info)
{
	struct pvdrm_device* pvdrm = NULL;
	int ret = 0;
	pvdrm = drm_device_to_pvdrm(dev);
	/* Setup mh2obj ht. */
	/* FIXME: lookup is needed? (for duplicate items) */
	/* FIXME: release side. */
	/* FIXME: Use drm_gem_create_mmap_offset instead. */
	obj->hash.key = info->map_handle >> PAGE_SHIFT;
	obj->domain = info->domain;
	obj->map_handle = info->map_handle;

	/* This gem is iomem. */
	if (obj->domain & NOUVEAU_GEM_DOMAIN_VRAM) {
		obj->backing = __get_free_pages(GFP_KERNEL, get_order(obj->base.size));
	}

	spin_lock(&pvdrm->mh2obj_lock);
	PVDRM_DEBUG("PVDRM: registering %lx / %llx domain:(%lx)\n", obj->hash.key, info->map_handle, (unsigned long)info->domain);
	ret = drm_ht_insert_item(&pvdrm->mh2obj, &obj->hash);
	spin_unlock(&pvdrm->mh2obj_lock);
	if (ret) {
		BUG();
		return;
	}
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
	pvdrm_gem_register_host_info(dev, file, obj, &req_out->info);

	/* Adjust gem information for guest environment. */
	req_out->info.handle = obj->handle;

	*result = obj;
	return 0;
}

struct drm_pvdrm_gem_object* pvdrm_gem_object_lookup(struct drm_device *dev, struct drm_file *file, uint32_t handle)
{
	return (struct drm_pvdrm_gem_object*)drm_gem_object_lookup(dev, file, handle);
}


int pvdrm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;
	struct drm_file* file_priv = filp->private_data;
	struct drm_device* dev = file_priv->minor->dev;
	struct drm_pvdrm_gem_mmap req;
	struct pvdrm_device* pvdrm = NULL;
	struct drm_hash_item *hash;
	struct drm_pvdrm_gem_object* obj = NULL;

	pvdrm = drm_device_to_pvdrm(dev);

	if (unlikely(vma->vm_pgoff < DRM_FILE_PAGE_OFFSET)) {
		return drm_mmap(filp, vma);
	}

	spin_lock(&pvdrm->mh2obj_lock);
	PVDRM_DEBUG("PVDRM: lookup %lx\n", vma->vm_pgoff);
	if (drm_ht_find_item(&pvdrm->mh2obj, vma->vm_pgoff, &hash)) {
		spin_unlock(&pvdrm->mh2obj_lock);
		BUG();
		return -EINVAL;
	}

	obj = drm_hash_entry(hash, struct drm_pvdrm_gem_object, hash);
	if (!obj) {
		spin_unlock(&pvdrm->mh2obj_lock);
		BUG();
		return -EINVAL;
	}
	spin_unlock(&pvdrm->mh2obj_lock);

	/* FIXME: memory reference. */
	vma->vm_flags |= VM_RESERVED | VM_IO | VM_PFNMAP | VM_DONTEXPAND;
	vma->vm_ops = dev->driver->gem_vm_ops;
	vma->vm_private_data = obj;
	vma->vm_page_prot =  pgprot_writecombine(vm_get_page_prot(vma->vm_flags));

	drm_gem_object_reference(&obj->base);

	drm_gem_vm_open(vma);

	req = (struct drm_pvdrm_gem_mmap) {
		.map_handle = obj->map_handle,
		.flags = vma->vm_flags,
		.vm_start = vma->vm_start,
		.vm_end = vma->vm_end,
		.handle = obj->host,
	};

	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_GEM_NOUVEAU_GEM_MMAP, &req, sizeof(struct drm_pvdrm_gem_mmap));
	return ret;
}

int pvdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
        int ret = 0;
	int i;
	struct drm_pvdrm_gem_object* obj = vma->vm_private_data;
	struct drm_device* dev = obj->base.dev;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	uint64_t backing = 0;
	uint64_t nr_pages = (obj->base.size >> PAGE_SHIFT);
	uint64_t offset = (uintptr_t)vmf->virtual_address - vma->vm_start;
	bool is_iomem = obj->domain & NOUVEAU_GEM_DOMAIN_VRAM;
	struct drm_pvdrm_gem_fault req;
	struct pvdrm_slot* slot = NULL;

	if (is_iomem) {
		backing = virt_to_pfn(obj->backing);
	}

	req = (struct drm_pvdrm_gem_fault) {
		.flags = vmf->flags,
		.pgoff = vmf->pgoff,
		.offset = offset,
		.map_handle = obj->map_handle,
		.mapped_count = 0xdeadbeef,
		.domain = obj->domain,
		.backing = backing,
		.nr_pages = nr_pages,
	};

	slot = pvdrm_slot_alloc(pvdrm);
	if (!slot) {
		BUG();
	}

	PVDRM_DEBUG("PVDRM: fault is called with 0x%lx, ref %d\n", vma->vm_pgoff, slot->ref);
	ret = pvdrm_slot_call(pvdrm, slot, PVDRM_GEM_NOUVEAU_GEM_FAULT, &req, sizeof(struct drm_pvdrm_gem_fault));
	PVDRM_DEBUG("PVDRM: fault is done %d.\n", ret);

	if (ret < 0) {
		BUG();
	}

	PVDRM_DEBUG("PVDRM: mapping pages %u\n", (unsigned)req.mapped_count);
	if (!is_iomem) {
		int32_t* refs = (int32_t*)slot->addr;

		/* FIXME: unmap and free pages. */
		struct page* pages[nr_pages];
		struct gnttab_map_grant_ref map[nr_pages];
		ret = alloc_xenballooned_pages(nr_pages, pages, false /* lowmem */);
		if (ret) {
			BUG();
		}
		for (i = 0; i < nr_pages; ++i) {
			uint32_t flags = GNTMAP_host_map;
			int32_t ref = refs[i];
			void* addr = pfn_to_kaddr(page_to_pfn(pages[i]));
			PVDRM_DEBUG("PVDRM: mapping pages page[%d] from dom%d = %d\n", i, pvdrm_to_xbdev(pvdrm)->otherend_id, mapping->ref);
			gnttab_set_map_op(&map[i], (unsigned long)addr, flags, ref, pvdrm_to_xbdev(pvdrm)->otherend_id);
		}

		ret = gnttab_map_refs(map, NULL, pages, nr_pages);
		if (ret) {
			PVDRM_ERROR("error with %d\n", ret);
			BUG();
		}

		for (i = 0; i < nr_pages; ++i) {
			int32_t ref = refs[i];
			unsigned long pfn = page_to_pfn(pages[i]);
			PVDRM_DEBUG("PVDRM: mapping pages page[%d] == pfn:(0x%lx)\n", i, pfn);
			ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + (PAGE_SIZE * i), pfn);
			if (ret) {
				BUG();
			}
		}
	} else {
		/* FIXME: Xen now put errors. */
		for (i = 0; i < nr_pages; ++i) {
			ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + (PAGE_SIZE * i), backing + i);
		}
	}

	pvdrm_slot_free(pvdrm, slot);

/* out: */
	switch (ret) {
	case -EIO:
		return VM_FAULT_SIGBUS;
	case -EAGAIN:
		set_need_resched();
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

/* vim: set sw=8 ts=8 et tw=80 : */
