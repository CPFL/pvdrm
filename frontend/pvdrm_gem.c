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

#include <drmP.h>
#include <drm_crtc_helper.h>

#include <common/pvdrm_bench.h>
#include <common/pvdrm_ignore_unused_variable_warning.h>
#include <common/pvdrm_log.h>
#include <common/pvdrm_slot.h>

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_gem.h"
#include "pvdrm_host_table.h"
#include "pvdrm_nouveau_abi16.h"

uint32_t pvdrm_gem_host(struct pvdrm_fpriv* fpriv, struct drm_pvdrm_gem_object* obj)
{
	int ret;
	uint32_t host = 0;
	ret = pvdrm_host_table_lookup(fpriv->hosts, obj, &host);
	BUG_ON(ret);
	return host;
}

int pvdrm_gem_refcount(const struct drm_pvdrm_gem_object* obj)
{
	return atomic_read(&obj->base.refcount.refcount);
}

int pvdrm_gem_object_init(struct drm_gem_object *obj)
{
	return 0;
}

static void pvdrm_gem_object_free_grant_refs(struct drm_pvdrm_gem_object* obj)
{
	if (obj->pages) {
		int i;
		int ret = 0;
		int invcount = 0;
		struct gnttab_unmap_grant_ref* unmap = kzalloc(sizeof(struct gnttab_unmap_grant_ref) * (obj->base.size / PAGE_SIZE), GFP_KERNEL);
		struct page** pages = kzalloc(sizeof(struct page*) * (obj->base.size / PAGE_SIZE), GFP_KERNEL);
		for (i = 0; i < (obj->base.size / PAGE_SIZE); ++i) {
			struct page* page = obj->pages[i];
			if (page) {
				pages[invcount] = page;
				gnttab_set_unmap_op(&unmap[invcount], (unsigned long)pfn_to_kaddr(page_to_pfn(page)), GNTMAP_host_map, obj->handles[i]);
				invcount++;
			}
		}
		ret = gnttab_unmap_refs(unmap, NULL, pages, invcount);
		BUG_ON(ret);
		free_xenballooned_pages(invcount, pages);
		kfree(pages);
		kfree(unmap);

		kfree(obj->pages);
		obj->pages = NULL;
		kfree(obj->handles);
		obj->handles = NULL;
	}
}

void pvdrm_gem_object_free(struct drm_gem_object *gem)
{
	struct drm_pvdrm_gem_object* obj = to_pvdrm_gem_object(gem);
	struct drm_device* dev = obj->base.dev;
	struct pvdrm_device* pvdrm = NULL;

	PVDRM_INFO("freeing GEM %lx.\n", (unsigned long)obj);
	pvdrm = drm_device_to_pvdrm(dev);

	/* FIXME: mmap list should be freed. */
	if (obj->backing) {
		/* FIXME: Free iomem mapped area asynchronously. */
		/* FIXME: Incorrect freeing. */
		/* printk(KERN_INFO "== backing:count:(%d),mapped:(%d),order:(%d)\n", atomic_read(&obj->backing->_count), page_mapped(obj->backing), get_order(obj->base.size)); */
		/* FIXME: Not sure why it causes memory corrupt... */
		/* put_page(obj->backing); */
		obj->backing = NULL;
	}

	pvdrm_gem_object_free_grant_refs(obj);
	drm_gem_object_release(&obj->base);
	if (obj->hash.key != -1) {
		unsigned long flags;
		spin_lock_irqsave(&pvdrm->mh2obj_lock, flags);
		drm_ht_remove_item(&pvdrm->mh2obj, &obj->hash);
		spin_unlock_irqrestore(&pvdrm->mh2obj_lock, flags);
	}
	kfree(obj);
}

int pvdrm_gem_object_open(struct drm_gem_object* gem, struct drm_file* file)
{
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(gem);
	/* pvdrm_host_table_insert(drm_file_to_fpriv(file)->hosts, gem); */
	PVDRM_IGNORE_UNUSED_VARIABLE_WARNING(obj);
	PVDRM_INFO("opening GEM %p count:(%d).\n", obj, pvdrm_gem_refcount(obj));
	return 0;
}

void pvdrm_gem_object_close(struct drm_gem_object* gem, struct drm_file* file)
{
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(gem);
	struct drm_device* dev = obj->base.dev;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	struct drm_gem_close req = {
		.handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj),
	};
	int ret;
	PVDRM_INFO("closing GEM %p count:(%d).\n", obj, pvdrm_gem_refcount(obj));
	if (pvdrm->gem_cache_enabled && obj->cacheable) {
		pvdrm_cache_insert(pvdrm->gem_cache, file, obj);
		PVDRM_INFO("Caching GEM %p count:(%d).\n", obj, pvdrm_gem_refcount(obj));
	}

	/* FIXME: It's not good solution. */
	if (atomic_read(&obj->base.refcount.refcount) == 1) {
		/* Last reference. After this closing call, it will be freed. */
		pvdrm_gem_object_free_grant_refs(obj);
	}

	pvdrm_nouveau_abi16_ioctl(file, PVDRM_GEM_NOUVEAU_GEM_CLOSE, &req, sizeof(struct drm_gem_close));

	ret = pvdrm_host_table_remove(drm_file_to_fpriv(file)->hosts, obj);
	BUG_ON(ret);
}

static int register_handle(struct drm_file* file, struct drm_pvdrm_gem_object* obj, uint32_t host, uint32_t* handle)
{
	int ret;
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);

	ret = drm_gem_handle_create(file, &obj->base, handle);
	if (!ret) {
		/* Drop reference from allocate - handle holds it now */
		drm_gem_object_unreference(&obj->base);
	}

	ret = pvdrm_host_table_insert(fpriv->hosts, obj, host);
	BUG_ON(ret);

	return ret;
}

struct drm_pvdrm_gem_object* pvdrm_gem_alloc_object(struct drm_device* dev, struct drm_file* file, uint32_t host, uint32_t size, uint32_t* handle)
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

	ret = register_handle(file, obj, host, handle);
	if (ret) {
		pvdrm_gem_object_free(&obj->base);
		return NULL;
	}

	return obj;
free:
	kfree(obj);
	return NULL;
}

void pvdrm_gem_register_host_info(struct drm_device* dev, struct drm_file *file, struct drm_pvdrm_gem_object* obj, struct drm_nouveau_gem_info* info)
{
	struct pvdrm_device* pvdrm = NULL;
	int ret = 0;
	unsigned long flags;
	pvdrm = drm_device_to_pvdrm(dev);

	obj->domain = info->domain;
	obj->map_handle = info->map_handle;

	/* Setup mh2obj ht. */
	/* FIXME: lookup is needed? (for duplicate items) */
	/* FIXME: release side. */
	/* FIXME: Use drm_gem_create_mmap_offset instead. */
	obj->hash.key = info->map_handle >> PAGE_SHIFT;
	spin_lock_irqsave(&pvdrm->mh2obj_lock, flags);
	PVDRM_DEBUG("registering %lx / %llx domain:(%lx)\n", obj->hash.key, info->map_handle, (unsigned long)info->domain);
	ret = drm_ht_insert_item(&pvdrm->mh2obj, &obj->hash);
	spin_unlock_irqrestore(&pvdrm->mh2obj_lock, flags);
	if (ret) {
		BUG();
		return;
	}
}

int pvdrm_gem_object_new(struct drm_device* dev, struct drm_file* file, struct drm_nouveau_gem_new* req_out, struct drm_pvdrm_gem_object** result)
{
	struct drm_pvdrm_gem_object* obj = NULL;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	int ret = 0;
	const unsigned req_domain = req_out->info.domain;
	const bool mappable = req_out->info.domain & NOUVEAU_GEM_DOMAIN_MAPPABLE;
	const bool dma = req_out->info.domain & NOUVEAU_GEM_DOMAIN_GART;
	uint32_t handle = 0;

	PVDRM_INFO("Checking req_domain:(%u) size:(%llx) check:(%d)\n", req_domain, req_out->info.size, (int)(mappable && dma));
	PVDRM_IGNORE_UNUSED_VARIABLE_WARNING(req_domain);
	if (pvdrm->gem_cache_enabled && (mappable && dma)) {
		obj = pvdrm_cache_fit(pvdrm->gem_cache, req_out->info.size);
		if (obj) {
			uint32_t host = 0;
			{
				struct drm_pvdrm_gem_global_handle req = {
					.handle = 0,
					.global = obj->global,
				};
				ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_GEM_FROM_GLOBAL_HANDLE, &req, sizeof(struct drm_pvdrm_gem_global_handle));
				BUG_ON(req.handle == 0);
				host = req.handle;
			}
			register_handle(file, obj, host, &handle);
			{
				req_out->info.handle = host;
				ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_INFO, &req_out->info, sizeof(struct drm_nouveau_gem_info));
			}
			PVDRM_INFO("Reviving %p with refcount:(%d) domain:(%u)\n", obj, pvdrm_gem_refcount(obj), obj->domain);
		}
	}

	if (!obj) {
		ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_NEW, req_out, sizeof(struct drm_nouveau_gem_new));
		if (ret) {
			return ret;
		}

		obj = pvdrm_gem_alloc_object(dev, file, req_out->info.handle, req_out->info.size, &handle);
		if (obj == NULL) {
			return -ENOMEM;
		}
		pvdrm_gem_register_host_info(dev, file, obj, &req_out->info);
		PVDRM_INFO("Allocating %p with refcount:(%d) domain:(%u)\n", obj, pvdrm_gem_refcount(obj), obj->domain);
	}

	BUG_ON(!obj);

	/* Adjust gem information for guest environment. */
	req_out->info.handle = handle;

	*result = obj;

	/* Caching the memory. */
	if (mappable) {
		if (dma) {
			PVDRM_INFO("Caching %p with refcount:(%d)\n", obj, pvdrm_gem_refcount(obj));
			obj->cacheable = true;
		} else if (obj->domain & NOUVEAU_GEM_DOMAIN_VRAM) {
		}
	}

	return ret;
}

struct drm_pvdrm_gem_object* pvdrm_gem_object_lookup(struct drm_device *dev, struct drm_file *file, uint32_t handle)
{
	return (struct drm_pvdrm_gem_object*)drm_gem_object_lookup(dev, file, handle);
}


int pvdrm_gem_mmap(struct file* filp, struct vm_area_struct* vma)
{
	int ret = 0;
	int i;
	unsigned long flags;
	struct drm_file* file = filp->private_data;
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);
	struct drm_device* dev = file->minor->dev;
	struct drm_pvdrm_gem_mmap req;
	struct pvdrm_device* pvdrm = NULL;
	struct drm_hash_item *hash;
	struct drm_pvdrm_gem_object* obj = NULL;

	pvdrm = drm_device_to_pvdrm(dev);

	if (unlikely(vma->vm_pgoff < DRM_FILE_PAGE_OFFSET)) {
		return drm_mmap(filp, vma);
	}

	spin_lock_irqsave(&pvdrm->mh2obj_lock, flags);
	PVDRM_DEBUG("lookup %lx\n", vma->vm_pgoff);
	if (drm_ht_find_item(&pvdrm->mh2obj, vma->vm_pgoff, &hash)) {
		spin_unlock_irqrestore(&pvdrm->mh2obj_lock, flags);
		BUG();
		return -EINVAL;
	}

	obj = drm_hash_entry(hash, struct drm_pvdrm_gem_object, hash);
	if (!obj) {
		spin_unlock_irqrestore(&pvdrm->mh2obj_lock, flags);
		BUG();
		return -EINVAL;
	}
	spin_unlock_irqrestore(&pvdrm->mh2obj_lock, flags);

	/* This gem is iomem. */
	if (!obj->backing && obj->domain & NOUVEAU_GEM_DOMAIN_VRAM) {
		obj->backing = alloc_pages(GFP_KERNEL, get_order(obj->base.size));
		/* printk(KERN_INFO "backing:(%d)\n", atomic_read(&obj->backing->_count)); */
		BUG_ON(!obj->backing);
	}

	/* FIXME: memory reference. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
#else
	vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_RESERVED;
#endif

	vma->vm_ops = dev->driver->gem_vm_ops;
	vma->vm_private_data = obj;
	vma->vm_page_prot =  pgprot_writecombine(vm_get_page_prot(vma->vm_flags));

	drm_gem_vm_open(vma);

	if (obj->pages) {
		for (i = 0; i < (obj->base.size / PAGE_SIZE); ++i) {
			struct page* page = obj->pages[i];
			if (page) {
				unsigned long pfn = page_to_pfn(page);
				ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + PAGE_SIZE * i, pfn);
				if (ret && ret != -EBUSY) {
					BUG();
				}
			}
		}
		return ret;
	}
	BUG_ON(obj->pages);

	req = (struct drm_pvdrm_gem_mmap) {
		.map_handle = obj->map_handle,
		.flags = vma->vm_flags,
		.vm_start = vma->vm_start,
		.vm_end = vma->vm_end,
		.handle = pvdrm_gem_host(fpriv, obj),
	};

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_GEM_NOUVEAU_GEM_MMAP, &req, sizeof(struct drm_pvdrm_gem_mmap));
	if (ret) {
		BUG();
	}

	/* Remap previously resolved pages. */
	/* FIXME: Page size alignment. */
	BUG_ON(obj->pages);
	obj->pages = kzalloc(sizeof(struct page*) * (obj->base.size / PAGE_SIZE), GFP_KERNEL);
	obj->handles = kzalloc(sizeof(grant_handle_t) * (obj->base.size / PAGE_SIZE), GFP_KERNEL);

	return ret;
}

int pvdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int ret = 0;
	int i;
	struct drm_pvdrm_gem_object* obj = vma->vm_private_data;
	struct drm_device* dev = obj->base.dev;
	struct drm_file* file = vma->vm_file->private_data;
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	uint64_t backing = 0;
	uint64_t offset = (uintptr_t)vmf->virtual_address - vma->vm_start;
	bool is_iomem = obj->domain & NOUVEAU_GEM_DOMAIN_VRAM;
	struct drm_pvdrm_gem_fault req;
	struct pvdrm_slot* slot = NULL;

	if (is_iomem) {
		BUG_ON(!obj->backing);
		backing = page_to_pfn(obj->backing);
	}

	req = (struct drm_pvdrm_gem_fault) {
		.flags = vmf->flags,
		.pgoff = vmf->pgoff,
		.offset = offset,
		.map_handle = obj->map_handle,
		.mapped_count = 0xdeadbeef,
		.domain = obj->domain,
		.backing = backing,
		.nr_pages = (obj->base.size >> PAGE_SHIFT) - (offset >> PAGE_SHIFT),
	};

	slot = pvdrm_slot_alloc(pvdrm, fpriv->host);
	if (!slot) {
		BUG();
	}

	PVDRM_DEBUG("fault is called with 0x%lx, ref %d\n", vma->vm_pgoff, slot->ref);
	PVDRM_INFO("Faulting %p.\n", obj);
	ret = pvdrm_slot_call(pvdrm, slot, PVDRM_GEM_NOUVEAU_GEM_FAULT, &req, sizeof(struct drm_pvdrm_gem_fault));
	PVDRM_DEBUG("fault is done %d.\n", ret);

	if (ret < 0) {
		BUG();
	}

	PVDRM_DEBUG("mapping pages %u\n", (unsigned)req.mapped_count);
	if (!is_iomem) {
		struct pvdrm_mapping* refs = (struct pvdrm_mapping*)slot->addr;

		/* FIXME: unmap and free pages. */
		struct page** pages = kzalloc(sizeof(struct page*) * PVDRM_GEM_FAULT_MAX_PAGES_PER_CALL, GFP_KERNEL);
		struct gnttab_map_grant_ref* map = kzalloc(sizeof(struct gnttab_map_grant_ref) * PVDRM_GEM_FAULT_MAX_PAGES_PER_CALL, GFP_KERNEL);
		struct pvdrm_bench bench;
		ret = alloc_xenballooned_pages(req.mapped_count, pages, false /* lowmem */);
		if (ret) {
			BUG();
		}
		for (i = 0; i < req.mapped_count; ++i) {
			uint32_t flags = GNTMAP_host_map;
			struct pvdrm_mapping* mapping = &refs[i];
			void* addr = pfn_to_kaddr(page_to_pfn(pages[i]));
			PVDRM_DEBUG("mapping pages page[%d] from dom%d = %d\n", mapping->i, pvdrm_to_xbdev(pvdrm)->otherend_id, mapping->ref);
			gnttab_set_map_op(&map[i], (unsigned long)addr, flags, mapping->ref, pvdrm_to_xbdev(pvdrm)->otherend_id);
		}
		PVDRM_BENCH(&bench) {
			ret = gnttab_map_refs(map, NULL, pages, req.mapped_count);
		}
		if (ret) {
			PVDRM_ERROR("error with %d\n", ret);
			BUG();
		}

		for (i = 0; i < req.mapped_count; ++i) {
			struct pvdrm_mapping* mapping = &refs[i];
			unsigned long pfn = page_to_pfn(pages[i]);
			PVDRM_DEBUG("mapping pages page[%d] == pfn:(0x%lx)\n", mapping->i, pfn);
			ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + (PAGE_SIZE * mapping->i), pfn);
			if (ret && ret != -EBUSY) {
				PVDRM_ERROR("ERROR:%d 0x%lx,0x%lx,0x%lx\n", ret, vma->vm_start, vma->vm_start + (PAGE_SIZE * mapping->i), pfn);
				BUG();
			} else if (ret == -EBUSY) {
				PVDRM_ERROR("FAIL mapping pages page[%d] == pfn:(0x%lx)\n", mapping->i, pfn);
			}
			BUG_ON(!obj->pages);
			obj->pages[mapping->i] = pages[i];
			obj->handles[mapping->i] = map[i].handle;
		}
		kfree(map);
		kfree(pages);
	} else {
		/* FIXME: Xen now put errors. */
		for (i = 0; i < req.mapped_count; ++i) {
			/* printk(KERN_INFO "B[%d] backing:(%d)\n", i, atomic_read(&obj->backing->_count)); */
			ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + offset + (PAGE_SIZE * i), page_to_pfn(&obj->backing[(offset >> PAGE_SHIFT) + i]));
			/* printk(KERN_INFO "A[%d] backing:(%d),ret:(%d)\n", i, atomic_read(&obj->backing->_count), ret); */
			if (ret && ret != -EBUSY) {
				PVDRM_ERROR("ERROR:%d 0x%lx,0x%lx\n", ret, vma->vm_start, vma->vm_start + offset + (PAGE_SIZE * i));
				BUG();
			} else if (ret == -EBUSY) {
				printk(KERN_INFO "FAIL!!\n");
			}
		}
	}

	pvdrm_slot_free(pvdrm, slot);

/* out: */
	switch (ret) {
	case -EIO:
		return VM_FAULT_SIGBUS;
	case -EAGAIN:
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}
