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

#include <linux/types.h>
#include <linux/delay.h>

#include <xen/xen.h>
#include <xen/page.h>
#include <xen/xenbus.h>
#include <xen/xenbus_dev.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <asm/xen/hypervisor.h>

#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_gem.h"
#include "pvdrm_slot.h"
#include "pvdrm_nouveau_abi16.h"

struct vma_priv {
        struct drm_device* dev;
        uint64_t map_handle;
};

/* FIXME: It's too dangerous. And it's not correct on ia32 environment. */
static struct page* extract_page(unsigned long address)
{
	struct mm_struct* mm = current->active_mm;
	pgd_t* pgd;
	pud_t* pud;
	pmd_t* pmd;
	pte_t* pte;
	pgd = pgd_offset(mm, address);
	pud = pud_offset(pgd, address);
	pmd = pmd_offset(pud, address);
	pte = pte_offset_map(pmd, address);
	return pte_page(*pte);
}

int pvdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int ret = 0;
        struct vma_priv* vma_priv = vma->vm_private_data;
	struct drm_device* dev = vma_priv->dev;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	const uint64_t map_handle = vma->vm_pgoff;
	struct drm_pvdrm_gem_mmap req = {
		.map_handle = map_handle,
		.flags = vmf->flags,
		.vm_start = vma->vm_start,
		.vm_end = vma->vm_end
	};
	int i;
	int pages = 0;
	pvdrm_slot_references references = { 0 };
	pgoff_t page_offset;

	/* Allocate pfn, pin it and pass it as memory window. */
	page_offset = ((unsigned long)vmf->virtual_address - vma->vm_start) >> PAGE_SHIFT;

	printk(KERN_INFO "PVDRM: fault is called with 0x%llx\n", map_handle);
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);
		slot->code = PVDRM_GEM_NOUVEAU_GEM_MMAP;
		memcpy(pvdrm_slot_payload(slot), &req, sizeof(struct drm_pvdrm_gem_mmap));
		ret = pvdrm_slot_request(pvdrm, slot);
		memcpy(&req, pvdrm_slot_payload(slot), sizeof(struct drm_pvdrm_gem_mmap));
		ret = slot->ret;
		memcpy(references, slot->u.references, sizeof(pvdrm_slot_references));
		pvdrm_slot_free(pvdrm, slot);
	}
	printk(KERN_INFO "PVDRM: fault is called with 0x%llx done %d.\n", ret);

	if (ret < 0) {
		goto out;
	}
	pages = ret;
	ret = 0;

	const uintptr_t vaddr = get_zeroed_page(GFP_KERNEL);
	ret = vm_insert_pfn(vma, (unsigned long)vmf->virtual_address, virt_to_pfn(vaddr));

	// printk(KERN_INFO "PVDRM: mapping pages %d\n", pages);
	// for (i = 0; i < pages; ++i) {
	// 	/* FIXME: Use gnttab_map_refs. */
	// 	void* addr = NULL;
	// 	printk(KERN_INFO "PVDRM: mapping pages page[%d] from dom%d = %d\n", i, pvdrm_to_xbdev(pvdrm)->otherend_id, references[i]);
        //         // msleep(2000);
	// 	ret = xenbus_map_ring_valloc(pvdrm_to_xbdev(pvdrm), references[i], &addr);
	// 	if (ret) {
        //                 printk(KERN_INFO "PVDRM: BUG! %d\n", ret);
	// 		/* FIXME: error... */
	// 		BUG();
	// 	}
	// 	const uintptr_t vaddr = get_zeroed_page(GFP_KERNEL);
	// 	printk(KERN_INFO "PVDRM: mapping pages page[%d] == 0x%llx / %d / 0x%llx / 0x%llx / 0x%llx / 0x%llx\n", i, page_to_pfn(extract_page((unsigned long)addr)), ret, pfn_to_mfn(page_to_pfn(extract_page((unsigned long)addr))), addr, virt_to_mfn(addr), virt_to_pfn(addr));
        //         // msleep(1000);
	// 	// ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + (PAGE_SIZE * i), page_to_pfn(extract_page((unsigned long)addr)));
	// 	ret = vm_insert_pfn(vma, (unsigned long)vma->vm_start + (PAGE_SIZE * i), virt_to_pfn(vaddr));
	// 	if (ret) {
	// 		BUG();
	// 	}
	// }
        // return VM_FAULT_NOPAGE;

#if 0
	struct drm_pvdrm_gem_object *obj = to_pvdrm_gem_object(vma->vm_private_data);
	struct drm_device *dev = obj->base.dev;
	struct pvdrm_device* pvdrm = drm_device_to_pvdrm(dev);
	unsigned long pfn;
	bool write = !!(vmf->flags & FAULT_FLAG_WRITE);


	/* FIXME: Implement it. */
#endif
out:
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
	printk(KERN_INFO "PVDRM: closing GEM %llx.\n", obj->host);
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
	obj->hash.key = info->map_handle >> PAGE_SHIFT;
	obj->map_handle = info->map_handle;

        spin_lock(&pvdrm->mh2obj_lock);
        printk(KERN_INFO "PVDRM: registering %lx / %llx\n", obj->hash.key, info->map_handle);
	ret = drm_ht_insert_item(&pvdrm->mh2obj, &obj->hash);
        spin_unlock(&pvdrm->mh2obj_lock);
	if (ret) {
		BUG();
		return NULL;
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
        int i;
	struct drm_file* file_priv = filp->private_data;
	struct drm_device* dev = file_priv->minor->dev;
        int count = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
        struct vma_priv* vma_priv = NULL;
        struct page* pages = NULL;
	const unsigned flags = vma->vm_flags | VM_RESERVED | VM_IO | VM_PFNMAP | VM_DONTEXPAND;
	struct drm_pvdrm_gem_mmap req = {
		.map_handle = vma->vm_pgoff,
		.flags = flags,
		.vm_start = vma->vm_start,
		.vm_end = vma->vm_end
	};
	struct pvdrm_device* pvdrm = NULL;
	struct drm_hash_item *hash;
	struct drm_pvdrm_gem_object* obj = NULL;

	pvdrm = drm_device_to_pvdrm(dev);

	if (unlikely(vma->vm_pgoff < DRM_FILE_PAGE_OFFSET)) {
		return drm_mmap(filp, vma);
	}

	spin_lock(&pvdrm->mh2obj_lock);
        printk(KERN_INFO "PVDRM: lookup %llx\n", vma->vm_pgoff);
	if (drm_ht_find_item(&pvdrm->mh2obj, vma->vm_pgoff >> PAGE_SHIFT, &hash)) {
		spin_unlock(&pvdrm->mh2obj_lock);
		BUG();
		return -EINVAL;
	}

	obj = drm_hash_entry(hash, struct drm_pvdrm_gem_object, hash);
	if (!obj) {
		BUG();
		return -EINVAL;
	}

	/* FIXME: memory reference. */
	vma->vm_flags = flags;
	vma->vm_ops = dev->driver->gem_vm_ops;
	vma->vm_private_data = obj;
	vma->vm_page_prot =  pgprot_writecombine(vm_get_page_prot(vma->vm_flags));

#if 0
        vma_priv = kmalloc(sizeof(*vma_priv), GFP_KERNEL);
        if (!vma_priv) {
                return -ENOMEM;
        }

        vma_priv->dev = dev;
        vma_priv->map_handle = vma->vm_pgoff;

	printk(KERN_INFO "PVDRM: fault is called with 0x%llx\n", map_handle);
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);
		slot->code = PVDRM_GEM_NOUVEAU_GEM_MMAP;
		memcpy(pvdrm_slot_payload(slot), &req, sizeof(struct drm_pvdrm_gem_mmap));
		ret = pvdrm_slot_request(pvdrm, slot);
		memcpy(&req, pvdrm_slot_payload(slot), sizeof(struct drm_pvdrm_gem_mmap));
		ret = slot->ret;
		memcpy(references, slot->u.references, sizeof(pvdrm_slot_references));
		pvdrm_slot_free(pvdrm, slot);
	}
	printk(KERN_INFO "PVDRM: fault is called with 0x%llx done %d.\n", ret);

	if (ret < 0) {
		goto out;
	}
	pages = ret;
	ret = 0;
#endif

        printk(KERN_INFO "PVDRM: mmap alloc with order %d\n", get_order(vma->vm_end - vma->vm_start));
        for (i = 0; i < count; ++i) {
                struct page* page = alloc_page(GFP_KERNEL);
                ret = vm_insert_page(vma, vma->vm_start + i * PAGE_SIZE, page);
                printk(KERN_INFO "PVDRM: mmap with ret %d\n", ret);
                if (ret) {
                        return ret;
                }
        }

	printk(KERN_INFO "PVDRM: mmap is called with 0x%llx\n", (unsigned long long)(vma->vm_pgoff));
        // return -EINVAL;
	return ret;
}

/* vim: set sw=8 ts=8 et tw=80 : */
