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

#include <linux/fs.h>

#include "pvdrm_back_drv.h"

struct pvdrm_back_vma* pvdrm_back_vma_find(struct pvdrm_back_file* file, uint64_t map_handle)
{
	struct list_head *listptr;
	struct pvdrm_back_vma* vma = NULL;
	list_for_each(listptr, &file->vmas) {
		vma = list_entry(listptr, struct pvdrm_back_vma, head);
		if (vma->map_handle == map_handle) {
			return vma;
		}
	}
	return NULL;
}

struct pvdrm_back_vma* pvdrm_back_vma_find_with_gem_object(struct pvdrm_back_file* file, const struct drm_gem_object* obj)
{
	struct list_head *listptr;
	struct pvdrm_back_vma* vma = NULL;
	list_for_each(listptr, &file->vmas) {
		vma = list_entry(listptr, struct pvdrm_back_vma, head);
		if (vma->obj == obj) {
			return vma;
		}
	}
	return NULL;
}

void pvdrm_back_vma_destroy(struct pvdrm_back_vma* vma, struct pvdrm_back_file* file)
{
	uint32_t i;

	PVDRM_INFO("Freeing VMA\n");

	if (!vma) {
		return;
	}

	PVDRM_INFO("Freeing grant refs\n");
	if (vma->refs) {
		for (i = 0; i < vma->pages; ++i) {
			int ref = vma->refs[i];
			if (ref > 0) {
				/* FIXME: free on guest. */
				gnttab_end_foreign_access(ref, 0, 0);
			}
		}
		kfree(vma->refs);
	}

	/* iomem case. */
	if (vma->backing) {
		unsigned long first_mfn = 0;
		unsigned long first_gfn = 0;
		unsigned long count = 0;
		for (i = 0; i < vma->pages; ++i) {
			struct pvdrm_back_backing_mapping* mapping = &vma->backing[i];
			if (mapping->gfn && mapping->mfn) {
				if (!first_mfn) {
					first_mfn = mapping->mfn;
					first_gfn = mapping->gfn;
					count = 1;
				} else {
					if ((first_mfn + count) == mapping->mfn) {
						count += 1;
					} else {
						int ret = pvdrm_back_memory_mapping(file->info, first_gfn, first_mfn, count, false);
						PVDRM_INFO("Freeing IOMEM gfn:(%x) mfn:(%x).\n", first_gfn, first_mfn);
						BUG_ON(ret < 0);
						first_mfn = mapping->mfn;
						first_gfn = mapping->gfn;
						count = 1;
					}
				}
			} else {
				if (first_mfn) {
					int ret = pvdrm_back_memory_mapping(file->info, first_gfn, first_mfn, count, false);
					PVDRM_INFO("Freeing IOMEM gfn:(%x) mfn:(%x).\n", first_gfn, first_mfn);
					BUG_ON(ret < 0);
					first_mfn = 0;
					first_gfn = 0;
					count = 0;
				}
			}
		}
		if (first_mfn) {
			int ret = pvdrm_back_memory_mapping(file->info, first_gfn, first_mfn, count, false);
			PVDRM_INFO("Freeing IOMEM gfn:(%x) mfn:(%x).\n", first_gfn, first_mfn);
			BUG_ON(ret < 0);
		}
		kfree(vma->backing);
	}

	if (vma->area) {
		free_vm_area(vma->area);
	}

	if (vma->pteps) {
		kfree(vma->pteps);
	}


	{
		struct pvdrm_back_vma* pos;
		struct pvdrm_back_vma* temp;
		list_for_each_entry_safe(pos, temp, &file->vmas, head) {
			if (pos == vma) {
				list_del(&pos->head);
				break;
			}
		}
	}
	kfree(vma);
}


struct pvdrm_back_vma* pvdrm_back_vma_new(struct pvdrm_back_device* info, struct pvdrm_back_file* file, struct drm_gem_object* obj, uintptr_t start, uintptr_t end, unsigned long flags, unsigned long long map_handle)
{
	unsigned long pages;
	unsigned long long size;
	uintptr_t addr;
	struct pvdrm_back_vma* vma;

	size = (end - start);
	pages = size >> PAGE_SHIFT;

	vma = kzalloc(sizeof(struct pvdrm_back_vma), GFP_KERNEL);
	if (!vma) {
		BUG();
	}

	vma->map_handle = map_handle;
	vma->pages = pages;
	vma->pteps = kzalloc(sizeof(pte_t*) * (pages + 1), GFP_KERNEL);
	vma->obj = obj;
	if (!vma->pteps) {
		BUG();
	}

	vma->refs = kzalloc(sizeof(int) * (pages + 1), GFP_KERNEL);
	if (!vma->refs) {
		BUG();
	}

	vma->backing = kzalloc(sizeof(struct pvdrm_back_backing_mapping) * (pages + 1), GFP_KERNEL);
	if (!vma->backing) {
		BUG();
	}

	vma->area = alloc_vm_area(size, vma->pteps);
	if (!vma->area) {
		BUG();
	}
	addr = (uintptr_t)vma->area->addr;

	PVDRM_DEBUG("allocated area addresss size:(%llu), addr:(0x%llx), map_handle:(%llu).\n", (unsigned long long)size, (unsigned long long)addr, map_handle);

	vma->base.vm_mm = current->active_mm;
	vma->base.vm_start = (unsigned long)addr;
	vma->base.vm_end = ((unsigned long)addr) + size;
	vma->base.vm_flags = flags;
	vma->base.vm_page_prot = pgprot_writecombine(vm_get_page_prot(flags));
	/* vma->vm_page_prot = vm_get_page_prot(vm_flags); */
	vma->base.vm_pgoff = map_handle >> PAGE_SHIFT;
	vma->base.vm_file = file->filp;

	list_add(&vma->head, &file->vmas);

	PVDRM_INFO("New vma:(0x%lx) <-> obj:(0x%lx)\n", (unsigned long)vma, (unsigned long)obj);

	return vma;
}
