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

#include <linux/atomic.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/version.h>

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
#include <asm/xen/hypercall.h>

#include <drmP.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
#include <drm_gem.h>
#endif

#include <common/pvdrm_wait.h>
#include <generated/pvdrm_imported.h>

#include "pvdrm_back_drv.h"

#include "xen_added_interface.h"  /* For domctl. */

MODULE_PARM_DESC(sequential, "Sequantially execute the ops submitted from the frontend.");
int pvdrm_back_sequential = 1;
module_param_named(sequential, pvdrm_back_sequential, int, 0400);

typedef struct {
	spinlock_t req_lock;
} pvdrm_back_core_t;

static pvdrm_back_core_t* pvdrm_back_core;

static void* pvdrm_back_slot_addr(struct pvdrm_back_device* info, const struct pvdrm_slot* slot)
{
	return info->slot_addrs[pvdrm_slot_id(info->mapped, slot)];
}

static struct pvdrm_back_work* pvdrm_back_slot_work(struct pvdrm_back_device* info, const struct pvdrm_slot* slot)
{
	return &info->works[pvdrm_slot_id(info->mapped, slot)];
}


static uint64_t pvdrm_back_count(struct pvdrm_back_device* info)
{
	return atomic_read(&info->mapped->count);
}

static struct pvdrm_slot* claim_slot(struct pvdrm_back_device* info)
{
	uint32_t id;
	uint32_t pos;
	atomic_dec(&info->mapped->count);
	pos = ((uint32_t)atomic_add_return(1, &info->get)) % PVDRM_SLOT_NR;
	id = info->mapped->ring[pos];
	return &info->mapped->slot[id];
}

struct copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
};

static int transfer(struct copier* copier, struct pvdrm_slot* slot, uint8_t* addr, bool copying)
{
	if (slot->u.transfer.nr_buffers) {
		/* FIXME: Check buffer size doesn't exceed for the security issues. */
		const size_t size = slot->u.transfer.nr_buffers * sizeof(struct drm_nouveau_gem_pushbuf_bo);
		addr = PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_bo));
		if (copying) {
			memcpy(copier->buffers, addr, size);
			copier->buffers += slot->u.transfer.nr_buffers;
		} else {
			copier->buffers = (struct drm_nouveau_gem_pushbuf_bo*)addr;
		}
		addr += size;
	}

	if (slot->u.transfer.nr_relocs) {
		const size_t size = slot->u.transfer.nr_relocs * sizeof(struct drm_nouveau_gem_pushbuf_reloc);
		addr = PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_reloc));
		if (copying) {
			memcpy(copier->relocs, addr, size);
			copier->relocs += slot->u.transfer.nr_relocs;
		} else {
			copier->relocs = (struct drm_nouveau_gem_pushbuf_reloc*)addr;
		}
		addr += size;
	}

	if (slot->u.transfer.nr_push) {
		const size_t size = slot->u.transfer.nr_push * sizeof(struct drm_nouveau_gem_pushbuf_push);
		addr = PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_push));
		if (copying) {
			memcpy(copier->push, addr, size);
			copier->push += slot->u.transfer.nr_push;
		} else {
			copier->push = (struct drm_nouveau_gem_pushbuf_push*)addr;
		}
		addr += size;
	}
	PVDRM_INFO("Transferring pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", slot->u.transfer.nr_buffers, slot->u.transfer.nr_relocs, slot->u.transfer.nr_push);

	return 0;
}

static int process_pushbuf(struct pvdrm_back_device* info, struct pvdrm_back_file* file, struct pvdrm_slot* slot)
{
	/* buffers, reloc, push is user space pointers of the guest domain. */
	int ret = 0;
	struct drm_nouveau_gem_pushbuf* req = pvdrm_slot_payload(slot);
	struct drm_nouveau_gem_pushbuf_bo* buffers = NULL;
	struct drm_nouveau_gem_pushbuf_reloc* relocs = NULL;
	struct drm_nouveau_gem_pushbuf_push* push = NULL;
	void* addr = pvdrm_back_slot_addr(info, slot);

	PVDRM_DEBUG("pushbuf with ref %d\n", slot->ref);
	if (req->nr_push == 0) {
		/* OK, there's no buffers. */
		PVDRM_DEBUG("pushbuf with no buffers...\n");
		/* FIXME: Check parameter is valid. */
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));
		goto destroy_data;
	}

	/* Validate pushbuf content size. */
	if (req->nr_buffers && req->buffers) {
		if (req->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS) {
			PVDRM_ERROR("pushbuf buffers are too large.\n");
			return -EINVAL;
		}
	}

	if (req->nr_relocs && req->relocs) {
		if (req->nr_relocs > NOUVEAU_GEM_MAX_RELOCS) {
			PVDRM_DEBUG("pushbuf relocs are too large.\n");
			ret = -EINVAL;
			goto destroy_data;
		}
	}

	if (req->nr_push && req->push) {
		if (req->nr_push > NOUVEAU_GEM_MAX_PUSH) {
			PVDRM_DEBUG("pushbuf push are too large.\n");
			ret = -EINVAL;
			goto destroy_data;
		}
	}

	/* In the most cases, buffers are very small and they are transfered
	 * within the one phase. So when transfer phases are only 1, instead of
	 * duplicating the buffer contents into host buffers, we use the guest
	 * buffer provided by the hypercall directly.
	 */
	if (!(slot->u.transfer.next > 0)) {
		struct copier copier = { 0 };
		ret = transfer(&copier, slot, addr, false);
		if (ret) {
			return ret;
		}

		req->buffers = (unsigned long)copier.buffers;
		req->relocs = (unsigned long)copier.relocs;
		req->push = (unsigned long)copier.push;
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));

		return ret;
	}

	buffers = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_bo) * req->nr_buffers, GFP_KERNEL);
	if (!buffers) {
		ret = -ENOMEM;
		goto destroy_data;
	}

	relocs = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_reloc) * req->nr_relocs, GFP_KERNEL);
	if (!relocs) {
		ret = -ENOMEM;
		goto destroy_data;
	}

	push = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_push) * req->nr_push, GFP_KERNEL);
	if (!push) {
		ret = -ENOMEM;
		goto destroy_data;
	}

	/* Copying data to the host side. */
	{
		int next = 0;
		struct copier copier = {
			.buffers = buffers,
			.relocs = relocs,
			.push = push,
		};
		PVDRM_DEBUG("Copying pushbufs...\n");
		do {
			PVDRM_DEBUG("Copy! pushbuf...\n");
			ret = transfer(&copier, slot, addr, true);
			if (ret) {
				goto destroy_data;
			}
			next = slot->u.transfer.next;
			if (next > 0) {
				pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
				pvdrm_fence_wait(&slot->__fence, 1, false);
			}
		} while (next > 0);
		PVDRM_DEBUG("Copying pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", req->nr_buffers, req->nr_relocs, req->nr_push);
	}

	req->buffers = (unsigned long)buffers;
	req->relocs = (unsigned long)relocs;
	req->push = (unsigned long)push;
	ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));

destroy_data:
	if (buffers) {
		kfree(buffers);
	}
	if (relocs) {
		kfree(relocs);
	}
	if (push) {
		kfree(push);
	}

	return ret;
}

int pvdrm_back_memory_mapping(struct pvdrm_back_device* info, uint64_t first_gfn, uint64_t first_mfn, uint64_t nr_mfns, bool add_mapping)
{
	struct xen_domctl domctl = { 0 };

	domctl.cmd = XEN_DOMCTL_memory_mapping;
	domctl.domain = info->xbdev->otherend_id;
	domctl.u.memory_mapping.first_gfn = first_gfn;
	domctl.u.memory_mapping.first_mfn = first_mfn;
	domctl.u.memory_mapping.nr_mfns = nr_mfns;
	domctl.u.memory_mapping.add_mapping = (add_mapping) ? 1 : 0;
	domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

	return _hypercall1(int, domctl, &domctl);
}

int pvdrm_back_iomem_permission(struct pvdrm_back_device *info, uint64_t first_mfn, uint64_t nr_mfns, bool allow_access)
{
    struct xen_domctl domctl = { 0 };

    domctl.cmd = XEN_DOMCTL_iomem_permission;
    domctl.domain = info->xbdev->otherend_id;
    domctl.u.iomem_permission.first_mfn = first_mfn;
    domctl.u.iomem_permission.nr_mfns = nr_mfns;
    domctl.u.iomem_permission.allow_access = (allow_access) ? 1 : 0;
    domctl.interface_version = XEN_DOMCTL_INTERFACE_VERSION;

    return _hypercall1(int, domctl, &domctl);
}


static inline uint32_t uint32_max(uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

static inline uint32_t uint32_min(uint32_t a, uint32_t b)
{
    return (a > b) ? b : a;
}

static int process_fault(struct pvdrm_back_device* info, struct pvdrm_back_file* file, struct pvdrm_slot* slot)
{
    int i;
    int ret = 0;
    int result = 0;
    uint64_t page_offset = 0;
	uint32_t max;
	uint32_t max_limited_by_vm_end;
	struct pvdrm_mapping* refs = NULL;
	struct drm_pvdrm_gem_fault* req = pvdrm_slot_payload(slot);
	struct pvdrm_back_vma* vma = NULL;
	bool is_iomem = req->domain & NOUVEAU_GEM_DOMAIN_VRAM;

	vma = pvdrm_back_vma_find(info->global, req->map_handle);
	if (!vma) {
		return -EINVAL;
	}

	page_offset = req->offset >> PAGE_SHIFT;
	PVDRM_DEBUG("Fault.... [%s] start with %llu, offset:(0x%lx) (0x%lx => 0x%lx . 0x%lx)\n", is_iomem ? "VRAM" : "SYSRAM", (unsigned long long)page_offset, (unsigned long)req->offset, vma->base.vm_start, vma->base.vm_end, (unsigned long)(vma->base.vm_start + req->offset));

	max = PVDRM_GEM_FAULT_MAX_PAGES_PER_CALL;
	max_limited_by_vm_end = ((vma->base.vm_end - vma->base.vm_start) >> PAGE_SHIFT) - page_offset;
	max = min(max_limited_by_vm_end, max);
	max = min(req->nr_pages, max);

	if (is_iomem) {
		max = 1;
	}

	for (i = 0; i < max; ++i) {
		/* Not mappable page? */
		if (pte_none(*vma->pteps[page_offset + i])) {
#if 1
			ret = IMPORTED(handle_mm_fault)(
					vma->base.vm_mm,
					&vma->base, vma->base.vm_start + req->offset + i * PAGE_SIZE,
					FAULT_FLAG_WRITE);
#else
			struct vm_fault vmf = {
				.flags = req->flags,
				.pgoff = req->pgoff + i,
				.virtual_address = (void*)(vma->base.vm_start + req->offset + i * PAGE_SIZE),
			};
			do {
				ret = vma->base.vm_ops->fault(&vma->base, &vmf);
				if (ret & VM_FAULT_ERROR) {
					BUG();
				}
			} while (ret & VM_FAULT_RETRY);
#endif
			if (ret & VM_FAULT_ERROR) {
				PVDRM_ERROR("Fault.... %d\n", ret);
				BUG();
			}

			if (ret & VM_FAULT_NOPAGE) {
				/* page is installed. */
			} else if (ret & VM_FAULT_LOCKED) {
				/* FIXME: should install page. */
			}
		}
	}
	// PVDRM_DEBUG("mmap is done with %u / 0x%llx / 0x%llx , ref %d\n", ret, (unsigned long long)vmf.virtual_address, (unsigned long long)page_to_phys(pte_page(*(vma->pteps[0]))), slot->ref);

	if (!is_iomem) {
		/* FIXME: This should be removed for optimizations. */
		refs = pvdrm_back_slot_addr(info, slot);
		for (i = 0; i < max; ++i) {
			int offset = page_offset + i;
			int ref = 0;
			if (vma->refs[page_offset + i] > 0) {
				ref = vma->refs[page_offset + i];
			} else {
				ref = gnttab_grant_foreign_access(info->xbdev->otherend_id, pfn_to_mfn(pte_pfn(*(vma->pteps[offset]))), 0);
			}
			PVDRM_INFO("to dom%d mmap is done with %d / 0x%llx\n", info->xbdev->otherend_id, ref, (unsigned long long)pfn_to_mfn(pte_pfn(*(vma->pteps[offset]))));
			if (ref < 0) {
				/* FIXME: bug... */
				xenbus_dev_fatal(info->xbdev, ref, "granting ring page");
				BUG();
			}
			vma->refs[offset] = ref;
			refs[i] = (struct pvdrm_mapping) {
				.i = offset,
				.ref = ref
			};
			++result;
		}
		req->mapped_count = result;
	} else {
		struct pvdrm_bench bench = {};
		unsigned long first_mfn = 0;
		unsigned long count = 0;
		/* PVDRM_DEBUG("mfn:(%lx) backing:(%lx) max:(%u)\n", mfn, (unsigned long)(req->backing + page_offset), (unsigned)max); */
		PVDRM_INFO("= IOMEM start\n");
		PVDRM_BENCH(&bench) {
			for (i = 0; i < max; ++i) {
				int offset = page_offset + i;
				unsigned long mfn = pfn_to_mfn(page_to_pfn(pte_page(*(vma->pteps[offset]))));
				if (!first_mfn) {
					first_mfn = mfn;
					count = 1;
				} else {
					if ((first_mfn + count) == mfn) {
						// Continuous.
						count += 1;
					} else {
						// Not continuous.
						PVDRM_INFO("| IOMEM %lu\n", count);
                        ret = pvdrm_back_iomem_permission(info, first_mfn, count, true);
						BUG_ON(ret < 0);
						ret = pvdrm_back_memory_mapping(info, req->backing + offset - count, first_mfn, count, true);
						BUG_ON(ret < 0);
						first_mfn = mfn;
						count = 1;
					}
				}
				vma->backing[offset] = (struct pvdrm_back_backing_mapping) {
					.gfn = req->backing + offset,
					.mfn = mfn
				};
			}
			if (first_mfn) {
				PVDRM_INFO("= IOMEM %lu\n", count);
                ret = pvdrm_back_iomem_permission(info, first_mfn, count, true);
                BUG_ON(ret < 0);
				ret = pvdrm_back_memory_mapping(info, req->backing + (page_offset + max) - count, first_mfn, count, true);
				BUG_ON(ret < 0);
			}
		}
		ret = 0;
		req->mapped_count = max;
	}
	return ret;
}

static int process_mmap(struct pvdrm_back_device* info, struct pvdrm_back_file* file, struct pvdrm_slot* slot)
{
	int ret = 0;
	struct drm_device* dev = pvdrm_back_file_to_drm_device(file);
	struct drm_pvdrm_gem_mmap* req = pvdrm_slot_payload(slot);
	struct drm_file* file_priv = pvdrm_back_file_to_drm_file(file);
	struct pvdrm_back_vma* vma = NULL;
	struct drm_gem_object* obj = NULL;

	/* FIXME: Maybe, becomes bug... (info->global). */
	vma = pvdrm_back_vma_find(info->global, req->map_handle);
	if (vma) {
		/* Already mapped. */
		return 0;
	}

	obj = drm_gem_object_lookup(dev, file_priv, req->handle);
	if (!obj) {
		return -EINVAL;
	}
	vma = pvdrm_back_vma_new(info, info->global, obj, req->vm_start, req->vm_end, req->flags, req->map_handle);
	if (!vma) {
		drm_gem_object_unreference_unlocked(obj);
		return -ENOMEM;
	}
	drm_gem_object_unreference_unlocked(obj);

	/* Call f_op->mmap operation directly since vm_mmap requires current->mm
	 * is not NULL.
	 */
	ret = file->filp->f_op->mmap(file->filp, &vma->base);
	if (ret) {
		BUG();
		return -EINVAL;
	}

	return ret;
}

static void process_slot(struct work_struct* arg)
{
	int ret;
	struct pvdrm_back_work* work = NULL;
	struct pvdrm_back_device* info = NULL;
	struct pvdrm_slot* slot = NULL;
	struct pvdrm_back_file* file = NULL;
	/* struct pvdrm_bench bench = { }; */
	/* mm_segment_t fs; */

	work = container_of(arg, struct pvdrm_back_work, base);
	BUG_ON(!work);
	info = work->info;
	BUG_ON(!info);
	slot = work->slot;
	BUG_ON(!slot);

	PVDRM_INFO("processing slot %s:(%d)\n", pvdrm_op_str(slot->code), slot->code);
	/* pvdrm_bench_open(&bench); */
	/* msleep(1000); */

	ret = 0;

	/* fs = get_fs(); */
	/* set_fs(get_ds()); */

	if (slot->file) {
		file = pvdrm_back_file_open_if_necessary(info, slot->file);
		if (!file) {
			ret = -EINVAL;
			goto done;
		}
	} else {
		/* Use global file. */
		file = info->global;
	}

	/* Processing slot. */
	switch (slot->code) {
	case PVDRM_FILE_OPEN: {
			struct drm_pvdrm_file_open* req = pvdrm_slot_payload(slot);
			struct pvdrm_back_file* target = pvdrm_back_file_new(info);
			if (!target) {
				ret = -ENOMEM;
				break;
			}
			req->file = target->handle;
			ret = 0;
		}
		break;

	case PVDRM_FILE_CLOSE: {
			struct drm_pvdrm_file_close* req = pvdrm_slot_payload(slot);
			struct pvdrm_back_file* target = pvdrm_back_file_lookup(info, req->file);
			if (!target) {
				ret = -EINVAL;
				break;
			}
			pvdrm_back_file_destroy(target);
		}
		break;

	case PVDRM_GEM_TO_PRIME_FD:
		ret = drm_ioctl(file->filp, DRM_IOCTL_PRIME_HANDLE_TO_FD, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_FROM_PRIME_FD:
		ret = drm_ioctl(file->filp, DRM_IOCTL_PRIME_FD_TO_HANDLE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GETPARAM:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GETPARAM, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC: {
			struct pvdrm_bench bench;
			PVDRM_BENCH_WITH_NAME(&bench, "channel alloc") {
				ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, (unsigned long)pvdrm_slot_payload(slot));
				PVDRM_DEBUG("allocate channel id %d\n", ((struct drm_nouveau_channel_alloc*)(pvdrm_slot_payload(slot)))->channel);
			}
		}
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_CHANNEL_FREE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_INFO:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_INFO, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_NEW:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_NEW, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF:
		ret = process_pushbuf(info, file, slot);
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_PREP, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_FINI, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GROBJ_ALLOC:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GROBJ_ALLOC, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GPUOBJ_FREE:
		ret = drm_ioctl(file->filp, DRM_IOCTL_NOUVEAU_GPUOBJ_FREE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_NOUVEAU_GEM_MMAP:
		/* FIXME: Need to check... */
		ret = process_mmap(info, file, slot);
		break;

	case PVDRM_GEM_NOUVEAU_GEM_FAULT:
		/* FIXME: Need to check... */
		ret = process_fault(info, file, slot);
		break;

	case PVDRM_GEM_NOUVEAU_GEM_CLOSE: {
			struct drm_pvdrm_gem_free* req = pvdrm_slot_payload(slot);
			struct drm_device* dev = pvdrm_back_file_to_drm_device(file);
			struct drm_file* file_priv = pvdrm_back_file_to_drm_file(file);
			struct drm_gem_object* obj = drm_gem_object_lookup(dev, file_priv, req->handle);
			PVDRM_DEBUG("Closing handle:(%x) with ref:(%d)\n", req->handle, atomic_read(&obj->refcount.refcount));
			if (!obj) {
				PVDRM_WARN("Invalid freeing handle:(%x)\n", req->handle);
				ret = -EINVAL;
				break;
			}

			/* FIXME: It's not good solution. */
			/* Lookup reference & handle reference, there's no global reference. */
			if (atomic_read(&obj->refcount.refcount) == 2) {
				struct pvdrm_back_vma* vma = NULL;
				vma = pvdrm_back_vma_find_with_gem_object(info->global, obj);
				if (vma) {
					PVDRM_DEBUG("Closing VMA:(%x)\n", req->handle);
					pvdrm_back_vma_destroy(vma, info->global);
				}
			}

			drm_gem_object_unreference_unlocked(obj);  /* Drop the reference from lookup. */
			ret = drm_gem_handle_delete(file_priv, req->handle);
		}
		break;

	case PVDRM_GEM_TO_GLOBAL_HANDLE: {
			struct drm_pvdrm_gem_global_handle* req = pvdrm_slot_payload(slot);
			struct drm_device* dev = pvdrm_back_file_to_drm_device(file);
			struct drm_gem_object* obj = drm_gem_object_lookup(dev, pvdrm_back_file_to_drm_file(file), req->handle);
			if (!obj) {
				PVDRM_WARN("Invalid making handle to global handle:(%x)\n", req->handle);
				ret = -EINVAL;
				break;
			}

			/* Generate global handle. */
			ret = drm_gem_handle_create(pvdrm_back_file_to_drm_file(info->global), obj, &req->global);

			drm_gem_object_unreference_unlocked(obj);  /* Drop the reference from lookup. */
		}
		break;

        case PVDRM_GEM_FROM_GLOBAL_HANDLE: {
			struct drm_pvdrm_gem_global_handle* req = pvdrm_slot_payload(slot);
			struct drm_device* dev = pvdrm_back_file_to_drm_device(info->global);
			struct drm_gem_object* obj = drm_gem_object_lookup(dev, pvdrm_back_file_to_drm_file(info->global), req->global);
			if (!obj) {
				PVDRM_WARN("Invalid making global handle to handle:(%x)\n", req->global);
				ret = -EINVAL;
				break;
			}

			/* Adapt global handle. */
			ret = drm_gem_handle_create(pvdrm_back_file_to_drm_file(file), obj, &req->handle);

			drm_gem_object_unreference_unlocked(obj);  /* Drop the reference from lookup. */

		}
		break;

	default:
		PVDRM_DEBUG("unhandled slot %s:(%d)\n", pvdrm_op_str(slot->code), slot->code);
		break;
	}
done:

	/* set_fs(fs); */

	slot->ret = ret;

	/* Emit fence. */
	pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
	/* pvdrm_bench_close(&bench, NULL); */
	/* PVDRM_INFO("slot %s:(%d) is done with %llums\n", pvdrm_op_str(slot->code), slot->code, bench.elapsed.tv_sec * 1000ULL + (bench.elapsed.tv_nsec / 1000000ULL)); */
}

static int polling(void *arg)
{

	int ret;
	struct pvdrm_back_device* info;
	mm_segment_t fs;

	ret = 0;
	info = arg;

	/* Kick state. */
	PVDRM_DEBUG("Starting backend thread.\n");
	xenbus_switch_state(info->xbdev, XenbusStateConnected);
	PVDRM_DEBUG("backend thread connected with dom%d.\n", info->xbdev->otherend_id);

	{
		void* addr = NULL;
		int i;

		ret = xenbus_scanf(XBT_NIL, info->xbdev->otherend, "counter-ref", "%u", &info->ref);
		if (ret < 0) {
			xenbus_dev_fatal(info->xbdev, ret, "reading counter-ref");
			return ret;
		}
		PVDRM_DEBUG("mapping %u.\n", info->ref);

		info->device_path = xenbus_read(XBT_NIL, info->xbdev->nodename, "device-path", NULL);
		BUG_ON(!info->device_path);
		PVDRM_DEBUG("device-path %s.\n", info->device_path);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
		ret = xenbus_map_ring_valloc(info->xbdev, (grant_ref_t *)&(info->ref), 1, &addr);
#else
		ret = xenbus_map_ring_valloc(info->xbdev, info->ref, &addr);
#endif
		if (ret) {
			xenbus_dev_fatal(info->xbdev, ret, "mapping counter-ref");
			return ret;
		}
		info->mapped = addr;
		atomic_set(&info->get, UINT32_MAX);
		PVDRM_DEBUG("sizeof mapped id is %u.\n", pvdrm_slot_id(info->mapped, &info->mapped->slot[1]));
		for (i = 0; i < PVDRM_SLOT_NR; ++i) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
			ret = xenbus_map_ring_valloc(info->xbdev, (grant_ref_t *)&(info->mapped->slot[i].ref), 1, &info->slot_addrs[i]);
#else
			ret = xenbus_map_ring_valloc(info->xbdev, info->mapped->slot[i].ref, &info->slot_addrs[i]);
#endif
			if (ret) {
				xenbus_dev_fatal(info->xbdev, ret, "mapping slot addrs");
				return ret;
			}
		}
	}

	/* Open global file. */
	fs = get_fs();
	set_fs(get_ds());
	info->global = pvdrm_back_file_new(info);
	BUG_ON(!info->global);
	pvdrm_back_file_open_if_necessary(info, info->global->handle);

	PVDRM_INFO("Start main loop.\n");

	while (true) {
		ret = PVDRM_WAIT(kthread_should_stop() || pvdrm_back_count(info), true);
		if (ret < 0) {
			break;
		}

		if (kthread_should_stop()) {
			PVDRM_INFO("Thread should stop.\n");
			break;
		}

		if (pvdrm_back_count(info)) {
			struct pvdrm_slot* slot = claim_slot(info);
			struct pvdrm_back_work* work = pvdrm_back_slot_work(info, slot);
			work->slot = slot;
			work->info = info;
			if (pvdrm_back_sequential) {
				process_slot(&work->base);
			} else {
				INIT_WORK(&work->base, process_slot);
				queue_work(info->wq, &work->base);
			}
		}
	}

	pvdrm_back_file_destroy(info->global);
	set_fs(fs);
	PVDRM_INFO("End main loop.\n");

	return 0;
}

static int pvdrm_back_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	int ret;
	struct pvdrm_back_device* info;

	PVDRM_INFO("Proving backend driver %d.\n", xen_pv_domain());

	info = kzalloc(sizeof(struct pvdrm_back_device), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}
	info->xbdev = xbdev;
	dev_set_drvdata(&xbdev->dev, info);
	idr_init(&info->file_idr);
	spin_lock_init(&info->file_lock);

	info->wq = alloc_ordered_workqueue("pvdrm-back%d", 0, xbdev->otherend_id);
	if (!info->wq) {
		BUG();
	}

	ret = xenbus_switch_state(xbdev, XenbusStateInitWait);
	if (ret) {
		PVDRM_ERROR("failed");
		return ret;
	}
	return 0;
}

static int pvdrm_back_remove(struct xenbus_device *xbdev)
{
	struct pvdrm_back_device* info = NULL;

	PVDRM_INFO("Removing backend driver.\n");

	info = dev_get_drvdata(&xbdev->dev);
	flush_workqueue(info->wq);
	destroy_workqueue(info->wq);
	kfree(info);

	return 0;
}

static void frontend_changed(struct xenbus_device *xbdev, enum xenbus_state frontend_state)
{
	struct pvdrm_back_device* info;
	int ret = 0;

	PVDRM_INFO("Frontend changed backend driver to state %s.\n", xenbus_strstate(frontend_state));

	info = dev_get_drvdata(&xbdev->dev);

	PVDRM_INFO("%s", xenbus_strstate(frontend_state));

	switch (frontend_state) {
	case XenbusStateInitialising:
		if (xbdev->state == XenbusStateClosed) {
			PVDRM_INFO("xenbus is closed...\n");
			xenbus_switch_state(xbdev, XenbusStateInitWait);
		}
		break;

	case XenbusStateInitialised:
		break;

	case XenbusStateConnected:
		if (xbdev->state == XenbusStateConnected) {
			break;
		}

		/* OK, connected. */
		info->thread = kthread_run(polling, (void*)info, "pvdrm-back");
		break;

	case XenbusStateClosing:
		xenbus_switch_state(xbdev, XenbusStateClosing);
		break;

	case XenbusStateClosed:
		xenbus_switch_state(xbdev, XenbusStateClosed);
		if (xenbus_dev_is_online(xbdev)) {
			break;
		}
		if (info->thread) {
			kthread_stop(info->thread);
			info->thread = NULL;
		}
		pvdrm_back_info_destroy_files(info);

		/* fall through if not online */
	case XenbusStateUnknown:
		/* implies xen_blkif_disconnect() via xen_blkbk_remove() */
		device_unregister(&xbdev->dev);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret) {
		xenbus_dev_fatal(xbdev, ret, "saw state %d at frontend", frontend_state);
	}
}

static const struct xenbus_device_id pvdrm_back_ids[] = {
	{ "vdrm" },
	{ "" }
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
static struct xenbus_driver pvdrm_back_driver = {
	.name = "pvdrm-back",
	.ids = pvdrm_back_ids,
	.probe = pvdrm_back_probe,
	.remove = pvdrm_back_remove,
	.otherend_changed = frontend_changed
};
#else
static DEFINE_XENBUS_DRIVER(pvdrm_back, ,
	.probe = pvdrm_back_probe,
	.remove = pvdrm_back_remove,
	.otherend_changed = frontend_changed
);
#endif

static int __init pvdrm_back_init(void)
{
	/* int i, threads; */

	if (!xen_domain())
		return -ENODEV;

	pvdrm_back_core = kzalloc(sizeof(pvdrm_back_core_t), GFP_KERNEL);
	if (!pvdrm_back_core) {
		BUG();
		return -ENOMEM;
	}
	spin_lock_init(&pvdrm_back_core->req_lock);
	PVDRM_INFO("Initialising backend driver.\n");

	return xenbus_register_backend(&pvdrm_back_driver);
}
module_init(pvdrm_back_init);

static void __exit pvdrm_back_exit(void)
{
	kfree(pvdrm_back_core);
	xenbus_unregister_driver(&pvdrm_back_driver);
}
module_exit(pvdrm_back_exit);

MODULE_AUTHOR("Yusuke Suzuki");
MODULE_DESCRIPTION("PVDRM backend driver");
MODULE_LICENSE("GPL");
