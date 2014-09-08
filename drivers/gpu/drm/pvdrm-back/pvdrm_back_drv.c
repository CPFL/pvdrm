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

#include "drmP.h"

#include "../pvdrm/pvdrm_slot.h"

#include "xen_added_interface.h"  /* For domctl. */

typedef struct {
	spinlock_t req_lock;
	wait_queue_head_t req;
} pvdrm_back_core_t;

static pvdrm_back_core_t* pvdrm_back_core;

struct pvdrm_back_device {
	struct xenbus_device* xbdev;
	struct task_struct* thread;
	struct file* filp;
	grant_ref_t ref;
	struct pvdrm_mapped* mapped;

	struct list_head vmas;
};

struct drm_file* pvdrm_back_device_to_drm_file(struct pvdrm_back_device* info)
{
	return info->filp->private_data;
}

struct drm_device* pvdrm_back_device_to_drm_device(struct pvdrm_back_device* info)
{
	return pvdrm_back_device_to_drm_file(info)->minor->dev;
}

struct pvdrm_back_vma {
	struct vm_area_struct base;
	struct list_head next;
	struct vm_struct* area;
	pte_t** pteps;
	int* refs;
	uint64_t map_handle;
	uint32_t handle;
};

static struct pvdrm_back_vma* pvdrm_back_vma_alloc(struct pvdrm_back_device* info, uintptr_t start, uintptr_t end, unsigned long flags, unsigned long long map_handle, uint32_t handle)
{
	unsigned long i;
	unsigned long pages;
	unsigned long long size;
	uintptr_t addr;
	struct pvdrm_back_vma* vma;
	struct drm_gem_object* obj = NULL;
	struct drm_file* file_priv = NULL;
	struct drm_device* dev = NULL;

	file_priv = pvdrm_back_device_to_drm_file(info);
	dev = pvdrm_back_device_to_drm_device(info);

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		return NULL;
	}
	drm_gem_object_unreference(obj);

	size = (end - start);
	pages = size >> PAGE_SHIFT;

	vma = kzalloc(sizeof(struct pvdrm_back_vma), GFP_KERNEL);
	if (!vma) {
		BUG();
	}

	vma->map_handle = map_handle;
	vma->pteps = kzalloc(sizeof(pte_t*) * (pages + 1), GFP_KERNEL);
	if (!vma->pteps) {
		BUG();
	}

	vma->refs = kzalloc(sizeof(int) * (pages + 1), GFP_KERNEL);
	if (!vma->refs) {
		BUG();
	}

	vma->area = alloc_vm_area(size, vma->pteps);
	if (!vma->area) {
		BUG();
	}
	addr = (uintptr_t)vma->area->addr;

	for (i = 0; i < pages; ++i) {
		printk(KERN_INFO "PTE[%lu] = %s\n", i, (pte_none(*vma->pteps[i])) ? "none" : "value...");
	}

	printk(KERN_INFO "PVDRM:allocated area addresss size:(%llu), addr:(0x%llx), map_handle:(%llu).\n",
			(unsigned long long)size,
			(unsigned long long)addr,
			map_handle);

	vma->base.vm_mm = current->active_mm;
	vma->base.vm_start = (unsigned long)addr;
	vma->base.vm_end = ((unsigned long)addr) + size;
	vma->base.vm_flags = flags;
	vma->base.vm_page_prot = pgprot_writecombine(vm_get_page_prot(flags));
	/* vma->vm_page_prot = vm_get_page_prot(vm_flags); */
	vma->base.vm_pgoff = map_handle >> PAGE_SHIFT;
	vma->base.vm_file = info->filp;

	list_add(&vma->next, &info->vmas);
	return vma;
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
	pos = ((uint32_t)atomic_add_return(1, &info->mapped->get)) % PVDRM_SLOT_NR;
	id = info->mapped->ring[pos];
	return &info->mapped->slot[id];
}

struct copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
};

static int transfer(struct copier* copier, struct pvdrm_slot* slot, uint8_t* addr)
{
	if (slot->u.transfer.nr_buffers) {
		/* FIXME: Check buffer size doesn't exceed for the security issues. */
		const size_t size = slot->u.transfer.nr_buffers * sizeof(struct drm_nouveau_gem_pushbuf_bo);
		memcpy(copier->buffers, addr, size);
		addr += size;
		copier->buffers += slot->u.transfer.nr_buffers;
	}

	if (slot->u.transfer.nr_relocs) {
		const size_t size = slot->u.transfer.nr_relocs * sizeof(struct drm_nouveau_gem_pushbuf_reloc);
		memcpy(copier->relocs, addr, size);
		addr += size;
		copier->relocs += slot->u.transfer.nr_relocs;
	}

	if (slot->u.transfer.nr_push) {
		const size_t size = slot->u.transfer.nr_push * sizeof(struct drm_nouveau_gem_pushbuf_push);
		memcpy(copier->push, addr, size);
		addr += size;
		copier->push += slot->u.transfer.nr_push;
	}
	printk(KERN_INFO "PVDRM: Transferring pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", slot->u.transfer.nr_buffers, slot->u.transfer.nr_relocs, slot->u.transfer.nr_push);

	return 0;
}

static int process_pushbuf(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	/* buffers, reloc, push is user space pointers of the guest domain. */
	int ret = 0;
	struct drm_nouveau_gem_pushbuf* req = pvdrm_slot_payload(slot);
	struct drm_nouveau_gem_pushbuf_bo* buffers = NULL;
	struct drm_nouveau_gem_pushbuf_reloc* relocs = NULL;
	struct drm_nouveau_gem_pushbuf_push* push = NULL;
	void* addr = NULL;

	printk(KERN_INFO "PVDRM: pushbuf with ref %d\n", slot->u.transfer.ref);
	if (slot->u.transfer.ref < 0 || req->nr_push == 0) {
		/* OK, there's no buffers. */
		printk(KERN_INFO "PVDRM: pushbuf with no buffers...\n");
		/* FIXME: Check parameter is valid. */
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));
		goto destroy_data;
	}

	if (req->nr_buffers && req->buffers) {
		if (req->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS) {
			printk(KERN_ERR "PVDRM: pushbuf buffers are too large.\n");
			return -EINVAL;
		}
		buffers = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_bo) * req->nr_buffers, GFP_KERNEL);
		if (!buffers) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	if (req->nr_relocs && req->relocs) {
		if (req->nr_relocs > NOUVEAU_GEM_MAX_RELOCS) {
			printk(KERN_ERR "PVDRM: pushbuf relocs are too large.\n");
			ret = -EINVAL;
			goto destroy_data;
		}
		relocs = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_reloc) * req->nr_relocs, GFP_KERNEL);
		if (!relocs) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	if (req->nr_push && req->push) {
		if (req->nr_push > NOUVEAU_GEM_MAX_PUSH) {
			printk(KERN_ERR "PVDRM: pushbuf push are too large.\n");
			ret = -EINVAL;
			goto destroy_data;
		}
		push = kzalloc(sizeof(struct drm_nouveau_gem_pushbuf_push) * req->nr_push, GFP_KERNEL);
		if (!push) {
			ret = -ENOMEM;
			goto destroy_data;
		}
	}

	ret = xenbus_map_ring_valloc(info->xbdev, slot->u.transfer.ref, &addr);
	if (ret) {
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
		printk(KERN_INFO "PVDRM: Copying pushbufs...\n");
		do {
			printk(KERN_INFO "PVDRM: Copy! pushbuf...\n");
			ret = transfer(&copier, slot, addr);
			if (ret) {
				goto unmap_ref;
			}
			next = slot->u.transfer.next;
			if (next > 0) {
				pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
				pvdrm_fence_wait(&slot->__fence, 1, false);
			}
		} while (next > 0);
		printk(KERN_INFO "PVDRM: Copying pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", req->nr_buffers, req->nr_relocs, req->nr_push);
	}

	req->buffers = (unsigned long)buffers;
	req->relocs = (unsigned long)relocs;
	req->push = (unsigned long)push;

	ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_PUSHBUF, (unsigned long)pvdrm_slot_payload(slot));

unmap_ref:
	if (addr) {
		xenbus_unmap_ring_vfree(info->xbdev, addr);
	}

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

static struct pvdrm_back_vma* pvdrm_back_vma_find(struct pvdrm_back_device* info, uint64_t map_handle)
{
	struct list_head *listptr;
	struct pvdrm_back_vma* vma = NULL;
	list_for_each(listptr, &info->vmas) {
		vma = list_entry(listptr, struct pvdrm_back_vma, next);
		if (vma->map_handle == map_handle) {
			return vma;
		}
	}
	return NULL;
}

static int memory_mapping(struct pvdrm_back_device* info, uint64_t first_gfn, uint64_t first_mfn, uint64_t nr_mfns, bool add_mapping)
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

static inline uint32_max(uint32_t a, uint32_t b)
{
	return (a > b) ? a : b;
}

static inline uint32_min(uint32_t a, uint32_t b)
{
	return (a > b) ? b : a;
}

static int process_fault(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	int i;
	int ret = 0;
	int result = 0;
	uint64_t page_offset = 0;
	uint32_t max;
	uint32_t max_limited_by_vm_end;
	bool already_faulted = false;
	struct pvdrm_mapping* refs = NULL;
	struct drm_pvdrm_gem_fault* req = pvdrm_slot_payload(slot);
	struct pvdrm_back_vma* vma = NULL;
	struct vm_fault vmf;
	bool is_iomem = req->domain & NOUVEAU_GEM_DOMAIN_VRAM;

	vma = pvdrm_back_vma_find(info, req->map_handle);
	if (!vma) {
		return -EINVAL;
	}

	page_offset = req->offset >> PAGE_SHIFT;
	printk(KERN_INFO "Fault.... start with %llu, offset:(0x%lx) (0x%lx => 0x%lx . 0x%lx)\n", (unsigned long long)page_offset, (unsigned long)req->offset, vma->base.vm_start, vma->base.vm_end, (unsigned long)(vma->base.vm_start + req->offset));
	if (!pte_none(*vma->pteps[page_offset])) {
		already_faulted = true;
	}

	if (!already_faulted) {
		vmf = (struct vm_fault) {
			.flags = req->flags,
			.pgoff = req->pgoff,
			.virtual_address = (void*)(vma->base.vm_start + req->offset),
		};
		printk(KERN_INFO "Fault.... start with %llu start!\n", (unsigned long long)page_offset);
		do {
			ret = vma->base.vm_ops->fault(&vma->base, &vmf);
			if (ret & VM_FAULT_ERROR) {
				BUG();
			}
		} while (ret & VM_FAULT_RETRY);

		if (ret & VM_FAULT_NOPAGE) {
			/* page is installed. */
		} else if (ret & VM_FAULT_LOCKED) {
			/* FIXME: should install page. */
		}
	}

	max = 16;  /* FIXME: Temporary value. */

	max_limited_by_vm_end = ((vma->base.vm_end - vma->base.vm_start) >> PAGE_SHIFT) - page_offset;
	max = min(max_limited_by_vm_end, max);
	max = min(req->nr_pages, max);

	for (i = 0; i < max; ++i) {
		/* Not mappable page? */
		if (pte_none(*vma->pteps[page_offset + i])) {
			break;
		}
		/* Already mapped in the quest? */
		if (vma->refs[page_offset + i] > 0) {
			break;
		}
	}
	max = i;

	if (!is_iomem) {
		// printk(KERN_INFO "PVDRM: mmap is done with %u / 0x%llx / 0x%llx , ref %d\n",
		// 		ret,
		// 		(unsigned long long)vmf.virtual_address,
		// 		(unsigned long long)page_to_phys(pte_page(*(vma->pteps[0]))),
		// 		req->ref);

		ret = xenbus_map_ring_valloc(info->xbdev, req->ref, (void*)&refs);
		if (ret) {
			BUG();
		}
		for (i = 0; i < max; ++i) {
			int offset = page_offset + i;
			int ref = gnttab_grant_foreign_access(info->xbdev->otherend_id, pfn_to_mfn(page_to_pfn(pte_page(*(vma->pteps[offset])))), 0);
			printk(KERN_INFO "PVDRM: to dom%d mmap is done with %d / 0x%llx\n",
					info->xbdev->otherend_id,
					ref,
					(unsigned long long)pfn_to_mfn(page_to_pfn(pte_page(*(vma->pteps[offset])))));
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
		xenbus_unmap_ring_vfree(info->xbdev, (void*)refs);
	} else {
		unsigned long mfn = pfn_to_mfn(page_to_pfn(pte_page(*(vma->pteps[page_offset]))));
		ret = memory_mapping(info, req->backing, mfn, max, true);
		req->mapped_count = max;
	}
	return ret;
}

static int process_mmap(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	int ret = 0;
	struct drm_pvdrm_gem_mmap* req = pvdrm_slot_payload(slot);
	struct pvdrm_back_vma* vma = NULL;

	vma = pvdrm_back_vma_alloc(info, req->vm_start, req->vm_end, req->flags, req->map_handle, req->handle);
	if (!vma) {
		BUG();
	}
	/* Call f_op->mmap operation directly since vm_mmap requires current->mm
	 * is not NULL.
	 */
	ret = info->filp->f_op->mmap(info->filp, &vma->base);
	if (ret) {
		BUG();
		return -EINVAL;
	}

	return ret;
}

static int process_slot(struct pvdrm_back_device* info, struct pvdrm_slot* slot)
{
	int ret;
	struct drm_file* file_priv = NULL;
	struct drm_device* dev = NULL;
	mm_segment_t fs;

	ret = 0;
	file_priv = pvdrm_back_device_to_drm_file(info);
	dev = pvdrm_back_device_to_drm_device(info);

	fs = get_fs();
	set_fs(get_ds());
	printk(KERN_INFO "PVDRM: processing slot %d\n", slot->code);
	/* msleep(1000); */

	/* Processing slot. */
	/* FIXME: Need to check in the host side. */
	switch (slot->code) {
	case PVDRM_IOCTL_NOUVEAU_GETPARAM:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GETPARAM, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, (unsigned long)pvdrm_slot_payload(slot));
		printk(KERN_INFO "PVDRM: allocate channel id %d\n", ((struct drm_nouveau_channel_alloc*)(pvdrm_slot_payload(slot)))->channel);
		break;

	case PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_CHANNEL_FREE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_INFO:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_INFO, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_NEW:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_NEW, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF:
		ret = process_pushbuf(info, slot);
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_PREP, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI:
		ret = drm_ioctl(info->filp, DRM_IOCTL_NOUVEAU_GEM_CPU_FINI, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_NOUVEAU_GEM_FREE: {
			/* FIXME: Need to investigate more... */
			struct drm_pvdrm_gem_free* req = pvdrm_slot_payload(slot);
			struct drm_gem_object* obj = drm_gem_object_lookup(dev, file_priv, req->handle);
			if (!obj) {
				ret = -EINVAL;
				break;
			}
			drm_gem_object_handle_free(obj);
			drm_gem_object_free(&obj->refcount);
			// kfree(obj);
			ret = 0;
		}
		break;

	case PVDRM_GEM_NOUVEAU_GEM_CLOSE:
		ret = drm_ioctl(info->filp, DRM_IOCTL_GEM_CLOSE, (unsigned long)pvdrm_slot_payload(slot));
		break;

	case PVDRM_GEM_NOUVEAU_GEM_MMAP:
		/* FIXME: Need to check... */
		ret = process_mmap(info, slot);
		break;

	case PVDRM_GEM_NOUVEAU_GEM_FAULT:
		/* FIXME: Need to check... */
		ret = process_fault(info, slot);
		break;

	default:
		printk(KERN_INFO "PVDRM: unhandled slot %d\n", slot->code);
		break;
	}
	set_fs(fs);

	slot->ret = ret;

	/* Emit fence. */
	pvdrm_fence_emit(&slot->__fence, PVDRM_FENCE_DONE);
	printk(KERN_INFO "PVDRM: slot %d is done\n", slot->code);
	return ret;
}

static int thread_main(void *arg)
{

	int ret;
	struct pvdrm_back_device* info;

	ret = 0;
	info = arg;

	/* Kick state. */
	printk(KERN_INFO "Starting PVDRM backend thread.\n");
	xenbus_switch_state(info->xbdev, XenbusStateConnected);
	printk(KERN_INFO "PVDRM backend thread connected with dom%d.\n", info->xbdev->otherend_id);

	{
		void* addr = NULL;

		ret = xenbus_scanf(XBT_NIL, info->xbdev->otherend, "counter-ref", "%u", &info->ref);
		if (ret < 0) {
			xenbus_dev_fatal(info->xbdev, ret, "reading counter-ref");
			return ret;
		}
		printk(KERN_INFO "PVDRM: mapping %u.\n", info->ref);

		ret = xenbus_map_ring_valloc(info->xbdev, info->ref, &addr);
		if (ret) {
			xenbus_dev_fatal(info->xbdev, ret, "mapping counter-ref");
			return ret;
		}
		info->mapped = addr;
		printk(KERN_INFO "PVDRM: sizeof mapped id is %u.\n", pvdrm_slot_id(info->mapped, &info->mapped->slot[31]));
	}

	/* Open DRM file. */
	{
		mm_segment_t fs;
		fs = get_fs();
		set_fs(get_ds());
		/* FIXME: Currently we use this path directly. */
		info->filp = filp_open("/dev/dri/card0", O_RDWR, 0);
		set_fs(fs);

		printk(KERN_INFO "PVDRM: Opened drm device.\n");
	}

	printk(KERN_INFO "PVDRM: Start main loop.\n");
	while (true) {
		while (!kthread_should_stop() && !pvdrm_back_count(info)) {
			/* Sleep. */
			ktime_t time;
			__set_current_state(TASK_INTERRUPTIBLE);
			time = ktime_set(0, 200);  /* This value derived from Paradice [ASPLOS '14]. */
			schedule_hrtimeout(&time, HRTIMER_MODE_REL);
		}

		if (kthread_should_stop()) {
			printk(KERN_INFO "PVDRM: Thread should stop.\n");
			break;
		}

		if (pvdrm_back_count(info)) {
			struct pvdrm_slot* slot = claim_slot(info);
			process_slot(info, slot);
		}
	}
	printk(KERN_INFO "PVDRM: End main loop.\n");

	/* Close DRM file. */
	if (info->filp) {
		mm_segment_t fs;
		fs = get_fs();
		set_fs(get_ds());
		filp_close(info->filp, NULL);
		set_fs(fs);
		info->filp = NULL;
	}

	return 0;
}

static int pvdrm_back_probe(struct xenbus_device *xbdev, const struct xenbus_device_id *id)
{
	int ret;
	struct pvdrm_back_device* info;

	printk(KERN_INFO "Proving PVDRM backend driver %d.\n", xen_pv_domain());

	info = kzalloc(sizeof(struct pvdrm_back_device), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}
	info->xbdev = xbdev;
	dev_set_drvdata(&xbdev->dev, info);
	INIT_LIST_HEAD(&info->vmas);

	ret = xenbus_switch_state(xbdev, XenbusStateInitWait);
	if (ret) {
		printk(KERN_INFO "PVDRM failed");
		return ret;
	}
	return 0;
}

static int pvdrm_back_remove(struct xenbus_device *xbdev)
{
	struct pvdrm_back_device* info = NULL;

	printk(KERN_INFO "Removing PVDRM backend driver.\n");

	info = dev_get_drvdata(&xbdev->dev);
	kfree(info);

	return 0;
}

static void frontend_changed(struct xenbus_device *xbdev, enum xenbus_state frontend_state)
{
	struct pvdrm_back_device* info;
	int ret = 0;

	printk(KERN_INFO "Frontend changed PVDRM backend driver to state %s.\n", xenbus_strstate(frontend_state));

	info = dev_get_drvdata(&xbdev->dev);

	printk(KERN_INFO "%s", xenbus_strstate(frontend_state));

	switch (frontend_state) {
	case XenbusStateInitialising:
		if (xbdev->state == XenbusStateClosed) {
			printk(KERN_INFO "PVDRM xenbus is closed...\n");
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
		info->thread = kthread_run(thread_main, (void*)info, "pvdrm-back");
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

static DEFINE_XENBUS_DRIVER(pvdrm_back, ,
	.probe = pvdrm_back_probe,
	.remove = pvdrm_back_remove,
	.otherend_changed = frontend_changed
);

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

#if 0
	threads = num_online_cpus();
	for (i = 0; i < threads; ++i) {
	}
#endif

	spin_lock_init(&pvdrm_back_core->req_lock);
	init_waitqueue_head(&pvdrm_back_core->req);

	printk(KERN_INFO "Initialising PVDRM backend driver.\n");

	return xenbus_register_backend(&pvdrm_back_driver);
}
module_init(pvdrm_back_init);

static void __exit pvdrm_back_exit(void)
{
	xenbus_unregister_driver(&pvdrm_back_driver);
}
module_exit(pvdrm_back_exit);

MODULE_AUTHOR("Yusuke Suzuki");
MODULE_DESCRIPTION("PVDRM backend driver");
MODULE_LICENSE("GPL");
/* vim: set sw=8 ts=8 et tw=80 : */
