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
#include <drm/nouveau_drm.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>

#include "drmP.h"
#include "drm.h"

/* Include nouveau's abi16 header directly. */
#include "../nouveau/nouveau_abi16.h"

#include "pvdrm_cast.h"
#include "pvdrm_gem.h"
#include "pvdrm_nouveau_abi16.h"
#include "pvdrm_pushbuf.h"
#include "pvdrm_slot.h"

struct pushbuf_copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	uint32_t nr_buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	uint32_t nr_relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
	uint32_t nr_push;
};

static size_t can_transfer_buffers_nr(struct pushbuf_copier* copier, size_t rest)
{
	if (copier->nr_buffers * sizeof(struct drm_nouveau_gem_pushbuf_bo) <= rest) {
		return copier->nr_buffers;
	}
	return rest / sizeof(struct drm_nouveau_gem_pushbuf_bo);
}

static size_t can_transfer_relocs_nr(struct pushbuf_copier* copier, size_t rest)
{
	if (copier->nr_relocs * sizeof(struct drm_nouveau_gem_pushbuf_reloc) <= rest) {
		return copier->nr_relocs;
	}
	return rest / sizeof(struct drm_nouveau_gem_pushbuf_reloc);
}

static size_t can_transfer_push_nr(struct pushbuf_copier* copier, size_t rest)
{
	if (copier->nr_push * sizeof(struct drm_nouveau_gem_pushbuf_push) <= rest) {
		return copier->nr_push;
	}
	return rest / sizeof(struct drm_nouveau_gem_pushbuf_push);
}

static int transfer(struct drm_device* dev, struct drm_file* file, struct pushbuf_copier* copier, uint8_t* addr, struct pvdrm_slot* slot)
{
	size_t rest = PAGE_SIZE;
	size_t nr;
	size_t i;

	slot->u.transfer.nr_buffers = 0;
	slot->u.transfer.nr_relocs = 0;
	slot->u.transfer.nr_push = 0;

	nr = can_transfer_buffers_nr(copier, rest);
	if (nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_bo) * nr;
		struct drm_nouveau_gem_pushbuf_bo* buffers = (struct drm_nouveau_gem_pushbuf_bo*)addr;
		copy_from_user(addr, copier->buffers, size);
		/* Since we first copies buffers to the page, these buffers in the kernel memory are always aligned correctly. */
		for (i = 0; i < nr; ++i) {
			uint32_t handle = buffers[i].handle;
			struct drm_pvdrm_gem_object* obj = NULL;
			obj = pvdrm_gem_object_lookup(dev, file, handle);
			if (!obj) {
				printk(KERN_ERR "PVDRM: pushbuf No valid obj with handle %u.\n", handle);
				return -EINVAL;
			}
			buffers[i].handle = obj->host;
			drm_gem_object_unreference(&obj->base);
		}

		copier->buffers += nr;
		copier->nr_buffers -= nr;
		addr += size;
		rest -= size;
		slot->u.transfer.nr_buffers = nr;
	}

	nr = can_transfer_relocs_nr(copier, rest);
	if (nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_reloc) * nr;
		copy_from_user(addr, copier->relocs, size);

		copier->relocs += nr;
		copier->nr_relocs -= nr;
		addr += size;
		rest -= size;
		slot->u.transfer.nr_relocs = nr;
	}

	nr = can_transfer_push_nr(copier, rest);
	if (nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_push) * nr;
		copy_from_user(addr, copier->push, size);

		copier->push += nr;
		copier->nr_push -= nr;
		addr += size;
		rest -= size;
		slot->u.transfer.nr_push = nr;
	}

	printk(KERN_INFO "PVDRM: Transferring pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", slot->u.transfer.nr_buffers, slot->u.transfer.nr_relocs, slot->u.transfer.nr_push);

	if (!copier->nr_buffers && !copier->nr_relocs && !copier->nr_push) {
		/* All data is transferred. */
		return 0;
	}

	/* Continue tranferring. */
	return 1;
}

int pvdrm_pushbuf(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_gem_pushbuf* req_out)
{
	struct pvdrm_device* pvdrm;
	struct drm_pvdrm_gem_object* chan;
	int ret = 0;
	uint8_t* vaddr = NULL;
	grant_ref_t ref = -ENOMEM;
	struct xenbus_device* xbdev = NULL;

	pvdrm = drm_device_to_pvdrm(dev);
	xbdev = pvdrm_to_xbdev(pvdrm);

	chan = pvdrm_gem_object_lookup(dev, file, req_out->channel);
	if (!chan) {
		goto exit;
	}
	req_out->channel = chan->host;

	if (req_out->nr_push == 0) {
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);
		printk(KERN_INFO "PVDRM: pushbuf with no buffers...\n");
		slot->code = PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF;
		memcpy(pvdrm_slot_payload(slot), req_out, sizeof(struct drm_nouveau_gem_pushbuf));
		slot->u.transfer.ref = -ENOMEM;
		ret = pvdrm_slot_request(pvdrm, slot);
		memcpy(req_out, pvdrm_slot_payload(slot), sizeof(struct drm_nouveau_gem_pushbuf));
		ret = slot->ret;
		pvdrm_slot_free(pvdrm, slot);
		goto close_channel;
	}

	if (req_out->nr_buffers && req_out->buffers) {
		if (req_out->nr_buffers > NOUVEAU_GEM_MAX_BUFFERS) {
			ret = -EINVAL;
			goto close_channel;
		}
	}

	if (req_out->nr_relocs && req_out->relocs) {
		if (req_out->nr_relocs > NOUVEAU_GEM_MAX_RELOCS) {
			ret = -EINVAL;
			goto close_channel;
		}
	}

	if (req_out->nr_push && req_out->push) {
		if (req_out->nr_push > NOUVEAU_GEM_MAX_PUSH) {
			ret = -EINVAL;
			goto close_channel;
		}
	}

	printk(KERN_INFO "PVDRM: Copying pushbufs...\n");

	/* Allocate slot and counter ref. */
	{
		vaddr = (void*)get_zeroed_page(GFP_NOIO | __GFP_HIGH);
		if (!vaddr) {
			ret = -ENOMEM;
			xenbus_dev_fatal(xbdev, ret, "allocating ring page");
			goto close_channel;
		}

		ref = xenbus_grant_ring(xbdev, virt_to_mfn(vaddr));
		if (ref < 0) {
			xenbus_dev_fatal(xbdev, ref, "granting ring page");
			ret = ref;
			goto free_page;
		}
	}

	{
		struct pushbuf_copier copier = {
			.buffers    = (void*)req_out->buffers,
			.nr_buffers = req_out->nr_buffers,
			.relocs     = (void*)req_out->relocs,
			.nr_relocs  = req_out->nr_relocs,
			.push       = (void*)req_out->push,
			.nr_push    = req_out->nr_push
		};
		int first = 1;
		int next = 0;
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm);
		slot->code = PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF;
		slot->u.transfer.ref = ref;
		memcpy(pvdrm_slot_payload(slot), req_out, sizeof(struct drm_nouveau_gem_pushbuf));

		/* Call. */
		do {
			printk(KERN_INFO "PVDRM: Copy! pushbuf...\n");
			next = transfer(dev, file, &copier, vaddr, slot);
			slot->u.transfer.next = next;
			if (first) {
				first = 0;
				pvdrm_slot_request_async(pvdrm, slot);
				pvdrm_fence_wait(&slot->__fence, PVDRM_FENCE_DONE, false);
			} else {
				pvdrm_fence_emit(&slot->__fence, 1);
				pvdrm_fence_wait(&slot->__fence, PVDRM_FENCE_DONE, false);
			}
		} while (next > 0);

		memcpy(req_out, pvdrm_slot_payload(slot), sizeof(struct drm_nouveau_gem_pushbuf));
		ret = slot->ret;
		pvdrm_slot_free(pvdrm, slot);
	}

	if (ref >= 0) {
		gnttab_free_grant_reference(ref);
	}

	printk(KERN_INFO "PVDRM: Copying pushbuf... Done.\n");

free_page:
	if (vaddr) {
		free_page((unsigned long)vaddr);
	}

close_channel:
	drm_gem_object_unreference(&chan->base);

exit:
	return ret;
}

/* vim: set sw=4 et ts=4: */
