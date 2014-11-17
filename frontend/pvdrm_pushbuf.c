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
#include <linux/kernel.h>
#include <drm/nouveau_drm.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>

#include <drmP.h>

#include <common/pvdrm_log.h>
#include <common/pvdrm_slot.h>

#include "pvdrm_cast.h"
#include "pvdrm_channel.h"
#include "pvdrm_gem.h"
#include "pvdrm_nouveau_abi16.h"
#include "pvdrm_pushbuf.h"

struct pushbuf_copier {
	struct drm_nouveau_gem_pushbuf_bo* buffers;
	uint32_t nr_buffers;
	struct drm_nouveau_gem_pushbuf_reloc* relocs;
	uint32_t nr_relocs;
	struct drm_nouveau_gem_pushbuf_push* push;
	uint32_t nr_push;
};

struct pvdrm_addr_and_nr {
	uint8_t* addr;
	size_t nr;
};

static struct pvdrm_addr_and_nr can_transfer_buffers_nr(struct pushbuf_copier* copier, uint8_t* addr, uint8_t* last)
{
	struct drm_nouveau_gem_pushbuf_bo* end = (struct drm_nouveau_gem_pushbuf_bo*)last;
	struct drm_nouveau_gem_pushbuf_bo* tip = (struct drm_nouveau_gem_pushbuf_bo*)PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_bo));
	size_t nr = copier->nr_buffers;

	if ((tip + nr) <= end) {
		return (struct pvdrm_addr_and_nr) {
			.addr = (void*)tip,
			.nr = nr,
		};
	}
	return (struct pvdrm_addr_and_nr) {
		.addr = (void*)tip,
		.nr = (end - tip),
	};
}

static struct pvdrm_addr_and_nr can_transfer_relocs_nr(struct pushbuf_copier* copier, uint8_t* addr, uint8_t* last)
{
	struct drm_nouveau_gem_pushbuf_reloc* end = (struct drm_nouveau_gem_pushbuf_reloc*)last;
	struct drm_nouveau_gem_pushbuf_reloc* tip = (struct drm_nouveau_gem_pushbuf_reloc*)PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_reloc));
	size_t nr = copier->nr_relocs;

	if ((tip + nr) <= end) {
		return (struct pvdrm_addr_and_nr) {
			.addr = (void*)tip,
			.nr = nr,
		};
	}
	return (struct pvdrm_addr_and_nr) {
		.addr = (void*)tip,
		.nr = (end - tip),
	};
}

static struct pvdrm_addr_and_nr can_transfer_push_nr(struct pushbuf_copier* copier, uint8_t* addr, uint8_t* last)
{
	struct drm_nouveau_gem_pushbuf_push* end = (struct drm_nouveau_gem_pushbuf_push*)last;
	struct drm_nouveau_gem_pushbuf_push* tip = (struct drm_nouveau_gem_pushbuf_push*)PTR_ALIGN(addr, sizeof(struct drm_nouveau_gem_pushbuf_push));
	size_t nr = copier->nr_push;

	if ((tip + nr) <= end) {
		return (struct pvdrm_addr_and_nr) {
			.addr = (void*)tip,
			.nr = nr,
		};
	}
	return (struct pvdrm_addr_and_nr) {
		.addr = (void*)tip,
		.nr = (end - tip),
	};
}

static int transfer(struct drm_device* dev, struct drm_file* file, struct pushbuf_copier* copier, uint8_t* addr, struct pvdrm_slot* slot)
{
	uint8_t* last = (uint8_t*)(((uintptr_t)addr) + PAGE_SIZE);
	struct pvdrm_addr_and_nr pair = { 0 };
	size_t i;

	slot->u.transfer.nr_buffers = 0;
	slot->u.transfer.nr_relocs = 0;
	slot->u.transfer.nr_push = 0;

	pair = can_transfer_buffers_nr(copier, addr, last);
	if (pair.nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_bo) * pair.nr;
		struct drm_nouveau_gem_pushbuf_bo* buffers = (struct drm_nouveau_gem_pushbuf_bo*)pair.addr;
		copy_from_user(pair.addr, copier->buffers, size);
		for (i = 0; i < pair.nr; ++i) {
			uint32_t handle = buffers[i].handle;
			struct drm_pvdrm_gem_object* obj = NULL;
			obj = pvdrm_gem_object_lookup(dev, file, handle);
			if (!obj) {
				PVDRM_ERROR("pushbuf No valid obj with handle %u.\n", handle);
				return -EINVAL;
			}
			buffers[i].handle = pvdrm_gem_host(drm_file_to_fpriv(file), obj);
			drm_gem_object_unreference(&obj->base);
		}

		copier->buffers += pair.nr;
		copier->nr_buffers -= pair.nr;
		addr = (pair.addr + size);
		slot->u.transfer.nr_buffers = pair.nr;
	}

	pair = can_transfer_relocs_nr(copier, addr, last);
	if (pair.nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_reloc) * pair.nr;
		copy_from_user(pair.addr, copier->relocs, size);

		copier->relocs += pair.nr;
		copier->nr_relocs -= pair.nr;
		addr = (pair.addr + size);
		slot->u.transfer.nr_relocs = pair.nr;
	}

	pair = can_transfer_push_nr(copier, addr, last);
	if (pair.nr) {
		const size_t size = sizeof(struct drm_nouveau_gem_pushbuf_push) * pair.nr;
		copy_from_user(pair.addr, copier->push, size);

		copier->push += pair.nr;
		copier->nr_push -= pair.nr;
		addr = (pair.addr + size);
		slot->u.transfer.nr_push = pair.nr;
	}

	PVDRM_DEBUG("Transferring pushbuf... Done. buffers:%u, relocs:%u, push:%u.\n", slot->u.transfer.nr_buffers, slot->u.transfer.nr_relocs, slot->u.transfer.nr_push);

	if (!copier->nr_buffers && !copier->nr_relocs && !copier->nr_push) {
		/* All data is transferred. */
		return 0;
	}

	/* Continue tranferring. */
	return 1;
}

int pvdrm_pushbuf(struct drm_device* dev, struct drm_file* file, struct drm_nouveau_gem_pushbuf* req_out)
{
	struct pvdrm_device* pvdrm;
	struct pvdrm_channel* chan;
	int ret = 0;
	struct xenbus_device* xbdev = NULL;
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);

	pvdrm = drm_device_to_pvdrm(dev);
	xbdev = pvdrm_to_xbdev(pvdrm);

	chan = pvdrm_channel_lookup(dev, req_out->channel);
	if (!chan) {
		goto exit;
	}
	req_out->channel = chan->host;

	if (req_out->nr_push == 0) {
		PVDRM_DEBUG("pushbuf with no buffers...\n");
		pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF, req_out, sizeof(struct drm_nouveau_gem_pushbuf));
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

	PVDRM_DEBUG("Copying pushbufs...\n");

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
		uint8_t* vaddr;
		struct pvdrm_slot* slot = NULL;

		slot = pvdrm_slot_alloc(pvdrm, fpriv->host);
		vaddr = (uint8_t*)slot->addr;
		slot->code = PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF;

		memcpy(pvdrm_slot_payload(slot), req_out, sizeof(struct drm_nouveau_gem_pushbuf));

		/* Call. */
		do {
			PVDRM_DEBUG("Copy! pushbuf...\n");
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

	PVDRM_DEBUG("Copying pushbuf... Done.\n");

close_channel:
	pvdrm_channel_unreference(chan);

exit:
	return ret;
}
