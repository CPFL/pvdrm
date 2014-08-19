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

#include <linux/atomic.h>
#include <linux/types.h>
#include <linux/semaphore.h>
#include <linux/string.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>

#include "pvdrm_drm.h"
#include "pvdrm_slot.h"

static int init_slot_internal(grant_ref_t* gref_head, struct pvdrm_slot_internal* internal)
{
	struct page* page;
	uintptr_t pfn, mfn;
	grant_ref_t ref = gnttab_claim_grant_reference(gref_head);

	if (!(page = alloc_page(GFP_HIGHUSER))) {
		BUG();
		return -ENOSYS;
	}

	pfn = page_to_pfn(page);
	mfn = pfn_to_mfn(pfn);
	gnttab_grant_foreign_access_ref(ref, /* DOM0 */ 0, mfn, 0);

	internal->addr = NULL;
	internal->page = page;
	internal->ref = ref;
	internal->used = false;

	return 0;
}


int pvdrm_slot_init(struct pvdrm_device* pvdrm)
{
	int i;
	int ret;
	struct pvdrm_slots* slots;
	grant_ref_t gref_head;

        printk(KERN_INFO "PVDRM: Initializing pvdrm slots.\n");
	ret = 0;
	slots = &pvdrm->slots;
	sema_init(&slots->sema, PVDRM_SLOT_NR);
	spin_lock_init(&slots->lock);

	/* Allocate slot and counter ref. */
	if (gnttab_alloc_grant_references(PVDRM_SLOT_NR + 1, &gref_head)) {
		BUG();
		return -ENOSYS;
	}

	/* Init counter. */
	init_slot_internal(&gref_head, &slots->counter_internal);
	if (ret) {
		return ret;
	}
	slots->counter_internal.addr = slots->counter = kmap(slots->counter_internal.page);

        printk(KERN_INFO "PVDRM: Initialized pvdrm counter.\n");

	/* Init slots. */
	for (i = 0; i < PVDRM_SLOT_NR; ++i) {
		ret = init_slot_internal(&gref_head, &slots->internals[i]);
		if (ret) {
			return ret;
		}
                /* Writing ref values into counter's ring to notify to the host. */
                slots->counter->ring[i] = slots->internals[i].ref;
	}

	gnttab_free_grant_references(gref_head);

        printk(KERN_INFO "PVDRM: Initialized pvdrm slots.\n");

	return 0;
}

struct pvdrm_slot* pvdrm_slot_alloc(struct pvdrm_device* pvdrm)
{
	int i;
	struct pvdrm_slots* slots;
	struct pvdrm_slot* slot;
	unsigned long flags;

	slots = &pvdrm->slots;

	down(&slots->sema);
	spin_lock_irqsave(&slots->lock, flags);
	for (i = 0; i < PVDRM_SLOT_NR; ++i) {
		if (!slots->internals[i].used) {
			slots->internals[i].used = true;
			break;
		}
	}

	BUG_ON(i == PVDRM_SLOT_NR);

	if (slots->internals[i].addr) {
		slot = slots->internals[i].addr;
	} else {
		slot = kmap(slots->internals[i].page);
		slot->__id = i;
		slots->internals[i].addr = slot;
	}
	spin_unlock_irqrestore(&slots->lock, flags);

	/* Init slot. */
	pvdrm_fence_init(&slot->__fence);
	slot->ret = 0;
	return slot;
}

void pvdrm_slot_free(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	struct pvdrm_slots* slots;
	unsigned long flags;

	slots = &pvdrm->slots;

	spin_lock_irqsave(&slots->lock, flags);

	BUG_ON(!slots->internals[slot->__id].used);
	slots->internals[slot->__id].used = false;

	spin_unlock_irqrestore(&slots->lock, flags);
	up(&slots->sema);
}

int pvdrm_slot_request(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	/* TODO: Implement it, emitting fence here. */
	struct pvdrm_slots* slots;
	int ret;

	slots = &pvdrm->slots;

	BUG_ON(!slots->internals[slot->__id].used);

	/* Request slot, increment counter. */
	mb();
	atomic_inc(&slots->counter->count);

	/* Wait. */
	ret = pvdrm_fence_wait(&slot->__fence, false);
	if (ret) {
		return ret;
	}
	return slot->ret;
}


/* vim: set sw=8 ts=8 et tw=80 : */
