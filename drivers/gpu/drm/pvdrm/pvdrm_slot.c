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
#include <linux/types.h>
#include <linux/semaphore.h>
#include <linux/string.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/grant_table.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>

#include "pvdrm_cast.h"
#include "pvdrm_drm.h"
#include "pvdrm_limits.h"
#include "pvdrm_log.h"
#include "pvdrm_slot.h"

static bool is_used(struct pvdrm_slot* slot)
{
        return slot->code != PVDRM_UNUSED;
}

static int pvdrm_slot_ensure_ref(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	int ret = 0;
	uintptr_t addr = 0;
	struct xenbus_device* xbdev = pvdrm_to_xbdev(pvdrm);

	if (slot->ref >= 0) {
		return 0;
	}

	/* Allocate ref. */
	addr = get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!addr) {
		ret = -ENOMEM;
		xenbus_dev_fatal(xbdev, ret, "allocating ring page");
		return ret;
	}

	ret = xenbus_grant_ring(xbdev, virt_to_mfn(addr));
	if (ret < 0) {
		xenbus_dev_fatal(xbdev, ret, "granting ring page");
		free_page(addr);
		return ret;
	}

	slot->ref = ret;
	slot->addr = (void*)addr;

	return 0;
}

int pvdrm_slots_init(struct pvdrm_device* pvdrm)
{
	int i;
	int ret;
	struct pvdrm_slots* slots;
	struct xenbus_device* xbdev;
        struct pvdrm_mapped* mapped;
        spinlock_t* lock;
	struct semaphore* sema;

        BUILD_BUG_ON(sizeof(struct pvdrm_mapped) > PAGE_SIZE);

        PVDRM_INFO("PVDRM: Initializing pvdrm slots.\n");
	ret = 0;

	slots = kzalloc(sizeof(struct pvdrm_slots), GFP_KERNEL);
	if (!slots) {
		return -ENOMEM;
	}
	pvdrm->slots = slots;

	xbdev = pvdrm_to_xbdev(pvdrm);

        sema = &slots->sema;
	sema_init(sema, PVDRM_SLOT_NR);

        lock = &slots->lock;
        spin_lock_init(lock);

	/* Allocate slot and counter ref. */
	{
		const uintptr_t vaddr = get_zeroed_page(GFP_NOIO | __GFP_HIGH);
		if (!vaddr) {
			ret = -ENOMEM;
			xenbus_dev_fatal(xbdev, ret, "allocating ring page");
			return ret;
		}

		ret = xenbus_grant_ring(xbdev, virt_to_mfn(vaddr));
		if (ret < 0) {
			xenbus_dev_fatal(xbdev, ret, "granting ring page");
			free_page(vaddr);
			return ret;
		}

		slots->ref = ret;
		slots->mapped = (void*)vaddr;
	}
	PVDRM_INFO("PVDRM: Initialising pvdrm counter reference %u.\n", slots->ref);

        mapped = slots->mapped;

	/* Init counter. */
        atomic_set(&mapped->count, 0);
        atomic_set(&slots->put, UINT32_MAX);

        PVDRM_INFO("PVDRM: Initialized pvdrm counter.\n");

	/* Init slots. */
	for (i = 0; i < PVDRM_SLOT_NR; ++i) {
                struct pvdrm_slot* slot = &mapped->slot[i];
                slot->code = PVDRM_UNUSED;
		slot->ref = -EINVAL;
                mapped->ring[i] = (uint8_t)-1;
		ret = pvdrm_slot_ensure_ref(pvdrm, slot);
		if (ret) {
			BUG();
		}
	}
        wmb();

        PVDRM_INFO("PVDRM: Initialized pvdrm slots.\n");

	return 0;
}

int pvdrm_slots_release(struct pvdrm_device* pvdrm)
{
        struct pvdrm_slots* slots;

        slots = pvdrm->slots;
        if (slots) {
                kfree(slots);
        }
        return 0;
}

struct pvdrm_slot* pvdrm_slot_alloc(struct pvdrm_device* pvdrm)
{
	int i;
	struct pvdrm_slots* slots;
	struct pvdrm_slot* slot;
	struct pvdrm_mapped* mapped;
	unsigned long flags;

	slots = pvdrm->slots;
        mapped = slots->mapped;

	down(&slots->sema);
	spin_lock_irqsave(&slots->lock, flags);

	for (i = 0; i < PVDRM_SLOT_NR; ++i) {
		if (!is_used(&mapped->slot[i])) {
                        slot = &mapped->slot[i];
                        slot->code = PVDRM_HELD;
			break;
		}
	}

	BUG_ON(i == PVDRM_SLOT_NR);

	spin_unlock_irqrestore(&slots->lock, flags);

	pvdrm_fence_emit(&slot->__fence, 0);
	slot->ret = 0;

	return slot;
}

void pvdrm_slot_free(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	struct pvdrm_slots* slots;
	unsigned long flags;
	struct pvdrm_mapped* mapped;

	slots = pvdrm->slots;
        mapped = slots->mapped;

	spin_lock_irqsave(&slots->lock, flags);

	BUG_ON(!is_used(slot));
        slot->code = PVDRM_UNUSED;

	spin_unlock_irqrestore(&slots->lock, flags);
	up(&slots->sema);
}

int pvdrm_slot_wait(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot, uint32_t seq)
{
	int ret;
	ret = pvdrm_fence_wait(&slot->__fence, seq, false);
	if (ret) {
		return ret;
	}
	return slot->ret;
}

int pvdrm_slot_request(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	pvdrm_slot_request_async(pvdrm, slot);
	return pvdrm_slot_wait(pvdrm, slot, PVDRM_FENCE_DONE);
}

void pvdrm_slot_request_async(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot)
{
	struct pvdrm_slots* slots;
	uint32_t pos;
	struct pvdrm_mapped* mapped;

	slots = pvdrm->slots;
        mapped = slots->mapped;

	BUG_ON(!is_used(slot));

	/* Request slot, increment counter. */
	pos = ((uint32_t)atomic_add_return(1, &slots->put)) % PVDRM_SLOT_NR;
        mapped->ring[pos] = pvdrm_slot_id(mapped, slot);
	wmb();
	atomic_inc(&mapped->count);
}

int pvdrm_slot_call(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot, int code, void *data, size_t size)
{
	int ret;
	slot->code = (code);
	memcpy(pvdrm_slot_payload(slot), data, size);
	ret = pvdrm_slot_request(pvdrm, slot);
	memcpy(data, pvdrm_slot_payload(slot), size);
	return ret;
}

/* vim: set sw=8 ts=8 et tw=80 : */
