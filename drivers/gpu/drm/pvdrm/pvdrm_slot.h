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
#ifndef PVDRM_SLOT_H_
#define PVDRM_SLOT_H_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <drm/nouveau_drm.h>

#include <xen/grant_table.h>
#define PVDRM_SLOT_NR 16

#include "drmP.h"
#include "../nouveau/nouveau_abi16.h"

#include "pvdrm_fence.h"

enum {
	/* used for unused flag. */
	PVDRM_UNUSED = -1,
	PVDRM_HELD = 0,

	/* ioctls. */
	PVDRM_IOCTL_NOUVEAU_GETPARAM = 1,
	/* PVDRM_IOCTL_NOUVEAU_SETPARAM, */
	PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC = 2,
	PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE = 3,
	PVDRM_IOCTL_NOUVEAU_GROBJ_ALLOC = 4,
	PVDRM_IOCTL_NOUVEAU_NOTIFIEROBJ_ALLOC = 5,
	PVDRM_IOCTL_NOUVEAU_GPUOBJ_FREE = 6,
	PVDRM_IOCTL_NOUVEAU_GEM_NEW = 7,
	PVDRM_IOCTL_NOUVEAU_GEM_PUSHBUF = 8,
	PVDRM_IOCTL_NOUVEAU_GEM_CPU_PREP = 9,
	PVDRM_IOCTL_NOUVEAU_GEM_CPU_FINI = 10,
	PVDRM_IOCTL_NOUVEAU_GEM_INFO = 11,

	/* gem operations. */
	PVDRM_GEM_NOUVEAU_GEM_FREE = 12,
	PVDRM_GEM_NOUVEAU_GEM_OPEN = 13,
	PVDRM_GEM_NOUVEAU_GEM_CLOSE = 14,
	PVDRM_GEM_NOUVEAU_GEM_MMAP = 15,
	PVDRM_GEM_NOUVEAU_GEM_FAULT = 16,
};

struct pvdrm_mapping {
	int32_t i;
	int32_t ref;
};

struct drm_pvdrm_file_open {
};

struct drm_pvdrm_file_release {
};

struct drm_pvdrm_gem_free {
	uint32_t handle;
};

struct drm_pvdrm_gem_open {
	uint32_t handle;
};

struct drm_pvdrm_gem_mmap {
	uint64_t map_handle;
	unsigned int flags;
	uint64_t vm_start;
	uint64_t vm_end;
	uint32_t handle;
};

struct drm_pvdrm_gem_fault {
#define PVDRM_GEM_FAULT_MAX_PAGES_PER_CALL 512
	uint64_t flags;
	uint64_t pgoff;
	uint64_t offset;
	uint64_t map_handle;
	uint64_t backing;
	uint32_t nr_pages;
	uint32_t domain;
	/* out */ uint64_t mapped_count;
};

struct pvdrm_slot {
	/* Headers */
	struct pvdrm_fence __fence;

	int32_t code;
	int32_t ret;

	int32_t ref;  /* Grant page reference to transfer additional data. */
	void* addr;   /* And granted page addr. */

	union {
		uint64_t __payload;  /* To calculate palyload address. */

		/* ioctl */
		struct drm_nouveau_getparam getparam;
		/* struct drm_nouveau_setparam setparam; */
		struct drm_nouveau_channel_alloc channel_alloc;
		struct drm_nouveau_channel_free channel_free;
		struct drm_nouveau_grobj_alloc grobj_alloc;
		struct drm_nouveau_notifierobj_alloc notifierobj_alloc;
		struct drm_nouveau_gpuobj_free gpuobj_free;
		struct drm_nouveau_gem_new gem_new;
		struct drm_nouveau_gem_pushbuf gem_pushbuf;
		struct drm_nouveau_gem_cpu_prep gem_cpu_prep;
		struct drm_nouveau_gem_cpu_fini gem_cpu_fini;
		struct drm_nouveau_gem_info gem_info;

		/* gem operations. */
		struct drm_pvdrm_gem_free gem_free;
		struct drm_gem_close gem_close;
		struct drm_pvdrm_gem_open gem_open;
		struct drm_pvdrm_gem_mmap gem_mmap;
		struct drm_pvdrm_gem_fault gem_fault;
	};

	// For pushbuf operations.
	union {
		struct {
			uint16_t nr_buffers;
			uint16_t nr_relocs;
			uint16_t nr_push;
			uint16_t next;
		} transfer;
	} u;
};

static inline void* pvdrm_slot_payload(struct pvdrm_slot* slot) {
	return ((uint8_t*)slot) + offsetof(struct pvdrm_slot, __payload);
}

struct pvdrm_mapped {
	struct pvdrm_slot slot[PVDRM_SLOT_NR];  /* Should be here. */
	uint8_t ring[PVDRM_SLOT_NR];
	atomic_t count;
};

struct pvdrm_slots {
	struct pvdrm_mapped* mapped;
	atomic_t put;
	grant_ref_t ref;
	struct semaphore sema;
	spinlock_t lock;
};

struct pvdrm_device;

int pvdrm_slots_init(struct pvdrm_device* pvdrm);
int pvdrm_slots_release(struct pvdrm_device* pvdrm);
static inline uint8_t pvdrm_slot_id(struct pvdrm_mapped* mapped, struct pvdrm_slot* slot)
{
	return (((uintptr_t)slot) - ((uintptr_t)mapped)) / sizeof(struct pvdrm_slot);
}

struct pvdrm_slot* pvdrm_slot_alloc(struct pvdrm_device* pvdrm);
void pvdrm_slot_free(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot);
int pvdrm_slot_request(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot);
void pvdrm_slot_request_async(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot);
int pvdrm_slot_wait(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot, uint32_t seq);

/* More useful interface. */
int pvdrm_slot_call(struct pvdrm_device* pvdrm, struct pvdrm_slot* slot, int code, void *data, size_t size);

#endif  /* PVDRM_SLOT_H_ */
/* vim: set sw=8 ts=8 et tw=80 : */
