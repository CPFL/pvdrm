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

#include "pvdrm_cache.h"
#include "pvdrm_cast.h"
#include "pvdrm_drm.h"
#include "pvdrm_gem.h"
#include "pvdrm_host_table.h"
#include "pvdrm_log.h"
#include "pvdrm_nouveau_abi16.h"

struct pvdrm_cache_entry {
	struct list_head head;
	struct drm_pvdrm_gem_object* obj;
};

struct pvdrm_cache* pvdrm_cache_new(struct pvdrm_device* pvdrm)
{
	struct pvdrm_cache* cache;
	cache = kzalloc(sizeof(struct pvdrm_cache), GFP_KERNEL);
	cache->pvdrm = pvdrm;
	cache->mem = kmem_cache_create("pvdrm_cache", sizeof(struct pvdrm_cache_entry), 0, 0, NULL);
	INIT_LIST_HEAD(&cache->entries);
	return cache;
}

static void debug_dump_cache(struct pvdrm_cache* cache)
{
#if 1
	struct pvdrm_cache_entry* pos;
	struct pvdrm_cache_entry* temp;
	PVDRM_DEBUG("Do\n");
	list_for_each_entry_safe(pos, temp, &cache->entries, head) {
		PVDRM_DEBUG("  Result obj:(%p) size:(%lx)\n", pos->obj, pos->obj->base.size);
	}
	PVDRM_DEBUG("Done\n");
#endif
}

/* FIXME: Should use binary search. */
void pvdrm_cache_insert(struct pvdrm_cache* cache, struct drm_file* file, struct drm_pvdrm_gem_object* obj)
{
	int ret;
	bool inserted = false;
	struct pvdrm_cache_entry* new;
	struct pvdrm_cache_entry* pos;
	struct pvdrm_cache_entry* temp;
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);

	new = kmem_cache_alloc(cache->mem, GFP_KERNEL);
	new->obj = obj;
	drm_gem_object_reference(&obj->base);   /* Ref cache refernece. */

	if (!obj->global) {
		/* Generate global handle. */
		struct drm_pvdrm_gem_global_handle req;
		uint32_t host = 0;

		ret = pvdrm_host_table_lookup(fpriv->hosts, obj, &host);
		if (ret) {
			BUG();
			return;
		}

		req = (struct drm_pvdrm_gem_global_handle) {
			.handle = host,
			.global = 0,
		};

		ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_GEM_TO_GLOBAL_HANDLE, &req, sizeof(struct drm_pvdrm_gem_global_handle));
		BUG_ON(req.global == 0);
		obj->global = req.global;
	}

	PVDRM_DEBUG("Inserting obj:(%p) size:(%lx)\n", obj, obj->base.size);
	list_for_each_entry_safe(pos, temp, &cache->entries, head) {
		if (obj->base.size > pos->obj->base.size) {
			list_add_tail(&new->head, &pos->head);
			inserted = true;
			break;
		}
	}
	if (!inserted) {
		list_add_tail(&new->head, &cache->entries);
	}
	debug_dump_cache(cache);
}

struct drm_pvdrm_gem_object* pvdrm_cache_fit(struct pvdrm_cache* cache, unsigned long size)
{
	struct pvdrm_cache_entry* best = NULL;
	struct pvdrm_cache_entry* pos;
	struct pvdrm_cache_entry* temp;
	struct drm_pvdrm_gem_object* obj = NULL;
	list_for_each_entry_safe(pos, temp, &cache->entries, head) {
		PVDRM_DEBUG("Dumping for %lu obj:(%p) size:(%lx) best:(%p)\n", size, pos->obj, pos->obj->base.size, best);
		if (size > pos->obj->base.size) {
			break;
		}
		best = pos;
	}
	if (best) {
		list_del(&best->head);
		kmem_cache_free(cache->mem, best);
		obj = best->obj;
	}
	debug_dump_cache(cache);
	return obj;
}
