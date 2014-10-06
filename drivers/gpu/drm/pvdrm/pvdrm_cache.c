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
#include "pvdrm_drm.h"
#include "pvdrm_gem.h"
#include "pvdrm_log.h"

struct pvdrm_cache* pvdrm_cache_new(struct pvdrm_device* device)
{
	struct pvdrm_cache* cache;
	cache = kzalloc(sizeof(struct pvdrm_cache), GFP_KERNEL);
	INIT_LIST_HEAD(&cache->objects);
	return cache;
}

static void debug_dump_cache(struct pvdrm_cache* cache)
{
#if 0
	struct drm_pvdrm_gem_object* pos;
	struct drm_pvdrm_gem_object* temp;
	PVDRM_DEBUG("Do\n");
	list_for_each_entry_safe(pos, temp, &cache->objects, cache_head) {
		PVDRM_DEBUG("  Result %u size:(%lx)\n", pos->host, pos->base.size);
	}
	PVDRM_DEBUG("Done\n");
#endif
}

/* FIXME: Should use binary search. */
void pvdrm_cache_insert(struct pvdrm_cache* cache, struct drm_pvdrm_gem_object* obj)
{
	bool inserted = false;
	struct drm_pvdrm_gem_object* pos;
	struct drm_pvdrm_gem_object* temp;
	PVDRM_DEBUG("Inserting %u size:(%lx)\n", obj->host, obj->base.size);
	list_for_each_entry_safe(pos, temp, &cache->objects, cache_head) {
		if (obj->base.size > pos->base.size) {
			list_add_tail(&obj->cache_head, &pos->cache_head);
			inserted = true;
			break;
		}
	}
	if (!inserted) {
		list_add_tail(&obj->cache_head, &cache->objects);
	}
	debug_dump_cache(cache);
}

struct drm_pvdrm_gem_object* pvdrm_cache_fit(struct pvdrm_cache* cache, unsigned long size)
{
	struct drm_pvdrm_gem_object* best = NULL;
	struct drm_pvdrm_gem_object* pos;
	struct drm_pvdrm_gem_object* temp;
	list_for_each_entry_safe(pos, temp, &cache->objects, cache_head) {
		PVDRM_DEBUG("Dumping for %u host:(%u) size:(%lx) best:(%p)\n", size, pos->handle, pos->base.size, best);
		if (size > pos->base.size) {
			break;
		}
		best = pos;
	}
	if (best) {
		list_del(&best->cache_head);
	}
	debug_dump_cache(cache);
	return best;
}
