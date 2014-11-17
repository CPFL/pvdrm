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

#ifndef PVDRM_HOST_TABLE_H_
#define PVDRM_HOST_TABLE_H_

#include <linux/types.h>
#include <drmP.h>

struct drm_pvdrm_gem_object;

struct pvdrm_host_table_entry {
	struct drm_hash_item hash;
	uint32_t host;
};

struct pvdrm_host_table {
	struct pvdrm_device* pvdrm;
	struct drm_open_hash hosts;
	struct kmem_cache* cache;
	spinlock_t lock;
};

struct pvdrm_host_table* pvdrm_host_table_new(struct pvdrm_device* pvdrm);
int pvdrm_host_table_insert(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj, uint32_t host);
int pvdrm_host_table_remove(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj);
int pvdrm_host_table_lookup(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj, uint32_t* host);

#endif  /* PVDRM_HOST_TABLE_H_ */
