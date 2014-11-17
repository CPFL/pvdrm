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

#include "pvdrm_drm.h"
#include "pvdrm_host_table.h"

struct pvdrm_host_table* pvdrm_host_table_new(struct pvdrm_device* pvdrm)
{
	struct pvdrm_host_table* table = kzalloc(sizeof(*table), GFP_KERNEL);
	if (!table) {
		return NULL;
	}

	table->pvdrm = pvdrm;
	if (drm_ht_create(&table->hosts, 16)) {
		return NULL;
	}
	spin_lock_init(&table->lock);

	return table;
}

int pvdrm_host_table_insert(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj, uint32_t host)
{
	unsigned long flags;
	int ret = 0;
	struct pvdrm_host_table_entry* entry;

	BUG_ON(!table->pvdrm->hosts_cache);

	entry = kmem_cache_alloc(table->pvdrm->hosts_cache, GFP_KERNEL);
	if (!entry) {
		return -ENOMEM;
	}
	entry->host = host;
	entry->hash.key = (unsigned long)obj;

	spin_lock_irqsave(&table->lock, flags);
	ret = drm_ht_insert_item(&table->hosts, &entry->hash);
	spin_unlock_irqrestore(&table->lock, flags);

	if (ret) {
		kmem_cache_free(table->pvdrm->hosts_cache, entry);
		return ret;
	}

	return 0;
}

int pvdrm_host_table_remove(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj)
{
	int ret;
	unsigned long flags;
	struct drm_hash_item *hash;
	struct pvdrm_host_table_entry* entry;

	BUG_ON(!table->pvdrm->hosts_cache);

	spin_lock_irqsave(&table->lock, flags);

	ret = drm_ht_find_item(&table->hosts, (unsigned long)obj, &hash);
	if (ret) {
		spin_unlock_irqrestore(&table->lock, flags);
		return ret;
	}

	ret = drm_ht_remove_item(&table->hosts, hash);
	if (ret) {
		spin_unlock_irqrestore(&table->lock, flags);
		return ret;
	}

	entry = drm_hash_entry(hash, struct pvdrm_host_table_entry, hash);
	if (!entry) {
		spin_unlock_irqrestore(&table->lock, flags);
		return -ENOMEM;
	}

	kmem_cache_free(table->pvdrm->hosts_cache, entry);

	spin_unlock_irqrestore(&table->lock, flags);
	return 0;
}

int pvdrm_host_table_lookup(struct pvdrm_host_table* table, struct drm_pvdrm_gem_object* obj, uint32_t* host)
{
	unsigned long flags;
	struct drm_hash_item *hash;
	struct pvdrm_host_table_entry* entry;

	BUG_ON(!table->pvdrm->hosts_cache);

	spin_lock_irqsave(&table->lock, flags);
	if (drm_ht_find_item(&table->hosts, (unsigned long)obj, &hash)) {
		spin_unlock_irqrestore(&table->lock, flags);
		return -EINVAL;
	}

	entry = drm_hash_entry(hash, struct pvdrm_host_table_entry, hash);
	if (!entry) {
		spin_unlock_irqrestore(&table->lock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&table->lock, flags);

	*host = entry->host;
	return 0;
}
