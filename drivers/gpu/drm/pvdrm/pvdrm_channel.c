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

#include "drmP.h"
#include "drm.h"

/* Include nouveau's abi16 header directly. */
#include "../nouveau/nouveau_abi16.h"

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_channel.h"
#include "pvdrm_gem.h"
#include "pvdrm_log.h"
#include "pvdrm_nouveau_abi16.h"
#include "pvdrm_slot.h"


static void pvdrm_channel_release(struct kref* ref)
{
	struct pvdrm_channel* chan = container_of(ref, struct pvdrm_channel, ref);
	PVDRM_DEBUG("Deallocating channel %d with host %d.\n", chan->channel, chan->host);
        kfree(chan);
}

void pvdrm_channel_reference(struct pvdrm_channel* chan)
{
	kref_get(&chan->ref);
}

void pvdrm_channel_unreference(struct pvdrm_channel* chan)
{
	kref_put(&chan->ref, pvdrm_channel_release);  /* Release. */
}

int pvdrm_channel_alloc(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_alloc *req_out, struct pvdrm_channel** result)
{
	struct pvdrm_channel *chan;
	struct pvdrm_device* pvdrm;
	int ret = 0;

	pvdrm = drm_device_to_pvdrm(dev);

	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, req_out, sizeof(struct drm_nouveau_channel_alloc));
	if (ret) {
		return ret;
	}

	chan = kzalloc(sizeof(struct pvdrm_channel), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->host = req_out->channel;
	kref_init(&chan->ref);

	if (idr_pre_get(&pvdrm->channels_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

again:
	spin_lock(&pvdrm->channels_lock);
	ret = idr_get_new_above(&pvdrm->channels_idr, chan, 1, (int *)&chan->channel);
	spin_unlock(&pvdrm->channels_lock);
	if (ret == -EAGAIN) {
		goto again;
	} else if (ret) {
		return ret;
	}

	/* Adjust gem information for guest environment. */
	PVDRM_DEBUG("PVDRM: Allocating guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->channel;

	*result = chan;
	return 0;
}

int pvdrm_channel_free(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_free *req_out)
{
	int ret = 0;
	struct pvdrm_channel* chan;
	struct pvdrm_device* pvdrm;

	pvdrm = drm_device_to_pvdrm(dev);
        chan = pvdrm_channel_lookup(dev, req_out->channel);
        if (!chan) {
		PVDRM_ERROR("PVDRM: Freeing channel %u.\n", req_out->channel);
		return -EINVAL;
        }

	PVDRM_DEBUG("PVDRM: Freeing guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->host;
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE, req_out, sizeof(struct drm_nouveau_channel_free));
	pvdrm_channel_unreference(chan);  /* Release. */
	pvdrm_channel_unreference(chan);  /* For freeing. */
	return ret;
}

struct pvdrm_channel* pvdrm_channel_lookup(struct drm_device *dev, uint32_t id)
{
        struct pvdrm_device* pvdrm = NULL;
        struct pvdrm_channel* chan = NULL;

        pvdrm = drm_device_to_pvdrm(dev);

	spin_lock(&pvdrm->channels_lock);
	chan = idr_find(&pvdrm->channels_idr, id);
	if (chan == NULL) {
		spin_unlock(&pvdrm->channels_lock);
		PVDRM_ERROR("PVDRM: Look up invalid channel %u.\n", id);
		return NULL;
	}
	pvdrm_channel_reference(chan);
	spin_unlock(&pvdrm->channels_lock);
        return chan;
}

/* vim: set sw=8 ts=8 et tw=80 : */
