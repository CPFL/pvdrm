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

#include <drmP.h>

#include <common/pvdrm_idr.h>
#include <common/pvdrm_log.h>
#include <common/pvdrm_slot.h>

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_channel.h"
#include "pvdrm_gem.h"
#include "pvdrm_nouveau_abi16.h"

static void pvdrm_channel_release(struct kref* ref)
{
	unsigned long flags;
	struct pvdrm_channel* chan = container_of(ref, struct pvdrm_channel, ref);
	struct pvdrm_device* pvdrm = chan->pvdrm;
	PVDRM_DEBUG("Deallocating channel %d with host %d.\n", chan->channel, chan->host);

	spin_lock_irqsave(&pvdrm->channels_lock, flags);
	idr_remove(&pvdrm->channels_idr, chan->channel);
	spin_unlock_irqrestore(&pvdrm->channels_lock, flags);

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

int pvdrm_channel_alloc(struct drm_device* dev, struct drm_file* file, struct drm_nouveau_channel_alloc* req_out, struct pvdrm_channel** result)
{
	struct pvdrm_channel *chan;
	struct pvdrm_device* pvdrm;
	int ret = 0;

	pvdrm = drm_device_to_pvdrm(dev);

	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, req_out, sizeof(struct drm_nouveau_channel_alloc));
	if (ret) {
		return ret;
	}

	chan = kzalloc(sizeof(struct pvdrm_channel), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->host = req_out->channel;
	chan->pvdrm = pvdrm;
	kref_init(&chan->ref);

	ret = pvdrm_idr_alloc(&pvdrm->channels_idr, &pvdrm->channels_lock, chan, 1);
	if (ret < 0) {
		pvdrm_channel_unreference(chan);
		return ret;
	}
	chan->channel = ret;

	/* Adjust gem information for guest environment. */
	PVDRM_DEBUG("Allocating guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->channel;

	*result = chan;
	return 0;
}

int pvdrm_channel_free(struct drm_device* dev, struct drm_file* file, struct drm_nouveau_channel_free* req_out)
{
	int ret = 0;
	struct pvdrm_channel* chan;
	struct pvdrm_device* pvdrm;

	pvdrm = drm_device_to_pvdrm(dev);
        chan = pvdrm_channel_lookup(dev, req_out->channel);
        if (!chan) {
		PVDRM_ERROR("Freeing channel %u.\n", req_out->channel);
		return -EINVAL;
        }

	PVDRM_DEBUG("Freeing guest channel %d with host %d.\n", chan->channel, chan->host);
	req_out->channel = chan->host;
	ret = pvdrm_nouveau_abi16_ioctl(file, PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE, req_out, sizeof(struct drm_nouveau_channel_free));
	pvdrm_channel_unreference(chan);  /* Release. */
	pvdrm_channel_unreference(chan);  /* For freeing. */
	return ret;
}

struct pvdrm_channel* pvdrm_channel_lookup(struct drm_device *dev, uint32_t id)
{
	unsigned long flags;
        struct pvdrm_device* pvdrm = NULL;
        struct pvdrm_channel* chan = NULL;

        pvdrm = drm_device_to_pvdrm(dev);

	spin_lock_irqsave(&pvdrm->channels_lock, flags);
	chan = idr_find(&pvdrm->channels_idr, id);
	if (chan == NULL) {
		spin_unlock_irqrestore(&pvdrm->channels_lock, flags);
		PVDRM_ERROR("Look up invalid channel %u.\n", id);
		return NULL;
	}
	pvdrm_channel_reference(chan);
	spin_unlock_irqrestore(&pvdrm->channels_lock, flags);
        return chan;
}
