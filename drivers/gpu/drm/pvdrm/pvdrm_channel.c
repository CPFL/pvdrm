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

#include "drmP.h"
#include "drm.h"

/* Include nouveau's abi16 header directly. */
#include "../nouveau/nouveau_abi16.h"

#include "pvdrm.h"
#include "pvdrm_cast.h"
#include "pvdrm_channel.h"
#include "pvdrm_gem.h"
#include "pvdrm_nouveau_abi16.h"
#include "pvdrm_slot.h"

int pvdrm_channel_alloc(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_alloc *req_out, struct pvdrm_channel** result)
{
	struct pvdrm_channel *channel;
	struct pvdrm_device* pvdrm;
	int ret = 0;

	pvdrm = drm_device_to_pvdrm(dev);

	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_ALLOC, req_out, sizeof(struct drm_nouveau_channel_alloc));
	if (ret) {
		return ret;
	}

	channel = kzalloc(sizeof(struct pvdrm_channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	channel->host = req_out->channel;
	kref_init(&channel->ref);

	if (idr_pre_get(&pvdrm->channels_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

again:
	spin_lock(&pvdrm->channels_lock);
	ret = idr_get_new_above(&pvdrm->channels_idr, channel, 1, (int *)&channel->channel);
	spin_unlock(&pvdrm->channels_lock);
	if (ret == -EAGAIN) {
		goto again;
	} else if (ret) {
		return ret;
	}

	/* Adjust gem information for guest environment. */
	printk(KERN_INFO "PVDRM: Allocating guest channel %d with host %d.\n", channel->channel, channel->host);
	req_out->channel = channel->channel;

	*result = channel;
	return 0;
}

static void pvdrm_channel_release(struct kref* ref)
{
	struct pvdrm_channel* channel = container_of(ref, struct pvdrm_channel, ref);
	printk(KERN_INFO "Deallocating channel %d with host %d.\n", channel->channel, channel->host);
        kfree(channel);
}

int pvdrm_channel_free(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_free *req_out)
{
	int ret = 0;
	struct pvdrm_channel* channel;
	struct pvdrm_device* pvdrm;

	pvdrm = drm_device_to_pvdrm(dev);

	spin_lock(&pvdrm->channels_lock);
	channel = idr_find(&pvdrm->channels_idr, req_out->channel);
	if (channel == NULL) {
		spin_unlock(&pvdrm->channels_lock);
		printk(KERN_INFO "PVDRM: Freeing invalid channel %d.\n", req_out->channel);
		return -EINVAL;
	}

	kref_get(&channel->ref);

	spin_unlock(&pvdrm->channels_lock);

	printk(KERN_INFO "PVDRM: Freeing guest channel %d with host %d.\n", channel->channel, channel->host);
	req_out->channel = channel->host;
	ret = pvdrm_nouveau_abi16_ioctl(dev, PVDRM_IOCTL_NOUVEAU_CHANNEL_FREE, req_out, sizeof(struct drm_nouveau_channel_free));
	kref_put(&channel->ref, pvdrm_channel_release);

	/* For free. */
	kref_put(&channel->ref, pvdrm_channel_release);
	return ret;
}

/* vim: set sw=8 ts=8 et tw=80 : */
