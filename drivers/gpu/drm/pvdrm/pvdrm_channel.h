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
#ifndef PVDRM_CHANNEL_H_
#define PVDRM_CHANNEL_H_

#include "drmP.h"
#include "nouveau_drm.h"

#include "pvdrm_gem.h"

struct drm_nouveau_channel_alloc;

struct pvdrm_channel {
	struct kref ref;
	uint32_t channel;
	uint32_t host;
};

int pvdrm_channel_alloc(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_alloc *req_out, struct pvdrm_channel** result);
int pvdrm_channel_free(struct drm_device *dev, struct drm_file *file, struct drm_nouveau_channel_free *req_out);
struct pvdrm_channel* pvdrm_channel_lookup(struct drm_device *dev, uint32_t id);
void pvdrm_channel_reference(struct pvdrm_channel* chan);
void pvdrm_channel_unreference(struct pvdrm_channel* chan);

#endif  /* PVDRM_CHANNEL_H_ */
/* vim: set sw=8 ts=8 et tw=80 : */
