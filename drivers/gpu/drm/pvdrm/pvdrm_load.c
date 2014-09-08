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

#include "pvdrm_cast.h"
#include "pvdrm_drm.h"
#include "pvdrm_load.h"
#include "pvdrm_log.h"
#include "pvdrm_slot.h"

static int pvdrm_init(struct pvdrm_device* pvdrm, struct drm_device *dev, unsigned long flags)
{
	// pvdrm_slot_init(pvdrm);
	return 0;
}

int pvdrm_load(struct drm_device *dev, unsigned long flags)
{
	struct pvdrm_device *pvdrm = NULL;
	int ret = 0;

	pvdrm = kzalloc(sizeof(struct pvdrm_device), GFP_KERNEL);
	if (!pvdrm) {
		return -ENOMEM;
        }

        /* Configure it. */
	dev->dev_private = (void*)pvdrm;
	pvdrm->dev = dev;

	ret = pvdrm_init(pvdrm, dev, flags);
	if (ret)
		goto out;

        PVDRM_INFO("PVDRM: loaded.\n");
out:
	if (ret)
		pvdrm_unload(dev);

	return ret;
}

int pvdrm_unload(struct drm_device *dev)
{
	struct pvdrm_device *pvdrm = NULL;
	int ret = 0;

        pvdrm = drm_device_to_pvdrm(dev);
        if (pvdrm) {
                pvdrm_slots_release(pvdrm);
		kfree(pvdrm);
                dev->dev_private = NULL;
        }

	return ret;
}

int pvdrm_open(struct drm_device *dev, struct drm_file *file)
{
	return 0;
}

void pvdrm_preclose(struct drm_device *dev, struct drm_file *file)
{
}

void pvdrm_postclose(struct drm_device *dev, struct drm_file *file)
{
}

/* vim: set sw=8 ts=8 et tw=80 : */
