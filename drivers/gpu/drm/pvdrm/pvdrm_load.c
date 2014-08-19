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

#include "pvdrm_drm.h"
#include "pvdrm_load.h"
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

	if (!(pvdrm = kzalloc(sizeof(pvdrm), GFP_KERNEL)))
		return -ENOMEM;
	dev->dev_private = (void *)pvdrm;
	pvdrm->dev = dev;

	ret = pvdrm_init(pvdrm, dev, flags);
	if (ret)
		goto out;

        printk(KERN_INFO "PVDRM: loaded.\n");
out:
	if (ret)
		pvdrm_unload(dev);

	return ret;
}

int pvdrm_unload(struct drm_device *dev)
{
	int ret = 0;
	if (dev->dev_private)
		kfree(dev->dev_private);
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
