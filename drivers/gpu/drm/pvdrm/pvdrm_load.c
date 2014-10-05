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
#include "pvdrm_nouveau_abi16.h"

/* Called after backend is connected. */
/* FIXME: There's no memory free. */
int pvdrm_connected(struct pvdrm_device* pvdrm, struct drm_device *dev)
{
	pvdrm_slots_init(pvdrm);

	/* Open global fpriv. */
	if (!device_get_devnode(&dev->primary->kdev, NULL, &pvdrm->devnode)) {
		return -ENOMEM;
	}
	{
		mm_segment_t fs = get_fs();
		set_fs(get_ds());
		pvdrm->global_filp = filp_open(pvdrm->devnode, O_RDWR, 0);
		set_fs(fs);
		/* FIXME: More stable way. */
		pvdrm->global_fpriv.file = pvdrm->global_filp->private_data;
	}
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

	if (drm_ht_create(&pvdrm->mh2obj, 16)) {
		goto out;
	}
	spin_lock_init(&pvdrm->mh2obj_lock);

	idr_init(&pvdrm->channels_idr);
	spin_lock_init(&pvdrm->channels_lock);
	pvdrm->wq = alloc_workqueue("pvdrm", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_NON_REENTRANT, 0);
	if (!pvdrm->wq) {
		BUG();
	}

	PVDRM_INFO("loaded.\n");
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

static int pvdrm_nouveau_global_call(struct drm_device* dev, int code, void *data, size_t size)
{
	struct pvdrm_device* pvdrm;
	int ret;
	pvdrm = drm_device_to_pvdrm(dev);
	{
		struct pvdrm_slot* slot = pvdrm_slot_alloc(pvdrm, PVDRM_FILE_GLOBAL_HANDLE);
		ret = pvdrm_slot_call(pvdrm, slot, code, data, size);
		pvdrm_slot_free(pvdrm, slot);
	}
	return ret;
}

int pvdrm_open(struct drm_device* dev, struct drm_file* file)
{
	struct drm_pvdrm_file_open req = { 0 };
	struct pvdrm_fpriv* fpriv = NULL;
	int ret = 0;
	ret = pvdrm_nouveau_global_call(dev, PVDRM_FILE_OPEN, &req, sizeof(struct drm_pvdrm_file_open));
	if (ret) {
		goto fail_hypercall;
	}

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv) {
		ret = -ENOMEM;
		goto fail_hypercall;
	}

	fpriv->host = req.file;
	fpriv->file = file;

	file->driver_priv = fpriv;

	return ret;

fail_hypercall:
	return ret;
}

void pvdrm_preclose(struct drm_device *dev, struct drm_file *file)
{
}

void pvdrm_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct pvdrm_fpriv* fpriv = drm_file_to_fpriv(file);
	struct drm_pvdrm_file_close req = {
		.file = fpriv->host,
	};
	pvdrm_nouveau_global_call(dev, PVDRM_FILE_CLOSE, &req, sizeof(struct drm_pvdrm_file_close));
	kfree(fpriv);
}
