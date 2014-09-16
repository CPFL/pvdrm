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

#include <linux/fs.h>
#include <asm/uaccess.h>

#include "pvdrm_back_drv.h"

struct pvdrm_back_file* pvdrm_back_file_new(struct pvdrm_back_device* info)
{
	struct file* filp = NULL;
	struct pvdrm_back_file* pvfile = NULL;
	int ret = 0;
	mm_segment_t fs;

	fs = get_fs();
	set_fs(get_ds());
	/* FIXME: Currently we use this path directly. We need to implement
	 * discovery functionality.*/
	filp = filp_open("/dev/dri/card0", O_RDWR, 0);
	set_fs(fs);
	PVDRM_INFO("Opened drm device.\n");

	pvfile = kzalloc(sizeof(*pvfile), GFP_KERNEL);
	if (!pvfile) {
		return NULL;
	}
	pvfile->info = info;
	pvfile->filp = filp;
	pvfile->handle = 0;

	if (idr_pre_get(&info->file_idr, GFP_KERNEL) == 0) {
		pvdrm_back_file_destroy(pvfile);
		return NULL;
	}
again:
	spin_lock(&info->file_lock);
	ret = idr_get_new_above(&info->file_idr, pvfile, 1, &pvfile->handle);
	spin_unlock(&info->file_lock);
	if (ret == -EAGAIN) {
		goto again;
	} else if (ret) {
		pvdrm_back_file_destroy(pvfile);
		return NULL;
	}

	return pvfile;
}

void pvdrm_back_file_destroy(struct pvdrm_back_file* filp)
{
	if (filp) {
		mm_segment_t fs;
		fs = get_fs();
		set_fs(get_ds());
		filp_close(filp->filp, NULL);
		set_fs(fs);

		if (filp->handle > 0) {
			spin_lock(&filp->info->file_lock);
			idr_remove(&filp->info->file_idr, filp->handle);
			spin_unlock(&filp->info->file_lock);
		}

		kfree(filp);
	}
}

/* vim: set sw=8 ts=8 et tw=80 : */
