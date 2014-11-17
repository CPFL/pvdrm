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
#ifndef PVDRM_CAST_H_
#define PVDRM_CAST_H_

#include <xen/xenbus.h>

#include <drmP.h>

#include "pvdrm_drm.h"

static inline struct pvdrm_device* drm_device_to_pvdrm(struct drm_device* dev)
{
        return dev->dev_private;
}

static inline struct xenbus_device* drm_device_to_xbdev(struct drm_device* dev)
{
        return (struct xenbus_device*)dev->platformdev;
}

static inline struct drm_device* pvdrm_to_drm_device(struct pvdrm_device* pvdrm)
{
        return pvdrm->dev;
}

static inline struct drm_device* xbdev_to_drm_device(struct xenbus_device* xbdev)
{
        return dev_get_drvdata(&xbdev->dev);
}

static inline struct xenbus_device* pvdrm_to_xbdev(struct pvdrm_device* pvdrm)
{
        return drm_device_to_xbdev(pvdrm_to_drm_device(pvdrm));
}

static inline struct pvdrm_device* xbdev_to_pvdrm(struct xenbus_device* xbdev)
{
        return drm_device_to_pvdrm(xbdev_to_drm_device(xbdev));
}

/* File and fpriv. */

static inline struct pvdrm_fpriv* drm_file_to_fpriv(struct drm_file* file)
{
	return file->driver_priv;
}

static inline struct drm_file* fpriv_to_drm_file(struct pvdrm_fpriv* fpriv)
{
	return fpriv->file;
}

static inline struct drm_device* drm_file_to_drm_device(struct drm_file* file)
{
	return file->minor->dev;
}

static inline struct pvdrm_device* drm_file_to_pvdrm(struct drm_file* file)
{
	return drm_device_to_pvdrm(drm_file_to_drm_device(file));
}


static inline struct drm_device* fpriv_to_drm_device(struct pvdrm_fpriv* fpriv)
{
	return drm_file_to_drm_device(fpriv_to_drm_file(fpriv));
}

static inline struct pvdrm_device* fpriv_to_pvdrm(struct pvdrm_fpriv* fpriv)
{
	return drm_file_to_pvdrm(fpriv_to_drm_file(fpriv));
}

#endif  /* PVDRM_CAST_H_ */
