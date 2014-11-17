/*
 * Derived from drm_platform.c
 *
 * Copyright 2003 José Fonseca.
 * Copyright 2003 Leif Delgass.
 * Copyright (c) 2009, Code Aurora Forum.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * Newly created code is licensed under the following BSD license.
 * Copyright (C) 2014 Yusuke Suzuki <yusuke.suzuki@sslab.ics.keio.ac.jp>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/export.h>
#include <linux/module.h>

#include <xen/xen.h>
#include <xen/xenbus.h>
#include <xen/events.h>
#include <xen/page.h>
#include <xen/platform_pci.h>
#include <xen/grant_table.h>

#include <drmP.h>

/**
 * Register.
 *
 * \param xbdev - Xenbus device struture
 * \return zero on success or a negative number on failure.
 *
 * Attempt to gets inter module "drm" information. If we are first
 * then register the character device and inter module information.
 * Try and register, if we fail to register, backout previous work.
 */

int drm_get_xenbus_dev(struct xenbus_device *xbdev, struct drm_driver *driver)
{
	struct drm_device *dev;
	int ret = 0;

	DRM_DEBUG("\n");

	dev = drm_dev_alloc(driver, &xbdev->dev);
	if (!dev)
		return -ENOMEM;

	dev->platformdev = (struct platform_device*)xbdev;
	dev->dev = &xbdev->dev;
	dev_set_drvdata(&xbdev->dev, dev);

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto err_free;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		 driver->name, driver->major, driver->minor, driver->patchlevel,
		 driver->date, dev->primary->index);

	return 0;
err_free:
	drm_dev_unref(dev);
	return ret;
}
EXPORT_SYMBOL(drm_get_xenbus_dev);

/* FIXME: Fix this function for xenbus. */
static int drm_xenbus_set_busid(struct drm_device *dev, struct drm_master *master)
{
	int len, ret, id;
	struct xenbus_device* xbdev;

	xbdev = (struct xenbus_device*)dev->platformdev;

	master->unique_len = 13 + strlen(xbdev->nodename);
	master->unique_size = master->unique_len;
	master->unique = kmalloc(master->unique_len + 1, GFP_KERNEL);

	if (master->unique == NULL)
		return -ENOMEM;

	id = xbdev->otherend_id;

	/* if only a single instance of the xenbus device, id will be
	 * set to -1.. use 0 instead to avoid a funny looking bus-id:
	 */
	if (id == -1)
		id = 0;

	len = snprintf(master->unique, master->unique_len,
			"Xenbus:%s:%02d", xbdev->nodename, id);

	if (len > master->unique_len) {
		DRM_ERROR("Unique buffer overflowed\n");
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	return ret;
}

static struct drm_bus drm_xenbus_bus = {
	.set_busid = drm_xenbus_set_busid,
};

/**
 * Xenbus device initialization. Called direct from modules.
 *
 * \return zero on success or a negative number on failure.
 *
 * Initializes a drm_device structures,registering the
 * stubs
 *
 * Expands the \c DRIVER_PREINIT and \c DRIVER_POST_INIT macros before and
 * after the initialization for driver customization.
 */

int drm_xenbus_init(struct drm_driver *driver, struct xenbus_device *xbdev)
{
	DRM_DEBUG("\n");

	driver->bus = &drm_xenbus_bus;
	return drm_get_xenbus_dev(xbdev, driver);
}
EXPORT_SYMBOL(drm_xenbus_init);
