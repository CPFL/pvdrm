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
 * Copyright (C) 2014 Yusuke Suzuki <utatane.tea@gmail.com>
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

#include "drmP.h"

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
	int ret;

	DRM_DEBUG("\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->xbdev = xbdev;
	dev->dev = &xbdev->dev;
	dev_set_drvdata(&xbdev->dev, dev);

	mutex_lock(&drm_global_mutex);

	ret = drm_fill_in_dev(dev, NULL, driver);

	if (ret) {
		printk(KERN_ERR "DRM: Fill_in_dev failed.\n");
		goto err_g1;
	}

	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		ret = drm_get_minor(dev, &dev->control, DRM_MINOR_CONTROL);
		if (ret)
			goto err_g1;
	}

	ret = drm_get_minor(dev, &dev->primary, DRM_MINOR_LEGACY);
	if (ret)
		goto err_g2;

	if (dev->driver->load) {
		ret = dev->driver->load(dev, 0);
		if (ret)
			goto err_g3;
	}

	/* setup the grouping for the legacy output */
	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		ret = drm_mode_group_init_legacy_group(dev,
				&dev->primary->mode_group);
		if (ret)
			goto err_g3;
	}

	list_add_tail(&dev->driver_item, &driver->device_list);

	mutex_unlock(&drm_global_mutex);

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		 driver->name, driver->major, driver->minor, driver->patchlevel,
		 driver->date, dev->primary->index);

	return 0;

err_g3:
	drm_put_minor(&dev->primary);
err_g2:
	if (drm_core_check_feature(dev, DRIVER_MODESET))
		drm_put_minor(&dev->control);
err_g1:
	kfree(dev);
	mutex_unlock(&drm_global_mutex);
	return ret;
}
EXPORT_SYMBOL(drm_get_xenbus_dev);

static int drm_xenbus_get_irq(struct drm_device *dev)
{
	/* FIXME: not implemented yet */
	return 0;
}

static const char *drm_xenbus_get_name(struct drm_device *dev)
{
	return dev->xbdev->otherend;
}

static int drm_xenbus_set_busid(struct drm_device *dev, struct drm_master *master)
{
	int len, ret, id;

	master->unique_len = 13 + strlen(dev->xbdev->otherend);
	master->unique_size = master->unique_len;
	master->unique = kmalloc(master->unique_len + 1, GFP_KERNEL);

	if (master->unique == NULL)
		return -ENOMEM;

	id = dev->xbdev->otherend_id;

	/* if only a single instance of the xenbus device, id will be
	 * set to -1.. use 0 instead to avoid a funny looking bus-id:
	 */
	if (id == -1)
		id = 0;

	len = snprintf(master->unique, master->unique_len,
			"Xenbus:%s:%02d", dev->xbdev->otherend, id);

	if (len > master->unique_len) {
		DRM_ERROR("Unique buffer overflowed\n");
		ret = -EINVAL;
		goto err;
	}

	dev->devname =
		kmalloc(strlen(dev->xbdev->otherend) +
			master->unique_len + 2, GFP_KERNEL);

	if (dev->devname == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	sprintf(dev->devname, "%s@%s", dev->xbdev->otherend,
		master->unique);
	return 0;
err:
	return ret;
}

static struct drm_bus drm_xenbus_bus = {
	.bus_type = DRIVER_BUS_XENBUS,
	.get_irq = drm_xenbus_get_irq,
	.get_name = drm_xenbus_get_name,
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

	driver->kdriver.xbdev = xbdev;
	driver->bus = &drm_xenbus_bus;
	INIT_LIST_HEAD(&driver->device_list);
	return drm_get_xenbus_dev(xbdev, driver);
}
EXPORT_SYMBOL(drm_xenbus_init);

void drm_xenbus_exit(struct drm_driver *driver, struct xenbus_device *xbdev)
{
	struct drm_device *dev, *tmp;
	DRM_DEBUG("\n");

	list_for_each_entry_safe(dev, tmp, &driver->device_list, driver_item)
		drm_put_dev(dev);
	DRM_INFO("Module unloaded\n");
}
EXPORT_SYMBOL(drm_xenbus_exit);
