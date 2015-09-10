/*
 * Copyright (C) 2011 matt mooney <mfm@muteddisk.com>
 *               2005-2007 Takahiro Hirofuchi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/usb/ch9.h>

#include <errno.h>
#include <unistd.h>

#include <libudev.h>

#include <dirent.h>

#include "usbip_common.h"
#include "usbip_device_driver.h"
#include "list.h"
#include "sysfs_utils.h"

#undef  PROGNAME
#define PROGNAME "libusbip"

struct usbip_device_driver *device_driver;
struct udev *udev_context;

#define copy_descr_attr16(dev, descr, attr)			\
	do {							\
		(dev)->attr = le16toh((descr)->attr);		\
	} while (0)

#define copy_descr_attr(dev, descr, attr)			\
	do {							\
		(dev)->attr = (descr)->attr;		\
	} while (0)

/* TODO - move to file common to host and device */
static int32_t read_attr_usbip_status(struct usbip_usb_device *udev)
{
	char status_attr_path[SYSFS_PATH_MAX];
	int fd;
	int length;
	char status;
	int value = 0;

	snprintf(status_attr_path, SYSFS_PATH_MAX, "%s/usbip_status",
		 udev->path);

	fd = open(status_attr_path, O_RDONLY);
	if (fd < 0) {
		err("error opening attribute %s", status_attr_path);
		return -1;
	}

	length = read(fd, &status, 1);
	if (length < 0) {
		err("error reading attribute %s", status_attr_path);
		close(fd);
		return -1;
	}

	value = atoi(&status);

	return value;
}

static
int read_usb_vudc_device(struct udev_device *sdev, struct usbip_usb_device *dev)
{
	const char *path, *name;
	char filepath[SYSFS_PATH_MAX];
	struct usb_device_descriptor descr;
	int ret = 0;
	FILE *fd = NULL;

	path = udev_device_get_syspath(sdev);
	snprintf(filepath, SYSFS_PATH_MAX, "%s/%s", path, VUDC_DEVICE_DESCR_FILE);
	fd = fopen(filepath, "r");
	if (!fd)
		return -1;
	ret = fread((char *) &descr, sizeof(descr), 1, fd);
	if (ret < 0)
		return -1;
	fclose(fd);

	copy_descr_attr(dev, &descr, bDeviceClass);
	copy_descr_attr(dev, &descr, bDeviceSubClass);
	copy_descr_attr(dev, &descr, bDeviceProtocol);
	copy_descr_attr(dev, &descr, bNumConfigurations);
	copy_descr_attr16(dev, &descr, idVendor);
	copy_descr_attr16(dev, &descr, idProduct);
	copy_descr_attr16(dev, &descr, bcdDevice);

	strncpy(dev->path, path, SYSFS_PATH_MAX);

	/* FIXME - depends on speed of vudc, constify / export */
	dev->speed = USB_SPEED_HIGH;

	/* Only used for user output, little sense to output them in general */
	dev->bNumInterfaces = 0;
	dev->bConfigurationValue = 0;
	dev->busnum = 0;

	name = udev_device_get_sysname(sdev);
	strncpy(dev->busid, name, SYSFS_BUS_ID_SIZE);
	return 0;
}

static
struct usbip_exported_device *usbip_exported_device_new(const char *path)
{
	struct usbip_exported_device *edev = NULL;
	int ret;
	ret = 0;

	edev = calloc(1, sizeof(struct usbip_exported_device));

	edev->sudev = udev_device_new_from_syspath(udev_context, path);
	if (!edev->sudev) {
		err("udev_device_new_from_syspath: %s", path);
		goto err;
	}

	ret = read_usb_vudc_device(edev->sudev, &edev->udev);
	if (ret)
		goto err;

	edev->status = read_attr_usbip_status(&edev->udev);
	if (edev->status < 0)
		goto err;

	/* FIXME - decide if we should fetch interfaces */

	return edev;
err:
	if (edev->sudev)
		udev_device_unref(edev->sudev);
	if (edev)
		free(edev);

	return NULL;
}

static int refresh_exported_devices(void)
{
	struct usbip_exported_device *edev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev;
	const char *path;
	const char *driver;

	enumerate = udev_enumerate_new(udev_context);
	udev_enumerate_add_match_subsystem(enumerate, "platform");
	udev_enumerate_scan_devices(enumerate);

	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices) {
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev_context, path);
		if (dev == NULL)
			continue;

		/* Check whether device uses usbip-vudc driver. */
		driver = udev_device_get_driver(dev);
		if (driver != NULL && !strcmp(driver, USBIP_DEVICE_DRV_NAME)) {
			edev = usbip_exported_device_new(path);
			if (!edev) {
				dbg("usbip_exported_device_new failed");
				continue;
			}

			list_add(&edev->node, &device_driver->edev_list);
			device_driver->ndevs++;
		}
	}

	return 0;
}

static void usbip_exported_device_destroy(void)
{
	struct list_head *i, *tmp;
	struct usbip_exported_device *edev;

	list_for_each_safe(i, tmp, &device_driver->edev_list) {
		edev = list_entry(i, struct usbip_exported_device, node);
		list_del(i);
		free(edev);
	}
}

int usbip_device_driver_open(void)
{
	int rc;

	udev_context = udev_new();
	if (!udev_context) {
		err("udev_new failed");
		return -1;
	}

	device_driver = calloc(1, sizeof(*device_driver));

	device_driver->ndevs = 0;
	INIT_LIST_HEAD(&device_driver->edev_list);

	rc = refresh_exported_devices();
	if (rc < 0)
		goto err_free_device_driver;

	return 0;

err_free_device_driver:
	free(device_driver);
	device_driver = NULL;

	udev_unref(udev_context);

	return -1;
}

void usbip_device_driver_close(void)
{
	if (!device_driver)
		return;

	usbip_exported_device_destroy();

	free(device_driver);
	device_driver = NULL;

	udev_unref(udev_context);
}

int usbip_device_refresh_device_list(void)
{
	int rc;

	usbip_exported_device_destroy();

	device_driver->ndevs = 0;
	INIT_LIST_HEAD(&device_driver->edev_list);

	rc = refresh_exported_devices();
	if (rc < 0)
		return -1;

	return 0;
}

int usbip_device_export_device(struct usbip_exported_device *edev, int sockfd)
{
	char attr_name[] = "usbip_sockfd";
	char sockfd_attr_path[SYSFS_PATH_MAX];
	char sockfd_buff[30];
	int ret;

	if (edev->status != SDEV_ST_AVAILABLE) {
		dbg("device not available: %s", edev->udev.busid);
		switch (edev->status) {
		case SDEV_ST_ERROR:
			dbg("status SDEV_ST_ERROR");
			break;
		case SDEV_ST_USED:
			dbg("status SDEV_ST_USED");
			break;
		default:
			dbg("status unknown: 0x%x", edev->status);
		}
		return -1;
	}


	/* only the first interface is true */
	snprintf(sockfd_attr_path, sizeof(sockfd_attr_path), "%s/%s",
		 edev->udev.path, attr_name);

	snprintf(sockfd_buff, sizeof(sockfd_buff), "%d\n", sockfd);

	ret = write_sysfs_attribute(sockfd_attr_path, sockfd_buff,
				    strlen(sockfd_buff));
	if (ret < 0) {
		err("write_sysfs_attribute failed: sockfd %s to %s",
		    sockfd_buff, sockfd_attr_path);
		return ret;
	}

	info("connect: %s", edev->udev.busid);

	return ret;
}

struct usbip_exported_device *usbip_device_get_device(int num)
{
	struct list_head *i;
	struct usbip_exported_device *edev;
	int cnt = 0;

	list_for_each(i, &device_driver->edev_list) {
		edev = list_entry(i, struct usbip_exported_device, node);
		if (num == cnt)
			return edev;
		else
			cnt++;
	}

	return NULL;
}
