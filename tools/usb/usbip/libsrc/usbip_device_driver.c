 /*
 * Copyright (C) 2015 Karol Kosik <karo9@interia.eu>
 * 		 2015 Samsung Electronics
 * 		 2015 Igor Kotrasinski <i.kotrasinsk@samsung.com>
 * Based on tools/usb/usbip/libsrc/usbip_host_driver.c by matt mooney and
 * Takahiro Hirofuchi
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
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

#include <fcntl.h>
#include <string.h>
#include <linux/usb/ch9.h>

#include <unistd.h>

#include "usbip_host_common.h"
#include "usbip_device_driver.h"

#undef  PROGNAME
#define PROGNAME "libusbip"

struct udev *udev_context;
static const char *udev_subsystem = "udc";
struct usbip_host_driver *device_driver;

#define copy_descr_attr16(dev, descr, attr)			\
		((dev)->attr = le16toh((descr)->attr))		\

#define copy_descr_attr(dev, descr, attr)			\
		((dev)->attr = (descr)->attr)			\

static struct {
	enum usb_device_speed speed;
	const char *name;
} speed_names[] = {
	{
		.speed = USB_SPEED_UNKNOWN,
		.name = "UNKNOWN",
	},
	{
		.speed = USB_SPEED_LOW,
		.name = "low-speed",
	},
	{
		.speed = USB_SPEED_FULL,
		.name = "full-speed",
	},
	{
		.speed = USB_SPEED_HIGH,
		.name = "high-speed",
	},
	{
		.speed = USB_SPEED_WIRELESS,
		.name = "wireless",
	},
	{
		.speed = USB_SPEED_SUPER,
		.name = "super-speed",
	},
};

#define SPEED_NUMBER 6

static
int read_usb_vudc_device(struct udev_device *sdev, struct usbip_usb_device *dev)
{
	const char *path, *name;
	char filepath[SYSFS_PATH_MAX];
	struct usb_device_descriptor descr;
	int ret = 0, i;
	FILE *fd = NULL;
	struct udev_device *plat;
	const char* speed;

	plat = udev_device_get_parent(sdev);
	path = udev_device_get_syspath(plat);
	snprintf(filepath, SYSFS_PATH_MAX, "%s/%s",
		 path, VUDC_DEVICE_DESCR_FILE);
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

	dev->speed = USB_SPEED_UNKNOWN;
	speed = udev_device_get_sysattr_value(sdev, "current_speed");
	if (speed) {
		for (i = 0; i < SPEED_NUMBER; i++) {
			if (!strcmp(speed_names[i].name, speed)) {
				dev->speed = speed_names[i].speed;
				break;
			}
		}
	}

	/* Only used for user output, little sense to output them in general */
	dev->bNumInterfaces = 0;
	dev->bConfigurationValue = 0;
	dev->busnum = 0;

	name = udev_device_get_sysname(plat);
	strncpy(dev->busid, name, SYSFS_BUS_ID_SIZE);
	return 0;
}

static int is_my_device(struct udev_device *dev)
{
	const char *driver;

	driver = udev_device_get_property_value(dev, "USB_UDC_NAME");
	return driver != NULL && !strcmp(driver, USBIP_DEVICE_DRV_NAME);
}

int usbip_device_driver_open(void)
{
	int rc;

	device_driver = calloc(1, sizeof(*device_driver));

	device_driver->ndevs = 0;
	INIT_LIST_HEAD(&device_driver->edev_list);
	device_driver->udev_subsystem = udev_subsystem;
	device_driver->o.read_device = read_usb_vudc_device;
	device_driver->o.read_interface = NULL; /* FIXME */
	device_driver->o.is_my_device = is_my_device;

	rc = usbip_common_driver_open(device_driver);
	if (rc < 0)
		goto err_free_device_driver;

	return 0;

err_free_device_driver:
	free(device_driver);
	device_driver = NULL;

	return -1;
}

void usbip_device_driver_close(void)
{
	if (!device_driver)
		return;

	usbip_common_driver_close(device_driver);

	free(device_driver);
	device_driver = NULL;
}

int usbip_device_refresh_device_list(void)
{
	return usbip_common_refresh_device_list(device_driver);
}

struct usbip_exported_device *usbip_device_get_device(int num)
{
	return usbip_common_get_device(device_driver, num);
}
