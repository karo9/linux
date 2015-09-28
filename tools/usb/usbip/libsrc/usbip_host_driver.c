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

#include <unistd.h>

#include "usbip_host_common.h"
#include "usbip_host_driver.h"

#undef  PROGNAME
#define PROGNAME "libusbip"

struct udev *udev_context;
static const char *udev_subsystem = "usb";
static const char *driver_name = USBIP_HOST_DRV_NAME;
struct usbip_host_driver *host_driver;

int usbip_host_driver_open(void)
{
	int rc;

	host_driver = calloc(1, sizeof(*host_driver));

	host_driver->ndevs = 0;
	INIT_LIST_HEAD(&host_driver->edev_list);
	host_driver->udev_subsystem = udev_subsystem;
	host_driver->name = driver_name;
	host_driver->o.read_device = read_usb_device;
	host_driver->o.read_interface = read_usb_interface;

	rc = usbip_common_driver_open(host_driver);
	if (rc < 0)
		goto err_free_host_driver;

	return 0;

err_free_host_driver:
	free(host_driver);
	host_driver = NULL;

	return -1;
}

void usbip_host_driver_close(void)
{
	if (!host_driver)
		return;

	usbip_common_driver_close(host_driver);

	free(host_driver);
	host_driver = NULL;
}

int usbip_host_refresh_device_list(void)
{
	return usbip_common_refresh_device_list(host_driver);
}

struct usbip_exported_device *usbip_host_get_device(int num)
{
	return usbip_common_get_device(host_driver, num);
}
