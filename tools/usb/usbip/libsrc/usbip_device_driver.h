/*
 * Copyright (C) 2015 Karol Kosik <karo9@interia.eu>
 * 		 2015 Samsung Electronics
 * 		 2015 Igor Kotrasinski <i.kotrasinsk@samsung.com>
 * Based on tools/usb/usbip/libsrc/usbip_host_driver.h by matt mooney and
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

#ifndef __USBIP_DEVICE_DRIVER_H
#define __USBIP_DEVICE_DRIVER_H

#include <stdint.h>
#include "usbip_common.h"
#include "usbip_host_common.h"
#include "list.h"

extern struct usbip_host_driver *device_driver;

int usbip_device_driver_open(void);
void usbip_device_driver_close(void);
int usbip_device_refresh_device_list(void);
struct usbip_exported_device *usbip_device_get_device(int num);

#endif /* __USBIP_DEVICE_DRIVER_H */
