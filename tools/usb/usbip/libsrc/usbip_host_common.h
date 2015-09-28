/*
 * Copyright (C) 2015 Samsung Electronics
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

#ifndef __USBIP_HOST_COMMON_H
#define __USBIP_HOST_COMMON_H

#include <stdint.h>
#include "usbip_common.h"
#include "list.h"

struct usbip_exported_device {
	struct udev_device *sudev;
	int32_t status;
	struct usbip_usb_device udev;
	struct list_head node;
	struct usbip_usb_interface uinf[];
};

#endif /* __USBIP_HOST_COMMON_H */
