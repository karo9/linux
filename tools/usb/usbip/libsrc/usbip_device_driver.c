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

#define GADGET_DIR "/sys/kernel/config/usb_gadget/"

struct usbip_device_driver *device_driver;

static
struct usbip_exported_device *usbip_exported_device_new(const char *path)
{
	FILE *fp;
	DIR *dir;
	struct dirent *ent;
	char buf1[100], buf2[100];
	unsigned int tmpNum;

	struct usbip_exported_device *edev = NULL;
	struct usbip_exported_device *edev_old;
	size_t size;
	int i;

	edev = calloc(1, sizeof(struct usbip_exported_device));

//	edev->sudev = udev_device_new_from_syspath(udev_context, sdevpath);
//	if (!edev->sudev) {
//		err("udev_device_new_from_syspath: %s", sdevpath);
//		goto err;
//	}
	edev->sudev = NULL;

	//read_usb_device(edev->sudev, &edev->udev);
	if((dir = opendir(path)) != NULL) 
	{
		while((ent = readdir(dir)) != NULL) 
		{
			if(ent->d_name[0] != '.')
			{
				strcpy(buf1, path);
				strcat(buf1, ent->d_name);

				if(strcmp(ent->d_name, "idProduct") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.idProduct = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("idProduct: %d\n", tmpNum);
				}
				else if(strcmp(ent->d_name, "idVendor") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.idVendor = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("idVendor: %d\n", tmpNum);
				}
				else if(strcmp(ent->d_name, "bcdDevice") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.bcdDevice = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("bcdDevice: %d\n", tmpNum);
				}
				else if(strcmp(ent->d_name, "bDeviceClass") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.bDeviceClass = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("bDeviceClass: %d\n", tmpNum);
				}
				else if(strcmp(ent->d_name, "bDeviceProtocol") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.bDeviceProtocol = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("bDeviceProtocol: %d\n", tmpNum);
				}
				else if(strcmp(ent->d_name, "bDeviceSubClass") == 0)
				{
					fp = fopen(buf1, "r");
					fread(buf2, 100, 1, fp);
					sscanf(buf2, "%x\n", &tmpNum);
					edev->udev.bDeviceSubClass = (uint16_t)tmpNum;
					fclose(fp);
					
					printf("bDeviceSubClass: %d\n", tmpNum);
				}
			}
			strncpy(edev->udev.path, path, SYSFS_PATH_MAX);
			// todo
			strncpy(edev->udev.busid, "g1", SYSFS_BUS_ID_SIZE);
			edev->udev.busnum = 1;
		}
		closedir(dir);
	} 
	
	// todo
	edev->udev.bConfigurationValue = 1;
	edev->udev.bNumConfigurations = 1;
	edev->udev.bNumInterfaces = 1;
	printf("Ustawiaw speed na USB_SPEED_HIGH\n");
	edev->udev.speed = USB_SPEED_HIGH;

	// todo
	edev->status = 0;
	if (edev->status < 0)
		goto err;

	/* reallocate buffer to include usb interface data */
	size = sizeof(struct usbip_exported_device) +
		edev->udev.bNumInterfaces * sizeof(struct usbip_usb_interface);

	edev_old = edev;
	edev = realloc(edev, size);
	if (!edev) {
		edev = edev_old;
		dbg("realloc failed");
		goto err;
	}

	for (i = 0; i < edev->udev.bNumInterfaces; i++)
	{
		//read_usb_interface(&edev->udev, i, &edev->uinf[i]);
	}

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
	DIR *dir;
	struct dirent *ent;
	char buf1[100], path[100];
	struct usbip_exported_device *edev;

	strcpy(buf1, GADGET_DIR);
	if((dir = opendir(buf1)) != NULL) 
	{
		while((ent = readdir(dir)) != NULL) 
		{
			if(ent->d_name[0] != '.')
			{
				strcpy(path, buf1);
				strcat(path, ent->d_name);
				strcat(path, "/");
				edev = usbip_exported_device_new(path);
				if (!edev) {
					dbg("usbip_exported_device_new failed");
					continue;
				}

				list_add(&edev->node, &device_driver->edev_list);
				device_driver->ndevs++;
			}
		}
		closedir(dir);
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

	return -1;
}

void usbip_device_driver_close(void)
{
	if (!device_driver)
		return;

	usbip_exported_device_destroy();

	free(device_driver);
	device_driver = NULL;
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

//TODO later
int usbip_device_export_device(struct usbip_exported_device *edev, int sockfd)
{
	//char attr_name[] = "usbip_sockfd";
	//char sockfd_attr_path[SYSFS_PATH_MAX];
	char sockfd_attr_path[] = "/sys/devices/platform/vudc.0/vudc_sockfd";
	char sockfd_buff[30];
	int ret;

/* 	if (edev->status != SDEV_ST_AVAILABLE) {
 * 		dbg("device not available: %s", edev->udev.busid);
 * 		switch (edev->status) {
 * 		case SDEV_ST_ERROR:
 * 			dbg("status SDEV_ST_ERROR");
 * 			break;
 * 		case SDEV_ST_USED:
 * 			dbg("status SDEV_ST_USED");
 * 			break;
 * 		default:
 * 			dbg("status unknown: 0x%x", edev->status);
 * 		}
 * 		return -1;
 * 	}
 */

	/* only the first interface is true */
/* 	snprintf(sockfd_attr_path, sizeof(sockfd_attr_path), "%s/%s",
 * 		 edev->udev.path, attr_name);
 */

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
