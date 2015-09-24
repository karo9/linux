/*
 * vudc.c -- USB over IP UDC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/list.h>
#include <linux/usb/gadget.h>
#include <linux/usb/ch9.h>
#include <linux/sysfs.h>
#include <linux/kthread.h>
#include <linux/byteorder/generic.h>

#include "usbip_common.h"
#include "stub.h"
#include "vudc.h"

#include <net/sock.h>

/* called with udc->lock held */
static ssize_t fetch_descriptor(struct usb_ctrlrequest* req, struct vudc* udc,
				char *out, ssize_t sz, ssize_t maxsz)
{
	struct vrequest *usb_req;
	int ret;
	int copysz;
	struct vep *ep0 = usb_ep_to_vep(udc->gadget.ep0);

	if (!udc->driver)	/* No device for export */
		return 0;
	spin_unlock(&udc->lock);
	ret = udc->driver->setup(&(udc->gadget), req);
	spin_lock(&udc->lock);
	if (ret < 0)
		goto exit;

	/* assuming request queue is empty; request is now on top */
	usb_req = list_entry(ep0->queue.prev, struct vrequest, queue);
	list_del(&(usb_req->queue));

	copysz = min(sz, (ssize_t) usb_req->req.length);
	if (maxsz < copysz) {
		ret = -1;
		goto clean_req;
	}

	memcpy(out, usb_req->req.buf, copysz);
	ret = copysz;

clean_req:
	usb_req->req.status = 0;
	usb_req->req.actual = usb_req->req.length;
	usb_gadget_giveback_request(&(ep0->ep), &(usb_req->req));
exit:
	return ret;
}

/* called with sdev->lock held */
int descriptor_cache(struct vudc *sdev)
{
	struct usb_device_descriptor *dev_d = sdev->dev_descr;
	struct usb_ctrlrequest req;
	int ret;
	int sz = 0;
	int max_sz = PAGE_SIZE;

	if (!sdev || !sdev->driver)
		return -1;

	req.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = cpu_to_le16(USB_DT_DEVICE << 8);
	req.wIndex = cpu_to_le16(0);
	req.wLength = cpu_to_le16(max_sz - sz);

	ret = fetch_descriptor(&req, sdev, (char *) dev_d, max_sz, max_sz);
	if (ret < 0) {
		dev_err(&sdev->plat->dev, "Couldn't fetch device descriptor!");
		return -1;
	}
	sz += ret;

	return 0;
}

/*
 * Fetches device descriptor from the gadget driver.
 */
static ssize_t dev_descr_show(struct device *dev,
			       struct device_attribute *attr, char *out)
{
	struct vudc *sdev;

	sdev = (struct vudc*) dev_get_drvdata(dev);
	if (!sdev->driver)
		return -ENODEV;
	memcpy(out, sdev->dev_descr, sizeof(struct usb_device_descriptor));
	return sizeof(struct usb_device_descriptor);
}
static DEVICE_ATTR_RO(dev_descr);

static ssize_t store_sockfd(struct device *dev, struct device_attribute *attr,
		     const char *in, size_t count)
{
	int rv;
	int sockfd = 0;
	struct vudc *vudc;
	int err;
	struct socket *socket;

	vudc = (struct vudc*)dev_get_drvdata(dev);
	if (!vudc || !vudc->driver) { /* Don't export what we don't have */
		dev_err(&vudc->plat->dev, "no device or gadget not bound");
		return -ENODEV;
	}

	rv = sscanf(in, "%d", &sockfd);
	if (rv != 1)
		return -EINVAL;

	if (sockfd != -1) {
		spin_lock_irq(&vudc->udev.lock);

		if (vudc->udev.status != SDEV_ST_AVAILABLE) {
			goto err;
		}

		socket = sockfd_lookup(sockfd, &err);
		if (!socket) {
			dev_err(&vudc->plat->dev, "failed to lookup sock");
			goto err;
		}

		vudc->udev.tcp_socket = socket;

		spin_unlock_irq(&vudc->udev.lock);

		vudc->udev.tcp_rx = kthread_get_run(&stub_rx_loop, &vudc->udev, "vudc_rx");
		vudc->udev.tcp_tx = kthread_get_run(&stub_tx_loop, &vudc->udev, "vudc_tx");

		spin_lock_irq(&vudc->udev.lock);
		vudc->udev.status = SDEV_ST_USED;
		spin_unlock_irq(&vudc->udev.lock);

		spin_lock_irq(&vudc->lock);
		do_gettimeofday(&vudc->start_time);
		v_start_timer(vudc);
		spin_unlock_irq(&vudc->lock);
	} else {
		spin_lock_irq(&vudc->udev.lock);
		if (vudc->udev.status != SDEV_ST_USED)
			goto err;
		spin_unlock_irq(&vudc->udev.lock);

		usbip_event_add(&vudc->udev, SDEV_EVENT_DOWN);
	}

	return count;

err:
	spin_unlock_irq(&vudc->udev.lock);
	return -EINVAL;
}
static DEVICE_ATTR(usbip_sockfd, S_IWUSR, NULL, store_sockfd);

static ssize_t usbip_status_show(struct device *dev,
			       struct device_attribute *attr, char *out)
{
	struct vudc *sdev = (struct vudc *) dev_get_drvdata(dev);
	int status;

	if (!sdev) {
		dev_err(&sdev->plat->dev, "no device");
		return -ENODEV;
	}
	spin_lock_irq(&sdev->udev.lock);
	status = sdev->udev.status;
	spin_unlock_irq(&sdev->udev.lock);

	return snprintf(out, PAGE_SIZE, "%d\n", status);
}
static DEVICE_ATTR_RO(usbip_status);

static struct attribute *dev_attrs[] = {
	&dev_attr_dev_descr.attr,
	&dev_attr_usbip_sockfd.attr,
	&dev_attr_usbip_status.attr,
	NULL,
};

const struct attribute_group vudc_attr_group = {
	.attrs = dev_attrs,
};
