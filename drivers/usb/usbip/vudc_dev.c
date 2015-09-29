/*
 * Copyright (C) 2015 Karol Kosik <karo9@interia.eu>
 * 		 2015 Samsung Electronics
 * Author:	 Igor Kotrasinski <i.kotrasinsk@samsung.com>
 *
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/kthread.h>
#include <linux/file.h>
#include <linux/byteorder/generic.h>

#include "usbip_common.h"
#include "vudc.h"

#define VIRTUAL_ENDPOINTS (1 /* ep0 */ + 15 /* in eps */ + 15 /* out eps */)

/* urb-related structures alloc / free */


static void free_urb(struct urb *urb)
{
	if (!urb)
		return;

	kfree(urb->setup_packet);
	urb->setup_packet = NULL;

	kfree(urb->transfer_buffer);
	urb->transfer_buffer = NULL;

	usb_free_urb(urb);
}

struct urbp *alloc_urbp(void)
{
	struct urbp *urb_p;

	urb_p = kzalloc(sizeof(*urb_p), GFP_KERNEL);
	if (!urb_p)
		return urb_p;

	urb_p->urb = NULL;
	urb_p->ep = NULL;
	INIT_LIST_HEAD(&urb_p->urb_entry);
	return urb_p;
}

static void free_urbp(struct urbp *urb_p)
{
	kfree(urb_p);
}

void free_urbp_and_urb(struct urbp *urb_p)
{
	if (!urb_p)
		return;
	free_urb(urb_p->urb);
	free_urbp(urb_p);
}


/* utilities ; almost verbatim from dummy_hcd.c */

/* called with spinlock held */
static void nuke(struct vudc *cdev, struct vep *ep)
{
	struct vrequest	*req;

	while (!list_empty(&ep->req_queue)) {
		req = list_first_entry(&ep->req_queue, struct vrequest,
				       req_entry);
		list_del_init(&req->req_entry);
		req->req.status = -ESHUTDOWN;

		spin_unlock(&cdev->lock);
		usb_gadget_giveback_request(&ep->ep, &req->req);
		spin_lock(&cdev->lock);
	}
}

/* caller must hold lock */
static void stop_activity(struct vudc *cdev)
{
	int i;
	struct urbp *urb_p, *tmp;

	cdev->address = 0;

	for (i = 0; i < VIRTUAL_ENDPOINTS; i++) {
		nuke(cdev, &cdev->ep[i]);
	}

	list_for_each_entry_safe(urb_p, tmp, &cdev->urb_queue, urb_entry) {
		list_del(&urb_p->urb_entry);
		free_urbp_and_urb(urb_p);
	}
}

struct vep *find_endpoint(struct vudc *cdev, u8 address)
{
	int i;

	if ((address & ~USB_DIR_IN) == 0)
		return &cdev->ep[0];

	for (i = 1; i < VIRTUAL_ENDPOINTS; i++) {
		struct vep *ep = &cdev->ep[i];

		if (!ep->desc)
			continue;
		if (ep->desc->bEndpointAddress == address)
			return ep;
	}
	return NULL;
}

/* gadget ops */

/* FIXME - this will probably misbehave when suspend/resume is added */
static int vgadget_get_frame(struct usb_gadget *_gadget)
{
	struct timeval now;
	struct vudc *cdev = usb_gadget_to_vudc(_gadget);

	do_gettimeofday(&now);
	return ((now.tv_sec - cdev->start_time.tv_sec) * 1000 +
			(now.tv_usec - cdev->start_time.tv_usec) / 1000)
			% 0x7FF;
}

static int vgadget_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	struct vudc *cdev = usb_gadget_to_vudc(_gadget);

	if (value)
		cdev->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		cdev->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);
	return 0;
}

static int vgadget_pullup(struct usb_gadget *_gadget, int value)
{
	struct vudc *cdev = usb_gadget_to_vudc(_gadget);

	if (value && cdev->driver) {
		cdev->gadget.speed = min_t(u8, USB_SPEED_HIGH,
					   cdev->driver->max_speed);
		cdev->ep[0].ep.maxpacket = 64;
	}
	return 0;
}

static int vgadget_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct vudc *cdev = usb_gadget_to_vudc(g);
	unsigned long flags;
	int ret;

	cdev->driver = driver;
	spin_lock_irqsave(&cdev->lock, flags);
	ret = descriptor_cache(cdev);
	spin_unlock_irqrestore(&cdev->lock, flags);
	if (ret) {
		usbip_event_add(&cdev->ud, VUDC_EVENT_ERROR_USB);
	}
	return 0;
}

static int vgadget_udc_stop(struct usb_gadget *g)
{
	struct vudc *cdev = usb_gadget_to_vudc(g);

	usbip_event_add(&cdev->ud, VUDC_EVENT_REMOVED);
	usbip_stop_eh(&cdev->ud); /* Wait for eh completion */
	usbip_start_eh(&cdev->ud);

	cdev->driver = NULL;
	return 0;
}

static const struct usb_gadget_ops vgadget_ops = {
	.get_frame	= vgadget_get_frame,
	.set_selfpowered = vgadget_set_selfpowered,
	.pullup		= vgadget_pullup,
	.udc_start	= vgadget_udc_start,
	.udc_stop	= vgadget_udc_stop,
};


/* endpoint ops */

static int vep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct vep *ep;
	struct vudc *cdev;
	unsigned maxp;
	unsigned long flags;

	ep = to_vep(_ep);
	cdev = ep_to_vudc(ep);

	if (!_ep || !desc || ep->desc || _ep->caps.type_control
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	if (!cdev->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&cdev->lock, flags);

	maxp = usb_endpoint_maxp(desc) & 0x7ff;
	_ep->maxpacket = maxp;
	ep->desc = desc;
	ep->type = usb_endpoint_type(desc);
	ep->halted = ep->wedged = 0;

	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}

static int vep_disable(struct usb_ep *_ep)
{
	struct vep *ep;
	struct vudc *cdev;
	unsigned long flags;

	ep = to_vep(_ep);
	cdev = ep_to_vudc(ep);
	if (!_ep || !ep->desc || _ep->caps.type_control)
		return -EINVAL;

	spin_lock_irqsave(&cdev->lock, flags);
	ep->desc = NULL;
	nuke(cdev, ep);
	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}

static struct usb_request *vep_alloc_request(struct usb_ep *_ep,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;

	if (!_ep)
		return NULL;
	ep = to_vep(_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->req_entry);

	return &req->req;
}

static void vep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vrequest *req;

	if (!_ep || !_req) {
		WARN_ON(1);
		return;
	}
	req = to_vrequest(_req);
	kfree(req);
}

static int vep_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;
	struct vudc *cdev;
	unsigned long flags;

	if (!_ep || !_req)
		return -EINVAL;

	ep = to_vep(_ep);
	req = to_vrequest(_req);
	cdev = ep_to_vudc(ep);

	spin_lock_irqsave(&cdev->lock, flags);
	_req->actual = 0;
	_req->status = -EINPROGRESS;

	list_add_tail(&req->req_entry, &ep->req_queue);
	spin_unlock_irqrestore(&cdev->lock, flags);

	return 0;
}

static int vep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vep *ep;
	struct vrequest *req;
	struct vudc *cdev;
	struct vrequest *lst;
	unsigned long flags;
	int retval = -EINVAL;

	if (!_ep || !_req)
		return retval;

	ep = to_vep(_ep);
	req = to_vrequest(_req);
	cdev = req->cdev;

	if (!cdev->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&cdev->lock, flags);
	list_for_each_entry(lst, &ep->req_queue, req_entry) {
		if (&lst->req == _req) {
			list_del_init(&lst->req_entry);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (retval == 0)
		usb_gadget_giveback_request(_ep, _req);

	return retval;
}

static int
vep_set_halt_and_wedge(struct usb_ep *_ep, int value, int wedged)
{
	struct vep *ep;
	struct vudc *cdev;
	int retval = 0;
	/* FIXME should we lock here? */
	ep = to_vep(_ep);
	if (!_ep)
		return -EINVAL;

	cdev = ep_to_vudc(ep);
	if (!cdev->driver)
		return -ESHUTDOWN;

	if (!value)
		ep->halted = ep->wedged = 0;
	else if (ep->desc && (ep->desc->bEndpointAddress & USB_DIR_IN) &&
			!list_empty(&ep->req_queue))
		retval = -EAGAIN;
	else {
		ep->halted = 1;
		if (wedged)
			ep->wedged = 1;
	}
	return retval;
}

static int
vep_set_halt(struct usb_ep *_ep, int value)
{
	return vep_set_halt_and_wedge(_ep, value, 0);
}

static int vep_set_wedge(struct usb_ep *_ep)
{
	return vep_set_halt_and_wedge(_ep, 1, 1);
}

static const struct usb_ep_ops vep_ops = {
	.enable		= vep_enable,
	.disable	= vep_disable,

	.alloc_request	= vep_alloc_request,
	.free_request	= vep_free_request,

	.queue		= vep_queue,
	.dequeue	= vep_dequeue,

	.set_halt	= vep_set_halt,
	.set_wedge	= vep_set_wedge,
};


/* shutdown / reset / error handlers */

static void vudc_shutdown(struct usbip_device *ud)
{
	struct vudc *cdev = container_of(ud, struct vudc, ud);
	unsigned long flags;

	dev_dbg(&cdev->pdev->dev, "device shutdown");
	if (ud->tcp_socket)
		kernel_sock_shutdown(ud->tcp_socket, SHUT_RDWR);

	if (ud->tcp_tx) {
		kthread_stop_put(ud->tcp_rx);
		ud->tcp_rx = NULL;
	}
	if (ud->tcp_tx) {
		kthread_stop_put(ud->tcp_tx);
		ud->tcp_tx = NULL;
	}

	if (ud->tcp_socket) {
		sockfd_put(ud->tcp_socket);
		ud->tcp_socket = NULL;
	}

	spin_lock_irqsave(&cdev->lock, flags);
	stop_activity(cdev);
	spin_unlock_irqrestore(&cdev->lock, flags);
	if (cdev->driver) /* We might be cleaning up after driver unbind */
		cdev->driver->disconnect(&cdev->gadget);
}

static void vudc_device_reset(struct usbip_device *ud)
{
	struct vudc *cdev = container_of(ud, struct vudc, ud);
	unsigned long flags;

	dev_dbg(&cdev->pdev->dev, "device reset");
	spin_lock_irqsave(&cdev->lock, flags);
	stop_activity(cdev);
	spin_unlock_irqrestore(&cdev->lock, flags);
	if (cdev->driver)
		usb_gadget_udc_reset(&cdev->gadget, cdev->driver);
	spin_lock_irqsave(&ud->lock, flags);
	ud->status = SDEV_ST_AVAILABLE;
	spin_unlock_irqrestore(&ud->lock, flags);
}

static void vudc_device_unusable(struct usbip_device *ud)
{
	unsigned long flags;

	spin_lock_irqsave(&ud->lock, flags);
	ud->status = SDEV_ST_ERROR;
	spin_unlock_irqrestore(&ud->lock, flags);
}

/* device setup / cleanup */

struct vudc_device *alloc_vudc_device(int devid)
{
	struct vudc_device *udc_dev = NULL;

	udc_dev = kzalloc(sizeof(*udc_dev), GFP_KERNEL);
	if (!udc_dev)
		goto out;

	INIT_LIST_HEAD(&udc_dev->dev_entry);

	udc_dev->pdev = platform_device_alloc(GADGET_NAME, devid);
	if (!udc_dev->pdev) {
		kfree(udc_dev);
		udc_dev = NULL;
	}

out:
	return udc_dev;
}

void put_vudc_device(struct vudc_device *udc_dev)
{
	platform_device_put(udc_dev->pdev);
	kfree(udc_dev);
}

static int init_vudc_hw(struct vudc *cdev)
{
	int i;
	struct usbip_device *ud = &cdev->ud;
	struct vep *ep;

	cdev->ep = kcalloc(VIRTUAL_ENDPOINTS, sizeof(*cdev->ep), GFP_KERNEL);
	if (!cdev->ep)
		goto nomem_ep;

	INIT_LIST_HEAD(&cdev->gadget.ep_list);

	/* create ep0 and 15 in, 15 out general purpose eps */
	for (i = 0; i < VIRTUAL_ENDPOINTS; ++i) {
		int is_out = i % 2;
		int num = (i + 1) / 2;

		ep = &cdev->ep[i];

		sprintf(ep->name, "ep%d%s", num,
			i ? (is_out ? "out" : "in") : "");
		ep->ep.name = ep->name;
		if (i == 0) {
			ep->ep.caps.type_control = true;
			ep->ep.caps.dir_out = true;
			ep->ep.caps.dir_in = true;
		} else {
			ep->ep.caps.type_iso = true;
			ep->ep.caps.type_int = true;
			ep->ep.caps.type_bulk = true;
		}

		if (is_out)
			ep->ep.caps.dir_out = true;
		else
			ep->ep.caps.dir_in = true;

		ep->ep.ops = &vep_ops;
		list_add_tail(&ep->ep.ep_list, &cdev->gadget.ep_list);
		ep->halted = ep->wedged = ep->already_seen =
			ep->setup_stage = 0;
		usb_ep_set_maxpacket_limit(&ep->ep, ~0);
		ep->ep.max_streams = 16;
		ep->gadget = &cdev->gadget;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->req_queue);
	}

	spin_lock_init(&cdev->lock);
	spin_lock_init(&cdev->lock_tx);
	INIT_LIST_HEAD(&cdev->urb_queue);
	INIT_LIST_HEAD(&cdev->tx_queue);
	init_waitqueue_head(&cdev->tx_waitq);

	spin_lock_init(&ud->lock);
	ud->status = SDEV_ST_AVAILABLE;
	ud->side = USBIP_VUDC;

	ud->eh_ops.shutdown = vudc_shutdown;
	ud->eh_ops.reset    = vudc_device_reset;
	ud->eh_ops.unusable = vudc_device_unusable;

	usbip_start_eh(ud);

	cdev->gadget.ep0 = &cdev->ep[0].ep;
	list_del_init(&cdev->ep[0].ep.ep_list);

	v_init_timer(cdev);
	return 0;

nomem_ep:
		return -ENOMEM;
}

static void cleanup_vudc_hw(struct vudc *cdev)
{
	usbip_event_add(&cdev->ud, VUDC_EVENT_REMOVED);
	usbip_stop_eh(&cdev->ud);
	kfree(cdev->ep);
}

/* platform driver ops */

int vudc_probe(struct platform_device *pdev)
{
	struct vudc *cdev;
	int retval = -ENOMEM;

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		goto out;

	cdev->gadget.name = GADGET_NAME;
	cdev->gadget.ops = &vgadget_ops;
	cdev->gadget.max_speed = USB_SPEED_HIGH;
	cdev->gadget.dev.parent = &pdev->dev;

	retval = init_vudc_hw(cdev);
	if (retval)
		goto err_init_vudc_hw;

	retval = usb_add_gadget_udc(&pdev->dev, &cdev->gadget);
	if (retval < 0)
		goto err_add_udc;

	retval = sysfs_create_group(&pdev->dev.kobj, &vudc_attr_group);
	if (retval) {
		dev_err(&cdev->pdev->dev, "create sysfs files\n");
		goto err_sysfs;
	}

	platform_set_drvdata(pdev, cdev);

	return retval;

err_sysfs:
	usb_del_gadget_udc(&cdev->gadget);
err_add_udc:
	cleanup_vudc_hw(cdev);
err_init_vudc_hw:
	kfree(cdev);
out:
	return retval;
}

int vudc_remove(struct platform_device *pdev)
{
	struct vudc *cdev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &vudc_attr_group);
	usb_del_gadget_udc(&cdev->gadget);
	cleanup_vudc_hw(cdev);
	kfree(cdev);
	return 0;
}
