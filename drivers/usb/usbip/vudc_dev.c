/*
 * vudc.c -- USB over IP UDC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include "stub.h"
#include "vudc.h"

#define DEBUG 1
#define debug_print(...) \
		do { if (DEBUG) printk(KERN_ERR __VA_ARGS__); } while (0)

/* String constants and ep names */

static const char gadget_name[] = "usbip-vudc";

const char ep0name[] = "ep0";
const char *const ep_name[] = {
	ep0name,				/* everyone has ep0 */

	"ep1in-bulk", "ep2out-bulk", "ep3in-iso", "ep4out-iso", "ep5in-int",
	"ep6in-bulk", "ep7out-bulk", "ep8in-iso", "ep9out-iso", "ep10in-int",
	"ep11in-bulk", "ep12out-bulk", "ep13in-iso", "ep14out-iso", "ep15in-int",

	"ep1out-bulk", "ep2in-bulk",

	"ep3out", "ep4in", "ep5out", "ep6out", "ep7in", "ep8out", "ep9in",
	"ep10out", "ep11out", "ep12in", "ep13out", "ep14in", "ep15out",
};
#define VIRTUAL_ENDPOINTS	ARRAY_SIZE(ep_name)


/* urb-related structures alloc / free */

int alloc_urb_from_cmd(struct urb **urbp, struct usbip_header *pdu)
{
	struct urb* urb;
	/* TODO - support isoc packets */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		goto err;

	usbip_pack_pdu(pdu, urb, USBIP_CMD_SUBMIT, 0);

	if (urb->transfer_buffer_length > 0) {
		urb->transfer_buffer = kzalloc(urb->transfer_buffer_length,
			GFP_KERNEL);
		if(!urb->transfer_buffer)
			goto free_urb;
	}

	urb->setup_packet = kmemdup(&pdu->u.cmd_submit.setup, 8,
			    GFP_KERNEL);
	if(!urb->setup_packet)
		goto free_buffer;

	/*
	 * FIXME - we only setup pipe enough for usbip functions
	 * to behave nicely
	 */
	if (pdu->base.direction == USBIP_DIR_IN)
		urb->pipe |= USB_DIR_IN;
	else
		urb->pipe &= (~USB_DIR_IN);

	*urbp = urb;
	return 0;

free_buffer:
	kfree(urb->transfer_buffer);
	urb->transfer_buffer = NULL;
free_urb:
	usb_free_urb(urb);
err:
	return -ENOMEM;
}

static void free_urb(struct urb *urb)
{
	if (!urb)
		return;

	if (urb->setup_packet) {
		kfree(urb->setup_packet);
		urb->setup_packet = NULL;
	}

	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = NULL;
	}

	usb_free_urb(urb);
	return;
}

struct urbp* alloc_urbp(void)
{
	struct urbp* urb_p = kmalloc(sizeof(struct urbp), GFP_KERNEL);
	if (!urb_p)
		return urb_p;

	urb_p->urb = NULL;
	urb_p->ep = NULL;
	INIT_LIST_HEAD(&urb_p->urb_q);
	return urb_p;
}

static void free_urbp(struct urbp* urb_p)
{
	if (!urb_p)
		return;
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
static void nuke(struct vudc *sdev, struct vep *ep)
{
	while (!list_empty(&ep->queue)) {
		struct vrequest	*req;

		req = list_entry(ep->queue.next, struct vrequest, queue);
		list_del_init(&req->queue);
		req->req.status = -ESHUTDOWN;

		spin_unlock(&sdev->lock);
		usb_gadget_giveback_request(&ep->ep, &req->req);
		spin_lock(&sdev->lock);
	}
}

/* caller must hold lock */
static void stop_activity(struct vudc *sdev)
{
	int i;
	struct urbp *urb_p, *tmp;

	sdev->address = 0;

	for (i = 0; i < VIRTUAL_ENDPOINTS; i++) {
		if (!ep_name[i])
			break;
		nuke(sdev, &sdev->ep[i]);
	}

	list_for_each_entry_safe(urb_p, tmp, &sdev->urb_q, urb_q) {
		list_del(&urb_p->urb_q);
		free_urbp_and_urb(urb_p);
	}
}

struct vep *find_endpoint(struct vudc *vudc, u8 address)
{
	int i;

	if ((address & ~USB_DIR_IN) == 0)
		return &vudc->ep[0];

	for (i = 1; i < VIRTUAL_ENDPOINTS; i++) {
		struct vep *ep = &vudc->ep[i];

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
	struct vudc *sdev;

	sdev = usb_gadget_to_vudc(_gadget);
	do_gettimeofday(&now);

	return ((now.tv_sec - sdev->start_time.tv_sec) * 1000 +
			(now.tv_usec - sdev->start_time.tv_usec) / 1000)
			% 0x7FF;
}

static int vgadget_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	struct vudc *sdev;

	sdev = usb_gadget_to_vudc(_gadget);
	if (value)
		sdev->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		sdev->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);
	return 0;
}

static int vgadget_pullup(struct usb_gadget *_gadget, int value)
{
	struct vudc *vudc = usb_gadget_to_vudc(_gadget);

	if (value && vudc->driver) {
		/* TODO - setup proper speed */
		vudc->gadget.speed = min_t(u8, USB_SPEED_HIGH,
					   vudc->driver->max_speed);

		if(vudc->gadget.speed == USB_SPEED_SUPER)
			vudc->ep[0].ep.maxpacket = 9;
		else
			vudc->ep[0].ep.maxpacket = 64;
	}
	return 0;
}

static int vgadget_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct vudc *vudc = usb_gadget_to_vudc(g);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&vudc->lock, flags);
	vudc->driver = driver;
	ret = descriptor_cache(vudc);
	spin_unlock_irqrestore(&vudc->lock, flags);
	if (ret) {
		/* FIXME: add correct event */
		usbip_event_add(&vudc->udev, SDEV_EVENT_ERROR_MALLOC);
	}
	return 0;
}

static int vgadget_udc_stop(struct usb_gadget *g)
{
	struct vudc *vudc = usb_gadget_to_vudc(g);
	unsigned long flags;

	usbip_event_add(&vudc->udev, SDEV_EVENT_REMOVED);
	usbip_stop_eh(&vudc->udev); /* Wait for eh completion */
	usbip_start_eh(&vudc->udev);

	spin_lock_irqsave(&vudc->lock, flags);
	vudc->driver = NULL;
	spin_unlock_irqrestore(&vudc->lock, flags);
	return 0;
}

const struct usb_gadget_ops vgadget_ops = {
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
	struct vudc *sdev;
	int retval;
	unsigned maxp;

	ep = usb_ep_to_vep(_ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	sdev = ep_to_vudc(ep);
	if (!sdev->driver)
		return -ESHUTDOWN;

	/* TODO - check if in state allowing for enable */

	maxp = usb_endpoint_maxp(desc) & 0x7ff;

	retval = -EINVAL;
	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_BULK:
		if (strstr(ep->ep.name, "-iso")
				|| strstr(ep->ep.name, "-int"))
			goto done;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (strstr(ep->ep.name, "-iso"))
			goto done;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (strstr(ep->ep.name, "-bulk")
				|| strstr(ep->ep.name, "-int"))
			goto done;
		break;
	default:
		goto done;
	}

	if (desc->bEndpointAddress & USB_DIR_IN) {
		if (strstr(ep->ep.name, "out"))
			goto done;
	} else {
		if (strstr(ep->ep.name, "in"))
			goto done;
	}

	_ep->maxpacket = maxp;
	ep->desc = desc;
	ep->type = usb_endpoint_type(desc);
	ep->halted = ep->wedged = 0;
	retval = 0;

done:
	return retval;
}

static int vep_disable(struct usb_ep *_ep)
{
	struct vep *ep;
	struct vudc *sdev;
	unsigned long flags;

	ep = usb_ep_to_vep(_ep);
	if (!_ep || !ep->desc || _ep->name == ep0name)
		return -EINVAL;
	sdev = ep_to_vudc(ep);

	spin_lock_irqsave(&sdev->lock, flags);
	ep->desc = NULL;
	nuke(sdev, ep);
	spin_unlock_irqrestore(&sdev->lock, flags);

	return 0;
}

static struct usb_request *vep_alloc_request(struct usb_ep *_ep,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;

	if (!_ep)
		return NULL;
	ep = usb_ep_to_vep(_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void vep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vrequest *req;

	if (!_ep || !_req) {
		WARN_ON(1);
		return;
	}
	req = usb_request_to_vrequest(_req);
	kfree(req);
}

static int vep_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;
	struct vudc *vudc;
	unsigned long flags;

	if (!_ep || !_req)
		return -EINVAL;

	ep = usb_ep_to_vep(_ep);
	req = usb_request_to_vrequest(_req);
	vudc = ep_to_vudc(ep);

	_req->actual = 0;
	_req->status = -EINPROGRESS;

	spin_lock_irqsave(&vudc->lock, flags);
	list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&vudc->lock, flags);

	return 0;
}

static int vep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vep *ep;
	struct vrequest *req;
	struct vudc *sdev;
	struct vrequest *lst;
	unsigned long flags;
	int retval = -EINVAL;

	if (!_ep || !_req)
		return retval;

	ep = usb_ep_to_vep(_ep);
	req = usb_request_to_vrequest(_req);
	sdev = req->sdev;

	if (!sdev->driver)
		return -ESHUTDOWN;

	spin_lock_irqsave(&sdev->lock, flags);
	list_for_each_entry(lst, &sdev->urb_q, queue) {
		if (&lst->req == _req) {
			list_del_init(&lst->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&sdev->lock, flags);

	if (retval == 0)
		usb_gadget_giveback_request(_ep, _req);

	return retval;
}

static int
vep_set_halt_and_wedge(struct usb_ep *_ep, int value, int wedged)
{
	struct vep		*ep;
	struct vudc		*sdev;

	if (!_ep)
		return -EINVAL;
	ep = usb_ep_to_vep(_ep);
	sdev = ep_to_vudc(ep);
	if (!sdev->driver)
		return -ESHUTDOWN;
	if (!value)
		ep->halted = ep->wedged = 0;
	else if (ep->desc && (ep->desc->bEndpointAddress & USB_DIR_IN) &&
			!list_empty(&ep->queue))
		return -EAGAIN;
	else {
		ep->halted = 1;
		if (wedged)
			ep->wedged = 1;
	}
	return 0;
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

const struct usb_ep_ops vep_ops = {
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
	struct vudc *sdev = container_of(ud, struct vudc, udev);
	unsigned long flags;

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

	spin_lock_irqsave(&sdev->lock, flags);
	stop_activity(sdev);
	spin_unlock_irqrestore(&sdev->lock, flags);
	if (sdev->driver) /* We might be cleaning up after driver unbind */
		sdev->driver->disconnect(&sdev->gadget);
}

static void vudc_device_reset(struct usbip_device *ud)
{
	struct vudc *sdev = container_of(ud, struct vudc, udev);
	unsigned long flags;

	spin_lock_irqsave(&sdev->lock, flags);
	stop_activity(sdev);
	spin_unlock_irqrestore(&sdev->lock, flags);
	usb_gadget_udc_reset(&sdev->gadget, sdev->driver);
	spin_lock_irq(&ud->lock);
	ud->status = SDEV_ST_AVAILABLE;
	spin_unlock_irq(&ud->lock);
}

static void vudc_device_unusable(struct usbip_device *ud)
{
	spin_lock_irq(&ud->lock);
	ud->status = SDEV_ST_ERROR;
	spin_unlock_irq(&ud->lock);
}

/* device setup / cleanup */

struct vudc_device *alloc_vudc_device(int devid)
{
	struct vudc_device *udc_dev = NULL;

	udc_dev = kzalloc(sizeof(*udc_dev), GFP_KERNEL);
	if (!udc_dev)
		goto out;

	INIT_LIST_HEAD(&udc_dev->list);

	udc_dev->dev = platform_device_alloc(gadget_name, devid);
	if (!udc_dev->dev) {
		kfree(udc_dev);
		udc_dev = NULL;
	}

out:
	return udc_dev;
}

void put_vudc_device(struct vudc_device *udc_dev)
{
	platform_device_put(udc_dev->dev);
	kfree(udc_dev);
}

static int init_vudc_hw(struct vudc *vudc)
{
	int i;
	struct usbip_device *udev = &vudc->udev;

	vudc->dev_descr = kmalloc(sizeof(struct usb_device_descriptor), GFP_KERNEL);
	if (!vudc->dev_descr)
		goto nomem_descr;
	vudc->ep = kcalloc(VIRTUAL_ENDPOINTS, sizeof(*vudc->ep), GFP_KERNEL);
	if (!vudc->ep)
		goto nomem_ep;

	INIT_LIST_HEAD(&vudc->gadget.ep_list);
	for(i = 0; i < VIRTUAL_ENDPOINTS; ++i) {
		struct vep *ep = &vudc->ep[i];

		if(!ep_name[i])
			break;

		ep->ep.name = ep_name[i];
		ep->ep.ops = &vep_ops;
		list_add_tail(&ep->ep.ep_list, &vudc->gadget.ep_list);
		ep->halted = ep->wedged = ep->already_seen =
			ep->setup_stage = 0;
		usb_ep_set_maxpacket_limit(&ep->ep, ~0);
		ep->ep.max_streams = 16;
		ep->gadget = &vudc->gadget;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->queue);
	}


	spin_lock_init(&vudc->lock);
	spin_lock_init(&vudc->lock_tx);
	INIT_LIST_HEAD(&vudc->urb_q);
	INIT_LIST_HEAD(&vudc->priv_tx);
	init_waitqueue_head(&vudc->tx_waitq);

	spin_lock_init(&udev->lock);
	udev->status = SDEV_ST_AVAILABLE;
	udev->side = USBIP_VUDC;

	udev->eh_ops.shutdown = vudc_shutdown;
	udev->eh_ops.reset    = vudc_device_reset;
	udev->eh_ops.unusable = vudc_device_unusable;

	usbip_start_eh(udev);

	vudc->gadget.ep0 = &vudc->ep[0].ep;
	list_del_init(&vudc->ep[0].ep.ep_list);

	v_init_timer(vudc);
	return 0;

	nomem_ep:
		kfree(vudc->dev_descr);
	nomem_descr:
		return -ENOMEM;
}

static void cleanup_vudc_hw(struct vudc *vudc)
{
	usbip_event_add(&vudc->udev, SDEV_EVENT_REMOVED);
	usbip_stop_eh(&vudc->udev);
	kfree(vudc->dev_descr);
	kfree(vudc->ep);
}

/* platform driver ops */

static int vudc_probe(struct platform_device *pdev)
{
	struct vudc *vudc;
	int retval = -ENOMEM;

	vudc = kzalloc(sizeof(*vudc), GFP_KERNEL);
	if (!vudc)
		goto out;

	vudc->gadget.name = gadget_name;
	vudc->gadget.ops = &vgadget_ops;
	vudc->gadget.max_speed = USB_SPEED_HIGH;
	vudc->gadget.dev.parent = &pdev->dev;

	retval = init_vudc_hw(vudc);
	if (retval)
		goto err_init_vudc_hw;

	retval = usb_add_gadget_udc(&pdev->dev, &vudc->gadget);
	if (retval < 0)
		goto err_add_udc;

	retval = sysfs_create_group(&pdev->dev.kobj, &vudc_attr_group);
	if (retval) {
		pr_err("create sysfs files\n");
		goto err_add_udc;
	}

	platform_set_drvdata(pdev, vudc);

	return retval;

err_add_udc:
	cleanup_vudc_hw(vudc);
err_init_vudc_hw:
	kfree(vudc);
out:
	return retval;
}

static int vudc_remove(struct platform_device *pdev)
{
	struct vudc *vudc = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &vudc_attr_group);
	usb_del_gadget_udc(&vudc->gadget);
	cleanup_vudc_hw(vudc);
	kfree(vudc);
	return 0;
}

static int vudc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO - not yet implemented in USB/IP */
	return -ENOSYS;
}

static int vudc_resume(struct platform_device *pdev)
{
	/* TODO - not yet implemented in USB/IP */
	return -ENOSYS;
}

struct platform_driver vudc_driver = {
	.probe		= vudc_probe,
	.remove		= vudc_remove,
	.suspend	= vudc_suspend,
	.resume		= vudc_resume,
	.driver		= {
		.name	= gadget_name,
	},
};
