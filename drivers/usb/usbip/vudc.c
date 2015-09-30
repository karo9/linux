/*
 * vudc.c -- USB over IP UDC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/net.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/sysfs.h>
#include <linux/kthread.h>
#include <linux/file.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>

#include "usbip_common.h"
#include "stub.h"

#include <net/sock.h>

#define DRIVER_DESC "USB over IP UDC"
#define DRIVER_VERSION "06 March 2015"

#define US_MAX_DESCR_LENGTH 1024 * 4

#define DEBUG 1
#define debug_print(...) \
		do { if (DEBUG) printk(KERN_ERR __VA_ARGS__); } while (0)

static const char driver_desc[] = DRIVER_DESC;
static const char gadget_name[] = "usbip-vudc";

MODULE_DESCRIPTION(DRIVER_DESC);
/* Add your names here */
MODULE_AUTHOR("Krzysztof Opasiak, Karol Kosik");
MODULE_LICENSE("GPL");

struct vudc_module_parameters {
	int num;
};

static struct vudc_module_parameters mod_data = {
	.num = 1
};

module_param_named(num, mod_data.num, uint, S_IRUGO);
MODULE_PARM_DESC(num, "number of emulated controllers");

static const char ep0name[] = "ep0";

static const char *const ep_name[] = {
	ep0name,				/* everyone has ep0 */

	"ep1in-bulk", "ep2out-bulk", "ep3in-iso", "ep4out-iso", "ep5in-int",
	"ep6in-bulk", "ep7out-bulk", "ep8in-iso", "ep9out-iso", "ep10in-int",
	"ep11in-bulk", "ep12out-bulk", "ep13in-iso", "ep14out-iso", "ep15in-int",

	"ep1out-bulk", "ep2in-bulk",

	"ep3out", "ep4in", "ep5out", "ep6out", "ep7in", "ep8out", "ep9in",
	"ep10out", "ep11out", "ep12in", "ep13out", "ep14in", "ep15out",
};
#define VIRTUAL_ENDPOINTS	ARRAY_SIZE(ep_name)

/* container for usb_ep to store some endpoint related data */
struct vep {
	struct usb_ep ep;
	unsigned type:2;
	/* Add here some fields if needed */

	const struct usb_endpoint_descriptor *desc;
	struct usb_gadget *gadget;
	struct list_head queue; // Request queue
	unsigned halted:1;
	unsigned wedged:1;
	unsigned already_seen:1;
	unsigned setup_stage:1;
};

/* container for usb_request to store some request related data */
struct vrequest {
	struct usb_request req;
	/* Add here some fields if needed */

	struct vudc *sdev;

	struct list_head queue; // Request queue
};

struct urbp {
	struct urb *urb;
	struct vep *ep;
	struct list_head urb_q;
	unsigned long seqnum;
	unsigned new:1;
};

struct vudc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *dev;
	/* Add here some fields if needed */

	struct usb_device_descriptor *dev_descr;

	struct usbip_device udev;
	struct timer_list tr_timer;

	struct list_head urb_q;

	spinlock_t lock_tx;
	struct list_head priv_tx;
	struct list_head unlink_tx;
	wait_queue_head_t tx_waitq;

	spinlock_t lock; //Proctect data
	struct vep ep[VIRTUAL_ENDPOINTS]; //VUDC enpoints
	int address; //VUDC address
	u16 devstatus;
};

/* suitable transator forom usb structures to our private one */
static inline struct vep *usb_ep_to_vep(struct usb_ep *_ep)
{
	return container_of(_ep, struct vep, ep);
}

static inline struct vrequest *usb_request_to_vrequest(
	struct usb_request *_req)
{
	return container_of(_req, struct vrequest, req);
}

static inline struct vudc *usb_gadget_to_vudc(
	struct usb_gadget *_gadget)
{
	return container_of(_gadget, struct vudc, gadget);
}

static inline struct vudc *ep_to_vudc(struct vep *ep)
{
	return container_of(ep->gadget, struct vudc, gadget);
}

static int alloc_urb_from_cmd(struct urb **urbp, struct usbip_header *pdu)
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

	/* FIXME - we only setup pipe enough for usbip functions
	 * to behave nicely */
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

static struct urbp* alloc_urbp(void)
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
	kfree(urb_p);
}

/* sysfs files */

static ssize_t usbip_status_show(struct device *dev,
			       struct device_attribute *attr, char *out)
{
	struct vudc *sdev = (struct vudc *) dev_get_drvdata(dev);
	int status;
	
	if (!sdev)
		return -ENODEV;
	spin_lock_irq(&sdev->udev.lock);
	status = sdev->udev.status;
	spin_unlock_irq(&sdev->udev.lock);

	return snprintf(out, PAGE_SIZE, "%d\n", status);
}
static DEVICE_ATTR_RO(usbip_status);

static ssize_t fetch_descriptor(struct usb_ctrlrequest* req, struct vudc* udc,
				char *out, ssize_t sz, ssize_t maxsz)
{
	struct vrequest *usb_req;
	int ret;
	int copysz;
	struct vep *ep0 = usb_ep_to_vep(udc->gadget.ep0);

	if (!udc->driver)	/* No device for export */
		return 0;
	ret = udc->driver->setup(&(udc->gadget), req);
	if (ret < 0) {
		debug_print("[vudc] Failed to setup device descriptor request!\n");
		goto exit;
	}

	/* FIXME: assuming request queue is empty; request is now on top */
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

static int descriptor_cache(struct vudc *sdev)
{
	struct usb_device_descriptor *dev_d = sdev->dev_descr;
	struct usb_ctrlrequest req;
	int ret;
	int sz = 0;
	int max_sz = US_MAX_DESCR_LENGTH;

	if (!sdev || !sdev->driver)
		return -1;

	req.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = cpu_to_le16(USB_DT_DEVICE << 8);
	req.wIndex = cpu_to_le16(0);
	req.wLength = cpu_to_le16(max_sz - sz);

	ret = fetch_descriptor(&req, sdev, (char *) dev_d, max_sz, max_sz);
	if (ret < 0) {
		debug_print("[vudc] Could not fetch device descriptor!\n");
		return -1;
	}
	sz += ret;

#if 0
	sz = 0;
	req.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = cpu_to_le16(USB_DT_CONFIG << 8);
	req.wIndex = cpu_to_le16(0);
	req.wLength = cpu_to_le16(max_sz - sz);

	for (i = 0; i < dev_desc->bNumConfigurations ; i++) {
		req.wValue = cpu_to_le16((USB_DT_CONFIG << 8) | i);
		ret = fetch_descriptor(&req, sdev, intf_d + sz,
				       sizeof(struct usb_config_descriptor),
				       max_sz - sz);
		if (ret < 0)
			goto err_intf;
		ret = extract_intfs(intf_d + sz, ret);
		if (ret < 0)
			goto err_intf;
	}
#endif
	return 0;
#if 0
err_intf:
	debug_print("[vudc] Could not fetch interface descriptor!\n");
	return -1;
#endif
}


/* ************************************************************************************************************ */

/*utilities ; alomst verbatim from dummy_hcd.c */

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
		free_urbp(urb_p);
	}
}

/* Adapted from dummy_hcd.c ; caller must hold lock */
static void transfer(struct vudc* sdev,
		struct urb *urb, struct vep *ep)
{
	struct vrequest	*req;

top:
	/* if there's no request queued, the device is NAKing; return */
	list_for_each_entry(req, &ep->queue, queue) {
		unsigned	host_len, dev_len, len;
		void		*ubuf_pos, *rbuf_pos;
		int		is_short, to_host;
		int		rescan = 0;

		/* 1..N packets of ep->ep.maxpacket each ... the last one
		 * may be short (including zero length).
		 *
		 * writer can send a zlp explicitly (length 0) or implicitly
		 * (length mod maxpacket zero, and 'zero' flag); they always
		 * terminate reads.
		 */
		host_len = urb->transfer_buffer_length - urb->actual_length;
		dev_len = req->req.length - req->req.actual;
		len = min(host_len, dev_len);

		/* FIXME update emulated data toggle too */

		to_host = usb_pipein(urb->pipe);
		if (unlikely(len == 0))
			is_short = 1;
		else {
			/* use an extra pass for the final short packet */
			if (len > ep->ep.maxpacket) {
				rescan = 1;
				len -= (len % ep->ep.maxpacket);
			}
			is_short = (len % ep->ep.maxpacket) != 0;

			ubuf_pos = urb->transfer_buffer + urb->actual_length;
			rbuf_pos = req->req.buf + req->req.actual;

			if(urb->pipe & USB_DIR_IN)
				memcpy(ubuf_pos, rbuf_pos, len);
			else
				memcpy(rbuf_pos, ubuf_pos, len);

			urb->actual_length += len;
			req->req.actual += len;
		}

		/* short packets terminate, maybe with overflow/underflow.
		 * it's only really an error to write too much.
		 *
		 * partially filling a buffer optionally blocks queue advances
		 * (so completion handlers can clean up the queue) but we don't
		 * need to emulate such data-in-flight.
		 */
		if (is_short) {
			if (host_len == dev_len) {
				req->req.status = 0;
				urb->status = 0;
			} else if (to_host) {
				req->req.status = 0;
				if (dev_len > host_len)
					urb->status = -EOVERFLOW;
				else
					urb->status = 0;
			} else if (!to_host) {
				urb->status = 0;
				if (host_len > dev_len)
					req->req.status = -EOVERFLOW;
				else
					req->req.status = 0;
			}

		/* many requests terminate without a short packet */
		} else {
			if (req->req.length == req->req.actual
					&& !req->req.zero)
				req->req.status = 0;
			if (urb->transfer_buffer_length == urb->actual_length
					&& !(urb->transfer_flags
						& URB_ZERO_PACKET))
				urb->status = 0;
		}

		/* device side completion --> continuable */
		if (req->req.status != -EINPROGRESS) {

	    //printk(KERN_ERR "transfer - device side completion\n");

			list_del_init(&req->queue);
			spin_unlock(&sdev->lock);
			usb_gadget_giveback_request(&ep->ep, &req->req);
			spin_lock(&sdev->lock);

			/* requests might have been unlinked... */
			rescan = 1;
		}

		/* host side completion --> terminate */
		if (urb->status != -EINPROGRESS)
			break;

		/* rescan to continue with any other queued i/o */
		if (rescan)
			goto top;
	}
}

static inline void setup_base_pdu(struct usbip_header_basic *base,
				  __u32 command, __u32 seqnum)
{
	base->command	= command;
	base->seqnum	= seqnum;
	base->devid	= 0;
	base->ep	= 0;
	base->direction = 0;
}

static void setup_ret_submit_pdu(struct usbip_header *rpdu, struct urbp *urb_p)
{
	setup_base_pdu(&rpdu->base, USBIP_RET_SUBMIT, urb_p->seqnum);
	usbip_pack_pdu(rpdu, urb_p->urb, USBIP_RET_SUBMIT, 1);
}

static void setup_ret_unlink_pdu(struct usbip_header *rpdu,
				 struct stub_unlink *unlink)
{
	setup_base_pdu(&rpdu->base, USBIP_RET_UNLINK, unlink->seqnum);
	rpdu->u.ret_unlink.status = unlink->status;
}

static void send_respond(struct urbp *urb_p, struct vudc *sdev)
{
		struct urb* urb = urb_p->urb;
		struct usbip_header pdu_header;
		struct usbip_iso_packet_descriptor *iso_buffer = NULL;
		struct kvec *iov = NULL;
		int iovnum = 0;
		int ret;
		size_t txsize;
		struct msghdr msg;

		txsize = 0;
		memset(&pdu_header, 0, sizeof(pdu_header));
		memset(&msg, 0, sizeof(msg));

		if (urb_p->ep->type == USB_ENDPOINT_XFER_ISOC)
			iovnum = 2 + urb->number_of_packets;
		else
			iovnum = 2;

		iov = kcalloc(iovnum, sizeof(struct kvec), GFP_KERNEL);

		if (!iov) {
			usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_MALLOC);
			return;
		}
		iovnum = 0;
		setup_ret_submit_pdu(&pdu_header, urb_p);
		usbip_dbg_stub_tx("setup txdata seqnum: %d urb: %p\n",
				  pdu_header.base.seqnum, urb);
		usbip_header_correct_endian(&pdu_header, 1);
		iov[iovnum].iov_base = &pdu_header;
		iov[iovnum].iov_len  = sizeof(pdu_header);
		iovnum++;
		txsize += sizeof(pdu_header);

		if (usb_pipein(urb->pipe) && urb->actual_length > 0) {
			iov[iovnum].iov_base = urb->transfer_buffer;
			iov[iovnum].iov_len  = urb->actual_length;
			iovnum++;
			txsize += urb->actual_length;
		}

		ret = kernel_sendmsg(sdev->udev.tcp_socket, &msg,
						iov,  iovnum, txsize);
		if (ret != txsize) {
			kfree(iov);
			kfree(iso_buffer);
			usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_TCP);
		}

		kfree(iov);
		kfree(iso_buffer);
		free_urb(urb);
		free_urbp(urb_p);
}

/* some members of urb must be substituted before. */
int usbip_recv_xbuff(struct usbip_device *ud, struct urb *urb)
{
	int ret;
	int size;


	printk(KERN_ERR "[xbuff] uno");
	if (urb->pipe & USB_DIR_IN)
		return 0;

	size = urb->transfer_buffer_length;

	printk(KERN_ERR "[xbuff] dos");
	/* no need to recv xbuff */
	if (!(size > 0))
		return 0;

	printk(KERN_ERR "Odbieram dodatkowy transfer_buffer - po stronie vudc");
	ret = usbip_recv(ud->tcp_socket, urb->transfer_buffer, size);
	if (ret != size)
			return -EPIPE;

	return ret;
}

static struct vep *find_endpoint(struct vudc *vudc, u8 address)
{
	int i;

	if ((address & ~USB_DIR_IN) == 0)
	{
		printk(KERN_ERR "[find_endpoint] Zwracam 0");
		return &vudc->ep[0];
	}

	for (i = 1; i < VIRTUAL_ENDPOINTS; i++) {
		struct vep *ep = &vudc->ep[i];

		if (!ep->desc)
			continue;
		if (ep->desc->bEndpointAddress == address)
		{
			printk(KERN_ERR "[find_endpoint] Zwracam %d", i);
			return ep;
		}
	}
	return NULL;
}

#define Dev_Request	(USB_TYPE_STANDARD | USB_RECIP_DEVICE)
#define Dev_InRequest	(Dev_Request | USB_DIR_IN)
#define Intf_Request	(USB_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define Intf_InRequest	(Intf_Request | USB_DIR_IN)
#define Ep_Request	(USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)
#define Ep_InRequest	(Ep_Request | USB_DIR_IN)

/**
 * handle_control_request() - handles all control transfers
 * @sdev: pointer to vudc
 * @urb: the urb request to handle
 * @setup: pointer to the setup data for a USB device control
 *	 request
 * @status: pointer to request handling status
 *
 * Return 0 - if the request was handled
 *	  1 - if the request wasn't handles
 *	  error code on error
 *
 * Adapted from drivers/usb/gadget/udc/dummy_hcd.c
 */
static int handle_control_request(struct vudc *sdev, struct urb *urb,
				  struct usb_ctrlrequest *setup,
				  int *status)
{
	struct vep		*ep2;
	int			ret_val = 1;
	unsigned	w_index;
	unsigned	w_value;

	w_index = le16_to_cpu(setup->wIndex);
	w_value = le16_to_cpu(setup->wValue);
	switch (setup->bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (setup->bRequestType != Dev_Request)
			break;
		sdev->address = w_value;
		ret_val = 0;
		*status = 0;
		break;
	case USB_REQ_SET_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				break;
			case USB_DEVICE_B_HNP_ENABLE:
				sdev->gadget.b_hnp_enable = 1;
				break;
			case USB_DEVICE_A_HNP_SUPPORT:
				sdev->gadget.a_hnp_support = 1;
				break;
			case USB_DEVICE_A_ALT_HNP_SUPPORT:
				sdev->gadget.a_alt_hnp_support = 1;
				break;
			//TODO - usb 3.0 support
			case USB_DEVICE_U1_ENABLE:
			case USB_DEVICE_U2_ENABLE:
			case USB_DEVICE_LTM_ENABLE:
				ret_val = -EOPNOTSUPP;
				break;
			default:
				ret_val = -EOPNOTSUPP;
			}
			if (ret_val == 0) {
				sdev->devstatus |= (1 << w_value);
				*status = 0;
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(sdev, w_index);
			if (!ep2 || ep2->ep.name == ep0name) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			ep2->halted = 1;
			ret_val = 0;
			*status = 0;
		}
		break;
	case USB_REQ_CLEAR_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				w_value = USB_DEVICE_REMOTE_WAKEUP;
				break;

			case USB_DEVICE_U1_ENABLE:
			case USB_DEVICE_U2_ENABLE:
			case USB_DEVICE_LTM_ENABLE:
				ret_val = -EOPNOTSUPP;
				break;
			default:
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (ret_val == 0) {
				sdev->devstatus &= ~(1 << w_value);
				*status = 0;
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(sdev, w_index);
			if (!ep2) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (!ep2->wedged)
				ep2->halted = 0;
			ret_val = 0;
			*status = 0;
		}
		break;
	case USB_REQ_GET_STATUS:
		if (setup->bRequestType == Dev_InRequest
				|| setup->bRequestType == Intf_InRequest
				|| setup->bRequestType == Ep_InRequest) {
			char *buf;
			/*
			 * device: remote wakeup, selfpowered
			 * interface: nothing
			 * endpoint: halt
			 */
			buf = (char *)urb->transfer_buffer;
			if (urb->transfer_buffer_length > 0) {
				if (setup->bRequestType == Ep_InRequest) {
					ep2 = find_endpoint(sdev, w_index);
					if (!ep2) {
						ret_val = -EOPNOTSUPP;
						break;
					}
					buf[0] = ep2->halted;
				} else if (setup->bRequestType ==
					   Dev_InRequest) {
					buf[0] = (u8)sdev->devstatus;
				} else
					buf[0] = 0;
			}
			if (urb->transfer_buffer_length > 1)
				buf[1] = 0;
			urb->actual_length = min_t(u32, 2,
				urb->transfer_buffer_length);
			ret_val = 0;
			*status = 0;
		}
		break;
	}
	return ret_val;
}

static void v_enqueue_ret_unlink(struct vudc *sdev, __u32 seqnum,
			     __u32 status)
{
	struct stub_unlink *unlink;

	unlink = kzalloc(sizeof(struct stub_unlink), GFP_ATOMIC);
	if (!unlink) {
		usbip_event_add(&sdev->udev, VDEV_EVENT_ERROR_MALLOC);
		return;
	}

	unlink->seqnum = seqnum;
	unlink->status = status;

	list_add_tail(&unlink->list, &sdev->unlink_tx);
}

static void v_timer(unsigned long _vudc)
{
	struct vudc *sdev = (struct vudc *) _vudc;
	struct urbp *urb_p, *tmp;
	unsigned long flags;
	int i, ret = 0;

	spin_lock_irqsave(&sdev->lock, flags);

	for (i = 0; i < VIRTUAL_ENDPOINTS; i++) {
		if (!ep_name[i])
			break;
		sdev->ep[i].already_seen = 0;
	}

	list_for_each_entry_safe(urb_p, tmp, &sdev->urb_q, urb_q) {
		struct urb *urb = urb_p->urb;
		struct vep *ep = urb_p->ep;
	if (!ep) {
		urb->status = -EPROTO;
		goto return_urb;
	}

	if (ep->already_seen)
		continue;
	ep->already_seen = 1;
	if (ep == &sdev->ep[0] && urb_p->new) {
		ep->setup_stage = 1;
		urb_p->new = 0;
	}
	if (ep->halted && !ep->setup_stage) {
		urb->status = -EPIPE;
		goto return_urb;
	}

	if (ep == &sdev->ep[0] && ep->setup_stage) {
		/* TODO - flush any stale requests */
		ep->setup_stage = 0;
		ep->halted = 0;

		ret = handle_control_request(sdev, urb,
			(struct usb_ctrlrequest *) urb->setup_packet, (&urb->status));
		if (ret > 0) {
			spin_unlock(&sdev->lock);
			ret = sdev->driver->setup(&sdev->gadget,
				(struct usb_ctrlrequest *) urb->setup_packet);
			spin_lock(&sdev->lock);
		}
		if (ret >= 0) {
			/* TODO - when different types are coded in, treat like bulk */
		} else {
			urb->status = -EPIPE;
			urb->actual_length = 0;
			goto return_urb;
		}
	}

		printk(KERN_ERR "Przed make transfer\n");
		transfer(sdev, urb, ep);

		/* FIXME - what does dummy_hcd actually do here? */
		if (urb->status == -EINPROGRESS)
			continue;

	return_urb:
		printk(KERN_ERR "Przed respond\n");
		if (ep)
			ep->already_seen = ep->setup_stage = 0;

		spin_lock(&sdev->lock_tx);
		if (!urb->unlinked)
			list_move_tail(&urb_p->urb_q, &sdev->priv_tx);
		else
			v_enqueue_ret_unlink(sdev, urb_p->seqnum, urb->unlinked);
		spin_unlock(&sdev->lock_tx);
		wake_up(&sdev->tx_waitq);
	}
	spin_unlock_irqrestore(&sdev->lock, flags);
}

static void stub_recv_cmd_unlink(struct vudc *sdev,
				struct usbip_header *pdu)
{
	unsigned long flags;
	struct urbp* urb_p;

	spin_lock_irqsave(&sdev->lock, flags);
	list_for_each_entry(urb_p, &sdev->urb_q, urb_q) {
		if (urb_p->seqnum != pdu->u.cmd_unlink.seqnum)
			continue;
		urb_p->urb->unlinked = -ECONNRESET;
		/* kick the timer */
		mod_timer(&sdev->tr_timer, jiffies);
		spin_unlock_irqrestore(&sdev->lock, flags);
		return;
	}
	/* Not found, completed / not queued */
	spin_lock(&sdev->lock_tx);
	v_enqueue_ret_unlink(sdev, pdu->base.seqnum, 0);
	spin_unlock(&sdev->lock_tx);
	spin_unlock_irqrestore(&sdev->lock, flags);
}

static void stub_recv_cmd_submit(struct vudc *sdev,
				 struct usbip_header *pdu)
{
	int ret;
	struct urbp* urb_p = alloc_urbp();
	u8 address;
	unsigned long flags;

	if (!urb_p) {
		usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_MALLOC);
		return;
	}

	/* base.ep is pipeendpoint(pipe) */
	address = pdu->base.ep;
	if (pdu->base.direction == USBIP_DIR_IN)
		address |= USB_DIR_IN;

	urb_p->ep = find_endpoint(sdev, address);
	urb_p->new = 1;
	urb_p->seqnum = pdu->base.seqnum;

	ret = alloc_urb_from_cmd(&urb_p->urb, pdu);
	if (ret) {
		usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_MALLOC);
		return;
	}

	urb_p->urb->status = -EINPROGRESS;
	ret = usbip_recv_xbuff(&sdev->udev, urb_p->urb);
	if (ret < 0)
		usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_TCP);

	usbip_dump_header(pdu);
	usbip_dump_urb(urb_p->urb);

	printk(KERN_ERR "Przed setup\n");

	spin_lock_irqsave(&sdev->lock, flags);
	if (!timer_pending(&sdev->tr_timer))
		mod_timer(&sdev->tr_timer, jiffies + 1);
	list_add_tail(&urb_p->urb_q, &sdev->urb_q);
	spin_unlock_irqrestore(&sdev->lock, flags);

	usbip_dbg_stub_rx("Leave\n");
}

static void stub_rx_pdu(struct usbip_device *ud)
{
	int ret;
	struct usbip_header pdu;
	struct vudc *sdev = container_of(ud, struct vudc, udev);

	usbip_dbg_stub_rx("Enter\n");

	memset(&pdu, 0, sizeof(pdu));
	ret = usbip_recv(ud->tcp_socket, &pdu, sizeof(pdu));
	if(ret != sizeof(pdu)) {
		usbip_event_add(ud, SDEV_EVENT_ERROR_TCP);
		return;
	}
	usbip_header_correct_endian(&pdu, 0);

	spin_lock_irq(&ud->lock);
	ret = (ud->status == SDEV_ST_USED);
	spin_unlock_irq(&ud->lock);
	if (!ret) {
		usbip_event_add(ud, SDEV_EVENT_ERROR_TCP);
		return;
	}

	switch (pdu.base.command) {
	case USBIP_CMD_UNLINK:
		stub_recv_cmd_unlink(sdev, &pdu);
		break;

	case USBIP_CMD_SUBMIT:
		stub_recv_cmd_submit(sdev, &pdu);
		break;

	default:
		printk(KERN_ERR "UNKOWN PDU\n");
		//dev_err(dev, "unknown pdu\n");
		break;
	}
	return;
}

int stub_rx_loop(void *data)
{
	struct usbip_device *ud = data;

	while (!kthread_should_stop()) {
		if (usbip_event_happened(ud))
			break;
		stub_rx_pdu(ud);
	}

	return 0;
}








/* ************************************************************************************************************ */

int v_send_ret_unlink(struct vudc * sdev)
{
	unsigned long flags;
	struct stub_unlink *unlink;

	struct msghdr msg;
	struct kvec iov[1];
	size_t txsize;

	size_t total_size = 0;

	spin_lock_irqsave(&sdev->lock_tx, flags);

	while (!list_empty(&sdev->unlink_tx)) {
		int ret;
		struct usbip_header pdu_header;
		unlink = list_entry(sdev->unlink_tx.next, struct stub_unlink, list);
		spin_unlock_irqrestore(&sdev->lock_tx, flags);

		txsize = 0;
		memset(&pdu_header, 0, sizeof(pdu_header));
		memset(&msg, 0, sizeof(msg));
		memset(&iov, 0, sizeof(iov));

		/* 1. setup usbip_header */
		setup_ret_unlink_pdu(&pdu_header, unlink);
		usbip_header_correct_endian(&pdu_header, 1);

		iov[0].iov_base = &pdu_header;
		iov[0].iov_len  = sizeof(pdu_header);
		txsize += sizeof(pdu_header);

		ret = kernel_sendmsg(sdev->udev.tcp_socket, &msg, iov,
				     1, txsize);
		if (ret != txsize) {
			usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_TCP);
			return -1;
		}

		total_size += txsize;
		spin_lock_irqsave(&sdev->lock_tx, flags);
	}

	spin_unlock_irqrestore(&sdev->lock_tx, flags);
	return total_size;
}

int v_send_ret_submit(struct vudc * sdev)
{
	unsigned long flags;
	spin_lock_irqsave(&sdev->lock_tx, flags);

	while(!list_empty(&sdev->priv_tx)) {
		struct list_head *urbh = sdev->priv_tx.next;
		struct urbp *urb_p = list_entry(urbh, struct urbp, urb_q);
		list_del_init(urbh);
		spin_unlock_irqrestore(&sdev->lock_tx, flags);

		send_respond(urb_p, sdev);
		spin_lock_irqsave(&sdev->lock_tx, flags);
	}
	spin_unlock_irqrestore(&sdev->lock_tx, flags);
	return 0;
}

int stub_tx_loop(void *data)
{
	struct usbip_device * udev = (struct usbip_device *) data;
	struct vudc * sdev = container_of(udev, struct vudc, udev);

	debug_print("[vudc] *** stub_tx_loop ***\n");

	while (!kthread_should_stop()) {
		if (usbip_event_happened(&sdev->udev))
			break;
		if (v_send_ret_submit(sdev) < 0)
			break;
		if (v_send_ret_unlink(sdev) < 0)
			break;

		wait_event_interruptible(sdev->tx_waitq,
						(!list_empty(&sdev->priv_tx) ||
						kthread_should_stop()));
	}

	debug_print("[vudc] ### stub_tx_loop ###\n");

	return 0;
}

static ssize_t store_sockfd(struct device *dev, struct device_attribute *attr,
		     const char *in, size_t count)
{
	int rv;
	int sockfd = 0;
	struct vudc *vudc;
	int err;
	struct socket *socket;

	debug_print("[vudc] *** example_out ***\n");

	vudc = (struct vudc*)dev_get_drvdata(dev);
	if (!vudc || !vudc->driver) /* Don't export what we don't have */
		return -ENODEV;

	rv = sscanf(in, "%d", &sockfd);
	if (rv != 1)
		return -EINVAL;

	if (sockfd != 1) {
		spin_lock_irq(&vudc->udev.lock);

		if (vudc->udev.status != SDEV_ST_AVAILABLE) {
			goto err;
		}

		socket = sockfd_lookup(sockfd, &err);
		if (!socket) {
			debug_print("[vudc] Failed to lookup sock");
			goto err;
		}

		vudc->udev.tcp_socket = socket;

		spin_unlock_irq(&vudc->udev.lock);

		vudc->udev.tcp_rx = kthread_get_run(&stub_rx_loop, &vudc->udev, "vudc_rx");
		vudc->udev.tcp_tx = kthread_get_run(&stub_tx_loop, &vudc->udev, "vudc_tx");

		spin_lock_irq(&vudc->udev.lock);
		vudc->udev.status = SDEV_ST_USED;
		spin_unlock_irq(&vudc->udev.lock);

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

/* endpoint related operations */

static int vep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct vep *ep;
	struct vudc *sdev;
	int retval;
	unsigned maxp;

	debug_print("[vudc] *** enable ***\n");

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

	debug_print("[vudc] *** disable ***\n");

	ep = usb_ep_to_vep(_ep);
	if (!_ep || !ep->desc || _ep->name == ep0name)
		return -EINVAL;
	sdev = ep_to_vudc(ep);

	spin_lock_irqsave(&sdev->lock, flags);
	ep->desc = NULL;
	nuke(sdev, ep);
	spin_unlock_irqrestore(&sdev->lock, flags);

	debug_print("[vudc] ### disable ###\n");
	return 0;
}

static struct usb_request *vep_alloc_request(struct usb_ep *_ep,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;

	debug_print("[vudc] *** vep_alloc_request ***\n");

	if (!_ep)
		return NULL;
	ep = usb_ep_to_vep(_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;

	/* Do some additional request initialization here */

	INIT_LIST_HEAD(&req->queue);

	debug_print("[vudc] ### vep_alloc_request ###\n");

	return &req->req;
}

static void vep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct vrequest *req;

	debug_print("[vudc] *** vep_free_request ***\n");

	if (!_ep || !_req) {
		WARN_ON(1);
		return;
	}

	req = usb_request_to_vrequest(_req);

	/* Do some cleanup here if desired*/

	kfree(req);

	debug_print("[vudc] ### vep_free_request ###\n");
}

static int vep_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags)
{
	struct vep *ep;
	struct vrequest *req;
	struct vudc *vudc;
	unsigned long flags;

	debug_print("[vudc] *** vep_queue ***\n");
	debug_print("[vudc] Endpoint name: %s\n", _ep->name);

	if (!_ep || !_req)
		return -EINVAL;

	ep = usb_ep_to_vep(_ep);
	req = usb_request_to_vrequest(_req);
	vudc = ep_to_vudc(ep);

	_req->actual = 0;
	_req->status = -EINPROGRESS;

	debug_print("[vudc] actual length: %d length: %d\n", _req->actual, _req->length);
	print_hex_dump(KERN_DEBUG, "vep_queue", DUMP_PREFIX_OFFSET, 16, 4, _req->buf, _req->length, false);

	/* TODO */
	spin_lock_irqsave(&vudc->lock, flags);

	list_add_tail(&req->queue, &ep->queue);

	spin_unlock_irqrestore(&vudc->lock, flags);

	debug_print("[vudc] ### vep_queue ###\n");

	//return -EINVAL;
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

	debug_print("[vudc] *** vep_dequeue ***\n");

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

	if (retval == 0) {
		usb_gadget_giveback_request(_ep, _req);
	}

	debug_print("[vudc] ### vep_dequeue ###\n");
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
	debug_print("[vudc] *** vep_set_halt ***\n");
	return vep_set_halt_and_wedge(_ep, value, 0);
	debug_print("[vudc] ### vep_set_halt ###\n");
}

static int vep_set_wedge(struct usb_ep *_ep)
{
	debug_print("[vudc] *** vep_set_wedge ***\n");
	return vep_set_halt_and_wedge(_ep, 1, 1);
	debug_print("[vudc] ### vep_set_wedge ###\n");
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

/* gadget related functions */

static int vgadget_get_frame(struct usb_gadget *_gadget)
{
	debug_print("[vudc] *** vgadget_get_frame ***\n");
	/* TODO */
	debug_print("[vudc] ### vgadget_get_frame ###\n");
	return 0;
}

static int vgadget_wakeup(struct usb_gadget *_gadget)
{
	debug_print("[vudc] *** vgadget_wakeup ***\n");
	/* TODO */
	debug_print("[vudc] ### vgadget_wakeup ###\n");
	return 0;
}

static int vgadget_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	debug_print("[vudc] *** vgadget_set_selfpowered ***\n");
	/* TODO */
	debug_print("[vudc] ### vgadget_set_selfpowered ###\n");
	return 0;
}

static int vgadget_pullup(struct usb_gadget *_gadget, int value)
{
	struct vudc *vudc = usb_gadget_to_vudc(_gadget);
	debug_print("[vudc] *** vgadget_pullup ***\n");

	if (value && vudc->driver) {
		//vudc->gadget.speed = vudc->driver->max_speed;
		vudc->gadget.speed = USB_SPEED_HIGH;

		if(vudc->gadget.speed == USB_SPEED_SUPER)
			vudc->ep[0].ep.maxpacket = 9;
		else
			vudc->ep[0].ep.maxpacket = 64;
	}
	/* TODO */
	debug_print("[vudc] ### vgadget_pullup ###\n");
	return 0;
}

static int vgadget_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct vudc *vudc = usb_gadget_to_vudc(g);
	unsigned long flags;
	int ret;

	debug_print("[vudc] *** vgadget_udc_start ***\n");

	spin_lock_irqsave(&vudc->lock, flags);
	vudc->driver = driver;
	spin_unlock_irqrestore(&vudc->lock, flags);
	ret = descriptor_cache(vudc);
	if (ret) {
		/* FIXME: add correct event */
		usbip_event_add(&vudc->udev, SDEV_EVENT_ERROR_MALLOC);
	}

	/* TODO */
	debug_print("[vudc] ### vgadget_udc_start ###\n");
	return 0;
}

static int vgadget_udc_stop(struct usb_gadget *g)
{
	struct vudc *vudc = usb_gadget_to_vudc(g);
	unsigned long flags;

	debug_print("[vudc] *** vgadget_udc_stop ***\n");
	usbip_event_add(&vudc->udev, SDEV_EVENT_REMOVED);
	usbip_stop_eh(&vudc->udev); /* Wait for eh completion */
	usbip_start_eh(&vudc->udev);

	spin_lock_irqsave(&vudc->lock, flags);
	vudc->driver = NULL;
	spin_unlock_irqrestore(&vudc->lock, flags);
	/* TODO */
	debug_print("[vudc] ### vgadget_udc_stop ###\n");
	return 0;
}

static const struct usb_gadget_ops vgadget_ops = {
	.get_frame	= vgadget_get_frame,
	.wakeup		= vgadget_wakeup,
	.set_selfpowered = vgadget_set_selfpowered,
	.pullup		= vgadget_pullup,
	.udc_start	= vgadget_udc_start,
	.udc_stop	= vgadget_udc_stop,
};

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

static int init_vudc_hw(struct vudc *vudc)
{
	int i;
	struct usbip_device *udev = &vudc->udev;

	debug_print("[vudc] *** init_vudc_hw ***\n");

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

	vudc->dev_descr = kmalloc(sizeof(struct usb_device_descriptor), GFP_KERNEL);
	if (!vudc->dev_descr)
		return -ENOMEM;

	spin_lock_init(&vudc->lock);
	spin_lock_init(&vudc->lock_tx);
	INIT_LIST_HEAD(&vudc->urb_q);
	INIT_LIST_HEAD(&vudc->priv_tx);
	INIT_LIST_HEAD(&vudc->unlink_tx);
	init_waitqueue_head(&vudc->tx_waitq);

	spin_lock_init(&udev->lock);
	udev->status = SDEV_ST_AVAILABLE;
	udev->side = USBIP_VUDC; /* FIXME - add to common*/

	udev->eh_ops.shutdown = vudc_shutdown;
	udev->eh_ops.reset    = vudc_device_reset;
	udev->eh_ops.unusable = vudc_device_unusable;

	usbip_start_eh(udev);

	vudc->gadget.ep0 = &vudc->ep[0].ep;
	list_del_init(&vudc->ep[0].ep.ep_list);

	setup_timer(&vudc->tr_timer, v_timer, (unsigned long) vudc);
	/* TODO */
	debug_print("[vudc] ### init_vudc_hw ###\n");
	return 0;
}

static void cleanup_vudc_hw(struct vudc *vudc)
{
	debug_print("[vudc] *** cleanup_vudc_hw ***\n");

	kfree(vudc->dev_descr);
	usbip_event_add(&vudc->udev, SDEV_EVENT_REMOVED);
	usbip_stop_eh(&vudc->udev);

	debug_print("[vudc] ### cleanup_vudc_hw ###\n");
	return;
}

static int vudc_probe(struct platform_device *pdev)
{
	struct vudc *vudc;
	int retval = -ENOMEM;

	debug_print("[vudc] *** vudc_probe ***\n");

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

	device_create_file(&pdev->dev, &dev_attr_usbip_status);
	device_create_file(&pdev->dev, &dev_attr_dev_descr);
	device_create_file(&pdev->dev, &dev_attr_usbip_sockfd);

	platform_set_drvdata(pdev, vudc);

	debug_print("[vudc] ### vudc_probe ###\n");

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

	debug_print("[vudc] *** vudc_remove ***\n");

	device_remove_file(&pdev->dev, &dev_attr_usbip_status);
	device_remove_file(&pdev->dev, &dev_attr_dev_descr);
	device_remove_file(&pdev->dev, &dev_attr_usbip_sockfd);

	usb_del_gadget_udc(&vudc->gadget);
	cleanup_vudc_hw(vudc);
	kfree(vudc);

	debug_print("[vudc] ### vudc_remove ###\n");

	return 0;
}

static int vudc_suspend(struct platform_device *pdev, pm_message_t state)
{
	debug_print("[vudc] *** vudc_suspend ***\n");
	/* TODO - not yet implemented in USB/IP */
	debug_print("[vudc] ### vudc_suspend ###\n");
	return 0;
}

static int vudc_resume(struct platform_device *pdev)
{
	debug_print("[vudc] *** vudc_resume ***\n");
	/* TODO - not yet implemented in USB/IP */
	debug_print("[vudc] ### vudc_resume ###\n");
	return 0;
}

static struct platform_driver vudc_driver = {
	.probe		= vudc_probe,
	.remove		= vudc_remove,
	.suspend	= vudc_suspend,
	.resume		= vudc_resume,
	.driver		= {
		.name	= gadget_name,
	},
};

struct vudc_device {
	struct platform_device *dev;
	struct list_head list;
};

static struct vudc_device *alloc_vudc_device(
	const char *dev_name, int devid)
{
	struct vudc_device *udc_dev = NULL;

	debug_print("[vudc] *** alloc_vudc_device ***\n");

	udc_dev = kzalloc(sizeof(*udc_dev), GFP_KERNEL);
	if (!udc_dev)
		goto out;

	INIT_LIST_HEAD(&udc_dev->list);

	udc_dev->dev = platform_device_alloc(dev_name, devid);
	if (!udc_dev->dev) {
		kfree(udc_dev);
		udc_dev = NULL;
	}

out:
	debug_print("[vudc] ### alloc_vudc_device ###\n");
	return udc_dev;
}

static void put_vudc_device(struct vudc_device *udc_dev)
{
	debug_print("[vudc] *** put_vudc_device ***\n");

	platform_device_put(udc_dev->dev);
	kfree(udc_dev);

	debug_print("[vudc] ### put_vudc_device ###\n");
}

static struct list_head vudc_devices = LIST_HEAD_INIT(vudc_devices);

static int __init init(void)
{
	int retval = -ENOMEM;
	int i;
	struct vudc_device *udc_dev = NULL, *udc_dev2 = NULL;

	debug_print("[vudc] *** init ***\n");

	if (usb_disabled())
		return -ENODEV;

	if (mod_data.num < 1) {
		pr_err("Number of emulated UDC must be greater than 1");
		return -EINVAL;
	}

	for (i = 0; i < mod_data.num; i++) {
		udc_dev = alloc_vudc_device(gadget_name, i);
		if (!udc_dev) {
			list_for_each_entry_safe(udc_dev, udc_dev2,
						 &vudc_devices, list) {
				list_del(&udc_dev->list);
				put_vudc_device(udc_dev);
			}
			goto out;
		}
		list_add_tail(&udc_dev->list, &vudc_devices);
	}

	retval = platform_driver_register(&vudc_driver);
	if (retval < 0)
		goto err_register_udc_driver;

	list_for_each_entry(udc_dev, &vudc_devices, list) {
		retval = platform_device_add(udc_dev->dev);
		if (retval < 0) {
			list_for_each_entry(udc_dev2, &vudc_devices, list) {
				if (udc_dev2 == udc_dev)
					break;
				platform_device_del(udc_dev2->dev);
			}
			goto err_add_udc;
		}
	}
	list_for_each_entry(udc_dev, &vudc_devices, list) {
		if (!platform_get_drvdata(udc_dev->dev)) {
			/*
			 * The udc was added successfully but its probe
			 * function failed for some reason.
			 */
			retval = -EINVAL;
			goto err_probe_udc;
		}
	}

	debug_print("[vudc] ### init ###\n");

	return retval;

err_probe_udc:
	list_for_each_entry(udc_dev, &vudc_devices, list)
		platform_device_del(udc_dev->dev);

err_add_udc:
	platform_driver_unregister(&vudc_driver);

err_register_udc_driver:
	list_for_each_entry_safe(udc_dev, udc_dev2, &vudc_devices, list) {
		list_del(&udc_dev->list);
		put_vudc_device(udc_dev);
	}

out:
	return retval;
}
module_init(init);

static void __exit cleanup(void)
{
	struct vudc_device *udc_dev = NULL, *udc_dev2 = NULL;

	debug_print("[vudc] *** cleanup ***\n");

	list_for_each_entry_safe(udc_dev, udc_dev2, &vudc_devices, list) {
		list_del(&udc_dev->list);
		platform_device_unregister(udc_dev->dev);
		put_vudc_device(udc_dev);
	}
	platform_driver_unregister(&vudc_driver);
	debug_print("[vudc] ### cleanup ###\n");
}
module_exit(cleanup);

