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

#include "usbip_common.h"
#include "stub.h"

#include <net/sock.h>

#define DRIVER_DESC "USB over IP UDC"
#define DRIVER_VERSION "06 March 2015"

#define DEBUG 1
#define debug_print(...) \
		do { if (DEBUG) printk(KERN_ERR __VA_ARGS__); } while (0)

static const char driver_desc[] = DRIVER_DESC;
static const char gadget_name[] = "vudc";

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
	/* Add here some fields if needed */

	const struct usb_endpoint_descriptor *desc;
	struct usb_gadget *gadget;
	struct list_head queue; // Request queue
	unsigned halted:1;
	unsigned wedged:1;
};

/* container for usb_request to store some request related data */
struct vrequest {
	struct usb_request req;
	struct urb *urb;
	/* Add here some fields if needed */

	unsigned long seqnum;
	struct vudc *sdev;

	struct list_head queue; // Request queue
};

struct vudc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *dev;
	/* Add here some fields if needed */

	struct usbip_device udev;
	struct task_struct *vudc_rx;
	struct task_struct *vudc_tx;

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

/* sysfs files */
static int sysfs_variable = 0;

static ssize_t example_in_show(struct device *dev, struct device_attribute *attr,
		     char *out)
{
	char *s = out;

	debug_print("[vudc] *** example_in_show ***\n");

	out += sprintf(out, "Hi, variable = %d\n", sysfs_variable);

	debug_print("[vudc] ### example_in_show ###\n");

	return out - s;
}
static DEVICE_ATTR_RO(example_in);

static ssize_t fetch_descriptor(struct usb_ctrlrequest* req, struct vudc* udc,
				char *out, ssize_t sz, ssize_t maxsz)
{
	struct vrequest *usb_req;
	int ret;
	int copysz;
	struct vep *ep0 = usb_ep_to_vep(udc->gadget.ep0);

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
 * Fetches device and interface descriptors from the gadget driver.
 * Writes the device descriptor first, then interface descriptors.
 */
static ssize_t descriptor_show(struct device *dev,
			       struct device_attribute *attr, char *out)
{
	struct vudc *udc;
	struct usb_ctrlrequest req;
	struct usb_device_descriptor *dev_desc;
	int ret;
	int sz = 0;
	int i;

	udc = (struct vudc*) dev_get_drvdata(dev);
	if (!udc || !udc->driver)
		return -1;

	req.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = cpu_to_le16(USB_DT_DEVICE << 8);
	req.wIndex = cpu_to_le16(0);
	req.wLength = cpu_to_le16(PAGE_SIZE - sz);
	ret = fetch_descriptor(&req, udc, out + sz, PAGE_SIZE - sz, PAGE_SIZE - sz);
	if (ret < 0) {
		debug_print("[vudc] Could not fetch device descriptor!\n");
		return -1;
	}
	sz += ret;
	dev_desc = (struct usb_device_descriptor *) out;

	req.bRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	req.bRequest = USB_REQ_GET_DESCRIPTOR;
	req.wValue = cpu_to_le16(USB_DT_CONFIG << 8);
	req.wIndex = cpu_to_le16(0);
	req.wLength = cpu_to_le16(PAGE_SIZE - sz);
	for (i = 0; i < dev_desc->bNumConfigurations ; i++) {
		req.wValue = cpu_to_le16((USB_DT_CONFIG << 8) | i);
		ret = fetch_descriptor(&req, udc, out + sz,
		                       sizeof(struct usb_config_descriptor), PAGE_SIZE - sz);
		if (ret < 0) {
			debug_print("[vudc] Could not fetch interface descriptor!\n");
			return -1;
		}
		sz += ret;
	}
	return sz;
}

static DEVICE_ATTR_RO(descriptor);

/* ************************************************************************************************************ */

/* Adapted from dummy_hcd.c ; caller must hold lock */
static void transfer(struct vudc* sdev,
		struct urb *urb, struct vep *ep)
{
	struct vrequest	*req;

top:
	/* if there's no request queued, the device is NAKing; return */
	list_for_each_entry(req, &ep->queue, queue) {
		unsigned	host_len, dev_len, len;
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

			if(urb->pipe == USBIP_DIR_IN)
				memcpy(urb->transfer_buffer, req->req.buf, len);
			else
				memcpy(req->req.buf, urb->transfer_buffer, len);

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

static void setup_ret_submit_pdu(struct usbip_header *rpdu, struct urb *urb)
{
	struct vrequest *priv = (struct vrequest *) urb->context;

	setup_base_pdu(&rpdu->base, USBIP_RET_SUBMIT, priv->seqnum);
	usbip_pack_pdu(rpdu, urb, USBIP_RET_SUBMIT, 1);
}


static void send_respond(struct urb *urb, struct vudc *sdev)
{

		struct usbip_header pdu_header;
		struct usbip_iso_packet_descriptor *iso_buffer = NULL;
		struct kvec *iov = NULL;
		int iovnum = 0;
		size_t txsize;
		struct msghdr msg;

		txsize = 0;
		memset(&pdu_header, 0, sizeof(pdu_header));
		memset(&msg, 0, sizeof(msg));

		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS)
			iovnum = 2 + urb->number_of_packets;
		else
			iovnum = 2;

		iov = kcalloc(iovnum, sizeof(struct kvec), GFP_KERNEL);

		iovnum = 0;

		setup_ret_submit_pdu(&pdu_header, urb);
		usbip_dbg_stub_tx("setup txdata seqnum: %d urb: %p\n",
				  pdu_header.base.seqnum, urb);
		usbip_header_correct_endian(&pdu_header, 1);

		iov[iovnum].iov_base = &pdu_header;
		iov[iovnum].iov_len  = sizeof(pdu_header);
		iovnum++;
		txsize += sizeof(pdu_header);

		iov[iovnum].iov_base = urb->transfer_buffer;
		iov[iovnum].iov_len  = urb->actual_length;
		iovnum++;
		txsize += urb->actual_length;

		kernel_sendmsg(sdev->udev.tcp_socket, &msg,
						iov,  iovnum, txsize);

		kfree(iov);
		kfree(iso_buffer);
}

/* some members of urb must be substituted before. */
int usbip_recv_xbuff(struct usbip_device *ud, struct urb *urb)
{
	int ret;
	int size;


	printk(KERN_ERR "[xbuff] uno");
	if (urb->pipe == USBIP_DIR_IN)
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
		sdev->address = w_value;
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
free_urb:
	usb_free_urb(urb);
err:
	return -ENOMEM;
}

static void stub_recv_cmd_submit(struct vudc *sdev,
				 struct usbip_header *pdu)
{
	int ret;
	struct vrequest *priv;
	struct vep *vep;
	struct urb *urb;
	struct urb **urbp = &urb;
	u8 address;
	int setup_handled = 1;
	unsigned long flags;

	priv = kzalloc(sizeof(struct vrequest), GFP_KERNEL);
	
	/* base.ep is pipeendpoint(pipe) */
	address = pdu->base.ep;
	if (pdu->base.direction == USBIP_DIR_IN)
		address |= USB_DIR_IN;

	vep = find_endpoint(sdev, address);

	priv->seqnum = pdu->base.seqnum;
	priv->sdev = sdev;
	printk(KERN_ERR "Ustawiam %p priv->seqnum na %d", priv, pdu->base.seqnum);
	printk(KERN_ERR "Otrzymuje: %p priv->seqnum na %lu", priv, priv->seqnum);

	ret = alloc_urb_from_cmd(urbp, pdu);
	if (ret) {
		/* TODO - handle -ENOMEM */
		return;
	}

	urb = *urbp;
	priv->urb = urb;
	urb->context                = (void *) priv;
	 
	usbip_recv_xbuff(&sdev->udev, urb);

	usbip_dump_header(pdu);
	usbip_dump_urb(urb);

	printk(KERN_ERR "Przed setup\n");

	spin_lock_irqsave(&sdev->lock, flags);
	if(vep == &sdev->ep[0]) {
		/* TODO - flush any stale requests */
		setup_handled = handle_control_request(sdev, urb,
		                (struct usb_ctrlrequest *) urb->setup_packet, &ret);
		if (setup_handled > 0)
			spin_unlock(&sdev->lock);
			ret = sdev->driver->setup(&sdev->gadget,
			                    (struct usb_ctrlrequest *) urb->setup_packet);
			spin_lock(&sdev->lock);
	}
	if (ret >= 0) {
		/* TODO - when different types are coded in, treat like bulk */
	}
	else {
		urb->status = -EPIPE;
		urb->actual_length = 0;
		spin_unlock_irqrestore(&sdev->lock, flags);
		goto return_urb;
	}

	printk(KERN_ERR "Przed make transfer\n");
	transfer(sdev, priv->urb, vep);
	spin_unlock_irqrestore(&sdev->lock, flags);

return_urb:
	printk(KERN_ERR "Przed respond\n");
	send_respond(priv->urb, sdev);

	usbip_dbg_stub_rx("Leave\n");
}

static int stub_rx_pdu(struct usbip_device *ud)
{
	int ret;
	struct usbip_header pdu;
	struct vudc *sdev = container_of(ud, struct vudc, udev);

	usbip_dbg_stub_rx("Enter\n");

	memset(&pdu, 0, sizeof(pdu));
	ret = usbip_recv(ud->tcp_socket, &pdu, sizeof(pdu));
	if(ret == 0) {
		return -1;
	}
	usbip_header_correct_endian(&pdu, 0);

	switch (pdu.base.command) {
	case USBIP_CMD_UNLINK:
		printk(KERN_ERR "USBIP_CMD_UNLINK - TODO\n");
		break;

	case USBIP_CMD_SUBMIT:
		stub_recv_cmd_submit(sdev, &pdu);
		break;

	default:
		printk(KERN_ERR "UNKOWN PDU\n");
		//dev_err(dev, "unknown pdu\n");
		break;
	}
	return 0;
}

int stub_rx_loop(void *data)
{
	struct usbip_device *ud = data;

	while (!kthread_should_stop())
		if(stub_rx_pdu(ud) == -1)
			break;

	return 0;
}








/* ************************************************************************************************************ */

int thread_tx(void *data)
{
	debug_print("[vudc] *** thread_tx ***\n");

	debug_print("[vudc] ### thread_tx ###\n");

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

	rv = sscanf(in, "%d", &sockfd);
	if (rv != 1)
		return -EINVAL;

	socket = sockfd_lookup(sockfd, &err);
	if(!socket)
	{
		debug_print("[vudc] Failed to lookup sock");
		return -EINVAL;
	}

	vudc->udev.side = USBIP_VUDC;
	vudc->udev.tcp_socket = socket;

	/*  Now create threads to take care of transmition */

	vudc->udev.tcp_rx = kthread_run(&stub_rx_loop, &vudc->udev, "vudc_rx");
	
	debug_print("[vudc] ### example_out ###\n");

	return count;
}
static DEVICE_ATTR(vudc_sockfd, S_IWUSR, NULL, store_sockfd);

/* endpoint related operations */

static int vep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct vep *ep;

	debug_print("[vudc] *** enable ***\n");

	if (!_ep || !desc)
		return -EINVAL;
	ep = usb_ep_to_vep(_ep);

	ep->desc = desc;
	/* TODO */

	debug_print("[vudc] ### enable ###\n");

	return 0;

}

static int vep_disable(struct usb_ep *_ep)
{
	struct vep *ep;

	debug_print("[vudc] *** disable ***\n");

	if (!_ep)
		return -EINVAL;
	ep = usb_ep_to_vep(_ep);

	/* TODO */

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

	debug_print("[vudc] *** vep_dequeue ***\n");

	if (!_ep || !_req)
		return -EINVAL;

	ep = usb_ep_to_vep(_ep);
	req = usb_request_to_vrequest(_req);

	/* TODO */

	debug_print("[vudc] ### vep_dequeue ###\n");

	return -EINVAL;
}

static int
vep_set_halt(struct usb_ep *_ep, int value)
{
	debug_print("[vudc] *** vep_set_halt ***\n");
	/* TODO */
	debug_print("[vudc] ### vep_set_halt ###\n");
	return -EINVAL;
}

static int vep_set_wedge(struct usb_ep *_ep)
{
	debug_print("[vudc] *** vep_set_wedge ***\n");
	/* TODO */
	debug_print("[vudc] ### vep_set_wedge ###\n");
	return -EINVAL;
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
	debug_print("[vudc] *** vgadget_udc_start ***\n");
	
	vudc->driver = driver;

	/* TODO */
	debug_print("[vudc] ### vgadget_udc_start ###\n");
	return 0;
}

static int vgadget_udc_stop(struct usb_gadget *g)
{
	debug_print("[vudc] *** vgadget_udc_stop ***\n");
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

static int init_vudc_hw(struct vudc *vudc)
{
	int i;

	debug_print("[vudc] *** init_vudc_hw ***\n");

	INIT_LIST_HEAD(&vudc->gadget.ep_list);
	for(i = 0; i < VIRTUAL_ENDPOINTS; ++i) {
		struct vep *ep = &vudc->ep[i];

		if(!ep_name[i])
			break;

		ep->ep.name = ep_name[i];
		ep->ep.ops = &vep_ops;
		list_add_tail(&ep->ep.ep_list, &vudc->gadget.ep_list);
		usb_ep_set_maxpacket_limit(&ep->ep, ~0);
		ep->ep.max_streams = 16;
		ep->gadget = &vudc->gadget;

		INIT_LIST_HEAD(&ep->queue);
	}

	vudc->gadget.ep0 = &vudc->ep[0].ep;
	list_del_init(&vudc->ep[0].ep.ep_list);

	/* TODO */
	debug_print("[vudc] ### init_vudc_hw ###\n");
	return 0;
}

static void cleanup_vudc_hw(struct vudc *vudc)
{
	debug_print("[vudc] *** cleanup_vudc_hw ***\n");
	/* TODO */
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

	device_create_file(&pdev->dev, &dev_attr_example_in);
	device_create_file(&pdev->dev, &dev_attr_descriptor);
	device_create_file(&pdev->dev, &dev_attr_vudc_sockfd);

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

	device_remove_file(&pdev->dev, &dev_attr_example_in);
	device_remove_file(&pdev->dev, &dev_attr_descriptor);
	device_remove_file(&pdev->dev, &dev_attr_vudc_sockfd);

	usb_del_gadget_udc(&vudc->gadget);
	cleanup_vudc_hw(vudc);
	kfree(vudc);

	debug_print("[vudc] ### vudc_remove ###\n");

	return 0;
}

static int vudc_suspend(struct platform_device *pdev, pm_message_t state)
{
	debug_print("[vudc] *** vudc_suspend ***\n");
	/* TODO */
	debug_print("[vudc] ### vudc_suspend ###\n");
	return 0;
}

static int vudc_resume(struct platform_device *pdev)
{
	debug_print("[vudc] *** vudc_resume ***\n");
	/* TODO */
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

