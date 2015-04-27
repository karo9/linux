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

	struct usb_gadget *gadget;
	struct list_head queue; // Request queue
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
				char *out, ssize_t maxsz)
{
	struct vrequest *usb_req;
	int ret;
	struct vep *ep0 = usb_ep_to_vep(udc->gadget.ep0);

	ret = udc->driver->setup(&(udc->gadget), req);
	if (ret < 0) {
		debug_print("[vudc] Failed to setup device descriptor request!\n");
		goto exit;
	}

	/* FIXME: assuming request queue is empty; request is now on top */
	usb_req = list_entry(ep0->queue.prev, struct vrequest, queue);
	list_del(&(usb_req->queue));

	if (maxsz < usb_req->req.length) {
		ret = -1;
		goto clean_req;
	}

	memcpy(out, usb_req->req.buf, usb_req->req.length);
	ret = usb_req->req.length;

clean_req:
	usb_req->req.status = 0;
	usb_req->req.actual = usb_req->req.length;
	usb_gadget_giveback_request(&(ep0->ep), &(usb_req->req));
exit:
	return ret;
}

/*
 * Fetches device and interface descriptors from the gadget driver.
 * Writes the device descriptor first, then interface descriptors in order as
 * in chapter 9.
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
	ret = fetch_descriptor(&req, udc, out + sz, PAGE_SIZE - sz);
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
		req.wIndex = cpu_to_le16(i);
		ret = fetch_descriptor(&req, udc, out + sz, PAGE_SIZE - sz);
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

static void usbip_dump_usb_ctrlrequest2(struct usb_ctrlrequest *cmd)
{
	if (!cmd) {
		pr_debug("       : null pointer\n");
		return;
	}

	pr_debug("       ");
	pr_debug("bRequestType(%02X) bRequest(%02X) wValue(%04X) wIndex(%04X) wLength(%04X) ",
		 cmd->bRequestType, cmd->bRequest,
		 cmd->wValue, cmd->wIndex, cmd->wLength);
	pr_debug("\n       ");

	if ((cmd->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		pr_debug("STANDARD ");
		switch (cmd->bRequest) {
		case USB_REQ_GET_STATUS:
			pr_debug("GET_STATUS\n");
			break;
		case USB_REQ_CLEAR_FEATURE:
			pr_debug("CLEAR_FEAT\n");
			break;
		case USB_REQ_SET_FEATURE:
			pr_debug("SET_FEAT\n");
			break;
		case USB_REQ_SET_ADDRESS:
			pr_debug("SET_ADDRRS\n");
			break;
		case USB_REQ_GET_DESCRIPTOR:
			pr_debug("GET_DESCRI\n");
			break;
		case USB_REQ_SET_DESCRIPTOR:
			pr_debug("SET_DESCRI\n");
			break;
		case USB_REQ_GET_CONFIGURATION:
			pr_debug("GET_CONFIG\n");
			break;
		case USB_REQ_SET_CONFIGURATION:
			pr_debug("SET_CONFIG\n");
			break;
		case USB_REQ_GET_INTERFACE:
			pr_debug("GET_INTERF\n");
			break;
		case USB_REQ_SET_INTERFACE:
			pr_debug("SET_INTERF\n");
			break;
		case USB_REQ_SYNCH_FRAME:
			pr_debug("SYNC_FRAME\n");
			break;
		default:
			pr_debug("REQ(%02X)\n", cmd->bRequest);
			break;
		}
		//usbip_dump_request_type(cmd->bRequestType);
	} else if ((cmd->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		pr_debug("CLASS\n");
	} else if ((cmd->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		pr_debug("VENDOR\n");
	} else if ((cmd->bRequestType & USB_TYPE_MASK) == USB_TYPE_RESERVED) {
		pr_debug("RESERVED\n");
	}
}

void usbip_dump_urb2(struct urb *urb)
{

	if (!urb) {
		pr_debug("urb: null pointer!!\n");
		return;
	}

	printk(KERN_ERR "   urb                   :%p\n", urb);
	printk(KERN_ERR "   dev                   :%p\n", urb->dev);

	printk(KERN_ERR "   pipe                  :%08x ", urb->pipe);

/* 	usbip_dump_pipe(urb->pipe);
 */

	printk(KERN_ERR "   status                :%d\n", urb->status);
	printk(KERN_ERR "   transfer_flags        :%08X\n", urb->transfer_flags);
	printk(KERN_ERR "   transfer_buffer       :%p\n", urb->transfer_buffer);
	printk(KERN_ERR "   transfer_buffer_length:%d\n",
						urb->transfer_buffer_length);
	printk(KERN_ERR "   actual_length         :%d\n", urb->actual_length);
	printk(KERN_ERR "   setup_packet          :%p\n", urb->setup_packet);

	//if (urb->setup_packet && usb_pipetype(urb->pipe) == PIPE_CONTROL)
		usbip_dump_usb_ctrlrequest2(
			(struct usb_ctrlrequest *)urb->setup_packet);


	printk(KERN_ERR "   start_frame           :%d\n", urb->start_frame);
	printk(KERN_ERR "   number_of_packets     :%d\n", urb->number_of_packets);
	printk(KERN_ERR "   interval              :%d\n", urb->interval);
	printk(KERN_ERR "   error_count           :%d\n", urb->error_count);
	printk(KERN_ERR "   context               :%p\n", urb->context);
	printk(KERN_ERR "   complete              :%p\n", urb->complete);
}

static void stub_recv_cmd_submit(struct vudc *sdev,
				 struct usbip_header *pdu)
{
	int ret;
	struct vrequest *priv;
	struct usbip_device *ud = &sdev->udev;
	size_t size;
	//int pipe = get_pipe(sdev, pdu->base.ep, pdu->base.direction);

	priv = kzalloc(sizeof(struct vrequest), GFP_KERNEL);

	priv->seqnum = pdu->base.seqnum;
	priv->sdev = sdev;
	//todo moze sie przyda
	//list_add_tail(&priv->list, &sdev->priv_init);

	ret = 0;
	priv->urb = usb_alloc_urb(0, GFP_KERNEL);

	if (!priv->urb) {
		return;
	}

	size = pdu->u.cmd_submit.transfer_buffer_length;
	if (size > 0) {
		printk(KERN_ERR "Potrzebna alokacja bufora\n");
		priv->urb->transfer_buffer = kzalloc(size, GFP_KERNEL);
		//ret = usbip_recv(ud->tcp_socket, priv->urb->transfer_buffer, size);
	}

	priv->urb->setup_packet = kmemdup(&pdu->u.cmd_submit.setup, 8,
					  GFP_KERNEL);

	priv->urb->context                = (void *) priv;
	//priv->urb->dev                    = udev;
	priv->urb->pipe                   = 0;
	//priv->urb->complete               = stub_complete;

	usbip_pack_pdu(pdu, priv->urb, USBIP_CMD_SUBMIT, 0);
	 
	usbip_dump_header(pdu);
	usbip_dump_urb2(priv->urb);
	//ret = usb_submit_urb(priv->urb, GFP_KERNEL);
	

	sdev->driver->setup(&sdev->gadget, priv->urb->setup_packet);

	usbip_dbg_stub_rx("Leave\n");
}

/* recv a pdu */
static void stub_rx_pdu(struct usbip_device *ud)
{
	int ret;
	struct usbip_header pdu;
	struct vudc *sdev = container_of(ud, struct vudc, udev);
	struct device *dev = &sdev->dev->dev;

	usbip_dbg_stub_rx("Enter\n");

	memset(&pdu, 0, sizeof(pdu));
	ret = usbip_recv(ud->tcp_socket, &pdu, sizeof(pdu));
	usbip_header_correct_endian(&pdu, 0);

	switch (pdu.base.command) {
	case USBIP_CMD_UNLINK:
		printk(KERN_ERR "USBIP_CMD_UNLINK - TODO\n");
		break;

	case USBIP_CMD_SUBMIT:
		stub_recv_cmd_submit(sdev, &pdu);
		break;

	default:
		dev_err(dev, "unknown pdu\n");
		break;
	}
}

int stub_rx_loop(void *data)
{
	struct usbip_device *ud = data;

	while (!kthread_should_stop())
		stub_rx_pdu(ud);

	return 0;
}








/* ************************************************************************************************************ */

int thread_rx(void *data)
{
	int ret;
  	struct socket *sock;
	struct usbip_header pdu;

	debug_print("[vudc] *** thread_rx ***\n");

	//Simple recv
 	sock = (struct socket *) data;

	ret = usbip_recv(sock, &pdu, sizeof(pdu));
	if(ret != sizeof(pdu)) {
		debug_print("[vudc] error while reciving pdu\n");
	}

	usbip_header_correct_endian(&pdu, 0);

	usbip_dump_header(&pdu);

	kernel_sock_shutdown(sock, SHUT_RDWR);
	sockfd_put(sock);

	debug_print("[vudc] ### thread_rx ###\n");
	return 0;
}

/* 	int result;
 * 	struct msghdr msg;
 * 	struct kvec iov;
 * 	int total;
 * 	char buf[10];
 * 	char response[10];
 * 	int size;
 * 	char *bp;
 * 	int osize;
 * 	struct socket *sock;
 * 	struct msghdr msg2;
 * 	struct kvec iov2;
 */
/* 	total = 0;
 * 	size = 10;
 * 	bp = buf;
 * 	osize = size;
 * 
 * 	sock->sk->sk_allocation = GFP_NOIO;
 * 	iov.iov_base    = buf;
 * 	iov.iov_len     = size;
 * 	msg.msg_name    = NULL;
 * 	msg.msg_namelen = 0;
 * 	msg.msg_control = NULL;
 * 	msg.msg_controllen = 0;
 * 	msg.msg_flags      = MSG_NOSIGNAL;
 * 
 * 	memset(&msg2, 0, sizeof(struct msghdr));
 * 	response[0] = 'O';
 * 	response[1] = 'k';
 * 	response[2] = '\0';
 * 
 * 	iov2.iov_base = response;
 * 	iov2.iov_len = 3;
 * 
 * 	result = 1;
 * 	while(result != 0)
 * 	{
 * 		debug_print("[vudc] Start listening \n");
 * 		result = kernel_recvmsg(sock, &msg, &iov, 1, size, MSG_WAITALL);
 * 		if (result <= 0) {
 * 			debug_print("[vudc] Error during recv\n");
 * 		}
 * 
 * 		buf[9] = '\0';
 * 		debug_print("[vudc] Message received: %s\n", buf);
 * 
 * 
 * 		kernel_sendmsg(sock, &msg2, &iov2, 1, 3);
 * 	}
 */
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
	//vudc->vudc_rx = kthread_run(&thread_rx, socket, "vudc_rx");
	
	//vudc->vudc_tx = kthread_run(&thread_tx, socket, "vudc_tx");

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

	if (!_ep || !_req)
		return -EINVAL;

	ep = usb_ep_to_vep(_ep);
	req = usb_request_to_vrequest(_req);
	vudc = ep_to_vudc(ep);

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
		vudc->gadget.speed = vudc->driver->max_speed;

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
	vudc->gadget.max_speed = USB_SPEED_SUPER;
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

