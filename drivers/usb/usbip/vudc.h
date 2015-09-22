/*
 * vudc.h -- USB over IP UDC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __USBIP_VUDC_H
#define __USBIP_VUDC_H

#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/ch9.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/sysfs.h>

#include "stub.h"


extern const char *const ep_name[];

struct vep {
	struct usb_ep ep;
	unsigned type:2;

	const struct usb_endpoint_descriptor *desc;
	struct usb_gadget *gadget;
	struct list_head queue; /* Request queue */
	unsigned halted:1;
	unsigned wedged:1;
	unsigned already_seen:1;
	unsigned setup_stage:1;
};

struct vrequest {
	struct usb_request req;
	struct vudc *sdev;
	struct list_head queue; /* Request queue */
};

struct urbp {
	struct urb *urb;
	struct vep *ep;
	struct list_head urb_q;
	unsigned long seqnum;
	unsigned new:1;
};

#define TX_UNLINK 1
#define TX_SUBMIT 0

struct tx_item {
	struct list_head tx_q;
	unsigned type:1;
	union {
		struct urbp *s;
		struct stub_unlink *u;
	};
};

struct vudc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *dev;

	struct usb_device_descriptor *dev_descr;

	struct usbip_device udev;
	struct timer_list tr_timer;
	struct timeval start_time;

	struct list_head urb_q;

	spinlock_t lock_tx;
	struct list_head priv_tx;
	wait_queue_head_t tx_waitq;

	spinlock_t lock;
	struct vep *ep;
	int address;
	u16 devstatus;
};

extern const struct attribute_group vudc_attr_group;

/* visible everywhere */

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

int descriptor_cache(struct vudc *sdev);
int stub_tx_loop(void *data);
int stub_rx_loop(void *data);

#endif /* __USBIP_VUDC_H */
