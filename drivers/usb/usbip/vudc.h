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

#ifndef __USBIP_VUDC_H
#define __USBIP_VUDC_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/ch9.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/sysfs.h>

#include "usbip_common.h"

extern const char *const ep_name[];

extern struct platform_driver vudc_driver;
extern const struct usb_ep_ops vep_ops;
extern const struct usb_gadget_ops vgadget_ops;

struct vep {
	struct usb_ep ep;
	unsigned type:2; /* type, as USB_ENDPOINT_XFER_* */

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
	struct vudc *cdev;
	struct list_head queue; /* Request queue */
};

struct urbp {
	struct urb *urb;
	struct vep *ep;
	struct list_head urb_q; /* urb queue */
	unsigned long seqnum;
	unsigned type:2; /* for tx, since ep type can change after */
	unsigned new:1;
};

struct v_unlink {
	unsigned long seqnum;
	struct list_head list;
	__u32 status;
};

#define TX_UNLINK 1
#define TX_SUBMIT 0

struct tx_item {
	struct list_head tx_q;
	unsigned type:1;
	union {
		struct urbp *s;
		struct v_unlink *u;
	};
};

enum tr_state {
	VUDC_TR_RUNNING,
	VUDC_TR_IDLE,
	VUDC_TR_STOPPED,
};

struct transfer_timer {
	struct timer_list timer;
	enum tr_state state;
	unsigned long frame_start;
	int frame_limit;
};

struct vudc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct platform_device *plat;

	struct usb_device_descriptor dev_desc;

	struct usbip_device udev;
	struct transfer_timer tr_timer;
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

struct vudc_device {
	struct platform_device *plat;
	struct list_head list;
};

extern const struct attribute_group vudc_attr_group;

/* visible everywhere */

static inline struct vep *to_vep(struct usb_ep *_ep)
{
	return container_of(_ep, struct vep, ep);
}

static inline struct vrequest *to_vrequest(
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

/* vudc_sysfs.c */

int descriptor_cache(struct vudc *cdev);

/* vudc_tx.c */

int v_tx_loop(void *data);
void v_enqueue_ret_unlink(struct vudc *cdev, __u32 seqnum, __u32 status);
void v_enqueue_ret_submit(struct vudc *cdev, struct urbp *urb_p);

/* vudc_rx.c */

int v_rx_loop(void *data);

/* vudc_transfer.c */

void v_init_timer(struct vudc *cdev);
void v_start_timer(struct vudc *cdev);
void v_kick_timer(struct vudc *cdev, unsigned long time);
void v_stop_timer(struct vudc *cdev);

/* vudc_dev.c */

int alloc_urb_from_cmd(struct urb **urbp, struct usbip_header *pdu, u8 type);
struct urbp *alloc_urbp(void);
void free_urbp_and_urb(struct urbp *urb_p);

struct vep *find_endpoint(struct vudc *cdev, u8 address);

struct vudc_device *alloc_vudc_device(int devid);
void put_vudc_device(struct vudc_device *udc_dev);

#endif /* __USBIP_VUDC_H */
