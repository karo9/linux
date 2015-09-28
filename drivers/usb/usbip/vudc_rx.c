/*
 * Copyright (C) 2015 Karol Kosik
 * 		 2015 Samsung Electronics
 * 		 2015 Igor Kotrasinski <i.kotrasinsk@samsung.com>
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

#include <net/sock.h>
#include <linux/list.h>
#include <linux/kthread.h>

#include "usbip_common.h"
#include "vudc.h"

static int v_recv_cmd_unlink(struct vudc *sdev,
				struct usbip_header *pdu)
{
	unsigned long flags;
	struct urbp *urb_p;

	spin_lock_irqsave(&sdev->lock, flags);
	list_for_each_entry(urb_p, &sdev->urb_q, urb_q) {
		if (urb_p->seqnum != pdu->u.cmd_unlink.seqnum)
			continue;
		urb_p->urb->unlinked = -ECONNRESET;
		urb_p->seqnum = pdu->base.seqnum;
		v_kick_timer(sdev, jiffies);
		spin_unlock_irqrestore(&sdev->lock, flags);
		return 0;
	}
	/* Not found, completed / not queued */
	spin_lock(&sdev->lock_tx);
	v_enqueue_ret_unlink(sdev, pdu->base.seqnum, 0);
	wake_up(&sdev->tx_waitq);
	spin_unlock(&sdev->lock_tx);
	spin_unlock_irqrestore(&sdev->lock, flags);

	return 0;
}

static int v_recv_cmd_submit(struct vudc *sdev,
				 struct usbip_header *pdu)
{
	int ret = 0;
	struct urbp *urb_p;
	u8 address;
	unsigned long flags;

	urb_p = alloc_urbp();
	if (!urb_p) {
		usbip_event_add(&sdev->udev, VUDC_EVENT_ERROR_MALLOC);
		return -ENOMEM;
	}

	/* base.ep is pipeendpoint(pipe) */
	address = pdu->base.ep;
	if (pdu->base.direction == USBIP_DIR_IN)
		address |= USB_DIR_IN;

	spin_lock_irq(&sdev->lock);
	urb_p->ep = find_endpoint(sdev, address);
	if (!urb_p->ep) {
		/* we don't know the type, there may be isoc data! */
		dev_err(&sdev->plat->dev, "request to nonexistent endpoint");
		spin_unlock_irq(&sdev->lock);
		usbip_event_add(&sdev->udev, VUDC_EVENT_ERROR_TCP);
		ret = -EPIPE;
		goto free_urbp;
	}
	urb_p->type = urb_p->ep->type;
	spin_unlock_irq(&sdev->lock);

	urb_p->new = 1;
	urb_p->seqnum = pdu->base.seqnum;

	ret = alloc_urb_from_cmd(&urb_p->urb, pdu, urb_p->ep->type);
	if (ret) {
		usbip_event_add(&sdev->udev, VUDC_EVENT_ERROR_MALLOC);
		ret = -ENOMEM;
		goto free_urbp;
	}

	urb_p->urb->status = -EINPROGRESS;

	/* FIXME: more pipe setup to please usbip_common */
	urb_p->urb->pipe &= ~(11 << 30);
	switch (urb_p->ep->type) {
	case USB_ENDPOINT_XFER_BULK:
		urb_p->urb->pipe |= (PIPE_BULK << 30);
		break;
	case USB_ENDPOINT_XFER_INT:
		urb_p->urb->pipe |= (PIPE_INTERRUPT << 30);
		break;
	case USB_ENDPOINT_XFER_CONTROL:
		urb_p->urb->pipe |= (PIPE_CONTROL << 30);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		urb_p->urb->pipe |= (PIPE_ISOCHRONOUS << 30);
		break;
	}

	if ((ret = usbip_recv_xbuff(&sdev->udev, urb_p->urb)) < 0)
		goto free_urbp;

	if ((ret = usbip_recv_iso(&sdev->udev, urb_p->urb)) < 0)
		goto free_urbp;

	spin_lock_irqsave(&sdev->lock, flags);
	v_kick_timer(sdev, jiffies);
	list_add_tail(&urb_p->urb_q, &sdev->urb_q);
	spin_unlock_irqrestore(&sdev->lock, flags);

	return 0;

free_urbp:
	free_urbp_and_urb(urb_p);
	return ret;
}

static int v_rx_pdu(struct usbip_device *ud)
{
	int ret;
	struct usbip_header pdu;
	struct vudc *sdev = container_of(ud, struct vudc, udev);

	memset(&pdu, 0, sizeof(pdu));
	ret = usbip_recv(ud->tcp_socket, &pdu, sizeof(pdu));
	if (ret != sizeof(pdu)) {
		usbip_event_add(ud, VUDC_EVENT_ERROR_TCP);
		if (ret >= 0)
			return -EPIPE;
		return ret;
	}
	usbip_header_correct_endian(&pdu, 0);

	spin_lock_irq(&ud->lock);
	ret = (ud->status == SDEV_ST_USED);
	spin_unlock_irq(&ud->lock);
	if (!ret) {
		usbip_event_add(ud, VUDC_EVENT_ERROR_TCP);
		return -EBUSY;
	}

	switch (pdu.base.command) {
	case USBIP_CMD_UNLINK:
		ret = v_recv_cmd_unlink(sdev, &pdu);
		break;
	case USBIP_CMD_SUBMIT:
		ret = v_recv_cmd_submit(sdev, &pdu);
		break;
	default:
		dev_err(&sdev->plat->dev, "rx: unknown command");
		break;
	}
	return ret;
}

int v_rx_loop(void *data)
{
	struct usbip_device *ud = data;
	struct vudc *sdev = container_of(ud, struct vudc, udev);
	int ret = 0;

	while (!kthread_should_stop()) {
		if (usbip_event_happened(ud))
			break;
		if ((ret = v_rx_pdu(ud)) < 0) {
			dev_err(&sdev->plat->dev,
				"v_rx exit with error %d", ret);
			break;
		}
	}
	return ret;
}
