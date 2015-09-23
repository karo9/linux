#include <net/sock.h>
#include <linux/list.h>
#include <linux/kthread.h>

#include "usbip_common.h"
#include "stub.h"

#include "vudc.h"

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
		urb_p->seqnum = pdu->base.seqnum;
		v_kick_timer(sdev, jiffies);
		spin_unlock_irqrestore(&sdev->lock, flags);
		return;
	}
	/* Not found, completed / not queued */
	spin_lock(&sdev->lock_tx);
	v_enqueue_ret_unlink(sdev, pdu->base.seqnum, 0);
	wake_up(&sdev->tx_waitq);
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
	if (!urb_p->ep) {
		/* we don't know the type, there may be isoc data! */
		usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_TCP);
		return;
	}
	urb_p->new = 1;
	urb_p->seqnum = pdu->base.seqnum;

	ret = alloc_urb_from_cmd(&urb_p->urb, pdu, urb_p->ep->type);
	if (ret) {
		usbip_event_add(&sdev->udev, SDEV_EVENT_ERROR_MALLOC);
		return;
	}

	urb_p->urb->status = -EINPROGRESS;

	/* FIXME: more pipe setup to please usbip_common */
	urb_p->urb->pipe &= ~(11 << 30);
	switch(urb_p->ep->type) {
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

	if (usbip_recv_xbuff(&sdev->udev, urb_p->urb) < 0)
		return;

	if (usbip_recv_iso(&sdev->udev, urb_p->urb) < 0)
		return;

	spin_lock_irqsave(&sdev->lock, flags);
	v_kick_timer(sdev, jiffies);
	list_add_tail(&urb_p->urb_q, &sdev->urb_q);
	spin_unlock_irqrestore(&sdev->lock, flags);
}

static void stub_rx_pdu(struct usbip_device *ud)
{
	int ret;
	struct usbip_header pdu;
	struct vudc *sdev = container_of(ud, struct vudc, udev);

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
		/* TODO - err message */
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
