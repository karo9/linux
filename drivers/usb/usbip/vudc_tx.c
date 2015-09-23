#include <net/sock.h>
#include <linux/list.h>
#include <linux/kthread.h>

#include "usbip_common.h"
#include "stub.h"

#include "vudc.h"

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

static int v_send_ret_unlink(struct vudc * sdev, struct stub_unlink *unlink)
{
	struct msghdr msg;
	struct kvec iov[1];
	size_t txsize;

	int ret;
	struct usbip_header pdu_header;

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
	kfree(unlink);

	return txsize;
}

static int v_send_ret_submit(struct vudc *sdev, struct urbp *urb_p)
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
		return -1;
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
		return -1;
	}

	kfree(iov);
	kfree(iso_buffer);
	free_urbp_and_urb(urb_p);
	return txsize;
}

static int v_send_ret(struct vudc *sdev)
{
	unsigned long flags;
	struct tx_item *txi;
	size_t total_size = 0;
	int ret;

	spin_lock_irqsave(&sdev->lock_tx, flags);
	while (!list_empty(&sdev->priv_tx)) {
		txi = list_entry(sdev->priv_tx.next, struct tx_item, tx_q);
		list_del_init(&txi->tx_q);
		spin_unlock_irqrestore(&sdev->lock_tx, flags);

		if (txi->type == TX_SUBMIT)
			ret = v_send_ret_submit(sdev, txi->s);
		else
			ret = v_send_ret_unlink(sdev, txi->u);
		kfree(txi);

		if (ret < 0)
			return -1;
		else
			total_size += ret;

		spin_lock_irqsave(&sdev->lock_tx, flags);
	}

	spin_unlock_irqrestore(&sdev->lock_tx, flags);
	return total_size;
}


int stub_tx_loop(void *data)
{
	struct usbip_device * udev = (struct usbip_device *) data;
	struct vudc * sdev = container_of(udev, struct vudc, udev);

	while (!kthread_should_stop()) {
		if (usbip_event_happened(&sdev->udev))
			break;
		if (v_send_ret(sdev) < 0)
			break;
		wait_event_interruptible(sdev->tx_waitq,
						(!list_empty(&sdev->priv_tx) ||
						kthread_should_stop()));
	}

	return 0;
}

/* called with spinlocks held */
void v_enqueue_ret_unlink(struct vudc *sdev, __u32 seqnum, __u32 status)
{
	struct tx_item *txi;
	struct stub_unlink *unlink;

	txi = kzalloc(sizeof(struct stub_unlink), GFP_ATOMIC);
	unlink = kzalloc(sizeof(struct stub_unlink), GFP_ATOMIC);
	if (!unlink || !txi) {
		usbip_event_add(&sdev->udev, VDEV_EVENT_ERROR_MALLOC);
		return;
	}

	unlink->seqnum = seqnum;
	unlink->status = status;
	txi->type = TX_UNLINK;
	txi->u = unlink;

	list_add_tail(&txi->tx_q, &sdev->priv_tx);
}

/* called with spinlocks held */
void v_enqueue_ret_submit(struct vudc *sdev, struct urbp *urb_p)
{
	struct tx_item *txi;

	txi = kzalloc(sizeof(struct stub_unlink), GFP_ATOMIC);
	if (!txi) {
		usbip_event_add(&sdev->udev, VDEV_EVENT_ERROR_MALLOC);
		return;
	}

	txi->type = TX_SUBMIT;
	txi->s = urb_p;

	list_add_tail(&txi->tx_q, &sdev->priv_tx);
}

