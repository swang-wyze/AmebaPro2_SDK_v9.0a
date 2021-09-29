/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

//#include "bluetooth/bluetooth.h"
#include <common/log.h>
#include <errno.h>
#include <kernel.h>
#include <drivers/bluetooth/hci_driver.h>
#include <hci_uart.h>
#include <btsnoop.h>

#define LOG_MODULE_NAME bt_driver

#ifndef H4_NONE
#define H4_NONE 0x00
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04
#define H4_ISO  0x05
#endif

static struct hci_h4 {
    struct k_sem rx_ind_sema;
    struct k_thread rx_thread_data;
}*h4 = NULL;

static K_KERNEL_STACK_DEFINE(rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);

//#define HCI_DEBUG
static void h4_data_dump(const char *tag, uint8_t type, uint8_t *data, uint32_t len)
{
#ifdef HCI_DEBUG
    int i = 0;
    printf("%s\n\r%02x ", tag, type);
    while(i < len) {
        printf("%02x ", data[i++]);
        if (i%16 == 0)
            printf("\n\r");
    }
    printf("\n\r");
#endif
}

static uint8_t h4_recv_ind(void)
{
    if (!h4->rx_thread_data.run)
        return HCI_FAIL;

    k_sem_give(&h4->rx_ind_sema);
    return HCI_SUCCESS;
}

static int h4_recv_data(uint8_t *buf, size_t count)
{
    ssize_t read_len = 0;
    ssize_t read_len_per = 0;
    ssize_t try_times = count + 10;

    while (try_times)
    {
        read_len_per = hci_uart_read(buf + read_len, count - read_len);
        read_len += read_len_per;
        if (read_len == count)
            break;
        if (0 == read_len_per)
        {
            try_times--;
            k_sem_take(&h4->rx_ind_sema, K_FOREVER);
            if (!h4->rx_thread_data.run)
                return 0;
        }
    }

    /* If read_len is not equal count, means ERROR! */
    if (read_len != count)
        BT_ASSERT_MSG(0, "H4 Read Failed! Exit H4 RX Thread!");
        
    return read_len;
}

static void h4_rx_thread(void *p1, void *p2, void *p3)
{
	unsigned char buffer[CONFIG_BT_RX_BUF_LEN];
	int hdr_len, data_len, ret;
	struct net_buf *buf;
	bool discardable;
	uint8_t type, event, evt_flags;
	union {
		struct bt_hci_evt_hdr evt;
		struct bt_hci_acl_hdr acl;
		struct bt_hci_iso_hdr iso;
	} hdr;

	while(1) {
		buf = NULL;
		ret = h4_recv_data(&type, sizeof(type));
		if (ret != sizeof(type))
			break;

		BT_ASSERT_MSG(type == H4_EVT || type == H4_ACL || type == H4_ISO, 
				"NOT support (0x%02x) type packet yet!!!", type);

		if (type == H4_EVT)
			hdr_len = sizeof(struct bt_hci_evt_hdr);
		else if (type == H4_ACL)
			hdr_len = sizeof(struct bt_hci_acl_hdr);
		else if (IS_ENABLED(CONFIG_BT_ISO) && type == H4_ISO)
			hdr_len = sizeof(struct bt_hci_iso_hdr);
		else
			continue;

		ret = h4_recv_data((uint8_t *)&hdr, hdr_len);
		if (ret != hdr_len)
			break;

		if (type == H4_EVT) {
			data_len = hdr.evt.len;
			discardable = false;

#if defined(CONFIG_BT_BREDR)
            if (BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI == hdr.evt.evt || 
                BT_HCI_EVT_EXTENDED_INQUIRY_RESULT  == hdr.evt.evt)
                discardable = true;
#endif

			if (hdr.evt.evt == BT_HCI_EVT_LE_META_EVENT) {
				ret = h4_recv_data(&event, 1);
				if (ret != 1)
					break;

				if (event == BT_HCI_EVT_LE_ADVERTISING_REPORT || 
                    event == BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT)
					discardable = true;
			}

			buf = bt_buf_get_evt(hdr.evt.evt, discardable,
					discardable ? K_NO_WAIT : K_FOREVER);
			if (buf == NULL) {
				if (discardable) {
					h4_recv_data(buffer, data_len - 1);
					continue;
				} else
					break;
			}

			memcpy(buf->data, &hdr, hdr_len);

			if (hdr.evt.evt == BT_HCI_EVT_LE_META_EVENT) {
				buf->data[sizeof(struct bt_hci_evt_hdr)] = event;
				hdr_len += 1;
				data_len -= 1;
			}

			evt_flags = bt_hci_evt_get_flags(hdr.evt.evt);
			bt_buf_set_type(buf, BT_BUF_EVT);
		} else if (type == H4_ACL) {
			buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
			if (buf == NULL)
				break;

			data_len = hdr.acl.len;
			evt_flags = BT_HCI_EVT_FLAG_RECV;
			bt_buf_set_type(buf, BT_BUF_ACL_IN);
			memcpy(buf->data, &hdr, hdr_len);
		} else if (IS_ENABLED(CONFIG_BT_ISO) && type == H4_ISO) {
			buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_FOREVER);
			if (buf == NULL)
				break;

			data_len = hdr.iso.len;
			evt_flags = BT_HCI_EVT_FLAG_RECV;
			bt_buf_set_type(buf, BT_BUF_ISO_IN);
			memcpy(buf->data, &hdr, hdr_len);
		} else {
			BT_ERR("Unknown HCI type!");
			break;
        }

		if (data_len + hdr_len > buf->size) {
			net_buf_unref(buf);
			continue;
		}

		ret = h4_recv_data(buf->data + hdr_len, data_len);
		if (ret != data_len)
			break;

		net_buf_add(buf, hdr_len + data_len);

		h4_data_dump("BT: RX", type, buf->data, hdr_len + data_len);

#ifdef CONFIG_BT_HCI_BTSNOOP
		uint8_t dir = H4_DIR_RX;
		uint32_t h4_len, flags;
		h4_len = hdr_len + data_len + 1;
		flags = dir | (type == H4_CMD || type == H4_EVT)<<1;
		btsnp_pkt_create_send(dir, h4_len, flags, type, buf->data);
#endif

#if defined(CONFIG_BT_RECV_IS_RX_THREAD)
        if (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO) {
            bt_recv_prio(buf);
        } else
#else
        ARG_UNUSED(evt_flags);
#endif
        {
            bt_recv(buf); //evt_flags & BT_HCI_EVT_FLAG_RECV
        }
	}

	/* Thread Self-Clean Flow */
	if (buf)
		net_buf_unref(buf);

    BT_DBG("h4_rx_thread Exited!");
}

static int h4_open(void)
{
	int ret = 0;
    k_tid_t thread;

	BT_DBG("");

    if (!h4) {
        h4 = k_malloc(sizeof(struct hci_h4));
        if (!h4)
            return -EINVAL;
        memset(h4, 0, sizeof(struct hci_h4));
    }

    k_sem_init(&h4->rx_ind_sema, 0, 1);
	thread = k_thread_create(&h4->rx_thread_data, rx_thread_stack,
    			K_KERNEL_STACK_SIZEOF(rx_thread_stack),
    			h4_rx_thread, NULL, NULL, NULL,
    			K_PRIO_COOP(CONFIG_BT_RX_PRIO),
    			0, K_NO_WAIT);
    if (!thread) {
        k_free(h4);
        return -EINVAL;
    }
    k_thread_name_set(&h4->rx_thread_data, "hci_rx");
    
    /* MUST Ensure UART_RX is NOT enable before h4_open */
    hci_uart_set_rx_ind(h4_recv_ind);
    hci_uart_set_irq(UART_RX, 1);

	return ret;
}

static int h4_close(void)
{
    k_thread_stop(&h4->rx_thread_data, (k_thread_exit_t)k_sem_give, &h4->rx_ind_sema);
    k_sem_free(&h4->rx_ind_sema);

    if (h4)
        k_free(h4);
    h4 = NULL;

    return 0;
}

static int h4_send(struct net_buf *buf)
{
	uint8_t *type;
	int ret;

	type = net_buf_push(buf, 1);

	switch (bt_buf_get_type(buf)) {
		case BT_BUF_ACL_OUT:
			*type = H4_ACL;
			break;
		case BT_BUF_CMD:
			*type = H4_CMD;
			break;
		case BT_BUF_ISO_OUT:
			*type = H4_ISO;
			break;
		default:
			ret = -EINVAL;
			goto bail;
	}

	h4_data_dump("BT: TX", *type, buf->data + 1, buf->len - 1);

#ifdef CONFIG_BT_HCI_BTSNOOP
	uint8_t dir;
	uint32_t h4_len, flags;
	dir = H4_DIR_TX;
	h4_len = buf->len;
	flags = dir | (*type == H4_CMD || *type == H4_EVT)<<1;
	btsnp_pkt_create_send(dir, h4_len, flags, *type, buf->data);
#endif

    uint16_t len = buf->len;
	ret = hci_uart_send(buf->data, len);
	if (ret != len)
		ret = -EINVAL;

bail:
	net_buf_unref(buf);

	return ret < 0 ? ret : 0;
}

static struct bt_hci_driver driver = {
    .name   = "H:4",
    .bus    = BT_HCI_DRIVER_BUS_UART,
    .open   = h4_open,
    .close  = h4_close,
    .send   = h4_send,
#if defined(CONFIG_BT_QUIRK_NO_RESET)
    .quirks = BT_QUIRK_NO_RESET,
#endif
};

int bt_uart_init(void)
{
	return bt_hci_driver_register(&driver);
}

int bt_uart_deinit(void)
{
    return bt_hci_driver_unregister();
}

