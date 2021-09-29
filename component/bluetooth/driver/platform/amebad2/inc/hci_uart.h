/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#ifndef _HCI_UART_H_
#define _HCI_UART_H_

#include "hci/hci_common.h"

typedef struct
{
    /* UART Open */
    uint8_t (*open)(void);

    /* UART Close */
    uint8_t (*close)(void);

    /* UART Send */
    uint16_t (*send)(uint8_t *buf, uint16_t len);

    /* UART Read */
    uint16_t (*read)(uint8_t *buf, uint16_t len);

    /* UART Set Rx ind */
    uint8_t (*set_rx_ind)(HCI_RECV_IND rx_ind);

    /* UART Set BaudRate */
    uint8_t (*set_bdrate)(uint32_t baudrate);

} HCI_UART_OPS;

extern HCI_UART_OPS hci_uart_ops;

static inline uint8_t hci_uart_set_bdrate(uint32_t baudrate) 
{
    return hci_uart_ops.set_bdrate(baudrate);
}

static inline uint8_t hci_uart_set_rx_ind(HCI_RECV_IND rx_ind) 
{
    return hci_uart_ops.set_rx_ind(rx_ind);
}

static inline uint16_t hci_uart_send(uint8_t *buf, uint16_t len)
{
    return hci_uart_ops.send(buf, len);
}

static inline uint16_t hci_uart_read(uint8_t *buf, uint16_t len)
{
    return hci_uart_ops.read(buf, len);
}

static inline uint8_t hci_uart_open(void)
{
    return hci_uart_ops.open();
}

static inline uint8_t hci_uart_close(void)
{
    return hci_uart_ops.close();
}

#endif
