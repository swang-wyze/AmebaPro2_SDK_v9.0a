/*******************************************************************************
 * Copyright (c) 2014, Realtek Semiconductor Corp.
 * All rights reserved.
 *
 * This module is a confidential and proprietary property of RealTek and
 * possession or use of this module requires written permission of RealTek.
 *******************************************************************************
 */

#include "cmsis.h"

#if CONFIG_SW_XMODEM_EN
#include "xmport_uart.h"
extern void *_memset(void *dst0, int Val, size_t length);

u8 xmodem_rx_buf_poll(void *pxm_port)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;

	if (pxm_uart_port->xm_uart_rx_buf_in != pxm_uart_port->xm_uart_rx_buf_out) {
		return 1;
	} else {
		return 0;
	}
}

static void xmodem_rx_buf_push(void *pxm_port, char rx_c)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;

	*((u8 *)(pxm_uart_port->pxm_uart_rx_buf +  pxm_uart_port->xm_uart_rx_buf_in)) = rx_c;
	pxm_uart_port->xm_uart_rx_buf_in++;
	if (pxm_uart_port->xm_uart_rx_buf_in == pxm_uart_port->xm_uart_rx_buf_sz) {
		pxm_uart_port->xm_uart_rx_buf_in = 0;
	}
}

u8 xmodem_rx_buf_pop(void *pxm_port)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;
	u8 c;

	c = *((u8 *)(pxm_uart_port->pxm_uart_rx_buf + pxm_uart_port->xm_uart_rx_buf_out));
	pxm_uart_port->xm_uart_rx_buf_out++;
	if (pxm_uart_port->xm_uart_rx_buf_out == pxm_uart_port->xm_uart_rx_buf_sz) {
		pxm_uart_port->xm_uart_rx_buf_out = 0;
	}
	return c;
}

s32 xmodem_rx_line_status(void *pxm_port)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;
	s32 ret;

	if (pxm_uart_port->pxm_uart_adp->rx_status == 0) {
		return 0;
	} else {
		ret = -pxm_uart_port->pxm_uart_adp->rx_status;
		pxm_uart_port->pxm_uart_adp->rx_status = 0;
		return (ret);
	}
}

void xmodem_uart_rx_ind(uint32_t para, uint32_t event)
{
	xmodem_uart_port_handler_t *pxm_uart_port = (xmodem_uart_port_handler_t *)para;
	UART_Type *puart = pxm_uart_port->pxm_uart_adp->base_addr;
	uint32_t bytes_in_fifo;

	pxm_uart_port->pxm_uart_adp->rx_status |= puart->lsr & UART_LSR_ERR;

	while (1) {
		bytes_in_fifo = puart->rflvr_b.rx_fifo_lv;
		if (bytes_in_fifo > 0) {
			for (; bytes_in_fifo > 0; bytes_in_fifo--) {
				xmodem_rx_buf_push((void *)para, (char)puart->rbr);
			}
		} else {
			break;
		}
	}
}

void xmodem_uart_init(xmodem_uart_port_handler_t *pxm_uart_port, hal_uart_adapter_t *puart_adp,
					  u8 *prx_buf, u32 rx_buf_sz)
{
	_memset((void *)pxm_uart_port, 0, sizeof(xmodem_uart_port_handler_t));
	hal_rtl_uart_rxind_hook(puart_adp, xmodem_uart_rx_ind, (uint32_t)pxm_uart_port, 0);
	// Todo: this is only UART0, should config other UART port as well
	hal_rtl_irq_enable(UART0_IRQn);
	pxm_uart_port->pxm_uart_adp = puart_adp;
	pxm_uart_port->pxm_uart_rx_buf = prx_buf;
	pxm_uart_port->xm_uart_rx_buf_sz = rx_buf_sz;
}

void xmodem_uart_deinit(xmodem_uart_port_handler_t *pxm_uart_port)
{
	hal_uart_adapter_t *puart_adp = pxm_uart_port->pxm_uart_adp;

	hal_rtl_uart_rxind_hook(puart_adp, NULL, 0, 0);
	_memset((void *)pxm_uart_port, 0, sizeof(xmodem_uart_port_handler_t));
}

void xmodem_uart_func_hook(XMODEM_COM_PORT *pXComPort, xmodem_uart_port_handler_t *pxm_uart_port)
{
	pXComPort->xm_port = pxm_uart_port;
	pXComPort->poll = xmodem_rx_buf_poll;
	pXComPort->put = xmodem_uart_putc;
	pXComPort->get = xmodem_rx_buf_pop;
	pXComPort->line_status = xmodem_rx_line_status;
}

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/
char xmodem_uart_getc(void *pxm_port)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;

	return hal_rtl_uart_getc(pxm_uart_port->pxm_uart_adp);
}

void xmodem_uart_putc(void *pxm_port, char c)
{
	xmodem_uart_port_handler_t *pxm_uart_port = pxm_port;

	hal_rtl_uart_wputc(pxm_uart_port->pxm_uart_adp, c);
}

#endif // CONFIG_SW_XMODEM_EN

