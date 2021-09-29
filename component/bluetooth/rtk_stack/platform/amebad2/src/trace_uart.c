/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#include "trace_uart.h"

extern void LOGUART_BT_SendData(uint8_t *InBuf, uint32_t Count);

bool trace_print(void *data, uint16_t len)
{
	LOGUART_BT_SendData(data, len);
	return true;
}
