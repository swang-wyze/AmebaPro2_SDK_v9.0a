/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#ifndef ZEPHYR_INCLUDE_DEFINES_H_
#define ZEPHYR_INCLUDE_DEFINES_H_

/******************************************************************************
 * FOR INTERNAL PRIV
 */
int bt_priv_set_dbg(uint8_t level, uint8_t format);
int bt_priv_set_drv(uint8_t enable, uint8_t drv_only);

/******************************************************************************
 * FOR HOST
 */
extern k_msg_queue g_tx_message_queue;

int bt_enable_dependence(void);

int bt_disable_dependence(void);

void bt_del_hci_tx_thread(void* ctx);

#if !defined(CONFIG_BT_RECV_IS_RX_THREAD)
void bt_del_hci_rx_thread(void* ctx);
#endif

int bt_deinit(void);

/******************************************************************************
 * FOR Z_STRUCT_SECTION_FOREACH
 */
 
#endif
