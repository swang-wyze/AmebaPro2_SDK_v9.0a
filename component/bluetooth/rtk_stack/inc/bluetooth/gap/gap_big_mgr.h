/**
*********************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      gap_big_mgr.h
* @brief     Header file for BIG
* @details   This file defines BIG related API.
* @author
* @date
* @version
* *********************************************************************************************************
*/

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef GAP_BIG_MGR_H
#define GAP_BIG_MGR_H

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef __cplusplus
extern "C"
{
#endif

#if F_BT_LE_5_2_ISOC_BIS_SUPPORT
#include "gap.h"

/*============================================================================*
 *                        Header Files
 *============================================================================*/


#define BIG_HANDLE_MIN 0x20
#define BIG_HANDLE_MAX 0xEF

#define GAP_BIG_MGR_MAX_BIS_NUM 4

#define GAP_INVALID_BIG_HANDLE   0xFF

/** @defgroup GAP_BIG_MGR_SYNC_RECEIVER_CREATE_SYNC_DEV_STATE GAP synchronized receiver device State
  * @{
  */
#define GAP_BIG_MGR_SYNC_RECEIVER_CREATE_SYNC_DEV_STATE_IDLE              0   //!< Idle
#define GAP_BIG_MGR_SYNC_RECEIVER_CREATE_SYNC_DEV_STATE_SYNCHRONIZING     1   //!< Synchronizing
/** @} End GAP_BIG_MGR_SYNC_RECEIVER_CREATE_SYNC_DEV_STATE */

/*****************************callback*************************/
//gap_big_mgr.h
#define MSG_BIG_MGR_SETUP_DATA_PATH       0x60 //gap_big_mgr_setup_data_path
#define MSG_BIG_MGR_REMOVE_DATA_PATH      0x61 //gap_big_mgr_remove_data_path

#define MSG_BIG_MGR_ISO_TEST_END          0x70 //gap_big_mgr_iso_test_end
#define MSG_BIG_MGR_TRANSMIT_TEST         0x74 //gap_big_mgr_transmit_test
#define MSG_BIG_MGR_RECEIVE_TEST          0x78 //gap_big_mgr_receive_test
#define MSG_BIG_MGR_READ_TEST_COUNTERS    0x79 //gap_big_mgr_read_test_counters

#define MSG_BIG_MGR_ISOC_BROADCAST_STATE_CHANGE_INFO          0x80 //!< Isochronous broadcasting state change info
#define MSG_BIG_MGR_ISOC_BROADCASTER_CREATE_BIG_CMPL_INFO     0x81 //Info of gap_big_mgr_isoc_broadcaster_create_big
#define MSG_BIG_MGR_READ_ISO_TX_SYNC                          0x82 //gap_big_mgr_read_iso_tx_sync

#define MSG_BIG_MGR_SYNC_RECEIVER_DEV_STATE_CHANGE_INFO       0x90 //!< Synchronized receiver device state change info
#define MSG_BIG_MGR_SYNC_RECEIVER_SYNC_STATE_CHANGE_INFO      0x91 //!< Synchronized receiver synchronous state change info
#define MSG_BIG_MGR_SYNC_RECEIVER_BIG_SYNC_ESTABLISHED_INFO   0x92 //Info of gap_big_mgr_sync_receiver_big_create_sync
#define MSG_BIG_MGR_READ_ISO_LINK_QUALITY                     0x93 //gap_big_mgr_read_iso_link_quality

typedef enum {
	BIG_MGR_ROLE_ISOC_BROADCASTER = 1,
	BIG_MGR_ROLE_SYNC_RECEIVER = 2,
} T_GAP_BIG_MGR_ROLE;

#if F_BT_LE_5_2_ISOC_BIS_BROADCASTER_SUPPORT
typedef enum {
	BIG_ISOC_BROADCAST_STATE_IDLE = 0x00,
	BIG_ISOC_BROADCAST_STATE_CREATING_EXT_ADV_STATE_PA_ADV_STATE_IDLE = 0x01,
	BIG_ISOC_BROADCAST_STATE_WAIT_EXT_ADV_STATE_PA_ADV_STATE_ADVERTISING = 0x02,
	BIG_ISOC_BROADCAST_STATE_CREATING_PA_ADV_STATE_IDLE = 0x03,
	BIG_ISOC_BROADCAST_STATE_WAIT_PA_ADV_STATE_ADVERTISING = 0x04,
	BIG_ISOC_BROADCAST_STATE_CREATING_EXT_ADV_STATE_IDLE = 0x05,
	BIG_ISOC_BROADCAST_STATE_WAIT_EXT_ADV_STATE_ADVERTISING = 0x06,
	BIG_ISOC_BROADCAST_STATE_CREATING = 0x07,
	BIG_ISOC_BROADCAST_STATE_BROADCASTING = 0x08,
	BIG_ISOC_BROADCAST_STATE_TERMINATING = 0x09,
} T_GAP_BIG_ISOC_BROADCAST_STATE;
#endif

#if F_BT_LE_5_2_ISOC_BIS_RECEIVER_SUPPORT
typedef enum {
	BIG_SYNC_RECEIVER_SYNC_STATE_TERMINATED = 0x00,
	BIG_SYNC_RECEIVER_SYNC_STATE_SYNCHRONIZING = 0x01,
	BIG_SYNC_RECEIVER_SYNC_STATE_SYNCHRONIZED = 0x02,
	BIG_SYNC_RECEIVER_SYNC_STATE_TERMINATING = 0x03,
} T_GAP_BIG_SYNC_RECEIVER_SYNC_STATE;
#endif

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint8_t  data_path_adding_path;
} T_BIG_MGR_SETUP_DATA_PATH_RSP;

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint8_t  data_path_removing_path;
} T_BIG_MGR_REMOVE_DATA_PATH_RSP;

typedef struct {
	uint8_t bis_idx;
	uint16_t bis_conn_handle;
} T_BIG_MGR_BIS_CONN_HANDLE_INFO;

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_SUPPORT
typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint32_t received_packet_count;
	uint32_t missed_packet_count;
	uint32_t failed_packet_count;
} T_BIG_MGR_ISO_TEST_END_RSP;
#endif

#if F_BT_LE_5_2_ISOC_BIS_BROADCASTER_SUPPORT
typedef struct {
	uint8_t num_bis;
	uint32_t sdu_interval;/*3bytes*/
	uint16_t max_sdu;
	uint16_t max_transport_latency;
	uint8_t rtn;
	uint8_t phy; /* bit-field */
	uint8_t  packing;
	uint8_t  framing;
	uint8_t encryption;
	uint8_t broadcast_code[16];
} T_BIG_MGR_ISOC_BROADCASTER_CREATE_BIG_PARAM;

typedef struct {
	uint16_t cause;
	uint8_t  big_handle;
	uint8_t adv_handle;
	uint32_t big_sync_delay;
	uint32_t transport_latency_big;
	uint8_t phy;
	uint8_t nse;
	uint8_t bn;
	uint8_t pto;
	uint8_t irc;
	uint16_t max_pdu;
	uint16_t iso_interval;
	uint8_t num_bis;
	T_BIG_MGR_BIS_CONN_HANDLE_INFO bis_conn_handle_info[GAP_BIG_MGR_MAX_BIS_NUM];
} T_BIG_MGR_ISOC_BC_CREATE_BIG_CMPL_INFO;

typedef struct {
	uint16_t cause;
	uint8_t big_handle;
	uint8_t adv_handle;
	T_GAP_BIG_ISOC_BROADCAST_STATE new_state;
} T_BIG_MGR_ISOC_BROADCAST_STATE_CHANGE_INFO;

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint8_t adv_handle;
	uint16_t packet_sequence_number;
	uint32_t time_stamp;
	uint32_t time_offset;
} T_BIG_MGR_READ_ISO_TX_SYNC_RSP;

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_BROADCASTER_SUPPORT
typedef struct {
	uint8_t num_bis;
	uint32_t sdu_interval;/*3bytes*/
	uint16_t iso_interval;
	uint8_t nse;
	uint16_t max_sdu;
	uint16_t max_pdu;
	uint8_t phy; /* bit-field */
	uint8_t packing;
	uint8_t framing;
	uint8_t bn;
	uint8_t irc;
	uint8_t pto;
	uint8_t encryption;
	uint8_t broadcast_code[16];
} T_BIG_MGR_ISOC_BROADCASTER_CREATE_BIG_TEST_PARAM;

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint8_t adv_handle;
} T_BIG_MGR_TRANSMIT_TEST_RSP;
#endif
#endif

#if F_BT_LE_5_2_ISOC_BIS_RECEIVER_SUPPORT
typedef struct {
	uint8_t gap_big_create_sync_state: 1;  //!< @ref GAP_BIG_MGR_SYNC_RECEIVER_CREATE_SYNC_DEV_STATE
	uint8_t gap_big_sync_receiver_rfu_state: 7;  //!< @ref reserved for future
} T_BIG_MGR_SYNC_RECEIVER_DEV_STATE;

typedef struct {
	T_BIG_MGR_SYNC_RECEIVER_DEV_STATE  state;
	uint16_t cause;
} T_BIG_MGR_SYNC_RECEIVER_DEV_STATE_CHANGE_INFO;

typedef struct {
	uint16_t cause;
	uint8_t big_handle;
	uint8_t sync_id;
	uint16_t  sync_handle;
	T_GAP_BIG_SYNC_RECEIVER_SYNC_STATE state;
} T_BIG_MGR_SYNC_RECEIVER_SYNC_STATE_CHANGE_INFO;

typedef struct {
	uint8_t encryption;
	uint8_t broadcast_code[16];
	uint8_t mse;
	uint16_t big_sync_timeout;
	uint8_t  num_bis;
	uint8_t  bis[GAP_BIG_MGR_MAX_BIS_NUM];
} T_BIG_MGR_SYNC_RECEIVER_BIG_CREATE_SYNC_PARAM;

typedef struct {
	uint16_t cause;
	uint8_t  big_handle;
	uint8_t sync_id;
	uint16_t sync_handle;
	uint32_t transport_latency_big;
	uint8_t nse;
	uint8_t bn;
	uint8_t pto;
	uint8_t irc;
	uint16_t max_pdu;
	uint16_t iso_interval;
	uint8_t num_bis;
	T_BIG_MGR_BIS_CONN_HANDLE_INFO bis_conn_handle_info[GAP_BIG_MGR_MAX_BIS_NUM];
} T_BIG_MGR_SYNC_RECEIVER_BIG_SYNC_ESTABLISHED_INFO;

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t  big_handle;
	uint8_t sync_id;
	uint16_t sync_handle;
	uint32_t tx_unacked_packets;
	uint32_t tx_flushed_packets;
	uint32_t tx_last_subevent_packets;
	uint32_t retransmitted_packets;
	uint32_t crc_error_packets;
	uint32_t rx_unreceived_packets;
	uint32_t duplicate_packets;
} T_BIG_MGR_READ_ISO_LINK_QUALITY_RSP;

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_RECEIVER_SUPPORT
typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t big_handle;
	uint8_t sync_id;
	uint16_t  sync_handle;
} T_BIG_MGR_RECEIVE_TEST_RSP;

typedef struct {
	uint16_t cause;
	uint16_t bis_conn_handle;
	uint8_t big_handle;
	uint8_t sync_id;
	uint16_t  sync_handle;
	uint32_t received_packet_count;
	uint32_t missed_packet_count;
	uint32_t failed_packet_count;
} T_BIG_MGR_READ_TEST_COUNTERS_RSP;
#endif
#endif

typedef union {
	T_BIG_MGR_SETUP_DATA_PATH_RSP                  *p_big_mgr_setup_data_path_rsp;
	T_BIG_MGR_REMOVE_DATA_PATH_RSP                 *p_big_mgr_remove_data_path_rsp;
#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_SUPPORT
	T_BIG_MGR_ISO_TEST_END_RSP                     *p_big_mgr_iso_test_end_rsp;
#endif

#if F_BT_LE_5_2_ISOC_BIS_BROADCASTER_SUPPORT
	T_BIG_MGR_ISOC_BROADCAST_STATE_CHANGE_INFO     *p_big_mgr_isoc_broadcast_state_change_info;
	T_BIG_MGR_ISOC_BC_CREATE_BIG_CMPL_INFO         *p_big_mgr_isoc_bc_create_big_cmpl_info;
	T_BIG_MGR_READ_ISO_TX_SYNC_RSP                 *p_big_mgr_read_iso_tx_sync_rsp;
#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_BROADCASTER_SUPPORT
	T_BIG_MGR_TRANSMIT_TEST_RSP                    *p_big_mgr_transmit_test_rsp;
#endif
#endif
#if F_BT_LE_5_2_ISOC_BIS_RECEIVER_SUPPORT
	T_BIG_MGR_SYNC_RECEIVER_DEV_STATE_CHANGE_INFO
	*p_big_mgr_sync_receiver_dev_state_change_info;
	T_BIG_MGR_SYNC_RECEIVER_SYNC_STATE_CHANGE_INFO
	*p_big_mgr_sync_receiver_sync_state_change_info;
	T_BIG_MGR_SYNC_RECEIVER_BIG_SYNC_ESTABLISHED_INFO
	*p_big_mgr_sync_rx_big_sync_established_info;
	T_BIG_MGR_READ_ISO_LINK_QUALITY_RSP    *p_big_mgr_read_iso_link_quality_rsp;
#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_RECEIVER_SUPPORT
	T_BIG_MGR_RECEIVE_TEST_RSP             *p_big_mgr_receive_test_rsp;
	T_BIG_MGR_READ_TEST_COUNTERS_RSP       *p_big_mgr_read_test_counters_rsp;
#endif
#endif
} T_BIG_MGR_CB_DATA;

T_GAP_CAUSE gap_big_mgr_init(uint8_t big_handle_num, uint8_t bis_num);

T_GAP_CAUSE gap_big_mgr_setup_data_path(uint16_t bis_conn_handle, uint8_t data_path_direction,
										uint8_t data_path_id, uint8_t codec_id[5], uint32_t controller_delay/*3bytes*/,
										uint8_t codec_config_len, uint8_t *p_codec_config);

T_GAP_CAUSE gap_big_mgr_remove_data_path(uint16_t bis_conn_handle, uint8_t data_path_direction);

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_SUPPORT
T_GAP_CAUSE gap_big_mgr_iso_test_end(uint16_t bis_conn_handle);
#endif

#if F_BT_LE_5_2_ISOC_BIS_BROADCASTER_SUPPORT
typedef T_APP_RESULT(*P_FUN_LE_BIG_MGR_ISOC_BC_CB)(uint8_t big_handle, uint8_t cb_type,
		void *p_cb_data);

T_GAP_CAUSE gap_big_mgr_isoc_broadcaster_init(uint8_t big_num, uint8_t bis_num,
		P_FUN_LE_BIG_MGR_ISOC_BC_CB cb_pfn);
T_GAP_CAUSE gap_big_mgr_isoc_broadcaster_create_big(uint8_t adv_handle,
		T_BIG_MGR_ISOC_BROADCASTER_CREATE_BIG_PARAM *p_create_big_param, uint8_t *p_big_handle);
T_GAP_CAUSE gap_big_mgr_isoc_broadcaster_terminate_big(uint8_t big_handle, uint8_t reason);

T_GAP_CAUSE gap_big_mgr_read_iso_tx_sync(uint16_t bis_conn_handle);

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_BROADCASTER_SUPPORT
T_GAP_CAUSE gap_big_mgr_isoc_broadcaster_create_big_test(uint8_t adv_handle,
		T_BIG_MGR_ISOC_BROADCASTER_CREATE_BIG_TEST_PARAM *p_create_big_test_param, uint8_t *p_big_handle);

T_GAP_CAUSE gap_big_mgr_transmit_test(uint16_t bis_conn_handle, uint8_t payload_type);
#endif
#endif

#if F_BT_LE_5_2_ISOC_BIS_RECEIVER_SUPPORT
typedef T_APP_RESULT(*P_FUN_LE_BIG_MGR_SYNC_RX_CB)(uint8_t big_handle, uint8_t cb_type,
		void *p_cb_data);

T_GAP_CAUSE gap_big_mgr_sync_receiver_init(uint8_t big_num, uint8_t bis_num,
		P_FUN_LE_BIG_MGR_SYNC_RX_CB cb_pfn);
T_GAP_CAUSE gap_big_mgr_sync_receiver_big_create_sync(uint16_t sync_handle,
		T_BIG_MGR_SYNC_RECEIVER_BIG_CREATE_SYNC_PARAM *p_big_create_sync_param, uint8_t *p_big_handle);
T_GAP_CAUSE gap_big_mgr_sync_receiver_big_terminate_sync(uint8_t big_handle);

T_GAP_CAUSE gap_big_mgr_read_iso_link_quality(uint16_t bis_conn_handle);

#if F_BT_LE_5_2_ISOC_TEST_MODE_BIS_RECEIVER_SUPPORT
T_GAP_CAUSE gap_big_mgr_receive_test(uint16_t bis_conn_handle, uint8_t payload_type);

T_GAP_CAUSE gap_big_mgr_read_test_counters(uint16_t bis_conn_handle);
#endif
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif /* GAP_BIG_MGR_H */
