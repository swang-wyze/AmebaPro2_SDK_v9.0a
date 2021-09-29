#ifndef _CIG_MGR_H_
#define _CIG_MGR_H_

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

#include "gap.h"
#include "ble_isoch_def.h"

#define CIG_MAX_NUM 4
#define CIS_MAX_NUM 4

#define CIG_ID_MIN 0x00
#define CIG_ID_MAX 0xEF

#define CIS_ID_MIN 0x00
#define CIS_ID_MAX 0xEF

#define CONTROLLER_DELAY_MIN 0x000000
#define CONTROLLER_DELAY_MAX 0x3D0900

/*****************************callback*************************/
//gap_cig_mgr.h
#define MSG_CIG_MGR_SETUP_DATA_PATH       0x01 //cig_mgr_setup_data_path
#define MSG_CIG_MGR_REMOVE_DATA_PATH      0x02 //cig_mgr_remove_data_path
#define MSG_CIG_MGR_READ_ISO_TX_SYNC      0x03 //cig_mgr_read_iso_tx_sync
#define MSG_CIG_MGR_READ_ISO_LINK_QUALITY 0x04 //cig_mgr_read_iso_link_quality

#define MSG_CIG_MGR_ISO_TEST_END          0x10 //cig_mgr_iso_test_end
#define MSG_CIG_MGR_TRANSMIT_TEST         0x11 //cig_mgr_transmit_test
#define MSG_CIG_MGR_RECEIVE_TEST          0x12 //cig_mgr_receive_test
#define MSG_CIG_MGR_READ_TEST_COUNTERS    0x13 //cig_mgr_read_test_counters

#define MSG_CIG_MGR_CIS_ESTABLISHED_INFO  0x20
#define MSG_CIG_MGR_DISCONNECT            0x21 //cig_mgr_disconnect
#define MSG_CIG_MGR_DISCONNECT_INFO       0x22

#define MSG_CIG_MGR_START_SETTING         0x30 //cig_mgr_start_setting
#define MSG_CIG_MGR_CREATE_CIS            0x31 //cig_mgr_create_cis_by_cig_id or cig_mgr_create_cis_by_cis_conn_handle
#define MSG_CIG_MGR_REMOVE_CIG            0x32 //cig_mgr_remove_cig

#define MSG_CIG_MGR_START_SETTING_TEST    0x3A //cig_mgr_start_setting_test

#define MSG_CIG_MGR_CIS_REQUEST_IND       0x40
#define MSG_CIG_MGR_ACCEPT_CIS_INFO       0x41
#define MSG_CIG_MGR_REJECT_CIS_INFO       0x42

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint8_t data_path_adding_path;
} T_CIG_MGR_SETUP_DATA_PATH_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint8_t data_path_removing_path;
} T_CIG_MGR_REMOVE_DATA_PATH_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint16_t packet_sequence_number;
	uint32_t time_stamp;
	uint32_t time_offset;
} T_CIG_MGR_READ_ISO_TX_SYNC_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint32_t tx_unacked_packets;
	uint32_t tx_flushed_packets;
	uint32_t tx_last_subevent_packets;
	uint32_t retransmitted_packets;
	uint32_t crc_error_packets;
	uint32_t rx_unreceived_packets;
	uint32_t duplicate_packets;
} T_CIG_MGR_READ_ISO_LINK_QUALITY_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint32_t received_packet_count;
	uint32_t missed_packet_count;
	uint32_t failed_packet_count;
} T_CIG_MGR_ISO_TEST_END_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_TRANSMIT_TEST_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_RECEIVE_TEST_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint32_t received_packet_count;
	uint32_t missed_packet_count;
	uint32_t failed_packet_count;
} T_CIG_MGR_READ_TEST_COUNTERS_RSP;

typedef struct {
	uint16_t cause;
	uint8_t  conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
	uint16_t cis_conn_handle;
	uint32_t cig_sync_delay;
	uint32_t cis_sync_delay;
	uint32_t transport_latency_m_s;
	uint32_t transport_latency_s_m;
	uint8_t phy_m_s;
	uint8_t phy_s_m;
	uint8_t nse;
	uint8_t bn_m_s;
	uint8_t bn_s_m;
	uint8_t ft_m_s;
	uint8_t ft_s_m;
	uint16_t max_pdu_m_s;
	uint16_t max_pdu_s_m;
	uint16_t iso_interval;
} T_CIG_MGR_CIS_ESTABLISHED_INFO;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_DISCONNECT_RSP;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_DISCONNECT_INFO;

typedef struct {
	uint8_t       cis_id;
	uint16_t      cis_conn_handle;
} T_CIG_START_SETTING_INFO;

typedef struct {
	uint16_t cause;
	uint8_t  cig_id;
	uint8_t  cis_count;
	T_CIG_START_SETTING_INFO  cis_info[8];
} T_CIG_MGR_START_SETTING_RSP;

typedef struct {
	uint8_t       cig_id;
	uint8_t       cis_id;
	T_ISOCH_STATE state;
	uint16_t      cis_conn_handle;
} T_CIS_CREATE_INFO;

typedef struct {
	uint16_t          cause;
	uint8_t           cis_count;
	T_CIS_CREATE_INFO cis_info[8];
} T_CIG_MGR_CREATE_CIS_RSP;

typedef struct {
	uint16_t cause;
	uint8_t cig_id;
} T_CIG_MGR_REMOVE_CIG_RSP;

typedef struct {
	uint16_t cause;
	uint8_t  cig_id;
	uint8_t  cis_count;
	T_CIG_START_SETTING_INFO  cis_info[8];
} T_CIG_MGR_START_SETTING_TEST_RSP;

typedef struct {
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_CIS_REQUEST_IND;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_ACCEPT_CIS_INFO;

typedef struct {
	uint16_t cause;
	uint16_t cis_conn_handle;
	uint8_t conn_id;
	uint8_t cig_id;
	uint8_t cis_id;
} T_CIG_MGR_REJECT_CIS_INFO;

typedef union {
	T_CIG_MGR_SETUP_DATA_PATH_RSP          *p_cig_mgr_setup_data_path_rsp;
	T_CIG_MGR_REMOVE_DATA_PATH_RSP         *p_cig_mgr_remove_data_path_rsp;
	T_CIG_MGR_READ_ISO_TX_SYNC_RSP         *p_cig_mgr_read_iso_tx_sync_rsp;
	T_CIG_MGR_READ_ISO_LINK_QUALITY_RSP    *p_cig_mgr_read_iso_link_quality_rsp;

	T_CIG_MGR_ISO_TEST_END_RSP             *p_cig_mgr_iso_test_end_rsp;
	T_CIG_MGR_TRANSMIT_TEST_RSP            *p_cig_mgr_transmit_test_rsp;
	T_CIG_MGR_RECEIVE_TEST_RSP             *p_cig_mgr_receive_test_rsp;
	T_CIG_MGR_READ_TEST_COUNTERS_RSP       *p_cig_mgr_read_test_counters_rsp;

	T_CIG_MGR_CIS_ESTABLISHED_INFO         *p_cig_mgr_cis_established_info;
	T_CIG_MGR_DISCONNECT_RSP               *p_cig_mgr_disconnect_rsp;
	T_CIG_MGR_DISCONNECT_INFO              *p_cig_mgr_disconnect_info;

	T_CIG_MGR_START_SETTING_RSP            *p_cig_mgr_start_setting_rsp;
	T_CIG_MGR_CREATE_CIS_RSP               *p_cig_mgr_create_cis_rsp;
	T_CIG_MGR_REMOVE_CIG_RSP               *p_cig_mgr_remove_cig_rsp;
	T_CIG_MGR_START_SETTING_TEST_RSP       *p_cig_mgr_start_setting_test_rsp;

	T_CIG_MGR_CIS_REQUEST_IND              *p_cig_mgr_cis_request_ind;
	T_CIG_MGR_ACCEPT_CIS_INFO              *p_cig_mgr_accept_cis_info;
	T_CIG_MGR_REJECT_CIS_INFO              *p_cig_mgr_reject_cis_info;
} T_CIG_MGR_CB_DATA;

typedef struct {
	uint8_t cis_id;
	uint8_t cig_id;
} T_CIS_INFO;

typedef struct {
	uint32_t sdu_interval_m_s;/*3bytes*/
	uint32_t sdu_interval_s_m;/*3bytes*/
	uint8_t  sca;
	uint8_t  packing;
	uint8_t  framing;
	uint16_t latency_m_s;
	uint16_t latency_s_m;
} T_CIG_CFG_PARAM;

typedef struct {
	uint16_t max_sdu_m_s;
	uint16_t max_sdu_s_m;
	uint8_t  phy_m_s;
	uint8_t  phy_s_m;
	uint8_t  rtn_m_s;
	uint8_t  rtn_s_m;
} T_CIS_CFG_PARAM;

typedef struct {
	uint32_t sdu_interval_m_s;/*3bytes*/
	uint32_t sdu_interval_s_m;/*3bytes*/
	uint8_t  ft_m_s;
	uint8_t  ft_s_m;
	uint16_t iso_interval;
	uint8_t sca;
	uint8_t  packing;
	uint8_t  framing;
} T_CIG_CFG_PARAM_TEST;

typedef struct {
	uint8_t             nse;
	uint16_t            max_sdu_m_s;
	uint16_t            max_sdu_s_m;
	uint16_t            max_pdu_m_s;
	uint16_t            max_pdu_s_m;
	uint8_t             phy_m_s;
	uint8_t             phy_s_m;
	uint8_t             bn_m_s;
	uint8_t             bn_s_m;
} T_CIS_CFG_PARAM_TEST;

typedef T_APP_RESULT(*P_FUN_LE_ISOCH_CB)(uint8_t cb_type, void *p_cb_data);
typedef T_APP_RESULT(*P_FUN_LE_CIG_MGR_CB)(uint8_t cig_id, uint8_t cb_type, void *p_cb_data);
typedef T_APP_RESULT(*P_FUN_LE_CIG_MGR_ACCEPTOR_CB)(uint8_t cb_type, void *p_cb_data);

T_GAP_CAUSE cig_mgr_init(uint8_t cig_num, uint8_t cis_num);

bool cig_mgr_get_conn_id(uint16_t cis_conn_handle, uint8_t *p_conn_id);
bool cig_mgr_get_cis_info(uint16_t cis_conn_handle, T_CIS_INFO *p_info);
bool cig_mgr_get_isoch_info(uint16_t cis_conn_handle, T_ISOCH_INFO *p_info);
void cig_mgr_print_info(uint8_t cig_id);//just for debug

/* For initiator and acceptor */
T_GAP_CAUSE cig_mgr_setup_data_path(uint16_t cis_conn_handle, uint8_t path_direction,
									uint8_t path_id,
									uint8_t codec_id[5], uint32_t controller_delay/*3bytes*/,
									uint8_t codec_len, uint8_t *p_codec_data);
T_GAP_CAUSE cig_mgr_remove_data_path(uint16_t cis_conn_handle, uint8_t path_direction);
T_GAP_CAUSE cig_mgr_read_iso_tx_sync(uint16_t cis_conn_handle);
T_GAP_CAUSE cig_mgr_read_iso_link_quality(uint16_t cis_conn_handle);
T_GAP_CAUSE cig_mgr_disconnect(uint16_t cis_conn_handle, uint8_t reason);

/* ISO Test mode*/
T_GAP_CAUSE cig_mgr_iso_test_end(uint16_t cis_conn_handle);
T_GAP_CAUSE cig_mgr_transmit_test(uint16_t cis_conn_handle, uint8_t payload_type);
T_GAP_CAUSE cig_mgr_receive_test(uint16_t cis_conn_handle, uint8_t payload_type);
T_GAP_CAUSE cig_mgr_read_test_counters(uint16_t cis_conn_handle);

/* Only for initiator */
T_GAP_CAUSE cig_mgr_reg_cig(uint8_t cig_id, P_FUN_LE_CIG_MGR_CB cb_pfn);
T_GAP_CAUSE cig_mgr_remove_cig(uint8_t cig_id);
T_GAP_CAUSE cig_mgr_add_cis(uint8_t cig_id, uint8_t cis_id);
T_GAP_CAUSE cig_mgr_set_cig_param(uint8_t cig_id, T_CIG_CFG_PARAM *p_param);
T_GAP_CAUSE cig_mgr_set_cis_param(uint8_t cis_id, T_CIS_CFG_PARAM *p_param);
T_GAP_CAUSE cig_mgr_set_cis_acl_link(uint8_t cis_id, uint8_t conn_id);
T_GAP_CAUSE cig_mgr_start_setting(uint8_t cig_id);
T_GAP_CAUSE cig_mgr_create_cis_by_cig_id(uint8_t cig_id);
T_GAP_CAUSE cig_mgr_create_cis_by_cis_conn_handle(uint8_t cig_id, uint8_t cis_count,
		uint16_t *p_cis_conn_handle);
bool cig_mgr_get_cis_handle(uint8_t cis_id, uint16_t *p_conn_handle);

/* ISO Test mode for initiator */
T_GAP_CAUSE cig_mgr_set_cig_param_test(uint8_t cig_id, T_CIG_CFG_PARAM_TEST *p_param);
T_GAP_CAUSE cig_mgr_set_cis_param_test(uint8_t cis_id, T_CIS_CFG_PARAM_TEST *p_param);
T_GAP_CAUSE cig_mgr_start_setting_test(uint8_t cig_id);

/* Only for acceptor */
void cig_mgr_reg_acceptor_cb(P_FUN_LE_CIG_MGR_ACCEPTOR_CB cb_pfn);
T_GAP_CAUSE cig_mgr_acceptor_config_sdu_param(uint16_t cis_conn_handle,
		bool acceptor_config_sdu_flag, uint32_t sdu_interval_m_s, uint32_t sdu_interval_s_m,
		uint16_t max_sdu_m_s, uint16_t max_sdu_s_m);

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif

#endif
