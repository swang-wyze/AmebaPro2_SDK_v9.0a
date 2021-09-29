#ifndef _BLE_ISOCH_DEF_H_
#define _BLE_ISOCH_DEF_H_

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

#define DATA_PATH_ADD_INPUT  0x00 //(Host to Controller)
#define DATA_PATH_ADD_OUTPUT 0x01 //(Controller to Host)

#define DATA_PATH_INPUT_FLAG  0x01 //(Host to Controller)
#define DATA_PATH_OUTPUT_FLAG 0x02 //(Controller to Host)
#define DATA_PATH_MASK        0x03

#define DATA_PATH_ID_HCI     0x00

typedef enum {
	ISOCH_STATE_IDLE,
	ISOCH_STATE_CONNECTING,
	ISOCH_STATE_CONN,
	ISOCH_STATE_DISCONNECTING,
	ISOCH_STATE_CONNECTING_CANCEL,
} T_ISOCH_STATE;

typedef enum {
	ISOCH_ROLE_INITIATOR,
	ISOCH_ROLE_ACCEPTOR,
} T_ISOCH_ROLE;

typedef struct {
	T_ISOCH_STATE state;
	T_ISOCH_ROLE  role;
	uint8_t       data_path_flags;
	uint8_t       data_path_adding_path;
	bool          data_path_adding;
	uint8_t       data_path_removing_path;
	bool          data_path_removing;

	uint32_t cig_sync_delay;
	uint32_t cis_sync_delay;
	uint32_t transport_latency_m_to_s;
	uint32_t transport_latency_s_to_m;
	uint8_t phy_m_to_s;
	uint8_t phy_s_to_m;
	uint8_t nse;
	uint8_t bn_m_to_s;
	uint8_t bn_s_to_m;
	uint8_t ft_m_to_s;
	uint8_t ft_s_to_m;
	uint16_t max_pdu_m_to_s;
	uint16_t max_pdu_s_to_m;
	uint16_t iso_interval;

	bool acceptor_config_sdu_flag;
	uint32_t sdu_interval_m_to_s;  /* Valid for initiator role. Valid for acceptor role if acceptor_config_sdu_flag is true */
	uint32_t sdu_interval_s_to_m;  /* Valid for initiator role. Valid for acceptor role if acceptor_config_sdu_flag is true */
	uint16_t max_sdu_m_to_s;       /* Valid for initiator role. Valid for acceptor role if acceptor_config_sdu_flag is true */
	uint16_t max_sdu_s_to_m;       /* Valid for initiator role. Valid for acceptor role if acceptor_config_sdu_flag is true */
} T_ISOCH_INFO;

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif

#endif
