/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */
#ifndef _BLE_EXT_ADV_MGR__
#define _BLE_EXT_ADV_MGR__

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BLE_EXTENDED_ADV Ble Extended Adv
  * @brief Ble extended advertising manager module
  * @{
  */


/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include "gap_ext_adv.h"
#include "gap_le.h"
#include "gap_msg.h"
/*============================================================================*
 *                              Constants
 *============================================================================*/

#define BLE_EXT_ADV_STATE_CHANGE 0x01 /**< used to notify adv set application callback function about the change of ext adv state*/
#define BLE_EXT_ADV_SET_CONN_INFO 0x02


#define BLE_EXT_ADV_MGR_VERSION   1 /**< used to set ble adv manager module version*/
/** @brief  the maximum of extend advertising event stored in extend adv queue */
#define MAX_NUM_OF_EXTEND_ADV_EVENT       0x64
/*============================================================================*
 *                         Types
 *============================================================================*/
/**
* @brief define application adv sate, for application only have two adv state :BLE_EXT_ADV_MGR_ADV_DISABLED or BLE_EXT_ADV_MGR_ADV_ENABLED
*/
typedef enum {
	BLE_EXT_ADV_MGR_ADV_DISABLED,/**< when call api ble_ext_adv_mgr_disable, the application adv state will be set to BLE_EXT_ADV_MGR_ADV_DISABLED*/
	BLE_EXT_ADV_MGR_ADV_ENABLED, /**< when call api ble_ext_adv_mgr_enable, the application adv state will be set to BLE_EXT_ADV_MGR_ADV_ENABLED*/
} T_BLE_EXT_ADV_MGR_STATE;

/**
 * @brief define the reason of  adv stop
 */
typedef enum {
	BLE_EXT_ADV_STOP_CAUSE_UNKNOWN,
	BLE_EXT_ADV_STOP_CAUSE_APP,
	BLE_EXT_ADV_STOP_CAUSE_CONN,
	BLE_EXT_ADV_STOP_CAUSE_TIMEOUT,
	BLE_EXT_ADV_STOP_CAUSE_ENABLE_FAILED,
} T_BLE_EXT_ADV_STOP_CAUSE;

/**
 * @brief used to notify application callback function about the change of adv state
 */
typedef struct {
	uint8_t                  adv_handle;
	T_BLE_EXT_ADV_MGR_STATE  state;
	T_BLE_EXT_ADV_STOP_CAUSE stop_cause; /**< Used when: BLE_EXT_ADV_MGR_ADV_DISABLED*/
	uint8_t
	app_cause; /**< Used when: BLE_EXT_ADV_MGR_ADV_DISABLED(BLE_EXT_ADV_STOP_CAUSE_APP)*/
} T_BLE_EXT_ADV_STATE_CHANGE;

typedef struct {
	uint8_t adv_handle;
	uint8_t conn_id;
} T_BLE_EXT_ADV_SET_CONN_INFO;
/**
 * @brief T_BLE_EXT_ADV_CB_DATA  @ref T_BLE_EXT_ADV_STATE_CHANGE
 */
typedef union {
	T_BLE_EXT_ADV_STATE_CHANGE  *p_ble_state_change;
	T_BLE_EXT_ADV_SET_CONN_INFO *p_ble_conn_info;
} T_BLE_EXT_ADV_CB_DATA;

/*============================================================================*
 *                              Variables
 *============================================================================*/


/*============================================================================*
 *                              Functisons
 *============================================================================*/
/**
  * @brief  Initialize parameters of non-connectable and non-scannable undirected
            advertising using extended advertising PDUs
  * @return void
  */

/**
*@brief    Allocate storage space for ble_ext_adv_set_table and create a queue to store extend advertising event
*@note     This api shall be used first if you want to use the module of ble_ext_adv_mgr
*@param[in] adv_handle_num the maximum of ble adv handle, each adv set has a adv handle
*@return   T_GAP_CAUSE
*@retval   GAP_CAUSE_SUCCESS   success
*@retval   GAP_CAUSE_INVALID_PARAM   the adv_handle_num is 0
*@retval   GAP_CAUSE_NO_RESOURCE   no space in RAM_TYPE_DATA_ON to allocate
*
 * <b>Example usage</b>
 * \code{.c}
    void app_ble_gap_init(void)
    {
        ble_ext_adv_mgr_init(MAX_ADV_SET_NUMBER);
    }
 * \endcode
*/
T_GAP_CAUSE ble_ext_adv_mgr_init(uint8_t adv_handle_num);

/**
*@brief    used to register callback for each adv set
*@note
*@param[in] app_callback @ref P_FUN_GAP_APP_CB
*@param[in] adv_handle  ble adv handle, each adv set has a adv handle
*@return    T_GAP_CAUSE @ref T_GAP_CAUSE
*/
T_GAP_CAUSE ble_ext_adv_mgr_register_callback(P_FUN_GAP_APP_CB app_callback, uint8_t adv_handle);

/**
*@brief    used to create adv handle and initialzied adv param, adv data, scan response data and random address before gap stack ready
*@note
*@param[out] adv_handle ble adv handle, each adv set has a adv handle
*@param[in] adv_event_prop @ref T_LE_EXT_ADV_LEGACY_ADV_PROPERTY
*@param[in] primary_adv_interval_min
*@param[in] primary_adv_interval_max
*@param[in] own_address_type @ref T_GAP_LOCAL_ADDR_TYPE
*@param[in] peer_address_type @ref T_GAP_REMOTE_ADDR_TYPE
*@param[in] p_peer_address  if not used, set default value NULL
*@param[in] filter_policy @ref T_GAP_ADV_FILTER_POLICY
*@param[in] adv_data_len  if you don't want to set adv data,set default length 0
*@param[in] p_adv_data if you don't want to set adv data,set default value NULL
*@param[in] adv_data_len if you don't want to set scan response data,set default length 0
*@param[in] p_scan_data if you don't want to set scan response data,set default value NULL
*@param[in] random_address if you don't want to set random_address,set default value NULL
*@return    T_GAP_CAUSE
*
 * <b>Example usage</b>
 * \code{.c}
    void le_common_adv_init(void)
    {
        ble_ext_adv_mgr_init_adv_params(&le_common_adv_handle, adv_event_prop, adv_interval_min,
                                    adv_interval_max, own_address_type, peer_address_type, peer_address,
                                    filter_policy, 23 + name_len, common_adv_data,
                                    scan_rsp_data_len + 2, scan_rsp_data, app_cfg_nv.le_random_addr);
    }
 * \endcode
*/
T_GAP_CAUSE ble_ext_adv_mgr_init_adv_params(uint8_t *adv_handle, uint16_t adv_event_prop,
		uint32_t primary_adv_interval_min, uint32_t primary_adv_interval_max,
		T_GAP_LOCAL_ADDR_TYPE own_address_type, T_GAP_REMOTE_ADDR_TYPE peer_address_type,
		uint8_t *p_peer_address,
		T_GAP_ADV_FILTER_POLICY filter_policy, uint16_t adv_data_len, uint8_t *p_adv_data,
		uint16_t scan_data_len, uint8_t *p_scan_data, uint8_t *random_address);

/**
 * @brief    enable an advertising set.
 * @note     if you want use @ref ble_ext_adv_mgr_enable() to enable a adv set, please get ble adv mgr adv state firstly,
 *           and when ble adv mgr adv state is BLE_EXT_ADV_MGR_ADV_DISABLED, you can use @ref ble_ext_adv_mgr_enable() to enable this adv set;
 *           otherwise this adv set already in enabled state,there is no need to call this api again.
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref ble_ext_adv_mgr_init_adv_params.
 * @param[in]   duration         If non-zero, indicates the duration that advertising set is enabled.
                                 0x0000:        No advertising duration.
                                 0x0001-0xFFFF: Advertising duration, in units of 10ms.
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        uint16_t duration = 0;
        if(ble_ext_adv_mgr_get_adv_state(adv_handle) == BLE_EXT_ADV_MGR_ADV_DISABLED)
        {
         ble_ext_adv_mgr_enable(adv_handle,duration);
        }
        else
        {
         APP_PRINT_INFO0("this adv set has already in enabled state");
        }
    }
 * \endcode
 */
T_GAP_CAUSE ble_ext_adv_mgr_enable(uint8_t adv_handle, uint16_t duration_10ms);

/**
 * @brief    disable an advertising set.
 * @note:    if you want use @ref ble_ext_adv_mgr_disable() to disable a adv set, please get ble adv mgr adv state firstly,
 *           and when ble adv mgr adv state is @ref BLE_EXT_ADV_MGR_ADV_ENABLED, you can use @ref ble_ext_adv_mgr_disable() to disable this adv set;
 *           otherwise this adv set already in disabled state,there is no need to call this api again.
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref ble_ext_adv_mgr_init_adv_params.
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operartion failure, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
       if(ble_ext_adv_mgr_get_adv_state(adv_handle) == BLE_EXT_ADV_MGR_ADV_ENABLED)
       {
        ble_ext_adv_mgr_disable(adv_handle,0xFF);
       }
       else
       {
        APP_PRINT_INFO0("this adv set has already in disabled state");
       }

    }
 * \endcode
 */
T_GAP_CAUSE ble_ext_adv_mgr_disable(uint8_t adv_handle, uint8_t app_cause);

/**
 * @brief    disable all advertising set.
 * @note:
 * @param[in]   app_cause       the reason of disable all adv set
 * @return none
 *
 */
T_GAP_CAUSE ble_ext_adv_mgr_disable_all(uint8_t app_cause);

/**
 * @brief       used to update adv data, this api shall be used after gap stack init ready
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref ble_ext_adv_mgr_init_adv_params.
 * @param[in]   adv_data_len
 * @param[in]   p_adv_data
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operartion failure, the advertising handle is not found.
 *
 */
T_GAP_CAUSE ble_ext_adv_mgr_set_adv_data(uint8_t adv_handle, uint16_t adv_data_len,
		uint8_t *p_adv_data);

/**
 * @brief       used to update random address, this api shall be used after stack init ready
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref ble_ext_adv_mgr_init_adv_params.
 * @param[in]   random_address
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operartion failure, the advertising handle is not found.
 *
 */
T_GAP_CAUSE ble_ext_adv_mgr_set_random(uint8_t adv_handle, uint8_t *random_address);

/**
 * @brief       used to update scan response data, this api shall be used after stack init ready
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref ble_ext_adv_mgr_init_adv_params.
 * @param[in]   scan_data_len
 * @param[in]   p_scan_data
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
 *
 */
T_GAP_CAUSE ble_ext_adv_mgr_set_scan_response_data(uint8_t adv_handle, uint16_t scan_data_len,
		uint8_t *p_scan_data);

/**
*@brief    used to update adv param, this api shall be used after stack init ready
*@note
*@param[in] adv_handle ble adv handle, each adv set has a adv handle
*@param[in] adv_event_prop @ref T_LE_EXT_ADV_LEGACY_ADV_PROPERTY
*@param[in] primary_adv_interval_min
*@param[in] primary_adv_interval_max
*@param[in] own_address_type @ref T_GAP_LOCAL_ADDR_TYPE
*@param[in] peer_address_type @ref T_GAP_REMOTE_ADDR_TYPE
*@param[in] p_peer_address  if not used, set default value NULL
*@param[in] filter_policy @ref T_GAP_ADV_FILTER_POLICY
*@return    T_GAP_CAUSE
*@retval GAP_CAUSE_SUCCESS  Operation success.
*@retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
*/
T_GAP_CAUSE ble_ext_adv_mgr_set_adv_param(uint8_t adv_handle,
		uint16_t adv_event_prop,
		uint32_t primary_adv_interval_min, uint32_t primary_adv_interval_max,
		uint8_t primary_adv_channel_map,
		T_GAP_LOCAL_ADDR_TYPE own_address_type,
		T_GAP_REMOTE_ADDR_TYPE peer_address_type, uint8_t *p_peer_address,
		T_GAP_ADV_FILTER_POLICY filter_policy);

T_GAP_CAUSE ble_ext_adv_mgr_set_multi_param(uint8_t adv_handle, uint8_t *random_address,
		uint16_t adv_interval,
		uint16_t adv_data_len, uint8_t *p_adv_data, uint16_t scan_data_len,
		uint8_t *p_scan_data);

/**
*@brief    used to update adv interval, this api shall be used after gap stack init ready
*@note
*@param[in] adv_handle ble adv handle, each adv set has a adv handle
*@param[in] adv_interval
*@return    T_GAP_CAUSE
* @retval GAP_CAUSE_SUCCESS  Operation success.
* @retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
*/
T_GAP_CAUSE ble_ext_adv_mgr_change_adv_interval(uint8_t adv_handle, uint16_t adv_interval);

/**
*@brief    used to handle extend adv state
*@note
*@param[in] adv_handle ble adv handle, each adv set has a adv handle
*@param[in] new_state @ref T_GAP_EXT_ADV_STATE
*@param[in] cause
*@return    none
* <b>Example usage</b>
 * \code{.c}
    void app_ble_handle_gap_msg(T_IO_MSG *p_bee_io_msg)
    {
    ......
       case GAP_MSG_LE_EXT_ADV_STATE_CHANGE:
        {
            ble_ext_adv_mgr_handle_adv_state(stack_msg.msg_data.gap_ext_adv_state_change.adv_handle,
                                         (T_GAP_EXT_ADV_STATE)stack_msg.msg_data.gap_ext_adv_state_change.new_state,
                                         stack_msg.msg_data.gap_ext_adv_state_change.cause);
        }
        break;
    ......
    }
 * \endcode
*/
void ble_ext_adv_mgr_handle_adv_state(uint8_t adv_handle, T_GAP_EXT_ADV_STATE new_state,
									  uint16_t cause);

/**
*@brief    used to handle extend adv start setting
*@note
*@param[in] cb_data @ref T_LE_CB_DATA
*@return    none
* <b>Example usage</b>
 * \code{.c}
    static T_APP_RESULT app_ble_gap_cb(uint8_t cb_type, void *p_cb_data)
    {
    ......
        case GAP_MSG_LE_EXT_ADV_START_SETTING:
            ble_ext_adv_mgr_handle_gap_cb(cb_type, &cb_data);
            break;
    ......
    }
 * \endcode
*/
void ble_ext_adv_mgr_handle_gap_cb(uint8_t cb_type, T_LE_CB_DATA *p_data);

/**
*@brief    used to get ble_ext_adv_mgr_adv_state @ref T_BLE_EXT_ADV_MGR_STATE
*@note
*@param[in] adv_handle  ble adv handle, each adv set has a adv handle
*@return    T_BLE_EXT_ADV_MGR_STATE @ref T_BLE_EXT_ADV_MGR_STATE
*/
T_BLE_EXT_ADV_MGR_STATE ble_ext_adv_mgr_get_adv_state(uint8_t adv_handle);

/**
*@brief    used to get current used maximum advertising interval
*@note
*@param[in] adv_handle  ble adv handle, each adv set has a adv handle
*@return  uint16_t
*@retval 0 Get the interval failed
*@retval 0x0020 - 0x4000 the current used maximum adv interval
*/
uint16_t ble_ext_adv_mgr_get_adv_interval(uint8_t adv_handle);

/**
*@brief    used to handle conn state for each adv set
*@note
*@param[in] conn_id
*@param[in] new_state @ref T_GAP_CONN_STATE
*@param[in] disc_cause
*@return    none
* <b>Example usage</b>
 * \code{.c}
    static void le_handle_new_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state,
                                         uint16_t disc_cause)
    {
        T_APP_LE_LINK *p_link;

        APP_PRINT_TRACE3("le_handle_new_conn_state_evt: conn_id %d, new_state %d, cause 0x%04x",
                         conn_id, new_state, disc_cause);
        ble_ext_adv_mgr_handle_conn_state(conn_id, new_state, disc_cause);
    }

    void app_ble_handle_gap_msg(T_IO_MSG *p_bee_io_msg)
    {
        APP_PRINT_TRACE1("app_ble_handle_gap_msg: subtype %d", p_bee_io_msg->subtype);
        T_LE_GAP_MSG stack_msg;
        T_APP_LE_LINK *p_link;

        memcpy(&stack_msg, &p_bee_io_msg->u.param, sizeof(p_bee_io_msg->u.param));

        switch (p_bee_io_msg->subtype)
        {
        case GAP_MSG_LE_DEV_STATE_CHANGE:
            {
                le_handle_dev_state_change_evt(stack_msg.msg_data.gap_dev_state_change.new_state,
                                               stack_msg.msg_data.gap_dev_state_change.cause);
            }
            break;

        case GAP_MSG_LE_CONN_STATE_CHANGE:
            {
                le_handle_new_conn_state_evt(stack_msg.msg_data.gap_conn_state_change.conn_id,
                                             (T_GAP_CONN_STATE)stack_msg.msg_data.gap_conn_state_change.new_state,
                                             stack_msg.msg_data.gap_conn_state_change.disc_cause);
            }
            break;
        }
    }
 * \endcode
*/
void ble_ext_adv_mgr_handle_conn_state(uint8_t conn_id, T_GAP_CONN_STATE new_state,
									   uint16_t disc_cause);

void ble_ext_adv_print_info(void);


/** @} */ /* End of group APP_DEVICE_Exported_Functions */
/** End of BLE_ADV
* @}
*/

#ifdef __cplusplus
}
#endif

#endif

#endif
