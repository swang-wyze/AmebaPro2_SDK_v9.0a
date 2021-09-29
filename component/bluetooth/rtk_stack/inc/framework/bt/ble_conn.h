/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#ifndef _BLE_CONN_H_
#define _BLE_CONN_H_

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#include "gap_le.h"
#include "gap_msg.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup BLE_CONN Ble Conn
  * @brief Ble Conn Param Update
  * @{
  */

/*============================================================================*
 *                              Functions
 *============================================================================*/
/** @defgroup BLE_CONN_Exported_Functions Ble Conn Functions
    * @{
    */

/**
* @brief  ble conn update init is used to create ble_update_table
* @param[in]  link_num the ble link number
* @return  bool
* @retval  true  success
* @retval  false  fail
*/

bool ble_conn_update_init(uint8_t link_num);

/**
*@brief  used to handle @ref T_GAP_CONN_PARAM_UPDATE
*@note
*@param[in]  T_GAP_CONN_PARAM_UPDATE  update_info
*@return  none
*@retval  none
*/
void ble_handle_conn_update_info(T_GAP_CONN_PARAM_UPDATE update_info);

/**
*@brief  used to handle @ref T_GAP_CONN_STATE
*@note
*@param[in]  T_GAP_CONN_STATE  new_state
*@param[in]  conn_id  ble conn id
*@return  none
*@retval  none
*/
void ble_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state);

/**
*@brief  used to set prefer conn param
*@note
*@param[in]  conn_id  ble conn id
*@param[in]  conn_interval_min the minimum of conn interval
*@param[in]  conn_interval_max the maximum of conn interval
*@param[in]  conn_latency the latency of ble conn
*@param[in]  supervision_timeout the supervision timeout of ble conn
*@return  bool
*@retval  true success
*@retval  false fail
*/
bool ble_set_prefer_conn_param(uint8_t conn_id, uint16_t conn_interval_min,
							   uint16_t conn_interval_max, uint16_t conn_latency,
							   uint16_t supervision_timeout);


/** @} */ /* End of group BLE_CONN_Exported_Functions */
/** End of BLE_CONN
* @}
*/

#ifdef __cplusplus
}
#endif

#endif

#endif /* _BLE_CONN_H_ */
