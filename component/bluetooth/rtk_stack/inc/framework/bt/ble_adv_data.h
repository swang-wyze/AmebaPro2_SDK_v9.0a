/**
 * @file   ble_adv_data.h
 * @brief  struct and interface about ble adv data manager
 * @author leon
 * @date   2020.9.3
 * @version 1.0
 * @par Copyright (c):
         Realsil Semiconductor Corporation. All rights reserved.
 */

#ifndef _BLE_ADV_DATA__
#define _BLE_ADV_DATA__

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BLE_ADV_DATA Ble Adv Data
  * @brief Ble extended advertising data manager module
  * @{
  */


#include "gap_msg.h"

/**
 *  @brief interface of enable adv for app user
 *  @param[in]  pp_handle   when add adv data success, manager return a handle for deleting
 *  @param[in]  adv_data_len  adv data length
 *  @param[in]  p_adv_data    pointer to adv data
 *  @param[in]  scan_resp_len scan rsp data length
 *  @param[in]  p_scan_resp   pointer to scan rsp data
 *  @return operation result
 *  @retval true  success
 *  @retval false failed
 */
bool ble_adv_data_add(void **pp_handle, uint16_t adv_data_len, uint8_t *p_adv_data,
					  uint16_t scan_resp_len, uint8_t *p_scan_resp);

/**
 *  @brief interface of disable adv for app user
 *  @param[in]  pp_handle  adv set handle which get from add operation
 *  @return operation result
 *  @retval true  success
 *  @retval false failed
 */
bool ble_adv_data_del(void *p_handle);

/**
 *  @brief interface of enable adv data manager
 *  @return operation result
 *  @retval true  success
 *  @retval false failed
 */
bool ble_adv_data_enable(void);

/**
 *  @brief interface of disenable adv data manager
 *  @return operation result
 *  @retval true  success
 *  @retval false failed
 */
bool ble_adv_data_disable(void);

/**
 *  @brief interface of init adv data manager
 *  @param[in]  adv_handle
 *  @param[in]  update_scan_data  support on not update scan rsp data while update adv data
 *  @return operation result
 *  @retval true  success
 *  @retval false failed
 */
bool ble_adv_data_init(uint8_t adv_handle, bool update_scan_data);

/**
 *  @brief interface of handle connect msg
 *  @param[in]  conn_id    connection id whick refer to link
 *  @param[in]  new_state  new link connection state
 *  @param[in]  disc_cause  disconnect cause if new_state is disconnected
 */
void ble_adv_data_handle_conn_state(uint8_t conn_id, T_GAP_CONN_STATE new_state,
									uint16_t disc_cause);

/** End of BLE_ADV_DATA
* @}
*/

#ifdef __cplusplus
}
#endif

#endif

#endif
