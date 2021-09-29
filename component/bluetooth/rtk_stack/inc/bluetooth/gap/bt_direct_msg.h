/**
*****************************************************************************************
*     Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    bt_direct_msg.h
  * @brief   This file contains all the constants and functions prototypes for BT direct message.
  * @details This file is used both bredr and le.
  * @author  jane
  * @date    2017-02-18
  * @version v1.0
  * *************************************************************************************
  */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef BT_DIRECT_MSG_H
#define BT_DIRECT_MSG_H

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include "upperstack_config.h"

/** @addtogroup BT_DIRECT_MSG BT Direct Message
  * @brief BT Direct Message module
  * @{
  */

/*============================================================================*
 *                         Macros
 *============================================================================*/
/** @defgroup BT_DIRECT_MSG_Exported_Macros BT Direct Message Exported Macros
  * @{
  */

/** @defgroup BT_DIRECT_MSG_TYPE BT Direct Message Type
 * @{
 */
#define BT_DIRECT_MSG_ISO_DATA_IND         0x01

/**
  * @}
  */

/** End of BT_DIRECT_MSG_Exported_Macros
  * @}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** @defgroup BT_DIRECT_MSG_Exported_Types BT Direct Message Exported Types
  * @{
  */
typedef enum {
	ISOCH_DATA_PKT_STATUS_VALID_DATA = 0,             /**< Valid data. The complete SDU was received correctly.*/
	ISOCH_DATA_PKT_STATUS_POSSIBLE_ERROR_DATA = 1,    /**< Possibly invalid data. The contents of the SDU may contain errors or part of the SDU may be missing. This is reported as "data with possible errors".*/
	ISOCH_DATA_PKT_STATUS_LOST_DATA = 2,              /**< Part(s) of the SDU were not received correctly. This is reported as "lost data".*/
} T_ISOCH_DATA_PKT_STATUS;

typedef struct {
	uint16_t conn_handle;
	T_ISOCH_DATA_PKT_STATUS pkt_status_flag;
	uint8_t   *p_buf;
	uint16_t  offset;
	uint16_t  iso_sdu_len;
	uint16_t  pkt_seq_num;
	bool      ts_flag;
	uint32_t  time_stamp;
} T_BT_DIRECT_ISO_DATA_IND;

typedef union {
	T_BT_DIRECT_ISO_DATA_IND        *p_bt_direct_iso;
} T_BT_DIRECT_CB_DATA;

/** End of BT_DIRECT_MSG_Exported_Types
  * @}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/**
 * @defgroup BT_DIRECT_MSG_EXPORT_Functions BT Direct Message Exported Functions
 *
 * @{
 */

/**
  * @brief Callback to notify app
  * @param[in] cb_type callback msy type @ref BT_DIRECT_MSG_TYPE.
  * @param[in] p_cb_data point to callback data @ref T_BT_DIRECT_CB_DATA.
  * @retval void
  */
typedef void(*P_FUN_BT_DIRECT_CB)(uint8_t cb_type, void *p_cb_data);

/**
 * @brief  Register callback to gap, when messages in @ref BT_DIRECT_MSG_TYPE happens, it will callback to app.
 * @param[in] app_callback Callback function provided by the APP to handle BT direct messages.
 *              @arg NULL -> Not send BT direct messages to APP.
 *              @arg Other -> Use application defined callback function.
 * @return void
 *
 * <b>Example usage</b>
 * \code{.c}
   void app_le_gap_init()
   {
       ...
       gap_register_direct_cb(app_gap_direct_callback);
   }
   \endcode
 */
void gap_register_direct_cb(P_FUN_BT_DIRECT_CB app_callback);

/** @} */ /* End of group BT_DIRECT_MSG_EXPORT_Functions */

/** @} */ /* End of group BT_DIRECT_MSG */

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

#endif /* BT_DIRECT_MSG_H */
