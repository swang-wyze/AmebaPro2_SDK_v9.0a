/**************************************************************************//**
 * @file    hal_sdhost.h
 * @brief   The HAL API implementation for SD Host controller
 * @version V1.00
 * @date    2017-07-12
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#ifndef _HAL_SDHOST_H_
#define _HAL_SDHOST_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_sdhost_ SDHOST
 * @{
 * brief The SD Host HAL RAM APIs. These APIs is provided for user application to control the SD Host hardware.
 */

extern const hal_sdhost_func_stubs_t hal_sdhost_stubs;

/**
 *  @brief To reg irq of the SD host controller.
 *
 *  @param[in]  irq_handler The SD host irq handler
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_irq_reg(irq_handler_t irq_handler)
{
	hal_sdhost_stubs.hal_sdhost_irq_reg(irq_handler);
}


/**
 *  @brief To unreg irq of the SD host controller.
 *
 *  @param[in]  void.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_irq_unreg(void)
{
	hal_sdhost_stubs.hal_sdhost_irq_unreg();
}


/**
 *  @brief To initialize the SD host controller.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  pin_sel The pinmux selection.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_init_host(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_init_host(psdhost_adapter);
}


/**
 *  @brief To initialize the SD memory card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_init_card(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_init_card(psdhost_adapter);
}

/**
 *  @brief To de-initialize the SD host controller.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_deinit(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_deinit(psdhost_adapter);
}

/**
 *  @brief To read data from the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  start_addr The start address to begin reading from the card.
 *  @param[in]  blk_cnt The block count.
 *  @param[in]  rbuf_32align The buffer to read data blocks (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_read_data(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, u8 *rbuf_32align)
{
	return hal_sdhost_stubs.hal_sdhost_read_data(psdhost_adapter, start_addr, blk_cnt, rbuf_32align);
}

/**
 *  @brief To write data to the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  start_addr The start address to begin writing to the card.
 *  @param[in]  blk_cnt The block count.
 *  @param[in]  wbuf_32align The buffer to write data blocks (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_write_data(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, const u8 *wbuf_32align)
{
	return hal_sdhost_stubs.hal_sdhost_write_data(psdhost_adapter, start_addr, blk_cnt, wbuf_32align);
}

/**
 *  @brief To erase data in the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  start_addr The start address to begin erasing.
 *  @param[in]  end_addr The end address to begin erasing.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_erase(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u64 end_addr)
{
	return hal_sdhost_stubs.hal_sdhost_erase(psdhost_adapter, start_addr, end_addr);
}


/**
 *  @brief To stop the SD bus transmission.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_stop_transmission(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_stop_transmission(psdhost_adapter);
}



/**
 *  @brief To get the current state of the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_get_card_status(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_get_card_status(psdhost_adapter);
}


/**
 *  @brief To get the SD status from the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  buf_32align The buffer to store the SD status (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_get_sd_status(hal_sdhost_adapter_t *psdhost_adapter, u8 *buf_32align)
{
	return hal_sdhost_stubs.hal_sdhost_get_sd_status(psdhost_adapter, buf_32align);
}


/**
 *  @brief To get the SCR register from the SD card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_get_scr(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_get_scr(psdhost_adapter);
}


/**
 *  @brief To switch the SD bus speed.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  speed The specified bus speed.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_switch_bus_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 speed)
{
	return hal_sdhost_stubs.hal_sdhost_switch_bus_speed(psdhost_adapter, speed);
}


/**
 *  @brief To get the current signaling level.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *
 *  @returns    The signaling level (1: 1.8V, 0: 3.3V).
 */
__STATIC_INLINE u8 hal_sdhost_get_curr_signal_level(hal_sdhost_adapter_t *psdhost_adapter)
{
	return hal_sdhost_stubs.hal_sdhost_get_curr_signal_level(psdhost_adapter);
}


/**
 *  @brief To get the speed mode supported by the card.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  value The supported speed mode.
 *
 *  @returns    The result.
 */
__STATIC_INLINE hal_status_t hal_sdhost_get_supported_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 *value)
{
	return hal_sdhost_stubs.hal_sdhost_get_supported_speed(psdhost_adapter, value);
}


/**
 *  @brief To hook a callback function for SD card insertion interrupt.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  pcallback The callback function.
 *  @param[in]  pdata The argument of the callback function.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_card_insert_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata)
{
	hal_sdhost_stubs.hal_sdhost_card_insert_hook(psdhost_adapter, pcallback, pdata);
}


/**
 *  @brief To hook a callback function for SD card removal interrupt.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  pcallback The callback function.
 *  @param[in]  pdata The argument of the callback function.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_card_remove_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata)
{
	hal_sdhost_stubs.hal_sdhost_card_remove_hook(psdhost_adapter, pcallback, pdata);
}


/**
 *  @brief To hook a callback function to make OS do a context-switch while waiting.
 *
 *  @param[in]  psdhost_adapter The SD host HAL adapter.
 *  @param[in]  task_yield The callback function.
 *
 *  @returns    void.
 */
__STATIC_INLINE void hal_sdhost_task_yield_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t task_yield, void *pdata)
{
	hal_sdhost_stubs.hal_sdhost_task_yield_hook(psdhost_adapter, task_yield, pdata);
}

__STATIC_INLINE void hal_sdhost_transfer_done_int_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata)
{
	hal_sdhost_stubs.hal_sdhost_transfer_done_int_hook(psdhost_adapter, pcallback, pdata);
}

/** @} */ /* End of group hs_hal_sdhost_ */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SDHOST_H_"

