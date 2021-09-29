/**************************************************************************//**
 * @file     hal_trng.h
 * @brief    The HAL API implementation for the TRNG device.
 * @version  V1.00
 * @date     2016-07-15
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

#ifndef _HAL_TRNG_H_
#define _HAL_TRNG_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup ls_hal_trng TRNG
 * @ingroup rtl8735b
 * @{
 * @brief The TRNG HAL module of the LS platform.
 */

/**
  * @brief The stubs functions table to exports TRNG HAL functions in ROM.
  */

extern hal_trng_func_stubs_t hal_trng_stubs;

void hal_trng_enable_128K(hal_trng_adapter_t *ptrng_adp);
void hal_trng_enable_32K(hal_trng_adapter_t *ptrng_adp);
void hal_trng_disable(hal_trng_adapter_t *ptrng_adp);
void hal_trng_reg_irq(irq_handler_t handler, uint32_t *arg);
hal_status_t hal_trng_control_setting(hal_trng_adapter_t *ptrng_adp, hal_rng_reg_t *ptrng_reg);
hal_status_t hal_trng_init(hal_trng_adapter_t *ptrng_adp) ;
hal_status_t hal_trng_deinit(hal_trng_adapter_t *ptrng_adp);
hal_status_t hal_trng_load_default(hal_trng_adapter_t *ptrng_adp, hal_rng_st_reg_t *ptrng_st_reg,  hal_rng_reg_t *ptrng_reg);
void hal_trng_reset(hal_trng_adapter_t *ptrng_adp);
void hal_trng_self_test_setting(hal_trng_adapter_t *ptrng_adp,  hal_rng_st_reg_t *ptrng_st_reg);
hal_status_t hal_trng_interrupt_reg(hal_trng_adapter_t *ptrng_adp, uint32_t *data);
u32 hal_trng_read_readybit(hal_trng_adapter_t *ptrng_adp);
u32 hal_trng_read_parity_error_interrupt(hal_trng_adapter_t *ptrng_adp);
hal_status_t hal_trng_clear_interrupt(hal_trng_adapter_t *ptrng_adp);
u32 hal_trng_read_data(hal_trng_adapter_t *ptrng_adp);

/** @} */ /* End of group ls_hal_trng */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_TRNG_H_"
