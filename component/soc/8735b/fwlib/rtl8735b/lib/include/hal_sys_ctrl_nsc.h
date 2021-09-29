/**************************************************************************//**
* @file        hal_efuse_nsc.h
* @brief       The HAL Non-secure callable API implementation for the EFUSE
*
* @version     V1.00
* @date        2018-07-26
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



#ifndef _HAL_SYS_CTRL_NSC_H_
#define _HAL_SYS_CTRL_NSC_H_
#include "cmsis.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

#ifdef  __cplusplus
extern "C"
{
#endif

/**
* @addtogroup hal_sys_ctrl_nsc system control NSC
* @{
*/

#if defined(CONFIG_BUILD_SECURE)
void NS_ENTRY hal_sys_peripheral_nsc(uint8_t id, uint8_t en);
void NS_ENTRY hal_sys_set_clk_nsc(uint8_t id, uint8_t sel_val);
uint32_t NS_ENTRY hal_sys_get_clk_nsc(uint8_t id);
void NS_ENTRY hal_sys_set_bus_idau_nsc(uint32_t idau_idx, uint32_t start_addr, uint32_t end_addr);
void NS_ENTRY hal_sdm_32k_enable_nsc(u8 bypass_mode);
u32 NS_ENTRY hal_read_sdm_32k_time_loss_nsc(void);
void NS_ENTRY hal_xtal_divider_enable_nsc(u8 enable);
void NS_ENTRY hal_aon_wdt_enable_nsc(u8 enable, u32 timeout);
void NS_ENTRY hal_osc4m_cal_nsc(void);
void NS_ENTRY hal_sys_bt_uart_mux_nsc(uint8_t sel);
void NS_ENTRY hal_32k_s1_sel_nsc(u8 sel);


#endif

#if defined(CONFIG_BUILD_NONSECURE)
void hal_sys_peripheral_nsc(uint8_t id, uint8_t en);
void hal_sys_set_clk_nsc(uint8_t id, uint8_t sel_val);
uint32_t hal_sys_get_clk_nsc(uint8_t id);
uint32_t hal_sys_set_bus_idau_nsc(uint32_t idau_idx, uint32_t start_addr, uint32_t end_addr);
void hal_sdm_32k_enable(u8 bypass_mode);
u32 hal_read_sdm_32k_time_loss(void);
void hal_xtal_divider_enable(u8 enable);
void hal_aon_wdt_enable(u8 enable, u32 timeout);
void hal_osc4m_cal(void);
void hal_sys_bt_uart_mux(uint8_t sel);
void hal_32k_s1_sel(u8 sel);

#define hal_sys_peripheral_en                 hal_sys_peripheral_nsc
#define hal_sys_set_clk                       hal_sys_set_clk_nsc
#define hal_sys_get_clk                       hal_sys_get_clk_nsc
#define hal_sys_set_bus_idau                  hal_sys_set_bus_idau_nsc
#define hal_sdm_32k_enable                    hal_sdm_32k_enable_nsc
#define hal_read_sdm_32k_time_loss            hal_read_sdm_32k_time_loss_nsc
#define hal_xtal_divider_enable               hal_xtal_divider_enable_nsc
#define hal_aon_wdt_enable                    hal_aon_wdt_enable_nsc
#define hal_osc4m_cal                         hal_osc4m_cal_nsc
#define hal_sys_bt_uart_mux                   hal_sys_bt_uart_mux_nsc
#define hal_32k_s1_sel                        hal_32k_s1_sel_nsc

#endif  // end of "#if defined(CONFIG_BUILD_NONSECURE)"

/** @} */ /* End of group hal_sys_ctrl_nsc */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SYS_CTRL_NSC_H_"
