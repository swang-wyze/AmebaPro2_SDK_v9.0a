/**************************************************************************//**
* @file        hal_sys_ctrl_nsc.c
* @brief       This file implements the entry functions of the Efuse Non-secure callable HAL functions.
*
* @version     V1.00
* @date        2021-03-26
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



#include "hal_sys_ctrl.h"
#include "hal_sys_ctrl_nsc.h"

//#if CONFIG_EFUSE_EN && CONFIG_EFUSE_NSC_EN

#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#undef SECTION_NS_ENTRY_FUNC
#undef NS_ENTRY
#define SECTION_NS_ENTRY_FUNC
#define NS_ENTRY
#endif
/**
 * @addtogroup hs_hal_efuse EFUSE
 * @{
 */

/**
 *  @brief The NSC function to enable syson .
 *
 *  @param[in]  enable  enable.
 *
 *  @return  TRUE
 *  @return  FALSE
 */
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_sys_peripheral_nsc(uint8_t id, uint8_t en)
{
	hal_sys_peripheral_en(id, en);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_sys_set_clk_nsc(uint8_t id, uint8_t sel_val)
{
	hal_sys_set_clk(id, sel_val);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_sys_get_clk_nsc(uint8_t id)
{
	return hal_sys_get_clk(id);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_sys_set_bus_idau_nsc(uint32_t idau_idx, uint32_t start_addr, uint32_t end_addr)
{
	hal_sys_set_bus_idau(idau_idx, start_addr, end_addr);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_sdm_32k_enable_nsc(u8 bypass_mode)
{
	hal_sdm_32k_enable(bypass_mode);
}

SECTION_NS_ENTRY_FUNC
u32 NS_ENTRY hal_read_sdm_32k_time_loss_nsc(void)
{
	hal_read_sdm_32k_time_loss();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_xtal_divider_enable_nsc(u8 enable)
{
	hal_xtal_divider_enable(enable);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_aon_wdt_enable_nsc(u8 enable, u32 timeout)
{
	hal_aon_wdt_enable(enable, timeout);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_osc4m_cal_nsc(void)
{
	hal_osc4m_cal();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_sys_bt_uart_mux_nsc(uint8_t sel)
{
	hal_sys_bt_uart_mux(sel);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_32k_s1_sel_nsc(u8 sel)
{
	hal_32k_s1_sel(sel);
}

/** @} */ /* End of group hs_hal_efuse */

//#endif

//#endif  /* end of "#if CONFIG_EFUSE_EN && CONFIG_EFUSE_NSC_EN" */

