/**************************************************************************//**
 * @file     rom_i2c.c
 * @brief    This file implements the I2C HAL functions.
 *
 * @version  V1.00
 * @date     2021-06-29
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


#include "cmsis.h"
#include "hal_irq.h"
#include "hal_i2c.h"
#ifdef CONFIG_I2C_EN

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
extern hal_i2c_func_stubs_t Image$$_STUB_I2C$$Base;     // symbol from linker script
#define __rom_stubs_hal_i2c Image$$_STUB_I2C$$Base
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern hal_i2c_func_stubs_t Image$$_STUB_I2C$$Base;     // symbol from linker script
#define __rom_stubs_hal_i2c Image$$_STUB_I2C$$Base
#elif defined ( __GNUC__ )
extern hal_i2c_func_stubs_t __rom_stubs_hal_i2c;     // symbol from linker script
#elif defined ( __ICCARM__ )
extern hal_i2c_func_stubs_t __rom_stubs_hal_i2c;     // symbol from linker script
#endif

/** \brief Description of hal_rtl_i2c_set_clk
 *
 *    hal_rtl_i2c_set_clk is used to set I2C clock.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return hal_status_t:      HAL status.
 */
hal_status_t hal_rtl_i2c_set_clk_patch(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t clock   = phal_i2c_adapter->init_dat.clock;
	uint8_t  idx     = phal_i2c_adapter->init_dat.index;
	uint32_t ic_hl_cnt;
	uint32_t ic_h_time;
	uint32_t ic_l_time;

	/* Get the IC-Clk setting first for the following process*/
	uint32_t sys_clk  = (100000000) / 1000000;

	if (idx == 2) {
		sys_clk  = (50000000) / 1000000;
	}

	DBG_I2C_INFO("hal_rtl_i2c_set_clk, spd mod: %x, sys_clk: %u\n\r", phal_i2c_adapter->init_dat.spd_mod, sys_clk);

	switch (phal_i2c_adapter->init_dat.spd_mod) {
	case I2CStandardSpeed:
		ic_h_time = ((1000000 / clock) * I2C_SS_MIN_SCL_HTIME) / (I2C_SS_MIN_SCL_HTIME + I2C_SS_MIN_SCL_LTIME);
		ic_l_time = ((1000000 / clock) * I2C_SS_MIN_SCL_LTIME) / (I2C_SS_MIN_SCL_HTIME + I2C_SS_MIN_SCL_LTIME);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_SS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>4) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 4;
		//}

		pi2c_reg->I2C_SS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("ICHtime:%x\n\r", pi2c_reg->I2C_SS_SCL_HCNT);
		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_SS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>3) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}
		pi2c_reg->I2C_SS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("ICLtime:%x\n\r", pi2c_reg->I2C_SS_SCL_LCNT);
		break;

	case I2CFastSpeed:
		ic_h_time = ((1000000 / clock) * I2C_FS_MIN_SCL_HTIME) / (I2C_FS_MIN_SCL_HTIME + I2C_FS_MIN_SCL_LTIME);
		ic_l_time = ((1000000 / clock) * I2C_FS_MIN_SCL_LTIME) / (I2C_FS_MIN_SCL_HTIME + I2C_FS_MIN_SCL_LTIME);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_FS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			DBG_I2C_ERR("h_cnt: %x\n\r", ic_hl_cnt);
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>4) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 4;
		//}

		pi2c_reg->I2C_FS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("fs_scl_hcnt%d: %x\n\r", idx, pi2c_reg->I2C_FS_SCL_HCNT);
		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_FS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			DBG_I2C_ERR("h_cnt: %x\n\r", ic_hl_cnt);
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>3) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}

		pi2c_reg->I2C_FS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("fs_scl_lcnt%d: %x\n\r", idx, pi2c_reg->I2C_FS_SCL_LCNT);
		break;

	case I2CHighSpeed:
		ic_hl_cnt = 400;
		pi2c_reg->I2C_SS_SCL_HCNT = ic_hl_cnt;

		ic_hl_cnt = 470;
		pi2c_reg->I2C_SS_SCL_LCNT = ic_hl_cnt;

		ic_hl_cnt = 60;
		pi2c_reg->I2C_FS_SCL_HCNT = ic_hl_cnt;

		ic_hl_cnt = 130;
		pi2c_reg->I2C_FS_SCL_LCNT = ic_hl_cnt;

		ic_h_time = ((1000000 / clock) * I2C_HS_MIN_SCL_HTIME_100) / (I2C_HS_MIN_SCL_HTIME_100 + I2C_HS_MIN_SCL_LTIME_100);
		ic_l_time = ((1000000 / clock) * I2C_HS_MIN_SCL_LTIME_100) / (I2C_HS_MIN_SCL_HTIME_100 + I2C_HS_MIN_SCL_LTIME_100);
		DBG_I2C_INFO("ICHtime:%x\n\r", ic_h_time);
		DBG_I2C_INFO("ICLtime:%x\n\r", ic_l_time);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_HS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>8) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}

		pi2c_reg->I2C_HS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("hs_scl_hcnt%d: %x\n\r", idx, pi2c_reg->I2C_HS_SCL_HCNT);

		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_HS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>6) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 6;
		//}

		pi2c_reg->I2C_HS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("hs_scl_lcnt%d: %x\n\r", idx, pi2c_reg->I2C_HS_SCL_LCNT);
		break;

	default:
		break;
	}

	return HAL_OK;
}

/** \brief Description of I2C0_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
static void I2C0_IRQHandler_Patch(void)
{
	hal_irq_clear_pending(I2C0_IRQn);
	if (__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[0] != NULL) {
		__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[0]((void *)__rom_stubs_hal_i2c.hal_i2c_group_adpt->adapter[0]);
	}

	DBG_I2C_INFO("I2C0_IRQHandler\n\r");
}
/** \brief Description of I2C1_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
static void I2C1_IRQHandler_Patch(void)
{
	hal_irq_clear_pending(I2C1_IRQn);
	if (__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[1] != NULL) {
		__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[1]((void *)__rom_stubs_hal_i2c.hal_i2c_group_adpt->adapter[1]);
	}

	DBG_I2C_INFO("I2C1_IRQHandler\n\r");
}
/** \brief Description of I2C2_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
static void I2C2_IRQHandler_Patch(void)
{
	hal_irq_clear_pending(I2C2_IRQn);
	if (__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[2] != NULL) {
		__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[2]((void *)__rom_stubs_hal_i2c.hal_i2c_group_adpt->adapter[2]);
	}

	DBG_I2C_INFO("I2C2_IRQHandler\n\r");
}
/** \brief Description of I2C3_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
static void I2C3_IRQHandler_Patch(void)
{
	hal_irq_clear_pending(I2C3_IRQn);
	if (__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[3] != NULL) {
		__rom_stubs_hal_i2c.hal_i2c_group_adpt->irq_fun[3]((void *)__rom_stubs_hal_i2c.hal_i2c_group_adpt->adapter[3]);
	}

	DBG_I2C_INFO("I2C3_IRQHandler\n\r");
}

hal_status_t hal_rtl_i2c_init_patch(hal_i2c_adapter_t *phal_i2c_adapter)
{
	if (/*hal_i2c_comm_irq_reg == 0*/1) {

		if (phal_i2c_adapter->init_dat.index == 0) {
			hal_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C0_IRQHandler_Patch);
		} else if (phal_i2c_adapter->init_dat.index == 1) {
			hal_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C1_IRQHandler_Patch);
		} else if (phal_i2c_adapter->init_dat.index == 2) {
			hal_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C2_IRQHandler_Patch);
		} else if (phal_i2c_adapter->init_dat.index == 3) {
			hal_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C3_IRQHandler_Patch);
		}

		hal_irq_set_priority(I2C0_IRQn + phal_i2c_adapter->init_dat.index, I2C0_IRQPri + phal_i2c_adapter->init_dat.index);
		//hal_i2c_comm_irq_reg = 1;
	}
	return HAL_OK;
}

#endif

/** @} */ /* End of group hs_hal_i2c_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_i2c */

