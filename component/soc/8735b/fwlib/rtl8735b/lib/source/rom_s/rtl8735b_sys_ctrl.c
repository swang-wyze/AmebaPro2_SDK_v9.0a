/**************************************************************************//**
* @file        rtl8735b_sys_ctrl.c
* @brief       This file implements the SYSON control HAL functions.
*
* @version     V1.00
* @date        2021-08-07
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
#include "rtl8735b_sys_ctrl.h"
#include "rtl8735b_syson_s_type.h"
#include "rtl8735b_pon_type.h"
#include "rtl8735b.h"


#define SECTION_SYS_CTRL_TEXT           SECTION(".rom.hal_sys_ctrl.text")
#define SECTION_SYS_CTRL_DATA           SECTION(".rom.hal_sys_ctrl.data")
#define SECTION_SYS_CTRL_RODATA         SECTION(".rom.hal_sys_ctrl.rodata")
#define SECTION_SYS_CTRL_BSS            SECTION(".rom.hal_sys_ctrl.bss")
#define SECTION_SYS_CTRL_STUBS          SECTION(".rom.hal_sys_ctrl.stubs")


/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hal_sys_ctrl_rom_func SYS CTRL HAL ROM APIs.
 * @ingroup hs_hal_i2c
 * @{
 * @brief The SYS CTRL HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of SYSON CTRL HAL APIs in the RAM space is provided for the user application.
 */

/**
  * \brief The stubs functions table to exports SYS CTRL HAL functions in ROM.
*/
SECTION_SYS_CTRL_STUBS const hal_sys_ctrl_func_stubs_t hal_sys_ctrl_stubs = {
	.hal_sys_peripheral_en          = hal_rtl_sys_peripheral_en,
	.hal_sys_set_clk                = hal_rtl_sys_set_clk,
	.hal_sys_get_clk                = hal_rtl_sys_get_clk,
	.hal_sys_boot_info_assign_val   = hal_rtl_sys_boot_info_assign_val,
	.hal_sys_boot_info_get_val      = hal_rtl_sys_boot_info_get_val,
	.hal_sys_boot_footpath_init     = hal_rtl_sys_boot_footpath_init,
	.hal_sys_boot_footpath_store    = hal_rtl_sys_boot_footpath_store,
	.hal_vdr_s_jtag_key_write       = hal_rtl_vdr_s_jtag_key_write,
	.hal_vdr_ns_jtag_key_write      = hal_rtl_vdr_ns_jtag_key_write,
	.hal_sys_get_video_info         = hal_rtl_sys_get_video_info,
	.hal_sys_set_video_info         = hal_rtl_sys_set_video_info,
	.hal_sys_get_chip_id            = hal_rtl_sys_get_chip_id,
	.hal_sys_boot_footpath_clear    = hal_rtl_sys_boot_footpath_clear
};


/**
  * @brief The table of BOOT INFO SW RESV registers.
  */
SECTION_SYS_CTRL_RODATA
const uint32_t *boot_info_sw_reg_list[BOOT_INFO_SW_RESV_MAX_SIZE] = {
	(uint32_t *) &(AON->AON_REG_AON_SW_BOOT0),
	(uint32_t *) &(AON->AON_REG_AON_SW_BOOT1),
	(uint32_t *) &(AON->AON_REG_AON_SW_BOOT2),
	(uint32_t *) &(AON->AON_REG_AON_SW_BOOT3),
	(uint32_t *) &(AON->AON_REG_AON_SW_BOOT4)
};

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
SECTION_SYS_CTRL_TEXT
uint8_t hal_rtl_sys_get_rma_state(void)
{
	PON_TypeDef *pon = PON;
	volatile uint32_t reg_val;
	uint8_t rma_state = RMA_NORMAL_STATE;
	reg_val = PON->PON_REG_RMA_CTRL0;
	rma_state = ((reg_val & PON_MASK_RMA_STATE) >> PON_SHIFT_RMA_STATE);
	return rma_state;
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_en_rma_mgn_pg(void)
{
	PON_TypeDef *pon = PON;
	volatile uint32_t reg_val;
	const uint8_t en_ctrl = 0x65;

	uint8_t rma_state = RMA_NORMAL_STATE;
	reg_val = PON->PON_REG_RMA_CTRL0;
	reg_val &= (~PON_MASK_PWD_EN);
	reg_val |= (en_ctrl << PON_SHIFT_PWD_EN);
	PON->PON_REG_RMA_CTRL0 = reg_val;
}
#endif

SECTION_SYS_CTRL_TEXT
uint32_t hal_rtl_sys_boot_info_get_val(uint8_t info_idx)
{
	// AON 0x140,0x148,0x14C,0x150,0x154
	volatile uint32_t reg_val;
	if (info_idx >= BOOT_INFO_SW_RESV_MAX_SIZE) {
		return 0x0;
	}
	reg_val = *((uint32_t *)boot_info_sw_reg_list[info_idx]);
	return reg_val;
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_boot_info_assign_val(uint8_t info_idx, uint32_t info_v)
{
	// AON 0x140,0x148,0x14C,0x150,0x154
	volatile uint32_t reg_val;
	if (info_idx >= BOOT_INFO_SW_RESV_MAX_SIZE) {
		return;
	}
	reg_val = info_v;
	*((uint32_t *)boot_info_sw_reg_list[info_idx]) = reg_val;
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_boot_footpath_init(uint8_t info_idx)
{
	// AON 0x140,0x148,0x14C,0x150,0x154
	if (info_idx >= BOOT_INFO_SW_RESV_MAX_SIZE) {
		return;
	}
	hal_rtl_sys_boot_info_assign_val(info_idx, 0x0);   // clear [31:0]
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_boot_footpath_store(uint8_t info_idx, uint8_t fp_v)
{
	// AON 0x140,0x148,0x14C,0x150,0x154
	volatile uint32_t reg_val;
	if (info_idx >= BOOT_INFO_SW_RESV_MAX_SIZE) {
		return;
	}
	if ((fp_v >= 0) && (fp_v < 32)) {
		reg_val = *((uint32_t *)boot_info_sw_reg_list[info_idx]);
		reg_val |= (0x1 << fp_v);
		*((uint32_t *)boot_info_sw_reg_list[info_idx]) = reg_val;
	}
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_boot_footpath_clear(uint8_t info_idx, uint8_t fp_v)
{
	// AON 0x140,0x148,0x14C,0x150,0x154
	volatile uint32_t reg_val;
	if (info_idx >= BOOT_INFO_SW_RESV_MAX_SIZE) {
		return;
	}
	if ((fp_v >= 0) && (fp_v < 32)) {
		reg_val = *((uint32_t *)boot_info_sw_reg_list[info_idx]);
		reg_val &= (~(0x1 << fp_v));
		*((uint32_t *)boot_info_sw_reg_list[info_idx]) = reg_val;
	}
}

/**

        \addtogroup hal_rtl_sys
        @{
*/
SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_peripheral_en(uint8_t id, uint8_t en)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	PON_TypeDef *pon = PON;
	AON_TypeDef *aon = AON;
	volatile uint32_t val;

	switch (id) {
	case EDDSA_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= SYSON_S_BIT_HS_ED255_CLK_EN | SYSON_S_BIT_HS_ED255_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &= ~(SYSON_S_BIT_HS_ED255_CLK_EN | SYSON_S_BIT_HS_ED255_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;
	case GPIO_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_GPIO_CTRL;
			val |= SYSON_S_BIT_SYS_GPIO_EN | SYSON_S_BIT_SYS_GPIO_PCLK_EN | SYSON_S_BIT_SYS_GPIO_SCLK_EN;
			syson_s->SYSON_S_REG_GPIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_GPIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_GPIO_EN | SYSON_S_BIT_SYS_GPIO_PCLK_EN | SYSON_S_BIT_SYS_GPIO_SCLK_EN);
			syson_s->SYSON_S_REG_GPIO_CTRL = val;
		}
		break;
	case GPIO_PON:
		if (en == ENABLE) {
			// 1. Enable PON GPIO PCLK/SCLK
			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val |= (PON_BIT_PCLK_GPIO_EN | PON_BIT_SCLK_GPIO_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;

			// 2. Enable PON GPIO
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val |= PON_BIT_GPIO_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;
		} else { // en == DISABLE // Note: PON will NOT be disabled
			// 1. Disable PON GPIO
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val &= ~(PON_BIT_GPIO_EN);
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			// 2. Disable PON GPIO PCLK/SCLK
			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val &= ~(PON_BIT_PCLK_GPIO_EN | PON_BIT_SCLK_GPIO_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		}
		break;
	case GPIO_AON:
		if (en == ENABLE) {
			// 1. Enable AON GPIO SCLK/PCLK
			val = aon->AON_REG_AON_CLK_CTRL;
			val |= AON_BIT_GPIO_PCLK_EN | AON_BIT_GPIO_SCLK_EN;
			aon->AON_REG_AON_CLK_CTRL = val;

			// 2. Enable AON GPIO
			val = aon->AON_REG_AON_FUNC_CTRL;
			val |= AON_BIT_GPIO_FEN;
			aon->AON_REG_AON_FUNC_CTRL = val;
		} else { // en == DISABLE
			// 1. Disable AON GPIO
			val = aon->AON_REG_AON_FUNC_CTRL;
			val &= ~(AON_BIT_GPIO_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// 2. Disable AON GPIO SCLK/PCLK
			val = aon->AON_REG_AON_CLK_CTRL;
			val &= ~(AON_BIT_GPIO_PCLK_EN | AON_BIT_GPIO_SCLK_EN);
			aon->AON_REG_AON_CLK_CTRL = val;
		}
		break;
	case DDR_SYS:
		if (en == ENABLE) {
			syson_s->SYSON_S_REG_SYS_LPDDR1_CTRL = SYSON_S_BIT_HS_LPDDR1_EN | SYSON_S_BIT_HS_LPDDR1_CLK_EN;
			syson_s->SYSON_S_REG_SYS_DDRPHY_CTRL = SYSON_S_BIT_HS_DDRPHY_CRT_RST | SYSON_S_BIT_PWDPAD15N | SYSON_S_BIT_DDRPHY_RBUS_EN | SYSON_S_BIT_DDRPHY_VCCON |
												   SYSON_S_BIT_HS_DDRPHY_RBUS_CLK_EN | SYSON_S_BIT_HS_DDRPHY_CRT_CLK_EN;
		} else { //en == DISABLE
		}
		break;
	case ENC_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val |= SYSON_S_BIT_SYS_ENCODER_EN | SYSON_S_BIT_SYS_ENCODER_CLK_EN;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
		} else { //en == DISABLE
		}
		break;
	case GDMA0_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |=  SYSON_S_BIT_HS_GDMA0_CLK_ALWS_EN |  SYSON_S_BIT_HS_GDMA0_CLK_EN |  SYSON_S_BIT_HS_GDMA0_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &=  ~(SYSON_S_BIT_HS_GDMA0_CLK_ALWS_EN |  SYSON_S_BIT_HS_GDMA0_CLK_EN |  SYSON_S_BIT_HS_GDMA0_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;
	case GDMA1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |=  SYSON_S_BIT_HS_GDMA1_CLK_ALWS_EN |  SYSON_S_BIT_HS_GDMA1_CLK_EN | SYSON_S_BIT_HS_GDMA1_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &=  ~(SYSON_S_BIT_HS_GDMA1_CLK_ALWS_EN |  SYSON_S_BIT_HS_GDMA1_CLK_EN | SYSON_S_BIT_HS_GDMA1_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;
	case FLASH_SYS: // NOR¡@flash
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
			val |= SYSON_S_BIT_NOR_FLASH_EN | SYSON_S_BIT_NOR_FLASH_CLK_EN;
			syson_s->SYSON_S_REG_SYS_FLASH_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
			val &= ~(SYSON_S_BIT_NOR_FLASH_EN | SYSON_S_BIT_NOR_FLASH_CLK_EN);
			syson_s->SYSON_S_REG_SYS_FLASH_CTRL = val;
		}
		break;
	case SPI0_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val |= SYSON_S_BIT_SYS_SPI0_EN | SYSON_S_BIT_SYS_SPI0_CLK_EN;
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SPI0_EN | SYSON_S_BIT_SYS_SPI0_CLK_EN);
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		}
		break;
	case SPI1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val |= SYSON_S_BIT_SYS_SPI1_EN | SYSON_S_BIT_SYS_SPI1_CLK_EN;
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SPI1_EN | SYSON_S_BIT_SYS_SPI1_CLK_EN);
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		}
		break;
	case HS_SPI0_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val |= SYSON_S_BIT_SYS_HS_SPI0_EN | SYSON_S_BIT_SYS_HS_SPI0_CLK_EN;
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val &= ~(SYSON_S_BIT_SYS_HS_SPI0_EN | SYSON_S_BIT_SYS_HS_SPI0_CLK_EN);
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		}
		break;
	case HS_SPI1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val |= SYSON_S_BIT_SYS_HS_SPI1_EN | SYSON_S_BIT_SYS_HS_SPI1_CLK_EN;
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_SPI_CTRL;
			val &= ~(SYSON_S_BIT_SYS_HS_SPI1_EN | SYSON_S_BIT_SYS_HS_SPI1_CLK_EN);
			syson_s->SYSON_S_REG_SYS_SPI_CTRL = val;
		}
		break;
	case UART0_SYS:
		if (en == ENABLE) {
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val |= PON_BIT_UART0_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val |= PON_BIT_UART0_BD_EN | PON_BIT_PCLK_UART0_EN | PON_BIT_SCLK_UART0_EN;
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		} else { //en == DISABLE
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val &= (~PON_BIT_UART0_EN);
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val &= ~(PON_BIT_UART0_BD_EN | PON_BIT_PCLK_UART0_EN | PON_BIT_SCLK_UART0_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		}
		break;
	case UART1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val |= SYSON_S_BIT_SYS_UART1_EN | SYSON_S_BIT_SYS_UART1_BD_EN | SYSON_S_BIT_SYS_UART1_PCLK_EN | SYSON_S_BIT_SYS_UART1_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val &= ~(SYSON_S_BIT_SYS_UART1_EN | SYSON_S_BIT_SYS_UART1_BD_EN | SYSON_S_BIT_SYS_UART1_PCLK_EN | SYSON_S_BIT_SYS_UART1_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		}
		break;
	case UART2_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val |= SYSON_S_BIT_SYS_UART2_EN | SYSON_S_BIT_SYS_UART2_BD_EN | SYSON_S_BIT_SYS_UART2_PCLK_EN | SYSON_S_BIT_SYS_UART2_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val &= ~(SYSON_S_BIT_SYS_UART2_EN | SYSON_S_BIT_SYS_UART2_BD_EN | SYSON_S_BIT_SYS_UART2_PCLK_EN | SYSON_S_BIT_SYS_UART2_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		}
		break;
	case UART3_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val |= SYSON_S_BIT_SYS_UART3_EN | SYSON_S_BIT_SYS_UART3_BD_EN | SYSON_S_BIT_SYS_UART3_PCLK_EN | SYSON_S_BIT_SYS_UART3_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val &= ~(SYSON_S_BIT_SYS_UART3_EN | SYSON_S_BIT_SYS_UART3_BD_EN | SYSON_S_BIT_SYS_UART3_PCLK_EN | SYSON_S_BIT_SYS_UART3_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		}
		break;
	case UART4_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val |= SYSON_S_BIT_SYS_BT_UART_EN | SYSON_S_BIT_SYS_BT_UART_BD_EN | SYSON_S_BIT_SYS_BT_UART_PCLK_EN | SYSON_S_BIT_SYS_BT_UART_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
			val &= ~(SYSON_S_BIT_SYS_BT_UART_EN | SYSON_S_BIT_SYS_BT_UART_BD_EN | SYSON_S_BIT_SYS_BT_UART_PCLK_EN | SYSON_S_BIT_SYS_BT_UART_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_UART_CTRL = val;
		}
		break;
	case TIMER0_SYS:
		if (en == ENABLE) {
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val |= PON_BIT_TIMER0_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val |= PON_BIT_SCLK_TIMER0_EN | PON_BIT_PCLK_TIMER0_EN;
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;

		} else { //en == DISABLE
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val &= ~PON_BIT_TIMER0_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val &= ~(PON_BIT_SCLK_TIMER0_EN | PON_BIT_PCLK_TIMER0_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		}
		break;
	case TIMER1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val |= SYSON_S_BIT_SYS_TG1_EN | SYSON_S_BIT_SYS_TG1_PCLK_EN | SYSON_S_BIT_SYS_TG1_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val &= ~(SYSON_S_BIT_SYS_TG1_EN | SYSON_S_BIT_SYS_TG1_PCLK_EN | SYSON_S_BIT_SYS_TG1_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		}
		break;
	case TIMER2_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val |= SYSON_S_BIT_SYS_TG2_EN | SYSON_S_BIT_SYS_TG2_PCLK_EN | SYSON_S_BIT_SYS_TG2_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val &= ~(SYSON_S_BIT_SYS_TG2_EN | SYSON_S_BIT_SYS_TG2_PCLK_EN | SYSON_S_BIT_SYS_TG2_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		}
		break;
	case TIMER3_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val |= SYSON_S_BIT_SYS_TG3_EN | SYSON_S_BIT_SYS_TG3_PCLK_EN | SYSON_S_BIT_SYS_TG3_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
			val &= ~(SYSON_S_BIT_SYS_TG3_EN | SYSON_S_BIT_SYS_TG3_PCLK_EN | SYSON_S_BIT_SYS_TG3_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		}
		break;
	case PWM_SYS:
		if (en == ENABLE) {
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val |= PON_BIT_PWM_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val |= (PON_BIT_SCLK_PWM_EN | PON_BIT_PCLK_PWM_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;

		} else { //en == DISABLE
			val = pon->PON_REG_PON_FUNC_EN_CTRL;
			val &= ~PON_BIT_PWM_EN;
			pon->PON_REG_PON_FUNC_EN_CTRL = val;

			val = pon->PON_REG_PON_FUNC_CLK_CTRL;
			val &= ~(PON_BIT_SCLK_PWM_EN | PON_BIT_PCLK_PWM_EN);
			pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		}
		break;
	case CRYPTO_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= SYSON_S_BIT_HS_IPSEC_EN | SYSON_S_BIT_HS_IPSEC_CLK_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &= ~(SYSON_S_BIT_HS_IPSEC_EN | SYSON_S_BIT_HS_IPSEC_CLK_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;
	case RSA_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_RSA_FUNC_CTRL;
			val |= (SYSON_S_BIT_RSA_EN | SYSON_S_BIT_RSA_CLK_EN);
			syson_s->SYSON_S_REG_SYS_RSA_FUNC_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_RSA_FUNC_CTRL;
			val &= ~(SYSON_S_BIT_RSA_EN | SYSON_S_BIT_RSA_CLK_EN);
			syson_s->SYSON_S_REG_SYS_RSA_FUNC_CTRL = val;
		}
		break;
	case I2C0_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val |= SYSON_S_BIT_SYS_I2C0_EN | SYSON_S_BIT_SYS_I2C0_PCLK_EN | SYSON_S_BIT_SYS_I2C0_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2C0_EN | SYSON_S_BIT_SYS_I2C0_PCLK_EN | SYSON_S_BIT_SYS_I2C0_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		}
		break;
	case I2C1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val |= SYSON_S_BIT_SYS_I2C1_EN | SYSON_S_BIT_SYS_I2C1_PCLK_EN | SYSON_S_BIT_SYS_I2C1_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2C1_EN | SYSON_S_BIT_SYS_I2C1_PCLK_EN | SYSON_S_BIT_SYS_I2C1_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		}
		break;

	case I2C2_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val |= SYSON_S_BIT_SYS_I2C2_EN | SYSON_S_BIT_SYS_I2C2_PCLK_EN | SYSON_S_BIT_SYS_I2C2_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2C2_EN | SYSON_S_BIT_SYS_I2C2_PCLK_EN | SYSON_S_BIT_SYS_I2C2_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		}
		break;

	case I2C3_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val |= SYSON_S_BIT_SYS_I2C3_EN | SYSON_S_BIT_SYS_I2C3_PCLK_EN | SYSON_S_BIT_SYS_I2C3_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2C_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2C3_EN | SYSON_S_BIT_SYS_I2C3_PCLK_EN | SYSON_S_BIT_SYS_I2C3_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_I2C_CTRL = val;

		}
		break;

	case ECDSA_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= SYSON_S_BIT_HS_ECC_EN | SYSON_S_BIT_HS_ECC_CLK_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &= ~(SYSON_S_BIT_HS_ECC_EN | SYSON_S_BIT_HS_ECC_CLK_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;

	case ADC_SYS:
		if (en == ENABLE) {
			// 0. Turn on SWR (Switching Regulator) 1.35V
			val = aon->AON_REG_SWR_SRC_CTRL0;
			val |= AON_BIT_SRC_POW_SW;
			aon->AON_REG_SWR_SRC_CTRL0 = val;

			// 1. Turn on pow BG.
			val = aon->AON_REG_AON_PLL_CTRL0;
			val |= AON_BIT_PLL_POW_BG;
			aon->AON_REG_AON_PLL_CTRL0 = val;
			hal_delay_us(100);

			// 2. Turn on pow BG current, together with power on MBIAS
			val = aon->AON_REG_AON_PLL_CTRL0;
			val |= (AON_BIT_PLL_POW_I | AON_BIT_PLL_POW_MBIAS);
			aon->AON_REG_AON_PLL_CTRL0 = val;
			hal_delay_us(10);

			// 3. Turn on LDO09 (1.3V to 0.9V LDO) and LDO18 (3.3V to 1.8V LDO) together
			val = aon->AON_REG_AON_PLL_CTRL0;
			val |= (AON_BIT_POW_LDO09 | AON_BIT_POW_LDO18) ;
			aon->AON_REG_AON_PLL_CTRL0 = val;

			hal_delay_us(10);

			// 4. Enable pow adc refgen
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL0;
			val |= SYSON_S_BIT_POW_REF;
			syson_s->SYSON_S_REG_SYS_ADC_CTRL0 = val;

			hal_delay_us(20);

			// 5. Enable ADC block, PCLK and SCLK
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL1;
			val |= (SYSON_S_BIT_SYS_ADC_EN | SYSON_S_BIT_SYS_ADC_PCLK_EN | SYSON_S_BIT_SYS_ADC_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_ADC_CTRL1 = val;

			// 6. Disable ADC analog to digital domain isolation
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL1;
			val &= ~(SYSON_S_BIT_SYS_ISO_ADC);
			syson_s->SYSON_S_REG_SYS_ADC_CTRL1 = val;

		} else { //en == DISABLE

			// As BG and MBIAS may be used by many IPs, ADC will not turn these signals off

			// 1. Re-enable ADC analog to digital domain isolation
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL1;
			val |= SYSON_S_BIT_SYS_ISO_ADC;
			syson_s->SYSON_S_REG_SYS_ADC_CTRL1 = val;

			// 2. Disable ADC block, PCLK and SCLK
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL1;
			val &= ~(SYSON_S_BIT_SYS_ADC_EN | SYSON_S_BIT_SYS_ADC_PCLK_EN | SYSON_S_BIT_SYS_ADC_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_ADC_CTRL1 = val;

			hal_delay_us(20);

			// 3. Disable pow adc refgen
			val = syson_s->SYSON_S_REG_SYS_ADC_CTRL0;
			val &= ~(SYSON_S_BIT_POW_REF);
			syson_s->SYSON_S_REG_SYS_ADC_CTRL0 = val;
		}
		break;

	case RTC_SYS:
		if (en == ENABLE) {
			// 1. Enable RTC clock
			val = aon->AON_REG_AON_CLK_CTRL;
			val |= (AON_BIT_RTC_CK_FEN);
			aon->AON_REG_AON_CLK_CTRL = val;

			// 2. Enable RTC
			val = aon->AON_REG_AON_FUNC_CTRL;
			val |= (AON_BIT_RTC_INTF_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// 3. Enable RTC WAKE_EVENT_MS
			//val = aon->AON_REG_AON_SLP_WAKE_EVENT_MSK0;
			//val |= (AON_BIT_AON_WEVT_AON_RTC_MSK);
			//aon->AON_REG_AON_FUNC_CTRL = val;
		} else { // en == DISABLE
			// 1. Disable RTC
			val = aon->AON_REG_AON_FUNC_CTRL;
			val &= ~(AON_BIT_RTC_INTF_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// 2. Disable RTC clock
			val = aon->AON_REG_AON_CLK_CTRL;
			val &= (AON_BIT_RTC_CK_FEN);
			aon->AON_REG_AON_CLK_CTRL = val;

			// 3. Disable RTC WAKE_EVENT_MS
			//val = aon->AON_REG_AON_SLP_WAKE_EVENT_MSK0;
			//val |= ~(AON_BIT_AON_WEVT_AON_RTC_MSK);
			//aon->AON_REG_AON_FUNC_CTRL = val;
		}
		break;

	case ISP_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val |= SYSON_S_BIT_SYS_ISP_EN ;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val |= SYSON_S_BIT_SYS_ISP_CLK_EN;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;

		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val &= ~SYSON_S_BIT_SYS_ISP_CLK_EN;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val &= ~SYSON_S_BIT_SYS_ISP_EN;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;


		}
		break;
	case NN_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
			val |= SYSON_S_BIT_SYS_NN_CLK_EN;
			syson_s->SYSON_S_REG_SYS_NN_CTRL = val;

			val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
			val |= SYSON_S_BIT_SYS_NN_EN;
			syson_s->SYSON_S_REG_SYS_NN_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
			val &= ~SYSON_S_BIT_SYS_NN_CLK_EN;
			syson_s->SYSON_S_REG_SYS_NN_CTRL = val;
			val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
			val &= ~SYSON_S_BIT_SYS_NN_EN;
			syson_s->SYSON_S_REG_SYS_NN_CTRL = val;
		}
		break;
	case SPORT_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_SPORT_EN | SYSON_S_BIT_SYS_SPORT_PCLK_EN | SYSON_S_BIT_SYS_SPORT_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SPORT_EN | SYSON_S_BIT_SYS_SPORT_PCLK_EN | SYSON_S_BIT_SYS_SPORT_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case I2S0_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val |= SYSON_S_BIT_SYS_I2S0_EN | SYSON_S_BIT_SYS_I2S0_CLK_EN;
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2S0_EN | SYSON_S_BIT_SYS_I2S0_CLK_EN);
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		}
		break;

	case I2S1_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val |= SYSON_S_BIT_SYS_I2S1_EN | SYSON_S_BIT_SYS_I2S1_CLK_EN;
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val &= ~(SYSON_S_BIT_SYS_I2S1_EN | SYSON_S_BIT_SYS_I2S1_CLK_EN);
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		}
		break;

	case LXBUS_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= SYSON_S_BIT_LXBUS_EN | SYSON_S_BIT_LXBUS_CLK_EN;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &= ~(SYSON_S_BIT_LXBUS_EN | SYSON_S_BIT_LXBUS_CLK_EN);
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
		}
		break;

	case SGPIO_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val |= SYSON_S_BIT_SYS_SGPIO0_INT_CLK_SEL | SYSON_S_BIT_SYS_SGPIO0_SCLK_EN | SYSON_S_BIT_SYS_SGPIO0_PCLK_EN | SYSON_S_BIT_SYS_SGPIO0_EN;
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_I2S_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SGPIO0_INT_CLK_SEL | SYSON_S_BIT_SYS_SGPIO0_SCLK_EN | SYSON_S_BIT_SYS_SGPIO0_PCLK_EN | SYSON_S_BIT_SYS_SGPIO0_EN);
			syson_s->SYSON_S_REG_SYS_I2S_CTRL = val;
		}
		break;

	case VOE_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val |= (SYSON_S_BIT_SYS_VOE_CLK_EN | SYSON_S_BIT_SYS_VOE_EN) ;
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;


		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
			val &= ~(SYSON_S_BIT_SYS_VOE_CLK_EN | SYSON_S_BIT_SYS_VOE_EN);
			syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;

		}
		break;


	case TRNG_32K:
		if (en == ENABLE) {

			//enable REG_AON_SRC_CLK_CTRL
			val = aon->AON_REG_AON_SRC_CLK_CTRL;
			val |= (AON_BIT_COMP_SCLK_SEL | (1 << AON_SHIFT_LP_CLK_SEL) | AON_BIT_TIMER_SCLK_SEL) ;
			aon->AON_REG_AON_SRC_CLK_CTRL = val;

			//  Enable PON in AON register
			val = aon->AON_REG_AON_FUNC_CTRL;
			val |= (AON_BIT_SDM_FEN | AON_BIT_SYS_PON_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// enable syson
			val = pon->PON_REG_PON_SYSON_CTRL;
			val |= (PON_BIT_SYSON_REG_EN | PON_BIT_SYSON_CLK_EN | PON_BIT_SYSON_LPC_EN | PON_BIT_PLATFORM_CLK_EN) ;
			pon->PON_REG_PON_SYSON_CTRL = val;

			//set SYSON_S_REG_SYS_PLATFORM_CTRL0
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val &= ~(SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL); //32k clock sources
			val |= (SYSON_S_BIT_HS_TRNG_HI_CLK_SEL  | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN) ;//32k clock sources
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;
			hal_delay_ms(5);

		} else { //en == DISABLE
			//disable SYSON_S_REG_SYS_PLATFORM_CTRL0
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= ~(SYSON_S_BIT_HS_TRNG_HI_CLK_SEL | SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN) ;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;

		}
		break;

	case TRNG_128K:
		if (en == ENABLE) {

			//enable REG_AON_SRC_CLK_CTRL
			val = aon->AON_REG_AON_SRC_CLK_CTRL;
			val |= (AON_BIT_COMP_SCLK_SEL | (1 << AON_SHIFT_LP_CLK_SEL) | AON_BIT_TIMER_SCLK_SEL) ;
			aon->AON_REG_AON_SRC_CLK_CTRL = val;

			//  Enable PON in AON register
			val = aon->AON_REG_AON_FUNC_CTRL;
			val |= (AON_BIT_SDM_FEN | AON_BIT_SYS_PON_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// enable syson
			val = pon->PON_REG_PON_SYSON_CTRL;
			val |= (PON_BIT_SYSON_REG_EN | PON_BIT_SYSON_CLK_EN | PON_BIT_SYSON_LPC_EN | PON_BIT_PLATFORM_CLK_EN) ;
			pon->PON_REG_PON_SYSON_CTRL = val;
			// set up trng
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= (SYSON_S_BIT_HS_TRNG_HI_CLK_SEL  | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN | SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL) ; //128L clock
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;


		} else { //en == DISABLE

			//disable SYSON_S_REG_SYS_PLATFORM_CTRL0
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= ~(SYSON_S_BIT_HS_TRNG_HI_CLK_SEL | SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN) ;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;

		}
		break;
	case TRNG_SYS:
		if (en == ENABLE) {

			//enable REG_AON_SRC_CLK_CTRL
			val = aon->AON_REG_AON_SRC_CLK_CTRL;
			val |= (AON_BIT_COMP_SCLK_SEL | (1 << AON_SHIFT_LP_CLK_SEL) | AON_BIT_TIMER_SCLK_SEL) ;
			aon->AON_REG_AON_SRC_CLK_CTRL = val;

			//  Enable PON in AON register
			val = aon->AON_REG_AON_FUNC_CTRL;
			val |= (AON_BIT_SDM_FEN | AON_BIT_SYS_PON_FEN);
			aon->AON_REG_AON_FUNC_CTRL = val;

			// enable syson
			val = pon->PON_REG_PON_SYSON_CTRL;
			val |= (PON_BIT_SYSON_REG_EN | PON_BIT_SYSON_CLK_EN | PON_BIT_SYSON_LPC_EN | PON_BIT_PLATFORM_CLK_EN) ;
			pon->PON_REG_PON_SYSON_CTRL = val;
			// set up trng
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= (SYSON_S_BIT_HS_TRNG_HI_CLK_SEL  | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN | SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL) ; //128L clock
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;


		} else { //en == DISABLE

			//disable SYSON_S_REG_SYS_PLATFORM_CTRL0
			val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
			val |= ~(SYSON_S_BIT_HS_TRNG_HI_CLK_SEL | SYSON_S_BIT_HS_TRNG_LOW_CLK_SEL | SYSON_S_BIT_HS_TRNG_CLK_EN | SYSON_S_BIT_HS_TRNG_EN) ;
			syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;

		}
		break;
	case SI_SYS:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_SI_EN | SYSON_S_BIT_SYS_SI_PCLK_EN | SYSON_S_BIT_SYS_SI_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SI_EN | SYSON_S_BIT_SYS_SI_PCLK_EN | SYSON_S_BIT_SYS_SI_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case AUDIO_CODEC_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_CODEC_EN;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_CODEC_EN);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case AUDIO_CODEC_SCLK_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_CODEC_SCLK_EN;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_CODEC_SCLK_EN);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case AUDIO_CODEC_PORB_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_CODEC_PORB;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_CODEC_PORB);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case AUDIO_CODEC_LDO_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_CODEC_LDO_EN;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_CODEC_LDO_EN);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case AUDIO_CODEC_EXTE_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val |= SYSON_S_BIT_SYS_SPORT_CODEC_SEL;
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		} else { //en == DISABLE
			val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
			val &= ~(SYSON_S_BIT_SYS_SPORT_CODEC_SEL);
			syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		}
		break;

	case LDO_SDIO_3V3_EN:
		if (en == ENABLE) {
			val = syson_s->SYSON_S_REG_LDO_SDIO_CTRL;

			// 1. Use PMOS Buffer, 3.3V
			val &= ~(SYSON_S_BIT_REG_BUFFERTYPE_SEL_L);
			// 2. Power Cut Enable
			val |= SYSON_S_BIT_EN_PC_BT_L;
			// 3. Normal Mode
			val &= ~(SYSON_S_MASK_REG_STANDBY_L);
			// Assign into SYSON_S_REG_LDO_SDIO_CTRL first
			syson_s->SYSON_S_REG_LDO_SDIO_CTRL = val;

			hal_delay_us(20);

			// 4. Enable LDO SDIO, then assign back to register
			val = syson_s->SYSON_S_REG_LDO_SDIO_CTRL;
			val &= ~(SYSON_S_BIT_PD_REGU_L);
			syson_s->SYSON_S_REG_LDO_SDIO_CTRL = val;

			hal_delay_us(200);

			// 5. Diode disable
			val = syson_s->SYSON_S_REG_LDO_SDIO_CTRL;
			val &= ~(SYSON_S_BIT_EN_DIODE_L);
			syson_s->SYSON_S_REG_LDO_SDIO_CTRL = val;

		} else { //en == DISABLE
			// 1. Disable LDO SDIO and enable diode
			val = syson_s->SYSON_S_REG_LDO_SDIO_CTRL;
			val |= (SYSON_S_BIT_PD_REGU_L | SYSON_S_BIT_EN_DIODE_L);
			syson_s->SYSON_S_REG_LDO_SDIO_CTRL = val;
		}
		break;

	default:
		break;
	}
}


SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_set_clk(uint8_t id, uint8_t sel_val)
{
	AON_TypeDef *aon = AON;
	PON_TypeDef *pon = PON;
	SYSON_S_TypeDef *syson_s = SYSON_S;

	volatile uint32_t val;
	switch (id) {
	case UART0_SYS:
		val = pon->PON_REG_PON_FUNC_CLK_CTRL;
		val &= (~PON_MASK_SCLK_UART0_SEL);
		val |= ((sel_val & 0x3) << PON_SHIFT_SCLK_UART0_SEL);
		pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		break;
	case PWM_SYS:
		val = pon->PON_REG_PON_FUNC_CLK_CTRL;
		val &= (~PON_MASK_SCLK_PWM_SEL);
		val |= ((sel_val & 0x3) << PON_SHIFT_SCLK_PWM_SEL);
		pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		break;
	case TIMER0_SYS:
		val = pon->PON_REG_PON_FUNC_CLK_CTRL;
		val &= (~PON_MASK_SCLK_TIMER0_SEL);
		val |= ((sel_val & 0x3) << PON_SHIFT_SCLK_TIMER0_SEL);
		pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		break;
	case TIMER1_SYS:
		val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
		val &= (~SYSON_S_MASK_SYS_TG1_SRC_SEL);
		val |= ((sel_val & 0x3) << SYSON_S_SHIFT_SYS_TG1_SRC_SEL);
		syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		break;
	case TIMER2_SYS:
		val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
		val &= (~SYSON_S_MASK_SYS_TG2_SRC_SEL);
		val |= ((sel_val & 0x3) << SYSON_S_SHIFT_SYS_TG2_SRC_SEL);
		syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		break;
	case TIMER3_SYS:
		val = syson_s->SYSON_S_REG_SYS_TIMER_CTRL;
		val &= (~SYSON_S_MASK_SYS_TG3_SRC_SEL);
		val |= ((sel_val & 0x3) << SYSON_S_SHIFT_SYS_TG3_SRC_SEL);
		syson_s->SYSON_S_REG_SYS_TIMER_CTRL = val;
		break;
	case ENC_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val &= ~SYSON_S_MASK_SYS_ENCODER_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_ENCODER_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
		break;
	case NN_SYS:
		val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
		val &= ~SYSON_S_MASK_SYS_NN_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_NN_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_NN_CTRL = val;
		break;
	case SPORT_SYS:
		val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
		val &= ~SYSON_S_MASK_SYS_SPORT_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_SPORT_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		break;
	case ISP_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val &= ~SYSON_S_MASK_SYS_ISP_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_ISP_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
		break;
	// 0: 200Mhz /s4, 1: 166Mhz /c5 2: 125Mhz /c2
	// 3: 100Mhz /s5, 4: 83Mhz /c6

	case MIPI_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val &= ~SYSON_S_MASK_SYS_MIPI_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_MIPI_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_ISP_CTRL = val;
		break;
	// 0: 250Mhz /c1, 1: 200Mhz /s4, 2: 166Mhz /c5
	// 3: 125Mhz /c2, 4: 100Mhz /s5, 5: 83Mhz /c6

	case FLASH_SYS:
		val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
		val &= ~SYSON_S_BIT_SYS_FLASH_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_FLASH_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_FLASH_CTRL = val;
		break;
	// 0: 250Mhz /c1, 1: 200Mhz

	case GPIO_AON:
		// interrupt clk source selection for AON GPIO
		// 0: 100kHz, /1: 4MHz
		val = aon->AON_REG_AON_CLK_CTRL;
		val &= ~AON_BIT_INT_CLK_SEL;
		val |= sel_val << AON_SHIFT_INT_CLK_SEL;
		aon->AON_REG_AON_CLK_CTRL = val;
		break;

	case GPIO_PON:
		// interrupt clk source for PON GPIO
		// 0:APB CLK /1:32kHz
		val = pon->PON_REG_PON_FUNC_CLK_CTRL;
		val &= ~PON_BIT_INTR_CLK_GPIO_SEL;
		val |= sel_val << PON_SHIFT_INTR_CLK_GPIO_SEL;
		pon->PON_REG_PON_FUNC_CLK_CTRL = val;
		break;

	case GPIO_SYS:
		// interrupt clk source for SYSON GPIO
		// 0: APB, /1: debounce clock
		val = syson_s->SYSON_S_REG_GPIO_CTRL;
		val &= ~SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_GPIO_INTR_CLK_SEL;
		syson_s->SYSON_S_REG_GPIO_CTRL = val;
		break;

	case AUDIO_CODEC_LDO_TUNE:
		val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
		val &= ~SYSON_S_MASK_SYS_CODEC_LDO_TUNE;
		val |= sel_val << SYSON_S_SHIFT_SYS_CODEC_LDO_TUNE;
		syson_s->SYSON_S_REG_SYS_AUDIO_CTRL = val;
		break;

	default:
		break;
	}
}


SECTION_SYS_CTRL_TEXT
uint32_t hal_rtl_sys_get_clk(uint8_t id)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	AON_TypeDef *aon = AON;
	PON_TypeDef *pon = PON;
	volatile uint32_t val = 0;


	switch (id) {
	case CPU_SYS:
		val = aon->AON_REG_AON_PMC_CTRL;
		val = (val & AON_MASK_SYSON_PLL_SWITCH) >> AON_SHIFT_SYSON_PLL_SWITCH;
		// 0:500Mhz, 1:400Mhz 2:300Mhz
		break;
	case ENC_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val = (val & SYSON_S_MASK_SYS_ENCODER_SRC_SEL) >> SYSON_S_SHIFT_SYS_ENCODER_SRC_SEL;
		// 0: 166Mhz /c5, 1: 125Mhz /c2 2: 100Mhz /s5
		break;
	case ISP_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val = (val & SYSON_S_MASK_SYS_ISP_SRC_SEL) >> SYSON_S_SHIFT_SYS_ISP_SRC_SEL;
		// 0: 200Mhz /s4, 1: 166Mhz /c5 2: 125Mhz /c2
		// 3: 100Mhz /s5, 4: 83Mhz /c6
		break;
	case MIPI_SYS:
		val = syson_s->SYSON_S_REG_SYS_ISP_CTRL;
		val = (val & SYSON_S_MASK_SYS_MIPI_SRC_SEL) >> SYSON_S_SHIFT_SYS_MIPI_SRC_SEL;
		// 0: 250Mhz /c1, 1: 200Mhz /s4, 2: 166Mhz /c5
		// 3: 125Mhz /c2, 4: 100Mhz /s5, 5: 83Mhz /c6
		break;
	case NN_SYS:
		val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
		val = (val & SYSON_S_MASK_SYS_NN_SRC_SEL) >> SYSON_S_SHIFT_SYS_NN_SRC_SEL;
		// 0: 500Mhz /c0, 1: 400Mhz /s15, 2: 250Mhz /c1
		break;
	case FLASH_SYS:
		val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
		val = (val & SYSON_S_BIT_SYS_FLASH_SRC_SEL) >> SYSON_S_SHIFT_SYS_FLASH_SRC_SEL;
		break;
	case SPORT_SYS:
		val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
		val = (val & SYSON_S_MASK_SYS_SPORT_SRC_SEL) >> SYSON_S_SHIFT_SYS_SPORT_SRC_SEL;
		// 0: 40Mhz , 1: 98.304Mhz, 2: 45.158Mhz
		break;

	case GPIO_AON:
		// Get interrupt clk source selection for AON GPIO
		val = aon->AON_REG_AON_CLK_CTRL;
		val = (val & AON_BIT_INT_CLK_SEL) >> AON_SHIFT_INT_CLK_SEL;
		// 0: 100kHz, /1: 4MHz
		break;

	case GPIO_PON:
		// interrupt clk source for PON GPIO
		val = pon->PON_REG_PON_FUNC_CLK_CTRL;
		val = (val & PON_BIT_INTR_CLK_GPIO_SEL) >> PON_SHIFT_INTR_CLK_GPIO_SEL;
		// 0:APB CLK /1:32kHz
		break;

	case GPIO_SYS:
		// interrupt clk source for SYSON GPIO
		val = syson_s->SYSON_S_REG_GPIO_CTRL;
		val = (val & SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL) >> SYSON_S_SHIFT_SYS_GPIO_INTR_CLK_SEL;
		// 0: APB, /1: debounce clock
		break;

	case AUDIO_CODEC_LDO_TUNE:
		val = syson_s->SYSON_S_REG_SYS_AUDIO_CTRL;
		val = (val & SYSON_S_MASK_SYS_CODEC_LDO_TUNE) >> SYSON_S_SHIFT_SYS_CODEC_LDO_TUNE;
		break;

	default:
		break;
	}
	return val;

}

/**
 *  @brief Write the non-fixed secure JTAG key.
 *
 *  @param[in] pkey The key data of the non-fixed secure JTAG.
 *
 *  @returns void
 */
SECTION_SYS_CTRL_TEXT
void hal_rtl_vdr_s_jtag_key_write(uint8_t *pkey)
{
	VNDR_S_TypeDef *vdr_s = (VNDR_S_TypeDef *)VNDR_S_BASE;
	volatile uint32_t *pkey_reg;
	uint32_t i;
	uint32_t key_data;

	pkey_reg = &(vdr_s->VNDR_S_REG_NON_FIXED_SECURE_KEY0);
	//dbg_printf("pkey_reg S:%x\r\n", pkey_reg); // weide for debug

	for (i = 0; i < 8; i++) {
		key_data = *(pkey + 3) << 24 | *(pkey + 2) << 16 | *(pkey + 1) << 8 | *(pkey);
		*pkey_reg = key_data;
		//dbg_printf("pkey_reg++ S:%x Data:%x\r\n", pkey_reg, *pkey_reg); // weide for debug
		pkey_reg++;
		pkey += 4;
	}
	vdr_s->VNDR_S_REG_SECURE_JTAG_SWD_CTRL |= VNDR_S_BIT_FW_READY_SC;
}


/**
 *  @brief Write the non-fixed non-secure JTAG key.
 *
 *  @param[in] pkey The key data of the non-fixed non-secure JTAG.
 *
 *  @returns void
 */
SECTION_SYS_CTRL_TEXT
void hal_rtl_vdr_ns_jtag_key_write(uint8_t *pkey)
{
	VNDR_S_TypeDef *vdr_s = (VNDR_S_TypeDef *)VNDR_S_BASE;
	volatile uint32_t *pkey_reg;
	uint32_t i;
	uint32_t key_data;

	pkey_reg = &(vdr_s->VNDR_S_REG_NON_FIXED_NON_SECURE_KEY0);
	//dbg_printf("pkey_reg NS:%x\r\n", pkey_reg); // weide for debug

	for (i = 0; i < 8; i++) {
		key_data = *(pkey + 3) << 24 | *(pkey + 2) << 16 | *(pkey + 1) << 8 | *(pkey);
		*pkey_reg = key_data;
		//dbg_printf("pkey_reg++ NS:%x Data:%x\r\n", pkey_reg, *pkey_reg); // weide for debug
		pkey_reg++;
		pkey += 4;
	}
	vdr_s->VNDR_S_REG_SECURE_JTAG_SWD_CTRL |= VNDR_S_BIT_FW_READY_NONSC;
}

SECTION_SYS_CTRL_TEXT
uint32_t hal_rtl_sys_get_video_info(uint8_t idx)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t val = 0;

	switch (idx) {
	case VIDEO_INFO_NN:
		val = ((aon->AON_REG_AON_OTP_SYSCFG5 & AON_BIT_NN_CTRL) >> AON_SHIFT_NN_CTRL);
		break;
	case VIDEO_INFO_H265:
		val = ((aon->AON_REG_AON_OTP_SYSCFG5 & AON_BIT_H265_CTRL) >> AON_SHIFT_H265_CTRL);
		break;
	case VIDEO_INFO_ENC:
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
		val = ((aon->AON_REG_AON_OTP_SYSCFG5 & AON_BIT_ENC_CTRL) >> AON_SHIFT_ENC_CTRL);
#endif
		break;
	case VIDEO_INFO_ISP:
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
		val = ((aon->AON_REG_AON_OTP_SYSCFG5 & AON_BIT_ISP_CTRL) >> AON_SHIFT_ISP_CTRL);
#endif
		break;
	default:
		dbg_printf("get video_info fail idx!(%d)\r\n", idx);
		break;
	}
	return val;
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_set_video_info(uint8_t idx, uint8_t en_ctrl)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t val = 0;

	val = (aon->AON_REG_AON_OTP_SYSCFG5);
	switch (idx) {
	case VIDEO_INFO_NN:
		if (ENABLE == en_ctrl) {
			val |= AON_BIT_NN_CTRL;
		} else {    // Disable
			val &= (~AON_BIT_NN_CTRL);
		}
		break;
	case VIDEO_INFO_H265:
		if (ENABLE == en_ctrl) {
			val |= AON_BIT_H265_CTRL;
		} else {    // Disable
			val &= (~AON_BIT_H265_CTRL);
		}
		break;
	case VIDEO_INFO_ENC:
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
		if (ENABLE == en_ctrl) {
			val |= AON_BIT_ENC_CTRL;
		} else {    // Disable
			val &= (~AON_BIT_ENC_CTRL);
		}
#endif
		break;
	case VIDEO_INFO_ISP:
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
		if (ENABLE == en_ctrl) {
			val |= AON_BIT_ISP_CTRL;
		} else {    // Disable
			val &= (~AON_BIT_ISP_CTRL);
		}
#endif
		break;
	case VIDEO_INFO_ALL:
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		val &= (~(AON_BIT_NN_CTRL | AON_BIT_H265_CTRL));       // clear [10:9]
#else
		val &= (~(AON_BIT_NN_CTRL | AON_BIT_H265_CTRL | AON_BIT_ENC_CTRL | AON_BIT_ISP_CTRL));       // clear [12:9]
#endif
		val |= ((en_ctrl & 0xF) << AON_SHIFT_NN_CTRL);
		break;
	default:
		dbg_printf("set video_info fail idx!(%d)\r\n", idx);
		return;
		break;
	}
	aon->AON_REG_AON_OTP_SYSCFG5 = val;
}

SECTION_SYS_CTRL_TEXT
void hal_rtl_sys_get_chip_id(uint32_t *pchip_id)
{
	uint32_t otp_idx = 0x0;
	uint8_t otp_data[OTP_CHIP_ID_MAX_SIZE];
	uint8_t *p_otp_v = NULL;

	hal_rtl_otp_init();
	p_otp_v = &otp_data[0];
	_memset(p_otp_v, 0xFF, OTP_CHIP_ID_MAX_SIZE);
	*pchip_id = 0x0;
	// Get chip ID from OTP
	hal_rtl_otp_rd_syss(OTP_ENABLE, OTP_CID_DAT_ADDR, p_otp_v, OTP_CHIP_ID_MAX_SIZE);
	for (otp_idx = 0; otp_idx < OTP_CHIP_ID_MAX_SIZE; otp_idx++) {
		//*(p_otp_v + otp_idx) = hal_rtl_otp_byte_rd_syss((OTP_CID_DAT_ADDR + otp_idx));
		*pchip_id |= (*(p_otp_v + otp_idx) << (8 * otp_idx));
	}
	hal_rtl_otp_deinit();
}


/** @} */ /* End of group hs_hal_efuse */
