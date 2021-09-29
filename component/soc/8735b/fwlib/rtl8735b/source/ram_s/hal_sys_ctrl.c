/**************************************************************************//**
* @file        hal_sys_ctrl.c
* @brief       This file implements the SYSON control HAL functions.
*
* @version     V1.00
* @date        2021-07-20
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
#include "hal.h"

#define IDAU_CONTOL_BASEADDR            (0x50004100)
#define IDAU_CONTOL_LIMIT_OFFSET        (0x4)
#define IDAU_CONTOL_ENCTRL_OFFSET       (0x40)

extern hal_sys_ctrl_func_stubs_t hal_sys_ctrl_stubs;     // symbol from linker script

/**

        \addtogroup hal_sys_ctrl
        @{
*/

void hal_sys_get_chip_id(uint32_t *pchip_id)
{
#if IS_CUT_TEST(CONFIG_CHIP_VER)

	uint32_t otp_idx = 0x0;
	uint8_t otp_data[OTP_CHIP_ID_MAX_SIZE];
	uint8_t *p_otp_v = NULL;

	hal_otp_init();
	p_otp_v = &otp_data[0];
	memset(p_otp_v, 0xFF, OTP_CHIP_ID_MAX_SIZE);
	*pchip_id = 0x0;
	// Get chip ID from OTP
	hal_otp_rd_syss(OTP_ENABLE, OTP_CID_DAT_ADDR, p_otp_v, OTP_CHIP_ID_MAX_SIZE);
	for (otp_idx = 0; otp_idx < OTP_CHIP_ID_MAX_SIZE; otp_idx++) {
		*pchip_id |= (*(p_otp_v + otp_idx) << (8 * otp_idx));
	}
	hal_otp_deinit();
#else
    hal_sys_ctrl_stubs.hal_sys_get_chip_id(pchip_id);
#endif
}
void hal_sys_peripheral_en(uint8_t id, uint8_t en)
{

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	SYSON_S_TypeDef *syson_s = SYSON_S;
	PON_TypeDef *pon = PON;
	AON_TypeDef *aon = AON;
	volatile uint32_t val;

	if ((id == GPIO_PON) ||
		(id == NN_SYS) ||
		(id == ADC_SYS) ||
		(id == SI_SYS) ||
		(id == AUDIO_CODEC_EN) ||
		(id == AUDIO_CODEC_SCLK_EN) ||
		(id == AUDIO_CODEC_PORB_EN) ||
		(id == AUDIO_CODEC_LDO_EN) ||
		(id == AUDIO_CODEC_EXTE_EN) ||
		(id == SGPIO_SYS) ||
		(id == LDO_SDIO_3V3_EN) ||
		(id == TRNG_128K) ||
		(id == TRNG_32K) ||
		(id == TRNG_SYS)) {
		hal_rtl_sys_peripheral_en_patch(id, en);
	} else {
		hal_sys_ctrl_stubs.hal_sys_peripheral_en(id, en);
	}
#else
	hal_sys_ctrl_stubs.hal_sys_peripheral_en(id, en);
#endif

}

void hal_sys_set_clk(uint8_t id, uint8_t sel_val)
{

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (
		(id == NN_SYS) ||
		(id == AUDIO_CODEC_LDO_TUNE)) {
		hal_rtl_sys_set_clk_patch(id, sel_val);
	} else {
		hal_sys_ctrl_stubs.hal_sys_set_clk(id, sel_val);
	}
#else
	hal_sys_ctrl_stubs.hal_sys_set_clk(id, sel_val);
#endif

}

uint32_t hal_sys_get_clk(uint8_t id)
{

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (
		(id == NN_SYS) ||
		(id == AUDIO_CODEC_LDO_TUNE)) {
		return hal_rtl_sys_get_clk_patch(id);
	} else {
		return hal_sys_ctrl_stubs.hal_sys_get_clk(id);
	}
#else
	return hal_sys_ctrl_stubs.hal_sys_get_clk(id);
#endif

}

uint32_t hal_sys_boot_info_get_val(uint8_t info_idx)
{
	return (hal_sys_ctrl_stubs.hal_sys_boot_info_get_val(info_idx));
}

void hal_sys_boot_info_assign_val(uint8_t info_idx, uint32_t info_v)
{
	hal_sys_ctrl_stubs.hal_sys_boot_info_assign_val(info_idx, info_v);
}

void hal_sys_boot_footpath_init(uint8_t info_idx)
{
	hal_sys_ctrl_stubs.hal_sys_boot_footpath_init(info_idx);
}

void hal_sys_boot_footpath_store(uint8_t info_idx, uint8_t fp_v)
{
	hal_sys_ctrl_stubs.hal_sys_boot_footpath_store(info_idx, fp_v);
}

void hal_sys_boot_footpath_clear(uint8_t info_idx, uint8_t fp_v)
{
	hal_sys_ctrl_stubs.hal_sys_boot_footpath_clear(info_idx, fp_v);
}

void hal_vdr_s_jtag_key_write(uint8_t *pkey)
{
	hal_sys_ctrl_stubs.hal_vdr_s_jtag_key_write(pkey);
}

void hal_vdr_ns_jtag_key_write(uint8_t *pkey)
{
	hal_sys_ctrl_stubs.hal_vdr_ns_jtag_key_write(pkey);
}

uint32_t hal_sys_get_video_info(uint8_t idx)
{

	return (hal_sys_ctrl_stubs.hal_sys_get_video_info(idx));

}


/**
 *  @brief To setup a Bus IDAU to open the bus accessing for a non-secure region of memory
 *
 *  @returns void
 */
void hal_sys_set_bus_idau(uint32_t idau_idx, uint32_t start_addr, uint32_t end_addr)
{
	__DSB();
	__ISB();
	HAL_WRITE32(IDAU_CONTOL_BASEADDR, (idau_idx * 8), start_addr);
	HAL_WRITE32(IDAU_CONTOL_BASEADDR, IDAU_CONTOL_LIMIT_OFFSET + (idau_idx * 8), end_addr);
	HAL_WRITE32(IDAU_CONTOL_BASEADDR, IDAU_CONTOL_ENCTRL_OFFSET, HAL_READ32(IDAU_CONTOL_BASEADDR, IDAU_CONTOL_ENCTRL_OFFSET) | (1 << idau_idx));
	__DSB();
	__ISB();
}

/**
 *  @brief To enable wlan SDM Function.
 *
 *  @param[in] bypass_mode: 1: bypass mode, simple /4 path for 128k -> 32K;
 *
 *  @returns void
 */
void hal_sdm_32k_enable(u8 bypass_mode)
{
	AON_TypeDef *aon = AON;
	aon->AON_REG_AON_FUNC_CTRL |= AON_BIT_SDM_FEN;
	aon->AON_REG_AON_CLK_CTRL |= AON_BIT_OSC_SDM_CK_EN;
	aon->AON_REG_AON_LSFIF_RWD = 0x80000000;
	aon->AON_REG_AON_LSFIF_CMD = 0;
	aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
	aon->AON_REG_AON_LSFIF_CMD |= (0x1 << AON_SHIFT_LSF_SEL);
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;
	u32 i = 0;

	for (i = 0; i < 1000000; i++) {
		if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
			break;  // break the for loop
		} else {
			hal_delay_us(100);
		}
	}

	if (bypass_mode) {
		aon->AON_REG_AON_LSFIF_RWD = 0xE0000000;   // Enable Bypass Mode
	} else {
		aon->AON_REG_AON_LSFIF_RWD = 0xC0000000;   // Disable Bypass Mode
	}
	i = 0;
	aon->AON_REG_AON_LSFIF_CMD = 0;
	aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
	aon->AON_REG_AON_LSFIF_CMD |= (0x1 << AON_SHIFT_LSF_SEL);
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;

	for (i = 0; i < 1000000; i++) {
		if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
			break;  // break the for loop
		} else {
			hal_delay_us(100);
		}
	}

}

/**
 *  @brief Reads the time loss calculation result of the 32K SDM hardware.
 *
 *  @returns    The time loss of the SDM calculation result.
 */
u32 hal_read_sdm_32k_time_loss(void)
{
	AON_TypeDef *aon = AON;
	aon->AON_REG_AON_FUNC_CTRL |= AON_BIT_SDM_FEN;
	aon->AON_REG_AON_CLK_CTRL |= AON_BIT_OSC_SDM_CK_EN;
	aon->AON_REG_AON_LSFIF_CMD = 0x6;
	aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
	aon->AON_REG_AON_LSFIF_CMD |= (0x1 << AON_SHIFT_LSF_SEL);
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;
	u32 i = 0;

	for (i = 0; i < 1000000; i++) {
		if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
			break;  // break the for loop
		} else {
			hal_delay_us(100);
		}
	}
	return aon->AON_REG_AON_LSFIF_RWD;
}

/**
 *  @brief To enable XTAL Divider Function.
 *
 *  @param[in] enable: 1: enable xtal divider to output 31.25kHz;
 *
 *  @returns void
 */
void hal_xtal_divider_enable(u8 enable)
{
	AON_TypeDef *aon = AON;
	if (enable == 1) {
		aon->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_LPS;
		aon->AON_REG_AON_SRC_CLK_CTRL |= AON_BIT_XTAL_DIVIDER_EN;
		aon->AON_REG_AON_LSFIF_RWD = 0x40008;
		aon->AON_REG_AON_LSFIF_CMD = 1;
		aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
		aon->AON_REG_AON_LSFIF_CMD |= (0x2 << AON_SHIFT_LSF_SEL);
		aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;
		u32 i = 0;

		for (i = 0; i < 1000000; i++) {
			if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
				break;  // break the for loop
			} else {
				hal_delay_us(100);
			}
		}
		i = 0;
		aon->AON_REG_AON_LSFIF_RWD = 0x40009;
		aon->AON_REG_AON_LSFIF_CMD = 2;
		aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
		aon->AON_REG_AON_LSFIF_CMD |= (0x2 << AON_SHIFT_LSF_SEL);
		aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;

		for (i = 0; i < 1000000; i++) {
			if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
				break;  // break the for loop
			} else {
				hal_delay_us(100);
			}
		}
		i = 0;
		aon->AON_REG_AON_LSFIF_RWD = 0x602010A;
		aon->AON_REG_AON_LSFIF_CMD = 3;
		aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
		aon->AON_REG_AON_LSFIF_CMD |= (0x2 << AON_SHIFT_LSF_SEL);
		aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;

		for (i = 0; i < 1000000; i++) {
			if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
				break;  // break the for loop
			} else {
				hal_delay_us(100);
			}
		}
		i = 0;
		aon->AON_REG_AON_LSFIF_RWD = 0xE00009D6;
		aon->AON_REG_AON_LSFIF_CMD = 0;
		aon->AON_REG_AON_LSFIF_CMD |= (AON_MASK_AON_LSFIF_WE | AON_BIT_AON_LSFIF_WR);
		aon->AON_REG_AON_LSFIF_CMD |= (0x2 << AON_SHIFT_LSF_SEL);
		aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_POLL;

		for (i = 0; i < 1000000; i++) {
			if ((aon->AON_REG_AON_LSFIF_CMD & AON_BIT_AON_LSFIF_POLL) != AON_BIT_AON_LSFIF_POLL) {
				break;  // break the for loop
			} else {
				hal_delay_us(100);
			}
		}
	} else {
		aon->AON_REG_AON_XTAL_CLK_CTRL1 &= ~(AON_BIT_EN_XTAL_DRV_LPS);
		aon->AON_REG_AON_SRC_CLK_CTRL &= ~(AON_BIT_XTAL_DIVIDER_EN);
	}

}

/**
 *  @brief To enableAON WDT Function.
 *
 *  @param[in] enable: 1: enable AON WDT;
 *  @param[in] timeout: set timeout value and max timeout value was 65535(65 sec);
 *
 *  @returns void
 */
void hal_aon_wdt_enable(u8 enable, u32 timeout)
{
	AON_TypeDef *aon = AON;
	if (enable == 1) {
		if (timeout > 65535) {
			timeout = 65535;
		}
		aon->AON_REG_AON_WDT_TIMER |= (timeout << AON_SHIFT_CNT);
		aon->AON_REG_AON_WDT_TIMER |= AON_BIT_WDT_EN_BYTE;
	} else {
		aon->AON_REG_AON_WDT_TIMER &= ~(AON_BIT_WDT_EN_BYTE);
	}

}

/**
 *  @brief To calbriate  OSC4M.
 *
 *  @returns void
 */
void hal_osc4m_cal(void)
{
	AON_TypeDef *aon = AON;
	VNDR_TypeDef *vendor = VNDR;
	u32 ctrl, sel, f, i;
	u8 state = 0;
	//dbg_printf("OSC4M CAL\r\n");
	while (state == 0) {
		ctrl = aon->AON_REG_AON_OSC4M_CTRL;//HAL_READ32(0x40009000, 0x50);
		sel = ctrl & 0x3C000;
		sel = sel >> 14;
		vendor->VNDR_REG_ANACK_CAL_CTRL |= VNDR_BIT_VDR_ANACK_CAL_CMD;//HAL_WRITE32(0x40002800, 0x14, 0x80000000);
		i = vendor->VNDR_REG_ANACK_CAL_CTRL;
		while ((i & VNDR_BIT_VDR_ANACK_CAL_CMD) != 0) {
			i = vendor->VNDR_REG_ANACK_CAL_CTRL;
		}
		i = i & 0xFFFF;
		if (i > 0) {
			f = 1000000 / i;
		}
		if ((f < 4100) && (f > 3899)) {
			state = 1;
		} else {
			if (f > 4000) {
				sel++;
				if (sel > 15) {
					sel = 15;
				}
				sel = ((sel << 14) | (sel << 18));
				ctrl = ctrl & 0xFFC03FFF;
				ctrl = ctrl | sel;

				//dbg_printf("CALV1:%x\r\n",ctrl);
				aon->AON_REG_AON_OSC4M_CTRL = ctrl;//HAL_WRITE32(0x40009000, 0x50, ctrl);
			} else {
				if (sel > 1) {
					sel--;
					sel = (sel << 14) | (sel << 18);
					ctrl = ctrl & 0xFFC03FFF;
					ctrl = ctrl | sel;
					//dbg_printf("CALV2:%x\r\n",ctrl);
					aon->AON_REG_AON_OSC4M_CTRL = ctrl;//HAL_WRITE32(0x40009000, 0x50, ctrl);
				} else {
					state = 3;
					dbg_printf("OSC4M VCAL Failed\r\n");
				}
			}
		}
	}
	while (state == 1) {
		ctrl = aon->AON_REG_AON_OSC4M_CTRL;//HAL_READ32(0x40009000, 0x50);
		sel = ctrl & 0x3FC0;
		sel = sel >> 6;
		vendor->VNDR_REG_ANACK_CAL_CTRL |= VNDR_BIT_VDR_ANACK_CAL_CMD;//HAL_WRITE32(0x40002800, 0x14, 0x80000000);
		i = vendor->VNDR_REG_ANACK_CAL_CTRL;
		while ((i & VNDR_BIT_VDR_ANACK_CAL_CMD) != 0) {
			i = vendor->VNDR_REG_ANACK_CAL_CTRL;
		}
		i = i & 0xFFFF;
		if (i > 0) {
			f = 1000000 / i;
		}
		if ((f < 4050) && (f > 3949)) {
			state = 3;
		} else {
			if (f > 4000) {
				sel++;
				if (sel > 127) {
					sel = 127;
				}
				sel = (sel << 6);
				ctrl = ctrl & 0xFFFFC03F;
				ctrl = ctrl | sel;
				//dbg_printf("CALR1:%x\r\n",ctrl);
				aon->AON_REG_AON_OSC4M_CTRL = ctrl;//HAL_WRITE32(0x40009000, 0x50, ctrl);
			} else {
				if (sel > 1) {
					sel--;
					sel = (sel << 6);
					ctrl = ctrl & 0xFFFFC03F;
					ctrl = ctrl | sel;
					//dbg_printf("CALR2:%x\r\n",ctrl);
					aon->AON_REG_AON_OSC4M_CTRL = ctrl;//HAL_WRITE32(0x40009000, 0x50, ctrl);
				} else {
					state = 3;
					dbg_printf("OSC4M RCAL Failed\r\n");
				}
			}
		}
	}

}

/**
 *  @brief To enable PLL 98.304MHz.
 *
 *  @param[in] enable: 1: enable, 0: disable
 *  @param[in] clk_src: 0 is that xtal is 40MHz. 1 is that xtal is 26MHz.
 *
 *  @returns void
 */
void hal_pll_98p304_ctrl(u8 en, u8 clk_src)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;

	if (en == ENABLE) {
		//(1)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_D2_I2S1 | SYSON_S_BIT_REG_CK_EN_I2S1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(2)
		if (clk_src == 0x00) {
			val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL1;
			val &= ~(SYSON_S_MASK_DIVN_SDM_I2S1);
			val |= (0x07) << SYSON_S_SHIFT_DIVN_SDM_I2S1;
			syson_s->SYSON_S_REG_I2S_PLL1_CTRL1 = val;

			val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL3;
			val &= ~(SYSON_S_MASK_F0N_SDM_I2S1 | SYSON_S_MASK_F0F_SDM_I2S1);
			val |= (0x06) << SYSON_S_SHIFT_F0N_SDM_I2S1;
			val |= (0x1495) << SYSON_S_SHIFT_F0F_SDM_I2S1;
			syson_s->SYSON_S_REG_I2S_PLL1_CTRL3 = val;

		} else {
			val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL1;
			val &= ~(SYSON_S_MASK_DIVN_SDM_I2S1);
			val |= (0x0B) << SYSON_S_SHIFT_DIVN_SDM_I2S1;
			syson_s->SYSON_S_REG_I2S_PLL1_CTRL1 = val;

			val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL3;
			val &= ~(SYSON_S_MASK_F0N_SDM_I2S1 | SYSON_S_MASK_F0F_SDM_I2S1);
			val |= (0x00) << SYSON_S_SHIFT_F0N_SDM_I2S1;
			val |= (0x1FAA) << SYSON_S_SHIFT_F0F_SDM_I2S1;
			syson_s->SYSON_S_REG_I2S_PLL1_CTRL3 = val;
		}

		//(3)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL1;
		val &= ~(SYSON_S_MASK_REG_CK_OUT_SEL_I2S1_1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL1 = val;

		//(4)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val |= SYSON_S_BIT_POW_ERC_I2S1;
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;
		hal_delay_us(1);

		//(5)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val |= SYSON_S_BIT_POW_PLL_I2S1;
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(6)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL1;
		val &= ~(SYSON_S_MASK_REG_CK_OUT_SEL_I2S1_1);
		val |= (0x0E) << SYSON_S_SHIFT_REG_CK_OUT_SEL_I2S1_1;
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL1 = val;

		//(7)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val |= SYSON_S_BIT_REG_CK_EN_D2_I2S1;
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(8)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val |= SYSON_S_BIT_REG_CK_EN_I2S1;
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(9)
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		val &= ~(SYSON_S_BIT_ISO_I2S1IPLL);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;

		while ((syson_s->SYSON_S_REG_SYS_CLK_CTRL & SYSON_S_BIT_I2S1IPLL_RDY) == 0x00);

	} else { //en == DISABLE
		//(1)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val &= ~(SYSON_S_BIT_POW_PLL_I2S1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(2)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_D2_I2S1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(3)
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_I2S1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

		//(4)
		hal_delay_us(1);
		val = syson_s->SYSON_S_REG_I2S_PLL1_CTRL0;
		val &= ~(SYSON_S_BIT_POW_ERC_I2S1);
		syson_s->SYSON_S_REG_I2S_PLL1_CTRL0 = val;

	}

}

/**
 *  @brief To enable PLL 45.158MHz.
 *
 *  @param[in] enable: 1: enable, 0: disable
 *  @param[in] clk_src: 0 is that xtal is 40MHz. 1 is that xtal is 26MHz.
 *
 *  @returns void
 */
void hal_pll_45p158_ctrl(u8 en, u8 clk_src)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;

	if (en == ENABLE) {
		//(1)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_D2_I2S2 | SYSON_S_BIT_REG_CK_EN_I2S2);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(2)
		if (clk_src == 0x00) {
			val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL1;
			val &= ~(SYSON_S_MASK_DIVN_SDM_I2S2);
			val |= (0x07) << SYSON_S_SHIFT_DIVN_SDM_I2S2;
			syson_s->SYSON_S_REG_I2S_PLL2_CTRL1 = val;

			val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL3;
			val &= ~(SYSON_S_MASK_F0N_SDM_I2S2 | SYSON_S_MASK_F0F_SDM_I2S2);
			val |= (0x00) << SYSON_S_SHIFT_F0N_SDM_I2S2;
			val |= (0x817) << SYSON_S_SHIFT_F0F_SDM_I2S2;
			syson_s->SYSON_S_REG_I2S_PLL2_CTRL3 = val;

		} else {
			val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL1;
			val &= ~(SYSON_S_MASK_DIVN_SDM_I2S2);
			val |= (0x0B) << SYSON_S_SHIFT_DIVN_SDM_I2S2;
			syson_s->SYSON_S_REG_I2S_PLL2_CTRL1 = val;

			val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL3;
			val &= ~(SYSON_S_MASK_F0N_SDM_I2S2 | SYSON_S_MASK_F0F_SDM_I2S2);
			val |= (0x07) << SYSON_S_SHIFT_F0N_SDM_I2S2;
			val |= (0x510) << SYSON_S_SHIFT_F0F_SDM_I2S2;
			syson_s->SYSON_S_REG_I2S_PLL2_CTRL3 = val;
		}

		//(3)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL1;
		val &= ~(SYSON_S_MASK_REG_CK_OUT_SEL_I2S2_1);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL1 = val;

		//(4)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val |= SYSON_S_BIT_POW_ERC_I2S2;
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;
		hal_delay_us(1);

		//(5)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val |= SYSON_S_BIT_POW_PLL_I2S2;
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(6)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL1;
		val &= ~(SYSON_S_MASK_REG_CK_OUT_SEL_I2S2_1);
		val |= (0x0C) << SYSON_S_SHIFT_REG_CK_OUT_SEL_I2S2_1;
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL1 = val;

		//(7)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val |= SYSON_S_BIT_REG_CK_EN_I2S2;
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(8)
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		val &= ~(SYSON_S_BIT_ISO_I2S2IPLL);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;

		while ((syson_s->SYSON_S_REG_SYS_CLK_CTRL & SYSON_S_BIT_I2S2IPLL_RDY) == 0x00);

	} else { //en == DISABLE
		//(1)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val &= ~(SYSON_S_BIT_POW_PLL_I2S2);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(2)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_D2_I2S2);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(3)
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val &= ~(SYSON_S_BIT_REG_CK_EN_I2S2);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

		//(4)
		hal_delay_us(1);
		val = syson_s->SYSON_S_REG_I2S_PLL2_CTRL0;
		val &= ~(SYSON_S_BIT_POW_ERC_I2S2);
		syson_s->SYSON_S_REG_I2S_PLL2_CTRL0 = val;

	}

}

void hal_sys_bt_uart_mux(uint8_t sel)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;

	if (sel == BT_UART_MUX_EXTERNAL) {
		val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
		val &= ~(SYSON_S_BIT_SYS_BT_UART_MUX_EN);
		syson_s->SYSON_S_REG_SYS_UART_CTRL = val;

	} else if (sel == BT_UART_MUX_INTERNAL) {
		val = syson_s->SYSON_S_REG_SYS_UART_CTRL;
		val |= SYSON_S_BIT_SYS_BT_UART_MUX_EN;
		syson_s->SYSON_S_REG_SYS_UART_CTRL = val;

	} else {
		dbg_printf("input param invalid\r\n");
	}
}

/**
 *  @brief To select  32k(S1) source  Function.
 *
 *  @param[in] sel:
 *              0: Clock from OSC 128k/4;
 *              1: Clock from OSC 128K with SDM;
 *              2: Clock from XTAL (high speed) with divider;
 *              3: Clock from XTAL 32K;
 *
 *  @returns void
 */
void hal_32k_s1_sel(u8 sel)
{
	AON_TypeDef *aon = AON;
	switch (sel) {
	case 0:
		aon->AON_REG_AON_SRC_CLK_CTRL &= ~(AON_MASK_LP_CLK_SEL);
		break;
	case 1:
		aon->AON_REG_AON_SRC_CLK_CTRL &= ~(AON_MASK_LP_CLK_SEL);
		aon->AON_REG_AON_SRC_CLK_CTRL |= (1 << AON_SHIFT_LP_CLK_SEL);
		break;
	case 2:
		aon->AON_REG_AON_SRC_CLK_CTRL &= ~(AON_MASK_LP_CLK_SEL);
		aon->AON_REG_AON_SRC_CLK_CTRL |= (3 << AON_SHIFT_LP_CLK_SEL);
		break;
	case 3:
		aon->AON_REG_AON_SRC_CLK_CTRL &= ~(AON_MASK_LP_CLK_SEL);
		aon->AON_REG_AON_SRC_CLK_CTRL |= (4 << AON_SHIFT_LP_CLK_SEL);
		break;
	}
}

/** @} */ /* End of group hal_sys_ctrl */
