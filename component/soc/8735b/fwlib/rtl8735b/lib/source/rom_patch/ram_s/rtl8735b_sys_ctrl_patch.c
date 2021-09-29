/**************************************************************************//**
* @file        rtl8735b_sys_ctrl_patch.c
* @brief       This file implements the SYSON control HAL ROM patch functions.
*
* @version     V1.00
* @date        2021-05-12
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

/**

        \addtogroup hal_sys_ctrl
        @{
*/

#if IS_CUT_TEST(CONFIG_CHIP_VER)
void hal_rtl_sys_peripheral_en_patch(uint8_t id, uint8_t en)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	PON_TypeDef *pon = PON;
	AON_TypeDef *aon = AON;
	volatile uint32_t val;

	switch (id) {
	case EDDSA_SYS:
		break;
	case GPIO_SYS:
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
		break;
	case DDR_SYS:
		break;
	case ENC_SYS:
		break;
	case GDMA0_SYS:
		break;
	case GDMA1_SYS:
		break;
	case FLASH_SYS: // NOR!@flash
		break;
	case SPI0_SYS:
		break;
	case SPI1_SYS:
		break;
	case HS_SPI0_SYS:
		break;
	case HS_SPI1_SYS:
		break;
	case UART0_SYS:
		break;
	case UART1_SYS:
		break;
	case UART2_SYS:
		break;
	case UART3_SYS:
		break;
	case UART4_SYS:
		break;
	case TIMER0_SYS:
		break;
	case TIMER1_SYS:
		break;
	case TIMER2_SYS:
		break;
	case TIMER3_SYS:
		break;
	case PWM_SYS:
		break;
	case CRYPTO_SYS:
		break;
	case RSA_SYS:
		break;
	case I2C0_SYS:
		break;
	case I2C1_SYS:
		break;
	case I2C2_SYS:
		break;
	case I2C3_SYS:
		break;
	case ECDSA_SYS:
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
		break;
	case ISP_SYS:
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
		break;
	case I2S0_SYS:
		break;
	case I2S1_SYS:
		break;
	case LXBUS_SYS:
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

	default:
		break;
	}
}

void hal_rtl_sys_set_clk_patch(uint8_t id, uint8_t sel_val)
{
	AON_TypeDef *aon = AON;
	PON_TypeDef *pon = PON;
	SYSON_S_TypeDef *syson_s = SYSON_S;

	volatile uint32_t val;
	switch (id) {
	case UART0_SYS:
		break;
	case PWM_SYS:
		break;
	case TIMER0_SYS:
		break;
	case TIMER1_SYS:
		break;
	case TIMER2_SYS:
		break;
	case TIMER3_SYS:
		break;
	case ENC_SYS:
		break;
	case NN_SYS:
		val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
		val &= ~SYSON_S_MASK_SYS_NN_SRC_SEL;
		val |= sel_val << SYSON_S_SHIFT_SYS_NN_SRC_SEL;
		syson_s->SYSON_S_REG_SYS_NN_CTRL = val;
		break;
	case SPORT_SYS:
		break;
	case ISP_SYS:
		break;
	case MIPI_SYS:
		break;
	case FLASH_SYS:
		break;
	case GPIO_AON:
		break;
	case GPIO_PON:
		break;
	case GPIO_SYS:
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

uint32_t hal_rtl_sys_get_clk_patch(uint8_t id)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	AON_TypeDef *aon = AON;
	PON_TypeDef *pon = PON;
	volatile uint32_t val = 0;

	switch (id) {
	case CPU_SYS:
		break;
	case ENC_SYS:
		break;
	case ISP_SYS:
		break;
	case MIPI_SYS:
		break;
	case NN_SYS:
		val = syson_s->SYSON_S_REG_SYS_NN_CTRL;
		val = (val & SYSON_S_MASK_SYS_NN_SRC_SEL) >> SYSON_S_SHIFT_SYS_NN_SRC_SEL;
		break;
	case FLASH_SYS:
		break;
	case SPORT_SYS:
		break;
	case GPIO_AON:
		break;
	case GPIO_PON:
		break;
	case GPIO_SYS:
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

#endif

/** @} */ /* End of group hal_sys_ctrl */

