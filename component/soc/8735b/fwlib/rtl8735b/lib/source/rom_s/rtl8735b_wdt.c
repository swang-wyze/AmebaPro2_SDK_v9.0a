/**************************************************************************//**
 * @file     rtl8735b_wdt.c
 * @brief    Implement flash controller ROM code functions.
 * @version  1.00
 * @date     2021-01-04
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
#include "rtl8735b_wdt.h"

#define SECTION_WDT_TEXT           SECTION(".rom.hal_wdt.text")
#define SECTION_WDT_DATA           SECTION(".rom.hal_wdt.data")
#define SECTION_WDT_RODATA         SECTION(".rom.hal_wdt.rodata")
#define SECTION_WDT_BSS            SECTION(".rom.hal_wdt.bss")
#define SECTION_WDT_STUBS          SECTION(".rom.hal_wdt.stubs")

SECTION_WDT_BSS hal_wdt_adapter_t pwdt_adapter;

extern void *_memset(void *dst0, int Val, SIZE_T length);

/**

        \addtogroup hal_spic Flash Controller
        @{
*/


/**
  \brief The golden pattern to calibrate flash controller data path delay.
*/


/// @cond DOXYGEN_ROM_HAL_API

/**

        \addtogroup hal_wdt_rom_func Flash Controller HAL ROM APIs
        \ingroup hal_wdt
        \brief Configure WDT functions to check CPU status.
               The user application(in RAM space) should not call these APIs directly.
        @{
*/

/**
  \brief The stubs functions table to exports flash controller HAL functions in ROM.
*/
SECTION_WDT_STUBS const hal_wdt_func_stubs_t hal_wdt_stubs = {
	.wdt_set_timeout = hal_rtl_wdt_set_timeout,
	.wdt_init = hal_rtl_wdt_init,
	.wdt_enable = hal_rtl_wdt_enable,
	.wdt_disable = hal_rtl_wdt_disable,
	.wdt_refresh = hal_rtl_wdt_refresh,
	.wdt_reg_irq = hal_rtl_wdt_reg_irq,
	.wdt_unreg_irq = hal_rtl_wdt_unreg_irq,
	.wdt_reset = hal_rtl_wdt_reset,
	.wdt_ctrl_aon_enable = hal_rtl_wdt_ctrl_aon_enable,
	.wdt_ctrl_aon_disable = hal_rtl_wdt_ctrl_aon_disable,
	.wdt_ctrl_all_enable = hal_rtl_wdt_ctrl_all_enable,
	.wdt_ctrl_all_disable = hal_rtl_wdt_ctrl_all_disable,
};


/**
 *  @brief Changes the watch dog timer timeout period.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_set_timeout(uint32_t time_us)
{
	VNDR_S_TypeDef *vendor_s = VNDR_S;
	uint32_t div = 1;
	uint32_t ticks;
	uint32_t cnt_limit;
	uint32_t value_found;
	uint32_t div_max;

	value_found = 0;
	for (cnt_limit = 1; cnt_limit < 12; cnt_limit++) {
		ticks = 2 << cnt_limit;
		// let max divider=3200 -> min resolution = 100 ms
		// CLK=32000HZ & divider=3200 -> 1 tick of count = 100000us
		if ((ticks * 100000) < time_us) {
			continue;
		}

		for (div_max = 1; div_max <= 3200; div_max = (div_max << 1)) {
			if (((div_max * ticks) << 5) > time_us) {
				break;
			}
		}

		for (div = (div_max >> 1); div <= 3200; div++) {
			// << 5 = x32, 1 ticks ~ 32us (1/32K)
			if (((div * ticks) << 5) > time_us) {
				value_found = 1;
				break;
			}
		}

		if (value_found) {
			div = div + (div / 42);
			break;
		}
	}

	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER &= ~(VNDR_S_MASK_CNT_LIMIT);
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER &= ~(VNDR_S_MASK_VNDR_DIVFACTOR);

	if (value_found) {
		vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= (div - 1);
		vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= (cnt_limit << VNDR_S_SHIFT_CNT_LIMIT);
	} else {
		vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= 32000;
		vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= (11 << VNDR_S_SHIFT_CNT_LIMIT);
	}
}

/**
 *  @brief Initials the watch dog timer and setup the timeout period.
 *         The system will be reset by the watch dog timeout event by default.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_init(uint32_t time_us)
{
	VNDR_S_TypeDef *vendor_s = VNDR_S;
	hal_rtl_wdt_set_timeout(time_us);
	// default reset the system when WDT timeout
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= VNDR_S_BIT_WDT_MODE;
}

/**
 *  @brief Enables the watch dog timer.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_enable(void)
{
	VNDR_S_TypeDef *vendor_s = VNDR_S;
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= VNDR_S_BIT_WDT_EN_BYTE;
}

/**
 *  @brief Disables the watch dog timer.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_disable(void)
{
	VNDR_S_TypeDef *vendor_s = VNDR_S;
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER &= ~(VNDR_S_BIT_WDT_EN_BYTE);
}

/**
 *  @brief Refresh(reload) the watch dog timer counter.
 *         To prevents the watch dog timer timeout event occurred.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_refresh(void)
{
	VNDR_S_TypeDef *vendor_s = VNDR_S;
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER |= VNDR_S_BIT_WDT_CLEAR;
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER &= ~(VNDR_S_BIT_WDT_CLEAR);
}

/**
 *  @brief Registers a handler for the watch dog timeout interrupt.
 *         The WDT timeout interrupt will trigger the NMI interrupt.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_reg_irq(irq_handler_t handler, void *arg)
{
	_memset(&pwdt_adapter, 0, sizeof(hal_wdt_adapter_t));
	VNDR_S_TypeDef *vendor_s = VNDR_S;
#if 0
	if (pwdt_adapter == NULL) {
		DBG_MISC_ERR("Misc. Adp is NULL\r\n");
		return;
	}
#endif
	pwdt_adapter.wdt_arg = arg;
	pwdt_adapter.wdt_handler = handler;
	vendor_s->VNDR_S_REG_SECURE_WATCH_DOG_TIMER &= ~(VNDR_S_BIT_WDT_MODE);
}

/**
 *  @brief Un-Registers a handler for the watch dog timeout interrupt.
 *
 *
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_unreg_irq(void)
{
	_memset(&pwdt_adapter, 0, sizeof(hal_wdt_adapter_t));
#if 0
	if (pwdt_adapter == NULL) {
		DBG_MISC_ERR("Misc. Adp is NULL\r\n");
		return;
	}
#endif
	pwdt_adapter.wdt_arg = NULL;
	pwdt_adapter.wdt_handler = NULL;
}

/**
 *  @brief Refresh(reload) the watch dog timer counter.
 *         To prevents the watch dog timer timeout event occurred.
 *
 *  @param[in]  reset :
 *                - BIT 0: 1: Mask WDOG_RST to reset AON  block.
 *                - BIT 1: 1: Mask WDOG_RST to reset PON  block
 *                - BIT 2: 1: Mask WDOG_RST to reset WLON  block
 *                - BIT 3: 0: Mask WDOG_RST to trigger CPU wram reset
 *                            1: Mask WDOG_RST to trigger CPU cold reset
 *                - BIT 4: 1: Mask WDOG_RST to reset SYSON  block
 *  @returns    void
 */
SECTION_WDT_TEXT
void hal_rtl_wdt_reset(u8 reset)
{
	PON_TypeDef *pon = PON;
	pon->PON_REG_WDT_RST_CTRL = reset;
	//HAL_WRITE32(0x40009800, 0x38, reset);
}

SECTION_WDT_TEXT
void hal_rtl_wdt_ctrl_aon_enable(void)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;

	// Enable AON WDT
	reg_val = aon->AON_REG_AON_WDT_TIMER;
	reg_val |= (AON_BIT_WDT_EN_BYTE);
	aon->AON_REG_AON_WDT_TIMER = reg_val;
}

SECTION_WDT_TEXT
void hal_rtl_wdt_ctrl_aon_disable(void)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;

	// Disable AON WDT
	reg_val = aon->AON_REG_AON_WDT_TIMER;
	reg_val &= (~AON_BIT_WDT_EN_BYTE);
	aon->AON_REG_AON_WDT_TIMER = reg_val;
}

SECTION_WDT_TEXT
void hal_rtl_wdt_ctrl_all_enable(void)
{
	// AON WDT Enable
	hal_rtl_wdt_ctrl_aon_enable();

	// VNDR WDT Enable
	hal_rtl_wdt_enable();
}

SECTION_WDT_TEXT
void hal_rtl_wdt_ctrl_all_disable(void)
{
	// AON WDT Disable
	hal_rtl_wdt_ctrl_aon_disable();

	// VNDR WDT Disable
	hal_rtl_wdt_disable();
}
/** *@} */ /* End of group hal_wdt_rom_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** *@} */
