/**************************************************************************//**
 * @file     hal_wdt.c
 * @brief    Implement WDT RAM code functions.
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
#include "hal.h"
extern const hal_wdt_func_stubs_t hal_wdt_stubs;



/**
 *  @brief Changes the watch dog timer timeout period.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
void hal_wdt_set_timeout(uint32_t time_us)
{
	hal_wdt_stubs.wdt_set_timeout(time_us << 1);
}

/**
 *  @brief Initials the watch dog timer and setup the timeout period.
 *         The system will be reset by the watch dog timeout event by default.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */

void hal_wdt_init(uint32_t time_us)
{
	hal_wdt_stubs.wdt_init(time_us << 1);
}

/**
 *  @brief Enables the watch dog timer.
 *
 *  @returns    void
 */

void hal_wdt_enable(void)
{
	hal_wdt_stubs.wdt_enable();
}

/**
 *  @brief Disables the watch dog timer.
 *
 *  @returns    void
 */

void hal_wdt_disable(void)
{
	hal_wdt_stubs.wdt_disable();
}

/**
 *  @brief Refresh(reload) the watch dog timer counter.
 *         To prevents the watch dog timer timeout event occurred.
 *
 *  @returns    void
 */

void hal_wdt_refresh(void)
{
	hal_wdt_stubs.wdt_refresh();
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

void hal_wdt_reg_irq(irq_handler_t handler, void *arg)
{
	hal_wdt_stubs.wdt_reg_irq(handler, &arg);
}

/**
 *  @brief Un-Registers a handler for the watch dog timeout interrupt.
 *
 *
 *  @returns    void
 */

void hal_wdt_unreg_irq(void)
{
	hal_wdt_stubs.wdt_unreg_irq();
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

void hal_wdt_reset(u8 reset)
{
	hal_wdt_stubs.wdt_reset(reset);
}

void hal_wdt_aon_enable()
{
	hal_wdt_stubs.wdt_ctrl_aon_enable();
}

void hal_wdt_aon_disable()
{
	hal_wdt_stubs.wdt_ctrl_aon_disable();
}

void hal_wdt_all_enable()
{
	hal_wdt_stubs.wdt_ctrl_all_enable();
}

void hal_wdt_all_disable()
{
	hal_wdt_stubs.wdt_ctrl_all_disable();
}

/** *@} */ /* End of group hal_wdt_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** *@} */
