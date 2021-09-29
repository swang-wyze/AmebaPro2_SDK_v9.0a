/**************************************************************************//**
* @file        hal_gpio_nsc.c
* @brief       This file implements the entry functions of the GPIO Non-Secure Callable HAL functions.
*
* @version     V1.00
* @date        2020-11-12
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
#include "hal_gpio.h"
#include "hal_gpio_nsc.h"

#include <arm_cmse.h>   /* Use CMSE intrinsics */ // weide temp

#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#undef SECTION_NS_ENTRY_FUNC
#undef NS_ENTRY
#define SECTION_NS_ENTRY_FUNC
#define NS_ENTRY
#endif

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_comm_init_nsc(phal_gpio_comm_adapter_t pgpio_comm_adp)
{

	hal_gpio_comm_init(pgpio_comm_adp);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_aon_gpio_comm_init_nsc(phal_aon_gpio_comm_adapter_t paon_gpio_comm_adp)
{

	hal_aon_gpio_comm_init(paon_gpio_comm_adp);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_pon_gpio_comm_init_nsc(phal_pon_gpio_comm_adapter_t ppon_gpio_comm_adp)
{

	hal_pon_gpio_comm_init(ppon_gpio_comm_adp);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_comm_deinit_nsc(void)
{

	hal_gpio_comm_deinit();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_aon_gpio_comm_deinit_nsc(void)
{

	hal_aon_gpio_comm_deinit();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_pon_gpio_comm_deinit_nsc(void)
{

	hal_pon_gpio_comm_deinit();
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_init_nsc(phal_gpio_adapter_t pgpio_adapter, uint32_t pin_name)
{

	return hal_gpio_init(pgpio_adapter, pin_name);

}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_deinit_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	hal_gpio_deinit(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_irq_init_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t pin_name,
		gpio_irq_callback_t callback, uint32_t arg)
{

	return hal_gpio_irq_init(pgpio_irq_adapter, pin_name, callback, arg);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_irq_deinit_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	hal_gpio_irq_deinit(pgpio_irq_adapter);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_port_init_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t port_idx,
		uint32_t mask, gpio_dir_t dir)
{

	return hal_gpio_port_init(pgpio_port_adapter, port_idx, mask, dir);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_port_deinit_nsc(phal_gpio_port_adapter_t pgpio_port_adapter)
{

	hal_gpio_port_deinit(pgpio_port_adapter);
}

SECTION_NS_ENTRY_FUNC
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
void NS_ENTRY hal_gpio_port_dir_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, gpio_dir_t dir)
{

	hal_gpio_port_dir(pgpio_port_adapter, dir);
}
#else // Test chip
void NS_ENTRY hal_gpio_port_dir_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, gpio_dir_t dir)
{

	hal_gpio_port_dir(pgpio_port_adapter, mask, dir);
}

#endif

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_reg_irq_nsc(gpio_type_t gpio_type, irq_handler_t irq_handler)
{

	hal_gpio_reg_irq(gpio_type, irq_handler);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_interrupt_clk_sel_nsc(gpio_type_t gpio_type, uint8_t clk_sel)
{

	return hal_gpio_interrupt_clk_sel(gpio_type, clk_sel);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_enter_critical_nsc(void)
{

	hal_gpio_enter_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_aon_gpio_enter_critical_nsc(void)
{

	hal_aon_gpio_enter_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_pon_gpio_enter_critical_nsc(void)
{

	hal_pon_gpio_enter_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_exit_critical_nsc(void)
{

	hal_gpio_exit_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_aon_gpio_exit_critical_nsc(void)
{

	hal_aon_gpio_exit_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_pon_gpio_exit_critical_nsc(void)
{

	hal_pon_gpio_exit_critical();
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_set_dir_nsc(phal_gpio_adapter_t pgpio_adapter, gpio_dir_t dir)
{

	hal_gpio_set_dir(pgpio_adapter, dir);
}

SECTION_NS_ENTRY_FUNC
gpio_dir_t NS_ENTRY hal_gpio_get_dir_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	return hal_gpio_get_dir(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_write_nsc(phal_gpio_adapter_t pgpio_adapter, uint32_t io_data)
{

	hal_gpio_write(pgpio_adapter, io_data);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_toggle_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	hal_gpio_toggle(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_gpio_read_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	return hal_gpio_read(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_debounce_enable_nsc(phal_gpio_adapter_t pgpio_adapter, uint32_t debounce_us)
{

	return hal_gpio_debounce_enable(pgpio_adapter, debounce_us);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_gpio_read_debounce_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	return hal_gpio_read_debounce(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_debounce_disable_nsc(phal_gpio_adapter_t pgpio_adapter)
{

	hal_gpio_debounce_disable(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_irq_set_trig_type_nsc(phal_gpio_irq_adapter_t pgpio_adapter, gpio_int_trig_type_t int_type)
{

	hal_gpio_irq_set_trig_type(pgpio_adapter, int_type);
}

SECTION_NS_ENTRY_FUNC
gpio_int_trig_type_t NS_ENTRY hal_gpio_irq_get_trig_type_nsc(phal_gpio_irq_adapter_t pgpio_adapter)
{

	return hal_gpio_irq_get_trig_type(pgpio_adapter);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_irq_enable_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	hal_gpio_irq_enable(pgpio_irq_adapter);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_irq_disable_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	hal_gpio_irq_disable(pgpio_irq_adapter);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_irq_debounce_enable_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter,
		uint32_t debounce_us)
{

	return hal_gpio_irq_debounce_enable(pgpio_irq_adapter, debounce_us);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_gpio_irq_debounce_disable_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	hal_gpio_irq_debounce_disable(pgpio_irq_adapter);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_gpio_irq_read_nsc(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	return hal_gpio_irq_read(pgpio_irq_adapter);
}

SECTION_NS_ENTRY_FUNC
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
void NS_ENTRY hal_gpio_port_write_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t io_data)
{

	hal_gpio_port_write(pgpio_port_adapter, io_data);
}
#else // Test chip
void NS_ENTRY hal_gpio_port_write_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, uint32_t io_data)
{

	hal_gpio_port_write(pgpio_port_adapter, mask, io_data);
}

#endif

SECTION_NS_ENTRY_FUNC
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
uint32_t NS_ENTRY hal_gpio_port_read_nsc(phal_gpio_port_adapter_t pgpio_port_adapter)
{

	return hal_gpio_port_read(pgpio_port_adapter);
}
#else // Test chip
uint32_t NS_ENTRY hal_gpio_port_read_nsc(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask)
{

	return hal_gpio_port_read(pgpio_port_adapter, mask);
}
#endif

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_pull_ctrl_nsc(uint32_t pin_name, pin_pull_type_t pull_type)
{

	return hal_gpio_pull_ctrl(pin_name, pull_type);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_schmitt_ctrl_nsc(uint32_t pin_name, BOOLEAN ctrl)
{

	return hal_gpio_schmitt_ctrl(pin_name, ctrl);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_drive_ctrl_nsc(uint32_t pin_name, uint8_t drv)
{

	return hal_gpio_drive_ctrl(pin_name, drv);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_gpio_slew_rate_ctrl_nsc(uint32_t pin_name, uint8_t slew_rate_func)
{

	return hal_gpio_slew_rate_ctrl(pin_name, slew_rate_func);
}

// Newly added functions will come here to facilitate debugging - added from 30JUL2021 onwards， only for MP A-cut chip and beyond

SECTION_NS_ENTRY_FUNC
void aon_gpio_sclk_source_select_nsc(uint32_t sclk_src_sel)
{

	aon_gpio_sclk_source_select(sclk_src_sel);
}