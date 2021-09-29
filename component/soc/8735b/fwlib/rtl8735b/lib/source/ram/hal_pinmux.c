/**************************************************************************//**
 * @file     hal_pinmux.c
 * @brief    The implementation of Pin Mux HAL API functions.
 *
 * @version  V1.00
 * @date     2021-07-29
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2020 Realtek Corporation. All rights reserved.
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

#include "hal_pinmux.h"



#if !defined(CONFIG_BUILD_NONSECURE)

extern const hal_pin_manag_func_stubs_t hal_pin_manag_stubs;

#if IS_CUT_TEST(CONFIG_CHIP_VER)
#define DEBUG_PORT_PIN_REGISTER_MAX_NUM             (7)
#ifndef GPIO_CTRL_PINMUX_SEL_OFFSET_H
#define GPIO_CTRL_PINMUX_SEL_OFFSET_H               (16)
#endif

const uint32_t *gpio_a_ram_ctrl_reg_list[] = {
	&(AON->AON_REG_AON_GPIOA_0_1_CTRL),
};
const uint32_t *gpio_f_ram_ctrl_reg_list[] = {
	&(PON->PON_REG_PON_GPIOF_0_1_CTRL),
	&(PON->PON_REG_PON_GPIOF_2_3_CTRL),
	&(PON->PON_REG_PON_GPIOF_4_5_CTRL),
	&(PON->PON_REG_PON_GPIOF_6_7_CTRL),
	&(PON->PON_REG_PON_GPIOF_8_9_CTRL),
	&(PON->PON_REG_PON_GPIOF_10_11_CTRL),
	&(PON->PON_REG_PON_GPIOF_12_13_CTRL),
	&(PON->PON_REG_PON_GPIOF_14_15_CTRL),
	&(PON->PON_REG_PON_GPIOF_16_17_CTRL)
};

const uint32_t _dbg_port_pins_register[DEBUG_PORT_PIN_REGISTER_MAX_NUM] = {
	PIN_A0, PIN_A1, PIN_F6, PIN_F7, PIN_F10, PIN_F11, PIN_F12
};
#endif

hal_status_t hal_pin_register_filter(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_register(pin_name, periphl_id);
}

#if IS_CUT_TEST(CONFIG_CHIP_VER)
hal_status_t hal_load_debuger_pin_register(hal_pin_mux_mang_t *pinmux_manag, uint32_t pin_name)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;
	uint8_t pinmux_sel;

	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		// Internal Port ?
		if (port_idx == PORT_INTERNAL) {
			return HAL_OK;
		} else {
			DBG_MISC_ERR("hal_rtl_pin_register: Unknown port (%u)\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}

	// jtag/swd pins load
	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			pinmux_sel = ((*((uint32_t *)(gpio_a_ram_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
		} else {
			// pin index is even
			pinmux_sel = (*((uint32_t *)(gpio_a_ram_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
		}
		break;
	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			pinmux_sel = ((*((uint32_t *)(gpio_f_ram_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
		} else {
			// pin index is even
			pinmux_sel = (*((uint32_t *)(gpio_f_ram_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
		}
		break;
	default:
		break;
	}
	if ((pinmux_sel != FUNC_GPIO) && (pinmux_sel == FUNC_JTAG)) {
		pinmux_manag->pinmux_reg_log[port_idx] |= (1 << pin_idx);
	}
	return HAL_OK;
}
#endif

void hal_pinmux_rom_info_manage(hal_pin_mux_mang_t *pinmux_manag, uint8_t op, uint8_t obj_id)
{
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	hal_pin_manag_stubs.hal_pinmux_rom_info_manage(pinmux_manag, op, obj_id);
#endif
	return;
}

/**
 *  @brief Initial the pin mux manager.
 *
 *  @param[in]  pinmux_manag  The pin mux manager entity.
 *
 *  @returns    void.
 */
void hal_pinmux_manager_init(hal_pin_mux_mang_t *pinmux_manag)
{
	if (pinmux_manag != NULL) {
		pinmux_manag->pin_reg_func = hal_pin_manag_stubs.hal_pin_register;
		pinmux_manag->pin_unreg_func = hal_pin_manag_stubs.hal_pin_unregister;
		pinmux_manag->ioport_pwrup_ctrl_func = hal_pin_manag_stubs.hal_pin_pwrup;
		pinmux_manag->ioport_pwrdn_ctrl_func = hal_pin_manag_stubs.hal_pin_pwrdwn;
		pinmux_manag->pin_mux_cfg_func = hal_pin_manag_stubs.hal_pin_mux_cfg;
		hal_pinmux_rom_info_manage(pinmux_manag, OP_RESTORE, ALL_LOG);
		hal_pin_manag_stubs.hal_pinmux_manager_init(pinmux_manag);
	}
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	uint32_t i;
	for (i = 0; i < DEBUG_PORT_PIN_REGISTER_MAX_NUM; i++) {
		hal_load_debuger_pin_register(pinmux_manag, _dbg_port_pins_register[i]);
	}
#endif
}


/**
 *  @brief To register a IO pin to the "in using" record.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. At least one of pin in the list
 *              is occupied by other peripheral device.
 */
hal_status_t hal_pin_register(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_register(pin_name, periphl_id);
}


/**
 *  @brief To unregister a IO pin from the "in using" record.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  Pin unregister OK.
 *  @return     HAL_ERR_PARA:  The peripheral ID doesn't match with the ID on the "in using" record.
 */
hal_status_t hal_pin_unregister(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_unregister(pin_name, periphl_id);
}


/**
 *  @brief Configures the pin mux to makes the given pin to connect to the specified peripheral.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. At least one of pin in the list
 *              is occupied by other peripheral device.
 */
hal_status_t hal_pin_mux_cfg(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_mux_cfg(pin_name, periphl_id);
}


/**
 *  @brief Gets current pin configuration for a given pin name.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[out]  pin_cfg  The pin configuration of the given pin name.
 *
 *  @return     HAL_OK:  get pin configuration OK.
 *  @return     HAL_ERR_PARA:  Invalid pin name.
 */
hal_status_t hal_pin_get_cfg(uint32_t pin_name, hal_pin_cfg_t *pin_cfg)
{
	return hal_pin_manag_stubs.hal_pin_get_cfg(pin_name, pin_cfg);
}


/**
 *  @brief To power on (enable) a IO pad .
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  IO pad power up OK.
 */
hal_status_t hal_pin_pwrup(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_pwrup(pin_name, periphl_id);
}


/**
 *  @brief To power down (disable) a IO pad.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  IO pad power down OK.
 */
hal_status_t hal_pin_pwrdwn(uint32_t pin_name, uint32_t periphl_id)
{
	return hal_pin_manag_stubs.hal_pin_pwrdwn(pin_name, periphl_id);
}

#if 0
/**
 *  @brief According to the chip ID to check if a pin is available for this chip package.
 *
 *  @param[in]   pin_name       The pin name to be validated.
 *  @param[in]   periphl_id     The ID of the peripheral for this given pin will be assigned to.
 *
 *  @return      HAL_ERR_UNKNOWN  eFuse read error.
 *  @return      HAL_ERR_HW  The given pin with the peripheral is no available for this chip.
 *  @return      HAL_OK  The given pin is available.
 */

hal_status_t hal_pinmux_validate(uint32_t pin_name, uint32_t periphl_id)
{
	hal_status_t ret = HAL_OK;

	return ret;
}
#endif

#endif  // #if !defined(CONFIG_BUILD_NONSECURE)

/**
 *  @brief To register a IO pin to the pin mux magagement.
 *         The pin mux management will do the checking of pin conflict and pin valid.
 *         And then power up the IO pad.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. This pin is occupied by other peripheral device.
 *  @return     HAL_ERR_HW: This pin is invalid for this IC.
 */
hal_status_t hal_pinmux_register(uint32_t pin_name, uint32_t periphl_id)
{
#if !defined(CONFIG_BUILD_NONSECURE)
	return hal_pin_manag_stubs.hal_pinmux_register(pin_name, periphl_id);
#else
	return hal_pinmux_register_nsc(pin_name, periphl_id);
#endif
}


/**
 *  @brief Unregister a IO pin from the pin mux management.
 *         The pin mux management will power down the IO pad.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  Pin unregister OK.
 *  @return     HAL_ERR_PARA: The peripheral doesn't own this pin.
 */
hal_status_t hal_pinmux_unregister(uint32_t pin_name, uint32_t periphl_id)
{
#if !defined(CONFIG_BUILD_NONSECURE)
	return hal_pin_manag_stubs.hal_pinmux_unregister(pin_name, periphl_id);
#else
	return hal_pinmux_unregister_nsc(pin_name, periphl_id);
#endif
}


