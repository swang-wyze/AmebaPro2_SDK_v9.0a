/**************************************************************************//**
 * @file     rtl8735b_pinmux.c
 * @brief    Implement function to detect pin-mux conflict.
 * @version  V1.00
 * @date     2021-07-17
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

#include "cmsis.h"
#include "rtl8735b.h"



#define SECTION_PINMUX_TEXT           SECTION(".rom.hal_pinmux.text")
#define SECTION_PINMUX_DATA           SECTION(".rom.hal_pinmux.data")
#define SECTION_PINMUX_RODATA         SECTION(".rom.hal_pinmux.rodata")
#define SECTION_PINMUX_BSS            SECTION(".rom.hal_pinmux.bss")
#define SECTION_PINMUX_STUBS          SECTION(".rom.hal_pinmux.stubs")


#define GPIO_CTRL_PINMUX_SEL_OFFSET_H       16
#define GPIO_CTRL_PINMUX_SEL_OFFSET_L       0




/**
  * @brief The table of GPIO A control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_a_ctrl_reg_list[] = {
	&(AON->AON_REG_AON_GPIOA_0_1_CTRL),
	&(AON->AON_REG_AON_GPIOA_2_3_CTRL),
	&(AON->AON_REG_AON_GPIOA_4_5_CTRL)
};

/**
  * @brief The table of GPIO B control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_b_ctrl_reg_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOB_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOB_2_CTRL)
};

/**
  * @brief The table of GPIO C control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_c_ctrl_reg_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOC_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOC_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOC_4_5_CTRL)
};

/**
  * @brief The table of GPIO D control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_d_ctrl_reg_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOD_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_6_7_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_8_9_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_10_11_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_12_13_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_14_15_CTRL),
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	&(SYSON->SYSON_REG_SYSON_GPIOD_16_CTRL)
#else
	&(SYSON->SYSON_REG_SYSON_GPIOD_16_17_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_18_19_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_20_CTRL)
#endif
};

/**
  * @brief The table of GPIO E control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_e_ctrl_reg_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOE_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_4_5_CTRL),
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	&(SYSON->SYSON_REG_SYSON_GPIOE_6_7_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_8_9_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_10_CTRL)
#else
	&(SYSON->SYSON_REG_SYSON_GPIOE_6_CTRL)
#endif
};

/**
  * @brief The table of GPIO F control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_f_ctrl_reg_list[] = {
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

/**
  * @brief The table of GPIO S control registers.
  */
SECTION_PINMUX_RODATA
const __IOM uint32_t *gpio_s_ctrl_reg_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOS_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_6_CTRL)
};

/**
  * @brief The string table of IO function name.
  */
SECTION_PINMUX_RODATA
const char *io_func_name_string[] = {
	"Primary",
	"DMIC/I2C",
	"PWM",
	"RFAFE_CTRL/JTAG",
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	"TPIU/BT_AOX",
#else
	"SDIO Host/BT_AOX",
#endif
	"SPI Master/I2S",
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	"SDIO Host/UART",
#else
	"TPIU/UART",
#endif
	"BT",
	"SPI Slave",
	"External BT",
	"Undefined Func Group",
	"Undefined Func Group",
	"Undefined Func Group",
	"Undefined Func Group",
	"Undefined Func Group",
	"GPIO"
};

/**
  * @brief The table of pinmux selection.
  */
SECTION_PINMUX_RODATA
const __IOM hal_pinmux_sel_t pinmux_sel_tbl[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_A0,  PIN_B0,  PIN_LIST_END},  // I2C0_SCL
	{PIN_A1,  PIN_B1,  PIN_LIST_END},  // I2C0_SDA
	{PIN_D15, PIN_F2,  PIN_LIST_END},  // I2C1_SCL
	{PIN_D16, PIN_F3,  PIN_LIST_END},  // I2C1_SDA
	{PIN_E7,  PIN_E9,  PIN_LIST_END},  // I2C2_SCL
	{PIN_E8,  PIN_E10, PIN_LIST_END},  // I2C2_SDA
	{PIN_E0,  PIN_E2,  PIN_LIST_END},  // DMIC_CLK
	{PIN_F1,  PIN_F14, PIN_LIST_END},  // SGPIO_RX
	{PIN_D14, PIN_E3,  PIN_LIST_END},  // WIFI_LED
	{PIN_F14, PIN_S1,  PIN_LIST_END},  // PWM8
	{PIN_F15, PIN_S4,  PIN_LIST_END},  // PWM9
	{PIN_F16, PIN_S5,  PIN_LIST_END},  // PWM10
	{PIN_F17, PIN_S6,  PIN_LIST_END},  // PWM11
	{PIN_A1,  PIN_F10, PIN_LIST_END},  // JTAG_CLK/SWD_CK
	{PIN_A0,  PIN_F11, PIN_LIST_END},  // JTAG_TMS/SWD_IO
	{PIN_D15, PIN_E4,  PIN_LIST_END},  // RFE_CTRL_3
	{PIN_D16, PIN_E7,  PIN_LIST_END},  // RFE_CTRL_4
	{PIN_E3,  PIN_E8,  PIN_LIST_END},  // RFE_CTRL_5
	{PIN_E1,  PIN_E5,  PIN_LIST_END},  // BT_AOX_0
	{PIN_E2,  PIN_E6,  PIN_LIST_END},  // BT_AOX_1
	{PIN_E3,  PIN_E7,  PIN_LIST_END},  // BT_AOX_2
	{PIN_E4,  PIN_E8,  PIN_LIST_END},  // BT_AOX_3
	{PIN_F4,  PIN_F12, PIN_LIST_END},  // UART1_IN
	{PIN_F5,  PIN_F13, PIN_LIST_END},  // UART1_OUT
	{PIN_D16, PIN_E2,  PIN_LIST_END},  // UART2_IN
	{PIN_D15, PIN_E1,  PIN_LIST_END},  // UART2_OUT
	{PIN_D13, PIN_E0,  PIN_E6},        // BT_UART_IN
	{PIN_D11, PIN_E1,  PIN_E5}         // BT_UART_OUT
#else
	{PIN_A0,  PIN_B0,  PIN_LIST_END},  // I2C0_SCL
	{PIN_A1,  PIN_B1,  PIN_LIST_END},  // I2C0_SDA
	{PIN_D19, PIN_F1,  PIN_LIST_END},  // I2C1_SCL
	{PIN_D20, PIN_F2,  PIN_LIST_END},  // I2C1_SDA
	{PIN_E3,  PIN_E5,  PIN_LIST_END},  // I2C2_SCL
	{PIN_E4,  PIN_E6,  PIN_LIST_END},  // I2C2_SDA
	{PIN_D14, PIN_D16, PIN_LIST_END},  // DMIC_CLK
	{PIN_F9,  PIN_F14, PIN_LIST_END},  // SGPIO_RX
	{PIN_D14, PIN_E0,  PIN_LIST_END},  // WIFI_LED
	{PIN_F14, PIN_S1,  PIN_LIST_END},  // PWM8
	{PIN_F15, PIN_S4,  PIN_LIST_END},  // PWM9
	{PIN_F16, PIN_S5,  PIN_LIST_END},  // PWM10
	{PIN_F17, PIN_S6,  PIN_LIST_END},  // PWM11
	{PIN_A1,  PIN_F9,  PIN_LIST_END},  // JTAG_CLK/SWD_CK
	{PIN_A0,  PIN_F10, PIN_LIST_END},  // JTAG_TMS/SWD_IO
	{PIN_D19, PIN_E0,  PIN_LIST_END},  // RFE_CTRL_3
	{PIN_D17, PIN_E3,  PIN_LIST_END},  // RFE_CTRL_4
	{PIN_D18, PIN_E4,  PIN_LIST_END},  // RFE_CTRL_5
	{PIN_D15, PIN_E1,  PIN_LIST_END},  // BT_AOX_0
	{PIN_D16, PIN_E2,  PIN_LIST_END},  // BT_AOX_1
	{PIN_D17, PIN_E3,  PIN_LIST_END},  // BT_AOX_2
	{PIN_D18, PIN_E4,  PIN_LIST_END},  // BT_AOX_3
	{PIN_F3,  PIN_F12, PIN_LIST_END},  // UART1_IN
	{PIN_F4,  PIN_F13, PIN_LIST_END},  // UART1_OUT
	{PIN_D16, PIN_D20, PIN_LIST_END},  // UART2_IN
	{PIN_D15, PIN_D19, PIN_E0},        // UART2_OUT
	{PIN_D12, PIN_D14, PIN_E2},        // BT_UART_IN
	{PIN_D10, PIN_D15, PIN_E1},        // BT_UART_OUT
	{PIN_D13, PIN_E0,  PIN_LIST_END}   // BT_LOG
#endif
};


/**
  * @brief Point to the global pin mux management adapter.
  */
SECTION_PINMUX_BSS hal_pin_mux_mang_t *ppinmux_manager;
SECTION_PINMUX_BSS uint32_t pinmux_reg_rec[PORT_MAX_NUM][32];

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
SECTION_PINMUX_BSS uint32_t pinmux_pwr_log[PORT_MAX_NUM];
SECTION_PINMUX_BSS uint32_t pinmux_reg_log[PORT_MAX_NUM];
#endif

/**
  * @brief The stubs functions table to exports pin mux management ROM HAL functions.
  */
SECTION_PINMUX_STUBS const hal_pin_manag_func_stubs_t hal_pin_manag_stubs = {
	.pppin_manager = &ppinmux_manager,
	.hal_pinmux_manager_init = hal_rtl_pinmux_manager_init,
	.hal_pin_register = hal_rtl_pin_register,
	.hal_pin_unregister = hal_rtl_pin_unregister,
	.hal_pin_mux_cfg = hal_rtl_pin_mux_cfg,
	.hal_pin_get_cfg = hal_rtl_pin_get_cfg,
	.hal_pin_pwrup = hal_rtl_pin_pwrup,
	.hal_pin_pwrdwn = hal_rtl_pin_pwrdwn,
	.hal_pinmux_register = hal_rtl_pinmux_register,
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	.hal_pinmux_unregister = hal_rtl_pinmux_unregister
#else
	.hal_pinmux_unregister = hal_rtl_pinmux_unregister,
	.hal_pinmux_rom_info_manage = hal_rtl_pinmux_rom_info_manage,
#endif
};

/**
 *  @brief To initial the pin mux manager.
 *
 *  @param[in] pinmux_manag The pin mux manager entity.
 *
 *  @returns void
 */
SECTION_PINMUX_TEXT
void hal_rtl_pinmux_manager_init(hal_pin_mux_mang_t *pinmux_manag)
{
	AON_TypeDef *aon = AON;


	pinmux_manag->ppinmux_reg_rec = &pinmux_reg_rec[0][0];
	ppinmux_manager = pinmux_manag;
	if (((aon->AON_REG_AON_PAD_CTRL) & AON_BIT_AON_GPIOA_PWD33) == 0) {
		ppinmux_manager->pinmux_pwr_log[PORT_A] = (1 << 5) | (1 << 4);
	}
}


SECTION_PINMUX_TEXT
uint8_t _hal_rtl_pin_reg_chk(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;


	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (ppinmux_manager->pinmux_reg_log[port_idx] & (1 << pin_idx)) {
		if ((ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx]) == periphl_id) {
			DBG_MISC_ERR("Peripheral[%08X] is already used by Pin%d[%d]\r\n", periphl_id, port_idx, pin_idx);
			return 1;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}


/**
 *  @brief To check the pinmux selection conflict by peripheral ID.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin selection OK.
 *  @return     HAL_ERR_CONFLICT:  Pin selection conflict. The specified pin/peripheral ID
 *              is occupied by other pin.
 */
SECTION_PINMUX_TEXT
hal_status_t _hal_rtl_pinmux_sel_chk(uint32_t pin_name, uint32_t periphl_id)
{
	uint8_t is_conflict = 0;
	uint8_t chk1 = 0;
	uint8_t chk2 = 0;


	switch (periphl_id) {
	case PID_I2C0:
		if (pin_name == pinmux_sel_tbl[0].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[0].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[0].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[0].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[1].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[1].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[1].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[1].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_I2C1:
		if (pin_name == pinmux_sel_tbl[2].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[2].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[2].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[2].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[3].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[3].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[3].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[3].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_I2C2:
		if (pin_name == pinmux_sel_tbl[4].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[4].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[4].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[4].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[5].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[5].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[5].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[5].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_DMIC:
		if (pin_name == pinmux_sel_tbl[6].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[6].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[6].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[6].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_SGPIO:
		if (pin_name == pinmux_sel_tbl[7].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[7].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[7].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[7].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_WLAN_LED:
		if (pin_name == pinmux_sel_tbl[8].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[8].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[8].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[8].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_PWM8:
		if (pin_name == pinmux_sel_tbl[9].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[9].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[9].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[9].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_PWM9:
		if (pin_name == pinmux_sel_tbl[10].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[10].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[10].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[10].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_PWM10:
		if (pin_name == pinmux_sel_tbl[11].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[11].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[11].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[11].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_PWM11:
		if (pin_name == pinmux_sel_tbl[12].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[12].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[12].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[12].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_JTAG:
		if (pin_name == pinmux_sel_tbl[13].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[13].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[13].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[13].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[14].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[14].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[14].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[14].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_RFE_CTRL:
		if (pin_name == pinmux_sel_tbl[15].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[15].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[15].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[15].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[16].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[16].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[16].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[16].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[17].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[17].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[17].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[17].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_BT_AOX:
		if (pin_name == pinmux_sel_tbl[18].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[18].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[18].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[18].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[19].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[19].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[19].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[19].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[20].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[20].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[20].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[20].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[21].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[21].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[21].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[21].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_UART1:
		if (pin_name == pinmux_sel_tbl[22].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[22].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[22].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[22].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[23].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[23].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[23].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[23].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
	case PID_UART2:
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if (pin_name == pinmux_sel_tbl[24].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[24].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[24].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[24].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[25].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[25].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
#else
		if (pin_name == pinmux_sel_tbl[24].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[24].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[24].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[24].pin_sel_0, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[25].pin_sel_0) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_1, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[25].pin_sel_1) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[25].pin_sel_2) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[25].pin_sel_1, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else {
			is_conflict = 0;
		}
#endif
		break;
	case PID_BT_UART:
		if (pin_name == pinmux_sel_tbl[26].pin_sel_0) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_1, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[26].pin_sel_1) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[26].pin_sel_2) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[26].pin_sel_1, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[27].pin_sel_0) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_1, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[27].pin_sel_1) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_2, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else if (pin_name == pinmux_sel_tbl[27].pin_sel_2) {
			chk1 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_0, periphl_id);
			chk2 = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[27].pin_sel_1, periphl_id);
			if ((!chk1) && (!chk2)) {
				is_conflict = 0;
			} else {
				is_conflict = 1;
			}
		} else {
			is_conflict = 0;
		}
		break;
#if !IS_CUT_TEST(CONFIG_CHIP_VER)
	case PID_BT_LOG:
		if (pin_name == pinmux_sel_tbl[28].pin_sel_0) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[28].pin_sel_1, periphl_id);
		} else if (pin_name == pinmux_sel_tbl[28].pin_sel_1) {
			is_conflict = _hal_rtl_pin_reg_chk(pinmux_sel_tbl[28].pin_sel_0, periphl_id);
		} else {
			is_conflict = 0;
		}
		break;
#endif
	default:
		is_conflict = 0;
		break;
	}

	if (is_conflict) {
		return HAL_ERR_CONFLICT;
	} else {
		return HAL_OK;
	}
}


/**
 *  @brief To register a IO pin to the "in using" record.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. The specified pin
 *              is occupied by other peripheral device.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_register(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;
	uint8_t pinmux_sel;


	DBG_MISC_INFO("\r\n%s => pin_name = 0x%x, pid = 0x%x\r\n", __func__, pin_name, periphl_id);
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

	// pinmux selection conflict
	if (_hal_rtl_pinmux_sel_chk(pin_name, periphl_id) != HAL_OK) {
		return HAL_ERR_CONFLICT;
	}

	// pin confict
	if (ppinmux_manager->pinmux_reg_log[port_idx] & (1 << pin_idx)) {
		DBG_MISC_ERR("Pin %u[%u] is conflicted\r\n", port_idx, pin_idx);

		if (port_idx == PORT_DDR) {
			DBG_MISC_ERR("It's configured as DDR now.\r\n");
		} else {
			switch (port_idx) {
			case PORT_A:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_B:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_C:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_D:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_E:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_F:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			case PORT_S:
				if (pin_idx % 2) {
					// pin index is odd
					pinmux_sel = ((*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)]))) >> GPIO_CTRL_PINMUX_SEL_OFFSET_H) & 0xF;  // bit[19:16]
				} else {
					// pin index is even
					pinmux_sel = (*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)]))) & 0xF;  // bit[3:0]
				}
				break;
			default:
				break;
			}
			DBG_MISC_ERR("It's configured as %s now.\r\n", io_func_name_string[pinmux_sel]);
		}
		if (ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx]) {
			DBG_MISC_ERR("It's using by peripheral %08X\r\n", ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx]);
		}

		return HAL_ERR_CONFLICT;
	}

	// no pin conflict, mark pin using bits
	ppinmux_manager->pinmux_reg_log[port_idx] |= (1 << pin_idx);
	ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx] = periphl_id;


	return HAL_OK;
}


/**
 *  @brief To unregister a IO pin from the "in using" record.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral own this pin.
 *
 *  @return     HAL_OK:  Pin unregister OK.
 *  @return     HAL_ERR_PARA:  The peripheral ID doesn't match with the ID on the "in using" record.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_unregister(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;


	DBG_MISC_INFO("\r\n%s => pin_name = 0x%x, pid = 0x%x\r\n", __func__, pin_name, periphl_id);
	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		// Internal Port ?
		if (port_idx == PORT_INTERNAL) {
			return HAL_OK;
		} else {
			DBG_MISC_ERR("hal_rtl_pin_unregister: Unknown port (%u)\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}

	if (ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx]) {
		if (periphl_id == (ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx])) {
			ppinmux_manager->pinmux_reg_log[port_idx] &= ~(1 << pin_idx);
			ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx] = 0;
		} else {
			DBG_MISC_ERR("Pin un-register caller (%08X) is not the owner (%08X)\r\n", periphl_id, ppinmux_manager->ppinmux_reg_rec[(port_idx << 5) + pin_idx]);
			return HAL_ERR_PARA;
		}
	} else {
		ppinmux_manager->pinmux_reg_log[port_idx] &= ~(1 << pin_idx);
	}


	return HAL_OK;
}


/**
 *  @brief Configures the pin mux to makes the given pin to connect to the specified peripheral.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use these pins.
 *
 *  @return     HAL_OK:  Pin register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. The specified pin
 *              is occupied by other peripheral device.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_mux_cfg(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;
	uint8_t pin_func;


	DBG_MISC_INFO("\r\n%s => pin_name = 0x%x, pid = 0x%x\r\n", __func__, pin_name, periphl_id);
	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		// Internal Port ?
		if (port_idx == PORT_INTERNAL) {
			return HAL_OK;
		} else {
			DBG_MISC_ERR("hal_rtl_pin_mux_cfg: Unknown port (%u)\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	} else if (port_idx == PORT_DDR) {
		return HAL_OK;
	}

	pin_func = (periphl_id >> 28) & 0xF;

	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_H));  // clear bit[19:16]
			*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_H); // set bit[19:16]
		} else {
			// pin index is even
			*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)])) &= (~(0xF << GPIO_CTRL_PINMUX_SEL_OFFSET_L));  // clear bit[3:0]
			*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)])) |= (pin_func << GPIO_CTRL_PINMUX_SEL_OFFSET_L); // set bit[3:0]
		}
		break;
	default:
		break;
	}


	return HAL_OK;
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
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_get_cfg(uint32_t pin_name, hal_pin_cfg_t *pin_cfg)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;


	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		DBG_MISC_ERR("hal_rtl_pin_get_cfg: Unknown port (%u)\r\n", port_idx);
		return HAL_ERR_PARA;
	} else if (port_idx == PORT_DDR) {
		return HAL_OK;
	}

	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_a_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_b_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_c_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_d_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_e_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_f_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			*((uint16_t *)pin_cfg) = ((*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)]))) >> 16) & 0xFFFF;
		} else {
			// pin index is even
			*((uint16_t *)pin_cfg) = (*((uint32_t *)(gpio_s_ctrl_reg_list[(pin_idx >> 1)]))) & 0xFFFF;
		}
		break;
	default:
		break;
	}


	return HAL_OK;
}


/**
 *  @brief To power on (enable) a IO pad .
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  IO pad power up OK.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_pwrup(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;
	AON_TypeDef *aon = AON;
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;


	DBG_MISC_INFO("\r\n%s => pin_name = 0x%x, pid = 0x%x\r\n", __func__, pin_name, periphl_id);
	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		// Internal Port ?
		if (port_idx == PORT_INTERNAL) {
			return HAL_OK;
		} else {
			DBG_MISC_ERR("hal_rtl_pin_pwrup: Unknown port (%u)\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}

	switch (port_idx) {
	case PORT_A:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_AON_GPIOA_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_B:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_SYSON_GPIOB_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_C:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_SYSON_GPIOC_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_D:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_SYSON_GPIOD_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_E:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_SYSON_GPIOE_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_F:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_PON_GPIOF_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_S:
		val = aon->AON_REG_AON_PAD_CTRL;
		val &= ~(AON_BIT_SYSON_GPIOS_PWD33);
		aon->AON_REG_AON_PAD_CTRL = val;
		break;
	case PORT_DDR:
		val = syson_s->SYSON_S_REG_SYS_DDRPHY_CTRL;
		val |= SYSON_S_BIT_DDRPHY_VCCON;
		syson_s->SYSON_S_REG_SYS_DDRPHY_CTRL = val;
		break;
	default:
		break;
	}
	ppinmux_manager->pinmux_pwr_log[port_idx] |= (1 << pin_idx);


	return HAL_OK;
}


/**
 *  @brief To power down (disable) a IO pad.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral owns this pin.
 *
 *  @return     HAL_OK:  IO pad power down OK.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pin_pwrdwn(uint32_t pin_name, uint32_t periphl_id)
{
	io_pin_t pin;
	uint32_t port_idx;
	uint32_t pin_idx;
	AON_TypeDef *aon = AON;
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;


	DBG_MISC_INFO("\r\n%s => pin_name = 0x%x, pid = 0x%x\r\n", __func__, pin_name, periphl_id);
	pin.pin_name = pin_name;
	port_idx = pin.pin_name_b.port;
	pin_idx = pin.pin_name_b.pin;

	if (port_idx >= PORT_MAX_NUM) {
		// Internal Port ?
		if (port_idx == PORT_INTERNAL) {
			return HAL_OK;
		} else {
			DBG_MISC_ERR("hal_rtl_pin_pwrdwn: Unknown port (%u)\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}

	ppinmux_manager->pinmux_pwr_log[port_idx] &= ~(1 << pin_idx);
	if (ppinmux_manager->pinmux_pwr_log[port_idx] == 0) {
		switch (port_idx) {
		case PORT_A:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_AON_GPIOA_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_B:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_SYSON_GPIOB_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_C:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_SYSON_GPIOC_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_D:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_SYSON_GPIOD_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_E:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_SYSON_GPIOE_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_F:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_PON_GPIOF_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_S:
			val = aon->AON_REG_AON_PAD_CTRL;
			val |= (AON_BIT_SYSON_GPIOS_PWD33);
			aon->AON_REG_AON_PAD_CTRL = val;
			break;
		case PORT_DDR:
			val = syson_s->SYSON_S_REG_SYS_DDRPHY_CTRL;
			val &= ~(SYSON_S_BIT_DDRPHY_VCCON);
			syson_s->SYSON_S_REG_SYS_DDRPHY_CTRL = val;
			break;
		default:
			break;
		}
	}


	return HAL_OK;
}


/**
 *  @brief To register a IO pin to the pin mux magagement.
 *         The pin mux management will do the checking of pin conflict and pin valid.
 *         And then power up the IO pad.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral will use this pin.
 *
 *  @return     HAL_OK:  Pin list register OK.
 *  @return     HAL_ERR_CONFLICT:  Pin conflict. The given pin
 *              is occupied by other peripheral device.
 *  @return     HAL_ERR_HW: the pin is invalid for this IC.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pinmux_register(uint32_t pin_name, uint32_t periphl_id)
{
	hal_status_t ret = HAL_OK;


	if (ppinmux_manager == NULL) {
		DBG_MISC_ERR("hal_rtl_pinmux_register: ppinmux_manager is NULL !!\r\n");
		return HAL_ERR_PARA;
	}

	if (ppinmux_manager->pin_reg_func != NULL) {
		ret = (ppinmux_manager->pin_reg_func)(pin_name, periphl_id);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	if (ppinmux_manager->pin_validat_func != NULL) {
		ret = (ppinmux_manager->pin_validat_func)(pin_name, periphl_id);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	if (ppinmux_manager->pin_mux_cfg_func != NULL) {
		ret = (ppinmux_manager->pin_mux_cfg_func)(pin_name, periphl_id);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	if (ppinmux_manager->ioport_pwrup_ctrl_func != NULL) {
		ret = (ppinmux_manager->ioport_pwrup_ctrl_func)(pin_name, periphl_id);
	}


	return ret;
}


/**
 *  @brief Unregister a IO pin from the pin mux management.
 *         The pin mux management will power down the IO pads.
 *
 *  @param[in]  pin_name  The IO pin.
 *  @param[in]  periphl_id  The ID of the peripheral own this pin.
 *
 *  @return     HAL_OK:  Pin unregister OK.
 *  @return     HAL_ERR_PARA:  The peripheral doesn't own this pin.
 */
SECTION_PINMUX_TEXT
hal_status_t hal_rtl_pinmux_unregister(uint32_t pin_name, uint32_t periphl_id)
{
	hal_status_t ret = HAL_OK;


	if (ppinmux_manager == NULL) {
		DBG_MISC_ERR("hal_rtl_pinmux_unregister: ppinmux_manager is NULL !!\r\n");
		return HAL_ERR_PARA;
	}

	if (ppinmux_manager->pin_unreg_func != NULL) {
		ret = (ppinmux_manager->pin_unreg_func)(pin_name, periphl_id);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	if (ppinmux_manager->pin_mux_cfg_func != NULL) {
		// Config pin-mux as GPIO (default function)
		ret = (ppinmux_manager->pin_mux_cfg_func)(pin_name, PID_GPIO);
		if (ret != HAL_OK) {
			return ret;
		}
	}

	if (ppinmux_manager->ioport_pwrdn_ctrl_func != NULL) {
		ret = (ppinmux_manager->ioport_pwrdn_ctrl_func)(pin_name, periphl_id);
	}


	return ret;
}

SECTION_PINMUX_TEXT
void hal_rtl_pinmux_rom_info_copy(uint32_t *p_dst, uint32_t *p_src, const uint32_t size)
{
	uint32_t i = 0;
	if ((p_dst == NULL) || (p_src == NULL)) {
		return;
	}
	if (size == 0) {
		return;
	}
	for (i = 0; i < size; i++) {
		*(p_dst + i) = *(p_src + i);
	}
}

/**
 *  @brief Backup or restore the pinmux info. in ROM.
 *
 *  @param[in]  op  OP_BACKUP or OP_RESTORE.
 *  @param[in]  obj_id  PWR_LOG or REG_LOG.
 *
 *  @returns void
 */
SECTION_PINMUX_TEXT
void hal_rtl_pinmux_rom_info_manage(hal_pin_mux_mang_t *pinmux_manag, uint8_t op, uint8_t obj_id)
{
	if (pinmux_manag == NULL) {
		return;
	}
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	switch (op) {
	case OP_BACKUP:
		if (obj_id == PWR_LOG) {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_pwr_log[0]), &(pinmux_manag->pinmux_pwr_log[0]), PORT_MAX_NUM);
		} else if (obj_id == REG_LOG) {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_reg_log[0]), &(pinmux_manag->pinmux_reg_log[0]), PORT_MAX_NUM);
		} else {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_pwr_log[0]), &(pinmux_manag->pinmux_pwr_log[0]), PORT_MAX_NUM);
			hal_rtl_pinmux_rom_info_copy(&(pinmux_reg_log[0]), &(pinmux_manag->pinmux_reg_log[0]), PORT_MAX_NUM);
		}
		break;
	case OP_RESTORE:
		if (obj_id == PWR_LOG) {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_manag->pinmux_pwr_log[0]), &(pinmux_pwr_log[0]), PORT_MAX_NUM);
		} else if (obj_id == REG_LOG) {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_manag->pinmux_reg_log[0]), &(pinmux_reg_log[0]), PORT_MAX_NUM);
		} else {
			hal_rtl_pinmux_rom_info_copy(&(pinmux_manag->pinmux_pwr_log[0]), &(pinmux_pwr_log[0]), PORT_MAX_NUM);
			hal_rtl_pinmux_rom_info_copy(&(pinmux_manag->pinmux_reg_log[0]), &(pinmux_reg_log[0]), PORT_MAX_NUM);
		}
		break;
	default:
		break;
	}
#endif
}


