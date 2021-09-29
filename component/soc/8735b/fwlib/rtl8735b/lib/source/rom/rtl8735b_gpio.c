/**************************************************************************//**
 * @file     rtl8735b_gpio.c
 * @brief    This file implements the GPIO HAL functions.
 *
 * @version  V1.00
 * @date     2020-11-11
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

#if CONFIG_GPIO_EN

#define SECTION_GPIO_TEXT           SECTION(".rom.hal_gpio.text")
#define SECTION_GPIO_DATA           SECTION(".rom.hal_gpio.data")
#define SECTION_GPIO_RODATA         SECTION(".rom.hal_gpio.rodata")
#define SECTION_GPIO_BSS            SECTION(".rom.hal_gpio.bss")
#define SECTION_GPIO_STUBS          SECTION(".rom.hal_gpio.stubs")


extern void *_memset(void *dst0, int Val, SIZE_T length);

/**
  * @brief The table of GPIO A control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_a_ctrl_list[] = {
	&(AON->AON_REG_AON_GPIOA_0_1_CTRL),
	&(AON->AON_REG_AON_GPIOA_2_3_CTRL),
	&(AON->AON_REG_AON_GPIOA_4_5_CTRL)
};

/**
  * @brief The table of GPIO B control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_b_ctrl_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOB_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOB_2_CTRL)
};

/**
  * @brief The table of GPIO C control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_c_ctrl_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOC_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOC_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOC_4_5_CTRL)
};

/**
  * @brief The table of GPIO D control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_d_ctrl_list[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER) // Test chip
	&(SYSON->SYSON_REG_SYSON_GPIOD_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_6_7_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_8_9_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_10_11_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_12_13_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_14_15_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_16_CTRL)
#else
	&(SYSON->SYSON_REG_SYSON_GPIOD_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_6_7_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_8_9_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_10_11_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_12_13_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_14_15_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_16_17_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_18_19_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOD_20_CTRL),
#endif
};

/**
  * @brief The table of GPIO E control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_e_ctrl_list[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER) // Test chip
	&(SYSON->SYSON_REG_SYSON_GPIOE_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_6_7_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_8_9_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_10_CTRL)
#else // MP chip
	&(SYSON->SYSON_REG_SYSON_GPIOE_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOE_6_CTRL)
#endif
};

/**
  * @brief The table of GPIO F control registers.
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_f_ctrl_list[] = {
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
  * These are for IO pad pull control, Schmitt Trigger, Drive Current
  * and Slew Rate control functions control registers. (System-Level)
  */
SECTION_GPIO_RODATA
const __IOM uint32_t *gpio_s_ctrl_list[] = {
	&(SYSON->SYSON_REG_SYSON_GPIOS_0_1_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_4_5_CTRL),
	&(SYSON->SYSON_REG_SYSON_GPIOS_6_CTRL)
};

/**
 * @addtogroup hs_hal_gpio GPIO
 * @{
 */

/**
  * @brief The global data structure to handle common resource
  *        for all SYSON GPIO adapters.
  */
SECTION_GPIO_BSS
phal_gpio_comm_adapter_t _pgpio_comm_adp;

/**
  * @brief The global data structure to handle common resource
  *        for all AON GPIO adapters.
  */
SECTION_GPIO_BSS
phal_aon_gpio_comm_adapter_t _paon_gpio_comm_adp;

/**
  * @brief The global data structure to handle common resource
  *        for all PON GPIO adapters.
  */
SECTION_GPIO_BSS
phal_pon_gpio_comm_adapter_t _ppon_gpio_comm_adp;

SECTION_GPIO_BSS
uint32_t pull_ctrl;

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode direction status registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_dmd_sts[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_DMD_STS),
	&(PON_GPIO->GPIO_PORT_A_DMD_STS),
	&(SYSON_GPIO->GPIO_PORT_A_DMD_STS),
	&(SYSON_GPIO->GPIO_PORT_B_DMD_STS)
};

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode input enable registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_idm_en[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_IDM_EN),
	&(PON_GPIO->GPIO_PORT_A_IDM_EN),
	&(SYSON_GPIO->GPIO_PORT_A_IDM_EN),
	&(SYSON_GPIO->GPIO_PORT_B_IDM_EN)
};

/**
  * @brief The table of all GPIO port (SYSON/AON/PON) data mode output enable registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_odm_en[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODM_EN),
	&(PON_GPIO->GPIO_PORT_A_ODM_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODM_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODM_EN)
};

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode output status registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_od_sts[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_OD_STS),
	&(PON_GPIO->GPIO_PORT_A_OD_STS),
	&(SYSON_GPIO->GPIO_PORT_A_OD_STS),
	&(SYSON_GPIO->GPIO_PORT_B_OD_STS)
};

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode output low registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_odl_en[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODL_EN),
	&(PON_GPIO->GPIO_PORT_A_ODL_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODL_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODL_EN)
};

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode output high registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_odh_en[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODH_EN),
	&(PON_GPIO->GPIO_PORT_A_ODH_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODH_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODH_EN)
};

/**
  * @brief The table of all GPIO (SYSON/AON/PON) port data mode output toggle registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_odt_en[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODT_EN),
	&(PON_GPIO->GPIO_PORT_A_ODT_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODT_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODT_EN)

};

/**
  * @brief The table of all GPIO port data pin status registers address. AON GPIO uses only Port A in ProII.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pport_dp_sts[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_DP_STS),
	&(PON_GPIO->GPIO_PORT_A_DP_STS),
	&(SYSON_GPIO->GPIO_PORT_A_DP_STS),
	&(SYSON_GPIO->GPIO_PORT_B_DP_STS)

};

/**
  * @brief The table of SYSON GPIO interrupt pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pgpio_irq_cfg[GPIO_MAX_INT_PIN] = {
	&(SYSON_GPIO->GPIO_INT0_SEL),
	&(SYSON_GPIO->GPIO_INT1_SEL),
	&(SYSON_GPIO->GPIO_INT2_SEL),
	&(SYSON_GPIO->GPIO_INT3_SEL),
	&(SYSON_GPIO->GPIO_INT4_SEL),
	&(SYSON_GPIO->GPIO_INT5_SEL),
	&(SYSON_GPIO->GPIO_INT6_SEL),
	&(SYSON_GPIO->GPIO_INT7_SEL),
	&(SYSON_GPIO->GPIO_INT8_SEL),
	&(SYSON_GPIO->GPIO_INT9_SEL),
	&(SYSON_GPIO->GPIO_INT10_SEL),
	&(SYSON_GPIO->GPIO_INT11_SEL),
	&(SYSON_GPIO->GPIO_INT12_SEL),
	&(SYSON_GPIO->GPIO_INT13_SEL),
	&(SYSON_GPIO->GPIO_INT14_SEL),
	&(SYSON_GPIO->GPIO_INT15_SEL)
};

/**
  * @brief The table of AON GPIO interrupt pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *paon_gpio_irq_cfg[GPIO_MAX_INT_PIN] = {
	&(AON_GPIO->GPIO_INT0_SEL),
	&(AON_GPIO->GPIO_INT1_SEL),
	&(AON_GPIO->GPIO_INT2_SEL),
	&(AON_GPIO->GPIO_INT3_SEL),
	&(AON_GPIO->GPIO_INT4_SEL),
	&(AON_GPIO->GPIO_INT5_SEL)
};

/**
  * @brief The table of PON GPIO interrupt pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *ppon_gpio_irq_cfg[GPIO_MAX_INT_PIN] = {
	&(PON_GPIO->GPIO_INT0_SEL),
	&(PON_GPIO->GPIO_INT1_SEL),
	&(PON_GPIO->GPIO_INT2_SEL),
	&(PON_GPIO->GPIO_INT3_SEL),
	&(PON_GPIO->GPIO_INT4_SEL),
	&(PON_GPIO->GPIO_INT5_SEL),
	&(PON_GPIO->GPIO_INT6_SEL),
	&(PON_GPIO->GPIO_INT7_SEL),
	&(PON_GPIO->GPIO_INT8_SEL),
	&(PON_GPIO->GPIO_INT9_SEL),
	&(PON_GPIO->GPIO_INT10_SEL),
	&(PON_GPIO->GPIO_INT11_SEL),
	&(PON_GPIO->GPIO_INT12_SEL),
	&(PON_GPIO->GPIO_INT13_SEL),
	&(PON_GPIO->GPIO_INT14_SEL),
	&(PON_GPIO->GPIO_INT15_SEL)
};

/**
  * @brief The table of SYSON GPIO debouncing pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *pgpio_debounce_cfg[GPIO_MAX_DEBOUNCE_PIN] = {
	&(SYSON_GPIO->GPIO_DEB0_SEL),
	&(SYSON_GPIO->GPIO_DEB1_SEL),
	&(SYSON_GPIO->GPIO_DEB2_SEL),
	&(SYSON_GPIO->GPIO_DEB3_SEL),
	&(SYSON_GPIO->GPIO_DEB4_SEL),
	&(SYSON_GPIO->GPIO_DEB5_SEL),
	&(SYSON_GPIO->GPIO_DEB6_SEL),
	&(SYSON_GPIO->GPIO_DEB7_SEL),
	&(SYSON_GPIO->GPIO_DEB8_SEL),
	&(SYSON_GPIO->GPIO_DEB9_SEL),
	&(SYSON_GPIO->GPIO_DEB10_SEL),
	&(SYSON_GPIO->GPIO_DEB11_SEL),
	&(SYSON_GPIO->GPIO_DEB12_SEL),
	&(SYSON_GPIO->GPIO_DEB13_SEL),
	&(SYSON_GPIO->GPIO_DEB14_SEL),
	&(SYSON_GPIO->GPIO_DEB15_SEL)
};

/**
  * @brief The table of AON GPIO debouncing pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *paon_gpio_debounce_cfg[GPIO_MAX_DEBOUNCE_PIN] = {
	&(AON_GPIO->GPIO_DEB0_SEL),
	&(AON_GPIO->GPIO_DEB1_SEL),
	&(AON_GPIO->GPIO_DEB2_SEL),
	&(AON_GPIO->GPIO_DEB3_SEL),
	&(AON_GPIO->GPIO_DEB4_SEL),
	&(AON_GPIO->GPIO_DEB5_SEL)
};

/**
  * @brief The table of PON GPIO debouncing pin selection registers address.
  */
SECTION_GPIO_RODATA
const volatile uint32_t *ppon_gpio_debounce_cfg[GPIO_MAX_DEBOUNCE_PIN] = {
	&(PON_GPIO->GPIO_DEB0_SEL),
	&(PON_GPIO->GPIO_DEB1_SEL),
	&(PON_GPIO->GPIO_DEB2_SEL),
	&(PON_GPIO->GPIO_DEB3_SEL),
	&(PON_GPIO->GPIO_DEB4_SEL),
	&(PON_GPIO->GPIO_DEB5_SEL),
	&(PON_GPIO->GPIO_DEB6_SEL),
	&(PON_GPIO->GPIO_DEB7_SEL),
	&(PON_GPIO->GPIO_DEB8_SEL),
	&(PON_GPIO->GPIO_DEB9_SEL),
	&(PON_GPIO->GPIO_DEB10_SEL),
	&(PON_GPIO->GPIO_DEB11_SEL),
	&(PON_GPIO->GPIO_DEB12_SEL),
	&(PON_GPIO->GPIO_DEB13_SEL),
	&(PON_GPIO->GPIO_DEB14_SEL),
	&(PON_GPIO->GPIO_DEB15_SEL)
};



/**
  * @brief The table to convert chip IO ports to internal GPIO IP ports.
  */
SECTION_GPIO_RODATA
uint32_t chip_port_2_ip_port[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER) // Test Chip
	(GPIO_IP_PORT0 << 24) | (0 << 18) | 0x0003F,      ///< Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins
	(GPIO_IP_PORT2 << 24) | (0 << 18) | 0x00007,      ///< Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins
	(GPIO_IP_PORT2 << 24) | (3 << 18) | 0x0003F,      ///< Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins
	(GPIO_IP_PORT2 << 24) | (9 << 18) | 0x1FFFF,      ///< Port D: GPIO IP Port2[25:9] // SYSON GPIO 17 pins
	(GPIO_IP_PORT2 << 24) | (26 << 18) | 0x0003F,     ///< Port E1: GPIO IP Port2[31:26] // SYSON GPIO 6 pins
	(GPIO_IP_PORT3 << 24) | (0 << 18) | 0x0001F,      ///< Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 1)
	(GPIO_IP_PORT1 << 24) | (0 << 18) | 0x3FFFF,      ///< Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins
	(GPIO_IP_PORT3 << 24) | (5 << 18) | 0x0007F       ///< Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 1)

#else // MP Chip
	(GPIO_IP_PORT0 << 28) | (0 << 21) | 0x0003F,	    ///< Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins
	(GPIO_IP_PORT2 << 28) | (0 << 21) | 0x00007,	    ///< Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins
	(GPIO_IP_PORT2 << 28) | (3 << 21) | 0x0003F,	    ///< Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins
	(GPIO_IP_PORT2 << 28) | (9 << 21) | 0x1FFFFF,     ///< Port D: GPIO IP Port2[29:9] // SYSON GPIO 21 pins
	(GPIO_IP_PORT2 << 28) | (30 << 21) | 0x0003,	    ///< Port E1: GPIO IP Port2[31:30] // SYSON GPIO 2 pins
	(GPIO_IP_PORT3 << 28) | (0 << 21) | 0x0001F,	    ///< Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 1)
	(GPIO_IP_PORT1 << 28) | (0 << 21) | 0x3FFFF,	    ///< Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins
	(GPIO_IP_PORT3 << 28) | (5 << 21) | 0x0007F	    ///< Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 1)
#endif
};

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_gpio_rom_func GPIO HAL ROM APIs.
 * @ingroup hs_hal_gpio
 * @{
 * @brief The GPIO HAL ROM APIs. The user application (in RAM space) should not call these APIs directly.
 *        There is another set of GPIO HAL APIs in the RAM space provided for user application.
 */

/**
  \brief  The stubs functions table of the GPIO HAL functions in ROM.
*/
SECTION_GPIO_STUBS const hal_gpio_func_stubs_t hal_gpio_stubs = {
	.ppgpio_comm_adp = &_pgpio_comm_adp,
	.ppaon_gpio_comm_adp = &_paon_gpio_comm_adp,
	.pppon_gpio_comm_adp = &_ppon_gpio_comm_adp,
	.hal_gpio_reg_irq = hal_rtl_gpio_reg_irq,
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
	.hal_gpio_interrupt_clk_sel = hal_rtl_gpio_interrupt_clk_sel,
#endif
	.hal_gpio_comm_init = hal_rtl_gpio_comm_init,
	.hal_aon_gpio_comm_init = hal_rtl_aon_gpio_comm_init, // new
	.hal_pon_gpio_comm_init = hal_rtl_pon_gpio_comm_init, // new
	.hal_gpio_comm_deinit = hal_rtl_gpio_comm_deinit,
	.hal_aon_gpio_comm_deinit = hal_rtl_aon_gpio_comm_deinit, // new
	.hal_pon_gpio_comm_deinit = hal_rtl_pon_gpio_comm_deinit, // new
	.hal_gpio_enter_critical = hal_rtl_gpio_enter_critical,
	.hal_aon_gpio_enter_critical = hal_rtl_aon_gpio_enter_critical, // new
	.hal_pon_gpio_enter_critical = hal_rtl_pon_gpio_enter_critical, // new
	.hal_gpio_exit_critical = hal_rtl_gpio_exit_critical,
	.hal_aon_gpio_exit_critical = hal_rtl_aon_gpio_exit_critical, // new
	.hal_pon_gpio_exit_critical = hal_rtl_pon_gpio_exit_critical, // new
	.hal_gpio_init = hal_rtl_gpio_init,
	.hal_gpio_deinit = hal_rtl_gpio_deinit,
	.hal_gpio_set_dir = hal_rtl_gpio_set_dir,
	.hal_gpio_get_dir = hal_rtl_gpio_get_dir,
	.hal_gpio_write = hal_rtl_gpio_write,
	.hal_gpio_toggle = hal_rtl_gpio_toggle,
	.hal_gpio_read = hal_rtl_gpio_read,
	.hal_gpio_debounce_enable = hal_rtl_gpio_debounce_enable,
	.hal_gpio_read_debounce = hal_rtl_gpio_read_debounce,
	.hal_gpio_debounce_disable = hal_rtl_gpio_debounce_disable,
	.hal_gpio_irq_init = hal_rtl_gpio_irq_init,
	.hal_gpio_irq_deinit = hal_rtl_gpio_irq_deinit,
	.hal_gpio_irq_set_trig_type = hal_rtl_gpio_irq_set_trig_type,
	.hal_gpio_irq_get_trig_type = hal_rtl_gpio_irq_get_trig_type,
	.hal_gpio_irq_enable = hal_rtl_gpio_irq_enable,
	.hal_gpio_irq_disable = hal_rtl_gpio_irq_disable,
	.hal_gpio_irq_debounce_enable = hal_rtl_gpio_irq_debounce_enable,
	.hal_gpio_irq_debounce_disable = hal_rtl_gpio_irq_debounce_disable,
	.hal_gpio_irq_read = hal_rtl_gpio_irq_read,
	.hal_gpio_port_init = hal_rtl_gpio_port_init,
	.hal_gpio_port_deinit = hal_rtl_gpio_port_deinit,
	.hal_gpio_port_write = hal_rtl_gpio_port_write,
	.hal_gpio_port_read = hal_rtl_gpio_port_read,
	.hal_gpio_pull_ctrl = hal_rtl_gpio_pull_ctrl,
	.hal_gpio_schmitt_ctrl = hal_rtl_gpio_schmitt_ctrl,
	.hal_gpio_port_dir = hal_rtl_gpio_port_dir,
	.hal_gpio_drive_ctrl = hal_rtl_gpio_drive_ctrl,
	.hal_gpio_slew_rate_ctrl = hal_rtl_gpio_slew_rate_ctrl
};

#if defined(CONFIG_BUILD_SECURE)
// Todo modify to RAM
//typedef void __attribute__((cmse_nonsecure_call)) ns_func(uint32_t, gpio_int_trig_type_t); // weide try

#endif

/**
 *  @brief The interrupt handler for AON GPIO interrupt pins.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void AON_IRQHandler(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_rtl_irq_clear_pending(AON_IRQn);

	int_sts = AON_GPIO->GPIO_INT_STS;

	if (_paon_gpio_comm_adp == NULL) {
		// Not initialed yet, ignore interrupt
		AON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _paon_gpio_comm_adp->gpio_irq_list_head;

	while ((pgpio_irq != NULL) && (int_sts != 0)) {

		int_idx = pgpio_irq->int_idx;
		bit_mask = 1 << int_idx;
		if (int_sts & bit_mask) {
			// this AON GPIO pin IRQ is pending
			// clear interrupt status
			AON_GPIO->GPIO_INT_CLR = bit_mask;
			int_sts &= ~bit_mask;
			if (pgpio_irq->irq_callback != NULL) {
				int_type = pgpio_irq->int_type;
				if (pgpio_irq->int_type == GPIO_IntType_EdgeDual) {
					if (*((volatile uint32_t *)(pgpio_irq->in_port)) & pgpio_irq->bit_mask) {
						int_type = GPIO_IntType_EdgeRising;
					} else {
						int_type = GPIO_IntType_EdgeFalling;
					}
				}

#if defined(CONFIG_BUILD_SECURE)
				// call the user registed call-back(S call NS)
#if 0
// TOdo, modify to RAM
				ns_func *fptr = cmse_nsfptr_create((ns_func *)(pgpio_irq->irq_callback));
				fptr(pgpio_irq->irq_callback_arg, int_type);
#endif
				//fptr((pgpio_irq->irq_callback)(pgpio_irq->irq_callback_arg, int_type)); // weide try
#else
				// call the user registed call-back (for Ignore Secure Build ONLY, because AON GPIO is Secure-only by default)
				(pgpio_irq->irq_callback)(pgpio_irq->irq_callback_arg, int_type);
#endif

			}
		}
		// check next AON GPIO IRQ pin
		pgpio_irq = pgpio_irq->pnext;
	}

	if (int_sts != 0) {
		// error: somehow, at least one AON GPIO pin IRQ didn't be handled
		// But we still need to clear all interrupt status
		AON_GPIO->GPIO_INT_CLR = int_sts;
		_paon_gpio_comm_adp->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();
}

/**
 *  @brief The interrupt handler for SYSON GPIO interrupt pins.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void GPIO_IRQHandler(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_rtl_irq_clear_pending(GPIO_IRQn);

	int_sts = SYSON_GPIO->GPIO_INT_STS;

	if (_pgpio_comm_adp == NULL) {
		// Not initialed yet, ignore interrupt
		SYSON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _pgpio_comm_adp->gpio_irq_list_head;

	while ((pgpio_irq != NULL) && (int_sts != 0)) {

		int_idx = pgpio_irq->int_idx;
		bit_mask = 1 << int_idx;
		if (int_sts & bit_mask) {
			// this SYSON GPIO pin IRQ is pending
			// clear interrupt status
			SYSON_GPIO->GPIO_INT_CLR = bit_mask;
			int_sts &= ~bit_mask;
			if (pgpio_irq->irq_callback != NULL) {
				int_type = pgpio_irq->int_type;
				if (pgpio_irq->int_type == GPIO_IntType_EdgeDual) {
					if (*((volatile uint32_t *)(pgpio_irq->in_port)) & pgpio_irq->bit_mask) {
						int_type = GPIO_IntType_EdgeRising;
					} else {
						int_type = GPIO_IntType_EdgeFalling;
					}
				}
				// call the user registed call-back
				(pgpio_irq->irq_callback)(pgpio_irq->irq_callback_arg, int_type);

			}
		}
		// check next SYSON GPIO IRQ pin
		pgpio_irq = pgpio_irq->pnext;
	}

	if (int_sts != 0) {
		// error: somehow, at least one SYSON GPIO pin IRQ didn't be handled
		// But we still need to clear all interrupt status
		SYSON_GPIO->GPIO_INT_CLR = int_sts;
		_pgpio_comm_adp->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();
}

/**
 *  @brief The interrupt handler for PON GPIO interrupt pins.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void PonGPIO_IRQHandler(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_rtl_irq_clear_pending(PonGPIO_IRQn);

	int_sts = PON_GPIO->GPIO_INT_STS;

	if (_ppon_gpio_comm_adp == NULL) {
		// Not initialed yet, ignore interrupt
		PON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _ppon_gpio_comm_adp->gpio_irq_list_head;

	while ((pgpio_irq != NULL) && (int_sts != 0)) {

		int_idx = pgpio_irq->int_idx;
		bit_mask = 1 << int_idx;
		if (int_sts & bit_mask) {
			// this PON GPIO pin IRQ is pending
			// clear interrupt status
			PON_GPIO->GPIO_INT_CLR = bit_mask;
			int_sts &= ~bit_mask;
			if (pgpio_irq->irq_callback != NULL) {
				int_type = pgpio_irq->int_type;
				if (pgpio_irq->int_type == GPIO_IntType_EdgeDual) {
					if (*((volatile uint32_t *)(pgpio_irq->in_port)) & pgpio_irq->bit_mask) {
						int_type = GPIO_IntType_EdgeRising;
					} else {
						int_type = GPIO_IntType_EdgeFalling;
					}
				}
				// call the user registed call-back
				(pgpio_irq->irq_callback)(pgpio_irq->irq_callback_arg, int_type);
			}
		}
		// check next PON GPIO IRQ pin
		pgpio_irq = pgpio_irq->pnext;
	}

	if (int_sts != 0) {
		// error: somehow, at least one PON GPIO pin IRQ didn't be handled
		// But we still need to clear all interrupt status
		PON_GPIO->GPIO_INT_CLR = int_sts;
		_ppon_gpio_comm_adp->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();

}

/**
 *  @brief To register an interrupt handler for all GPIO interrupt pins and enable
 *         the GPIO interrupt.
 *
 *  @param[in]  irq_handler  The GPIO interrupt handler.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_reg_irq(gpio_type_t gpio_type, irq_handler_t irq_handler)  // possible source of problems!
{
	// IRQ vector may has been registered, disable and re-register it - in order of GPIO type
	if (gpio_type == AonGPIO) {
#if !defined(CONFIG_BUILD_NONSECURE) // Ignore Secure
		DBG_GPIO_INFO("NTZ ROM AON GPIO reg irq\r\n");
		// AON GPIO
		hal_rtl_irq_disable(AON_IRQn);
		__ISB();
		hal_rtl_irq_set_vector(AON_IRQn, (uint32_t)irq_handler);  // need to tell if repeated irq_handler has been registered?
		hal_rtl_irq_enable(AON_IRQn);
#endif

	} else if (gpio_type == SysonGPIO) {
		// SYSON GPIO
		hal_rtl_irq_disable(GPIO_IRQn);
		__ISB();
		hal_rtl_irq_set_vector(GPIO_IRQn, (uint32_t)irq_handler);  // need to tell if repeated irq_handler has been registered?
		hal_rtl_irq_enable(GPIO_IRQn);

	} else if (gpio_type == PonGPIO) {
		// PON GPIO
		hal_rtl_irq_disable(PonGPIO_IRQn);
		__ISB();
		hal_rtl_irq_set_vector(PonGPIO_IRQn, (uint32_t)irq_handler);  // need to tell if repeated irq_handler has been registered?
		hal_rtl_irq_enable(PonGPIO_IRQn);
	}

}

/**
 *  @brief Changes the interrupt clock selection for the various GPIO types -
 *         AON GPIO/PON GPIO/SYSON GPIO.
 *
 *  @param[in]  gpio_type   The type of GPIO - AON/PON/SYSON GPIO
 *  @param[in]  clk_sel     The clock selection according to the
 *                          various types of GPIO
 *
 *  @return     HAL_ERR:  Input arguments are invalid.
 *  @return     HAL_OK:  Debounce function is enabled on this GPIO.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_interrupt_clk_sel(gpio_type_t gpio_type, uint8_t clk_sel)
{

	if (gpio_type > 2) { // means user didn't choose AON/PON/SYSON GPIO
		DBG_GPIO_ERR("Invalid GPIO type selection!\r\n");
		return HAL_ERR_PARA;
	}

	if (clk_sel > 1) {
		DBG_GPIO_WARN("clk_sel is greater than 1!\r\n");
	}

	if (gpio_type == AonGPIO) {
		if (clk_sel) {
			// HW setting
			AON->AON_REG_AON_CLK_CTRL |= AON_BIT_GPIO_INTR_CLK;
			DBG_GPIO_INFO("AON GPIO interrupt clock: sclk\r\n");
		} else {
			// HW setting 2
			AON->AON_REG_AON_CLK_CTRL &= ~(AON_BIT_GPIO_INTR_CLK);
			DBG_GPIO_INFO("AON GPIO interrupt clock: pclk\r\n");
		}

	} else if (gpio_type == PonGPIO) {
		if (clk_sel) {
			// HW setting
			PON->PON_REG_PON_FUNC_CLK_CTRL |= PON_BIT_INTR_CLK_GPIO_SEL;
			DBG_GPIO_INFO("PON GPIO interrupt clock: 32kHz clock\r\n");
		} else {
			// HW setting 2
			PON->PON_REG_PON_FUNC_CLK_CTRL &= ~(PON_BIT_INTR_CLK_GPIO_SEL);
			DBG_GPIO_INFO("PON GPIO interrupt clock: APB clk\r\n");
		}

	} else if (gpio_type == SysonGPIO) { // possible source of problems!!! - careful of S/NS
		if (clk_sel) {
			// HW setting
			SYSON_S->SYSON_S_REG_GPIO_CTRL |= SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL;
			DBG_GPIO_INFO("SYSON GPIO interrupt clock: Debounce clk\r\n");
		} else {
			// HW setting 2
			SYSON_S->SYSON_S_REG_GPIO_CTRL &= ~(SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL);
			DBG_GPIO_INFO("SYSON GPIO interrupt clock: APB clk\r\n");
		}
	}

	return HAL_OK;
}

/**
 *  @brief Initials the SYSON GPIO group common resource.
 *           - Enable the SYSON GPIO hardware function.
 *           - Initial the SYSON GPIO group adapter.
 *
 *  @param[in] pgpio_comm_adp  The SYSON GPIO group adapter. For all SYSON GPIO pins common
 *             resource handling.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_comm_init(phal_gpio_comm_adapter_t pgpio_comm_adp)
{
	pgpio_comm_adp->gpio_irq_using = 0;
	pgpio_comm_adp->gpio_irq_list_head = NULL;
	pgpio_comm_adp->gpio_irq_list_tail = NULL;

	_pgpio_comm_adp = pgpio_comm_adp;

	hal_rtl_irq_set_priority(GPIO_IRQn, GPIO_IRQPri);
	hal_rtl_gpio_reg_irq(SysonGPIO, (irq_handler_t)&GPIO_IRQHandler);
}

/**
 *  @brief Initials the AON GPIO group common resource.
 *           - Enable the AON GPIO hardware function.
 *           - Initial the AON GPIO group adapter.
 *
 *  @param[in] pgpio_comm_adp  The AON GPIO group adapter. For all AON GPIO pins common
 *             resource handling.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_aon_gpio_comm_init(phal_aon_gpio_comm_adapter_t paon_gpio_comm_adp)
{
	paon_gpio_comm_adp->gpio_irq_using = 0;
	paon_gpio_comm_adp->gpio_irq_list_head = NULL;
	paon_gpio_comm_adp->gpio_irq_list_tail = NULL;

	_paon_gpio_comm_adp = paon_gpio_comm_adp;

#if !defined(CONFIG_BUILD_NONSECURE) // Ignore Secure
	DBG_GPIO_INFO("NTZ ROM AON GPIO comm init\r\n");
	hal_rtl_irq_set_priority(AON_IRQn, AON_IRQPri);
	hal_rtl_gpio_reg_irq(AonGPIO, (irq_handler_t)&AON_IRQHandler);
#endif
}

/**
 *  @brief Initials the PON GPIO group common resource.
 *           - Enable the PON GPIO hardware function.
 *           - Initial the PON GPIO group adapter.
 *
 *  @param[in] pgpio_comm_adp  The PON GPIO group adapter. For all PON GPIO pins common
 *             resource handling.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_pon_gpio_comm_init(phal_pon_gpio_comm_adapter_t ppon_gpio_comm_adp)
{
	ppon_gpio_comm_adp->gpio_irq_using = 0;
	ppon_gpio_comm_adp->gpio_irq_list_head = NULL;
	ppon_gpio_comm_adp->gpio_irq_list_tail = NULL;

	_ppon_gpio_comm_adp = ppon_gpio_comm_adp;

	hal_rtl_irq_set_priority(PonGPIO_IRQn, PonGPIO_IRQPri);
	hal_rtl_gpio_reg_irq(PonGPIO, (irq_handler_t)&PonGPIO_IRQHandler);
}

/**
 *  @brief To de-initial the SYSON GPIO common resource.
 *           - Disable the SYSON GPIO hardware function.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_comm_deinit(void)
{
	if (_pgpio_comm_adp == NULL) {
		return;
	}
#if !defined(CONFIG_BUILD_NONSECURE)
	hal_rtl_sys_peripheral_en(GPIO_SYS, DISABLE);
#endif

	_pgpio_comm_adp = NULL;
}

/**
 *  @brief To de-initial the AON GPIO common resource.
 *           - Disable the AON GPIO hardware function.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_aon_gpio_comm_deinit(void)
{
	if (_paon_gpio_comm_adp == NULL) {
		return;
	}
#if !defined(CONFIG_BUILD_NONSECURE)
	hal_rtl_sys_peripheral_en(GPIO_AON, DISABLE);
#endif

	_paon_gpio_comm_adp = NULL;
}

/**
 *  @brief To de-initial the PON GPIO common resource.
 *           - Disable the PON GPIO hardware function.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_pon_gpio_comm_deinit(void)
{
	if (_ppon_gpio_comm_adp == NULL) {
		return;
	}
#if !defined(CONFIG_BUILD_NONSECURE)
	hal_rtl_sys_peripheral_en(GPIO_PON, DISABLE);
#endif

	_ppon_gpio_comm_adp = NULL;
}

/**
 *  @brief To enter a critical section of code, mainly it
 *         disable the SYSON GPIO interrupt to prevent any race condition.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_enter_critical(void)
{

	if (__get_IPSR() == (GPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_rtl_irq_disable(GPIO_IRQn);

	__ISB();
	_pgpio_comm_adp->critical_lv++;
	__DSB();
}

/**
 *  @brief To enter a critical section of code, mainly it
 *         disable the AON GPIO interrupt to prevent any race condition.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_aon_gpio_enter_critical(void)
{

	if (__get_IPSR() == (AON_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_rtl_irq_disable(AON_IRQn);

	__ISB();
	_paon_gpio_comm_adp->critical_lv++;
	__DSB();
}

/**
 *  @brief To enter a critical section of code, mainly it
 *         disable the PON GPIO interrupt to prevent any race condition.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_pon_gpio_enter_critical(void)
{

	if (__get_IPSR() == (PonGPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_rtl_irq_disable(PonGPIO_IRQn);

	__ISB();
	_ppon_gpio_comm_adp->critical_lv++;
	__DSB();
}

/**
 *  @brief To exit a critical code section, it will re-enable the SYSON GPIO interrupt
 *         only when the exiting critical section is the top level.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_exit_critical(void)
{
	if (__get_IPSR() == (GPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_pgpio_comm_adp->critical_lv > 0) {
		_pgpio_comm_adp->critical_lv--;
		__DSB();
		if (_pgpio_comm_adp->critical_lv == 0) {
			hal_rtl_irq_enable(GPIO_IRQn);
			__ISB();
		}
	}
}

/**
 *  @brief To exit a critical code section, it will re-enable the AON GPIO interrupt
 *         only when the exiting critical section is the top level.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_aon_gpio_exit_critical(void)
{
	if (__get_IPSR() == (AON_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_paon_gpio_comm_adp->critical_lv > 0) {
		_paon_gpio_comm_adp->critical_lv--;
		__DSB();
		if (_paon_gpio_comm_adp->critical_lv == 0) {
			hal_rtl_irq_enable(AON_IRQn);
			__ISB();
		}
	}
}

/**
 *  @brief To exit a critical code section, it will re-enable the PON GPIO interrupt
 *         only when the exiting critical section is the top level.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_pon_gpio_exit_critical(void)
{
	if (__get_IPSR() == (PonGPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_ppon_gpio_comm_adp->critical_lv > 0) {
		_ppon_gpio_comm_adp->critical_lv--;
		__DSB();
		if (_ppon_gpio_comm_adp->critical_lv == 0) {
			hal_rtl_irq_enable(PonGPIO_IRQn);
			__ISB();
		}
	}
}

/**
 *  @brief Initials a GPIO pin.
 *           - Defult configure the GPIO pin as a normal input pin (not an interrupt pin).
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *  @param[in]  pin_name  The GPIO pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GPIO pin initialization OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_init(phal_gpio_adapter_t pgpio_adapter, uint32_t pin_name)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);
	uint32_t bit_mask;
	uint32_t *port_idm_en;

// Default to 0, which safe because if GPIO pin init fails, an early return via HAL_ERR_PARA would ensue.
	uint8_t port_array_idx = 0; // number indicator for array indices, e.g. pport_dp_sts[] based on port_idx

	// These parameters tell hal_rtl_gpio_init() [ROM] that GPIO power and clock have been turned on in hal_gpio_init() [RAM].
	// Thus we will not turn on GPIO power and clock again in ROM. This is only for the case with Secure Build.
#if !defined(CONFIG_BUILD_NONSECURE)
	uint32_t s_ram_gpio_on_local = pgpio_adapter->s_ram_gpio_on;
	uint32_t s_ram_aon_gpio_on_local = pgpio_adapter->s_ram_aon_gpio_on;
	uint32_t s_ram_pon_gpio_on_local = pgpio_adapter->s_ram_pon_gpio_on;
#endif

	// 0: AON GPIO, 1: SYSON GPIO, 2: PON GPIO
	// Default to 0, which safe because if GPIO pin init fails, an early return via HAL_ERR_PARA would ensue.
	gpio_type_t gpio_type = 0;

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("GPIO Init Invalid: port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}
	// convert chip pin definition to IP pin definition

	/* *** Test Chip ***
	 * Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins - GPIOA0 - A5
	 * Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins - GPIOB0 - B2
	 * Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins - GPIOC0 - C5
	 * Port D: GPIO IP Port2[25:9] // SYSON GPIO 17 pins - GPIOD0 - D16
	 * Port E1: GPIO IP Port2[31:26] // SYSON GPIO 6 pins - GPIOE0 - E5
	 * Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 3 from Port 2) - GPIOE6 - E10
	 * Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins - GPIOF0 - F17
	 * Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 3 from Port 2) - GPIO

	 *** MP Chip ***
	 * Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins - GPIOA0 - A5
	 * Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins - GPIOB0 - B2
	 * Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins - GPIOC0 - C5
	 * Port D: GPIO IP Port2[29:9] // SYSON GPIO 21 pins - GPIOD0 - D20
	 * Port E1: GPIO IP Port2[31:30] // SYSON GPIO 2 pins - GPIOE0 - E1
	 * Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 3 from Port 2) - GPIOE2 - E6
	 * Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins - GPIOF0 - F17
	 * Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 3 from Port 2) - GPIO
	 */

	switch (port_idx) {
	case PORT_A: // AON GPIO
		port_array_idx = 0;
		pin_idx += 0;
		gpio_type = AonGPIO;
		break;
	case PORT_B:
		port_array_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 0;
		gpio_type = SysonGPIO;
		break;
	case PORT_C:
		port_array_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 3; // pin starts at 4th position, Pin 3 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	case PORT_D:
		port_array_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 9; // pin starts at 10th position, Pin 9 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	case PORT_E:
#if IS_CUT_TEST(CONFIG_CHIP_VER) // Test Chip
		if (pin_idx > 5) {
			port_array_idx = 3;
			pin_idx -= 6; // because pin_idx = 6 will be mapped to Group B[0], pin_idx = 7 mapped to Group B[1], etc
		} else { // means still at Port A [31:26]
			port_array_idx = 2;
			pin_idx += 26;
		}
#else // MP Chip
		if (pin_idx > 1) {
			port_array_idx = 3;
			pin_idx -= 2; // because pin_idx = 2 will be mapped to Group B[0], pin_idx = 3 mapped to Group B[1], etc
		} else { // means still at Port A [31:30]
			port_array_idx = 2;
			pin_idx += 30;
		}
#endif
		gpio_type = SysonGPIO;
		break;
	case PORT_F: // PON GPIO
		port_array_idx = 1;
		pin_idx += 0;
		gpio_type = PonGPIO;
		break;
	case PORT_S: // SYSON GPIO Group B
		port_array_idx = 3;
		pin_idx += 5; // pin starts at 6th position, Pin 5 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	}

	_memset((void *) pgpio_adapter, 0, sizeof(hal_gpio_adapter_t));

	bit_mask = 1 << pin_idx;
	DBG_GPIO_INFO("bit_mask:%x\r\n", bit_mask);

	pgpio_adapter->pin_name = pin_name;
	pgpio_adapter->port_idx = port_array_idx;
	pgpio_adapter->pin_idx = pin_idx;
	pgpio_adapter->bit_mask = 1 << pin_idx;
	pgpio_adapter->in_port = (uint32_t *)pport_dp_sts[port_array_idx];
	DBG_GPIO_INFO("in_port: %x\r\n", pgpio_adapter->in_port);
	pgpio_adapter->out0_port = (uint32_t *)pport_odl_en[port_array_idx];
	DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_adapter->out0_port);
	pgpio_adapter->out1_port = (uint32_t *)pport_odh_en[port_array_idx];
	DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_adapter->out1_port);
	pgpio_adapter->outt_port = (uint32_t *)pport_odt_en[port_array_idx];
	DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_adapter->outt_port);
	pgpio_adapter->debounce_idx = 0xFF; // mark as not using debounce

	// 1. Set IRQ priorities and 2. Register IRQ handlers
	if (gpio_type == AonGPIO) {
		// AON GPIO
#if !defined(CONFIG_BUILD_NONSECURE)
		if (!s_ram_aon_gpio_on_local) { // if GPIO RAM has not turned on power and clock for AON GPIO, do it here.
			hal_rtl_sys_peripheral_en(GPIO_AON, ENABLE);
		}
#endif

	} else if (gpio_type == SysonGPIO) {
		// SYSON GPIO
#if !defined(CONFIG_BUILD_NONSECURE)
		if (!s_ram_gpio_on_local) { // if GPIO RAM has not turned on power and clock for SYSON GPIO, do it here.
			hal_rtl_sys_peripheral_en(GPIO_SYS, ENABLE);
		}
#endif

	} else {
		// PON GPIO
#if !defined(CONFIG_BUILD_NONSECURE)
		if (!s_ram_pon_gpio_on_local) { // if GPIO RAM has not turned on power and clock for PON GPIO, do it here.
			hal_rtl_sys_peripheral_en(GPIO_PON, ENABLE);
		}
#endif

	}

	// default configure it as an input pin
	port_idm_en = (uint32_t *)pport_idm_en[port_array_idx];
	*port_idm_en = bit_mask;

	return HAL_OK;
}

/**
 *  @brief To de-initial a GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_deinit(phal_gpio_adapter_t pgpio_adapter)
{
	uint32_t *port_idm_en;
	port_idm_en = (uint32_t *)pport_idm_en[pgpio_adapter->port_idx];

	// switch to input mode
	*port_idm_en = 1 << pgpio_adapter->pin_idx;
}

/**
 *  @brief To set the direction of the given GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *  @param[in]  dir  The direction (IN or OUT).
 *                     - 0: input.
 *                     - 1: output.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_set_dir(phal_gpio_adapter_t pgpio_adapter, gpio_dir_t dir)
{
	uint32_t *port_dir_en;

	if (dir == GPIO_IN) {
		port_dir_en = (uint32_t *)pport_idm_en[pgpio_adapter->port_idx];
	} else {
		port_dir_en = (uint32_t *)pport_odm_en[pgpio_adapter->port_idx];
	}

	*port_dir_en = 1 << pgpio_adapter->pin_idx;
}

/**
 *  @brief Gets current direction of the specified GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns    The GPIO pin direction.
 *                - 0: input.
 *                - 1: output.
 */
SECTION_GPIO_TEXT
gpio_dir_t hal_rtl_gpio_get_dir(phal_gpio_adapter_t pgpio_adapter)
{
	uint32_t *pport_dir_sts;

	pport_dir_sts = (uint32_t *)pport_dmd_sts[pgpio_adapter->port_idx];

	if (*pport_dir_sts & (1 << pgpio_adapter->pin_idx)) {
		return GPIO_OUT;
	} else {
		return GPIO_IN;
	}
}

/**
 *  @brief Sets the output level of the specified GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *  @param[in]  io_data  The value to be set to the GPIO pin, 0 or 1.
 *                - 0: the GPIO pin output low level.
 *                - 1: the GPIO pin output high level.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_write(phal_gpio_adapter_t pgpio_adapter, uint32_t io_data)
{
	if (io_data) {
		*(pgpio_adapter->out1_port) = pgpio_adapter->bit_mask;
		//*((uint32_t *)pport_odh_en[pgpio_adapter->port_idx]) = pgpio_adapter->bit_mask; // test see if need this
	} else {
		//(pgpio_adapter->out0_port) = (uint32_t *)pgpio_adapter->bit_mask; // old
		*(pgpio_adapter->out0_port) = pgpio_adapter->bit_mask;
		//*((uint32_t *)pport_odl_en[pgpio_adapter->port_idx]) = pgpio_adapter->bit_mask; // test see if need this
	}
}

/**
 *  @brief To toggle (0 -> 1 or 1 -> 0) the output level of the specified GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_toggle(phal_gpio_adapter_t pgpio_adapter)
{
	*(pgpio_adapter->outt_port) = pgpio_adapter->bit_mask;
}

/**
 *  @brief Reads the input level of the specified GPIO pin(direction is IN).
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns The gotton level (0 or 1) of the GPIO input pin.
 *             - 0: the GPIO pin input level is low.
 *             - 1: the GPIO pin input level is high.
 */
SECTION_GPIO_TEXT
uint32_t hal_rtl_gpio_read(phal_gpio_adapter_t pgpio_adapter)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

	if (pgpio_adapter->debounce_idx >= GPIO_MAX_DEBOUNCE_PIN) {
		DBG_GPIO_INFO("val in_port:%x\r\n", *((volatile uint32_t *)(pgpio_adapter->in_port)));
		return ((*((volatile uint32_t *)(pgpio_adapter->in_port)) & pgpio_adapter->bit_mask) ? 1 : 0);
	} else {
		DBG_GPIO_INFO("In GPIO HAL ROM Read with Debounce!\r\n");
		if (port_idx == PORT_A) {
			return (AON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);

		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			return (SYSON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);

		} else if (port_idx == PORT_F) {
			return (PON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);

		} else { // weide added to rm control reaches non-void
			DBG_GPIO_INFO("Error in port_idx parameter. port_idx: %x\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}
}

/**
 *  @brief Enable the debounce function for the given GPIO pin.
 *         The debounce resource(circuit) is limited, not all GPIO pin
 *         can has debounce function.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *  @param[in]  debounce_us  The time filter for the debounce, in micro-second.
 *                           But the time resolution is 31.25us (1/32K) and the
 *                           maximum time is 512 ms.
 *
 *  @return     HAL_NO_RESOURCE:  No debounce resource. (All debounce circuit are allocated).
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  Debounce function is enabled on this GPIO.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_debounce_enable(phal_gpio_adapter_t pgpio_adapter, uint32_t debounce_us)
{
	uint32_t i;
	uint32_t bit_mask;
	uint32_t debounce_cyc;
	uint32_t *pdeb_cfg;
	hal_status_t ret = HAL_NO_RESOURCE;

	uint8_t gpio_gp_sel; // GPIO Group Select Signal. For ProII, GPIOA-F and S will be '0', except GPIOE5 (Pin 6) to E10 and GPIOS will be '1'.

	uint8_t pin_idx;
	uint8_t port_idx;

	if (pgpio_adapter == NULL) {
		return HAL_ERR_PARA;
	}

	pin_idx = PIN_NAME_2_PIN(pgpio_adapter->pin_name);
	port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

	switch (port_idx) {
	case PORT_A: // AON GPIO
		gpio_gp_sel = 0; // for INTx SEL reg
		break;
	case PORT_B:
		gpio_gp_sel = 0; // for INTx SEL reg
		break;
	case PORT_C:
		gpio_gp_sel = 0; // for INTx SEL reg
		break;
	case PORT_D:
		gpio_gp_sel = 0; // for INTx SEL reg
		break;
	case PORT_E:
		if (pin_idx > 5) {
			gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B/Port B)
		} else { // means still at Port A [31:26]
			gpio_gp_sel = 0; // for INTx SEL reg
		}
		break;
	case PORT_F: // PON GPIO
		gpio_gp_sel = 0; // for INTx SEL reg
		break;
	case PORT_S: // SYSON GPIO Port B
		gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B/Port B)
		break;
	}


	if (port_idx == PORT_A) {
		if (_paon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_en: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_en: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_F) {
		if (_ppon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_en: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	}

	// check does this GPIO pin works on IN ?
	if ((*(pport_dmd_sts[pgpio_adapter->port_idx]) & (1 << pgpio_adapter->pin_idx)) != 0) {
		// currently work with OUT mode, should not use debounce
		return HAL_ERR_PARA;
	}

	if (port_idx == PORT_A) {
		// to find a free debounce function block first
		for (i = 0; i < AON_GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_paon_gpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_paon_gpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)paon_gpio_debounce_cfg[i];
				// Weide new code
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear before setting new debounce cycle time
				*pdeb_cfg |= (pgpio_adapter->pin_idx) | (gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL | debounce_cyc << GPIO_SHIFT_DEB_CYC;
				AON_GPIO->GPIO_DEB_EN |= bit_mask; // enable this debounce function

				ret = HAL_OK;
				break;
			}
		}

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// to find a free debounce function block first
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_pgpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_pgpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)pgpio_debounce_cfg[i];
				// Weide new code
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear before setting new debounce cycle time
				*pdeb_cfg |= (pgpio_adapter->pin_idx) | (gpio_gp_sel & 0x03) << GPIO_SHIFT_DEB_GP_SEL | debounce_cyc << GPIO_SHIFT_DEB_CYC;
				SYSON_GPIO->GPIO_DEB_EN |= bit_mask; // enable this debounce function

				ret = HAL_OK;
				break;
			}
		}

	} else if (port_idx == PORT_F) {
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_ppon_gpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_ppon_gpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)ppon_gpio_debounce_cfg[i];
				// Weide new code
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear before setting new debounce cycle time
				*pdeb_cfg |= (pgpio_adapter->pin_idx) | (gpio_gp_sel & 0x03) << GPIO_SHIFT_DEB_GP_SEL | debounce_cyc << GPIO_SHIFT_DEB_CYC;
				PON_GPIO->GPIO_DEB_EN |= bit_mask; // enable this debounce function

				ret = HAL_OK;
				break;
			}
		}

	}

	return ret;
}

/**
 *  @brief Reads the given GPIO pin input level with de-bounced.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns The GPIO pin input level after debouncing.
 *             - 0: the GPIO pin input level is low.
 *             - 1: the GPIO pin input level is high.
 */
SECTION_GPIO_TEXT
uint32_t hal_rtl_gpio_read_debounce(phal_gpio_adapter_t pgpio_adapter)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

	if (port_idx == PORT_A) {
		return (AON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		return (SYSON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);
	} else if (port_idx == PORT_F) {
		return (PON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_adapter->debounce_idx) ? 1 : 0);
	} else { // weide added to rm control reaches non-void
		DBG_GPIO_INFO("Error in port_idx parameter. port_idx: %x\r\n", port_idx);
		return HAL_ERR_PARA;
	}

}

/**
 *  @brief Disables the debounce function of the given GPIO pin.
 *         The released debounce circuit can be assign to other GPIO pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_debounce_disable(phal_gpio_adapter_t pgpio_adapter)
{
	uint32_t deb_idx;
	uint32_t bit_mask;

	uint8_t port_idx;

	if (pgpio_adapter == NULL) {
		return;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if (_paon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_dis: AON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_adapter->debounce_idx;
		if (deb_idx >= AON_GPIO_MAX_DEBOUNCE_PIN) { // AON GPIO only max 6 pins
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		AON_GPIO->GPIO_DEB_DIS |= bit_mask;
		_paon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_dis: SYSON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_adapter->debounce_idx;
		if (deb_idx >= GPIO_MAX_DEBOUNCE_PIN) {
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		SYSON_GPIO->GPIO_DEB_DIS |= bit_mask;
		_pgpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if (_ppon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_deb_dis: PON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_adapter->debounce_idx;
		if (deb_idx >= GPIO_MAX_DEBOUNCE_PIN) {
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		PON_GPIO->GPIO_DEB_DIS |= bit_mask;
		_ppon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	}

}

/**
 *  @brief To insert a new GPIO IRQ adapter to the GPIO group
 *         interrupt process list. The GPIO interrupt handler only
 *         process the GPIO IRQ pins in the process list.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter to be added.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void _hal_rtl_gpio_irq_list_add(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	if (pgpio_irq_adapter == NULL) {
		return;
	}

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_rtl_aon_gpio_enter_critical();
		if (_paon_gpio_comm_adp->gpio_irq_list_head == NULL) {
			// the first one
			_paon_gpio_comm_adp->gpio_irq_list_head = pgpio_irq_adapter;
			_paon_gpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_paon_gpio_comm_adp->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_paon_gpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_aon_gpio_exit_critical();

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_rtl_gpio_enter_critical();
		if (_pgpio_comm_adp->gpio_irq_list_head == NULL) {
			// the first one
			_pgpio_comm_adp->gpio_irq_list_head = pgpio_irq_adapter;
			_pgpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_pgpio_comm_adp->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_pgpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_gpio_exit_critical();

	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_rtl_pon_gpio_enter_critical();
		if (_ppon_gpio_comm_adp->gpio_irq_list_head == NULL) {
			// the first one
			_ppon_gpio_comm_adp->gpio_irq_list_head = pgpio_irq_adapter;
			_ppon_gpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_ppon_gpio_comm_adp->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_ppon_gpio_comm_adp->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_pon_gpio_exit_critical();
	}

}

/**
 *  @brief To remove the given GPIO IRQ adapter from the GPIO group
 *         interrupt process list.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter to be removed.
 *
 *  @return     HAL_NOT_FOUND:  The given GPIO IRQ pin adapter not in the GPIO group interrupt process list.
 *  @return     HAL_OK:  The given GPIO IRQ pin adapter is removed.
 */
SECTION_GPIO_TEXT
hal_status_t _hal_rtl_gpio_irq_list_remove(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	// unsure, must recheck
	hal_status_t ret = HAL_NOT_FOUND;
	phal_gpio_irq_adapter_t ptemp;
	phal_gpio_irq_adapter_t pprev;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_rtl_aon_gpio_enter_critical();
		if ((pgpio_irq_adapter == _paon_gpio_comm_adp->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _paon_gpio_comm_adp->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_paon_gpio_comm_adp->gpio_irq_list_head = NULL;
			_paon_gpio_comm_adp->gpio_irq_list_tail = NULL;
			hal_rtl_aon_gpio_exit_critical();
			return HAL_OK;
		} else {
			ptemp = _paon_gpio_comm_adp->gpio_irq_list_head;
			pprev = _paon_gpio_comm_adp->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _paon_gpio_comm_adp->gpio_irq_list_head) {
						_paon_gpio_comm_adp->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _paon_gpio_comm_adp->gpio_irq_list_tail) {
						_paon_gpio_comm_adp->gpio_irq_list_tail = pprev;
						pprev->pnext = NULL;
					} else {
						pprev->pnext = ptemp->pnext;
					}
					ptemp->pnext = NULL;
					ret = HAL_OK;
					break;
				}
				pprev = ptemp;
				ptemp = ptemp->pnext;
			}
		}
		hal_rtl_aon_gpio_exit_critical();

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_rtl_gpio_enter_critical();
		if ((pgpio_irq_adapter == _pgpio_comm_adp->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _pgpio_comm_adp->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_pgpio_comm_adp->gpio_irq_list_head = NULL;
			_pgpio_comm_adp->gpio_irq_list_tail = NULL;
			hal_rtl_gpio_exit_critical();
			return HAL_OK;
		} else {
			ptemp = _pgpio_comm_adp->gpio_irq_list_head;
			pprev = _pgpio_comm_adp->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _pgpio_comm_adp->gpio_irq_list_head) {
						_pgpio_comm_adp->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _pgpio_comm_adp->gpio_irq_list_tail) {
						_pgpio_comm_adp->gpio_irq_list_tail = pprev;
						pprev->pnext = NULL;
					} else {
						pprev->pnext = ptemp->pnext;
					}
					ptemp->pnext = NULL;
					ret = HAL_OK;
					break;
				}
				pprev = ptemp;
				ptemp = ptemp->pnext;
			}
		}
		hal_rtl_gpio_exit_critical();
	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_rtl_pon_gpio_enter_critical();
		if ((pgpio_irq_adapter == _ppon_gpio_comm_adp->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _ppon_gpio_comm_adp->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_ppon_gpio_comm_adp->gpio_irq_list_head = NULL;
			_ppon_gpio_comm_adp->gpio_irq_list_tail = NULL;
			hal_rtl_pon_gpio_exit_critical();
			return HAL_OK;
		} else {
			ptemp = _ppon_gpio_comm_adp->gpio_irq_list_head;
			pprev = _ppon_gpio_comm_adp->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _ppon_gpio_comm_adp->gpio_irq_list_head) {
						_ppon_gpio_comm_adp->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _ppon_gpio_comm_adp->gpio_irq_list_tail) {
						_ppon_gpio_comm_adp->gpio_irq_list_tail = pprev;
						pprev->pnext = NULL;
					} else {
						pprev->pnext = ptemp->pnext;
					}
					ptemp->pnext = NULL;
					ret = HAL_OK;
					break;
				}
				pprev = ptemp;
				ptemp = ptemp->pnext;
			}
		}
		hal_rtl_pon_gpio_exit_critical();
	}
	return ret;
}

/**
 *  @brief Initials a GPIO IRQ pin.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *  @param[in]  pin_name  The GPIO IRQ pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *  @param[in]  callback  The GPIO IRQ call back function.
 *  @param[in]  arg  The argument of the GPIO IRQ call back function. It is an application
 *                   private data to be passed by the call back function.
 *
 *  @return     HAL_NOT_READY:  The GPIO group adapter does not been initialed yet.
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: No GPIO IRQ resource (circuit). The GPIO IRQ resource is limited,
 *                               not all GPIO pin can has GPIO IRQ function.
 *  @return     HAL_OK: GPIO IRQ pin initialization OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_irq_init(phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t pin_name,
								   gpio_irq_callback_t callback, uint32_t arg)
{
	uint32_t i;
	uint32_t *pirq_cfg;
	uint32_t *port_idm_en;
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);

	uint8_t gpio_gp_sel; // GPIO Group Select Signal. For ProII, GPIOA-F and S will be '0', except GPIOE5 (Pin 6) to E10 and GPIOS will be '1'.
	uint8_t array_index; // comment here

	if (port_idx == PORT_A) {
		// AON GPIO
		if (_paon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if (_ppon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	}

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("GPIO IRQ Init Invalid: port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}

	// convert chip pin definition to IP pin definition
	/* Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins - GPIOA0 - A5
	 * Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins - GPIOB0 - B2
	 * Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins - GPIOC0 - C5
	 * Port D: GPIO IP Port2[29:9] // SYSON GPIO 21 pins - GPIOD0 - D20
	 * Port E1: GPIO IP Port2[31:30] // SYSON GPIO 2 pins - GPIOE0 - E1
	 * Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 3 from Port 2) - GPIOE2 - E6
	 * Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins - GPIOF0 - F17
	 * Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 3 from Port 2) - GPIO
	 */
	switch (port_idx) {
	case PORT_A: // AON GPIO
		array_index = 0;
		pin_idx += 0;
		gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		break;
	case PORT_B:
		array_index = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 0;
		gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		break;
	case PORT_C:
		array_index = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 3; // pin starts at 4th position, Pin 3 (Pin 0 is the 1st pin)
		gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		break;
	case PORT_D:
		array_index = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 9; // pin starts at 10th position, Pin 9 (Pin 0 is the 1st pin)
		gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		break;
	case PORT_E:
#if IS_CUT_TEST(CONFIG_CHIP_VER) // Test Chip
		if (pin_idx > 5) {
			array_index = 3;
			pin_idx -= 6; // because pin_idx = 6 will be mapped to Group B[0], pin_idx = 7 mapped to Group B[1], etc
			gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B)
		} else { // means still at Port A [31:26]
			array_index = 2;
			pin_idx += 26;
			gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		}
#else // MP Chip
		if (pin_idx > 1) {
			array_index = 3;
			pin_idx -= 2; // because pin_idx = 2 will be mapped to Group B[0], pin_idx = 3 mapped to Group B[1], etc
			gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B)
		} else { // means still at Port A [31:26]
			array_index = 2;
			pin_idx += 30;
			gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		}
#endif
		break;
	case PORT_F: // PON GPIO
		array_index = 1;
		pin_idx += 0;
		gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		break;
	case PORT_S: // SYSON GPIO Group B
		array_index = 3;
		pin_idx += 5; // pin starts at 6th position, Pin 5 (Pin 0 is the 1st pin)
		gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B)
		break;
	}

	_memset((void *) pgpio_irq_adapter, 0, sizeof(hal_gpio_irq_adapter_t));

	if (port_idx == PORT_A) {

		// find a free AON GPIO IRQ pin // AON_GPIO_MAX_INT_PIN = 6, cos AON GPIO has only 6 pins
		for (i = 0; i < AON_GPIO_MAX_INT_PIN; i++) {
			if ((_paon_gpio_comm_adp->gpio_irq_using & (1 << i)) == 0) {
				// find a free IRQ pin
				_paon_gpio_comm_adp->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as not using debounce

				pirq_cfg = (uint32_t *)paon_gpio_irq_cfg[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg |= pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp
				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en[array_index];
				*port_idm_en |= 1 << pin_idx;
				break;
			}
		}

		if (i >= AON_GPIO_MAX_INT_PIN) {
			// didn't find available AON GPIO IRQ pin
			return HAL_NO_RESOURCE;
		}

		// Add to AON GPIO IRQ adapter list
		pgpio_irq_adapter->irq_callback = callback;
		pgpio_irq_adapter->irq_callback_arg = arg;
		_hal_rtl_gpio_irq_list_add(pgpio_irq_adapter);

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {

		// find a free SYSON GPIO IRQ pin
		for (i = 0; i < GPIO_MAX_INT_PIN; i++) {
			if ((_pgpio_comm_adp->gpio_irq_using & (1 << i)) == 0) {
				// find a free IRQ pin
				_pgpio_comm_adp->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as no using debounce

				pirq_cfg = (uint32_t *)pgpio_irq_cfg[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg = pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp

				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en[array_index];
				*port_idm_en |= 1 << pin_idx;
				break;
			}
		}

		if (i >= GPIO_MAX_INT_PIN) {
			// didn't find available GPIO IRQ pin
			return HAL_NO_RESOURCE;
		}

		// Add to SYSON GPIO IRQ adapter list
		pgpio_irq_adapter->irq_callback = callback;
		pgpio_irq_adapter->irq_callback_arg = arg;
		_hal_rtl_gpio_irq_list_add(pgpio_irq_adapter);

	} else if (port_idx == PORT_F) {

		// find a free PON GPIO IRQ pin
		for (i = 0; i < GPIO_MAX_INT_PIN; i++) {
			if ((_ppon_gpio_comm_adp->gpio_irq_using & (1 << i)) == 0) {
				// find a free GPIO IRQ pin
				_ppon_gpio_comm_adp->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as no using debounce

				pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg = pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp

				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en[array_index];
				*port_idm_en |= 1 << pin_idx;
				break;
			}
		}

		if (i >= GPIO_MAX_INT_PIN) {
			// didn't find available GPIO IRQ pin
			return HAL_NO_RESOURCE;
		}

		// Add to PON GPIO IRQ adapter list
		pgpio_irq_adapter->irq_callback = callback;
		pgpio_irq_adapter->irq_callback_arg = arg;
		_hal_rtl_gpio_irq_list_add(pgpio_irq_adapter);
	}

	return HAL_OK;
}

/**
 *  @brief To de-initial and disable a GPIO IRQ pin.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_irq_deinit(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	uint32_t int_idx;
	uint32_t bit_mask;
	uint8_t port_idx;

	if (pgpio_irq_adapter == NULL) {
		return;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if (_paon_gpio_comm_adp == NULL) {
			return;
		}

		int_idx = pgpio_irq_adapter->int_idx;
		if (int_idx >= AON_GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}

		bit_mask = 1 << int_idx;

		// Disable IRQ function and debounce function
		AON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;

		_paon_gpio_comm_adp->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < AON_GPIO_MAX_DEBOUNCE_PIN) {
			AON_GPIO->GPIO_DEB_DIS = bit_mask;
			_paon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		AON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		AON_GPIO->GPIO_INT_CLR = bit_mask;

		_hal_rtl_gpio_irq_list_remove(pgpio_irq_adapter);

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if (_pgpio_comm_adp == NULL) {
			return;
		}

		int_idx = pgpio_irq_adapter->int_idx;
		if (int_idx >= GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}

		bit_mask = 1 << int_idx;

		// Disable IRQ function and debounce function
		SYSON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;

		_pgpio_comm_adp->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < GPIO_MAX_DEBOUNCE_PIN) {
			SYSON_GPIO->GPIO_DEB_DIS = bit_mask;
			_pgpio_comm_adp->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		SYSON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		SYSON_GPIO->GPIO_INT_CLR = bit_mask;

		_hal_rtl_gpio_irq_list_remove(pgpio_irq_adapter);

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if (_ppon_gpio_comm_adp == NULL) {
			return;
		}

		int_idx = pgpio_irq_adapter->int_idx;
		if (int_idx >= GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}

		bit_mask = 1 << int_idx;

		// Disable IRQ function and debounce function
		PON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;

		_ppon_gpio_comm_adp->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < GPIO_MAX_DEBOUNCE_PIN) {
			PON_GPIO->GPIO_DEB_DIS = bit_mask;
			_ppon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		PON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		PON_GPIO->GPIO_INT_CLR = bit_mask;

		_hal_rtl_gpio_irq_list_remove(pgpio_irq_adapter);

	}

}

/**
 *  @brief Configures the interrupt trigger type of a given GPIO IRQ pin.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *  @param[in]  int_type  The interrupt trigger type.
 *                     - 1: Rising edge trigger.
 *                     - 2: Falling edge trigger.
 *                     - 3: Low level trigger.
 *                     - 4: High level trigger.
 *                     - 5: Dual(rising and falling) edge trigger.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_irq_set_trig_type(phal_gpio_irq_adapter_t pgpio_adapter, gpio_int_trig_type_t int_type)
{
	uint32_t int_idx;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);  // why is pgpio_adapter, not pgpio_irq_adapter?

	if (pgpio_adapter->int_idx >= GPIO_MAX_INT_PIN) {
		DBG_GPIO_ERR("Set INT trigger type: Invalid INT Index 0x%x\r\n", pgpio_adapter->int_idx);
		return;
	}

	int_idx = 1 << pgpio_adapter->int_idx;

	switch (int_type) {
	case GPIO_IntType_LevelHigh:
		if (port_idx == PORT_A) {
			AON_GPIO->GPIO_IR_EN = int_idx;
			AON_GPIO->GPIO_LI_EN = int_idx;
		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			SYSON_GPIO->GPIO_IR_EN = int_idx;
			SYSON_GPIO->GPIO_LI_EN = int_idx;
		} else if (port_idx == PORT_F) {
			PON_GPIO->GPIO_IR_EN = int_idx;
			PON_GPIO->GPIO_LI_EN = int_idx;
		}
		break;

	case GPIO_IntType_LevelLow:
		if (port_idx == PORT_A) {
			AON_GPIO->GPIO_IF_EN = int_idx;
			AON_GPIO->GPIO_LI_EN = int_idx;
		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			SYSON_GPIO->GPIO_IF_EN = int_idx;
			SYSON_GPIO->GPIO_LI_EN = int_idx;
		} else if (port_idx == PORT_F) {
			PON_GPIO->GPIO_IF_EN = int_idx;
			PON_GPIO->GPIO_LI_EN = int_idx;
		}
		break;

	case GPIO_IntType_EdgeRising:
		if (port_idx == PORT_A) {
			AON_GPIO->GPIO_IR_EN = int_idx;
			AON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			SYSON_GPIO->GPIO_IR_EN = int_idx;
			SYSON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_F) {
			PON_GPIO->GPIO_IR_EN = int_idx;
			PON_GPIO->GPIO_EI_EN = int_idx;
		}
		break;

	case GPIO_IntType_EdgeFalling:
		if (port_idx == PORT_A) {
			AON_GPIO->GPIO_IF_EN = int_idx;
			AON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			SYSON_GPIO->GPIO_IF_EN = int_idx;
			SYSON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_F) {
			PON_GPIO->GPIO_IF_EN = int_idx;
			PON_GPIO->GPIO_EI_EN = int_idx;
		}
		break;

	case GPIO_IntType_EdgeDual:
		if (port_idx == PORT_A) {
			AON_GPIO->GPIO_ID_EN = int_idx;
			AON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			SYSON_GPIO->GPIO_ID_EN = int_idx;
			SYSON_GPIO->GPIO_EI_EN = int_idx;
		} else if (port_idx == PORT_F) {
			PON_GPIO->GPIO_ID_EN = int_idx;
			PON_GPIO->GPIO_EI_EN = int_idx;
		}
		break;

	default:
		DBG_GPIO_ERR("Invalid INT trigger type 0x%x\r\n", int_type);
		break;
	}
}

/**
 *  @brief Gets current GPIO interrupt trigger type configuration.
 *
 *  @param[in]  pgpio_adapter  The GPIO pin adapter.
 *
 *  @returns    The interrupt trigger type.
 *                - 1: Rising edge trigger.
 *                - 2: Falling edge trigger.
 *                - 3: Low level trigger.
 *                - 4: High level trigger.
 *                - 5: Dual(rising and falling) edge trigger.
 */
SECTION_GPIO_TEXT
gpio_int_trig_type_t hal_rtl_gpio_irq_get_trig_type(phal_gpio_irq_adapter_t pgpio_adapter)
{

	uint32_t int_idx_mask;
	uint32_t int_idx;
	uint32_t ip_sts;
	gpio_int_trig_type_t trig_type;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);  // why is pgpio_adapter, not pgpio_irq_adapter?

	int_idx = pgpio_adapter->int_idx;
	int_idx_mask = 1 << int_idx;

	if (port_idx == PORT_A) {
		ip_sts = (AON_GPIO->GPIO_IP_STS >> (int_idx << 1)) & 0x03;

		if (AON_GPIO->GPIO_IT_STS & int_idx_mask) {
			// Level trigger
			if (ip_sts == GPIO_LevelHigh) {
				trig_type = GPIO_IntType_LevelHigh;
			} else if (ip_sts == GPIO_LevelLow) {
				trig_type = GPIO_IntType_LevelLow;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		} else {
			// Edge trigger
			if (ip_sts == GPIO_EdgeRising) {
				trig_type = GPIO_IntType_EdgeRising;
			} else if (ip_sts == GPIO_EdgeFalling) {
				trig_type = GPIO_IntType_EdgeFalling;
			} else if (ip_sts == GPIO_EdgeDual) {
				trig_type = GPIO_IntType_EdgeDual;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		}
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		ip_sts = (SYSON_GPIO->GPIO_IP_STS >> (int_idx << 1)) & 0x03;

		if (SYSON_GPIO->GPIO_IT_STS & int_idx_mask) {
			// Level trigger
			if (ip_sts == GPIO_LevelHigh) {
				trig_type = GPIO_IntType_LevelHigh;
			} else if (ip_sts == GPIO_LevelLow) {
				trig_type = GPIO_IntType_LevelLow;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		} else {
			// Edge trigger
			if (ip_sts == GPIO_EdgeRising) {
				trig_type = GPIO_IntType_EdgeRising;
			} else if (ip_sts == GPIO_EdgeFalling) {
				trig_type = GPIO_IntType_EdgeFalling;
			} else if (ip_sts == GPIO_EdgeDual) {
				trig_type = GPIO_IntType_EdgeDual;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		}
	} else {
		ip_sts = (PON_GPIO->GPIO_IP_STS >> (int_idx << 1)) & 0x03;

		if (PON_GPIO->GPIO_IT_STS & int_idx_mask) {
			// Level trigger
			if (ip_sts == GPIO_LevelHigh) {
				trig_type = GPIO_IntType_LevelHigh;
			} else if (ip_sts == GPIO_LevelLow) {
				trig_type = GPIO_IntType_LevelLow;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		} else {
			// Edge trigger
			if (ip_sts == GPIO_EdgeRising) {
				trig_type = GPIO_IntType_EdgeRising;
			} else if (ip_sts == GPIO_EdgeFalling) {
				trig_type = GPIO_IntType_EdgeFalling;
			} else if (ip_sts == GPIO_EdgeDual) {
				trig_type = GPIO_IntType_EdgeDual;
			} else {
				// error here: unknown interrupt trigger polarity
				trig_type = GPIO_IntType_Invalid;
			}
		}
	}

	return trig_type;
}

/**
 *  @brief Enables the interrupt of the given GPIO IRQ pin.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *
 *  @returns void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_irq_enable(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	uint32_t int_idx;
	uint32_t bit_mask;

	uint8_t port_idx;

	if (pgpio_irq_adapter == NULL) {
		return;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	int_idx = pgpio_irq_adapter->int_idx;
	if (port_idx == PORT_A) {
		// AON GPIO - only 6 pins
		if (int_idx >= AON_GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}

	} else {
		// Either PON or SYSON GPIO
		if (int_idx >= GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}
	}

	bit_mask = 1 << int_idx;

	if (port_idx == PORT_A) {
		// clear pending IRQ
		AON_GPIO->GPIO_INT_CLR = bit_mask;
		__DSB();

		// enable IRQ
		AON_GPIO->GPIO_INT_FUNC_EN = bit_mask;
		AON_GPIO->GPIO_INT_EN = bit_mask;

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// clear pending IRQ
		SYSON_GPIO->GPIO_INT_CLR = bit_mask;
		__DSB();

		// enable IRQ
		SYSON_GPIO->GPIO_INT_FUNC_EN = bit_mask;
		SYSON_GPIO->GPIO_INT_EN = bit_mask;

	} else if (port_idx == PORT_F) {
		// clear pending IRQ
		PON_GPIO->GPIO_INT_CLR = bit_mask;
		__DSB();

		// enable IRQ
		PON_GPIO->GPIO_INT_FUNC_EN = bit_mask;
		PON_GPIO->GPIO_INT_EN = bit_mask;
	}

}

/**
 *  @brief Disables the interrupt of the given GPIO IRQ pin.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_irq_disable(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	uint32_t int_idx;
	uint32_t bit_mask;

	uint8_t port_idx;

	if (pgpio_irq_adapter == NULL) {
		return;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	int_idx = pgpio_irq_adapter->int_idx;
	if (port_idx == PORT_A) {
		// AON GPIO - only 6 pins
		if (int_idx >= AON_GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}

	} else {
		// Either PON or SYSON GPIO
		if (int_idx >= GPIO_MAX_INT_PIN) {
			// err: invalid INT function index
			return;
		}
	}

	bit_mask = 1 << int_idx;

	if (port_idx == PORT_A) {
		// disable IRQ
		AON_GPIO->GPIO_INT_DIS = bit_mask;
		AON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;
		__DSB();
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// disable IRQ
		SYSON_GPIO->GPIO_INT_DIS = bit_mask;
		SYSON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;
		__DSB();
	} else if (port_idx == PORT_F) {
		// disable IRQ
		PON_GPIO->GPIO_INT_DIS = bit_mask;
		PON_GPIO->GPIO_INT_FUNC_DIS = bit_mask;
		__DSB();
	}

}

/**
 *  @brief Enables the debounce function of the given GPIO IRQ pin.
 *         The debounce resource(circuit) is limited, not all GPIO pin
 *         can has debounce function.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *  @param[in]  debounce_us  The time filter for the debounce, in micro-second.
 *                           But the time resolution is 31.25us (1/32K) and the
 *                           maximum time is 512 ms.
 *
 *  @return     HAL_NO_RESOURCE:  No debounce resource. (All debounce circuit are allocated).
 *  @return     HAL_OK:  Debounce function is enabled on this GPIO.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_irq_debounce_enable(phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t debounce_us)
{
	uint32_t i;
	uint32_t bit_mask;
	uint32_t debounce_cyc;
	uint32_t *pdeb_cfg;
	uint32_t *pirq_cfg;
	hal_status_t ret = HAL_NO_RESOURCE;
	uint8_t gpio_gp_sel; // GPIO Group Select Signal. For ProII, GPIOA-F and S will be '0', except GPIOE5 (Pin 6) to E10 and GPIOS will be '1'.

	uint8_t port_idx;
	uint8_t pin_idx;

	if (pgpio_irq_adapter == NULL) {
		return HAL_ERR_PARA;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);
	pin_idx = PIN_NAME_2_PIN(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if (_paon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
		gpio_gp_sel = 0; // for DEB SEL reg

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E) {
		// SYSON GPIO
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
		if ((port_idx == PORT_E) && (pin_idx > 5)) {
			gpio_gp_sel = 1; // for DEB SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B/Port B) - ONLY FOR GPIOE (PORT_E)

		} else { // means still at Port A [31:26]
			gpio_gp_sel = 0; // for DEB SEL reg

		}

	} else if (port_idx == PORT_S) {
		// SYSON GPIO
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

		gpio_gp_sel = 1; // for DEB SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B/Port B) - THIS IS GPIOS

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if (_ppon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
		gpio_gp_sel = 0; // for DEB SEL reg
	}

	if (port_idx == PORT_A) {
		// AON GPIO
		// to find a free debounce function block first
		for (i = 0; i < AON_GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_paon_gpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_paon_gpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)paon_gpio_debounce_cfg[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC)); // old

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((
								  debounce_cyc) << GPIO_SHIFT_DEB_CYC));  // new
				AON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)paon_gpio_irq_cfg[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_pgpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_pgpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)pgpio_debounce_cfg[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));
				SYSON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)pgpio_irq_cfg[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}
	} else if (port_idx == PORT_F) {
		// PON GPIO
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_ppon_gpio_comm_adp->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_ppon_gpio_comm_adp->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)ppon_gpio_debounce_cfg[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));
				PON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}
	}

	return ret;
}

/**
 *  @brief To disable the debounce function of the given GPIO IRQ pin.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_irq_debounce_disable(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	uint32_t int_idx;
	uint32_t deb_idx;
	uint32_t bit_mask;
	uint32_t *pirq_cfg;

	uint8_t port_idx;

	if (pgpio_irq_adapter == NULL) {
		return;
	}

	port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if (_paon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_dis: AON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_irq_adapter->debounce_idx;
		if (deb_idx >= AON_GPIO_MAX_DEBOUNCE_PIN) {
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// configure GPIO IRQ to GPIO pin input as interrupt trigger source directly
		int_idx = pgpio_irq_adapter->int_idx;
		bit_mask = 1 << int_idx;
		pirq_cfg = (uint32_t *)paon_gpio_irq_cfg[int_idx];


		if (AON_GPIO->GPIO_IE_STS & bit_mask) {
			// interrupt is enabled, it should be disable and re-enable
			AON_GPIO->GPIO_INT_DIS = bit_mask;
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL); // Clear the bit, int_sur_sel
			AON_GPIO->GPIO_INT_EN = bit_mask;
		} else {
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL);
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		AON_GPIO->GPIO_DEB_DIS = bit_mask;    // disable this debounce function
		_paon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		if (_pgpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_dis: SYSON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_irq_adapter->debounce_idx;
		if (deb_idx >= GPIO_MAX_DEBOUNCE_PIN) {
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// configure GPIO IRQ to GPIO pin input as interrupt trigger source directly
		int_idx = pgpio_irq_adapter->int_idx;
		bit_mask = 1 << int_idx;
		pirq_cfg = (uint32_t *)pgpio_irq_cfg[int_idx];


		if (SYSON_GPIO->GPIO_IE_STS & bit_mask) {
			// interrupt is enabled, it should be disable and re-enable
			SYSON_GPIO->GPIO_INT_DIS = bit_mask;
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL); // Clear the bit, int_sur_sel
			SYSON_GPIO->GPIO_INT_EN = bit_mask;
		} else {
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL);
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		SYSON_GPIO->GPIO_DEB_DIS = bit_mask;    // disable this debounce function
		_pgpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_F) {
		if (_ppon_gpio_comm_adp == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_dis: PON GPIO Common not initialized\r\n");
			return;
		}

		deb_idx = pgpio_irq_adapter->debounce_idx;
		if (deb_idx >= GPIO_MAX_DEBOUNCE_PIN) {
			// invalid debounce function index or didn't assign a debounce function
			return;
		}

		// configure GPIO IRQ to GPIO pin input as interrupt trigger source directly
		int_idx = pgpio_irq_adapter->int_idx;
		bit_mask = 1 << int_idx;
		pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg[int_idx];


		if (PON_GPIO->GPIO_IE_STS & bit_mask) {
			// interrupt is enabled, it should be disable and re-enable
			PON_GPIO->GPIO_INT_DIS = bit_mask;
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL); // Clear the bit, int_sur_sel
			PON_GPIO->GPIO_INT_EN = bit_mask;
		} else {
			*pirq_cfg &= ~(1 << GPIO_SHIFT_INT_SUR_SEL);
		}

		// disable the debounce function
		bit_mask = 1 << deb_idx;
		PON_GPIO->GPIO_DEB_DIS = bit_mask;    // disable this debounce function
		_ppon_gpio_comm_adp->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;
	}

}

/**
 *  @brief Reads the input level of the given GPIO IRQ pin.
 *         If a debounce function is enabled on this GPIO IRQ pin,
 *         then reads the de-bounced input level.
 *
 *  @param[in]  pgpio_irq_adapter  The GPIO IRQ pin adapter.
 *
 *  @returns    The input level(0 or 1) of the GPIO IRQ pin.
 *                - 0: the GPIO IRQ pin input level is low.
 *                - 1: the GPIO IRQ pin input level is high.
 */
SECTION_GPIO_TEXT
uint32_t hal_rtl_gpio_irq_read(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (pgpio_irq_adapter->debounce_idx >= GPIO_MAX_DEBOUNCE_PIN) {
		return (*((volatile uint32_t *)(pgpio_irq_adapter->in_port)) & pgpio_irq_adapter->bit_mask ? 1 : 0);
	} else {
		DBG_GPIO_INFO("In GPIO HAL ROM IRQ Read with Debounce!\r\n");
		if (port_idx == PORT_A) {
			return (AON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_irq_adapter->debounce_idx) ? 1 : 0);
		}

		else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
			return (SYSON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_irq_adapter->debounce_idx) ? 1 : 0);

		} else if (port_idx == PORT_F) {
			return (PON_GPIO->GPIO_DEB_DP_STS & (1 << pgpio_irq_adapter->debounce_idx) ? 1 : 0);

		} else { // weide added to rm control reaches non-void
			DBG_GPIO_INFO("Error in port_idx parameter. port_idx: %x\r\n", port_idx);
			return HAL_ERR_PARA;
		}
	}
}

/**
 *  @brief Initials a GPIO port. A GPIO port has 32 GPIO pins. However, may not all
 *         GPIO pin will be bound out. Checks the IC package to know what pins are
 *         available on this GPIO port.
 *
 *  @param[in]  pgpio_port_adapter  The GPIO port adapter.
 *  @param[in]  port_idx The GPIO port index.
 *                - 0: Port A
 *                - 1: Port B
 *                - 2: Port C
 *                - 3: Port D
 *                - 4: Port E1
 *                - 5: Port E2
 *                - 6: Port F
 *                - 7: Port S
 *  @param[in]  mask  The GPIO pin mask to indicate what pin are included in this port.
 *                    Bit 0 maps to pin 0; bit 1 maps to pin 1; ... etc. The bit value 1
 *                    means the maped pin is included in this GPIO port.
 *  @param[in]  dir  The GPIO port direction, IN or OUT.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GPIO port initialization OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_port_init(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t port_id,
									uint32_t mask, gpio_dir_t dir)
{
	uint32_t *port_dir_en;
	uint32_t port_idx;
	gpio_port_t *pip_port;

	_memset((void *) pgpio_port_adapter, 0, sizeof(hal_gpio_port_adapter_t));

	// Handle power and clock for GPIO Groups
	if (port_id == 0) { // AON GPIO (Port A)
#if !defined(CONFIG_BUILD_NONSECURE)
		hal_rtl_sys_peripheral_en(GPIO_AON, ENABLE);
#endif

	} else if (port_id == 6) { // PON GPIO (Port F)
#if !defined(CONFIG_BUILD_NONSECURE)
		hal_rtl_sys_peripheral_en(GPIO_PON, ENABLE);
#endif

	} else { // SYSON GPIO - No need to handle port_id exceed total port number - already handled above
#if !defined(CONFIG_BUILD_NONSECURE)
		hal_rtl_sys_peripheral_en(GPIO_SYS, ENABLE);
#endif
	}

	if (port_id == PORT_E) {

		pip_port = (gpio_port_t *)&chip_port_2_ip_port[port_id];
		DBG_GPIO_INFO("pip_port <= 0x3: %x\r\n", *pip_port);
		port_idx = pip_port->port_name_b.port;

		pgpio_port_adapter->port_idx = port_idx;
		DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
		pgpio_port_adapter->pin_offset = pip_port->port_name_b.offset;
		DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);
		pgpio_port_adapter->pin_mask = mask & pip_port->port_name_b.mask;
		DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);
		pgpio_port_adapter->in_port = (uint32_t *)pport_dp_sts[port_idx];
		DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port);
		pgpio_port_adapter->out0_port = (uint32_t *)pport_odl_en[port_idx];
		DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port);
		pgpio_port_adapter->out1_port = (uint32_t *)pport_odh_en[port_idx];
		DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port);
		pgpio_port_adapter->outt_port = (uint32_t *)pport_odt_en[port_idx];
		DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port);

		// set direction
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en[port_idx];
		} else {
			port_dir_en = (uint32_t *)pport_odm_en[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pip_port->port_name_b.offset;

		if (mask > 0x3) { // in GPIO Group B (overflowed from Group A to Group B)

			pip_port = (gpio_port_t *)&chip_port_2_ip_port[port_id + 1]; // internal port
			DBG_GPIO_INFO("pip_port > 0x3F: %x\r\n", *pip_port);

			port_idx = pip_port->port_name_b.port;
			DBG_GPIO_INFO("port_idx in overflow: %x\r\n", port_idx);

			pgpio_port_adapter->port_idx = port_idx; // don't store new port_idx val from overflow
			DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx); // don't store new port_idx val from overflow
			pgpio_port_adapter->pin_offset_PORTE = pip_port->port_name_b.offset;
			DBG_GPIO_INFO("pin_offset_PORTE: %x\r\n", pgpio_port_adapter->pin_offset_PORTE);
			pgpio_port_adapter->pin_mask |= (((mask >> 2) & pip_port->port_name_b.mask) << 2);
			DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);

			pgpio_port_adapter->in_port_PORTE = (uint32_t *)pport_dp_sts[port_idx];
			DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port_PORTE);
			pgpio_port_adapter->out0_port_PORTE = (uint32_t *)pport_odl_en[port_idx];
			DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port_PORTE);
			pgpio_port_adapter->out1_port_PORTE = (uint32_t *)pport_odh_en[port_idx];
			DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port_PORTE);
			pgpio_port_adapter->outt_port_PORTE = (uint32_t *)pport_odt_en[port_idx];
			DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port_PORTE);

			// set direction
			if (dir == GPIO_IN) {
				port_dir_en = (uint32_t *)pport_idm_en[port_idx];
			} else {
				port_dir_en = (uint32_t *)pport_odm_en[port_idx];
			}

			*port_dir_en = ((pgpio_port_adapter->pin_mask >> 2) << pip_port->port_name_b.offset);
		}

	} else {

		pip_port = (gpio_port_t *)&chip_port_2_ip_port[port_id];
		port_idx = pip_port->port_name_b.port;

		pgpio_port_adapter->port_idx = port_idx;
		DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
		pgpio_port_adapter->pin_offset = pip_port->port_name_b.offset;
		DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);
		pgpio_port_adapter->pin_mask = mask & pip_port->port_name_b.mask;
		DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);
		pgpio_port_adapter->in_port = (uint32_t *)pport_dp_sts[port_idx];
		DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port);
		pgpio_port_adapter->out0_port = (uint32_t *)pport_odl_en[port_idx];
		DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port);
		pgpio_port_adapter->out1_port = (uint32_t *)pport_odh_en[port_idx];
		DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port);
		pgpio_port_adapter->outt_port = (uint32_t *)pport_odt_en[port_idx];
		DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port);

		// set direction
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en[port_idx];
		} else {
			port_dir_en = (uint32_t *)pport_odm_en[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pip_port->port_name_b.offset;
	}

	return HAL_OK;
}

/**
 *  @brief To de-initial a GPIO port. All pins in this GPIO port will be switched
 *         as input pin.
 *
 *  @param[in]  pgpio_port_adapter The GPIO port adapter.
 *
 *  @returns    void
 */
SECTION_GPIO_TEXT
void hal_rtl_gpio_port_deinit(phal_gpio_port_adapter_t pgpio_port_adapter)
{
	uint32_t *port_idm_en;

	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 30)) {

		DBG_GPIO_INFO("GPIO E overflowed; handling... (deinit)\r\n");
		port_idm_en = (uint32_t *)pport_idm_en[pgpio_port_adapter->port_idx];

		// First, handle the overflow case - GPIO Port "E2"; incoming port_idx will be 3 if GPIO E overflowed in port_init
		// switch to input mode
		*port_idm_en = ((pgpio_port_adapter->pin_mask >> 2) << pgpio_port_adapter->pin_offset_PORTE);

		// Then we handle GPIO E without overflow case, just in case the first 2 bits of GPIO E have something
		DBG_GPIO_INFO("In GPIO E overflowed; handling no overflow case...(deinit)\r\n");
		port_idm_en = (uint32_t *)pport_idm_en[pgpio_port_adapter->port_idx - 1];

		// switch to input mode
		*port_idm_en = ((pgpio_port_adapter->pin_mask & 0x3) << pgpio_port_adapter->pin_offset);

	} else {
		port_idm_en = (uint32_t *)pport_idm_en[pgpio_port_adapter->port_idx];

		// switch to input mode
		*port_idm_en = pgpio_port_adapter->pin_mask << pgpio_port_adapter->pin_offset;
	}
}

/**
 *  @brief Writes a value to the given GPIO output port.
 *
 *  @param[in]  pgpio_port_adapter  The GPIO port adapter.
 *  @param[in]  io_data  The value to be write to the GPIO port.
 *
 *  @returns    void
 */
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
SECTION_GPIO_TEXT
void hal_rtl_gpio_port_write(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t io_data)
{
	uint32_t port_value;

	DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
	DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);

	// write bits of out 1 (means write to output data high register)
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 30)) { // GPIO Port "E2"

		DBG_GPIO_INFO("GPIO E overflowed 1 in hal_rtl_gpio_port_write; handling...\r\n");
		if (pgpio_port_adapter->pin_mask > 0x3) {
			// First take care of the part that overflowed to Group B GPIO, or in other words, it went into Port "E2"
			port_value = (((io_data & pgpio_port_adapter->pin_mask) >> 2) << (pgpio_port_adapter->pin_offset_PORTE));
			DBG_GPIO_INFO("port_val for overflow case 1: %x\r\n", port_value);
			*(pgpio_port_adapter->out1_port_PORTE) = port_value;

			// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
			// ** Do "pgpio_port_adapter->pin_mask & 0x3", to extract the mask for the part which didn't overflow **
			port_value = ((io_data & (pgpio_port_adapter->pin_mask & 0x3)) << (pgpio_port_adapter->pin_offset));
			//DBG_GPIO_INFO("port_val: %x\r\n", port_value);
			DBG_GPIO_INFO("port_val no overflow case 1: %x\r\n", port_value);
			*(pgpio_port_adapter->out1_port) = port_value;

		}
	} else { // treat as normal case, for all other GPIO ports including GPIO E without overflow, excluding GPIO E with overflow
		port_value = ((io_data & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
		DBG_GPIO_INFO("port_val normal ports 1: %x\r\n", port_value);
		*(pgpio_port_adapter->out1_port) = port_value;
	}

	// write bits of out 0 (means write to output data low register)
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 30)) { // GPIO Port "E2"
		DBG_GPIO_INFO("GPIO E overflowed 0 in hal_rtl_gpio_port_write; handling...\r\n");
		// look at pin_mask
		if (pgpio_port_adapter->pin_mask > 0x3) {
			// First take care of the part that overflowed to Group B GPIO, or it went into Port "E2"
			port_value = ((((~io_data) & pgpio_port_adapter->pin_mask) >> 2) << (pgpio_port_adapter->pin_offset_PORTE));
			DBG_GPIO_INFO("port_val for overflow case 0: %x\r\n", port_value);
			*(pgpio_port_adapter->out0_port_PORTE) = port_value;

			// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
			// ** Do "pgpio_port_adapter->pin_mask & 0x3", to extract the mask for the part which didn't overflow **
			port_value = (((~io_data) & (pgpio_port_adapter->pin_mask & 0x3)) << (pgpio_port_adapter->pin_offset));
			DBG_GPIO_INFO("port_val no overflow case 0: %x\r\n", port_value);
			*(pgpio_port_adapter->out0_port) = port_value;

		}
	} else { // treat as normal case, for all other GPIO ports including GPIO E without overflow, excluding GPIO E with overflow
		port_value = (((~io_data) & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
		DBG_GPIO_INFO("port_val normal ports 0: %x\r\n", port_value);
		*(pgpio_port_adapter->out0_port) = port_value;
	}

}
#else // Test chip - just a dummy here, Test chip will call patch code in rtl8735b_gpio_patch.c
SECTION_GPIO_TEXT
void hal_rtl_gpio_port_write(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, uint32_t io_data)
{
	uint32_t port_value;

#if 1 // weide mod
	// write bits of out 1
	port_value = io_data & pgpio_port_adapter->pin_mask; // weide added so that pin_mask (mask) won't be stuck at its value given in port_init
	*(pgpio_port_adapter->out1_port) = port_value << pgpio_port_adapter->pin_offset;
#else
	port_value = ((io_data & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
	DBG_GPIO_INFO("port_val: %x\r\n", port_value);
	*(pgpio_port_adapter->out1_port) = port_value;
#endif

#if 1
	// write bits of out 0
	port_value = (~io_data) & pgpio_port_adapter->pin_mask; // weide added so that pin_mask (mask) won't be stuck at its value given in port_init
	*(pgpio_port_adapter->out0_port) = port_value << pgpio_port_adapter->pin_offset;
#else
	port_value = (((~io_data) & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
	DBG_GPIO_INFO("port_val: %x\r\n", port_value);
	*(pgpio_port_adapter->out0_port) = port_value;
#endif

}
#endif


/**
 *  @brief Read the GPIO port.
 *
 *  @param[in]  pgpio_port_adapter  The GPIO port adapter.
 *
 *  @returns    The level status of the GPIO port.
 */

#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
SECTION_GPIO_TEXT
uint32_t hal_rtl_gpio_port_read(phal_gpio_port_adapter_t pgpio_port_adapter)
{
	uint32_t read_value_portE_overflow;
	uint32_t read_value_portE;

	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 30)) { // GPIO Port "E2"

		DBG_GPIO_INFO("GPIO E overflowed in hal_rtl_gpio_port_read; handling... (read)\r\n");
		// First take care of the part that overflowed to Group B GPIO, or it went into Port "E2"
		read_value_portE_overflow = (*((volatile uint32_t *)(pgpio_port_adapter->in_port_PORTE)) & (pgpio_port_adapter->pin_mask >> 2));
		DBG_GPIO_INFO("read_value for overflow case (read): %x\r\n", read_value_portE_overflow);

		// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
		// ** Do "pgpio_port_adapter->pin_mask & 0x3", to extract the mask for the part which didn't overflow **
		read_value_portE = ((*((volatile uint32_t *)(pgpio_port_adapter->in_port)) >> pgpio_port_adapter->pin_offset) & (pgpio_port_adapter->pin_mask & 0x3));
		DBG_GPIO_INFO("read_value Port E no overflow (read): %x\r\n", read_value_portE);
		read_value_portE = (read_value_portE) | (read_value_portE_overflow << 2);
		DBG_GPIO_INFO("Final read_value (read): %x\r\n", read_value_portE);

		return read_value_portE;

	} else { // all other ports, including PORT_E without overflow, exlcuding PORT_E with overflow
		return ((*((volatile uint32_t *)(pgpio_port_adapter->in_port)) >> pgpio_port_adapter->pin_offset) & pgpio_port_adapter->pin_mask);
	}
}
#else // Test chip - just a dummy here, Test chip will call patch code in rtl8735b_gpio_patch.c
SECTION_GPIO_TEXT
uint32_t hal_rtl_gpio_port_read(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask)
{
	pgpio_port_adapter->pin_mask = mask; // weide added so that pin_mask (mask) won't be stuck at its value given in port_init
	return ((*((volatile uint32_t *)(pgpio_port_adapter->in_port)) >> pgpio_port_adapter->pin_offset) & pgpio_port_adapter->pin_mask);
}
#endif


/**
 *  @brief To set the direction of the given GPIO port.
 *
 *  @param[in]  pgpio_port_adapter The GPIO port adapter.
 *  @param[in]  dir  The GPIO port direction, IN or OUT.
 *                     -0: input.
 *                     -1: output.
 *
 *  @returns    void
 */
#if !IS_CUT_TEST(CONFIG_CHIP_VER) // MP chip
SECTION_GPIO_TEXT
void hal_rtl_gpio_port_dir(phal_gpio_port_adapter_t pgpio_port_adapter, gpio_dir_t dir)
{
	uint32_t *port_dir_en;
	uint32_t port_idx;

	port_idx = pgpio_port_adapter->port_idx;
	DBG_GPIO_INFO("port_idx in port_dir: %x\r\n", port_idx);
	if (port_idx >= GPIO_MAX_PORT_NUM) {
		DBG_GPIO_ERR("hal_gpio_port_dir: Invalid port_idx=%u\r\n", port_idx);
		return;
	}

	// set direction
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 30)) { // GPIO Port "E2"
		DBG_GPIO_INFO("GPIO E overflowed in hal_rtl_gpio_port_dir; handling...\r\n");
		// First, handle the overflow case - GPIO Port "E2"; incoming port_idx will be 3 if GPIO E overflowed in port_init
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en[port_idx];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en[port_idx];
		}

		*port_dir_en = ((pgpio_port_adapter->pin_mask >> 2) << pgpio_port_adapter->pin_offset_PORTE);

		// Then we handle GPIO E without overflow case, just in case the first 2 bits of GPIO E have something
		DBG_GPIO_INFO("In GPIO E overflowed; handling no overflow case...\r\n");
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en[port_idx - 1];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en[port_idx - 1];
		}

		*port_dir_en = ((pgpio_port_adapter->pin_mask & 0x3) << pgpio_port_adapter->pin_offset);

	} else {

		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en[port_idx];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pgpio_port_adapter->pin_offset;
	}
}
#else // Test chip - just a dummy here, Test chip will call patch code in rtl8735b_gpio_patch.c
SECTION_GPIO_TEXT
void hal_rtl_gpio_port_dir(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, gpio_dir_t dir)
{
	uint32_t *port_dir_en;
	uint32_t port_idx;

	port_idx = pgpio_port_adapter->port_idx;
	if (port_idx >= GPIO_MAX_PORT_NUM) {
		DBG_GPIO_ERR("hal_gpio_port_dir: Invalid port_idx=%u\r\n", port_idx);
		return;
	}

	// set direction
	if (dir == GPIO_IN) {
		port_dir_en = (uint32_t *)pport_idm_en[port_idx];

	} else {
		port_dir_en = (uint32_t *)pport_odm_en[port_idx];
	}

	pgpio_port_adapter->pin_mask = mask; // weide added so that pin_mask (mask) won't be stuck at its value given in port_init

	*port_dir_en = pgpio_port_adapter->pin_mask << pgpio_port_adapter->pin_offset;
}

#endif

/**
 *  @brief Configures the pull type of the given GPIO pin.
 *
 *  @param[in]  pin_name  The GPIO pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *  @param[in]  pull_type  The pull type:
 *                - 0: No pull.
 *                - 1: Pull high.
 *                - 2: Pull low.
 *                - other value: No pull.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  Configures IO pad pull type OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_pull_ctrl(uint32_t pin_name, pin_pull_type_t pull_type)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("hal_gpio_pull_ctrl: Invalid pin, port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}

	switch (pull_type) {
	case Pin_PullUp:
		pull_ctrl = GPIO_REG_PullCtrl_SetHigh;
		break;

	case Pin_PullDown:
		pull_ctrl = GPIO_REG_PullCtrl_SetLow;
		break;

	case Pin_PullNone:
		pull_ctrl = GPIO_REG_PullCtrl_Set_HighZ;
		break;

	default:
		DBG_GPIO_ERR("hal_gpio_pull_ctrl: Unknown pull type(%x), set as Pull-None\r\n", pull_type);
		pull_ctrl = GPIO_REG_PullCtrl_Set_HighZ;
		break;
	}

	/* Weide customized for AON GPIO
	 pull_ctrl will first be a simple value, then it will be shifted to become a mask.
	 Specific bits will be cleared in case user wants to change from PullUp to PullDown
	 OR operator is used at last to set the specific bits */
	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1]))) &= (~AON_MASK_AON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_a_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << AON_SHIFT_AON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_a_ctrl_list[(pin_idx >> 1)]))) &= (~AON_MASK_AON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_a_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << AON_SHIFT_AON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_b_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_b_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_b_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_b_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_c_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_c_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_c_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_c_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_d_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_d_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_d_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_d_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_e_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_e_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_e_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_e_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_f_ctrl_list[(pin_idx >> 1)]))) &= (~PON_MASK_PON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_f_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << PON_SHIFT_PON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_f_ctrl_list[(pin_idx >> 1)]))) &= (~PON_MASK_PON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_f_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << PON_SHIFT_PON_GPIO0_PULL_CTRL;
		}
		break;
	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			(*((uint32_t *)(gpio_s_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
			(*((uint32_t *)(gpio_s_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL;
		} else {
			// pin index is even
			(*((uint32_t *)(gpio_s_ctrl_list[(pin_idx >> 1)]))) &= (~SYSON_MASK_SYSON_GPIO0_PULL_CTRL);
			(*((uint32_t *)(gpio_s_ctrl_list[(pin_idx >> 1)]))) |= pull_ctrl << SYSON_SHIFT_SYSON_GPIO0_PULL_CTRL;
		}
		break;
	default:
		break;
	}

	return HAL_OK;
}

/**
 *  @brief The schmitt trigger on/off control on the given GPIO pin .
 *
 *  @param[in]  pin_name  The GPIO pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *  @param[in]  ctrl  The on/off control:
 *                      - 0: disable the schmitt trigger.
 *                      - 1: enable the schmitt trigger.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  Schmitt trigger enable control setup OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_schmitt_ctrl(uint32_t pin_name, BOOLEAN ctrl)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("hal_gpio_schmitt_ctrl: Invalid pin, port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}
#if 0 // old code; transition phase
	if (ctrl) { // Schmitt Trigger ON

		if (!(pin_idx % 2)) { // pin number (pin_idx) is even
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) |= AON_BIT_AON_GPIO0_SMT_EN;
		} else { // pin number (pin_idx) is odd
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) |= AON_BIT_AON_GPIO1_SMT_EN;
		}
	}

	else { // Schmitt Trigger OFF

		if (!(pin_idx % 2)) { // pin number (pin_idx) is even
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) &= ~(AON_BIT_AON_GPIO0_SMT_EN);
		} else { // pin number (pin_idx) is odd
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) &= ~(AON_BIT_AON_GPIO1_SMT_EN);
		}
	}

#else // new; transition phase
	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_BIT_AON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_BIT_AON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_BIT_AON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_BIT_AON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_BIT_PON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_BIT_PON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_BIT_PON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_BIT_PON_GPIO0_SMT_EN);
			}
		}
		break;

	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SMT_EN);
			}
		} else {
			// pin index is even
			if (ctrl) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SMT_EN;
			} else { // Schmitt Trigger OFF
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SMT_EN);
			}
		}
		break;
	default:
		break;
	}
#endif

	return HAL_OK;
}

/**
 *  @brief Controls the IO pad drive power current on the given GPIO pin .
 *
 *  @param[in]  pin_name  The GPIO pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *  @param[in]  drv  The I/O pad driving power selection:
                    0: 4mA
                    1: 8mA
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  IO pad driving current setup OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_drive_ctrl(uint32_t pin_name, uint8_t drv)
{

	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("hal_gpio_schmitt_ctrl: Invalid pin, port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}
#if 0 // old working code
	if (drv) { // Set to 8mA
		if (!(pin_idx % 2)) { // pin number is even
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) |= (drv << AON_SHIFT_AON_GPIO0_DRIVING); // set both bits[11:10]
		} else { // pin number is odd
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) |= (drv << AON_SHIFT_AON_GPIO1_DRIVING); // set both bits[27:26]
		}
	} else { // Set to 4mA
		if (!(pin_idx % 2)) { // pin number is even
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) &= ~(drv << AON_SHIFT_AON_GPIO0_DRIVING); // clear both bits[11:10]
		} else { // pin number is odd
			*((uint32_t *)(pin_pull_ctrl_cum_schmitt_ctrl_reg[pin_idx])) &= ~(drv << AON_SHIFT_AON_GPIO1_DRIVING); // clear both bits[27:26]
		}
	}
#else
	// new transition code
	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_MASK_AON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_MASK_AON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_MASK_AON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_MASK_AON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO1_DRIVING;
			} else {// Set to 4mA
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_MASK_PON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_MASK_PON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_MASK_PON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_MASK_PON_GPIO0_DRIVING);
			}
		}
		break;

	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO1_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO1_DRIVING);
			}
		} else {
			// pin index is even
			if (drv) { // Set to 8mA
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_MASK_SYSON_GPIO0_DRIVING;
			} else { // Set to 4mA
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_MASK_SYSON_GPIO0_DRIVING);
			}
		}
		break;
	default:
		break;
	}

	return HAL_OK;
#endif
}

/**
 *  @brief Controls the slew rate function enable/disable on the given GPIO pin .
 *
 *  @param[in]  pin_name  The GPIO pin.
 *                - bit[7:5]: the GPIO port number. Each port has 32 GPIO pins.
 *                - bit[4:0]: the pin number of the GPIO port.
 *  @param[in]  slew_rate_func  The slew rate function enable selection:
                    0: Disable
                    1: Enable
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  Slew rate function setup OK.
 */
SECTION_GPIO_TEXT
hal_status_t hal_rtl_gpio_slew_rate_ctrl(uint32_t pin_name, uint8_t slew_rate_func)
{

	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("hal_gpio_slew_rate_ctrl: Invalid pin, port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}

	switch (port_idx) {
	case PORT_A:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_BIT_AON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_BIT_AON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) |= AON_BIT_AON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_a_ctrl_list[pin_idx >> 1])) &= ~(AON_BIT_AON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_B:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_b_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_C:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SLEW_RATE;
			} else {// Slew rate function DISABLE
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_c_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_D:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_d_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_E:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_e_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_F:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_BIT_PON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_BIT_PON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Schmitt Trigger ON
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) |= PON_BIT_PON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_f_ctrl_list[pin_idx >> 1])) &= ~(PON_BIT_PON_GPIO0_SLEW_RATE);
			}
		}
		break;

	case PORT_S:
		if (pin_idx % 2) {
			// pin index is odd
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO1_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO1_SLEW_RATE);
			}
		} else {
			// pin index is even
			if (slew_rate_func) { // Slew rate function ENABLE
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) |= SYSON_BIT_SYSON_GPIO0_SLEW_RATE;
			} else { // Slew rate function DISABLE
				*((uint32_t *)(gpio_s_ctrl_list[pin_idx >> 1])) &= ~(SYSON_BIT_SYSON_GPIO0_SLEW_RATE);
			}
		}
		break;
	default:
		break;
	}

	return HAL_OK;

}


/** @} */ /* End of group hs_hal_gpio_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_gpio */

#endif  // #if CONFIG_GPIO_EN

