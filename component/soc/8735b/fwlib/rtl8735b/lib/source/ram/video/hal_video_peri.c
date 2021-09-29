/**************************************************************************//**
 * @file     hal_video_peri.c
 * @brief    This file implements the Video HAL functions.
 *
 * @version  V1.00
 * @date     2021-01-14
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
//#include <rtsavisp.h>
#include "hal_sys_ctrl.h"
#include "hal_sys_ctrl_nsc.h"



#if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9
#include "cmsis.h"
#else
#include "cmsis_voe.h"
#include "voe.h"
#endif

//#include "hal_i2c_simple.h"
#include "hal_pinmux.h"

/** \brief Description of hal_i2c_pin_register
 *
 *	  hal_i2c_pin_register is used for i2c pinmux initialization and register.
 *
 *	 \param[in] hal_i2c_adapter_t *phal_i2c_adapter:		Pointer to I2C control adaptor.
 *	 \return hal_status_t:		  hal_status_t.
 */
hal_status_t hal_i2c_pin_register_simple(volatile hal_i2c_adapter_t *phal_i2c_adapter)
{
	hal_status_t ret;

	hal_sys_peripheral_en(I2C0_SYS + phal_i2c_adapter->init_dat.index, 1);

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.scl_pin) != (uint8_t)0xFF) {
		if (phal_i2c_adapter->init_dat.index == 3) {
			ret = hal_pinmux_register(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C3);
		} else {
			ret = hal_pinmux_register(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.sda_pin) != (uint8_t)0xFF) {

		if (phal_i2c_adapter->init_dat.index == 3) {
			ret |= hal_pinmux_register(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C3);
		} else {
			ret |= hal_pinmux_register(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	return ret;
}

/** \brief Description of hal_i2c_pin_unregister
 *
 *	  hal_i2c_pin_unregister is used for adc pinmux un-initialization and un-register.
 *
 *	 \param[in] hal_i2c_adapter_t *phal_i2c_adapter:		Pointer to I2C control adaptor.
 *	 \return hal_status_t:		  hal_status_t.
 */
hal_status_t hal_i2c_pin_unregister_simple(volatile hal_i2c_adapter_t *phal_i2c_adapter)
{
	hal_status_t ret;

	hal_sys_peripheral_en(I2C0_SYS + phal_i2c_adapter->init_dat.index, 0);

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.scl_pin) != (uint8_t)0xFF) {
		if (phal_i2c_adapter->init_dat.index == 3) {
			ret = hal_pinmux_unregister(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C3);
		} else {
			ret = hal_pinmux_unregister(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.sda_pin) != (uint8_t)0xFF) {
		if (phal_i2c_adapter->init_dat.index == 3) {
			ret |= hal_pinmux_unregister(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C3);
		} else {
			ret |= hal_pinmux_unregister(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	return ret;
}

void hal_gpio_comm_init_simple(phal_gpio_comm_adapter_t pgpio_comm_adp, uint32_t pin_name)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_sys_peripheral_en(GPIO_AON, ENABLE);
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_sys_peripheral_en(GPIO_SYS, ENABLE);

	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_sys_peripheral_en(GPIO_PON, ENABLE);
	}

	hal_pinmux_register(pin_name, PID_GPIO);
}

/**
 *  @brief To de-initial the SYSON GPIO common resource.
 *           - Disable the SYSON GPIO hardware function.
 *
 *  @returns    void
 */
void hal_gpio_comm_deinit_simple(phal_gpio_comm_adapter_t pgpio_comm_adp, uint32_t pin_name)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_sys_peripheral_en(GPIO_AON, DISABLE);
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_sys_peripheral_en(GPIO_SYS, DISABLE);

	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_sys_peripheral_en(GPIO_PON, DISABLE);
	}

	hal_pinmux_unregister(pin_name, PID_GPIO);
}

