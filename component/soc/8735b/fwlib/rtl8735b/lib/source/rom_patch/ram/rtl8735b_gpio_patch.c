/**************************************************************************//**
* @file        rtl8735b_gpio_patch.c
* @brief       This file implements the GPIO HAL ROM patch functions.
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

        \addtogroup hal_gpio
        @{
*/

#if IS_CUT_TEST(CONFIG_CHIP_VER)

#if defined(CONFIG_BUILD_SECURE)
typedef void __attribute__((cmse_nonsecure_call)) ns_func(uint32_t, gpio_int_trig_type_t); // weide try
#endif

/**** Variables declaration for test chip patching [START] ***/

phal_gpio_comm_adapter_t _pgpio_comm_adp_ram_patch;
phal_aon_gpio_comm_adapter_t _paon_gpio_comm_adp_ram_patch;
phal_pon_gpio_comm_adapter_t _ppon_gpio_comm_adp_ram_patch;

const volatile uint32_t *pport_dp_sts_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_DP_STS),
	&(PON_GPIO->GPIO_PORT_A_DP_STS),
	&(SYSON_GPIO->GPIO_PORT_A_DP_STS),
	&(SYSON_GPIO->GPIO_PORT_B_DP_STS)
};

const volatile uint32_t *pport_odl_en_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODL_EN),
	&(PON_GPIO->GPIO_PORT_A_ODL_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODL_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODL_EN)
};

const volatile uint32_t *pport_odh_en_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODH_EN),
	&(PON_GPIO->GPIO_PORT_A_ODH_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODH_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODH_EN)
};

const volatile uint32_t *pport_odt_en_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODT_EN),
	&(PON_GPIO->GPIO_PORT_A_ODT_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODT_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODT_EN)
};

const volatile uint32_t *pport_dmd_sts_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_DMD_STS),
	&(PON_GPIO->GPIO_PORT_A_DMD_STS),
	&(SYSON_GPIO->GPIO_PORT_A_DMD_STS),
	&(SYSON_GPIO->GPIO_PORT_B_DMD_STS)
};


const volatile uint32_t *pport_idm_en_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_IDM_EN),
	&(PON_GPIO->GPIO_PORT_A_IDM_EN),
	&(SYSON_GPIO->GPIO_PORT_A_IDM_EN),
	&(SYSON_GPIO->GPIO_PORT_B_IDM_EN)
};

const volatile uint32_t *pport_odm_en_ram_patch[GPIO_MAX_PORT_NUM] = {
	&(AON_GPIO->GPIO_PORT_A_ODM_EN),
	&(PON_GPIO->GPIO_PORT_A_ODM_EN),
	&(SYSON_GPIO->GPIO_PORT_A_ODM_EN),
	&(SYSON_GPIO->GPIO_PORT_B_ODM_EN)
};

const volatile uint32_t *pgpio_irq_cfg_ram_patch[GPIO_MAX_INT_PIN] = {
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

const volatile uint32_t *paon_gpio_irq_cfg_ram_patch[GPIO_MAX_INT_PIN] = {
	&(AON_GPIO->GPIO_INT0_SEL),
	&(AON_GPIO->GPIO_INT1_SEL),
	&(AON_GPIO->GPIO_INT2_SEL),
	&(AON_GPIO->GPIO_INT3_SEL),
	&(AON_GPIO->GPIO_INT4_SEL),
	&(AON_GPIO->GPIO_INT5_SEL)
};

const volatile uint32_t *ppon_gpio_irq_cfg_ram_patch[GPIO_MAX_INT_PIN] = {
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

const volatile uint32_t *pgpio_debounce_cfg_ram_patch[GPIO_MAX_DEBOUNCE_PIN] = {
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

const volatile uint32_t *paon_gpio_debounce_cfg_ram_patch[GPIO_MAX_DEBOUNCE_PIN] = {
	&(AON_GPIO->GPIO_DEB0_SEL),
	&(AON_GPIO->GPIO_DEB1_SEL),
	&(AON_GPIO->GPIO_DEB2_SEL),
	&(AON_GPIO->GPIO_DEB3_SEL),
	&(AON_GPIO->GPIO_DEB4_SEL),
	&(AON_GPIO->GPIO_DEB5_SEL)
};

const volatile uint32_t *ppon_gpio_debounce_cfg_ram_patch[GPIO_MAX_DEBOUNCE_PIN] = {
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

uint32_t chip_port_2_ip_port_ram_patch[] = {
	(GPIO_IP_PORT0 << 24) | (0 << 18) | 0x0003F,      ///< Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins
	(GPIO_IP_PORT2 << 24) | (0 << 18) | 0x00007,      ///< Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins
	(GPIO_IP_PORT2 << 24) | (3 << 18) | 0x0003F,      ///< Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins
	(GPIO_IP_PORT2 << 24) | (9 << 18) | 0x1FFFF,      ///< Port D: GPIO IP Port2[25:9] // SYSON GPIO 17 pins
	(GPIO_IP_PORT2 << 24) | (26 << 18) | 0x0003F,     ///< Port E1: GPIO IP Port2[31:26] // SYSON GPIO 6 pins
	(GPIO_IP_PORT3 << 24) | (0 << 18) | 0x0001F,      ///< Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 1)
	(GPIO_IP_PORT1 << 24) | (0 << 18) | 0x3FFFF,      ///< Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins
	(GPIO_IP_PORT3 << 24) | (5 << 18) | 0x0007F       ///< Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 1)
};

// IRQ Handlers - unchanged from ROM, but appear here due to patching; taking from ROM; only name changed

void AON_IRQHandler_ram_patch(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_irq_clear_pending(AON_IRQn);

	int_sts = AON_GPIO->GPIO_INT_STS;

	if (_paon_gpio_comm_adp_ram_patch == NULL) {
		// Not initialed yet, ignore interrupt
		AON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _paon_gpio_comm_adp_ram_patch->gpio_irq_list_head;

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
				ns_func *fptr = cmse_nsfptr_create((ns_func *)(pgpio_irq->irq_callback));
				fptr(pgpio_irq->irq_callback_arg, int_type);
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
		_paon_gpio_comm_adp_ram_patch->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();

}

void GPIO_IRQHandler_ram_patch(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_irq_clear_pending(GPIO_IRQn);

	int_sts = SYSON_GPIO->GPIO_INT_STS;

	if (_pgpio_comm_adp_ram_patch == NULL) {
		// Not initialed yet, ignore interrupt
		SYSON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _pgpio_comm_adp_ram_patch->gpio_irq_list_head;

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
		_pgpio_comm_adp_ram_patch->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();

}

void PonGPIO_IRQHandler_ram_patch(void)
{
	uint32_t int_sts;
	uint32_t int_idx;
	uint32_t bit_mask;
	phal_gpio_irq_adapter_t pgpio_irq;
	gpio_int_trig_type_t int_type;

	hal_irq_clear_pending(PonGPIO_IRQn);

	int_sts = PON_GPIO->GPIO_INT_STS;

	if (_ppon_gpio_comm_adp_ram_patch == NULL) {
		// Not initialed yet, ignore interrupt
		PON_GPIO->GPIO_INT_CLR = int_sts;
		__DSB();
		__ISB();
		return;
	}

	pgpio_irq = _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head;

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
		_ppon_gpio_comm_adp_ram_patch->err_flag.irq_err = 1;
	}
	__DSB();
	__ISB();

}

void hal_rtl_gpio_enter_critical_patch(void)
{
	if (__get_IPSR() == (GPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_irq_disable(GPIO_IRQn);

	__ISB();
	_pgpio_comm_adp_ram_patch->critical_lv++;
	__DSB();
}

void hal_rtl_aon_gpio_enter_critical_patch(void)
{
	if (__get_IPSR() == (AON_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_irq_disable(AON_IRQn);

	__ISB();
	_paon_gpio_comm_adp_ram_patch->critical_lv++;
	__DSB();
}

void hal_rtl_pon_gpio_enter_critical_patch(void)
{
	if (__get_IPSR() == (PonGPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	hal_irq_disable(PonGPIO_IRQn);

	__ISB();
	_ppon_gpio_comm_adp_ram_patch->critical_lv++;
	__DSB();
}

void hal_rtl_gpio_exit_critical_patch(void)
{
	if (__get_IPSR() == (GPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_pgpio_comm_adp_ram_patch->critical_lv > 0) {
		_pgpio_comm_adp_ram_patch->critical_lv--;
		__DSB();
		if (_pgpio_comm_adp_ram_patch->critical_lv == 0) {
			hal_irq_enable(GPIO_IRQn);
			__ISB();
		}
	}
}

void hal_rtl_aon_gpio_exit_critical_patch(void)
{
	if (__get_IPSR() == (AON_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_paon_gpio_comm_adp_ram_patch->critical_lv > 0) {
		_paon_gpio_comm_adp_ram_patch->critical_lv--;
		__DSB();
		if (_paon_gpio_comm_adp_ram_patch->critical_lv == 0) {
			hal_irq_enable(AON_IRQn);
			__ISB();
		}
	}
}

void hal_rtl_pon_gpio_exit_critical_patch(void)
{
	if (__get_IPSR() == (PonGPIO_IRQn + 16)) {
		// In an ISR
		return;
	}

	if (_ppon_gpio_comm_adp_ram_patch->critical_lv > 0) {
		_ppon_gpio_comm_adp_ram_patch->critical_lv--;
		__DSB();
		if (_ppon_gpio_comm_adp_ram_patch->critical_lv == 0) {
			hal_irq_enable(PonGPIO_IRQn);
			__ISB();
		}
	}
}

void hal_rtl_gpio_irq_list_add_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	if (pgpio_irq_adapter == NULL) {
		return;
	}

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_rtl_aon_gpio_enter_critical_patch();
		if (_paon_gpio_comm_adp_ram_patch->gpio_irq_list_head == NULL) {
			// the first one
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_head = pgpio_irq_adapter;
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_aon_gpio_exit_critical_patch();

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_rtl_gpio_enter_critical_patch();
		if (_pgpio_comm_adp_ram_patch->gpio_irq_list_head == NULL) {
			// the first one
			_pgpio_comm_adp_ram_patch->gpio_irq_list_head = pgpio_irq_adapter;
			_pgpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_pgpio_comm_adp_ram_patch->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_pgpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_gpio_exit_critical_patch();

	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_rtl_pon_gpio_enter_critical_patch();
		if (_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head == NULL) {
			// the first one
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head = pgpio_irq_adapter;
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		} else {
			// add to the tail of the list
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail->pnext = pgpio_irq_adapter;
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pgpio_irq_adapter;
		}
		hal_rtl_pon_gpio_exit_critical_patch();
	}

}

hal_status_t hal_rtl_gpio_irq_list_remove_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	hal_status_t ret = HAL_NOT_FOUND;
	phal_gpio_irq_adapter_t ptemp;
	phal_gpio_irq_adapter_t pprev;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		hal_rtl_aon_gpio_enter_critical_patch();
		if ((pgpio_irq_adapter == _paon_gpio_comm_adp_ram_patch->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_head = NULL;
			_paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = NULL;
			hal_rtl_aon_gpio_exit_critical_patch();
			return HAL_OK;
		} else {
			ptemp = _paon_gpio_comm_adp_ram_patch->gpio_irq_list_head;
			pprev = _paon_gpio_comm_adp_ram_patch->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _paon_gpio_comm_adp_ram_patch->gpio_irq_list_head) {
						_paon_gpio_comm_adp_ram_patch->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail) {
						_paon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pprev;
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
		hal_rtl_aon_gpio_exit_critical_patch();

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		hal_rtl_gpio_enter_critical_patch();
		if ((pgpio_irq_adapter == _pgpio_comm_adp_ram_patch->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _pgpio_comm_adp_ram_patch->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_pgpio_comm_adp_ram_patch->gpio_irq_list_head = NULL;
			_pgpio_comm_adp_ram_patch->gpio_irq_list_tail = NULL;
			hal_rtl_gpio_exit_critical_patch();
			return HAL_OK;
		} else {
			ptemp = _pgpio_comm_adp_ram_patch->gpio_irq_list_head;
			pprev = _pgpio_comm_adp_ram_patch->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _pgpio_comm_adp_ram_patch->gpio_irq_list_head) {
						_pgpio_comm_adp_ram_patch->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _pgpio_comm_adp_ram_patch->gpio_irq_list_tail) {
						_pgpio_comm_adp_ram_patch->gpio_irq_list_tail = pprev;
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
		hal_rtl_gpio_exit_critical_patch();
	} else if (port_idx == PORT_F) {
		// PON GPIO
		hal_rtl_pon_gpio_enter_critical_patch();
		if ((pgpio_irq_adapter == _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head) &&
			(pgpio_irq_adapter == _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail)) {
			// it's the only one adapter in the list
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head = NULL;
			_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = NULL;
			hal_rtl_pon_gpio_exit_critical_patch();
			return HAL_OK;
		} else {
			ptemp = _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head;
			pprev = _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head;
			while (ptemp != NULL) {
				if (ptemp == pgpio_irq_adapter) {
					// found the adapter in the list
					if (ptemp == _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head) {
						_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_head = ptemp->pnext;
					} else if (ptemp == _ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail) {
						_ppon_gpio_comm_adp_ram_patch->gpio_irq_list_tail = pprev;
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
		hal_rtl_pon_gpio_exit_critical_patch();
	}
	return ret;
}

/**** Variables declaration and internal function defintions for test chip patching [END] ***/

/**** Functions for test chip patching [START] ****/

// EXACT SAME function exists in ROM, but defined here again (independent of ROM) for the purpose of test chip patching [START]

void hal_rtl_gpio_reg_irq_patch(gpio_type_t gpio_type, irq_handler_t irq_handler)   // possible source of problems!
{
	// IRQ vector may has been registered, disable and re-register it - in order of GPIO type
	if (gpio_type == AonGPIO) {
		// AON GPIO
		hal_irq_disable(AON_IRQn);
		__ISB();
		hal_irq_set_vector(AON_IRQn, (uint32_t)irq_handler);
		hal_irq_enable(AON_IRQn);

	} else if (gpio_type == SysonGPIO) {
		// SYSON GPIO
		hal_irq_disable(GPIO_IRQn);
		__ISB();
		hal_irq_set_vector(GPIO_IRQn, (uint32_t)irq_handler);
		hal_irq_enable(GPIO_IRQn);

	} else if (gpio_type == PonGPIO) {
		// PON GPIO
		hal_irq_disable(PonGPIO_IRQn);
		__ISB();
		hal_irq_set_vector(PonGPIO_IRQn, (uint32_t)irq_handler);
		hal_irq_enable(PonGPIO_IRQn);
	}

}

hal_status_t hal_rtl_gpio_interrupt_clk_sel_patch(gpio_type_t gpio_type, uint8_t clk_sel)
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
			DBG_GPIO_INFO("AON GPIO interrupt clock: 32kHz clock\r\n");
		} else {
			// HW setting 2
			PON->PON_REG_PON_FUNC_CLK_CTRL &= ~(PON_BIT_INTR_CLK_GPIO_SEL);
			DBG_GPIO_INFO("AON GPIO interrupt clock: APB clk\r\n");
		}

	} else if (gpio_type == SysonGPIO) { // possible source of problems!!! - careful of S/NS
		if (clk_sel) {
			// HW setting
			SYSON_S->SYSON_S_REG_GPIO_CTRL |= SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL;
			DBG_GPIO_INFO("AON GPIO interrupt clock: Debounce clk\r\n");
		} else {
			// HW setting 2
			SYSON_S->SYSON_S_REG_GPIO_CTRL &= ~(SYSON_S_BIT_SYS_GPIO_INTR_CLK_SEL);
			DBG_GPIO_INFO("AON GPIO interrupt clock: APB clk\r\n");
		}
	}

	return HAL_OK;
}

// EXACT SAME function exists in ROM, but defined here again (independent of ROM) for the purpose of test chip patching [END]

hal_status_t hal_rtl_gpio_init_patch(phal_gpio_adapter_t pgpio_adapter, uint32_t pin_name)
{
	uint8_t port_idx = PIN_NAME_2_PORT(pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pin_name);
	uint32_t bit_mask;
	uint32_t *port_idm_en;
	gpio_type_t gpio_type; // 0: AON GPIO, 1: SYSON GPIO, 2: PON GPIO

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("GPIO Init Invalid: port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}
	// convert chip pin definition to IP pin definition
	/* Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins
	 * Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins
	 * Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins
	 * Port D: GPIO IP Port2[25:9] // SYSON GPIO 17 pins
	 * Port E1: GPIO IP Port2[31:26] // SYSON GPIO 6 pins
	 * Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 3 from Port 2)
	 * Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins
	 * Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 3 from Port 2)
	 */
	switch (port_idx) {
	case PORT_A: // AON GPIO
		port_idx = 0;
		pin_idx += 0;
		gpio_type = AonGPIO;
		break;
	case PORT_B:
		port_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 0;
		gpio_type = SysonGPIO;
		break;
	case PORT_C:
		port_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 3; // pin starts at 4th position, Pin 3 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	case PORT_D:
		port_idx = 2; // for the sake of pport_odl/odh/odt/dmd arrays; Still at Port A (SYSON GPIO)
		pin_idx += 9; // pin starts at 10th position, Pin 9 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	case PORT_E:
		if (pin_idx > 5) {
			port_idx = 3;
			pin_idx -= 6; // because pin_idx = 6 will be mapped to Group B[0], pin_idx = 7 mapped to Group B[1], etc
		} else { // means still at Port A [31:26]
			port_idx = 2;
			pin_idx += 26;
		}
		gpio_type = SysonGPIO;
		break;
	case PORT_F: // PON GPIO
		port_idx = 1;
		pin_idx += 0;
		gpio_type = PonGPIO;
		break;
	case PORT_S: // SYSON GPIO Group B
		port_idx = 3;
		pin_idx += 5; // pin starts at 6th position, Pin 5 (Pin 0 is the 1st pin)
		gpio_type = SysonGPIO;
		break;
	}

	memset((void *) pgpio_adapter, 0, sizeof(hal_gpio_adapter_t));

	bit_mask = 1 << pin_idx;
	DBG_GPIO_INFO("bit_mask:%x\r\n", bit_mask);

	pgpio_adapter->pin_name = pin_name;
	pgpio_adapter->port_idx = port_idx;
	pgpio_adapter->pin_idx = pin_idx;
	pgpio_adapter->bit_mask = 1 << pin_idx;
	pgpio_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[port_idx];
	DBG_GPIO_INFO("in_port: %x\r\n", pgpio_adapter->in_port);
	pgpio_adapter->out0_port = (uint32_t *)pport_odl_en_ram_patch[port_idx];
	DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_adapter->out0_port);
	pgpio_adapter->out1_port = (uint32_t *)pport_odh_en_ram_patch[port_idx];
	DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_adapter->out1_port);
	pgpio_adapter->outt_port = (uint32_t *)pport_odt_en_ram_patch[port_idx];
	DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_adapter->outt_port);
	pgpio_adapter->debounce_idx = 0xFF; // mark as not using debounce

	// 1. Set IRQ priorities and 2. Register IRQ handlers
	if (gpio_type == AonGPIO) {
		// AON GPIO
		hal_sys_peripheral_en(GPIO_AON, ENABLE);

	} else if (gpio_type == SysonGPIO) {
		// SYSON GPIO
		hal_sys_peripheral_en(GPIO_SYS, ENABLE);

	} else if (gpio_type == PonGPIO) {
		// PON GPIO
		hal_sys_peripheral_en(GPIO_PON, ENABLE);

	}

	// default configure it as an input pin
	port_idm_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];
	*port_idm_en = bit_mask;

	return HAL_OK;
}

hal_status_t hal_rtl_gpio_port_init_patch(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t port_id,
		uint32_t mask, gpio_dir_t dir)
{
	uint32_t *port_dir_en;
	uint32_t port_idx;
	gpio_port_t *pip_port;

	if (port_id >= PORT_MAX_NUM) {
		DBG_GPIO_ERR("GPIO port init invalid: port=%u \r\n", port_id);
		return HAL_ERR_PARA;
	}
	// need to plus 1 to MAX_PIN_IN_PORT??
	/*if (mask >= (1 << MAX_PIN_IN_PORT)) { // may have to change this fit AON GPIO number???
	    DBG_GPIO_ERR ("GPIO port Init Invalid: mask=0x%08x \r\n", mask);
	    return HAL_ERR_PARA;
	}*/ // weide temp remove 9 Feb 2021

	memset((void *) pgpio_port_adapter, 0, sizeof(hal_gpio_port_adapter_t));

	//pgpio_port_adapter->chip_port_idx = port_id; // weide added, to facilitate pinmux reg and unreg process

	if (port_id == PORT_E) {

		pip_port = (gpio_port_t *)&chip_port_2_ip_port_ram_patch[port_id];
		DBG_GPIO_INFO("pip_port <= 0x3F: %x\r\n", *pip_port);
		port_idx = pip_port->port_name_b.port;

		pgpio_port_adapter->port_idx = port_idx;
		DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
		pgpio_port_adapter->pin_offset = pip_port->port_name_b.offset;
		DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);
		pgpio_port_adapter->pin_mask = mask & pip_port->port_name_b.mask;
		DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);
		pgpio_port_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[port_idx];
		DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port);
		pgpio_port_adapter->out0_port = (uint32_t *)pport_odl_en_ram_patch[port_idx];
		DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port);
		pgpio_port_adapter->out1_port = (uint32_t *)pport_odh_en_ram_patch[port_idx];
		DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port);
		pgpio_port_adapter->outt_port = (uint32_t *)pport_odt_en_ram_patch[port_idx];
		DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port);

		// set direction
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];
		} else {
			port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pip_port->port_name_b.offset;

		if (mask > 0x3F) { // in GPIO Group B (overflowed from Group A to Group B)

			pip_port = (gpio_port_t *)&chip_port_2_ip_port_ram_patch[port_id + 1]; // internal port
			DBG_GPIO_INFO("pip_port > 0x3F: %x\r\n", *pip_port);

			port_idx = pip_port->port_name_b.port;
			DBG_GPIO_INFO("port_idx in overflow: %x\r\n", port_idx);

			pgpio_port_adapter->port_idx = port_idx; // don't store new port_idx val from overflow
			DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx); // don't store new port_idx val from overflow
			pgpio_port_adapter->pin_offset_PORTE = pip_port->port_name_b.offset;
			DBG_GPIO_INFO("pin_offset_PORTE: %x\r\n", pgpio_port_adapter->pin_offset_PORTE);
			pgpio_port_adapter->pin_mask |= (((mask >> 6) & pip_port->port_name_b.mask) << 6);
			DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);

			pgpio_port_adapter->in_port_PORTE = (uint32_t *)pport_dp_sts_ram_patch[port_idx];
			DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port_PORTE);
			pgpio_port_adapter->out0_port_PORTE = (uint32_t *)pport_odl_en_ram_patch[port_idx];
			DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port_PORTE);
			pgpio_port_adapter->out1_port_PORTE = (uint32_t *)pport_odh_en_ram_patch[port_idx];
			DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port_PORTE);
			pgpio_port_adapter->outt_port_PORTE = (uint32_t *)pport_odt_en_ram_patch[port_idx];
			DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port_PORTE);

			// set direction
			if (dir == GPIO_IN) {
				port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];
			} else {
				port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx];
			}

			*port_dir_en = ((pgpio_port_adapter->pin_mask >> 6) << pip_port->port_name_b.offset);
		}

	} else {

		pip_port = (gpio_port_t *)&chip_port_2_ip_port_ram_patch[port_id];
		port_idx = pip_port->port_name_b.port;

		pgpio_port_adapter->port_idx = port_idx;
		DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
		pgpio_port_adapter->pin_offset = pip_port->port_name_b.offset;
		DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);
		pgpio_port_adapter->pin_mask = mask & pip_port->port_name_b.mask;
		DBG_GPIO_INFO("pin_mask: %x\r\n", pgpio_port_adapter->pin_mask);
		pgpio_port_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[port_idx];
		DBG_GPIO_INFO("in_port: %x\r\n", pgpio_port_adapter->in_port);
		pgpio_port_adapter->out0_port = (uint32_t *)pport_odl_en_ram_patch[port_idx];
		DBG_GPIO_INFO("out0_port: %x\r\n", pgpio_port_adapter->out0_port);
		pgpio_port_adapter->out1_port = (uint32_t *)pport_odh_en_ram_patch[port_idx];
		DBG_GPIO_INFO("out1_port: %x\r\n", pgpio_port_adapter->out1_port);
		pgpio_port_adapter->outt_port = (uint32_t *)pport_odt_en_ram_patch[port_idx];
		DBG_GPIO_INFO("outt_port: %x\r\n", pgpio_port_adapter->outt_port);

		// set direction
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];
		} else {
			port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pip_port->port_name_b.offset;
	}

	return HAL_OK;
}

void hal_rtl_gpio_port_write_patch(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, uint32_t io_data)
{
	// mask is dummy
	uint32_t port_value;
	uint32_t pin_mask_for_PortE_overflow; // temp variable to store the pin mask for Port E2 (Port E GPIO overflow to Group B GPIO)

	DBG_GPIO_INFO("port_idx: %x\r\n", pgpio_port_adapter->port_idx);
	DBG_GPIO_INFO("pin_offset: %x\r\n", pgpio_port_adapter->pin_offset);
#if 0 // weide mod
	// write bits of out 1
	port_value = io_data & pgpio_port_adapter->pin_mask;
	*(pgpio_port_adapter->out1_port) = port_value << pgpio_port_adapter->pin_offset;
#else
	// write bits of out 1 (means write to output data high register)
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 26)) { // GPIO Port "E2"

		DBG_GPIO_INFO("GPIO E overflowed 1; handling...\r\n");
		if (pgpio_port_adapter->pin_mask > 0x3F) {
			// First take care of the part that overflowed to Group B GPIO, or in other words, it went into Port "E2"
			port_value = (((io_data & pgpio_port_adapter->pin_mask) >> 6) << (pgpio_port_adapter->pin_offset_PORTE));
			//DBG_GPIO_INFO("port_val for overflow case: %x\r\n", port_value);
			DBG_GPIO_INFO("port_val for overflow case 1: %x\r\n", port_value);
			*(pgpio_port_adapter->out1_port_PORTE) = port_value;

			// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
			// ** Do "pgpio_port_adapter->pin_mask & 0x3F", to extract the mask for the part which didn't overflow **
			port_value = ((io_data & (pgpio_port_adapter->pin_mask & 0x3F)) << (pgpio_port_adapter->pin_offset));
			//DBG_GPIO_INFO("port_val: %x\r\n", port_value);
			DBG_GPIO_INFO("port_val no overflow case 1: %x\r\n", port_value);
			*(pgpio_port_adapter->out1_port) = port_value;

		} else { // No overflow case, pin_mask is less than or equal to 0x3F - treat as normal case - may never enter here
			port_value = ((io_data & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
			DBG_GPIO_INFO("port_val 0x3F or less 1: %x\r\n", port_value);
			*(pgpio_port_adapter->out1_port) = port_value;
		}
	} else { // treat as normal case, for all other GPIO ports - only GPIO E very the special
		port_value = ((io_data & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
		DBG_GPIO_INFO("port_val normal ports 1: %x\r\n", port_value);
		*(pgpio_port_adapter->out1_port) = port_value;
	}

#endif

#if 0
	// write bits of out 0
	port_value = (~io_data) & pgpio_port_adapter->pin_mask;
	*(pgpio_port_adapter->out0_port) = port_value << pgpio_port_adapter->pin_offset;
#else
	// write bits of out 0 (means write to output data low register)
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 26)) { // GPIO Port "E2"
		DBG_GPIO_INFO("GPIO E overflowed 0; handling...\r\n");
		// look at pin_mask
		if (pgpio_port_adapter->pin_mask > 0x3F) {
			// First take care of the part that overflowed to Group B GPIO, or it went into Port "E2"
			port_value = ((((~io_data) & pgpio_port_adapter->pin_mask) >> 6) << (pgpio_port_adapter->pin_offset_PORTE));
			//DBG_GPIO_INFO("port_val for overflow case: %x\r\n", port_value);
			DBG_GPIO_INFO("port_val for overflow case 0: %x\r\n", port_value);
			*(pgpio_port_adapter->out0_port_PORTE) = port_value;

			// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
			// ** Do "pgpio_port_adapter->pin_mask & 0x3F", to extract the mask for the part which didn't overflow **
			port_value = (((~io_data) & (pgpio_port_adapter->pin_mask & 0x3F)) << (pgpio_port_adapter->pin_offset));
			//DBG_GPIO_INFO("port_val: %x\r\n", port_value);
			DBG_GPIO_INFO("port_val no overflow case 0: %x\r\n", port_value);
			*(pgpio_port_adapter->out0_port) = port_value;

		} else { // No overflow case, pin_mask is less than or equal to 0x3F - treat as normal case
			/* port_value = (((~io_data) & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
			DBG_GPIO_INFO("port_val 0x3F or less 0: %x\r\n", port_value);
			*(pgpio_port_adapter->out0_port) = port_value; */ // not used - Weide 28JUN2021
		}
	} else { // treat as normal case, for all other GPIO ports - only GPIO E very the special
		port_value = (((~io_data) & pgpio_port_adapter->pin_mask) << (pgpio_port_adapter->pin_offset));
		DBG_GPIO_INFO("port_val normal ports 0: %x\r\n", port_value);
		*(pgpio_port_adapter->out0_port) = port_value;
	}

#endif
}

uint32_t hal_rtl_gpio_port_read_patch(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask)
{
	uint32_t read_value_portE_overflow;
	uint32_t read_value_portE;

	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 26)) { // GPIO Port "E2"

		DBG_GPIO_INFO("GPIO E overflowed 0; handling... (read)\r\n");

		// First take care of the part that overflowed to Group B GPIO, or it went into Port "E2"
		read_value_portE_overflow = (*((volatile uint32_t *)(pgpio_port_adapter->in_port_PORTE)) & (pgpio_port_adapter->pin_mask >> 6));
		DBG_GPIO_INFO("read_value for overflow case (read): %x\r\n", read_value_portE_overflow);

		// Then, take care of the part that didn't overflow, i.e. still in Group A GPIO, and is in Port "E1"
		// ** Do "pgpio_port_adapter->pin_mask & 0x3F", to extract the mask for the part which didn't overflow **
		read_value_portE = ((*((volatile uint32_t *)(pgpio_port_adapter->in_port)) >> pgpio_port_adapter->pin_offset) & (pgpio_port_adapter->pin_mask & 0x3F));
		DBG_GPIO_INFO("read_value Port E no overflow (read): %x\r\n", read_value_portE);
		read_value_portE = (read_value_portE) | (read_value_portE_overflow << 6);
		DBG_GPIO_INFO("Final read_value (read): %x\r\n", read_value_portE);

		return read_value_portE;

	} else { // all other ports other than PORT_E
		return ((*((volatile uint32_t *)(pgpio_port_adapter->in_port)) >> pgpio_port_adapter->pin_offset) & pgpio_port_adapter->pin_mask);
	}
}

void hal_rtl_gpio_port_dir_patch(phal_gpio_port_adapter_t pgpio_port_adapter, uint32_t mask, gpio_dir_t dir)
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
	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 26)) { // GPIO Port "E2"
		DBG_GPIO_INFO("GPIO E overflowed; handling...\r\n");
		// First, handle the overflow case - GPIO Port "E2"; incoming port_idx will be 3 if GPIO E overflowed in port_init
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx];
		}

		*port_dir_en = ((pgpio_port_adapter->pin_mask >> 6) << pgpio_port_adapter->pin_offset_PORTE);

		// Then we handle GPIO E without overflow case, just in case the first 6 bits of GPIO E have something
		DBG_GPIO_INFO("In GPIO E overflowed; handling no overflow case...\r\n");
		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx - 1];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx - 1];
		}

		*port_dir_en = ((pgpio_port_adapter->pin_mask & 0x3F) << pgpio_port_adapter->pin_offset);

	} else {

		if (dir == GPIO_IN) {
			port_dir_en = (uint32_t *)pport_idm_en_ram_patch[port_idx];

		} else {
			port_dir_en = (uint32_t *)pport_odm_en_ram_patch[port_idx];
		}

		*port_dir_en = pgpio_port_adapter->pin_mask << pgpio_port_adapter->pin_offset;
	}
}

void hal_rtl_gpio_port_deinit_patch(phal_gpio_port_adapter_t pgpio_port_adapter)
{
	uint32_t *port_idm_en;

	if ((pgpio_port_adapter->port_idx == 3) && (pgpio_port_adapter->pin_offset == 26)) {

		DBG_GPIO_INFO("GPIO E overflowed; handling... (deinit)\r\n");
		port_idm_en = (uint32_t *)pport_idm_en_ram_patch[pgpio_port_adapter->port_idx];

		// First, handle the overflow case - GPIO Port "E2"; incoming port_idx will be 3 if GPIO E overflowed in port_init
		// switch to input mode
		*port_idm_en = ((pgpio_port_adapter->pin_mask >> 6) << pgpio_port_adapter->pin_offset_PORTE);

		// Then we handle GPIO E without overflow case, just in case the first 6 bits of GPIO E have something
		DBG_GPIO_INFO("In GPIO E overflowed; handling no overflow case...(deinit)\r\n");
		port_idm_en = (uint32_t *)pport_idm_en_ram_patch[pgpio_port_adapter->port_idx - 1];

		// switch to input mode
		*port_idm_en = ((pgpio_port_adapter->pin_mask & 0x3F) << pgpio_port_adapter->pin_offset);

	} else {
		port_idm_en = (uint32_t *)pport_idm_en_ram_patch[pgpio_port_adapter->port_idx];

		// switch to input mode
		*port_idm_en = pgpio_port_adapter->pin_mask << pgpio_port_adapter->pin_offset;
	}

}

void hal_rtl_gpio_irq_disable_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	uint32_t int_idx;
	uint32_t bit_mask;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (pgpio_irq_adapter == NULL) {
		return;
	}

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

hal_status_t hal_rtl_gpio_irq_init_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t pin_name,
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
		if (_paon_gpio_comm_adp_ram_patch == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if (_pgpio_comm_adp_ram_patch == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if (_ppon_gpio_comm_adp_ram_patch == NULL) {
			DBG_GPIO_ERR("hal_gpio_irq_init: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	}

	if ((port_idx >= PORT_MAX_NUM) || (pin_idx >= MAX_PIN_IN_PORT)) {
		DBG_GPIO_ERR("GPIO IRQ Init Invalid: port=%u pin=%u\r\n", port_idx, pin_idx);
		return HAL_ERR_PARA;
	}

	// convert chip pin definition to IP pin definition
	/* Port A: GPIO IP Port0[5:0] // ***AON*** GPIO 6 pins
	 * Port B: GPIO IP Port2[2:0] // SYSON GPIO 3 pins
	 * Port C: GPIO IP Port2[8:3] // SYSON GPIO 6 pins
	 * Port D: GPIO IP Port2[25:9] // SYSON GPIO 17 pins
	 * Port E1: GPIO IP Port2[31:26] // SYSON GPIO 6 pins
	 * Port E2: GPIO IP Port3[4:0] // SYSON GPIO 5 pins (overflow to Port 3 from Port 2)
	 * Port F: GPIO IP Port1[17:0] // ***PON*** GPIO 18 pins
	 * Port S: GPIO IP Port3[11:5] // SYSON GPIO 7 pins (overflow to Port 3 from Port 2)
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
		if (pin_idx > 5) {
			array_index = 3;
			pin_idx -= 6; // because pin_idx = 6 will be mapped to Group B[0], pin_idx = 7 mapped to Group B[1], etc
			gpio_gp_sel = 1; // for INTx SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B)
		} else { // means still at Port A [31:26]
			array_index = 2;
			pin_idx += 26;
			gpio_gp_sel = 0; // for INTx SEL reg - Still at Group A
		}
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

	memset((void *) pgpio_irq_adapter, 0, sizeof(hal_gpio_irq_adapter_t));

	if (port_idx == PORT_A) {

		// find a free AON GPIO IRQ pin // AON_GPIO_MAX_INT_PIN = 6, cos AON GPIO has only 6 pins
		for (i = 0; i < AON_GPIO_MAX_INT_PIN; i++) {
			if ((_paon_gpio_comm_adp_ram_patch->gpio_irq_using & (1 << i)) == 0) {
				// find a free IRQ pin
				_paon_gpio_comm_adp_ram_patch->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as not using debounce

				pirq_cfg = (uint32_t *)paon_gpio_irq_cfg_ram_patch[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg |= pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp
				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en_ram_patch[array_index];
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
		hal_rtl_gpio_irq_list_add_patch(pgpio_irq_adapter);

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {

		// find a free SYSON GPIO IRQ pin
		for (i = 0; i < GPIO_MAX_INT_PIN; i++) {
			if ((_pgpio_comm_adp_ram_patch->gpio_irq_using & (1 << i)) == 0) {
				// find a free IRQ pin
				_pgpio_comm_adp_ram_patch->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as no using debounce

				pirq_cfg = (uint32_t *)pgpio_irq_cfg_ram_patch[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg = pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp

				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en_ram_patch[array_index];
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
		hal_rtl_gpio_irq_list_add_patch(pgpio_irq_adapter);

	} else if (port_idx == PORT_F) {

		// find a free PON GPIO IRQ pin
		for (i = 0; i < GPIO_MAX_INT_PIN; i++) {
			if ((_ppon_gpio_comm_adp_ram_patch->gpio_irq_using & (1 << i)) == 0) {
				// find a free GPIO IRQ pin
				_ppon_gpio_comm_adp_ram_patch->gpio_irq_using |= (1 << i);
				pgpio_irq_adapter->pin_name = pin_name;
				pgpio_irq_adapter->ip_pin_name = PIN_NAME(port_idx, pin_idx);
				pgpio_irq_adapter->int_idx = i;
				pgpio_irq_adapter->in_port = (uint32_t *)pport_dp_sts_ram_patch[array_index];
				pgpio_irq_adapter->bit_mask = 1 << pin_idx;
				pgpio_irq_adapter->debounce_idx = 0xFF; // mask as no using debounce

				pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg_ram_patch[i];
				*pirq_cfg ^= *pirq_cfg; // zero INTx_SEL reg, x=[0,15]; choose to toggle entire reg to clear; unsure
				*pirq_cfg = pin_idx | (gpio_gp_sel << GPIO_SHIFT_INT_GP_SEL);
				DBG_GPIO_INFO("pirq_cfg contents (irq_init): %x\r\n", *pirq_cfg); // weide temp

				// configure data mode as input
				port_idm_en = (uint32_t *)pport_idm_en_ram_patch[array_index];
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
		hal_rtl_gpio_irq_list_add_patch(pgpio_irq_adapter);
	}

	return HAL_OK;
}


void hal_rtl_gpio_irq_deinit_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{

	uint32_t int_idx;
	uint32_t bit_mask;
	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if ((pgpio_irq_adapter == NULL) || (_paon_gpio_comm_adp_ram_patch == NULL)) {
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

		_paon_gpio_comm_adp_ram_patch->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < AON_GPIO_MAX_DEBOUNCE_PIN) {
			AON_GPIO->GPIO_DEB_DIS = bit_mask;
			_paon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		AON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		AON_GPIO->GPIO_INT_CLR = bit_mask;

		hal_rtl_gpio_irq_list_remove_patch(pgpio_irq_adapter);

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if ((pgpio_irq_adapter == NULL) || (_pgpio_comm_adp_ram_patch == NULL)) {
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

		_pgpio_comm_adp_ram_patch->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < GPIO_MAX_DEBOUNCE_PIN) {
			SYSON_GPIO->GPIO_DEB_DIS = bit_mask;
			_pgpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		SYSON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		SYSON_GPIO->GPIO_INT_CLR = bit_mask;

		hal_rtl_gpio_irq_list_remove_patch(pgpio_irq_adapter);

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if ((pgpio_irq_adapter == NULL) || (_ppon_gpio_comm_adp_ram_patch == NULL)) {
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

		_ppon_gpio_comm_adp_ram_patch->gpio_irq_using &= ~bit_mask;
		if (pgpio_irq_adapter->debounce_idx < GPIO_MAX_DEBOUNCE_PIN) {
			PON_GPIO->GPIO_DEB_DIS = bit_mask;
			_ppon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		}

		// disable IRQ
		PON_GPIO->GPIO_INT_DIS = bit_mask;
		__DSB();
		// clear pending IRQ
		PON_GPIO->GPIO_INT_CLR = bit_mask;

		hal_rtl_gpio_irq_list_remove_patch(pgpio_irq_adapter);

	}

}


// Function definition exactly same as ROM - appears here because IRQ-related functions need patching
hal_status_t hal_rtl_gpio_debounce_enable_patch(phal_gpio_adapter_t pgpio_adapter, uint32_t debounce_us)
{
	uint32_t i;
	uint32_t bit_mask;
	uint32_t debounce_cyc;
	uint32_t *pdeb_cfg;
	hal_status_t ret = HAL_NO_RESOURCE;

	uint8_t gpio_gp_sel; // GPIO Group Select Signal. For ProII, GPIOA-F and S will be '0', except GPIOE5 (Pin 6) to E10 and GPIOS will be '1'.

	uint8_t pin_idx = PIN_NAME_2_PIN(pgpio_adapter->pin_name);
	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

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
		if ((_paon_gpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_deb_en: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		if ((_pgpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_deb_en: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

	} else if (port_idx == PORT_F) {
		if ((_ppon_gpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_deb_en: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
	}

	// check does this GPIO pin works on IN ?
	if ((*(pport_dmd_sts_ram_patch[pgpio_adapter->port_idx]) & (1 << pgpio_adapter->pin_idx)) != 0) {
		// currently work with OUT mode, should not use debounce
		return HAL_ERR_PARA;
	}

	if (port_idx == PORT_A) {
		// to find a free debounce function block first
		for (i = 0; i < AON_GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_paon_gpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_paon_gpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)paon_gpio_debounce_cfg_ram_patch[i];
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
			if ((_pgpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_pgpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)pgpio_debounce_cfg_ram_patch[i];
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
			if ((_ppon_gpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_ppon_gpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}

				pdeb_cfg = (uint32_t *)ppon_gpio_debounce_cfg_ram_patch[i];
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

// Function definition exactly same as ROM - appears here because IRQ-related functions need patching
void hal_rtl_gpio_debounce_disable_patch(phal_gpio_adapter_t pgpio_adapter)
{
	uint32_t deb_idx;
	uint32_t bit_mask;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if ((_paon_gpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
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
		_paon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		if ((_pgpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
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
		_pgpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if ((_ppon_gpio_comm_adp_ram_patch == NULL) || (pgpio_adapter == NULL)) {
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
		_ppon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_adapter->debounce_idx = 0xFF;

	}

}

// Function definition exactly same as ROM - appears here because IRQ-related functions need patching
hal_status_t hal_rtl_gpio_irq_debounce_enable_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter, uint32_t debounce_us)
{
	uint32_t i;
	uint32_t bit_mask;
	uint32_t debounce_cyc;
	uint32_t *pdeb_cfg;
	uint32_t *pirq_cfg;
	hal_status_t ret = HAL_NO_RESOURCE;
	uint8_t gpio_gp_sel; // GPIO Group Select Signal. For ProII, GPIOA-F and S will be '0', except GPIOE5 (Pin 6) to E10 and GPIOS will be '1'.

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);
	uint8_t pin_idx = PIN_NAME_2_PIN(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if ((_paon_gpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: AON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
		gpio_gp_sel = 0; // for DEB SEL reg

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E) {
		// SYSON GPIO
		if ((_pgpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
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
		if ((_pgpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: SYSON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}

		gpio_gp_sel = 1; // for DEB SEL reg - WARN!!!! GPIO Group Sel is 1 (Group B/Port B) - THIS IS GPIOS

	} else if (port_idx == PORT_F) {
		// PON GPIO
		if ((_ppon_gpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
			DBG_GPIO_ERR("hal_gpio_irq_deb_en: PON GPIO Common not initialized\r\n");
			return HAL_NOT_READY;
		}
		gpio_gp_sel = 0; // for DEB SEL reg
	}

	if (port_idx == PORT_A) {
		// AON GPIO
		// to find a free debounce function block first
		for (i = 0; i < AON_GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_paon_gpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_paon_gpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)paon_gpio_debounce_cfg_ram_patch[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC)); // old

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((
								  debounce_cyc) << GPIO_SHIFT_DEB_CYC));  // new
				AON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)paon_gpio_irq_cfg_ram_patch[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		// SYSON GPIO
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_pgpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_pgpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)pgpio_debounce_cfg_ram_patch[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));
				SYSON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)pgpio_irq_cfg_ram_patch[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}
	} else if (port_idx == PORT_F) {
		// PON GPIO
		for (i = 0; i < GPIO_MAX_DEBOUNCE_PIN; i++) {
			if ((_ppon_gpio_comm_adp_ram_patch->gpio_deb_using & (1 << i)) == 0) {
				// found a free debounce function block
				bit_mask = 1 << i;
				_ppon_gpio_comm_adp_ram_patch->gpio_deb_using |= bit_mask;
				pgpio_irq_adapter->debounce_idx = i;
				// configure debounce function
				debounce_cyc = debounce_us * 32 / 1000;
				if (debounce_cyc == 0) {
					debounce_cyc = 1;
				} else if (debounce_cyc > 16383) {
					debounce_cyc = 16383;   // cycle count only has 14 bits
				}


				pdeb_cfg = (uint32_t *)ppon_gpio_debounce_cfg_ram_patch[i];
				//*pdeb_cfg = (PIN_NAME_2_PIN (pgpio_irq_adapter->ip_pin_name) | ((PIN_NAME_2_PORT (pgpio_irq_adapter->ip_pin_name) & 0x03) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));

				// Weide new code:
				*pdeb_cfg &= ~(GPIO_MASK_DEB_CYC); // clear debounce duration before setting new one
				*pdeb_cfg |= (PIN_NAME_2_PIN(pgpio_irq_adapter->ip_pin_name) | ((gpio_gp_sel & 0x01) << GPIO_SHIFT_DEB_GP_SEL) | ((debounce_cyc) << GPIO_SHIFT_DEB_CYC));
				PON_GPIO->GPIO_DEB_EN |= bit_mask;

				// configure GPIO IRQ to use this debounce as interrupt trigger source
				pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg_ram_patch[pgpio_irq_adapter->int_idx];
				*pirq_cfg |= i << GPIO_SHIFT_INT_DEB_SEL | 1 << GPIO_SHIFT_INT_SUR_SEL;

				ret = HAL_OK;
				break;
			}
		}
	}

	return ret;
}


// Function definition exactly same as ROM - appears here because IRQ-related functions need patching
void hal_rtl_gpio_irq_debounce_disable_patch(phal_gpio_irq_adapter_t pgpio_irq_adapter)
{
	uint32_t int_idx;
	uint32_t deb_idx;
	uint32_t bit_mask;
	uint32_t *pirq_cfg;

	uint8_t port_idx = PIN_NAME_2_PORT(pgpio_irq_adapter->pin_name);

	if (port_idx == PORT_A) {
		// AON GPIO
		if ((_paon_gpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
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
		pirq_cfg = (uint32_t *)paon_gpio_irq_cfg_ram_patch[int_idx];


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
		_paon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_B || port_idx == PORT_C || port_idx == PORT_D || port_idx == PORT_E || port_idx == PORT_S) {
		if ((_pgpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
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
		pirq_cfg = (uint32_t *)pgpio_irq_cfg_ram_patch[int_idx];


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
		_pgpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;

	} else if (port_idx == PORT_F) {
		if ((_ppon_gpio_comm_adp_ram_patch == NULL) || (pgpio_irq_adapter == NULL)) {
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
		pirq_cfg = (uint32_t *)ppon_gpio_irq_cfg_ram_patch[int_idx];


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
		_ppon_gpio_comm_adp_ram_patch->gpio_deb_using &= ~bit_mask;
		pgpio_irq_adapter->debounce_idx = 0xFF;
	}

}

void hal_rtl_gpio_comm_init_patch(phal_gpio_comm_adapter_t pgpio_comm_adp)
{
	pgpio_comm_adp->gpio_irq_using = 0;
	pgpio_comm_adp->gpio_irq_list_head = NULL;
	pgpio_comm_adp->gpio_irq_list_tail = NULL;

	_pgpio_comm_adp_ram_patch = pgpio_comm_adp;

	hal_irq_set_priority(GPIO_IRQn, GPIO_IRQPri);
	hal_rtl_gpio_reg_irq_patch(SysonGPIO, (irq_handler_t)&GPIO_IRQHandler_ram_patch);
}

void hal_rtl_aon_gpio_comm_init_patch(phal_aon_gpio_comm_adapter_t paon_gpio_comm_adp)
{
	paon_gpio_comm_adp->gpio_irq_using = 0;
	paon_gpio_comm_adp->gpio_irq_list_head = NULL;
	paon_gpio_comm_adp->gpio_irq_list_tail = NULL;

	_paon_gpio_comm_adp_ram_patch = paon_gpio_comm_adp;

	hal_irq_set_priority(AON_IRQn, AON_IRQPri);
	hal_rtl_gpio_reg_irq_patch(AonGPIO, (irq_handler_t)&AON_IRQHandler_ram_patch);
}

void hal_rtl_pon_gpio_comm_init_patch(phal_pon_gpio_comm_adapter_t ppon_gpio_comm_adp)
{
	ppon_gpio_comm_adp->gpio_irq_using = 0;
	ppon_gpio_comm_adp->gpio_irq_list_head = NULL;
	ppon_gpio_comm_adp->gpio_irq_list_tail = NULL;

	_ppon_gpio_comm_adp_ram_patch = ppon_gpio_comm_adp;

	hal_irq_set_priority(PonGPIO_IRQn, PonGPIO_IRQPri);
	hal_rtl_gpio_reg_irq_patch(PonGPIO, (irq_handler_t)&PonGPIO_IRQHandler_ram_patch);
}

void hal_rtl_gpio_comm_deinit_patch(void)
{
	if (_pgpio_comm_adp_ram_patch == NULL) {
		return;
	}

	hal_sys_peripheral_en(GPIO_SYS, DISABLE);

	_pgpio_comm_adp_ram_patch = NULL;
}

void hal_rtl_aon_gpio_comm_deinit_patch(void)
{
	if (_paon_gpio_comm_adp_ram_patch == NULL) {
		return;
	}

	hal_sys_peripheral_en(GPIO_AON, DISABLE);

	_paon_gpio_comm_adp_ram_patch = NULL;
}

void hal_rtl_pon_gpio_comm_deinit_patch(void)
{
	if (_ppon_gpio_comm_adp_ram_patch == NULL) {
		return;
	}

	hal_sys_peripheral_en(GPIO_PON, DISABLE);

	_ppon_gpio_comm_adp_ram_patch = NULL;
}

#endif

/** @} */ /* End of group hal_gpio */
