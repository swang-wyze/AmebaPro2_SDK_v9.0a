/**************************************************************************//**
 * @file     rtl8710C_crypto.c
 * @brief    This file implements the CRYPTO Secure and Non-secure HAL functions in ROM.
 *
 * @version  V1.00
 * @date     2020-08-20
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
#include "rtl8735b_ecdsa.h"

#if CONFIG_ECDSA_EN

#define SECTION_ECDSA_TEXT           SECTION(".rom.hal_ecdsa.text")
#define SECTION_ECDSA_DATA           SECTION(".rom.hal_ecdsa.data")
#define SECTION_ECDSA_RODATA         SECTION(".rom.hal_ecdsa.rodata")
#define SECTION_ECDSA_BSS            SECTION(".rom.hal_ecdsa.bss")
#define SECTION_ECDSA_STUBS          SECTION(".rom.hal_ecdsa.stubs")

/**
 * @addtogroup hal_rtl_ecdsa ECDSA
 * @{
 */

/**
  * @brief The global common data structure to store common resource
  *        for ECDSA adapters.
  */
SECTION_ECDSA_BSS hal_ecdsa_group_adapter_t ecdsa_group_adapter;

/**
 * @addtogroup hal_rtl_ecdsa_func ECDSA HAL ROM APIs.
 * @ingroup hal_ecdsa
 * @{
 * @brief The ECDSA HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of ECDSA HAL APIs in the RAM space is provided for the user application.
 */

/**
  * @brief The stubs functions table to exports ECDSA HAL functions in ROM.
  */
SECTION_ECDSA_STUBS const hal_ecdsa_func_stubs_t hal_ecdsa_stubs = {
	.pecdsa_group_adapter = &ecdsa_group_adapter,
	.hal_ecdsa_irq_reg = hal_rtl_ecdsa_irq_reg,
	.hal_ecdsa_irq_unreg = hal_rtl_ecdsa_irq_unreg,
	.hal_ecdsa_init_clk_ctrl = hal_rtl_ecdsa_init_clk_ctrl,
	.hal_ecdsa_deinit_clk_ctrl = hal_rtl_ecdsa_deinit_clk_ctrl,
	.hal_ecdsa_init = hal_rtl_ecdsa_init,
	.hal_ecdsa_deinit = hal_rtl_ecdsa_deinit,
	.hal_ecdsa_reset = hal_rtl_ecdsa_reset,
	.hal_ecdsa_clr_finish_int = hal_rtl_ecdsa_clr_finish_int,
	.hal_ecdsa_mask_finish_int = hal_rtl_ecdsa_mask_finish_int,
	.hal_ecdsa_set_curve = hal_rtl_ecdsa_set_curve,
	.hal_ecdsa_set_mode = hal_rtl_ecdsa_set_mode,
	.hal_ecdsa_set_bit_num = hal_rtl_ecdsa_set_bit_num,
	.hal_ecdsa_start_en = hal_rtl_ecdsa_start_en,
	.hal_ecdsa_hash_en = hal_rtl_ecdsa_hash_en,
	.hal_ecdsa_hash_256_en = hal_rtl_ecdsa_hash_256_en,
	.hal_ecdsa_select_prk = hal_rtl_ecdsa_select_prk,
	.hal_ecdsa_set_prk = hal_rtl_ecdsa_set_prk,
	.hal_ecdsa_set_random_k = hal_rtl_ecdsa_set_random_k,
	.hal_ecdsa_set_pbk = hal_rtl_ecdsa_set_pbk,
	.hal_ecdsa_set_hash = hal_rtl_ecdsa_set_hash,
	.hal_ecdsa_set_rs = hal_rtl_ecdsa_set_rs,
	.hal_ecdsa_set_base_point = hal_rtl_ecdsa_set_base_point,
	.hal_ecdsa_set_base_point_2 = hal_rtl_ecdsa_set_base_point_2,
	.hal_ecdsa_set_cor_a = hal_rtl_ecdsa_set_cor_a,
	.hal_ecdsa_set_prime = hal_rtl_ecdsa_set_prime,
	.hal_ecdsa_set_order_n = hal_rtl_ecdsa_set_order_n,
	.hal_ecdsa_get_result_x_y = hal_rtl_ecdsa_get_result_x_y,
	.hal_ecdsa_get_veri_result = hal_rtl_ecdsa_get_veri_result,
	.hal_ecdsa_get_veri_err_sta = hal_rtl_ecdsa_get_veri_err_sta,
	.hal_ecdsa_get_finish_sta = hal_rtl_ecdsa_get_finish_sta,
	.hal_ecdsa_get_pbk = hal_rtl_ecdsa_get_pbk,
	.hal_ecdsa_get_rs = hal_rtl_ecdsa_get_rs,
	.hal_ecdsa_get_sign_err_sta = hal_rtl_ecdsa_get_sign_err_sta,
	.hal_ecdsa_get_scal_mul_err_sta = hal_rtl_ecdsa_get_scal_mul_err_sta,
	.hal_ecdsa_get_err_sta = hal_rtl_ecdsa_get_err_sta,
	.hal_ecdsa_get_idel_sta = hal_rtl_ecdsa_get_idle_sta,
	.hal_ecdsa_verify = hal_rtl_ecdsa_verify,
	.hal_ecdsa_signature = hal_rtl_ecdsa_signature,
	.hal_ecdsa_hash = hal_rtl_ecdsa_hash,
	.hal_ecdsa_gen_public_key = hal_rtl_ecdsa_gen_public_key,
	.hal_ecdsa_cb_handler = hal_rtl_ecdsa_cb_handler,
};

/**
 *  @brief This is the common interrupt handler for ECDSA.
 *         It read the system register to know the interrupt event
 *         is triggered by which ECDSA device.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void ECDSA_IRQHandler(void)
{
	hal_ecdsa_adapter_t *pecdsa_adapter;

	hal_rtl_irq_clear_pending(ECDSA_IRQn);

	pecdsa_adapter = ecdsa_group_adapter.pecdsa_adapter[ECDSA_id];

	hal_rtl_ecdsa_clr_finish_int(pecdsa_adapter);
	//-------------------------------------------
	//Input callback function
	if (pecdsa_adapter->ecdsa_irq_user_cb != NULL) {
		pecdsa_adapter->ecdsa_irq_user_cb(pecdsa_adapter->ecdsa_user_arg);
	}
	//-------------------------------------------

}

/**
 *  @brief To register a IRQ handler for ECDSA.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_irq_reg(void)
{
	hal_rtl_irq_disable(ECDSA_IRQn);
	__ISB();

	// Register ECDSA common IRQ handler
	hal_rtl_irq_set_vector(ECDSA_IRQn, (uint32_t)ECDSA_IRQHandler);
	hal_rtl_irq_clear_pending(ECDSA_IRQn);
	hal_rtl_irq_enable(ECDSA_IRQn);
}

/**
 *  @brief To un-register the ECDSA IRQ handler.
 *
 *  @param[in]  pecdsa_adapter The ECDSA adapter.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_irq_unreg(void)
{
	// No any ECDSA port has IRQ handler, so disable the common interrupt
	hal_rtl_irq_disable(ECDSA_IRQn);
	__ISB();
}

/**
 *  @brief Init ECDSA clock control.
 *
 *  @param[in]  pecdsa_adapter The ECDSA adapter.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_init_clk_ctrl(hal_ecdsa_adapter_t *pecdsa_adapter)
{
#if !defined(CONFIG_BUILD_NONSECURE)
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	hal_rtl_sys_peripheral_en(ECDSA_SYS, ENABLE);
	pecdsa_reg->ECDSA_ECR1_REG |= ECDSA_BIT_ENG_CLK_ENABLE;
	hal_rtl_ecdsa_reset(pecdsa_adapter);
#endif
}

/**
 *  @brief De-init ECDSA clock control.
 *
 *  @param[in]  pecdsa_adapter The ECDSA adapter.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_deinit_clk_ctrl(hal_ecdsa_adapter_t *pecdsa_adapter)
{
#if !defined(CONFIG_BUILD_NONSECURE)
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	hal_rtl_ecdsa_reset(pecdsa_adapter);
	pecdsa_reg->ECDSA_ECR1_REG &= ~(ECDSA_BIT_ENG_CLK_ENABLE);
	hal_rtl_sys_peripheral_en(ECDSA_SYS, DISABLE);
#endif
}

/**
 *  @brief Initialize the ECDSA hardware and turn on the ECDSA
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @return     HAL_ERR_PARA
 *  @return     HAL_OK
 */
SECTION_ECDSA_TEXT
HAL_Status hal_rtl_ecdsa_init(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint8_t i;
	uint8_t result = 0x00;

	//Check if ECDSA has been registered.
	for (i = 0; i < ECDSA_Max_Id; i++) {
		if (ecdsa_group_adapter.pecdsa_adapter[i] != NULL) {
			DBG_ECDSA_ERR("The ecdsa had been registered. \r\n");
			result = 1;
			break;
		}
	}

	if (result == 0x00) {

		hal_rtl_ecdsa_irq_reg();

		//Set IRQ priority
		hal_rtl_irq_set_priority(ECDSA_IRQn, ECDSA_IRQPri);

		pecdsa_adapter->base_addr = (ECDSA_TypeDef *)ECDSA;
		ecdsa_group_adapter.pecdsa_adapter[ECDSA_id] = pecdsa_adapter;

		hal_rtl_ecdsa_init_clk_ctrl(pecdsa_adapter);

		return HAL_OK;
	} else {

		return HAL_ERR_PARA;
	}

}

/**
 *  @brief De-initialize of the ECDSA hardware and turn off the ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @return     HAL_ERR_PARA
 *  @return     HAL_OK
 */
SECTION_ECDSA_TEXT
HAL_Status hal_rtl_ecdsa_deinit(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	if (pecdsa_adapter == 0) {
		DBG_ECDSA_ERR("hal_rtl_ecdsa_deinit: Null Pointer\r\n");
		return HAL_ERR_PARA;
	}

	/*ECDSA Interrupt DeInitialization*/
	hal_rtl_ecdsa_irq_unreg();

	hal_rtl_ecdsa_deinit_clk_ctrl(pecdsa_adapter);

	ecdsa_group_adapter.pecdsa_adapter[ECDSA_id] = NULL;

	return HAL_OK;
}

/**
 *  @brief Reset the ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_reset(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	pecdsa_reg->ECDSA_ECR1_REG |= ECDSA_BIT_ENG_RST;
	pecdsa_reg->ECDSA_ECR1_REG &= ~(ECDSA_BIT_ENG_RST);
}

/**
 *  @brief Clear the finish interrupt of ECDSA
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_clr_finish_int(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	pecdsa_reg->ECDSA_EISR_REG |= ECDSA_BIT_FINISH_INT;
}

/**
 *  @brief Mask the finish interrupt of ECDSA
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_mask_finish_int(hal_ecdsa_adapter_t *pecdsa_adapter, uint8_t enable)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	if (enable == 1) {
		pecdsa_reg->ECDSA_ECR2_REG |= ECDSA_BIT_FINISH_INT_MASK;
	} else {
		pecdsa_reg->ECDSA_ECR2_REG &= ~(ECDSA_BIT_FINISH_INT_MASK);
	}
}

/**
 *  @brief Set the ECDSA curve.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] curve Select an elliptic curve.
 *  @param[in] pcurve_table Set the curve table address.
 *  @param[in] bit_num Select the bit length.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_curve(hal_ecdsa_adapter_t *pecdsa_adapter, ecdsa_curve_t curve, hal_ecdsa_curve_table_t *pcurve_table, ecdsa_bit_num_t bit_num)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t temp_reg;

	temp_reg = pecdsa_reg->ECDSA_ECR2_REG;

	temp_reg &= ~(ECDSA_MASK_CURVE_SEL);

	temp_reg |= (curve << ECDSA_SHIFT_CURVE_SEL);

	pecdsa_reg->ECDSA_ECR2_REG = temp_reg;

	if (curve == ECDSA_OTHERS) {

		if (pcurve_table != NULL) {
			hal_rtl_ecdsa_set_base_point(pecdsa_adapter, pcurve_table->ppoint_x, pcurve_table->ppoint_y);

			hal_rtl_ecdsa_set_cor_a(pecdsa_adapter, pcurve_table->pa_adr);

			hal_rtl_ecdsa_set_prime(pecdsa_adapter, pcurve_table->pprime);

			hal_rtl_ecdsa_set_order_n(pecdsa_adapter, pcurve_table->porder_n);
		} else {
			DBG_ECDSA_ERR("Need to set the curve parameter. \r\n");
		}

		hal_rtl_ecdsa_set_bit_num(pecdsa_adapter, bit_num);
	}

}

/**
 *  @brief Set the ECDSA mode.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] mode Select an operation mode.
 *  @param[in] func If the operating mode is ECDSA_BASIC_FUNC, need to set this operation unit.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_mode(hal_ecdsa_adapter_t *pecdsa_adapter, ecdsa_mode_t mode, ecdsa_basic_func_t func)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t temp_reg;

	if (func == ECDSA_NONE) {
		func = ECDSA_FIELD_SUB;
	}

	temp_reg = pecdsa_reg->ECDSA_ECR2_REG;

	temp_reg &= ~(ECDSA_MASK_MODE_SEL | ECDSA_MASK_FUNC_SEL);

	temp_reg |= (mode << ECDSA_SHIFT_MODE_SEL) | (func << ECDSA_SHIFT_FUNC_SEL);

	pecdsa_reg->ECDSA_ECR2_REG = temp_reg;
}

/**
 *  @brief Set the basic function of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] bit_num Select the bit length.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_bit_num(hal_ecdsa_adapter_t *pecdsa_adapter, ecdsa_bit_num_t bit_num)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t temp_reg;
	uint32_t extend_temp;

	if ((bit_num & 0x8) == 0x00) {
		extend_temp = 0;
	} else {
		extend_temp = 1;
	}

	bit_num &= 0x7;

	temp_reg = pecdsa_reg->ECDSA_ECR2_REG;

	temp_reg &= ~(ECDSA_MASK_NUM | ECDSA_BIT_EXTEND_N);

	temp_reg |= (bit_num << ECDSA_SHIFT_NUM) | (extend_temp << ECDSA_SHIFT_EXTEND_N);

	pecdsa_reg->ECDSA_ECR2_REG = temp_reg;

}

/**
 *  @brief To enable or disable the ECDSA engine.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] enable Enable control: 0 is disable, 1 is enable.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_start_en(hal_ecdsa_adapter_t *pecdsa_adapter, uint8_t enable)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	if (enable == 1) {
		pecdsa_reg->ECDSA_EISR_REG |= ECDSA_BIT_ENG_START;
	} else {
		pecdsa_reg->ECDSA_EISR_REG &= ~(ECDSA_BIT_ENG_START);
	}
}

/**
 *  @brief To enable or disable the ECDSA hask ok.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] enable Enable control: 0 is that hash is ready, 1 is that hash is not ready.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_hash_en(hal_ecdsa_adapter_t *pecdsa_adapter, uint8_t enable)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	if (enable == 1) {
		pecdsa_reg->ECDSA_EISR_REG |= ECDSA_BIT_HASH_OK;
	} else {
		pecdsa_reg->ECDSA_EISR_REG &= ~(ECDSA_BIT_HASH_OK);
	}
}

/**
 *  @brief Force the hash length is 256 bits, and seed up the operation.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] enable Enable control: 0 is disable, 1 is enable.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_hash_256_en(hal_ecdsa_adapter_t *pecdsa_adapter, uint8_t enable)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	if (enable == 1) {
		pecdsa_reg->ECDSA_ECR2_REG |= ECDSA_BIT_HASH256_EN;
	} else {
		pecdsa_reg->ECDSA_ECR2_REG &= ~(ECDSA_BIT_HASH256_EN);
	}

}

/**
 *  @brief Select the private key of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] sel_prk Select the private key.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_select_prk(hal_ecdsa_adapter_t *pecdsa_adapter, ecdsa_sel_prk_t sel_prk)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t reg_temp;

	reg_temp = pecdsa_reg->ECDSA_ECR2_REG;
	reg_temp &= ~(ECDSA_MASK_PRI_KEY_SEL);
	reg_temp |= (sel_prk << ECDSA_SHIFT_PRI_KEY_SEL);
	pecdsa_reg->ECDSA_ECR2_REG = reg_temp;
}

/**
 *  @brief Set the private key of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppriv_key Set the private key.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_prk(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppriv_key)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;
	uint8_t no_load = 0;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_SIGN) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_X_P0_REG));
	} else if (mode_temp == ECDSA_SCALAR_MUL) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_H_P0_REG));
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the private key. \r\n");
		no_load = 1;
	}

	if (no_load == 0x00) {
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppriv_key + i);
		}
	}

}

/**
 *  @brief Set the random k of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] prdk Set the random k.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_random_k(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *prdk)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_SIGN) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_Y_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(prdk + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the random k. \r\n");
	}
}

/**
 *  @brief Set the public key of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppub_key_x Set the public key x.
 *  @param[in] ppub_key_y Set the public key y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_pbk(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppub_key_x, uint32_t *ppub_key_y)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_VERI) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_X_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppub_key_x + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_Y_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppub_key_y + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the public key. \r\n");
	}
}

/**
 *  @brief Set the hash of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] phash Set the hash.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_hash(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *phash)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if ((mode_temp == ECDSA_VERI) || (mode_temp == ECDSA_SIGN)) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_H_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(phash + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the hash. \r\n");
	}
}

/**
 *  @brief Set the R and S of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] pr_adr Set the R.
 *  @param[in] ps_adr Set the S.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_rs(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *pr_adr, uint32_t *ps_adr)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_VERI) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_VERIFY_R_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(pr_adr + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_VERIFY_S_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ps_adr + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need R and S. \r\n");
	}

}

/**
 *  @brief Set the base point of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppoint_x Set the base point X.
 *  @param[in] ppoint_y Set the base point y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_base_point(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppoint_x, uint32_t *ppoint_y)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if ((mode_temp == ECDSA_VERI) || (mode_temp == ECDSA_SIGN)) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_GX_PS0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_x + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_GY_PS0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_y + i);
		}
	} else if ((mode_temp == ECDSA_SCALAR_MUL) || (mode_temp == ECDSA_BASIC_FUNC)) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_X_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_x + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_Y_P0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_y + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the base point. \r\n");
	}

}

/**
 *  @brief Set the base point 2 of ECDSA for the basic mode.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppoint_x Set the base point2 X.
 *  @param[in] ppoint_y Set the base point2 y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_base_point_2(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppoint_x, uint32_t *ppoint_y)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_BASIC_FUNC) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_GX_PS0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_x + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_GY_PS0_REG));
		for (i = 0; i < 8; i++) {
			*(paddress_temp + i) = *(ppoint_y + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the base point 2. \r\n");
	}

}

/**
 *  @brief Set the "a" parameter of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] pa_adr Set the "a" parameter.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_cor_a(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *pa_adr)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;

	paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_A_P0_REG));
	for (i = 0; i < 8; i++) {
		*(paddress_temp + i) = *(pa_adr + i);
	}
}

/**
 *  @brief Set the prime of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] phash Set the prime.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_prime(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *pprime)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;

	paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_P_P0_REG));
	for (i = 0; i < 8; i++) {
		*(paddress_temp + i) = *(pprime + i);
	}
}

/**
 *  @brief Set the order_n of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] phash Set the order_n.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_set_order_n(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *porder_n)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;

	paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_N_P0_REG));
	for (i = 0; i < 8; i++) {
		*(paddress_temp + i) = *(porder_n + i);
	}
}

/**
 *  @brief Get the result x and y of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] prx_adr Get the result x.
 *  @param[in] pry_adr Get the result y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_get_result_x_y(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *prx_adr, uint32_t *pry_adr)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if ((mode_temp == ECDSA_VERI) || (mode_temp == ECDSA_BASIC_FUNC)) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_XO_P0_REG));
		for (i = 0; i < 8; i++) {
			*(prx_adr + i) = *(paddress_temp + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_YO_P0_REG));
		for (i = 0; i < 8; i++) {
			*(pry_adr + i) = *(paddress_temp + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need result x and y. \r\n");
	}

}

/**
 *  @brief Get the verification result of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The verification result
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_veri_result(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t r_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	r_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_VERIFY_PASS) >> ECDSA_SHIFT_VERIFY_PASS;

	return r_temp;
}

/**
 *  @brief Get the verification error status of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The verification error status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_veri_err_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t err_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_MASK_VERIFY_ERR) >> ECDSA_SHIFT_VERIFY_ERR;

	return err_temp;
}

/**
 *  @brief Get the infinity error status of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The infinity error status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_inf_err_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t err_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_INFINITY_KEY) >> ECDSA_SHIFT_INFINITY_KEY;

	return err_temp;
}

/**
 *  @brief Get the finish status of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The finish status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_finish_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t f_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	f_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_FINISH_INT) >> ECDSA_SHIFT_FINISH_INT;

	return f_temp;
}

/**
 *  @brief Get the public key of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppub_key_x Get the public key x.
 *  @param[in] ppub_key_y Get the public key y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_get_pbk(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppub_key_x, uint32_t *ppub_key_y)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if ((mode_temp == ECDSA_SIGN) || (mode_temp == ECDSA_SCALAR_MUL)) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_XO_P0_REG));
		for (i = 0; i < 8; i++) {
			*(ppub_key_x + i) = *(paddress_temp + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_YO_P0_REG));
		for (i = 0; i < 8; i++) {
			*(ppub_key_y + i) = *(paddress_temp + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need the public key. \r\n");
	}

}

/**
 *  @brief Get the R and S of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] pr_adr Get the R.
 *  @param[in] ps_adr Get the S.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_get_rs(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *pr_adr, uint32_t *ps_adr)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint32_t *paddress_temp;
	uint8_t i;
	uint8_t mode_temp;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	if (mode_temp == ECDSA_SIGN) {
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_SIGN_R_S0_REG));
		for (i = 0; i < 8; i++) {
			*(pr_adr + i) = *(paddress_temp + i);
		}
		paddress_temp = (uint32_t *)(&(pecdsa_reg->ECDSA_ENG_SIGN_S_S0_REG));
		for (i = 0; i < 8; i++) {
			*(ps_adr + i) = *(paddress_temp + i);
		}
	} else {
		DBG_ECDSA_WARN("This mode doesn't need R and S. \r\n");
	}

}

/**
 *  @brief Get the signature error status of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The signature error status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_sign_err_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t err_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_MASK_SIGN_ERR) >> ECDSA_SHIFT_SIGN_ERR;

	return err_temp;
}

/**
 *  @brief Get the scalar multiplication error status of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The scalar multiplication error status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_scal_mul_err_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t err_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_SHARE_KEY_ERR) >> ECDSA_SHIFT_SHARE_KEY_ERR;

	return err_temp;
}

/**
 *  @brief Get the error status of ECDSA according to different modes.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The error status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_err_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;
	uint8_t mode_temp;
	uint32_t err_temp, err_temp1;

	mode_temp = ((pecdsa_reg->ECDSA_ECR2_REG) & ECDSA_MASK_MODE_SEL) >> ECDSA_SHIFT_MODE_SEL;

	switch (mode_temp) {
	case ECDSA_VERI:
		err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_MASK_VERIFY_ERR) >> ECDSA_SHIFT_VERIFY_ERR;
		break;
	case ECDSA_SIGN:
		err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_MASK_SIGN_ERR) >> ECDSA_SHIFT_SIGN_ERR;
		break;
	case ECDSA_SCALAR_MUL:
		err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_SHARE_KEY_ERR) >> ECDSA_SHIFT_SHARE_KEY_ERR;
		break;
	default :
		err_temp = 0;
		break;
	}

	err_temp1 = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_INFINITY_KEY) >> ECDSA_SHIFT_INFINITY_KEY;

	err_temp = err_temp | (err_temp1 << 1);

	return err_temp;
}

/**
 *  @brief Get the error status of ECDSA according to different modes.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *
 *  @returns The idle status
 */
SECTION_ECDSA_TEXT
uint32_t hal_rtl_ecdsa_get_idle_sta(hal_ecdsa_adapter_t *pecdsa_adapter)
{
	uint32_t err_temp;

	ECDSA_TypeDef *pecdsa_reg = (ECDSA_TypeDef *)pecdsa_adapter->base_addr;

	err_temp = ((pecdsa_reg->ECDSA_EISR_REG) & ECDSA_BIT_ENG_IDLE) >> ECDSA_SHIFT_ENG_IDLE;

	return err_temp;
}

/**
 *  @brief Excute the verification of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] pveri_input Set verification inputs.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_verify(hal_ecdsa_adapter_t *pecdsa_adapter, hal_ecdsa_veri_input_t *pveri_input)
{
	hal_rtl_ecdsa_set_mode(pecdsa_adapter, ECDSA_VERI, ECDSA_NONE);

	hal_rtl_ecdsa_set_pbk(pecdsa_adapter, pveri_input->ppub_key_x, pveri_input->ppub_key_y);

	hal_rtl_ecdsa_set_rs(pecdsa_adapter, pveri_input->pr_adr, pveri_input->ps_adr);

	hal_rtl_ecdsa_start_en(pecdsa_adapter, ENABLE);
}

/**
 *  @brief Excute the signature of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppriv_key Set the private key.
 *  @param[in] prdk Set the random k.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_signature(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppriv_key, uint32_t *prdk)
{
	hal_rtl_ecdsa_set_mode(pecdsa_adapter, ECDSA_SIGN, ECDSA_NONE);

	hal_rtl_ecdsa_set_prk(pecdsa_adapter, ppriv_key);

	hal_rtl_ecdsa_set_random_k(pecdsa_adapter, prdk);

	hal_rtl_ecdsa_start_en(pecdsa_adapter, ENABLE);
}

/**
 *  @brief Set the hash is ready.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] phash Set the hash.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_hash(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *phash)
{
	hal_rtl_ecdsa_set_hash(pecdsa_adapter, phash);

	hal_rtl_ecdsa_hash_en(pecdsa_adapter, ENABLE);
}

/**
 *  @brief Generate the public key of ECDSA.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in] ppriv_key Set the private key.
 *  @param[in] ppoint_x Set the base point X.
 *  @param[in] ppoint_y Set the base point y.
 *
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_gen_public_key(hal_ecdsa_adapter_t *pecdsa_adapter, uint32_t *ppriv_key, uint32_t *ppoint_x, uint32_t *ppoint_y)
{
	hal_rtl_ecdsa_set_mode(pecdsa_adapter, ECDSA_SCALAR_MUL, ECDSA_NONE);

	hal_rtl_ecdsa_set_prk(pecdsa_adapter, ppriv_key);

	hal_rtl_ecdsa_set_base_point(pecdsa_adapter, ppoint_x, ppoint_y);

	hal_rtl_ecdsa_start_en(pecdsa_adapter, ENABLE);

}

/**
 *  @brief Register the ECDSA callback function and argument.
 *
 *  @param[in] pecdsa_adapter The ECDSA adapter.
 *  @param[in]  callback The callback function.
 *  @param[in]  arg The callback argument.
 *  @returns void
 */
SECTION_ECDSA_TEXT
void hal_rtl_ecdsa_cb_handler(hal_ecdsa_adapter_t *pecdsa_adapter, ecdsa_irq_user_cb_t callback, void *arg)
{
	if (callback != NULL) {
		pecdsa_adapter->ecdsa_irq_user_cb = callback;
		pecdsa_adapter->ecdsa_user_arg = arg;
	}
}

/** @} */ /* End of group hal_rtl_ecdsa_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hal_rtl_ecdsa */


#endif  // end of "#if CONFIG_ECDSA_EN"

