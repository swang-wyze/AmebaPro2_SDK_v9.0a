/**************************************************************************//**
 * @file     rtl8735b_crypto_patch.c
 * @brief    This file implements the CRYPTO Secure and Non-secure HAL patch rom functions in RAM.
 *
 * @version  V1.00
 * @date     2021-06-23
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
#include "hal_cache.h"
#include "hal_crypto.h"
#include "rtl8735b_crypto.h"
#include "rtl8735b_crypto_type.h"

#if CONFIG_CRYPTO_EN

#if !defined(CONFIG_BUILD_NONSECURE)

extern hal_crypto_func_stubs_t hal_crypto_stubs_s;

#define CRC_104_DMA_PATCH         (1<<7)
#define HAL_CRYPTO_FUNC_STUBS_PATCH   (hal_crypto_stubs_s)
#endif

/**

        \addtogroup hal_crypto
        @{
*/

#if IS_CUT_TEST(CONFIG_CHIP_VER)

#if !defined(CONFIG_BUILD_NONSECURE)
uint8_t hal_rtl_crypto_crc_isbusy_patch(hal_crypto_adapter_t *pcrypto_adapter)
{
	CRYPTO_Type *pcrypto;
	pcrypto = pcrypto_adapter->base_addr;
	volatile uint8_t regValue;

	regValue = pcrypto->crc_stat_reg_b.busy;

	return regValue;
}

void hal_rtl_crypto_crc_set_dma_patch(hal_crypto_adapter_t *pcrypto_adapter, u8 enabled)
{
	CRYPTO_Type *pcrypto;
	pcrypto = pcrypto_adapter->base_addr;
	volatile uint32_t regValue;

	regValue = pcrypto->crc_op_reg;

	if (enabled) {
		regValue |= CRC_104_DMA_PATCH;
	} else {
		regValue &= ~((uint32_t)(CRC_104_DMA_PATCH));
	}

	pcrypto->crc_op_reg = regValue;
}

void hal_rtl_crypto_crc_set_swap_patch(hal_crypto_adapter_t *pcrypto_adapter, u8 iswap_type, u8 oswap)
{
	CRYPTO_Type *pcrypto;
	pcrypto = pcrypto_adapter->base_addr;
	volatile uint32_t regValue;

	regValue = pcrypto->crc_op_reg;

	//
	regValue &= ~((uint32_t)(0xF));
	regValue |= ((uint32_t)(iswap_type)) & 0x7;
	regValue |= ((uint32_t)(oswap & 0x1)) << 3;
	//

	pcrypto->crc_op_reg = regValue;
}

void hal_rtl_crypto_crc_set_data_length_patch(hal_crypto_adapter_t *pcrypto_adapter, uint32_t data_len)
{
	CRYPTO_Type *pcrypto;
	pcrypto = pcrypto_adapter->base_addr;
	volatile uint32_t regValue;

	regValue = pcrypto->crc_op_reg;

	regValue &= ~((uint32_t)0xFFFFF000);

	if (data_len != 0) { // not last
		regValue |= data_len << 16;
	} else {
		regValue |= 0x1000;
	}

	//dbg_printf("pcrypto->crc_op_reg %x\r\n", regValue);
	pcrypto->crc_op_reg = regValue;
}

void hal_rtl_crypto_read_crc_setting_patch(hal_crypto_adapter_t *pcrypto_adapter)
{
	CRYPTO_Type *pcrypto;
	pcrypto = pcrypto_adapter->base_addr;
	volatile uint32_t regValue;

	regValue = pcrypto->crc_op_reg;
	DBG_CRYPTO_INFO("[0x104] : 0x%x \r\n", regValue);

	pcrypto_adapter->crc_sel = (regValue >> 4) & 0x7;
	pcrypto_adapter->crc_oswap = (regValue >> 3) & 0x1;
	pcrypto_adapter->crc_iswap = (regValue) & 0x7;

	pcrypto_adapter->crc_oxor = pcrypto->crc_oxor_reg;
	pcrypto_adapter->crc_init = pcrypto->crc_iv_reg;
	pcrypto_adapter->crc_poly = pcrypto->crc_poly_reg;
#if 1
	DBG_CRYPTO_INFO("sel: 0x%x \r\n", pcrypto_adapter->crc_sel);
	DBG_CRYPTO_INFO("poly: 0x%x \r\n", pcrypto_adapter->crc_poly);
	DBG_CRYPTO_INFO("init: 0x%x \r\n", pcrypto_adapter->crc_init);
	DBG_CRYPTO_INFO("oxor: 0x%x \r\n", pcrypto_adapter->crc_oxor);
	DBG_CRYPTO_INFO("iswap: 0x%x \r\n", pcrypto_adapter->crc_iswap);
	DBG_CRYPTO_INFO("oswap: 0x%x \r\n", pcrypto_adapter->crc_oswap);
#endif
}


int hal_rtl_crypto_crc_dma_patch(hal_crypto_adapter_t *pcrypto_adapter, IN const u8 *message, IN const uint32_t msglen)
{
	CRYPTO_Type *pcrypto;
	volatile uint32_t regValue;
	int ret = SUCCESS;
	uint32_t cache_len = (msglen & 0x1F) ? ((msglen / 32) + 1) * 32 : msglen;
	int8_t first_cache_pos = 0;
	int8_t first_cache_len = 0;
	int8_t is_cmdpreprocess = 0;
	uint32_t tmp_crcinit;
	uint32_t last_msglen;


	if (pcrypto_adapter == NULL) {
		return _ERRNO_CRYPTO_NULL_POINTER;
	}
	if (pcrypto_adapter->isInit != _TRUE) {
		return _ERRNO_CRYPTO_ENGINE_NOT_INIT; // not init yet
	}
	if ((pcrypto_adapter->base_addr) == NULL) {
		return _ERRNO_CRYPTO_NULL_POINTER;
	} else {
		pcrypto = pcrypto_adapter->base_addr;
	}

	// check if busy
	if (hal_rtl_crypto_crc_isbusy_patch(pcrypto_adapter)) {
		DBG_CRYPTO_ERR("[CRC_DMA] CRC engine is busy\r\n");
		return FAIL;
	}



	/***********************************************
	* Note:
	* DMA mode need to check whether is 32bytes alignment, because of cache clean.
	* - If not-32bytes alignment addr,but the space in cache line is enough for message,
	*   then copy message to a 32bytes aligment addr and jump to dma_process.
	* - If not-32bytes alignment addr,and the space in cache line isn't enough for message,
	*   then do cmd mode preporcess, after jump to dma_process.
	* - If 32bytes alignment addr, do dma_proces.
	*
	************************************************/
	first_cache_pos = (uint32_t)message & 0x1F;
	first_cache_len = (32 - first_cache_pos) % 32;

	if (first_cache_pos) {
		if (first_cache_len >= msglen) {
			DBG_CRYPTO_INFO("[CRC_DMA] msg_addr not 32 aligned & cache_len >= msglen\r\n");
			memcpy((void *) & (pcrypto_adapter->crc_first_cache_line[0]), (const void *)message, msglen);
			message = (const u8 *) & (pcrypto_adapter->crc_first_cache_line[0]);
			goto dma_process;
		} else {
			//cmd mode preporcess
			is_cmdpreprocess = TRUE;
			hal_rtl_crypto_read_crc_setting_patch(pcrypto_adapter);

			//reset_crc_setting for cmd(refout,crcxor)
			pcrypto->crc_oxor_reg = (uint32_t)0x0;
			hal_rtl_crypto_crc_set_swap_patch(pcrypto_adapter, pcrypto_adapter->crc_iswap, 0x0);
			regValue = pcrypto->crc_op_reg;
			DBG_CRYPTO_INFO("[0x104] : 0x%x \r\n", regValue);

			// check if busy
			if (hal_rtl_crypto_crc_isbusy_patch(pcrypto_adapter)) {
				DBG_CRYPTO_ERR("[CRC_DMA] After set CRC, CRC engine is busy\r\n");
				return FAIL;
			}

			ret = HAL_CRYPTO_FUNC_STUBS_PATCH.hal_crypto_crc_cmd(pcrypto_adapter, message, first_cache_len);
			if (ret == SUCCESS) {
				tmp_crcinit = HAL_CRYPTO_FUNC_STUBS_PATCH.hal_crypto_crc_get_result(pcrypto_adapter);
				message += first_cache_len;
				last_msglen = msglen;
				last_msglen -= first_cache_len;
				cache_len = (last_msglen & 0x1F) ? ((last_msglen / 32) + 1) * 32 : last_msglen;
				DBG_CRYPTO_INFO("[CRC_DMA] last_msglen =%d, cache_len = %d\r\n", last_msglen, cache_len);

				// check if busy
				if (hal_rtl_crypto_crc_isbusy_patch(pcrypto_adapter)) {
					DBG_CRYPTO_ERR("[CRC_DMA] After CRC_CMD_preprocess, CRC engine is busy\r\n");
					return FAIL;
				}
				//reset_crc_setting for dma(refout,crcxor,crcinit)
				DBG_CRYPTO_INFO("[CRC_DMA] last_crcinit = 0x%x \r\n", tmp_crcinit);
				//iv
				pcrypto->crc_iv_reg = tmp_crcinit;

				//oxor
				pcrypto->crc_oxor_reg = (pcrypto_adapter->crc_oxor);
				hal_rtl_crypto_crc_set_swap_patch(pcrypto_adapter, (pcrypto_adapter->crc_iswap), (pcrypto_adapter->crc_oswap));

				regValue = pcrypto->crc_op_reg;
				DBG_CRYPTO_INFO("[0x104] : 0x%x \r\n", regValue);

				goto dma_process;
			} else {
				DBG_CRYPTO_ERR("[CRC_DMA] CRC_CMD_preprocess fail!\r\n");
				return (ret);
			}
		}
	} else {

dma_process:

		hal_rtl_crypto_crc_set_dma_patch(pcrypto_adapter, 1);

		// set data length
		if (is_cmdpreprocess == TRUE) {
			DBG_CRYPTO_INFO("[CRC_DMA] last_msglen = %d\r\n", last_msglen);
			hal_rtl_crypto_crc_set_data_length_patch(pcrypto_adapter, last_msglen);
		} else {
			hal_rtl_crypto_crc_set_data_length_patch(pcrypto_adapter, msglen);
		}

		// Need 32 bytes alignment address
		dcache_clean_by_addr((uint32_t)(message), cache_len);
		pcrypto->crc_data_reg = (uint32_t)(message);

		pcrypto->crc_stat_reg_b.crc_ok = 1;
		pcrypto->crc_stat_reg_b.crc_intr_mask = 0;
		pcrypto->crc_stat_reg_b.crc_little_endian = 1;
		pcrypto->crc_rst_reg = 0x1;

		regValue = pcrypto->crc_stat_reg;
	}
	return ret;
}
#endif
#endif

#endif
/** @} */ /* End of group hal_crypto */
