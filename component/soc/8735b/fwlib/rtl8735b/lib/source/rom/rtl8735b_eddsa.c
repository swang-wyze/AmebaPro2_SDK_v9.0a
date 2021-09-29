/**************************************************************************//**
 * @file     rtl8735b_eddsa.c
 * @brief    This file implements the EDDSA Secure and Non-secure HAL functions in ROM.
 *
 * @version  V1.00
 * @date     2021-06-11
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
#include "rtl8735b_eddsa.h"
#include "rtl8735b_eddsa_type.h"
#include "rtl8735b_eddsa_ctrl.h"
#include "rtl8735b_syson_s_type.h"
#include "rtl8735b.h"

#if CONFIG_EDDSA_EN
/**
 * @addtogroup hs_hal_eddsa EDDSA
 * @{
 */

/// @cond DOXYGEN_ROM_HAL_API

/// @endcond
/** @} */ /* End of group hs_hal_eddsa */

//
//

#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
#undef EDDSA_MODULE
#define EDDSA_MODULE (EDDSA_NS_MODULE)

#undef EDDSA_REG_BASE
#define EDDSA_REG_BASE (EDDSA_NS_BASE)

#else

#undef EDDSA_MODULE
#define EDDSA_MODULE (EDDSA_S_MODULE)

#undef EDDSA_REG_BASE
#define EDDSA_REG_BASE (EDDSA_S_BASE)
#endif

#define SECTION_EDDSA_TEXT           SECTION(".rom.hal_eddsa.text")
#define SECTION_EDDSA_DATA           SECTION(".rom.hal_eddsa.data")
#define SECTION_EDDSA_RODATA         SECTION(".rom.hal_eddsa.rodata")
#define SECTION_EDDSA_BSS            SECTION(".rom.hal_eddsa.bss")
#define SECTION_EDDSA_STUBS          SECTION(".rom.hal_eddsa.stubs")

extern void dump_for_one_bytes(u8 *pdata, u32 len);

#undef __dbg_mem_dump
#define __dbg_mem_dump(enable, start, size, prefix) do{ \
    if(enable){ \
        DBG_EDDSA_INFO(prefix "\r\n");\
        dump_for_one_bytes ((u8 *)start, size);\
    }\
}while(0)


SECTION_EDDSA_STUBS
const hal_eddsa_func_stubs_t hal_eddsa_stubs = {
	.hal_eddsa_init   = hal_rtl_eddsa_init,
	.hal_eddsa_deinit = hal_rtl_eddsa_deinit,
	.hal_eddsa_sign_verify = hal_rtl_eddsa_sign_verify,
	.hal_eddsa_sign_verify_autoflow = hal_rtl_eddsa_sign_verify_autoflow,
	.hal_eddsa_sign_verify_semi_auto = hal_rtl_eddsa_sign_verify_semi_auto,
	.hal_eddsa_sign_verify_steps = hal_rtl_eddsa_sign_verify_steps,
	.hal_eddsa_get_enc_Rplus = hal_rtl_eddsa_get_enc_Rplus,
	.hal_eddsa_get_Rplus = hal_rtl_eddsa_get_Rplus,
	.hal_eddsa_get_kA = hal_rtl_eddsa_get_kA,
	.hal_eddsa_get_sB = hal_rtl_eddsa_get_sB,
	.hal_eddsa_get_recover_pbk = hal_rtl_eddsa_get_recover_pbk,
	.hal_eddsa_set_hash_digest = hal_rtl_eddsa_set_hash_digest,
	.hal_eddsa_set_recover_pbk = hal_rtl_eddsa_set_recover_pbk,
	.hal_eddsa_set_encode_pbk = hal_rtl_eddsa_set_encode_pbk,
	.hal_eddsa_set_signature_s = hal_rtl_eddsa_set_signature_s,
	.hal_eddsa_set_word_from_byte = hal_rtl_eddsa_set_word_from_byte,
	.hal_eddsa_get_byte_from_word = hal_rtl_eddsa_get_byte_from_word,
	.hal_eddsa_set_x1_in_val = hal_rtl_eddsa_set_x1_in_val,
	.hal_eddsa_set_y1_in_val = hal_rtl_eddsa_set_y1_in_val,
	.hal_eddsa_set_x2_in_val = hal_rtl_eddsa_set_x2_in_val,
	.hal_eddsa_set_y2_in_val = hal_rtl_eddsa_set_y2_in_val,
	.hal_eddsa_get_x1_out_val = hal_rtl_eddsa_get_x1_out_val,
	.hal_eddsa_get_y1_out_val = hal_rtl_eddsa_get_y1_out_val,
	.hal_eddsa_get_x2_out_val = hal_rtl_eddsa_get_x2_out_val,
	.hal_eddsa_get_y2_out_val = hal_rtl_eddsa_get_y2_out_val,
	.hal_eddsa_verify_32 = hal_rtl_eddsa_verify_32,
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	.hal_eddsa_sign_verify_core = hal_rtl_eddsa_sign_verify_core,
	.hal_eddsa_hook_hash_op_f = hal_rtl_eddsa_hook_hash_op_f,
	.hal_eddsa_hash_init_hook = hal_rtl_eddsa_hash_init_hook,
	.hal_eddsa_hash_update_hook = hal_rtl_eddsa_hash_update_hook,
	.hal_eddsa_hash_final_hook = hal_rtl_eddsa_hash_final_hook,
	.hal_eddsa_hook_sign_vrf_hash_op_f = hal_rtl_eddsa_hook_sign_vrf_hash_op_f,
#endif
};

static inline void *eddsa_rom_memset(void *dst, int c, size_t n)
{
	if (n != 0) {
		unsigned char *d = dst;

		do {
			*d++ = (unsigned char)c;
		} while (--n > 0);
	}
	return dst;
}

/**
 * @addtogroup hs_hal_eddsa EDDSA
 * @{
 */

/// @cond DOXYGEN_ROM_HAL_API

#if IS_CUT_TEST(CONFIG_CHIP_VER)
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_init(phal_crypto_eddsa_t adtr)
{
#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
#else
	hal_rtl_sys_peripheral_en(EDDSA_SYS, ENABLE);
#endif
	return SUCCESS;
}

#else
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_init(phal_crypto_eddsa_t peddsa_adtr, hal_crypto_adapter_t *pcrypto_adapter, uint8_t hash_crypto_sel)
{
	int ret = SUCCESS;
	if (peddsa_adtr == NULL) {
		return HAL_ERR_PARA;
	}
	_memset(peddsa_adtr, 0x0, sizeof(hal_crypto_eddsa_t));
	peddsa_adtr->base_addr = EDDSA_MODULE;
	if (EDDSA_HASH_CRYPTO_HW_SEL_EN == hash_crypto_sel) {
		// init ceypto engine
		if (NULL == pcrypto_adapter) {
			return HAL_ERR_PARA;
		} else {
			if (_TRUE != pcrypto_adapter->isInit) {
				dbg_printf("EdDSA: HW Crypto not init yet!\r\n");
				return HAL_NOT_READY;
			} else {
				peddsa_adtr->pcrypto_adtr = pcrypto_adapter;
				peddsa_adtr->isHWCrypto_Init = _TRUE;
			}
		}
	}
	peddsa_adtr->hash_crypto_sel = hash_crypto_sel;
#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
#else
	hal_rtl_sys_peripheral_en(EDDSA_SYS, ENABLE);
#endif
	peddsa_adtr->isInit = _TRUE;
	peddsa_adtr->isMemDump = DISABLE;
	ret = hal_rtl_eddsa_hook_hash_op_f(peddsa_adtr, NULL, NULL, NULL);
	hal_rtl_eddsa_hook_sign_vrf_hash_op_f(peddsa_adtr, NULL);
	return ret;
}
#endif

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_hook_hash_op_f(hal_crypto_eddsa_t *peddsa_adtr, eddsa_hash_init_func_t hash_init_f,
								 eddsa_hash_update_func_t hash_update_f, eddsa_hash_final_func_t hash_final_f)
{
	int ret = SUCCESS;
	if (NULL == peddsa_adtr) {
		return HAL_ERR_PARA;
	}
	if (peddsa_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}

	hal_rtl_eddsa_hash_init_hook(peddsa_adtr, hash_init_f);
	hal_rtl_eddsa_hash_update_hook(peddsa_adtr, hash_update_f);
	hal_rtl_eddsa_hash_final_hook(peddsa_adtr, hash_final_f);

	return ret;
}

SECTION_EDDSA_TEXT
static int hal_rtl_eddsa_sha2_512_init(void *arg)
{
	hal_crypto_eddsa_t *peddsa_adtr = (hal_crypto_eddsa_t *)arg;
	int ret;
	if (NULL == peddsa_adtr) {
		return HAL_ERR_PARA;
	}
	if (EDDSA_HASH_CRYPTO_HW_SEL_EN == peddsa_adtr->hash_crypto_sel) {
		if (NULL == peddsa_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		ret = hal_rtl_crypto_auth_init(peddsa_adtr->pcrypto_adtr, AUTH_TYPE_SHA2_512_ALL, NULL, 0);
	}
	return ret;
}

SECTION_EDDSA_TEXT
static int hal_rtl_eddsa_sha2_512_update(void *arg, const uint8_t *message, const uint32_t msglen)
{
	hal_crypto_eddsa_t *peddsa_adtr = (hal_crypto_eddsa_t *)arg;
	int ret;
	if (NULL == peddsa_adtr) {
		return HAL_ERR_PARA;
	}
	if (EDDSA_HASH_CRYPTO_HW_SEL_EN == peddsa_adtr->hash_crypto_sel) {
		if (NULL == peddsa_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		ret = hal_rtl_crypto_auth_update(peddsa_adtr->pcrypto_adtr, AUTH_TYPE_SHA2_512_ALL, message, msglen);
	}
	return ret;
}

SECTION_EDDSA_TEXT
static int hal_rtl_eddsa_sha2_512_final(void *arg, uint8_t *pDigest)
{
	hal_crypto_eddsa_t *peddsa_adtr = (hal_crypto_eddsa_t *)arg;
	int ret;
	if (NULL == peddsa_adtr) {
		return HAL_ERR_PARA;
	}
	if (EDDSA_HASH_CRYPTO_HW_SEL_EN == peddsa_adtr->hash_crypto_sel) {
		if (NULL == peddsa_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		ret = hal_rtl_crypto_auth_final(peddsa_adtr->pcrypto_adtr, AUTH_TYPE_SHA2_512_ALL, pDigest);
	}
	return ret;
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_hash_init_hook(hal_crypto_eddsa_t *peddsa_adtr, eddsa_hash_init_func_t hash_init_f)
{
	if (NULL != hash_init_f) {
		peddsa_adtr->hash_init_f = hash_init_f;
	} else {
		if (EDDSA_HASH_CRYPTO_HW_SEL_EN == (peddsa_adtr->hash_crypto_sel)) {
			peddsa_adtr->hash_init_f = hal_rtl_eddsa_sha2_512_init;
		}
	}
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_hash_update_hook(hal_crypto_eddsa_t *peddsa_adtr, eddsa_hash_update_func_t hash_update_f)
{
	u16 hmac_alg_sel = 0;
	if (NULL != hash_update_f) {
		peddsa_adtr->hash_update_f = hash_update_f;
	} else {
		if (EDDSA_HASH_CRYPTO_HW_SEL_EN == (peddsa_adtr->hash_crypto_sel)) {
			peddsa_adtr->hash_update_f = hal_rtl_eddsa_sha2_512_update;
		}
	}
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_hash_final_hook(hal_crypto_eddsa_t *peddsa_adtr, eddsa_hash_final_func_t hash_final_f)
{
	u16 hmac_alg_sel = 0;
	if (NULL != hash_final_f) {
		peddsa_adtr->hash_final_f = hash_final_f;
	} else {
		if (EDDSA_HASH_CRYPTO_HW_SEL_EN == (peddsa_adtr->hash_crypto_sel)) {
			peddsa_adtr->hash_final_f = hal_rtl_eddsa_sha2_512_final;
		}
	}
}

SECTION_EDDSA_TEXT
static int hal_rtl_eddsa_sign_vrf_hash_op(void *arg)
{
	int ret = SUCCESS;
	phal_crypto_eddsa_t peddsa_adtr = (hal_crypto_eddsa_t *)arg;
	const unsigned char *sig, *msg, *pk;
	unsigned char *p_digest;
	unsigned long long mlen;
	/* 2. SW implement
	 * Points A  :public key
	 * Points R  :signature first 32bytes [0:31]
	 * M: message
	 * h: 64bytes digest
	 */

	pk = peddsa_adtr->p_pbk;
	sig = peddsa_adtr->p_signature;
	msg = peddsa_adtr->p_msg;
	mlen = peddsa_adtr->msglen;
	p_digest = &(peddsa_adtr->sha512_digest[0]);

	peddsa_adtr->hash_init_f(peddsa_adtr);
	peddsa_adtr->hash_update_f(peddsa_adtr, sig, 32);
	peddsa_adtr->hash_update_f(peddsa_adtr, pk, 32);
	peddsa_adtr->hash_update_f(peddsa_adtr, msg, mlen);
	peddsa_adtr->hash_final_f(peddsa_adtr, p_digest);
	//__mem_dump(p_digest, 64 , "sign_vrf_hash_op_digest");
	return ret;
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_hook_sign_vrf_hash_op_f(hal_crypto_eddsa_t *peddsa_adtr, eddsa_sign_vrf_hash_op sign_vrf_hash_f)
{
	if (NULL != sign_vrf_hash_f) {
		peddsa_adtr->sign_vrf_hash_f = sign_vrf_hash_f;
	} else {
		peddsa_adtr->sign_vrf_hash_f = hal_rtl_eddsa_sign_vrf_hash_op;
	}
}
#endif

SECTION_EDDSA_TEXT
int hal_rtl_eddsa_deinit(phal_crypto_eddsa_t adtr)
{
#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
#else
	hal_rtl_sys_peripheral_en(EDDSA_SYS, DISABLE);
#endif
	return SUCCESS;
}

SECTION_EDDSA_TEXT
int hal_rtl_eddsa_verify_32(const unsigned char *new_v, const unsigned char *old_v)
{
	unsigned int differentbits = 0;
	uint32_t i;
	for (i = 0; i < 32; i++) {
		differentbits |= (new_v[i] ^ old_v[i]);
	}
	return (1 & ((differentbits - 1) >> 8)) - 1;
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_seq_set_word_from_byte(const uint32_t base_addr, const uint32_t seq_offset, const uint32_t s_val_len,
		const unsigned char *s_value)
{

	volatile u32 val = 0x0;
	uint32_t i;
	const unsigned char *p_s_value = s_value;
	for (i = 0 ; i < s_val_len; i += 4) {
#if 0
		(val) = (((uint32_t)(s_value)[(i)    ]
				  | ((uint32_t)(s_value)[(i) + 1] <<  8)
				  | ((uint32_t)(s_value)[(i) + 2] << 16)
				  | ((uint32_t)(s_value)[(i) + 3] << 24));
#endif

				 val |= ((uint32_t)p_s_value[i]);
				 val |= ((uint32_t)p_s_value[i + 1] << 8);
				 val |= ((uint32_t)p_s_value[i + 2] << 16);
				 val |= ((uint32_t)p_s_value[i + 3] << 24);

				 HAL_WRITE32((uint32_t)base_addr, (seq_offset + i), val);
	}
}

#if 0
// save for debug use
SECTION_EDDSA_TEXT
void hal_rtl_eddsa_seq_set_word_from_byte(const uint32_t base_addr, const uint32_t seq_offset, const uint32_t s_val_len,
		const unsigned char *s_value)
{

	volatile u32 val = 0x0;
	uint32_t i;
	//__dbg_mem_dump(peddsa_adapter->isMemDump, s_value,s_val_len,"set seq_value");
	for (i = 0 ; i < s_val_len; i += 4) {
		(val) = (((uint32_t)(s_value)[(i)    ])
				 | ((uint32_t)(s_value)[(i) + 1] <<  8)
				 | ((uint32_t)(s_value)[(i) + 2] << 16)
				 | ((uint32_t)(s_value)[(i) + 3] << 24));

		HAL_WRITE32((uint32_t)base_addr, (seq_offset + i), val);
	}
}

// save for debug use
SECTION_EDDSA_TEXT
void hal_rtl_eddsa_seq_get_word_from_byte(const uint32_t base_addr, const uint32_t seq_offset, const uint32_t s_val_len,
		unsigned char *s_value)
{

	volatile u32 val;
	uint32_t i;
	for (i = 0 ; i < s_val_len; i += 4) {
		val = HAL_READ32((uint32_t)base_addr, (seq_offset + i));
		(s_value)[(i)    ] = (unsigned char)(((val)) & 0xFF);
		(s_value)[(i) + 1] = (unsigned char)(((val) >>  8) & 0xFF);
		(s_value)[(i) + 2] = (unsigned char)(((val) >> 16) & 0xFF);
		(s_value)[(i) + 3] = (unsigned char)(((val) >> 24) & 0xFF);
	}
	//__dbg_mem_dump(peddsa_adapter->isMemDump, s_value,s_val_len,"get seq_value");
}


SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_word_value(const uint32_t base_addr, const uint32_t offset, const uint32_t w_value)
{

	volatile u32 val = 0x0;
	val |= w_value;
	HAL_WRITE32((uint32_t)base_addr, offset, val);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_word_value(const uint32_t base_addr, const uint32_t offset, uint32_t *w_buf)
{

	volatile u32 val = 0x0;
	val = HAL_READ32((uint32_t)base_addr, offset);
	*w_buf = val;
}
#endif

SECTION_EDDSA_TEXT
uint32_t hal_rtl_eddsa_set_word_from_byte(const unsigned char *s_value)
{
	volatile uint32_t val = 0x0;
	(val) = (((uint32_t)(s_value)[0])
			 | ((uint32_t)(s_value)[1] <<  8)
			 | ((uint32_t)(s_value)[2] << 16)
			 | ((uint32_t)(s_value)[3] << 24));
	return val;
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_x1_in_val(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	peddsa_obj->EDDSA_ENG_X1_P0_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 0)));
	peddsa_obj->EDDSA_ENG_X1_P1_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 1)));
	peddsa_obj->EDDSA_ENG_X1_P2_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 2)));
	peddsa_obj->EDDSA_ENG_X1_P3_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 3)));
	peddsa_obj->EDDSA_ENG_X1_P4_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 4)));
	peddsa_obj->EDDSA_ENG_X1_P5_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 5)));
	peddsa_obj->EDDSA_ENG_X1_P6_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 6)));
	peddsa_obj->EDDSA_ENG_X1_P7_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_y1_in_val(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	peddsa_obj->EDDSA_ENG_Y1_P0_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 0)));
	peddsa_obj->EDDSA_ENG_Y1_P1_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 1)));
	peddsa_obj->EDDSA_ENG_Y1_P2_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 2)));
	peddsa_obj->EDDSA_ENG_Y1_P3_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 3)));
	peddsa_obj->EDDSA_ENG_Y1_P4_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 4)));
	peddsa_obj->EDDSA_ENG_Y1_P5_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 5)));
	peddsa_obj->EDDSA_ENG_Y1_P6_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 6)));
	peddsa_obj->EDDSA_ENG_Y1_P7_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_x2_in_val(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	peddsa_obj->EDDSA_ENG_X2_P0_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 0)));
	peddsa_obj->EDDSA_ENG_X2_P1_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 1)));
	peddsa_obj->EDDSA_ENG_X2_P2_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 2)));
	peddsa_obj->EDDSA_ENG_X2_P3_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 3)));
	peddsa_obj->EDDSA_ENG_X2_P4_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 4)));
	peddsa_obj->EDDSA_ENG_X2_P5_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 5)));
	peddsa_obj->EDDSA_ENG_X2_P6_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 6)));
	peddsa_obj->EDDSA_ENG_X2_P7_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_y2_in_val(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	peddsa_obj->EDDSA_ENG_Y2_P0_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 0)));
	peddsa_obj->EDDSA_ENG_Y2_P1_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 1)));
	peddsa_obj->EDDSA_ENG_Y2_P2_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 2)));
	peddsa_obj->EDDSA_ENG_Y2_P3_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 3)));
	peddsa_obj->EDDSA_ENG_Y2_P4_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 4)));
	peddsa_obj->EDDSA_ENG_Y2_P5_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 5)));
	peddsa_obj->EDDSA_ENG_Y2_P6_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 6)));
	peddsa_obj->EDDSA_ENG_Y2_P7_REG = hal_rtl_eddsa_set_word_from_byte((s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_byte_from_word(const uint32_t reg_val, unsigned char *s_value)
{

	volatile uint32_t val = reg_val;
	(s_value)[0    ] = (unsigned char)(((val)) & 0xFF);
	(s_value)[1    ] = (unsigned char)(((val) >>  8) & 0xFF);
	(s_value)[2    ] = (unsigned char)(((val) >> 16) & 0xFF);
	(s_value)[3    ] = (unsigned char)(((val) >> 24) & 0xFF);
	//__dbg_mem_dump(peddsa_adapter->isMemDump, s_value,s_val_len,"get seq_value");
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_x1_out_val(const uint32_t base_addr, unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	volatile uint32_t val;
	val = peddsa_obj->EDDSA_ENG_XO1_P0_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 0)));
	val = peddsa_obj->EDDSA_ENG_XO1_P1_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 1)));
	val = peddsa_obj->EDDSA_ENG_XO1_P2_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 2)));
	val = peddsa_obj->EDDSA_ENG_XO1_P3_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 3)));
	val = peddsa_obj->EDDSA_ENG_XO1_P4_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 4)));
	val = peddsa_obj->EDDSA_ENG_XO1_P5_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 5)));
	val = peddsa_obj->EDDSA_ENG_XO1_P6_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 6)));
	val = peddsa_obj->EDDSA_ENG_XO1_P7_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_y1_out_val(const uint32_t base_addr, unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	volatile uint32_t val;
	val = peddsa_obj->EDDSA_ENG_YO1_P0_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 0)));
	val = peddsa_obj->EDDSA_ENG_YO1_P1_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 1)));
	val = peddsa_obj->EDDSA_ENG_YO1_P2_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 2)));
	val = peddsa_obj->EDDSA_ENG_YO1_P3_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 3)));
	val = peddsa_obj->EDDSA_ENG_YO1_P4_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 4)));
	val = peddsa_obj->EDDSA_ENG_YO1_P5_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 5)));
	val = peddsa_obj->EDDSA_ENG_YO1_P6_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 6)));
	val = peddsa_obj->EDDSA_ENG_YO1_P7_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_x2_out_val(const uint32_t base_addr, unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	volatile uint32_t val;
	val = peddsa_obj->EDDSA_ENG_XO2_P0_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 0)));
	val = peddsa_obj->EDDSA_ENG_XO2_P1_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 1)));
	val = peddsa_obj->EDDSA_ENG_XO2_P2_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 2)));
	val = peddsa_obj->EDDSA_ENG_XO2_P3_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 3)));
	val = peddsa_obj->EDDSA_ENG_XO2_P4_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 4)));
	val = peddsa_obj->EDDSA_ENG_XO2_P5_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 5)));
	val = peddsa_obj->EDDSA_ENG_XO2_P6_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 6)));
	val = peddsa_obj->EDDSA_ENG_XO2_P7_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_y2_out_val(const uint32_t base_addr, unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	volatile uint32_t val;
	val = peddsa_obj->EDDSA_ENG_YO2_P0_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 0)));
	val = peddsa_obj->EDDSA_ENG_YO2_P1_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 1)));
	val = peddsa_obj->EDDSA_ENG_YO2_P2_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 2)));
	val = peddsa_obj->EDDSA_ENG_YO2_P3_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 3)));
	val = peddsa_obj->EDDSA_ENG_YO2_P4_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 4)));
	val = peddsa_obj->EDDSA_ENG_YO2_P5_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 5)));
	val = peddsa_obj->EDDSA_ENG_YO2_P6_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 6)));
	val = peddsa_obj->EDDSA_ENG_YO2_P7_REG;
	hal_rtl_eddsa_get_byte_from_word((const uint32_t)val, (s_value + (POINT_DATA_VAL_OFFSET * 7)));
}

#if 0
SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_word_value(const uint32_t base_addr, const uint32_t offset, const uint32_t w_value)
{

	volatile u32 val = 0x0;
	val |= w_value;
	HAL_WRITE32((uint32_t)base_addr, offset, val);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_word_value(const uint32_t base_addr, const uint32_t offset, uint32_t *w_buf)
{

	volatile u32 val = 0x0;
	val = HAL_READ32((uint32_t)base_addr, offset);
	*w_buf = val;
}
#endif

SECTION_EDDSA_TEXT
int hal_rtl_eddsa_polling_wait_done(phal_crypto_eddsa_t peddsa_adapter)
{

	EDDSA_TypeDef *peddsa_obj;
	volatile uint32_t val;
	uint32_t int_sts;
	uint32_t loopWait;
	peddsa_obj = peddsa_adapter->base_addr;

#if 0
	// RAM CODE HOOK CALLBACK FUNC
	if (pcrypto_adapter->wait_done_func != NULL) {
		DBG_CRYPTO_INFO("Interrupt Handler way[Use pcrypto_adapter->wait_done_func]!\r\n");
		return pcrypto_adapter->wait_done_func(pcrypto_adapter);
	}
#endif
	// wait until ipsec engine stop
	loopWait = 1000000; /* hope long enough */
	while (1) {
		val = peddsa_obj->EDDSA_ENG_CONF_INT_REG;
		//hal_rtl_eddsa_get_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CONF_INT_REG_OFFSET, &val);

		int_sts = (val & (EDDSA_BIT_MODE0_FINISH | EDDSA_BIT_MODE1_FINISH |
						  EDDSA_BIT_MODE2_FINISH | EDDSA_BIT_MODE3_FINISH));
		//DBG_EDDSA_INFO("int_sts = 0x%x\r\n",int_sts);
		if (int_sts) {
			val = 0x0;
			if (int_sts & EDDSA_BIT_MODE0_FINISH) {
				val |= EDDSA_BIT_MODE0_FINISH;
				DBG_EDDSA_INFO("mode0 finsih!\r\n");
			}
			if (int_sts & EDDSA_BIT_MODE1_FINISH) {
				val |= EDDSA_BIT_MODE1_FINISH;
				DBG_EDDSA_INFO("mode1 finsih!\r\n");
			}
			if (int_sts & EDDSA_BIT_MODE2_FINISH) {
				val |= EDDSA_BIT_MODE2_FINISH;
				DBG_EDDSA_INFO("mode2 finsih!\r\n");
			}
			if (int_sts & EDDSA_BIT_MODE3_FINISH) {
				val |= EDDSA_BIT_MODE3_FINISH;
				DBG_EDDSA_INFO("mode3 finsih!\r\n");
			}

			if (val) {
				// Clear interrupt status
				DBG_EDDSA_INFO("clear interrupt 0x%x\r\n", val);
				peddsa_obj->EDDSA_ENG_CONF_INT_REG = val;
				//HAL_WRITE32((uint32_t)peddsa_obj, EDDSA_ENG_CONF_INT_REG_OFFSET, val);
			}
			break;
		}

		loopWait--;
		if (loopWait == 0) {
			DBG_EDDSA_INFO("Wait Timeout! int_sts = 0x%x\r\n", val);
			return FAIL; /* error occurs */
		}
	}
	return SUCCESS;

}

#endif

#if IS_CUT_TEST(CONFIG_CHIP_VER)
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify(phal_crypto_eddsa_t peddsa_adapter)
{
	switch (peddsa_adapter->flow_mode) {
	case EDDSA_FLOW_STEPS:
		return hal_rtl_eddsa_sign_verify_steps(peddsa_adapter);
		break;
	case EDDSA_FLOW_SEMIAUTO:
		return hal_rtl_eddsa_sign_verify_semi_auto(peddsa_adapter);
		break;
	case EDDSA_FLOW_AUTOFLOW:
		return hal_rtl_eddsa_sign_verify_autoflow(peddsa_adapter);
		break;
	default:
		return hal_rtl_eddsa_sign_verify_steps(peddsa_adapter);
		break;
	}
}

#else
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify(phal_crypto_eddsa_t peddsa_adtr,
							  const unsigned char *sig, const unsigned char *msg,
							  const unsigned char *pk, const uint32_t msglen,
							  unsigned char mode_sel, unsigned char recover_en)
{
	int ret = SUCCESS;
	peddsa_adtr->p_pbk = (uint8_t *) pk;
	peddsa_adtr->p_signature = (uint8_t *) sig;
	peddsa_adtr->p_msg = (uint8_t *) msg;
	peddsa_adtr->msglen = msglen;
	//DBG_EDDSA_INFO("p_pbk = 0x%x\r\n", peddsa_adtr->p_pbk);
	//DBG_EDDSA_INFO("p_sig = 0x%x\r\n", peddsa_adtr->p_signature);
	//DBG_EDDSA_INFO("p_msg = 0x%x\r\n", peddsa_adtr->p_msg);
	//DBG_EDDSA_INFO("msglen = %d\r\n", peddsa_adtr->msglen);

	// Whether turn-on recover
	peddsa_adtr->point_recover_en = recover_en;

	// select
	peddsa_adtr->sB_write_out_en = 1;

	// select
	peddsa_adtr->kA_write_out_en = 1;

	// auto-step select[Depend on what kinds of autoflow step(all or partial)]
	peddsa_adtr->flow_mode = mode_sel;
	ret = hal_rtl_eddsa_sign_verify_core(peddsa_adtr);
	return ret;
}

SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify_core(phal_crypto_eddsa_t peddsa_adapter)
{
	switch (peddsa_adapter->flow_mode) {
	case EDDSA_FLOW_STEPS:
		return hal_rtl_eddsa_sign_verify_steps(peddsa_adapter);
		break;
	case EDDSA_FLOW_SEMIAUTO:
		return hal_rtl_eddsa_sign_verify_semi_auto(peddsa_adapter);
		break;
	case EDDSA_FLOW_AUTOFLOW:
		return hal_rtl_eddsa_sign_verify_autoflow(peddsa_adapter);
		break;
	default:
		return hal_rtl_eddsa_sign_verify_steps(peddsa_adapter);
		break;
	}
}
#endif

// Autoflow all
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify_autoflow(phal_crypto_eddsa_t peddsa_adapter)
{

	EDDSA_TypeDef *peddsa_obj;
	peddsa_obj = peddsa_adapter->base_addr;
	const unsigned char *pbk        = peddsa_adapter->p_pbk;
	const unsigned char *sig_r      = (peddsa_adapter->p_signature);
	const unsigned char *sig_s      = ((peddsa_adapter->p_signature) + SIGNATURE_OFFSET_S);
	unsigned char *p_x_buf, *p_y_buf;
	unsigned char *p_digest, *p_rcheck;
	int ret = SUCCESS;

	// Set signature_s
	hal_rtl_eddsa_set_signature_s((uint32_t)peddsa_obj, sig_s);
	DBG_EDDSA_INFO("set signature ok\r\n");

	if (1 == (peddsa_adapter->point_recover_en)) {
		// Set encode public key
		hal_rtl_eddsa_set_encode_pbk((uint32_t)peddsa_obj, pbk);
		DBG_EDDSA_INFO("set public key ok\r\n");
	}

	// Set recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	uint32_t val = 0x0;
	if (0 == (peddsa_adapter->point_recover_en)) {
		DBG_EDDSA_WARN("point_recover_en should be turned-on when autoflow\r\n");
		return HAL_ERR_PARA;
	}
	// Turn-on by default
	val |= EDDSA_BIT_RECOVER_X_EN;
	val |= (EDDSA_MODE0 << EDDSA_SHIFT_MOD_SEL);
	val |= (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL);
	val |= EDDSA_BIT_AUTO_SP;

	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = EDDSA_BIT_ENG_START;

	// Run hash SHA512 sw algorithm
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (peddsa_adapter->hash_exec_func != NULL) {
		ret = (peddsa_adapter->hash_exec_func)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("No-hook hash function\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#else
	if (peddsa_adapter->sign_vrf_hash_f != NULL) {
		ret = (peddsa_adapter->sign_vrf_hash_f)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("No-hook hash function\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#endif

	// Set hash digest 64 bytes (input x1,input y1)
	if (1 == (peddsa_adapter->isHashok)) {
		p_digest = &(peddsa_adapter->sha512_digest[0]);
		uint8_t sha512_bypass = 0;
		hal_rtl_eddsa_set_hash_digest((uint32_t)peddsa_obj, p_digest,
									  (p_digest + POINT_AXIS_VAL_SIZE), sha512_bypass);
	} else {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Set hash ok
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = EDDSA_BIT_HASH_OK;

	// Bypass step2, step3

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Rx, Ry
	p_x_buf = &(peddsa_adapter->rcheck_point[0][0]);
	p_y_buf = &(peddsa_adapter->rcheck_point[1][0]);
	hal_rtl_eddsa_get_Rplus((uint32_t)peddsa_obj, p_x_buf, p_y_buf);

	// Check encode Ry -> yo2
	p_rcheck = &(peddsa_adapter->rcheck_enc[0]);
	hal_rtl_eddsa_get_enc_Rplus((uint32_t)peddsa_obj, p_rcheck);

	ret = hal_rtl_eddsa_verify_32(p_rcheck, sig_r);
	if (ret != SUCCESS) {
		ret = FAIL;
		__dbg_mem_dump(peddsa_adapter->isMemDump, p_rcheck, POINT_AXIS_VAL_SIZE, "dump p_rcheck");
		__dbg_mem_dump(peddsa_adapter->isMemDump, sig_r, POINT_AXIS_VAL_SIZE, "dump signature_r");
	} else {
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[0][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_x");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[1][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_y");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_enc[0]), POINT_AXIS_VAL_SIZE, "dump rcheck_enc");
	}

hal_rtl_eddsa_sign_verify_end:
	return ret;
}

// Step1+Autoflow(Step2+3)
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify_semi_auto(phal_crypto_eddsa_t peddsa_adapter)
{

	EDDSA_TypeDef *peddsa_obj;
	peddsa_obj = peddsa_adapter->base_addr;
	const unsigned char *pbk        = peddsa_adapter->p_pbk;
	const unsigned char *sig_r      = (peddsa_adapter->p_signature);
	const unsigned char *sig_s      = ((peddsa_adapter->p_signature) + SIGNATURE_OFFSET_S);
	unsigned char *p_x_buf, *p_y_buf;
	unsigned char *p_digest, *p_rcheck;
	int ret = SUCCESS;

	// Set signature_s
	hal_rtl_eddsa_set_signature_s((uint32_t)peddsa_obj, sig_s);
	DBG_EDDSA_INFO("set signature ok\r\n");

	if (1 == (peddsa_adapter->point_recover_en)) {
		// Set encode public key
		hal_rtl_eddsa_set_encode_pbk((uint32_t)peddsa_obj, pbk);
		DBG_EDDSA_INFO("set public key ok\r\n");
	}

	// Set recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	uint32_t val = 0x0;
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
	val |= ((EDDSA_MODE0 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = EDDSA_BIT_ENG_START;

	// Run hash SHA512 sw algorithm
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (peddsa_adapter->hash_exec_func != NULL) {
		ret = (peddsa_adapter->hash_exec_func)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("Non-hook Hash function!\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#else
	if (peddsa_adapter->sign_vrf_hash_f != NULL) {
		ret = (peddsa_adapter->sign_vrf_hash_f)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("Non-hook Hash function!\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#endif

	if (0 == (peddsa_adapter->point_recover_en)) {
		if (peddsa_adapter->decompress_exec_func != NULL) {
			ret = (peddsa_adapter->decompress_exec_func)(peddsa_adapter);
			if (ret != SUCCESS) {
				goto hal_rtl_eddsa_sign_verify_end;
			} else {
				peddsa_adapter->isDecompress_pbk_ok = 1;
			}
		} else {
			DBG_EDDSA_INFO("Non-hook Decompress function!\r\n");
			ret = FAIL;
			goto hal_rtl_eddsa_sign_verify_end;
		}
	}

	// Check mode 0 interrupt signal occur
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Recover public key values(xo2,yo2) if recover x point enable
	if (1 == (peddsa_adapter->point_recover_en)) {
		p_x_buf = &(peddsa_adapter->point_recover[0][0]);
		p_y_buf = &(peddsa_adapter->point_recover[1][0]);
		hal_rtl_eddsa_get_recover_pbk((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

	// Check s x B point(x,y)
	if (1 == (peddsa_adapter->sB_write_out_en)) {
		p_x_buf = &(peddsa_adapter->sB_point[0][0]);
		p_y_buf = &(peddsa_adapter->sB_point[1][0]);
		hal_rtl_eddsa_get_sB((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

	// Set Recover public key values(xo2,yo2) if recover x point disable
	if (1 == (peddsa_adapter->isDecompress_pbk_ok)) {
		p_x_buf = &(peddsa_adapter->dec_pbk[0][0]);
		p_y_buf = &(peddsa_adapter->dec_pbk[1][0]);
		hal_rtl_eddsa_set_recover_pbk((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

	// Set hash digest 64 bytes (input x1,input y1)
	if (1 == (peddsa_adapter->isHashok)) {
		p_digest = &(peddsa_adapter->sha512_digest[0]);
		uint8_t sha512_bypass = 0;
		hal_rtl_eddsa_set_hash_digest((uint32_t)peddsa_obj, p_digest, (p_digest + POINT_AXIS_VAL_SIZE), sha512_bypass);
	} else {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Set Autoflow(Step2+3) recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	val = 0x0;
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
	val |= ((EDDSA_MODE1 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL)
			| EDDSA_BIT_AUTO_SP);
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = EDDSA_BIT_ENG_START;

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Rx, Ry
	p_x_buf = &(peddsa_adapter->rcheck_point[0][0]);
	p_y_buf = &(peddsa_adapter->rcheck_point[1][0]);
	hal_rtl_eddsa_get_Rplus((uint32_t)peddsa_obj, p_x_buf, p_y_buf);

	// Check encode Ry -> yo2
	p_rcheck = &(peddsa_adapter->rcheck_enc[0]);
	hal_rtl_eddsa_get_enc_Rplus((uint32_t)peddsa_obj, p_rcheck);

	ret = hal_rtl_eddsa_verify_32(p_rcheck, sig_r);
	if (ret != SUCCESS) {
		ret = FAIL;
		__dbg_mem_dump(peddsa_adapter->isMemDump, p_rcheck, POINT_AXIS_VAL_SIZE, "dump p_rcheck");
		__dbg_mem_dump(peddsa_adapter->isMemDump, sig_r, POINT_AXIS_VAL_SIZE, "dump signature_r");
	} else {
		if (1 == (peddsa_adapter->sB_write_out_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[0][0]), POINT_AXIS_VAL_SIZE, "dump sxB_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[1][0]), POINT_AXIS_VAL_SIZE, "dump sxB_y");
		}
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[0][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_x");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[1][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_y");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_enc[0]), POINT_AXIS_VAL_SIZE, "dump rcheck_enc");
	}

hal_rtl_eddsa_sign_verify_end:
	return ret;
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_signature_s(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_set_x1_in_val((uint32_t)peddsa_obj, s_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_encode_pbk(const uint32_t base_addr, const unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_set_y2_in_val((uint32_t)peddsa_obj, s_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_recover_pbk(const uint32_t base_addr, const unsigned char *s_x_value, const unsigned char *s_y_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_set_x2_in_val((uint32_t)peddsa_obj, s_x_value);
	hal_rtl_eddsa_set_y2_in_val((uint32_t)peddsa_obj, s_y_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_set_hash_digest(const uint32_t base_addr, const unsigned char *s_x_value, const unsigned char *s_y_value, uint8_t bypass_en)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	if (0 == bypass_en) {
		// Set input: h[255:0] & h[511:256]
		hal_rtl_eddsa_set_x1_in_val((uint32_t)peddsa_obj, s_x_value);
		hal_rtl_eddsa_set_y1_in_val((uint32_t)peddsa_obj, s_y_value);
	} else {
		// Only Set input: h[255:0]
		hal_rtl_eddsa_set_x1_in_val((uint32_t)peddsa_obj, s_x_value);
	}
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_recover_pbk(const uint32_t base_addr, unsigned char *s_x_value, unsigned char *s_y_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_get_x2_out_val((uint32_t)peddsa_obj, s_x_value);
	hal_rtl_eddsa_get_y2_out_val((uint32_t)peddsa_obj, s_y_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_sB(const uint32_t base_addr, unsigned char *s_x_value, unsigned char *s_y_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_get_x1_out_val((uint32_t)peddsa_obj, s_x_value);
	hal_rtl_eddsa_get_y1_out_val((uint32_t)peddsa_obj, s_y_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_kA(const uint32_t base_addr, unsigned char *s_x_value, unsigned char *s_y_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_get_x2_out_val((uint32_t)peddsa_obj, s_x_value);
	hal_rtl_eddsa_get_y2_out_val((uint32_t)peddsa_obj, s_y_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_Rplus(const uint32_t base_addr, unsigned char *s_x_value, unsigned char *s_y_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_get_x1_out_val((uint32_t)peddsa_obj, s_x_value);
	hal_rtl_eddsa_get_y1_out_val((uint32_t)peddsa_obj, s_y_value);
}

SECTION_EDDSA_TEXT
void hal_rtl_eddsa_get_enc_Rplus(const uint32_t base_addr, unsigned char *s_value)
{
	EDDSA_TypeDef *peddsa_obj = (EDDSA_TypeDef *)base_addr;
	hal_rtl_eddsa_get_y2_out_val((uint32_t)peddsa_obj, s_value);
}


#if 1
// [Unify coding styles] Step1+Step2+Step3
SECTION_EDDSA_TEXT
int hal_rtl_eddsa_sign_verify_steps(phal_crypto_eddsa_t peddsa_adapter)
{

	EDDSA_TypeDef *peddsa_obj;
	peddsa_obj = peddsa_adapter->base_addr;
	const unsigned char *pbk        = peddsa_adapter->p_pbk;
	const unsigned char *sig_r      = (peddsa_adapter->p_signature);
	const unsigned char *sig_s      = ((peddsa_adapter->p_signature) + SIGNATURE_OFFSET_S);
	unsigned char *p_x_buf, *p_y_buf;
	unsigned char *p_digest, *p_rcheck;
	int ret = SUCCESS;

	// Set signature_s
	hal_rtl_eddsa_set_signature_s((uint32_t)peddsa_obj, sig_s);
	DBG_EDDSA_INFO("set signature ok\r\n");

	if (1 == (peddsa_adapter->point_recover_en)) {
		// Set encode public key
		hal_rtl_eddsa_set_encode_pbk((uint32_t)peddsa_obj, pbk);
		DBG_EDDSA_INFO("set public key ok\r\n");
	}

	// Set recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	uint32_t val = 0x0;
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
	val |= ((EDDSA_MODE0 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	val = EDDSA_BIT_ENG_START;
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = val;

	// Run hash SHA512 sw algorithm
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (peddsa_adapter->hash_exec_func != NULL) {
		ret = (peddsa_adapter->hash_exec_func)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("Non-hook Hash function!\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#else
	if (peddsa_adapter->sign_vrf_hash_f != NULL) {
		ret = (peddsa_adapter->sign_vrf_hash_f)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("Non-hook Hash function!\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
#endif

	if (0 == (peddsa_adapter->point_recover_en)) {
		if (peddsa_adapter->decompress_exec_func != NULL) {
			ret = (peddsa_adapter->decompress_exec_func)(peddsa_adapter);
			if (ret != SUCCESS) {
				goto hal_rtl_eddsa_sign_verify_end;
			} else {
				peddsa_adapter->isDecompress_pbk_ok = 1;
			}
		} else {
			DBG_EDDSA_INFO("Non-hook Decompress function!\r\n");
			ret = FAIL;
			goto hal_rtl_eddsa_sign_verify_end;
		}
	}

	// Check mode 0 interrupt signal occur
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Recover public key values(xo2,yo2) if recover x point enable
	if (1 == (peddsa_adapter->point_recover_en)) {
		p_x_buf = &(peddsa_adapter->point_recover[0][0]);
		p_y_buf = &(peddsa_adapter->point_recover[1][0]);
		hal_rtl_eddsa_get_recover_pbk((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

	// Check s x B point(xo1,yo1)
	if (1 == (peddsa_adapter->sB_write_out_en)) {
		p_x_buf = &(peddsa_adapter->sB_point[0][0]);
		p_y_buf = &(peddsa_adapter->sB_point[1][0]);
		hal_rtl_eddsa_get_sB((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

#if 1
	// Set Recover public key values(xo2,yo2) if recover x point disable
	if ((1 == (peddsa_adapter->isDecompress_pbk_ok)) || (1 == (peddsa_adapter->point_recover_en))) {
		if (1 == (peddsa_adapter->isDecompress_pbk_ok)) {
			p_x_buf = &(peddsa_adapter->dec_pbk[0][0]);
			p_y_buf = &(peddsa_adapter->dec_pbk[1][0]);
		} else if (1 == (peddsa_adapter->point_recover_en)) {
			p_x_buf = &(peddsa_adapter->point_recover[0][0]);
			p_y_buf = &(peddsa_adapter->point_recover[1][0]);
		}
		hal_rtl_eddsa_set_recover_pbk((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}
#endif
#if 0
	// SW set a wrong value
	p_x_buf = &(peddsa_adapter->point_recover[0][0]);
	p_y_buf = &(peddsa_adapter->point_recover[1][0]);
	eddsa_rom_memset(p_x_buf, 0x0, 32);
	eddsa_rom_memset(p_y_buf, 0x0, 32);
	hal_rtl_eddsa_set_recover_pbk((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
#endif
	// Set hash digest 64 bytes (input x1,input y1)
	if (1 == (peddsa_adapter->isHashok)) {
		p_digest = &(peddsa_adapter->sha512_digest[0]);
		uint8_t sha512_bypass = 0;
		hal_rtl_eddsa_set_hash_digest((uint32_t)peddsa_obj, p_digest, (p_digest + POINT_AXIS_VAL_SIZE), sha512_bypass);
	} else {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}


	// Set Step2, recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	val = 0x0;
#if 0
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
#endif
	val |= ((EDDSA_MODE1 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	val = EDDSA_BIT_ENG_START;
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = val;

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check h x A result(xo2,yo2)
	if (1 == (peddsa_adapter->kA_write_out_en)) {
		p_x_buf = &(peddsa_adapter->kA_point[0][0]);
		p_y_buf = &(peddsa_adapter->kA_point[1][0]);
		hal_rtl_eddsa_get_kA((uint32_t)peddsa_obj, p_x_buf, p_y_buf);
	}

	// Set Step3, mode_self/func_sel
	val = 0x0;
	val |= ((EDDSA_MODE2 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	peddsa_obj->EDDSA_ENG_CTRL_REG = val;

	// Set engine start
	val = EDDSA_BIT_ENG_START;
	peddsa_obj->EDDSA_ENG_CONF_INT_REG = val;

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Rx, Ry
	p_x_buf = &(peddsa_adapter->rcheck_point[0][0]);
	p_y_buf = &(peddsa_adapter->rcheck_point[1][0]);
	hal_rtl_eddsa_get_Rplus((uint32_t)peddsa_obj, p_x_buf, p_y_buf);

	// Check encode Ry -> yo2
	p_rcheck = &(peddsa_adapter->rcheck_enc[0]);
	hal_rtl_eddsa_get_enc_Rplus((uint32_t)peddsa_obj, p_rcheck);

	ret = hal_rtl_eddsa_verify_32(p_rcheck, sig_r);
	if (ret != SUCCESS) {
		ret = FAIL;
		__dbg_mem_dump(peddsa_adapter->isMemDump, p_rcheck, POINT_AXIS_VAL_SIZE, "dump p_rcheck");
		__dbg_mem_dump(peddsa_adapter->isMemDump, sig_r, POINT_AXIS_VAL_SIZE, "dump signature_r");
	} else {
		if (1 == (peddsa_adapter->point_recover_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->point_recover[0][0]), POINT_AXIS_VAL_SIZE, "dump hwdec_pbk_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->point_recover[1][0]), POINT_AXIS_VAL_SIZE, "dump hwdec_pbk_y");
		}
		if (1 == (peddsa_adapter->sB_write_out_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[0][0]), POINT_AXIS_VAL_SIZE, "dump sxB_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[1][0]), POINT_AXIS_VAL_SIZE, "dump sxB_y");
		}
		if (1 == (peddsa_adapter->kA_write_out_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->kA_point[0][0]), POINT_AXIS_VAL_SIZE, "dump kxA_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->kA_point[1][0]), POINT_AXIS_VAL_SIZE, "dump kxA_y");
		}
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[0][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_x");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[1][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_y");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_enc[0]), POINT_AXIS_VAL_SIZE, "dump rcheck_enc");
	}

hal_rtl_eddsa_sign_verify_end:
	return ret;
}
#endif


#if 0

#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)

/**
 *   \brief  The data struct of CRYPTO non-secure stub functions. ROM code functions are accessed in RAM code through stub functions.
 */
SECTION_CRYPTO_STUBS
const hal_crypto_func_stubs_t hal_crypto_stubs_ns = {
	.hal_crypto_engine_init   = hal_rtl_crypto_engine_init,
	.hal_crypto_engine_deinit = hal_rtl_crypto_engine_deinit,
	.hal_crypto_engine_platform_en_ctrl = hal_rtl_crypto_en_ctrl,

	.hal_crypto_auth         = hal_rtl_crypto_auth,
	.hal_crypto_auth_init    = hal_rtl_crypto_auth_init,
	.hal_crypto_auth_process = hal_rtl_crypto_auth_process,
	.hal_crypto_auth_update  = hal_rtl_crypto_auth_update,
	.hal_crypto_auth_final   = hal_rtl_crypto_auth_final,

	.hal_crypto_cipher_init     = hal_rtl_crypto_cipher_init,
	.hal_crypto_cipher_encrypt  = hal_rtl_crypto_cipher_encrypt,
	.hal_crypto_cipher_decrypt  = hal_rtl_crypto_cipher_decrypt,

	.hal_crypto_mix_mode_init     = hal_rtl_crypto_mix_mode_init,
	.hal_crypto_mix_mode_process  = hal_rtl_crypto_mix_mode_process,

	.hal_crypto_crc_setting       = hal_rtl_crypto_crc_setting,
	.hal_crypto_crc_cmd           = hal_rtl_crypto_crc_cmd,
	.hal_crypto_crc_dma           = hal_rtl_crypto_crc_dma,
	.hal_crypto_crc_get_result    = hal_rtl_crypto_crc_get_result,
	.hal_crypto_crc32_cmd         = hal_rtl_crypto_crc32_cmd,
	.hal_crypto_crc32_dma         = hal_rtl_crypto_crc32_dma,

	.hal_crypto_engine            = hal_rtl_crypto_engine,
	.hal_crypto_engine_get_result = hal_rtl_crypto_engine_get_result,
	.hal_crypto_set_srcdesc       = hal_rtl_crypto_set_srcdesc,
	.hal_crypto_set_dstdesc       = hal_rtl_crypto_set_dstdesc,
	.hal_crypto_engine_setup_cl_buffer = hal_rtl_crypto_engine_setup_cl_buffer,
};

#else

/**
 *   \brief  The data struct of CRYPTO secure stub functions. ROM code functions are accessed in RAM code through stub functions.
 */
SECTION_CRYPTO_STUBS
const hal_crypto_func_stubs_t hal_crypto_stubs_s = {
	.hal_crypto_engine_init   = hal_rtl_crypto_engine_init,
	.hal_crypto_engine_deinit = hal_rtl_crypto_engine_deinit,
	.hal_crypto_engine_platform_en_ctrl = hal_rtl_crypto_en_ctrl,

	.hal_crypto_auth         = hal_rtl_crypto_auth,
	.hal_crypto_auth_init    = hal_rtl_crypto_auth_init,
	.hal_crypto_auth_process = hal_rtl_crypto_auth_process,
	.hal_crypto_auth_update  = hal_rtl_crypto_auth_update,
	.hal_crypto_auth_final   = hal_rtl_crypto_auth_final,

	.hal_crypto_cipher_init     = hal_rtl_crypto_cipher_init,
	.hal_crypto_cipher_encrypt  = hal_rtl_crypto_cipher_encrypt,
	.hal_crypto_cipher_decrypt  = hal_rtl_crypto_cipher_decrypt,

	.hal_crypto_mix_mode_init     = hal_rtl_crypto_mix_mode_init,
	.hal_crypto_mix_mode_process  = hal_rtl_crypto_mix_mode_process,

	.hal_crypto_engine            = hal_rtl_crypto_engine,
	.hal_crypto_engine_get_result = hal_rtl_crypto_engine_get_result,
	.hal_crypto_set_srcdesc       = hal_rtl_crypto_set_srcdesc,
	.hal_crypto_set_dstdesc       = hal_rtl_crypto_set_dstdesc,
	.hal_crypto_engine_setup_cl_buffer = hal_rtl_crypto_engine_setup_cl_buffer,
};

#endif

/** @} */ /* End of group hs_hal_eddsa_rom_func */
/// @endcond
/** @} */ /* End of group hs_hal_eddsa */

#if 0
// Non-unify version
// Step1+Step2+Step3
SECTION_EDDSA_TEXT
int _hal_rtl_eddsa_sign_verify(steps)(phal_crypto_eddsa_t peddsa_adapter)
{

	EDDSA_TypeDef *peddsa_obj;
	peddsa_obj = peddsa_adapter->base_addr;
	const unsigned char *pbk        = peddsa_adapter->p_pbk;
	const unsigned char *sig_r      = (peddsa_adapter->p_signature);
	const unsigned char *sig_s      = ((peddsa_adapter->p_signature) + SIGNATURE_OFFSET_S);
	unsigned char *p_x_buf, *p_y_buf;
	unsigned char *p_digest, *p_rcheck;
	int ret = SUCCESS;

	// Set signature_s
	hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_X1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, sig_s);
	DBG_EDDSA_INFO("set signature ok\r\n");

	if (1 == (peddsa_adapter->point_recover_en)) {
		// Set encode public key
		hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_Y2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, pbk);
	}

	// Set recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	uint32_t val = 0x0;
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
	val |= ((EDDSA_MODE0 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CTRL_REG_OFFSET, val);

	// Set engine start
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CONF_INT_REG_OFFSET, EDDSA_BIT_ENG_START);

	// Run hash SHA512 sw algorithm
	if (peddsa_adapter->hash_exec_func != NULL) {
		ret = (peddsa_adapter->hash_exec_func)(peddsa_adapter);
		if (ret != SUCCESS) {
			goto hal_rtl_eddsa_sign_verify_end;
		} else {
			peddsa_adapter->isHashok = 1;
		}
	} else {
		DBG_EDDSA_INFO("Non-hook Hash function!\r\n");
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}
	if (0 == (peddsa_adapter->point_recover_en)) {
		if (peddsa_adapter->decompress_exec_func != NULL) {
			ret = (peddsa_adapter->decompress_exec_func)(peddsa_adapter);
			if (ret != SUCCESS) {
				goto hal_rtl_eddsa_sign_verify_end;
			} else {
				peddsa_adapter->isDecompress_pbk_ok = 1;
			}
		} else {
			DBG_EDDSA_INFO("Non-hook Decompress function!\r\n");
			ret = FAIL;
			goto hal_rtl_eddsa_sign_verify_end;
		}
	}

	// Check mode 0 interrupt signal occur
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		ret = FAIL;
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Recover public key values(xo2,yo2) if recover x point enable
	if (1 == (peddsa_adapter->point_recover_en)) {
		p_x_buf = &(peddsa_adapter->point_recover[0][0]);
		p_y_buf = &(peddsa_adapter->point_recover[1][0]);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_XO2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_YO2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);
	}

	// Check s x B point(xo1,yo1)
	if (1 == (peddsa_adapter->sB_write_out_en)) {
		p_x_buf = &(peddsa_adapter->sB_point[0][0]);
		p_y_buf = &(peddsa_adapter->sB_point[1][0]);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_XO1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_YO1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);
	}

#if 1
	// Set Recover public key values(xo2,yo2) if recover x point disable
	if ((1 == (peddsa_adapter->isDecompress_pbk_ok)) || (1 == (peddsa_adapter->point_recover_en))) {
		if (1 == (peddsa_adapter->isDecompress_pbk_ok)) {
			p_x_buf = &(peddsa_adapter->dec_pbk[0][0]);
			p_y_buf = &(peddsa_adapter->dec_pbk[1][0]);
		} else if (1 == (peddsa_adapter->point_recover_en)) {
			p_x_buf = &(peddsa_adapter->point_recover[0][0]);
			p_y_buf = &(peddsa_adapter->point_recover[1][0]);
		}
		hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_X2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
		hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_Y2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);
	}
#endif
#if 0
	// SW set a wrong value
	p_x_buf = &(peddsa_adapter->point_recover[0][0]);
	p_y_buf = &(peddsa_adapter->point_recover[1][0]);
	eddsa_rom_memset(p_x_buf, 0x0, 32);
	eddsa_rom_memset(p_y_buf, 0x0, 32);
	hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_X2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
	hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_Y2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);
#endif
	// Set hash digest 64 bytes (input x1,input y1)
	if (1 == (peddsa_adapter->isHashok)) {
		p_digest = &(peddsa_adapter->sha512_digest[0]);
		hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_X1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_digest);
		hal_rtl_eddsa_seq_set_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_Y1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, (p_digest + POINT_AXIS_VAL_SIZE));
	} else {
		goto hal_rtl_eddsa_sign_verify_end;
	}


	// Set Step2, recover x point(public key) or not & autoflow step enable or not & mode_self/func_sel
	val = 0x0;
#if 0
	if (1 == (peddsa_adapter->point_recover_en)) {
		val |= EDDSA_BIT_RECOVER_X_EN;
	}
#endif
	val |= ((EDDSA_MODE1 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CTRL_REG_OFFSET, val);

	// Set engine start
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CONF_INT_REG_OFFSET, EDDSA_BIT_ENG_START);

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check h x A result(xo2,yo2)
	if (1 == (peddsa_adapter->kA_write_out_en)) {
		p_x_buf = &(peddsa_adapter->kA_point[0][0]);
		p_y_buf = &(peddsa_adapter->kA_point[1][0]);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_XO2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
		hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_YO2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);
	}

	// Set Step3, mode_self/func_sel
	val = 0x0;
	val |= ((EDDSA_MODE2 << EDDSA_SHIFT_MOD_SEL) | (EDDSA_FUN_DECP_ADDP_SUB_MOD << EDDSA_SHIFT_FUN_SEL));
	DBG_EDDSA_INFO("int_ctrl_val = 0x%x\r\n", val);
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CTRL_REG_OFFSET, val);

	// Set engine start
	hal_rtl_eddsa_set_word_value((uint32_t)peddsa_obj, EDDSA_ENG_CONF_INT_REG_OFFSET, EDDSA_BIT_ENG_START);

	// Check interrupt signal
	ret = hal_rtl_eddsa_polling_wait_done(peddsa_adapter);
	if (ret != SUCCESS) {
		goto hal_rtl_eddsa_sign_verify_end;
	}

	// Check Rx, Ry
	p_x_buf = &(peddsa_adapter->rcheck_point[0][0]);
	p_y_buf = &(peddsa_adapter->rcheck_point[1][0]);
	hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_XO1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_x_buf);
	hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_YO1_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_y_buf);

	// Check encode Ry -> yo2
	p_rcheck = &(peddsa_adapter->rcheck_enc[0]);
	hal_rtl_eddsa_seq_get_word_from_byte((uint32_t)peddsa_obj, EDDSA_ENG_YO2_P0_REG_OFFSET, POINT_AXIS_VAL_SIZE, p_rcheck);

	ret = hal_rtl_eddsa_verify_32(p_rcheck, sig_r);
	if (ret != SUCCESS) {
		ret = FAIL;
		__dbg_mem_dump(peddsa_adapter->isMemDump, p_rcheck, POINT_AXIS_VAL_SIZE, "dump p_rcheck");
		__dbg_mem_dump(peddsa_adapter->isMemDump, sig_r, POINT_AXIS_VAL_SIZE, "dump signature_r");
	} else {
		if (1 == (peddsa_adapter->point_recover_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->point_recover[0][0]), POINT_AXIS_VAL_SIZE, "dump hwdec_pbk_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->point_recover[1][0]), POINT_AXIS_VAL_SIZE, "dump hwdec_pbk_y");
		}
		if (1 == (peddsa_adapter->sB_write_out_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[0][0]), POINT_AXIS_VAL_SIZE, "dump sxB_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->sB_point[1][0]), POINT_AXIS_VAL_SIZE, "dump sxB_y");
		}
		if (1 == (peddsa_adapter->kA_write_out_en)) {
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->kA_point[0][0]), POINT_AXIS_VAL_SIZE, "dump kxA_x");
			__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->kA_point[1][0]), POINT_AXIS_VAL_SIZE, "dump kxA_y");
		}
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[0][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_x");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_point[1][0]), POINT_AXIS_VAL_SIZE, "dump rcheck_point_y");
		__dbg_mem_dump(peddsa_adapter->isMemDump, &(peddsa_adapter->rcheck_enc[0]), POINT_AXIS_VAL_SIZE, "dump rcheck_enc");
	}

hal_rtl_eddsa_sign_verify_end:
	return ret;
}
#endif


#endif  // end of "#if CONFIG_EDDSA_EN"

