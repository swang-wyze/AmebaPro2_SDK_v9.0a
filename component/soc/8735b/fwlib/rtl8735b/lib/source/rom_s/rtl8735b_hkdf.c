/**************************************************************************//**
 * @file     rtl8735b_hkdf.c
 * @brief    Implement "key derivation function" hkdf code functions.
 *
 * @version  V1.00
 * @date     2021-07-21
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

// TODO add doxygen
/**
  \brief todo
*/
#include "cmsis.h"
#include "memory.h"

#if CONFIG_HKDF_EN

#include "basic_types.h"
#include "rtl8735b.h"
#include "rtl8735b_hkdf.h"

#define SECTION_HKDF_TEXT           SECTION(".rom.hal_hkdf.text")
#define SECTION_HKDF_DATA           SECTION(".rom.hal_hkdf.data")
#define SECTION_HKDF_RODATA         SECTION(".rom.hal_hkdf.rodata")
#define SECTION_HKDF_BSS            SECTION(".rom.hal_hkdf.bss")
#define SECTION_HKDF_STUBS          SECTION(".rom.hal_hkdf.stubs")
#define SECTION_HKDF_EXT_STUBS      SECTION(".rom.hal_hkdf_extend.stubs")


// TODO doxygen
#if IS_CUT_TEST(CONFIG_CHIP_VER)
SECTION_HKDF_STUBS
const hal_hkdf_func_stubs_t hal_hkdf_stubs = {
	.hal_hkdf_derive = hal_rtl_hkdf_derive,
	.hal_hkdf_extract = hal_rtl_hkdf_extract,
	.hal_hkdf_expand = hal_rtl_hkdf_expand
};
#else

SECTION_HKDF_STUBS
const hal_hkdf_func_stubs_t hal_hkdf_stubs = {
	.hal_hkdf_init = hal_rtl_hkdf_init,
	.hal_hkdf_deinit = hal_rtl_hkdf_deinit,
	.hal_hkdf_hook_operate_f = hal_rtl_hkdf_hook_operate_f,
#if 0
	.hal_hkdf_derive = hal_rtl_hkdf_derive,
#endif
	.hal_hkdf_extract = hal_rtl_hkdf_extract,
	.hal_hkdf_extract_engine = hal_rtl_hkdf_extract_engine,
	.hal_hkdf_expand = hal_rtl_hkdf_expand,
	.hal_hkdf_expand_engine = hal_rtl_hkdf_expand_engine,
	.hal_hkdf_secure_set_cfg = hal_rtl_hkdf_secure_set_cfg,
	.hal_hkdf_extract_secure_set_cfg = hal_rtl_hkdf_extract_secure_set_cfg,
	.hal_hkdf_expand_secure_set_cfg = hal_rtl_hkdf_expand_secure_set_cfg,
	.hal_hkdf_extract_secure = hal_rtl_hkdf_extract_secure,
	.hal_hkdf_expand_secure = hal_rtl_hkdf_expand_secure,
	.hal_hkdf_expand_register_info_extend = hal_rtl_hkdf_expand_register_info_extend,
	.hal_hkdf_expand_unregister_info_extend = hal_rtl_hkdf_expand_unregister_info_extend,
};

SECTION_HKDF_EXT_STUBS
const hal_hkdf_extend_func_stubs_t hal_hkdf_ext_stubs = {
	.hal_hkdf_hmac_init_hook = hal_rtl_hkdf_hmac_init_hook,
	.hal_hkdf_hmac_update_hook = hal_rtl_hkdf_hmac_update_hook,
	.hal_hkdf_hmac_final_hook = hal_rtl_hkdf_hmac_final_hook,
	.hal_hkdf_extract_secure_all = hal_rtl_hkdf_extract_secure_all,
	.hal_hkdf_expand_secure_all = hal_rtl_hkdf_expand_secure_all,
};
#endif

#if IS_CUT_TEST(CONFIG_CHIP_VER)
// For Test-chip

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract(hal_hkdf_adapter_t *adtr)
{
	u8 null_salt[64];
	int ret;

	if (NULL == adtr->salt) {
		// FIXME if not provided, it is set to a string of HashLen zeros
		u8 *local_salt = (uint8_t *)(((((uint32_t)null_salt - 1) >> 5) + 1) << 5);   // make salt always is a 32-byte aligned address
		size_t local_salt_len = 32;
		_memset((void *)local_salt, 0, local_salt_len);
		ret = adtr->hmac_func_init(local_salt, local_salt_len);
	} else {
		ret = adtr->hmac_func_init(adtr->salt, adtr->salt_len);
	}
	// salt = key, salt_len = key_len
	//ret = crypto_hmac_sha2_256_init (salt, salt_len);
	if (ret != SUCCESS) {
		goto __hkdf_extract_exit;
	}

	//ret = crypto_hmac_sha2_256_update (ikm, ikm_len);
	ret = adtr->hmac_func_update(adtr->ikm, adtr->ikm_len);
	if (ret != SUCCESS) {
		goto __hkdf_extract_exit;
	}

	//ret = crypto_hmac_sha2_256_final (prk);
	ret = adtr->hmac_func_final(adtr->prk);

__hkdf_extract_exit:
	return (ret);
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand(hal_hkdf_adapter_t *adtr)
{
	const size_t hash_len = 32;
	size_t where = 0;
	size_t n;
	size_t t_len = 0;
	size_t i;
	int32_t ret = 0;
	uint8_t t_buf[64];
	uint8_t *t;
	const size_t t_buf_len = 32;

	t = (uint8_t *)(((((uint32_t)t_buf - 1) >> 5) + 1) << 5);   // make t always is a 32-byte aligned address

	if (adtr->nonce == NULL) {
		adtr->nonce = (const unsigned char *) "";
		adtr->nonce_len = 0;
	}

	n = adtr->okm_len / hash_len;

	if ((adtr->okm_len % hash_len) != 0) {
		n++;
	}

	/*
	 * Per RFC 5869 Section 2.3, okm_len must not exceed
	 * 255 times the hash length
	 */
	if (n > 255) {
		ret =  -HAL_ERR_PARA;
		goto __hkdf_expand_exit;
	}

	/*
	 * Compute T = T(1) | T(2) | T(3) | ... | T(N)
	 * Where T(N) is defined in RFC 5869 Section 2.3
	 */
	for (i = 1; i <= n; i++) {
		size_t num_to_copy;
		unsigned char c = i & 0xff;

		//ret = crypto_hmac_sha2_256_init (prk, prk_len);
		ret = adtr->hmac_func_init(adtr->prk, adtr->prk_len);
		if (ret != SUCCESS) {
			goto __hkdf_expand_exit;
		}

		if (t_len > 0) {
			//ret = crypto_hmac_sha2_256_update (t, t_len);
			ret = adtr->hmac_func_update(t, t_len);
			if (ret != SUCCESS) {
				goto __hkdf_expand_exit;
			}
		}

		if ((adtr->nonce_len > 0) && (adtr->nonce != NULL)) {
			//ret = crypto_hmac_sha2_256_update (info, info_len);
			ret = adtr->hmac_func_update(adtr->nonce, adtr->nonce_len);
			if (ret != SUCCESS) {
				goto __hkdf_expand_exit;
			}
		}

		/* The constant concatenated to the end of each T(n) is a single octet.
		 * */
		//ret = crypto_hmac_sha2_256_update (&c, 1);
		ret = adtr->hmac_func_update(&c, 1);
		if (ret != SUCCESS) {
			goto __hkdf_expand_exit;
		}

		//ret = crypto_hmac_sha2_256_final (t);
		ret = adtr->hmac_func_final(t);
		if (ret != SUCCESS) {
			goto __hkdf_expand_exit;
		}

		num_to_copy = i != n ? hash_len : adtr->okm_len - where;
		_memcpy(adtr->okm + where, t, num_to_copy);
		where += hash_len;
		t_len = hash_len;
	}

__hkdf_expand_exit:
	_memset(t, 0, t_buf_len);

	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_derive(hal_hkdf_adapter_t *adtr)
{
	// TODO check input invalid
	int ret;
	uint8_t prk_buf[64];
	uint8_t *prk;
	const size_t prk_len = 32;

	// make prk always is a 32-byte aligned address
	prk = (uint8_t *)(((((uint32_t)prk_buf - 1) >> 5) + 1) << 5);
	// FIXME how many of prk_buf will be used


	// TODO Arg check
	if (NULL == adtr->hmac_func_init) {
		return HAL_ERR_PARA;
	} else if (NULL == adtr->hmac_func_update) {
		return HAL_ERR_PARA;
	} else if (NULL == adtr->hmac_func_final) {
		return HAL_ERR_PARA;
	}
	adtr->prk = prk;
	adtr->prk_len = 32;
	ret = hal_rtl_hkdf_extract(adtr);

	if (ret == SUCCESS) {
		ret = hal_rtl_hkdf_expand(adtr);
	}

	_memset(prk, 0, prk_len);

	return ret;
}
SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_hmac_sha256_secure_init(hal_hkdf_adapter_t *phkdf_adtr, hal_crypto_adapter_t *pcrypto_adapter, const u8 crypto_sel)
{
	hal_status_t ret = HAL_NOT_READY;
	DBG_CRYPTO_ERR("Test-chip non-support\r\n");
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract_secure_all(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_idx, const u8 wb_idx, const u8 *msg_buf)
{
	hal_status_t ret = HAL_NOT_READY;
	DBG_CRYPTO_ERR("Test-chip non-support\r\n");
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_secure_all(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_idx, const u8 wb_idx, const u8 *msg_buf)
{
	hal_status_t ret = HAL_NOT_READY;
	DBG_CRYPTO_ERR("Test-chip non-support\r\n");
	return ret;
}

#else

// After A-cut

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_hmac_sha256_secure_init(hal_hkdf_adapter_t *phkdf_adtr, hal_crypto_adapter_t *pcrypto_adapter, const u8 crypto_sel)
{
	hal_status_t  ret = HAL_OK;
	ret = hal_rtl_hkdf_hmac_init(phkdf_adtr, pcrypto_adapter, crypto_sel, HKDF_HMAC_SHA256);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_hmac_init(hal_hkdf_adapter_t *phkdf_adtr, hal_crypto_adapter_t *pcrypto_adapter,
									const u8 crypto_sel, const u8 hmac_alg_sel)
{
	hal_status_t  ret = HAL_OK;
	u32 crypto_cfg;
	crypto_cfg = hkdf_set_crypto_cfg(crypto_sel, hmac_alg_sel);
	ret = hal_rtl_hkdf_init(phkdf_adtr, pcrypto_adapter, crypto_cfg, HKDF_ORI_SEL_DIS);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_init(hal_hkdf_adapter_t *phkdf_adtr, hal_crypto_adapter_t *pcrypto_adapter, const u32 crypto_cfg, const u8 ori_en)
{
	hal_status_t  ret = HAL_OK;
	int32_t cryp_ret;
	u16 crypto_sel = 0, hmac_alg_sel = 0;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}

	_memset(phkdf_adtr, 0x0, sizeof(hal_hkdf_adapter_t));
	hmac_alg_sel = hkdf_get_low_2byte(crypto_cfg);
	crypto_sel = hkdf_get_high_2byte(crypto_cfg);

	if (HKDF_CRYPTO_HW_SEL_EN == crypto_sel) {
		// init ceypto engine
		if (NULL == pcrypto_adapter) {
			return HAL_ERR_PARA;
		} else {
			if (_TRUE != pcrypto_adapter->isInit) {
				DBG_CRYPTO_ERR("hkdf: HW Crypto not init yet!\r\n");
				return HAL_NOT_READY;
			} else {
				phkdf_adtr->pcrypto_adtr = pcrypto_adapter;
				phkdf_adtr->isHWCrypto_Init = _TRUE;
			}
		}
	}

	phkdf_adtr->crypto_sel = crypto_sel;
	switch (hmac_alg_sel) {
	case HKDF_HMAC_SHA256:
		phkdf_adtr->digest_len = HKDF_SHA2_256_HASH_SIZE;
		phkdf_adtr->extract_key_len = HKDF_SHA256_KEY_LENGTH;
		phkdf_adtr->expand_key_len = HKDF_SHA256_KEY_LENGTH;
		phkdf_adtr->seq_hkdf_buf_len = HKDF_SHA2_256_HASH_SIZE;
		break;
	case HKDF_HMAC_SHA384:
		break;
	case HKDF_HMAC_SHA512:
		break;
	default:
		hmac_alg_sel = HKDF_HMAC_SHA256;
		phkdf_adtr->digest_len = HKDF_SHA2_256_HASH_SIZE;
		phkdf_adtr->extract_key_len = HKDF_SHA256_KEY_LENGTH;
		phkdf_adtr->expand_key_len = HKDF_SHA256_KEY_LENGTH;
		phkdf_adtr->seq_hkdf_buf_len = HKDF_SHA2_256_HASH_SIZE;
		break;
	}
	phkdf_adtr->hmac_alg_sel = hmac_alg_sel;
	phkdf_adtr->ori_en = ori_en;
	phkdf_adtr->isInit = _TRUE;
	ret = hal_rtl_hkdf_hook_operate_f(phkdf_adtr, NULL, NULL, NULL);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_deinit(hal_hkdf_adapter_t *phkdf_adtr)
{
	hal_status_t  ret = HAL_OK;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}
	phkdf_adtr->isInit = _FALSE;
	phkdf_adtr->isHWCrypto_Init = _FALSE;
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_hook_operate_f(hal_hkdf_adapter_t *phkdf_adtr, hkdf_hmac_init_func_t hmac_init_f,
		hkdf_hmac_update_func_t hmac_update_f, hkdf_hmac_final_func_t hmac_final_f)
{
	hal_status_t  ret = HAL_OK;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}

	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}

	hal_rtl_hkdf_hmac_init_hook(phkdf_adtr, hmac_init_f);
	hal_rtl_hkdf_hmac_update_hook(phkdf_adtr, hmac_update_f);
	hal_rtl_hkdf_hmac_final_hook(phkdf_adtr, hmac_final_f);
	return ret;
}

SECTION_HKDF_TEXT
static int hal_rtl_hkdf_hmac_sha2_256_init(void *arg, const u8 *key, const u32 keylen)
{
	hal_hkdf_adapter_t *phkdf_adtr = (hal_hkdf_adapter_t *)arg;
	int ret;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}
	if (HKDF_CRYPTO_HW_SEL_EN == phkdf_adtr->crypto_sel) {
		if (NULL == phkdf_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		if (HKDF_KEY_STG_SKCFG_NONE == phkdf_adtr->sk_cfg) {
			ret = hal_rtl_crypto_auth_init(phkdf_adtr->pcrypto_adtr, AUTH_TYPE_HMAC_SHA2_256_ALL, key, keylen);
		} else {
			ret = hal_rtl_crypto_auth_sk_init(phkdf_adtr->pcrypto_adtr, AUTH_TYPE_HMAC_SHA2_256_ALL, key, (phkdf_adtr->sk_cfg));
		}
	}
	return ret;
}

SECTION_HKDF_TEXT
static int hal_rtl_hkdf_hmac_sha2_256_update(void *arg, const u8 *message, const u32 msglen)
{
	hal_hkdf_adapter_t *phkdf_adtr = (hal_hkdf_adapter_t *)arg;
	int ret;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}
	if (HKDF_CRYPTO_HW_SEL_EN == phkdf_adtr->crypto_sel) {
		if (NULL == phkdf_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		ret = hal_rtl_crypto_auth_update(phkdf_adtr->pcrypto_adtr, AUTH_TYPE_HMAC_SHA2_256_ALL, message, msglen);
	}
	return ret;
}

SECTION_HKDF_TEXT
static int hal_rtl_hkdf_hmac_sha2_256_final(void *arg, u8 *pDigest)
{
	hal_hkdf_adapter_t *phkdf_adtr = (hal_hkdf_adapter_t *)arg;
	int ret;
	if (NULL == phkdf_adtr) {
		return HAL_ERR_PARA;
	}
	if (HKDF_CRYPTO_HW_SEL_EN == phkdf_adtr->crypto_sel) {
		if (NULL == phkdf_adtr->pcrypto_adtr) {
			return HAL_ERR_PARA;
		}
		if (HKDF_KEY_STG_SKCFG_NONE == phkdf_adtr->sk_cfg) {
			ret = hal_rtl_crypto_auth_final(phkdf_adtr->pcrypto_adtr, AUTH_TYPE_HMAC_SHA2_256_ALL, pDigest);
		} else {
			ret = hal_rtl_crypto_auth_sk_final(phkdf_adtr->pcrypto_adtr, AUTH_TYPE_HMAC_SHA2_256_ALL, pDigest);
		}
	}
	return ret;
}

SECTION_HKDF_TEXT
void hal_rtl_hkdf_hmac_init_hook(hal_hkdf_adapter_t *phkdf_adtr, hkdf_hmac_init_func_t hmac_init_f)
{
	u16 hmac_alg_sel = 0;
	if (NULL != hmac_init_f) {
		phkdf_adtr->hmac_init_f = hmac_init_f;
	} else {
		hmac_alg_sel = (phkdf_adtr->hmac_alg_sel);
		switch (hmac_alg_sel) {
		case HKDF_HMAC_SHA256:
			phkdf_adtr->hmac_init_f = hal_rtl_hkdf_hmac_sha2_256_init;
			break;
		case HKDF_HMAC_SHA384:
			break;
		case HKDF_HMAC_SHA512:
			break;
		default:
			phkdf_adtr->hmac_init_f = hal_rtl_hkdf_hmac_sha2_256_init;
			break;
		}
	}
}

SECTION_HKDF_TEXT
void hal_rtl_hkdf_hmac_update_hook(hal_hkdf_adapter_t *phkdf_adtr, hkdf_hmac_update_func_t hmac_update_f)
{
	u16 hmac_alg_sel = 0;
	if (NULL != hmac_update_f) {
		phkdf_adtr->hmac_update_f = hmac_update_f;
	} else {
		hmac_alg_sel = (phkdf_adtr->hmac_alg_sel);
		switch (hmac_alg_sel) {
		case HKDF_HMAC_SHA256:
			phkdf_adtr->hmac_update_f = hal_rtl_hkdf_hmac_sha2_256_update;
			break;
		case HKDF_HMAC_SHA384:
			break;
		case HKDF_HMAC_SHA512:
			break;
		default:
			phkdf_adtr->hmac_update_f = hal_rtl_hkdf_hmac_sha2_256_update;
			break;
		}
	}
}

SECTION_HKDF_TEXT
void hal_rtl_hkdf_hmac_final_hook(hal_hkdf_adapter_t *phkdf_adtr, hkdf_hmac_final_func_t hmac_final_f)
{
	u16 hmac_alg_sel = 0;
	if (NULL != hmac_final_f) {
		phkdf_adtr->hmac_final_f = hmac_final_f;
	} else {
		hmac_alg_sel = (phkdf_adtr->hmac_alg_sel);
		switch (hmac_alg_sel) {
		case HKDF_HMAC_SHA256:
			phkdf_adtr->hmac_final_f = hal_rtl_hkdf_hmac_sha2_256_final;
			break;
		case HKDF_HMAC_SHA384:
			break;
		case HKDF_HMAC_SHA512:
			break;
		default:
			phkdf_adtr->hmac_final_f = hal_rtl_hkdf_hmac_sha2_256_final;
			break;
		}
	}
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_secure_set_cfg(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_sel, const u8 sk_idx,
		const u8 wb_sel, const u8 wb_idx)
{
	hal_status_t ret = HAL_OK;
	hal_crypto_key_cfg_t key_cfg;
	hal_crypto_wb_cfg_t wb_cfg;

	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}

	phkdf_adtr->sk_cfg = 0x0;

	key_cfg.b.idx = sk_idx;
	key_cfg.b.sel = sk_sel;
	hal_rtl_crypto_set_sk_cfg_info(&(phkdf_adtr->sk_cfg), (key_cfg.w), CRYPTO_KEY_STG_ROLE_KEYCFG);
	DBG_CRYPTO_INFO("set key_cfg = 0x%x\r\n", key_cfg);

	wb_cfg.b.idx = wb_idx;
	wb_cfg.b.sel = wb_sel;
	hal_rtl_crypto_set_sk_cfg_info(&(phkdf_adtr->sk_cfg), (wb_cfg.w), CRYPTO_KEY_STG_ROLE_WBCFG);
	DBG_CRYPTO_INFO("set wb_cfg = 0x%x\r\n", wb_cfg);
	DBG_CRYPTO_INFO("set sk_cfg = 0x%x\r\n", phkdf_adtr->sk_cfg);

	return (ret);
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract_secure_set_cfg(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_sel, const u8 sk_idx,
		const u8 wb_sel, const u8 wb_idx)
{
	hal_status_t ret = HAL_OK;
	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}

	ret = hal_rtl_hkdf_secure_set_cfg(phkdf_adtr, sk_sel, sk_idx, wb_sel, wb_idx);
	if (HAL_OK == ret) {
		phkdf_adtr->key_extract_sk_sel = sk_sel;
		phkdf_adtr->extract_sk_idx = sk_idx;
		phkdf_adtr->key_extract_stg_wb_sel = wb_sel;
		phkdf_adtr->extract_wb_idx = wb_idx;
	}
	phkdf_adtr->use_key_stg = HKDF_USE_KEY_STG_EN;
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_secure_set_cfg(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_sel, const u8 sk_idx,
		const u8 wb_sel, const u8 wb_idx)
{
	hal_status_t ret = HAL_OK;
	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}

	ret = hal_rtl_hkdf_secure_set_cfg(phkdf_adtr, sk_sel, sk_idx, wb_sel, wb_idx);
	if (HAL_OK == ret) {
		phkdf_adtr->key_expand_sk_sel = sk_sel;
		phkdf_adtr->expand_sk_idx = sk_idx;
		phkdf_adtr->key_expand_stg_wb_sel = wb_sel;
		phkdf_adtr->expand_wb_idx = wb_idx;
	}
	phkdf_adtr->use_key_stg = HKDF_USE_KEY_STG_EN;
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract_engine(hal_hkdf_adapter_t *phkdf_adtr, const u8 *key_buf, const u8 *msg_buf, const u32 msg_len,
		u8 *pkm_buf)
{
	int crypto_ret;
	hal_status_t ret = HAL_OK;
	u32 key_len = 0;
	u32 info_len = 0;
	u8 *p_key = NULL;
	u8 *p_info = NULL;
	u8 *p_prk = NULL;

	p_key = key_buf;
	key_len = (phkdf_adtr->extract_key_len);
	info_len = msg_len;
	p_info = msg_buf;
	p_prk = pkm_buf;

	if (key_buf != NULL) {
		if ((u32)(key_buf) & 0x1F) {
			_memcpy(&(phkdf_adtr->ikm_buf[0]), key_buf, key_len);
			p_key = &(phkdf_adtr->ikm_buf[0]);
		} else {
			p_key = key_buf;
		}
	}

	crypto_ret = phkdf_adtr->hmac_init_f(phkdf_adtr, p_key, key_len);
	if (crypto_ret != SUCCESS) {
		ret = HAL_ERR_PARA;
		goto __hal_rtl_hkdf_extract_engine_exit;
	}

	crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, p_info, info_len);
	if (crypto_ret != SUCCESS) {
		ret = HAL_ERR_PARA;
		goto __hal_rtl_hkdf_extract_engine_exit;
	}

	crypto_ret = phkdf_adtr->hmac_final_f(phkdf_adtr, p_prk);
	if (crypto_ret != SUCCESS) {
		ret = HAL_ERR_PARA;
		goto __hal_rtl_hkdf_extract_engine_exit;
	}

__hal_rtl_hkdf_extract_engine_exit:
	return (ret);
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_engine(hal_hkdf_adapter_t *phkdf_adtr, const u8 *key_buf, const u8 *msg_buf, const u32 msg_len,
										u8 *okm_buf)
{
	hal_status_t ret = HAL_OK;
	int crypto_ret = 0;
	u32 info_len = 0;
	u32 where = 0;
	u32 okm_len = 0;
	u32 n = 0;
	u32 i = 0;
	u32 t_len = 0;
	u32 hash_len = 32;
	u32 t_seq_len = 0;
	u32 prk_len = 0;
	u8 *p_tmp_seq = NULL;
	u8 *p_prk = NULL;
	u8 *p_info = NULL;
	u8 *p_okm = NULL;

	p_prk = key_buf;
	prk_len = (phkdf_adtr->expand_key_len);
	p_info = msg_buf;
	info_len = msg_len;
	p_okm = okm_buf;
	okm_len = (phkdf_adtr->okm_len);
	hash_len = (phkdf_adtr->digest_len);

	if (key_buf != NULL) {
		if ((u32)(key_buf) & 0x1F) {
			_memcpy(&(phkdf_adtr->prk_buf[0]), key_buf, prk_len);
			p_prk = &(phkdf_adtr->prk_buf[0]);
		} else {
			p_prk = key_buf;
		}
	}

	if (KEY_STG_WBTYPE_WB_ONLY_STG != phkdf_adtr->key_expand_stg_wb_sel) {
		p_tmp_seq = &(phkdf_adtr->seq_hkdf_buf[0]);
		t_seq_len = (phkdf_adtr->seq_hkdf_buf_len);
		if (p_okm == NULL) {
			ret = HAL_ERR_PARA;
			goto __hal_rtl_hkdf_expand_engine_exit;
		}
	}

	if (p_info == NULL) {
		p_info = (const unsigned char *) "";
		info_len = 0;
	}

	n = okm_len / hash_len;

	if ((okm_len % hash_len) != 0) {
		n++;
	}

	/*
	 * Per RFC 5869 Section 2.3, okm_len must not exceed
	 * 255 times the hash length
	 */
	if (n > 255) {
		ret =  HAL_ERR_PARA;
		goto __hal_rtl_hkdf_expand_engine_exit;
	}

	/*
	 * Compute T = T(1) | T(2) | T(3) | ... | T(N)
	 * Where T(N) is defined in RFC 5869 Section 2.3
	 */
	for (i = 1; i <= n; i++) {
		size_t num_to_copy;
		unsigned char c = i & 0xff;

		crypto_ret = phkdf_adtr->hmac_init_f(phkdf_adtr, p_prk, prk_len);
		if (crypto_ret != SUCCESS) {
			ret = HAL_ERR_PARA;
			goto __hal_rtl_hkdf_expand_engine_exit;
		}

		if (t_len > 0) {
			crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, p_tmp_seq, t_len);
			if (crypto_ret != SUCCESS) {
				ret = HAL_ERR_PARA;
				goto __hal_rtl_hkdf_expand_engine_exit;
			}
		}

		if ((info_len > 0) && (p_info != NULL)) {
			crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, p_info, info_len);
			if (crypto_ret != SUCCESS) {
				ret = HAL_ERR_PARA;
				goto __hal_rtl_hkdf_expand_engine_exit;
			}
		}

		/*  if info extend(info2/3) register, then hkdf include those info
		 *  For example: use this for device life cycle changed
		 */
		if ((phkdf_adtr->info2_len > 0) && (phkdf_adtr->info2 != NULL)) {
			crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, (phkdf_adtr->info2), (phkdf_adtr->info2_len));
			if (crypto_ret != SUCCESS) {
				ret = HAL_ERR_PARA;
				goto __hal_rtl_hkdf_expand_engine_exit;
			}
		}

		if ((phkdf_adtr->info3_len > 0) && (phkdf_adtr->info3 != NULL)) {
			crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, (phkdf_adtr->info3), (phkdf_adtr->info3_len));
			if (crypto_ret != SUCCESS) {
				ret = HAL_ERR_PARA;
				goto __hal_rtl_hkdf_expand_engine_exit;
			}
		}

		/* The constant concatenated to the end of each T(n) is a single octet.
		 * */
		crypto_ret = phkdf_adtr->hmac_update_f(phkdf_adtr, &c, 1);
		if (crypto_ret != SUCCESS) {
			ret = HAL_ERR_PARA;
			goto __hal_rtl_hkdf_expand_engine_exit;
		}

		crypto_ret = phkdf_adtr->hmac_final_f(phkdf_adtr, p_tmp_seq);
		if (crypto_ret != SUCCESS) {
			ret = HAL_ERR_PARA;
			goto __hal_rtl_hkdf_expand_engine_exit;
		}

		if (KEY_STG_WBTYPE_WB_ONLY_STG != phkdf_adtr->key_expand_stg_wb_sel) {
			num_to_copy = i != n ? hash_len : (okm_len - where);
			_memcpy(p_okm + where, p_tmp_seq, num_to_copy);
			where += hash_len;
			t_len = hash_len;
		}
	}

__hal_rtl_hkdf_expand_engine_exit:
	if (KEY_STG_WBTYPE_WB_ONLY_STG != phkdf_adtr->key_expand_stg_wb_sel) {
		_memset(p_tmp_seq, 0x0, t_seq_len);
	}

	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract_secure(hal_hkdf_adapter_t *phkdf_adtr, const u8 *key_buf, const u8 *msg_buf,
		const u32 msg_len, u8 *pkm_buf)
{
	hal_status_t ret = HAL_OK;
	u8 *p_key = NULL;
	u8 *p_km = NULL;

	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}
	if (KEY_STG_SKTYPE_LD_SK != phkdf_adtr->key_extract_sk_sel) {
		p_key = key_buf;
	}
	if (KEY_STG_WBTYPE_WB_ONLY_STG != phkdf_adtr->key_extract_stg_wb_sel) {
		p_km = pkm_buf;
	}
	ret = hal_rtl_hkdf_extract_engine(phkdf_adtr, p_key, msg_buf,  msg_len, p_km);

	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract_secure_all(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_idx, const u8 wb_idx, const u8 *msg_buf)
{
	hal_status_t  ret = HAL_OK;
	uint8_t *p_hshks = NULL;
	if (msg_buf == NULL) {
		return HAL_ERR_PARA;
	} else {
		p_hshks = msg_buf;
	}

	ret = hal_rtl_hkdf_extract_secure_set_cfg(phkdf_adtr, KEY_STG_SKTYPE_LD_SK, sk_idx, KEY_STG_WBTYPE_WB_ONLY_STG, wb_idx);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = hal_rtl_hkdf_extract_secure(phkdf_adtr, NULL, p_hshks, HKDF_SHA2_256_HASH_SIZE, NULL);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_extract(hal_hkdf_adapter_t *phkdf_adtr, const u8 *salt, const u32 salt_len,
								  const u8 *ikm, const u32 ikm_len, u8 *prk)
{
	hal_status_t ret = HAL_OK;
	u8 *p_key = NULL;
	u8 *p_msg = NULL;
	u32 msg_len = 0;

	if (HKDF_ORI_SEL_EN == phkdf_adtr->ori_en) {
		if (NULL == salt) {
			return HAL_ERR_MEM;
		}
		if (salt_len != (phkdf_adtr->extract_key_len)) {
			return HAL_ERR_PARA;
		}
		p_key = (u8 *)salt;
		p_msg = (u8 *)ikm;
		msg_len = ikm_len;
	} else {
		if (NULL == ikm) {
			return HAL_ERR_MEM;
		}
		if (ikm_len != (phkdf_adtr->extract_key_len)) {
			return HAL_ERR_PARA;
		}
		p_key = (u8 *)ikm;
		p_msg = (u8 *)salt;
		msg_len = salt_len;
	}
	ret = hal_rtl_hkdf_extract_engine(phkdf_adtr, p_key, p_msg, msg_len, prk);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_register_info_extend(hal_hkdf_adapter_t *phkdf_adtr, const u8 *info, const u32 info_len, const u8 info_role)
{
	hal_status_t ret = HAL_OK;

	if (NULL == info) {
		return HAL_ERR_MEM;
	} else {
		if (0 == info_len) {
			return HAL_ERR_PARA;
		}
	}

	// register info 2/3
	switch (info_role) {
	case HKDF_EXPAND_INFO2:
		phkdf_adtr->info2 = info;
		phkdf_adtr->info2_len = info_len;
		break;
	case HKDF_EXPAND_INFO3:
		phkdf_adtr->info3 = info;
		phkdf_adtr->info3_len = info_len;
		break;
	default:
		break;
	}

	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_unregister_info_extend(hal_hkdf_adapter_t *phkdf_adtr, const u8 info_role)
{
	hal_status_t ret = HAL_OK;

	// unregister info 2/3
	switch (info_role) {
	case HKDF_EXPAND_INFO2:
		phkdf_adtr->info2 = NULL;
		phkdf_adtr->info2_len = 0;
		break;
	case HKDF_EXPAND_INFO3:
		phkdf_adtr->info3 = NULL;
		phkdf_adtr->info3_len = 0;
		break;
	default:
		break;
	}

	return ret;
}


SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_secure(hal_hkdf_adapter_t *phkdf_adtr, const u8 *prk, const u8 *nonce,
										const u32 nonce_len, u8 *okm, u32 okm_len)
{
	hal_status_t ret = HAL_OK;
	u8 *p_key = NULL;
	u8 *p_msg = NULL;
	u8 *p_okm = NULL;
	u32 msg_len = 0;

	if (phkdf_adtr->isInit != _TRUE) {
		return HAL_NOT_READY; // not init yet
	}
	if (okm_len != HKDF_SHA2_256_HASH_SIZE) {
		return HAL_ERR_PARA;
	} else {
		phkdf_adtr->okm_len = okm_len;
	}
	if (KEY_STG_SKTYPE_LD_SK != phkdf_adtr->key_expand_sk_sel) {
		p_key = prk;
	}
	if (KEY_STG_WBTYPE_WB_ONLY_STG != phkdf_adtr->key_expand_stg_wb_sel) {
		p_okm = okm;
	}

	p_msg = (u8 *)nonce;
	msg_len = nonce_len;
	ret = hal_rtl_hkdf_expand_engine(phkdf_adtr, p_key, p_msg, msg_len, p_okm);

	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand_secure_all(hal_hkdf_adapter_t *phkdf_adtr, const u8 sk_idx, const u8 wb_idx, const u8 *nonce)
{
	hal_status_t  ret = HAL_OK;
	uint8_t *p_hshkn = NULL;
	if (nonce == NULL) {
		return HAL_ERR_PARA;
	} else {
		p_hshkn = nonce;
	}

	ret = hal_rtl_hkdf_expand_secure_set_cfg(phkdf_adtr, KEY_STG_SKTYPE_LD_SK, sk_idx, KEY_STG_WBTYPE_WB_ONLY_STG, wb_idx);
	if (ret != HAL_OK) {
		return ret;
	}
	ret = hal_rtl_hkdf_expand_secure(phkdf_adtr, NULL, p_hshkn, HKDF_SHA2_256_HASH_SIZE, NULL, HKDF_SHA2_256_HASH_SIZE);
	return ret;
}

SECTION_HKDF_TEXT
hal_status_t hal_rtl_hkdf_expand(hal_hkdf_adapter_t *phkdf_adtr, const u8 *prk,
								 const u8 *nonce, const u32 nonce_len, u8 *okm, u32 okm_len)
{
	hal_status_t ret = HAL_OK;
	u8 *p_key = NULL;
	u8 *p_msg = NULL;
	u8 *p_okm = NULL;
	u32 msg_len = 0;

	p_key = (u8 *)prk;
	p_msg = (u8 *)nonce;
	msg_len = nonce_len;
	p_okm = (u8 *)okm;
	phkdf_adtr->okm_len = okm_len;

	ret = hal_rtl_hkdf_expand_engine(phkdf_adtr, p_key, p_msg, msg_len, p_okm);
	return ret;
}
#endif

#endif // end of "#if CONFIG_ROM_HKDF_EN"
