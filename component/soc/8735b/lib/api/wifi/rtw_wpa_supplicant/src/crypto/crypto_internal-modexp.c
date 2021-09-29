/*
 * Crypto wrapper for internal crypto implementation - modexp
 * Copyright (c) 2006-2009, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */
#include <drv_types.h>

#ifdef CONFIG_WPS

#include "utils/includes.h"

#include "utils/common.h"
#include "tls/rom/rom_wps_big_num.h"
#include "crypto.h"


// dh_group5_generator[1] = { 0x02 }, 1, ?, 192, dh_group5_prime[192], 192, ?, ?
int crypto_mod_exp(const u8 *base, size_t base_len,
				   const u8 *power, size_t power_len,
				   const u8 *modulus, size_t modulus_len,
				   u8 *result, size_t *result_len)
{
	struct bignum *bn_base, *bn_exp, *bn_modulus, *bn_result;
	int ret = -1;
	//int i;

	bn_base = bignum_init();
	////DBG_871X("bn_base->used = %d,bn_base->alloc = %d,bn_base->sign = %d",bn_base->used,bn_base->alloc,bn_base->sign);
	//DBG_871X("bn_exp");
	bn_exp = bignum_init();
	////DBG_871X("bn_exp->used = %d,bn_exp->alloc = %d,bn_exp->sign = %d",bn_exp->used,bn_exp->alloc,bn_exp->sign);
	//DBG_871X("bn_modulus");
	bn_modulus = bignum_init();
	////DBG_871X("bn_modulus->used = %d,bn_modulus->alloc = %d,bn_modulus->sign = %d",bn_modulus->used,bn_modulus->alloc,bn_modulus->sign);
	//DBG_871X("bn_result");
	bn_result = bignum_init();
	////DBG_871X("bn_result->used = %d,bn_result->alloc = %d,bn_result->sign = %d",bn_result->used,bn_result->alloc,bn_result->sign);

	if (bn_base == NULL || bn_exp == NULL || bn_modulus == NULL || bn_result == NULL) {
		DBG_871X("bn_base || bn_exp || bn_modulus || bn_result == NULL");
		goto error;
	}

	if (bignum_set_unsigned_bin(bn_base, base, base_len) < 0 ||
		bignum_set_unsigned_bin(bn_exp, power, power_len) < 0 ||
		bignum_set_unsigned_bin(bn_modulus, modulus, modulus_len) < 0) {
		DBG_871X("bignum_set_unsigned_bin fail!!!");
		goto error;
	}

	if (bignum_exptmod(bn_base, bn_exp, bn_modulus, bn_result) < 0) {
		DBG_871X("bignum_exptmod fail!!!");
		goto error;
	}

	ret = bignum_get_unsigned_bin(bn_result, result, result_len);
	//DBG_871X("bignum_get_unsigned_bin() => return = %d", ret);
error:
	bignum_deinit(bn_base);
	bignum_deinit(bn_exp);
	bignum_deinit(bn_modulus);
	bignum_deinit(bn_result);

	//DBG_871X("<===crypto_mod_exp()");
	return ret;
}
#endif
