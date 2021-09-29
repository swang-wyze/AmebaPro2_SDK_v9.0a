#include <string.h>
#if 0
#include "api.h"
#include "crypto_hash_sha512.h"
#include "ge.h"
#include "sc.h"
#include "utils.h"
#else
#include <rom_homekit_sha512.h>
#include "core/rom_homekit_ed25519_ge.h"
#include "core/rom_homekit_ed25519_sc.h"
#endif
#include <rom_apple_func_rename.h>

#include "basic_types.h"
#define APPLE_ROM_TEXT_SECTION  SECTION(".rom.apple.text")
#define APPLE_ROM_DATA_SECTION  SECTION(".rom.apple.dtcm")

#define memcpy _memcpy

APPLE_ROM_TEXT_SECTION
int
rom_ed25519_crypto_sign_detached(unsigned char *sig, unsigned long long *siglen,
								 const unsigned char *m, unsigned long long mlen,
								 const unsigned char *sk)
{
//  crypto_hash_sha512_state hs;
	sha512_context hs;

	unsigned char pk[32];
	unsigned char az[64];
	unsigned char nonce[64];
	unsigned char hram[64];
	ge_p3 R;

//  memmove(pk, sk + 32, 32);
	memcpy(pk, sk + 32, 32);

//  crypto_hash_sha512(az, sk, 32);
	rom_sha512(sk, 32, az, 0);

	az[0] &= 248;
	az[31] &= 63;
	az[31] |= 64;
#if 0
	crypto_hash_sha512_init(&hs);
	crypto_hash_sha512_update(&hs, az + 32, 32);
	crypto_hash_sha512_update(&hs, m, mlen);
	crypto_hash_sha512_final(&hs, nonce);
#else
	rom_sha512_starts(&hs, 0);
	rom_sha512_update(&hs, az + 32, 32);
	rom_sha512_update(&hs, m, mlen);
	rom_sha512_finish(&hs, nonce);
#endif
//  memmove(sig + 32, pk, 32);
	memcpy(sig + 32, pk, 32);

	rom_ed25519_sc_reduce(nonce);
	rom_ed25519_ge_scalarmult_base(&R, nonce);
	rom_ed25519_ge_p3_tobytes(sig, &R);
#if 0
	crypto_hash_sha512_init(&hs);
	crypto_hash_sha512_update(&hs, sig, 64);
	crypto_hash_sha512_update(&hs, m, mlen);
	crypto_hash_sha512_final(&hs, hram);
#else
	rom_sha512_starts(&hs, 0);
	rom_sha512_update(&hs, sig, 64);
	rom_sha512_update(&hs, m, mlen);
	rom_sha512_finish(&hs, hram);
#endif
	rom_ed25519_sc_reduce(hram);
	rom_ed25519_sc_muladd(sig + 32, hram, az, nonce);
#if 0
	sodium_memzero(az, sizeof az);
	sodium_memzero(nonce, sizeof nonce);
#endif
	if (siglen != NULL) {
		*siglen = 64U;
	}
	return 0;
}
#if 0
int
crypto_sign(unsigned char *sm, unsigned long long *smlen,
			const unsigned char *m, unsigned long long mlen,
			const unsigned char *sk)
{
	unsigned long long siglen;

	memmove(sm + crypto_sign_ed25519_BYTES, m, mlen);
	/* LCOV_EXCL_START */
	if (crypto_sign_detached(sm, &siglen, sm + crypto_sign_ed25519_BYTES,
							 mlen, sk) != 0 ||
		siglen != crypto_sign_ed25519_BYTES) {
		if (smlen != NULL) {
			*smlen = 0;
		}
		memset(sm, 0, mlen + crypto_sign_ed25519_BYTES);
		return -1;
	}
	/* LCOV_EXCL_STOP */

	if (smlen != NULL) {
		*smlen = mlen + siglen;
	}
	return 0;
}
#endif
