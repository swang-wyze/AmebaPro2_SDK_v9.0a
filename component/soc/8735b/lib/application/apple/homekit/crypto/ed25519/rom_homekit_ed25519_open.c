#include <string.h>
#if 0
#include <limits.h>
#include "api.h"
#include "crypto_hash_sha512.h"
#include "crypto_verify_32.h"
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

APPLE_ROM_TEXT_SECTION
static int crypto_verify_32(const unsigned char *x, const unsigned char *y)
{
	unsigned int differentbits = 0;
#define F(i) differentbits |= x[i] ^ y[i];
	F(0)
	F(1)
	F(2)
	F(3)
	F(4)
	F(5)
	F(6)
	F(7)
	F(8)
	F(9)
	F(10)
	F(11)
	F(12)
	F(13)
	F(14)
	F(15)
	F(16)
	F(17)
	F(18)
	F(19)
	F(20)
	F(21)
	F(22)
	F(23)
	F(24)
	F(25)
	F(26)
	F(27)
	F(28)
	F(29)
	F(30)
	F(31)
	return (1 & ((differentbits - 1) >> 8)) - 1;
}

APPLE_ROM_TEXT_SECTION
static int
sodium_memcmp(const void *const b1_, const void *const b2_, size_t len)
{
	const unsigned char *b1 = (const unsigned char *) b1_;
	const unsigned char *b2 = (const unsigned char *) b2_;
	size_t               i;
	unsigned char        d = (unsigned char) 0U;

	for (i = 0U; i < len; i++) {
		d |= b1[i] ^ b2[i];
	}
	return (int)((1 & ((d - 1) >> 8)) - 1);
}

APPLE_ROM_TEXT_SECTION
int
rom_ed25519_crypto_sign_verify_detached(const unsigned char *sig, const unsigned char *m,
										unsigned long long mlen, const unsigned char *pk)
{
//  crypto_hash_sha512_state hs;
	sha512_context hs;

	unsigned char h[64];
	unsigned char rcheck[32];
	unsigned int  i;
	unsigned char d = 0;
	ge_p3 A;
	ge_p2 R;

	if (sig[63] & 224) {
		return -1;
	}
	if (rom_ed25519_ge_frombytes_negate_vartime(&A, pk) != 0) {
		return -1;
	}
	for (i = 0; i < 32; ++i) {
		d |= pk[i];
	}
	if (d == 0) {
		return -1;
	}
#if 0
	crypto_hash_sha512_init(&hs);
	crypto_hash_sha512_update(&hs, sig, 32);
	crypto_hash_sha512_update(&hs, pk, 32);
	crypto_hash_sha512_update(&hs, m, mlen);
	crypto_hash_sha512_final(&hs, h);
#else
	rom_sha512_starts(&hs, 0);
	rom_sha512_update(&hs, sig, 32);
	rom_sha512_update(&hs, pk, 32);
	rom_sha512_update(&hs, m, mlen);
	rom_sha512_finish(&hs, h);
#endif
	rom_ed25519_sc_reduce(h);

	rom_ed25519_ge_double_scalarmult_vartime(&R, h, &A, sig + 32);
	rom_ed25519_ge_tobytes(rcheck, &R);

	return crypto_verify_32(rcheck, sig) | (-(rcheck - sig == 0)) |
		   sodium_memcmp(sig, rcheck, 32);
}
#if 0
int
crypto_sign_open(unsigned char *m, unsigned long long *mlen,
				 const unsigned char *sm, unsigned long long smlen,
				 const unsigned char *pk)
{
	if (smlen < 64 || smlen > SIZE_MAX) {
		goto badsig;
	}
	if (crypto_sign_verify_detached(sm, sm + 64, smlen - 64, pk) != 0) {
		memset(m, 0, smlen - 64);
		goto badsig;
	}
	*mlen = smlen - 64;
	memmove(m, sm + 64, *mlen);

	return 0;

badsig:
	*mlen = 0;
	return -1;
}
#endif
