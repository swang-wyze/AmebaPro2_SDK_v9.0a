#include "rom/rom_apple_func_stubs.h"

extern const apple_func_stubs_t __rom_stubs_apple;

/* ed25519 */
int rom_ed25519_gen_keypair(unsigned char pk[ED25519_PUBKEY_LEN], unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN],
							unsigned char seed[ED25519_SKEY_LEN])
{
	return __rom_stubs_apple.rom_ed25519_gen_keypair(pk, skpk, seed);
}

int rom_ed25519_gen_signature(unsigned char sig[ED25519_SIG_SIZE], const unsigned char *m, unsigned long long mlen,
							  const unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN])
{
	return __rom_stubs_apple.rom_ed25519_gen_signature(sig, m, mlen, skpk);
}

int rom_ed25519_verify_signature(const unsigned char sig[ED25519_SIG_SIZE], const unsigned char *m, unsigned long long mlen,
								 const unsigned char pk[ED25519_PUBKEY_LEN])
{
	return __rom_stubs_apple.rom_ed25519_verify_signature(sig, m, mlen, pk);
}

/* sha512 */
void rom_sha512_starts(sha512_context *ctx, int is384)
{
	__rom_stubs_apple.rom_sha512_starts(ctx, is384);
}

void rom_sha512_update(sha512_context *ctx, const unsigned char *input, size_t ilen)
{
	__rom_stubs_apple.rom_sha512_update(ctx, input, ilen);
}

void rom_sha512_finish(sha512_context *ctx, unsigned char output[64])
{
	__rom_stubs_apple.rom_sha512_finish(ctx, output);
}

void rom_sha512(const unsigned char *input, size_t ilen, unsigned char output[64], int is384)
{
	__rom_stubs_apple.rom_sha512(input, ilen, output, is384);
}

void rom_sha512_hmac_starts(sha512_context *ctx, const unsigned char *key, size_t keylen, int is384)
{
	__rom_stubs_apple.rom_sha512_hmac_starts(ctx, key, keylen, is384);
}

void rom_sha512_hmac_update(sha512_context *ctx, const unsigned char *input, size_t ilen)
{
	__rom_stubs_apple.rom_sha512_hmac_update(ctx, input, ilen);
}

void rom_sha512_hmac_finish(sha512_context *ctx, unsigned char output[64])
{
	__rom_stubs_apple.rom_sha512_hmac_finish(ctx, output);
}

void rom_sha512_hmac_reset(sha512_context *ctx)
{
	__rom_stubs_apple.rom_sha512_hmac_reset(ctx);
}

void rom_sha512_hmac(const unsigned char *key, size_t keylen, const unsigned char *input, size_t ilen, unsigned char output[64], int is384)
{
	__rom_stubs_apple.rom_sha512_hmac(key, keylen, input, ilen, output, is384);
}

int rom_sha512_hkdf(const unsigned char *ikm, size_t ikm_len, const unsigned char *salt, size_t salt_len, const unsigned char *info, size_t info_len,
					unsigned char *okm, size_t okm_len)
{
	return __rom_stubs_apple.rom_sha512_hkdf(ikm, ikm_len, salt, salt_len, info, info_len, okm, okm_len);
}

/* curve25519 */
void curve25519_donna(unsigned char *outKey, const unsigned char *inSecret, const unsigned char *inBasePoint)
{
	__rom_stubs_apple.curve25519_donna(outKey, inSecret, inBasePoint);
}

