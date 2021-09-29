#include "rom_apple_func_stubs.h"
#include <section_config.h>

// ed25519
int _rom_ed25519_gen_keypair(unsigned char pk[ED25519_PUBKEY_LEN], unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN],
							 unsigned char seed[ED25519_SKEY_LEN]);
int _rom_ed25519_gen_signature(unsigned char sig[ED25519_SIG_SIZE], const unsigned char *m, unsigned long long mlen,
							   const unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN]);
int _rom_ed25519_verify_signature(const unsigned char sig[ED25519_SIG_SIZE], const unsigned char *m, unsigned long long mlen,
								  const unsigned char pk[ED25519_PUBKEY_LEN]);
// sha512
void _rom_sha512_starts(sha512_context *ctx, int is384);
void _rom_sha512_update(sha512_context *ctx, const unsigned char *input, size_t ilen);
void _rom_sha512_finish(sha512_context *ctx, unsigned char output[64]);
void _rom_sha512(const unsigned char *input, size_t ilen, unsigned char output[64], int is384);
void _rom_sha512_hmac_starts(sha512_context *ctx, const unsigned char *key, size_t keylen, int is384);
void _rom_sha512_hmac_update(sha512_context *ctx, const unsigned char *input, size_t ilen);
void _rom_sha512_hmac_finish(sha512_context *ctx, unsigned char output[64]);
void _rom_sha512_hmac_reset(sha512_context *ctx);
void _rom_sha512_hmac(const unsigned char *key, size_t keylen, const unsigned char *input, size_t ilen, unsigned char output[64], int is384);
int _rom_sha512_hkdf(const unsigned char *ikm, size_t ikm_len, const unsigned char *salt, size_t salt_len, const unsigned char *info, size_t info_len,
					 unsigned char *okm, size_t okm_len);
// curve25519
void _curve25519_donna(unsigned char *outKey, const unsigned char *inSecret, const unsigned char *inBasePoint);

APPLE_ROM_STUB_SECTION
const apple_func_stubs_t apple_func_stubs = {
	// ed25519
	.rom_ed25519_gen_keypair = _rom_ed25519_gen_keypair,
	.rom_ed25519_gen_signature = _rom_ed25519_gen_signature,
	.rom_ed25519_verify_signature = _rom_ed25519_verify_signature,
	// sha512
	.rom_sha512_starts = _rom_sha512_starts,
	.rom_sha512_update = _rom_sha512_update,
	.rom_sha512_finish = _rom_sha512_finish,
	.rom_sha512 = _rom_sha512,
	.rom_sha512_hmac_starts = _rom_sha512_hmac_starts,
	.rom_sha512_hmac_update = _rom_sha512_hmac_update,
	.rom_sha512_hmac_finish = _rom_sha512_hmac_finish,
	.rom_sha512_hmac_reset = _rom_sha512_hmac_reset,
	.rom_sha512_hmac = _rom_sha512_hmac,
	.rom_sha512_hkdf = _rom_sha512_hkdf,
	// curve25519
	.curve25519_donna = _curve25519_donna
};
