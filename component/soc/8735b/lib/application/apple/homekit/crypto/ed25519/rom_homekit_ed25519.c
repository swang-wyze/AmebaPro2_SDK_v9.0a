#include <stdlib.h>

#include "rom_homekit_ed25519.h"
#include <rom_apple_func_rename.h>

#include "basic_types.h"
#define APPLE_ROM_TEXT_SECTION  SECTION(".rom.apple.text")
#define APPLE_ROM_DATA_SECTION  SECTION(".rom.apple.dtcm")

APPLE_ROM_TEXT_SECTION
int rom_ed25519_gen_keypair(unsigned char pk[ED25519_PUBKEY_LEN],
							unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN],
							unsigned char seed[ED25519_SKEY_LEN])
{
	return rom_ed25519_crypto_sign_seed_keypair(pk, skpk, seed);
}

APPLE_ROM_TEXT_SECTION
int rom_ed25519_gen_signature(unsigned char sig[ED25519_SIG_SIZE],
							  const unsigned char *m, unsigned long long mlen,
							  const unsigned char skpk[ED25519_SKEY_LEN + ED25519_PUBKEY_LEN])
{
	return rom_ed25519_crypto_sign_detached(sig, NULL, m, mlen, skpk);
}

APPLE_ROM_TEXT_SECTION
int rom_ed25519_verify_signature(const unsigned char sig[ED25519_SIG_SIZE],
								 const unsigned char *m, unsigned long long mlen,
								 const unsigned char pk[ED25519_PUBKEY_LEN])
{
	return rom_ed25519_crypto_sign_verify_detached(sig, m, mlen, pk);
}

