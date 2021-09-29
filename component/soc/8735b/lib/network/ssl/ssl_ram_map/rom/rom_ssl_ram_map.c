#include <rom_ssl_ram_map.h>
#include <section_config.h>

/* RAM table referred by SSL ROM */
SSL_RAM_MAP_SECTION
struct _rom_ssl_ram_map rom_ssl_ram_map;

SSL_ROM_TEXT_SECTION
void _init_rom_ssl_ram_map(
	void *(*ssl_calloc)(unsigned int, unsigned int),
	void (*ssl_free)(void *),
	int (*ssl_printf)(const char *, ...),
	u32 use_hw_crypto_func
)
{
	rom_ssl_ram_map.ssl_calloc = ssl_calloc;
	rom_ssl_ram_map.ssl_free = ssl_free;
	rom_ssl_ram_map.ssl_printf = ssl_printf;
	rom_ssl_ram_map.use_hw_crypto_func = use_hw_crypto_func;
}

SSL_ROM_TEXT_SECTION
void _init_rom_ssl_hw_crypto_aes_ecb(
	int (*hw_crypto_aes_ecb_init)(const u8 *, const u32),
	int (*hw_crypto_aes_ecb_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_ecb_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
)
{
	rom_ssl_ram_map.hw_crypto_aes_ecb_init = hw_crypto_aes_ecb_init;
	rom_ssl_ram_map.hw_crypto_aes_ecb_decrypt = hw_crypto_aes_ecb_decrypt;
	rom_ssl_ram_map.hw_crypto_aes_ecb_encrypt = hw_crypto_aes_ecb_encrypt;
}

SSL_ROM_TEXT_SECTION
void _init_rom_ssl_hw_crypto_aes_cbc(
	int (*hw_crypto_aes_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_aes_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
)
{
	rom_ssl_ram_map.hw_crypto_aes_cbc_init = hw_crypto_aes_cbc_init;
	rom_ssl_ram_map.hw_crypto_aes_cbc_decrypt = hw_crypto_aes_cbc_decrypt;
	rom_ssl_ram_map.hw_crypto_aes_cbc_encrypt = hw_crypto_aes_cbc_encrypt;
}

SSL_ROM_TEXT_SECTION
void _init_rom_ssl_hw_crypto_des_cbc(
	int (*hw_crypto_des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
)
{
	rom_ssl_ram_map.hw_crypto_des_cbc_init = hw_crypto_des_cbc_init;
	rom_ssl_ram_map.hw_crypto_des_cbc_decrypt = hw_crypto_des_cbc_decrypt;
	rom_ssl_ram_map.hw_crypto_des_cbc_encrypt = hw_crypto_des_cbc_encrypt;
}

SSL_ROM_TEXT_SECTION
void _init_rom_ssl_hw_crypto_3des_cbc(
	int (*hw_crypto_3des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_3des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_3des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
)
{
	rom_ssl_ram_map.hw_crypto_3des_cbc_init = hw_crypto_3des_cbc_init;
	rom_ssl_ram_map.hw_crypto_3des_cbc_decrypt = hw_crypto_3des_cbc_decrypt;
	rom_ssl_ram_map.hw_crypto_3des_cbc_encrypt = hw_crypto_3des_cbc_encrypt;
}