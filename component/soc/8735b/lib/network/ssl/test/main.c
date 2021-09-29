#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include "hal_crypto.h"
#include "mbedtls/platform.h"

void init_rom_ssl_ram_map(
	void *(*ssl_calloc)(unsigned int, unsigned int),
	void (*ssl_free)(void *),
	int (*ssl_printf)(const char *, ...),
	u32 use_hw_crypto_func
);

void init_rom_ssl_hw_crypto_aes_ecb(
	int (*hw_crypto_aes_ecb_init)(const u8 *, const u32),
	int (*hw_crypto_aes_ecb_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_ecb_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);

void init_rom_ssl_hw_crypto_aes_cbc(
	int (*hw_crypto_aes_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_aes_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);

void init_rom_ssl_hw_crypto_des_cbc(
	int (*hw_crypto_des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);

void init_rom_ssl_hw_crypto_3des_cbc(
	int (*hw_crypto_3des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_3des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_3des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);

extern int mbedtls_mpi_self_test(int verbose);
extern int mbedtls_ecp_self_test(int verbose);
extern int mbedtls_sha1_self_test(int verbose);
extern int mbedtls_sha256_self_test(int verbose);
extern int mbedtls_sha512_self_test(int verbose);
extern int mbedtls_md5_self_test(int verbose);
extern int mbedtls_aes_self_test(int verbose);
extern int mbedtls_des_self_test(int verbose);
extern int mbedtls_base64_self_test(int verbose);
extern int mbedtls_rsa_self_test(int verbose);
extern int mbedtls_ctr_drbg_self_test(int verbose);
extern int mbedtls_hmac_drbg_self_test(int verbose);
extern int mbedtls_dhm_self_test(int verbose);
extern int mbedtls_ecjpake_self_test(int verbose);
extern int mbedtls_arc4_self_test(int verbose);

static void *_calloc_func(size_t nelements, size_t elementSize)
{
	size_t size;
	void *ptr = NULL;

	size = nelements * elementSize;
	ptr = pvPortMalloc(size);

	if (ptr) {
		memset(ptr, 0, size);
	}

	return ptr;
}

void test_thread(void *param)
{
	// if use config_all.h to test from RAM
	mbedtls_platform_set_calloc_free(_calloc_func, vPortFree);

	init_rom_ssl_ram_map(_calloc_func, vPortFree, NULL, 0);

	printf("\n mbedtls_mpi_self_test: %s\n", (mbedtls_mpi_self_test(1) == 0) ? "pass" : "failed");

	// ECP test need to define MBEDTLS_SELF_TEST in ecp.c
	//printf("\n mbedtls_ecp_self_test: %s\n", (mbedtls_ecp_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_sha1_self_test: %s\n", (mbedtls_sha1_self_test(1) == 0) ? "pass" : "failed");
	printf("\n mbedtls_sha256_self_test: %s\n", (mbedtls_sha256_self_test(1) == 0) ? "pass" : "failed");
	printf("\n mbedtls_sha512_self_test: %s\n", (mbedtls_sha512_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_md5_self_test: %s\n", (mbedtls_md5_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_aes_self_test: %s\n", (mbedtls_aes_self_test(1) == 0) ? "pass" : "failed");
	printf("\n mbedtls_des_self_test: %s\n", (mbedtls_des_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_base64_self_test: %s\n", (mbedtls_base64_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_rsa_self_test: %s\n", (mbedtls_rsa_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_ctr_drbg_self_test: %s\n", (mbedtls_ctr_drbg_self_test(1) == 0) ? "pass" : "failed");
	printf("\n mbedtls_hmac_drbg_self_test: %s\n", (mbedtls_hmac_drbg_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_dhm_self_test: %s\n", (mbedtls_dhm_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_ecjpake_self_test: %s\n", (mbedtls_ecjpake_self_test(1) == 0) ? "pass" : "failed");

	printf("\n mbedtls_arc4_self_test: %s\n", (mbedtls_arc4_self_test(1) == 0) ? "pass" : "failed");
#if 0
	// test hw crypto
	hal_crypto_engine_init();
	init_rom_ssl_ram_map(_calloc_func, vPortFree, NULL, 1);
	init_rom_ssl_hw_crypto_aes_ecb(hal_crypto_aes_ecb_init, hal_crypto_aes_ecb_decrypt, hal_crypto_aes_ecb_encrypt);
	init_rom_ssl_hw_crypto_aes_cbc(hal_crypto_aes_cbc_init, hal_crypto_aes_cbc_decrypt, hal_crypto_aes_cbc_encrypt);
//	init_rom_ssl_hw_crypto_des_cbc(hal_crypto_des_cbc_init, hal_crypto_des_cbc_decrypt, hal_crypto_des_cbc_encrypt);
//	init_rom_ssl_hw_crypto_3des_cbc(hal_crypto_3des_cbc_init, hal_crypto_3des_cbc_decrypt, hal_crypto_3des_cbc_encrypt);

	printf("\n mbedtls_aes_self_test with hw crypto: %s\n", (mbedtls_aes_self_test(1) == 0) ? "pass" : "failed");
//	printf("\n mbedtls_des_self_test with hw crypto: %s\n", (mbedtls_des_self_test(1) == 0) ? "pass" : "failed");
#endif
	vTaskDelete(NULL);
}

int main(void)
{
	if (xTaskCreate(test_thread, ((const char *)"test_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(test_thread) failed", __FUNCTION__);
	}

	vTaskStartScheduler();
	while (1);
	return 0;
}
