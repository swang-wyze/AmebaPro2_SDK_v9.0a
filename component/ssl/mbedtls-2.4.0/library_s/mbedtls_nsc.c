#include "cmsis.h"
#include "platform_stdlib.h"

#include "mbedtls/config.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"
#include "crypto_api.h"
#if defined(ENABLE_AMAZON_COMMON)
#include "threading_alt.h"
#include "mbedtls/threading.h"
#endif

//extern const char *client_key_s;
// maybe replaced by client_key_s in main_s.c if it is existed
__weak const char *client_key_s = \
"-----BEGIN EC PARAMETERS-----\r\n" \
"BggqhkjOPQMBBw==\r\n" \
"-----END EC PARAMETERS-----\r\n" \
"-----BEGIN EC PRIVATE KEY-----\r\n" \
"MHcCAQEEIAQxciQmaeuPLUa8VueFj9fTEdqbqLY8jW84NuKtxf+ToAoGCCqGSM49\r\n" \
"AwEHoUQDQgAEtxzt0vQIGEeVZMklv+ZJnkjpSj9IlhZhRyfY4rFyieaD3wo3cnw0\r\n" \
"LJTKAEZCGC7y+4qzkzFY/FVUG+zxwkWVsw==\r\n" \
"-----END EC PRIVATE KEY-----\r\n";

static void* _calloc(size_t count, size_t size)
{
	void *ptr = pvPortMalloc(count * size);
	if(ptr)	memset(ptr, 0, count * size);
	return ptr;
}

#define _free		vPortFree

static int _random(void *p_rng, unsigned char *output, size_t output_len)
{
	/* To avoid gcc warnings */
	( void ) p_rng;

	static unsigned int seed = 0;
	if(seed == 0) {
#if defined(CONFIG_PLATFORM_8710C)
		crypto_random_generate((uint8_t *)&seed, sizeof(seed));
#elif (defined CONFIG_PLATFORM_8721D)|| (defined CONFIG_PLATFORM_AMEBAD2)
		extern u32 RandSeedTZ;
		seed = RandSeedTZ;
#endif
		srand(seed);
	}

	int rand_num = 0;
	while(output_len) {
		int r = rand();
		if(output_len > sizeof(int)) {
			memcpy(&output[rand_num], &r, sizeof(int));
			rand_num += sizeof(int);
			output_len -= sizeof(int);
		}
		else {
			memcpy(&output[rand_num], &r, output_len);
			rand_num += output_len;
			output_len = 0;
		}
	}

	return 0;
}

#if defined(__ICCARM__)
void (__cmse_nonsecure_call *ns_device_mutex_lock)(uint32_t) = NULL;
void (__cmse_nonsecure_call *ns_device_mutex_unlock)(uint32_t) = NULL;
#else
void __attribute__((cmse_nonsecure_call)) (*ns_device_mutex_lock)(uint32_t) = NULL;
void __attribute__((cmse_nonsecure_call)) (*ns_device_mutex_unlock)(uint32_t) = NULL;
#endif

void NS_ENTRY secure_set_ns_device_lock(
	void (*device_mutex_lock_func)(uint32_t),
	void (*device_mutex_unlock_func)(uint32_t))
{
#if defined(__ICCARM__)
	ns_device_mutex_lock = cmse_nsfptr_create((void (__cmse_nonsecure_call *)(uint32_t)) device_mutex_lock_func);
	ns_device_mutex_unlock = cmse_nsfptr_create((void (__cmse_nonsecure_call *)(uint32_t)) device_mutex_unlock_func);
#else
	ns_device_mutex_lock = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(uint32_t)) device_mutex_lock_func);
	ns_device_mutex_unlock = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(uint32_t)) device_mutex_unlock_func);
#endif
}

void NS_ENTRY secure_mbedtls_ssl_conf_rng(mbedtls_ssl_config *conf, void *p_rng)
{
	mbedtls_ssl_conf_rng(conf, _random, p_rng);
}

int NS_ENTRY secure_mbedtls_platform_set_calloc_free(void)
{
	return 	mbedtls_platform_set_calloc_free(_calloc, _free);
}

mbedtls_pk_context* NS_ENTRY secure_mbedtls_pk_parse_key(void)
{

	mbedtls_pk_context *client_pk = (mbedtls_pk_context *) mbedtls_calloc(1, sizeof(mbedtls_pk_context));

	if(client_pk) {
		mbedtls_pk_init(client_pk);

		if(mbedtls_pk_parse_key(client_pk, (unsigned char const *)client_key_s, strlen(client_key_s) + 1, NULL, 0) != 0) {
			printf("\n\r ERROR: mbedtls_pk_parse_key \n\r");
			goto error;
		}
	}
	else {
		printf("\n\r ERROR: mbedtls_calloc \n\r");
		goto error;
	}

	return client_pk;

error:
	if(client_pk) {
		mbedtls_pk_free(client_pk);
		mbedtls_free(client_pk);
	}

	return NULL;
}

#if defined(ENABLE_AMAZON_COMMON)
mbedtls_pk_type_t NS_ENTRY secure_mbedtls_pk_get_type(const mbedtls_pk_context *ctx)
{
        return mbedtls_pk_get_type(ctx);
}
#endif

void NS_ENTRY secure_mbedtls_pk_free(mbedtls_pk_context *pk)
{
	mbedtls_pk_free(pk);
	mbedtls_free(pk);
}

int NS_ENTRY secure_mbedtls_pk_can_do(const mbedtls_pk_context *ctx, mbedtls_pk_type_t type)
{
	return mbedtls_pk_can_do(ctx, type);
}

unsigned char NS_ENTRY secure_mbedtls_ssl_sig_from_pk(mbedtls_pk_context *pk)
{
#if defined(MBEDTLS_RSA_C)
    if( mbedtls_pk_can_do( pk, MBEDTLS_PK_RSA ) )
        return( MBEDTLS_SSL_SIG_RSA );
#endif
#if defined(MBEDTLS_ECDSA_C)
    if( mbedtls_pk_can_do( pk, MBEDTLS_PK_ECDSA ) )
        return( MBEDTLS_SSL_SIG_ECDSA );
#endif
    return( MBEDTLS_SSL_SIG_ANON );
}

struct secure_mbedtls_pk_sign_param {
	mbedtls_pk_context *ctx;
	mbedtls_md_type_t md_alg;
	unsigned char *hash;
	size_t hash_len;
	unsigned char *sig;
	size_t *sig_len;
	int (*f_rng)(void *, unsigned char *, size_t);
	void *p_rng;
};

int NS_ENTRY secure_mbedtls_pk_sign(struct secure_mbedtls_pk_sign_param *param)
{

	return mbedtls_pk_sign(param->ctx, param->md_alg, param->hash, param->hash_len,
			param->sig, param->sig_len, _random, param->p_rng);
}

#if defined(ENABLE_AMAZON_COMMON)
void s_mbedtls_mutex_init( mbedtls_threading_mutex_t * mutex )
{
   
}

void s_mbedtls_mutex_free( mbedtls_threading_mutex_t * mutex )
{
    
}

int s_mbedtls_mutex_lock( mbedtls_threading_mutex_t * mutex )
{
    __disable_irq();
    return 0;
}

int s_mbedtls_mutex_unlock( mbedtls_threading_mutex_t * mutex )
{
    __enable_irq();
    return 0;
}

void (*mbedtls_mutex_init)( mbedtls_threading_mutex_t * ) = s_mbedtls_mutex_init;
void (*mbedtls_mutex_free)( mbedtls_threading_mutex_t * ) = s_mbedtls_mutex_free;
int (*mbedtls_mutex_lock)( mbedtls_threading_mutex_t * ) = s_mbedtls_mutex_lock;
int (*mbedtls_mutex_unlock)( mbedtls_threading_mutex_t * ) = s_mbedtls_mutex_unlock;
#endif