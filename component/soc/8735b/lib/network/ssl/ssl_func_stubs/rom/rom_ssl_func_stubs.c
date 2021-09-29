#include "rom_ssl_func_stubs.h"
#include <section_config.h>

// ssl_ram_map
void _init_rom_ssl_ram_map(
	void *(*ssl_calloc)(unsigned int, unsigned int),
	void (*ssl_free)(void *),
	int (*ssl_printf)(const char *, ...),
	u32 use_hw_crypto_func
);
void _init_rom_ssl_hw_crypto_aes_ecb(
	int (*hw_crypto_aes_ecb_init)(const u8 *, const u32),
	int (*hw_crypto_aes_ecb_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_ecb_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);
void _init_rom_ssl_hw_crypto_aes_cbc(
	int (*hw_crypto_aes_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_aes_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_aes_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);
void _init_rom_ssl_hw_crypto_des_cbc(
	int (*hw_crypto_des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);
void _init_rom_ssl_hw_crypto_3des_cbc(
	int (*hw_crypto_3des_cbc_init)(const u8 *, const u32),
	int (*hw_crypto_3des_cbc_decrypt)(const u8 *, const u32, const u8 *, const u32, u8 *),
	int (*hw_crypto_3des_cbc_encrypt)(const u8 *, const u32, const u8 *, const u32, u8 *)
);
// bignum
void _mbedtls_mpi_init(mbedtls_mpi *X);
void _mbedtls_mpi_free(mbedtls_mpi *X);
int _mbedtls_mpi_grow(mbedtls_mpi *X, size_t nblimbs);
int _mbedtls_mpi_shrink(mbedtls_mpi *X, size_t nblimbs);
int _mbedtls_mpi_copy(mbedtls_mpi *X, const mbedtls_mpi *Y);
void _mbedtls_mpi_swap(mbedtls_mpi *X, mbedtls_mpi *Y);
int _mbedtls_mpi_safe_cond_assign(mbedtls_mpi *X, const mbedtls_mpi *Y, unsigned char assign);
int _mbedtls_mpi_safe_cond_swap(mbedtls_mpi *X, mbedtls_mpi *Y, unsigned char swap);
int _mbedtls_mpi_lset(mbedtls_mpi *X, mbedtls_mpi_sint z);
int _mbedtls_mpi_get_bit(const mbedtls_mpi *X, size_t pos);
int _mbedtls_mpi_set_bit(mbedtls_mpi *X, size_t pos, unsigned char val);
size_t _mbedtls_mpi_lsb(const mbedtls_mpi *X);
size_t _mbedtls_mpi_bitlen(const mbedtls_mpi *X);
size_t _mbedtls_mpi_size(const mbedtls_mpi *X);
int _mbedtls_mpi_read_binary(mbedtls_mpi *X, const unsigned char *buf, size_t buflen);
int _mbedtls_mpi_write_binary(const mbedtls_mpi *X, unsigned char *buf, size_t buflen);
int _mbedtls_mpi_shift_l(mbedtls_mpi *X, size_t count);
int _mbedtls_mpi_shift_r(mbedtls_mpi *X, size_t count);
int _mbedtls_mpi_cmp_abs(const mbedtls_mpi *X, const mbedtls_mpi *Y);
int _mbedtls_mpi_cmp_mpi(const mbedtls_mpi *X, const mbedtls_mpi *Y);
int _mbedtls_mpi_cmp_int(const mbedtls_mpi *X, mbedtls_mpi_sint z);
int _mbedtls_mpi_add_abs(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_sub_abs(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_add_mpi(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_sub_mpi(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_add_int(mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_sint b);
int _mbedtls_mpi_sub_int(mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_sint b);
int _mbedtls_mpi_mul_mpi(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_read_string(mbedtls_mpi *X, int radix, const char *s);
int _mbedtls_mpi_mul_int(mbedtls_mpi *X, const mbedtls_mpi *A, mbedtls_mpi_uint b);
int _mbedtls_mpi_div_mpi(mbedtls_mpi *Q, mbedtls_mpi *R, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_div_int(mbedtls_mpi *Q, mbedtls_mpi *R, const mbedtls_mpi *A, mbedtls_mpi_sint b);
int _mbedtls_mpi_mod_mpi(mbedtls_mpi *R, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_mod_int(mbedtls_mpi_uint *r, const mbedtls_mpi *A, mbedtls_mpi_sint b);
int _mbedtls_mpi_write_string(const mbedtls_mpi *X, int radix, char *buf, size_t buflen, size_t *olen);
int _mbedtls_mpi_exp_mod(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *E, const mbedtls_mpi *N, mbedtls_mpi *_RR);
int _mbedtls_mpi_gcd(mbedtls_mpi *G, const mbedtls_mpi *A, const mbedtls_mpi *B);
int _mbedtls_mpi_fill_random(mbedtls_mpi *X, size_t size, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_mpi_inv_mod(mbedtls_mpi *X, const mbedtls_mpi *A, const mbedtls_mpi *N);
int _mbedtls_mpi_is_prime(const mbedtls_mpi *X, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_mpi_gen_prime(mbedtls_mpi *X, size_t nbits, int dh_flag, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
// ecp
const mbedtls_ecp_curve_info *_mbedtls_ecp_curve_list(void);
const mbedtls_ecp_curve_info *_mbedtls_ecp_curve_info_from_grp_id(mbedtls_ecp_group_id grp_id);
const mbedtls_ecp_curve_info *_mbedtls_ecp_curve_info_from_tls_id(uint16_t tls_id);
const mbedtls_ecp_curve_info *_mbedtls_ecp_curve_info_from_name(const char *name);
void _mbedtls_ecp_point_init(mbedtls_ecp_point *pt);
void _mbedtls_ecp_group_init(mbedtls_ecp_group *grp);
void _mbedtls_ecp_keypair_init(mbedtls_ecp_keypair *key);
void _mbedtls_ecp_point_free(mbedtls_ecp_point *pt);
void _mbedtls_ecp_group_free(mbedtls_ecp_group *grp);
void _mbedtls_ecp_keypair_free(mbedtls_ecp_keypair *key);
int _mbedtls_ecp_copy(mbedtls_ecp_point *P, const mbedtls_ecp_point *Q);
int _mbedtls_ecp_group_copy(mbedtls_ecp_group *dst, const mbedtls_ecp_group *src);
int _mbedtls_ecp_set_zero(mbedtls_ecp_point *pt);
int _mbedtls_ecp_is_zero(mbedtls_ecp_point *pt);
int _mbedtls_ecp_point_cmp(const mbedtls_ecp_point *P, const mbedtls_ecp_point *Q);
int _mbedtls_ecp_point_read_string(mbedtls_ecp_point *P, int radix, const char *x, const char *y);
int _mbedtls_ecp_point_write_binary(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *P, int format, size_t *olen, unsigned char *buf, size_t buflen);
int _mbedtls_ecp_point_read_binary(const mbedtls_ecp_group *grp, mbedtls_ecp_point *pt, const unsigned char *buf, size_t ilen);
int _mbedtls_ecp_tls_read_point(const mbedtls_ecp_group *grp, mbedtls_ecp_point *pt, const unsigned char **buf, size_t buf_len);
int _mbedtls_ecp_tls_write_point(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *pt, int format, size_t *olen, unsigned char *buf, size_t blen);
int _mbedtls_ecp_tls_read_group(mbedtls_ecp_group *grp, const unsigned char **buf, size_t len);
int _mbedtls_ecp_tls_write_group(const mbedtls_ecp_group *grp, size_t *olen, unsigned char *buf, size_t blen);
int _mbedtls_ecp_check_pubkey(const mbedtls_ecp_group *grp, const mbedtls_ecp_point *pt);
int _mbedtls_ecp_check_privkey(const mbedtls_ecp_group *grp, const mbedtls_mpi *d);
int _mbedtls_ecp_mul(mbedtls_ecp_group *grp, mbedtls_ecp_point *R, const mbedtls_mpi *m, const mbedtls_ecp_point *P, int (*f_rng)(void *, unsigned char *,
					 size_t), void *p_rng);
int _mbedtls_ecp_muladd(mbedtls_ecp_group *grp, mbedtls_ecp_point *R, const mbedtls_mpi *m, const mbedtls_ecp_point *P, const mbedtls_mpi *n,
						const mbedtls_ecp_point *Q);
int _mbedtls_ecp_gen_keypair_base(mbedtls_ecp_group *grp, const mbedtls_ecp_point *G, mbedtls_mpi *d, mbedtls_ecp_point *Q, int (*f_rng)(void *,
								  unsigned char *, size_t), void *p_rng);
int _mbedtls_ecp_gen_keypair(mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecp_gen_key(mbedtls_ecp_group_id grp_id, mbedtls_ecp_keypair *key, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecp_check_pub_priv(const mbedtls_ecp_keypair *pub, const mbedtls_ecp_keypair *prv);
// ecp_curves
int _mbedtls_ecp_group_load(mbedtls_ecp_group *grp, mbedtls_ecp_group_id id);
// sha1
void _mbedtls_sha1_init(mbedtls_sha1_context *ctx);
void _mbedtls_sha1_free(mbedtls_sha1_context *ctx);
void _mbedtls_sha1_clone(mbedtls_sha1_context *dst, const mbedtls_sha1_context *src);
void _mbedtls_sha1_starts(mbedtls_sha1_context *ctx);
void _mbedtls_sha1_process(mbedtls_sha1_context *ctx, const unsigned char data[64]);
void _mbedtls_sha1_update(mbedtls_sha1_context *ctx, const unsigned char *input, size_t ilen);
void _mbedtls_sha1_finish(mbedtls_sha1_context *ctx, unsigned char output[20]);
void _mbedtls_sha1(const unsigned char *input, size_t ilen, unsigned char output[20]);
// sha256
void _mbedtls_sha256_init(mbedtls_sha256_context *ctx);
void _mbedtls_sha256_free(mbedtls_sha256_context *ctx);
void _mbedtls_sha256_clone(mbedtls_sha256_context *dst, const mbedtls_sha256_context *src);
void _mbedtls_sha256_starts(mbedtls_sha256_context *ctx, int is224);
void _mbedtls_sha256_process(mbedtls_sha256_context *ctx, const unsigned char data[64]);
void _mbedtls_sha256_update(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen);
void _mbedtls_sha256_finish(mbedtls_sha256_context *ctx, unsigned char output[32]);
void _mbedtls_sha256(const unsigned char *input, size_t ilen, unsigned char output[32], int is224);
// sha512
void _mbedtls_sha512_init(mbedtls_sha512_context *ctx);
void _mbedtls_sha512_free(mbedtls_sha512_context *ctx);
void _mbedtls_sha512_clone(mbedtls_sha512_context *dst, const mbedtls_sha512_context *src);
void _mbedtls_sha512_starts(mbedtls_sha512_context *ctx, int is384);
void _mbedtls_sha512_process(mbedtls_sha512_context *ctx, const unsigned char data[128]);
void _mbedtls_sha512_update(mbedtls_sha512_context *ctx, const unsigned char *input, size_t ilen);
void _mbedtls_sha512_finish(mbedtls_sha512_context *ctx, unsigned char output[64]);
void _mbedtls_sha512(const unsigned char *input, size_t ilen, unsigned char output[64], int is384);
// oid
int _mbedtls_oid_get_attr_short_name(const mbedtls_asn1_buf *oid, const char **short_name);
int _mbedtls_oid_get_x509_ext_type(const mbedtls_asn1_buf *oid, int *ext_type);
int _mbedtls_oid_get_extended_key_usage(const mbedtls_asn1_buf *oid, const char **description);
int _mbedtls_oid_get_sig_alg_desc(const mbedtls_asn1_buf *oid, const char **description);
int _mbedtls_oid_get_sig_alg(const mbedtls_asn1_buf *oid, mbedtls_md_type_t *md_alg, mbedtls_pk_type_t *pk_alg);
int _mbedtls_oid_get_oid_by_sig_alg(mbedtls_pk_type_t pk_alg, mbedtls_md_type_t md_alg, const char **oid, size_t *olen);
int _mbedtls_oid_get_pk_alg(const mbedtls_asn1_buf *oid, mbedtls_pk_type_t *pk_alg);
int _mbedtls_oid_get_oid_by_pk_alg(mbedtls_pk_type_t pk_alg, const char **oid, size_t *olen);
int _mbedtls_oid_get_ec_grp(const mbedtls_asn1_buf *oid, mbedtls_ecp_group_id *grp_id);
int _mbedtls_oid_get_oid_by_ec_grp(mbedtls_ecp_group_id grp_id, const char **oid, size_t *olen);
int _mbedtls_oid_get_cipher_alg(const mbedtls_asn1_buf *oid, mbedtls_cipher_type_t *cipher_alg);
int _mbedtls_oid_get_md_alg(const mbedtls_asn1_buf *oid, mbedtls_md_type_t *md_alg);
int _mbedtls_oid_get_oid_by_md(mbedtls_md_type_t md_alg, const char **oid, size_t *olen);
int _mbedtls_oid_get_pkcs12_pbe_alg(const mbedtls_asn1_buf *oid, mbedtls_md_type_t *md_alg, mbedtls_cipher_type_t *cipher_alg);
// md5
void _mbedtls_md5_init(mbedtls_md5_context *ctx);
void _mbedtls_md5_free(mbedtls_md5_context *ctx);
void _mbedtls_md5_clone(mbedtls_md5_context *dst, const mbedtls_md5_context *src);
void _mbedtls_md5_starts(mbedtls_md5_context *ctx);
void _mbedtls_md5_process(mbedtls_md5_context *ctx, const unsigned char data[64]);
void _mbedtls_md5_update(mbedtls_md5_context *ctx, const unsigned char *input, size_t ilen);
void _mbedtls_md5_finish(mbedtls_md5_context *ctx, unsigned char output[16]);
void _mbedtls_md5(const unsigned char *input, size_t ilen, unsigned char output[16]);
// aes
void _mbedtls_aes_init(mbedtls_aes_context *ctx);
void _mbedtls_aes_free(mbedtls_aes_context *ctx);
int _mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits);
int _mbedtls_aes_setkey_dec(mbedtls_aes_context *ctx, const unsigned char *key, unsigned int keybits);
void _mbedtls_aes_encrypt(mbedtls_aes_context *ctx, const unsigned char input[16], unsigned char output[16]);
void _mbedtls_aes_decrypt(mbedtls_aes_context *ctx, const unsigned char input[16], unsigned char output[16]);
int _mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx, int mode, const unsigned char input[16], unsigned char output[16]);
int _mbedtls_aes_crypt_cbc(mbedtls_aes_context *ctx, int mode, size_t length, unsigned char iv[16], const unsigned char *input, unsigned char *output);
int _mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx, int mode, size_t length, size_t *iv_off, unsigned char iv[16], const unsigned char *input,
							  unsigned char *output);
int _mbedtls_aes_crypt_cfb8(mbedtls_aes_context *ctx, int mode, size_t length, unsigned char iv[16], const unsigned char *input, unsigned char *output);
int _mbedtls_aes_crypt_ctr(mbedtls_aes_context *ctx, size_t length, size_t *nc_off, unsigned char nonce_counter[16], unsigned char stream_block[16],
						   const unsigned char *input, unsigned char *output);
// des
void _mbedtls_des_init(mbedtls_des_context *ctx);
void _mbedtls_des_free(mbedtls_des_context *ctx);
void _mbedtls_des3_init(mbedtls_des3_context *ctx);
void _mbedtls_des3_free(mbedtls_des3_context *ctx);
void _mbedtls_des_key_set_parity(unsigned char key[MBEDTLS_DES_KEY_SIZE]);
int _mbedtls_des_key_check_key_parity(const unsigned char key[MBEDTLS_DES_KEY_SIZE]);
int _mbedtls_des_key_check_weak(const unsigned char key[MBEDTLS_DES_KEY_SIZE]);
void _mbedtls_des_setkey(uint32_t SK[32], const unsigned char key[MBEDTLS_DES_KEY_SIZE]);
int _mbedtls_des_setkey_enc(mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE]);
int _mbedtls_des_setkey_dec(mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE]);
int _mbedtls_des3_set2key_enc(mbedtls_des3_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2]);
int _mbedtls_des3_set2key_dec(mbedtls_des3_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2]);
int _mbedtls_des3_set3key_enc(mbedtls_des3_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3]);
int _mbedtls_des3_set3key_dec(mbedtls_des3_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3]);
int _mbedtls_des_crypt_ecb(mbedtls_des_context *ctx, const unsigned char input[8], unsigned char output[8]);
int _mbedtls_des_crypt_cbc(mbedtls_des_context *ctx, int mode, size_t length, unsigned char iv[8], const unsigned char *input, unsigned char *output);
int _mbedtls_des3_crypt_ecb(mbedtls_des3_context *ctx, const unsigned char input[8], unsigned char output[8]);
int _mbedtls_des3_crypt_cbc(mbedtls_des3_context *ctx, int mode, size_t length, unsigned char iv[8], const unsigned char *input, unsigned char *output);
// md
const int *_mbedtls_md_list(void);
const mbedtls_md_info_t *_mbedtls_md_info_from_string(const char *md_name);
const mbedtls_md_info_t *_mbedtls_md_info_from_type(mbedtls_md_type_t md_type);
void _mbedtls_md_init(mbedtls_md_context_t *ctx);
void _mbedtls_md_free(mbedtls_md_context_t *ctx);
int _mbedtls_md_clone(mbedtls_md_context_t *dst, const mbedtls_md_context_t *src);
int _mbedtls_md_setup(mbedtls_md_context_t *ctx, const mbedtls_md_info_t *md_info, int hmac);
int _mbedtls_md_init_ctx(mbedtls_md_context_t *ctx, const mbedtls_md_info_t *md_info);
int _mbedtls_md_starts(mbedtls_md_context_t *ctx);
int _mbedtls_md_update(mbedtls_md_context_t *ctx, const unsigned char *input, size_t ilen);
int _mbedtls_md_finish(mbedtls_md_context_t *ctx, unsigned char *output);
int _mbedtls_md(const mbedtls_md_info_t *md_info, const unsigned char *input, size_t ilen, unsigned char *output);
int _mbedtls_md_hmac_starts(mbedtls_md_context_t *ctx, const unsigned char *key, size_t keylen);
int _mbedtls_md_hmac_update(mbedtls_md_context_t *ctx, const unsigned char *input, size_t ilen);
int _mbedtls_md_hmac_finish(mbedtls_md_context_t *ctx, unsigned char *output);
int _mbedtls_md_hmac_reset(mbedtls_md_context_t *ctx);
int _mbedtls_md_hmac(const mbedtls_md_info_t *md_info, const unsigned char *key, size_t keylen, const unsigned char *input, size_t ilen, unsigned char *output);
int _mbedtls_md_process(mbedtls_md_context_t *ctx, const unsigned char *data);
unsigned char _mbedtls_md_get_size(const mbedtls_md_info_t *md_info);
mbedtls_md_type_t _mbedtls_md_get_type(const mbedtls_md_info_t *md_info);
const char *_mbedtls_md_get_name(const mbedtls_md_info_t *md_info);
// asn1parse
int _mbedtls_asn1_get_len(unsigned char **p, const unsigned char *end, size_t *len);
int _mbedtls_asn1_get_tag(unsigned char **p, const unsigned char *end, size_t *len, int tag);
int _mbedtls_asn1_get_bool(unsigned char **p, const unsigned char *end, int *val);
int _mbedtls_asn1_get_int(unsigned char **p, const unsigned char *end, int *val);
int _mbedtls_asn1_get_mpi(unsigned char **p, const unsigned char *end, mbedtls_mpi *X);
int _mbedtls_asn1_get_bitstring(unsigned char **p, const unsigned char *end, mbedtls_asn1_bitstring *bs);
int _mbedtls_asn1_get_bitstring_null(unsigned char **p, const unsigned char *end, size_t *len);
int _mbedtls_asn1_get_sequence_of(unsigned char **p, const unsigned char *end, mbedtls_asn1_sequence *cur, int tag);
int _mbedtls_asn1_get_alg(unsigned char **p, const unsigned char *end, mbedtls_asn1_buf *alg, mbedtls_asn1_buf *params);
int _mbedtls_asn1_get_alg_null(unsigned char **p, const unsigned char *end, mbedtls_asn1_buf *alg);
void _mbedtls_asn1_free_named_data(mbedtls_asn1_named_data *cur);
void _mbedtls_asn1_free_named_data_list(mbedtls_asn1_named_data **head);
mbedtls_asn1_named_data *_mbedtls_asn1_find_named_data(mbedtls_asn1_named_data *list, const char *oid, size_t len);
// asn1write
int _mbedtls_asn1_write_len(unsigned char **p, unsigned char *start, size_t len);
int _mbedtls_asn1_write_tag(unsigned char **p, unsigned char *start, unsigned char tag);
int _mbedtls_asn1_write_raw_buffer(unsigned char **p, unsigned char *start, const unsigned char *buf, size_t size);
int _mbedtls_asn1_write_mpi(unsigned char **p, unsigned char *start, const mbedtls_mpi *X);
int _mbedtls_asn1_write_null(unsigned char **p, unsigned char *start);
int _mbedtls_asn1_write_oid(unsigned char **p, unsigned char *start, const char *oid, size_t oid_len);
int _mbedtls_asn1_write_algorithm_identifier(unsigned char **p, unsigned char *start, const char *oid, size_t oid_len, size_t par_len);
int _mbedtls_asn1_write_bool(unsigned char **p, unsigned char *start, int boolean);
int _mbedtls_asn1_write_int(unsigned char **p, unsigned char *start, int val);
int _mbedtls_asn1_write_printable_string(unsigned char **p, unsigned char *start, const char *text, size_t text_len);
int _mbedtls_asn1_write_ia5_string(unsigned char **p, unsigned char *start, const char *text, size_t text_len);
int _mbedtls_asn1_write_bitstring(unsigned char **p, unsigned char *start, const unsigned char *buf, size_t bits);
int _mbedtls_asn1_write_octet_string(unsigned char **p, unsigned char *start, const unsigned char *buf, size_t size);
mbedtls_asn1_named_data *_mbedtls_asn1_store_named_data(mbedtls_asn1_named_data **head, const char *oid, size_t oid_len, const unsigned char *val,
		size_t val_len);
// base64
int _mbedtls_base64_encode(unsigned char *dst, size_t dlen, size_t *olen, const unsigned char *src, size_t slen);
int _mbedtls_base64_decode(unsigned char *dst, size_t dlen, size_t *olen, const unsigned char *src, size_t slen);
// rsa
void _mbedtls_rsa_init(mbedtls_rsa_context *ctx, int padding, int hash_id);
void _mbedtls_rsa_set_padding(mbedtls_rsa_context *ctx, int padding, int hash_id);
int _mbedtls_rsa_check_pubkey(const mbedtls_rsa_context *ctx);
int _mbedtls_rsa_check_privkey(const mbedtls_rsa_context *ctx);
int _mbedtls_rsa_check_pub_priv(const mbedtls_rsa_context *pub, const mbedtls_rsa_context *prv);
int _mbedtls_rsa_public(mbedtls_rsa_context *ctx, const unsigned char *input, unsigned char *output);
int _mbedtls_rsa_private(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, const unsigned char *input,
						 unsigned char *output);
int _mbedtls_rsa_rsaes_oaep_encrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, const unsigned char *label,
									size_t label_len, size_t ilen, const unsigned char *input, unsigned char *output);
int _mbedtls_rsa_rsaes_pkcs1_v15_encrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, size_t ilen,
		const unsigned char *input, unsigned char *output);
int _mbedtls_rsa_pkcs1_encrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, size_t ilen,
							   const unsigned char *input, unsigned char *output);
int _mbedtls_rsa_rsaes_oaep_decrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, const unsigned char *label,
									size_t label_len, size_t *olen, const unsigned char *input, unsigned char *output, size_t output_max_len);
int _mbedtls_rsa_rsaes_pkcs1_v15_decrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, size_t *olen,
		const unsigned char *input, unsigned char *output, size_t output_max_len);
int _mbedtls_rsa_pkcs1_decrypt(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, size_t *olen,
							   const unsigned char *input, unsigned char *output, size_t output_max_len);
int _mbedtls_rsa_rsassa_pss_sign(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
								 unsigned int hashlen, const unsigned char *hash, unsigned char *sig);
int _mbedtls_rsa_rsassa_pkcs1_v15_sign(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
									   unsigned int hashlen, const unsigned char *hash, unsigned char *sig);
int _mbedtls_rsa_pkcs1_sign(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
							unsigned int hashlen, const unsigned char *hash, unsigned char *sig);
int _mbedtls_rsa_rsassa_pss_verify_ext(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
									   unsigned int hashlen, const unsigned char *hash, mbedtls_md_type_t mgf1_hash_id, int expected_salt_len, const unsigned char *sig);
int _mbedtls_rsa_rsassa_pss_verify(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
								   unsigned int hashlen, const unsigned char *hash, const unsigned char *sig);
int _mbedtls_rsa_rsassa_pkcs1_v15_verify(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode,
		mbedtls_md_type_t md_alg, unsigned int hashlen, const unsigned char *hash, const unsigned char *sig);
int _mbedtls_rsa_pkcs1_verify(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, int mode, mbedtls_md_type_t md_alg,
							  unsigned int hashlen, const unsigned char *hash, const unsigned char *sig);
void _mbedtls_rsa_free(mbedtls_rsa_context *ctx);
int _mbedtls_rsa_gen_key(mbedtls_rsa_context *ctx, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng, unsigned int nbits, int exponent);
int _mbedtls_rsa_copy(mbedtls_rsa_context *dst, const mbedtls_rsa_context *src);
// ctr_drbg
void _mbedtls_ctr_drbg_init(mbedtls_ctr_drbg_context *ctx);
void _mbedtls_ctr_drbg_free(mbedtls_ctr_drbg_context *ctx);
void _mbedtls_ctr_drbg_set_prediction_resistance(mbedtls_ctr_drbg_context *ctx, int resistance);
void _mbedtls_ctr_drbg_set_entropy_len(mbedtls_ctr_drbg_context *ctx, size_t len);
void _mbedtls_ctr_drbg_set_reseed_interval(mbedtls_ctr_drbg_context *ctx, int interval);
void _mbedtls_ctr_drbg_update(mbedtls_ctr_drbg_context *ctx, const unsigned char *additional, size_t add_len);
int _mbedtls_ctr_drbg_reseed(mbedtls_ctr_drbg_context *ctx, const unsigned char *additional, size_t len);
int _mbedtls_ctr_drbg_seed_entropy_len(mbedtls_ctr_drbg_context *ctx, int (*f_entropy)(void *, unsigned char *, size_t), void *p_entropy,
									   const unsigned char *custom, size_t len, size_t entropy_len);
int _mbedtls_ctr_drbg_seed(mbedtls_ctr_drbg_context *ctx, int (*f_entropy)(void *, unsigned char *, size_t), void *p_entropy, const unsigned char *custom,
						   size_t len);
int _mbedtls_ctr_drbg_random_with_add(void *p_rng, unsigned char *output, size_t output_len, const unsigned char *additional, size_t add_len);
int _mbedtls_ctr_drbg_random(void *p_rng, unsigned char *output, size_t output_len);
// hmac_drbg
void _mbedtls_hmac_drbg_init(mbedtls_hmac_drbg_context *ctx);
void _mbedtls_hmac_drbg_update(mbedtls_hmac_drbg_context *ctx, const unsigned char *additional, size_t add_len);
int _mbedtls_hmac_drbg_seed_buf(mbedtls_hmac_drbg_context *ctx, const mbedtls_md_info_t *md_info, const unsigned char *data, size_t data_len);
int _mbedtls_hmac_drbg_reseed(mbedtls_hmac_drbg_context *ctx, const unsigned char *additional, size_t len);
int _mbedtls_hmac_drbg_seed(mbedtls_hmac_drbg_context *ctx, const mbedtls_md_info_t *md_info, int (*f_entropy)(void *, unsigned char *, size_t),
							void *p_entropy, const unsigned char *custom, size_t len);
void _mbedtls_hmac_drbg_set_prediction_resistance(mbedtls_hmac_drbg_context *ctx, int resistance);
void _mbedtls_hmac_drbg_set_entropy_len(mbedtls_hmac_drbg_context *ctx, size_t len);
void _mbedtls_hmac_drbg_set_reseed_interval(mbedtls_hmac_drbg_context *ctx, int interval);
int _mbedtls_hmac_drbg_random_with_add(void *p_rng, unsigned char *output, size_t out_len, const unsigned char *additional, size_t add_len);
int _mbedtls_hmac_drbg_random(void *p_rng, unsigned char *output, size_t out_len);
void _mbedtls_hmac_drbg_free(mbedtls_hmac_drbg_context *ctx);
// pem
void _mbedtls_pem_init(mbedtls_pem_context *ctx);
int _mbedtls_pem_read_buffer(mbedtls_pem_context *ctx, const char *header, const char *footer, const unsigned char *data, const unsigned char *pwd,
							 size_t pwdlen, size_t *use_len);
void _mbedtls_pem_free(mbedtls_pem_context *ctx);
int _mbedtls_pem_write_buffer(const char *header, const char *footer, const unsigned char *der_data, size_t der_len, unsigned char *buf, size_t buf_len,
							  size_t *olen);
// dhm
void _mbedtls_dhm_init(mbedtls_dhm_context *ctx);
int _mbedtls_dhm_read_params(mbedtls_dhm_context *ctx, unsigned char **p, const unsigned char *end);
int _mbedtls_dhm_make_params(mbedtls_dhm_context *ctx, int x_size, unsigned char *output, size_t *olen, int (*f_rng)(void *, unsigned char *, size_t),
							 void *p_rng);
int _mbedtls_dhm_read_public(mbedtls_dhm_context *ctx, const unsigned char *input, size_t ilen);
int _mbedtls_dhm_make_public(mbedtls_dhm_context *ctx, int x_size, unsigned char *output, size_t olen, int (*f_rng)(void *, unsigned char *, size_t),
							 void *p_rng);
int _mbedtls_dhm_calc_secret(mbedtls_dhm_context *ctx, unsigned char *output, size_t output_size, size_t *olen, int (*f_rng)(void *, unsigned char *, size_t),
							 void *p_rng);
void _mbedtls_dhm_free(mbedtls_dhm_context *ctx);
int _mbedtls_dhm_parse_dhm(mbedtls_dhm_context *dhm, const unsigned char *dhmin, size_t dhminlen);
// ecjpake
void _mbedtls_ecjpake_init(mbedtls_ecjpake_context *ctx);
void _mbedtls_ecjpake_free(mbedtls_ecjpake_context *ctx);
int _mbedtls_ecjpake_setup(mbedtls_ecjpake_context *ctx, mbedtls_ecjpake_role role, mbedtls_md_type_t hash, mbedtls_ecp_group_id curve,
						   const unsigned char *secret, size_t len);
int _mbedtls_ecjpake_check(const mbedtls_ecjpake_context *ctx);
int _mbedtls_ecjpake_read_round_one(mbedtls_ecjpake_context *ctx, const unsigned char *buf, size_t len);
int _mbedtls_ecjpake_write_round_one(mbedtls_ecjpake_context *ctx, unsigned char *buf, size_t len, size_t *olen, int (*f_rng)(void *, unsigned char *, size_t),
									 void *p_rng);
int _mbedtls_ecjpake_read_round_two(mbedtls_ecjpake_context *ctx, const unsigned char *buf, size_t len);
int _mbedtls_ecjpake_write_round_two(mbedtls_ecjpake_context *ctx, unsigned char *buf, size_t len, size_t *olen, int (*f_rng)(void *, unsigned char *, size_t),
									 void *p_rng);
int _mbedtls_ecjpake_derive_secret(mbedtls_ecjpake_context *ctx, unsigned char *buf, size_t len, size_t *olen, int (*f_rng)(void *, unsigned char *, size_t),
								   void *p_rng);
// arc4
void _mbedtls_arc4_init(mbedtls_arc4_context *ctx);
void _mbedtls_arc4_free(mbedtls_arc4_context *ctx);
void _mbedtls_arc4_setup(mbedtls_arc4_context *ctx, const unsigned char *key, unsigned int keylen);
int _mbedtls_arc4_crypt(mbedtls_arc4_context *ctx, size_t length, const unsigned char *input, unsigned char *output);
// ecdh
int _mbedtls_ecdh_gen_public(mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecdh_compute_shared(mbedtls_ecp_group *grp, mbedtls_mpi *z, const mbedtls_ecp_point *Q, const mbedtls_mpi *d, int (*f_rng)(void *, unsigned char *,
								 size_t), void *p_rng);
void _mbedtls_ecdh_init(mbedtls_ecdh_context *ctx);
void _mbedtls_ecdh_free(mbedtls_ecdh_context *ctx);
int _mbedtls_ecdh_make_params(mbedtls_ecdh_context *ctx, size_t *olen, unsigned char *buf, size_t blen, int (*f_rng)(void *, unsigned char *, size_t),
							  void *p_rng);
int _mbedtls_ecdh_read_params(mbedtls_ecdh_context *ctx, const unsigned char **buf, const unsigned char *end);
int _mbedtls_ecdh_get_params(mbedtls_ecdh_context *ctx, const mbedtls_ecp_keypair *key, mbedtls_ecdh_side side);
int _mbedtls_ecdh_make_public(mbedtls_ecdh_context *ctx, size_t *olen, unsigned char *buf, size_t blen, int (*f_rng)(void *, unsigned char *, size_t),
							  void *p_rng);
int _mbedtls_ecdh_read_public(mbedtls_ecdh_context *ctx, const unsigned char *buf, size_t blen);
int _mbedtls_ecdh_calc_secret(mbedtls_ecdh_context *ctx, size_t *olen, unsigned char *buf, size_t blen, int (*f_rng)(void *, unsigned char *, size_t),
							  void *p_rng);
// ecdsa
int _mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s, const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
						int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecdsa_sign_det(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s, const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
							mbedtls_md_type_t md_alg);
int _mbedtls_ecdsa_verify(mbedtls_ecp_group *grp, const unsigned char *buf, size_t blen, const mbedtls_ecp_point *Q, const mbedtls_mpi *r,
						  const mbedtls_mpi *s);
int _mbedtls_ecdsa_write_signature(mbedtls_ecdsa_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hlen, unsigned char *sig,
								   size_t *slen, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecdsa_write_signature_det(mbedtls_ecdsa_context *ctx, const unsigned char *hash, size_t hlen, unsigned char *sig, size_t *slen,
									   mbedtls_md_type_t md_alg);
int _mbedtls_ecdsa_read_signature(mbedtls_ecdsa_context *ctx, const unsigned char *hash, size_t hlen, const unsigned char *sig, size_t slen);
int _mbedtls_ecdsa_genkey(mbedtls_ecdsa_context *ctx, mbedtls_ecp_group_id gid, int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_ecdsa_from_keypair(mbedtls_ecdsa_context *ctx, const mbedtls_ecp_keypair *key);
void _mbedtls_ecdsa_init(mbedtls_ecdsa_context *ctx);
void _mbedtls_ecdsa_free(mbedtls_ecdsa_context *ctx);
// pk
void _mbedtls_pk_init(mbedtls_pk_context *ctx);
void _mbedtls_pk_free(mbedtls_pk_context *ctx);
const mbedtls_pk_info_t *_mbedtls_pk_info_from_type(mbedtls_pk_type_t pk_type);
int _mbedtls_pk_setup(mbedtls_pk_context *ctx, const mbedtls_pk_info_t *info);
int _mbedtls_pk_setup_rsa_alt(mbedtls_pk_context *ctx, void *key, mbedtls_pk_rsa_alt_decrypt_func decrypt_func, mbedtls_pk_rsa_alt_sign_func sign_func,
							  mbedtls_pk_rsa_alt_key_len_func key_len_func);
int _mbedtls_pk_can_do(const mbedtls_pk_context *ctx, mbedtls_pk_type_t type);
int _mbedtls_pk_verify(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, const unsigned char *sig, size_t sig_len);
int _mbedtls_pk_verify_ext(mbedtls_pk_type_t type, const void *options, mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash,
						   size_t hash_len, const unsigned char *sig, size_t sig_len);
int _mbedtls_pk_sign(mbedtls_pk_context *ctx, mbedtls_md_type_t md_alg, const unsigned char *hash, size_t hash_len, unsigned char *sig, size_t *sig_len,
					 int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_pk_decrypt(mbedtls_pk_context *ctx, const unsigned char *input, size_t ilen, unsigned char *output, size_t *olen, size_t osize,
						int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_pk_encrypt(mbedtls_pk_context *ctx, const unsigned char *input, size_t ilen, unsigned char *output, size_t *olen, size_t osize,
						int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);
int _mbedtls_pk_check_pair(const mbedtls_pk_context *pub, const mbedtls_pk_context *prv);
size_t _mbedtls_pk_get_bitlen(const mbedtls_pk_context *ctx);
int _mbedtls_pk_debug(const mbedtls_pk_context *ctx, mbedtls_pk_debug_item *items);
const char *_mbedtls_pk_get_name(const mbedtls_pk_context *ctx);
mbedtls_pk_type_t _mbedtls_pk_get_type(const mbedtls_pk_context *ctx);
// pkwrite
int _mbedtls_pk_write_pubkey(unsigned char **p, unsigned char *start, const mbedtls_pk_context *key);
int _mbedtls_pk_write_pubkey_der(mbedtls_pk_context *key, unsigned char *buf, size_t size);
int _mbedtls_pk_write_key_der(mbedtls_pk_context *key, unsigned char *buf, size_t size);
int _mbedtls_pk_write_pubkey_pem(mbedtls_pk_context *key, unsigned char *buf, size_t size);
int _mbedtls_pk_write_key_pem(mbedtls_pk_context *key, unsigned char *buf, size_t size);

#if defined(CONFIG_BUILD_SECURE) && (CONFIG_BUILD_SECURE == 1)
SSL_ROM_STUB_SECTION
const ssl_func_stubs_t ssl_func_stubs = {
	// ssl_ram_map
	.init_rom_ssl_ram_map = _init_rom_ssl_ram_map,
	.init_rom_ssl_hw_crypto_aes_ecb = _init_rom_ssl_hw_crypto_aes_ecb,
	.init_rom_ssl_hw_crypto_aes_cbc = _init_rom_ssl_hw_crypto_aes_cbc,
	.init_rom_ssl_hw_crypto_des_cbc = _init_rom_ssl_hw_crypto_des_cbc,
	.init_rom_ssl_hw_crypto_3des_cbc = _init_rom_ssl_hw_crypto_3des_cbc,
	// bignum
	.mbedtls_mpi_init = _mbedtls_mpi_init,
	.mbedtls_mpi_free = _mbedtls_mpi_free,
	.mbedtls_mpi_grow = _mbedtls_mpi_grow,
	.mbedtls_mpi_shrink = _mbedtls_mpi_shrink,
	.mbedtls_mpi_copy = _mbedtls_mpi_copy,
	.mbedtls_mpi_swap = _mbedtls_mpi_swap,
	.mbedtls_mpi_safe_cond_assign = _mbedtls_mpi_safe_cond_assign,
	.mbedtls_mpi_safe_cond_swap = _mbedtls_mpi_safe_cond_swap,
	.mbedtls_mpi_lset = _mbedtls_mpi_lset,
	.mbedtls_mpi_get_bit = _mbedtls_mpi_get_bit,
	.mbedtls_mpi_set_bit = _mbedtls_mpi_set_bit,
	.mbedtls_mpi_lsb = _mbedtls_mpi_lsb,
	.mbedtls_mpi_bitlen = _mbedtls_mpi_bitlen,
	.mbedtls_mpi_size = _mbedtls_mpi_size,
	.mbedtls_mpi_read_binary = _mbedtls_mpi_read_binary,
	.mbedtls_mpi_write_binary = _mbedtls_mpi_write_binary,
	.mbedtls_mpi_shift_l = _mbedtls_mpi_shift_l,
	.mbedtls_mpi_shift_r = _mbedtls_mpi_shift_r,
	.mbedtls_mpi_cmp_abs = _mbedtls_mpi_cmp_abs,
	.mbedtls_mpi_cmp_mpi = _mbedtls_mpi_cmp_mpi,
	.mbedtls_mpi_cmp_int = _mbedtls_mpi_cmp_int,
	.mbedtls_mpi_add_abs = _mbedtls_mpi_add_abs,
	.mbedtls_mpi_sub_abs = _mbedtls_mpi_sub_abs,
	.mbedtls_mpi_add_mpi = _mbedtls_mpi_add_mpi,
	.mbedtls_mpi_sub_mpi = _mbedtls_mpi_sub_mpi,
	.mbedtls_mpi_add_int = _mbedtls_mpi_add_int,
	.mbedtls_mpi_sub_int = _mbedtls_mpi_sub_int,
	.mbedtls_mpi_mul_mpi = _mbedtls_mpi_mul_mpi,
	.mbedtls_mpi_read_string = _mbedtls_mpi_read_string,
	.mbedtls_mpi_mul_int = _mbedtls_mpi_mul_int,
	.mbedtls_mpi_div_mpi = _mbedtls_mpi_div_mpi,
	.mbedtls_mpi_div_int = _mbedtls_mpi_div_int,
	.mbedtls_mpi_mod_mpi = _mbedtls_mpi_mod_mpi,
	.mbedtls_mpi_mod_int = _mbedtls_mpi_mod_int,
	.mbedtls_mpi_write_string = _mbedtls_mpi_write_string,
	.mbedtls_mpi_exp_mod = _mbedtls_mpi_exp_mod,
	.mbedtls_mpi_gcd = _mbedtls_mpi_gcd,
	.mbedtls_mpi_fill_random = _mbedtls_mpi_fill_random,
	.mbedtls_mpi_inv_mod = _mbedtls_mpi_inv_mod,
	.mbedtls_mpi_is_prime = _mbedtls_mpi_is_prime,
	.mbedtls_mpi_gen_prime = _mbedtls_mpi_gen_prime,
	// ecp
	.mbedtls_ecp_curve_list = _mbedtls_ecp_curve_list,
	.mbedtls_ecp_curve_info_from_grp_id = _mbedtls_ecp_curve_info_from_grp_id,
	.mbedtls_ecp_curve_info_from_tls_id = _mbedtls_ecp_curve_info_from_tls_id,
	.mbedtls_ecp_curve_info_from_name = _mbedtls_ecp_curve_info_from_name,
	.mbedtls_ecp_point_init = _mbedtls_ecp_point_init,
	.mbedtls_ecp_group_init = _mbedtls_ecp_group_init,
	.mbedtls_ecp_keypair_init = _mbedtls_ecp_keypair_init,
	.mbedtls_ecp_point_free = _mbedtls_ecp_point_free,
	.mbedtls_ecp_group_free = _mbedtls_ecp_group_free,
	.mbedtls_ecp_keypair_free = _mbedtls_ecp_keypair_free,
	.mbedtls_ecp_copy = _mbedtls_ecp_copy,
	.mbedtls_ecp_group_copy = _mbedtls_ecp_group_copy,
	.mbedtls_ecp_set_zero = _mbedtls_ecp_set_zero,
	.mbedtls_ecp_is_zero = _mbedtls_ecp_is_zero,
	.mbedtls_ecp_point_cmp = _mbedtls_ecp_point_cmp,
	.mbedtls_ecp_point_read_string = _mbedtls_ecp_point_read_string,
	.mbedtls_ecp_point_write_binary = _mbedtls_ecp_point_write_binary,
	.mbedtls_ecp_point_read_binary = _mbedtls_ecp_point_read_binary,
	.mbedtls_ecp_tls_read_point = _mbedtls_ecp_tls_read_point,
	.mbedtls_ecp_tls_write_point = _mbedtls_ecp_tls_write_point,
	.mbedtls_ecp_tls_read_group = _mbedtls_ecp_tls_read_group,
	.mbedtls_ecp_tls_write_group = _mbedtls_ecp_tls_write_group,
	.mbedtls_ecp_check_pubkey = _mbedtls_ecp_check_pubkey,
	.mbedtls_ecp_check_privkey = _mbedtls_ecp_check_privkey,
	.mbedtls_ecp_mul = _mbedtls_ecp_mul,
	.mbedtls_ecp_muladd = _mbedtls_ecp_muladd,
	.mbedtls_ecp_gen_keypair_base = _mbedtls_ecp_gen_keypair_base,
	.mbedtls_ecp_gen_keypair = _mbedtls_ecp_gen_keypair,
	.mbedtls_ecp_gen_key = _mbedtls_ecp_gen_key,
	.mbedtls_ecp_check_pub_priv = _mbedtls_ecp_check_pub_priv,
	// ecp_curves
	.mbedtls_ecp_group_load = _mbedtls_ecp_group_load
};
#else
SSL_ROM_STUB_SECTION
const ssl_func_stubs_t ssl_func_stubs = {
	// ssl_ram_map
	.init_rom_ssl_ram_map = _init_rom_ssl_ram_map,
	.init_rom_ssl_hw_crypto_aes_ecb = _init_rom_ssl_hw_crypto_aes_ecb,
	.init_rom_ssl_hw_crypto_aes_cbc = _init_rom_ssl_hw_crypto_aes_cbc,
	.init_rom_ssl_hw_crypto_des_cbc = _init_rom_ssl_hw_crypto_des_cbc,
	.init_rom_ssl_hw_crypto_3des_cbc = _init_rom_ssl_hw_crypto_3des_cbc,
	// bignum
	.mbedtls_mpi_init = _mbedtls_mpi_init,
	.mbedtls_mpi_free = _mbedtls_mpi_free,
	.mbedtls_mpi_grow = _mbedtls_mpi_grow,
	.mbedtls_mpi_shrink = _mbedtls_mpi_shrink,
	.mbedtls_mpi_copy = _mbedtls_mpi_copy,
	.mbedtls_mpi_swap = _mbedtls_mpi_swap,
	.mbedtls_mpi_safe_cond_assign = _mbedtls_mpi_safe_cond_assign,
	.mbedtls_mpi_safe_cond_swap = _mbedtls_mpi_safe_cond_swap,
	.mbedtls_mpi_lset = _mbedtls_mpi_lset,
	.mbedtls_mpi_get_bit = _mbedtls_mpi_get_bit,
	.mbedtls_mpi_set_bit = _mbedtls_mpi_set_bit,
	.mbedtls_mpi_lsb = _mbedtls_mpi_lsb,
	.mbedtls_mpi_bitlen = _mbedtls_mpi_bitlen,
	.mbedtls_mpi_size = _mbedtls_mpi_size,
	.mbedtls_mpi_read_binary = _mbedtls_mpi_read_binary,
	.mbedtls_mpi_write_binary = _mbedtls_mpi_write_binary,
	.mbedtls_mpi_shift_l = _mbedtls_mpi_shift_l,
	.mbedtls_mpi_shift_r = _mbedtls_mpi_shift_r,
	.mbedtls_mpi_cmp_abs = _mbedtls_mpi_cmp_abs,
	.mbedtls_mpi_cmp_mpi = _mbedtls_mpi_cmp_mpi,
	.mbedtls_mpi_cmp_int = _mbedtls_mpi_cmp_int,
	.mbedtls_mpi_add_abs = _mbedtls_mpi_add_abs,
	.mbedtls_mpi_sub_abs = _mbedtls_mpi_sub_abs,
	.mbedtls_mpi_add_mpi = _mbedtls_mpi_add_mpi,
	.mbedtls_mpi_sub_mpi = _mbedtls_mpi_sub_mpi,
	.mbedtls_mpi_add_int = _mbedtls_mpi_add_int,
	.mbedtls_mpi_sub_int = _mbedtls_mpi_sub_int,
	.mbedtls_mpi_mul_mpi = _mbedtls_mpi_mul_mpi,
	.mbedtls_mpi_read_string = _mbedtls_mpi_read_string,
	.mbedtls_mpi_mul_int = _mbedtls_mpi_mul_int,
	.mbedtls_mpi_div_mpi = _mbedtls_mpi_div_mpi,
	.mbedtls_mpi_div_int = _mbedtls_mpi_div_int,
	.mbedtls_mpi_mod_mpi = _mbedtls_mpi_mod_mpi,
	.mbedtls_mpi_mod_int = _mbedtls_mpi_mod_int,
	.mbedtls_mpi_write_string = _mbedtls_mpi_write_string,
	.mbedtls_mpi_exp_mod = _mbedtls_mpi_exp_mod,
	.mbedtls_mpi_gcd = _mbedtls_mpi_gcd,
	.mbedtls_mpi_fill_random = _mbedtls_mpi_fill_random,
	.mbedtls_mpi_inv_mod = _mbedtls_mpi_inv_mod,
	.mbedtls_mpi_is_prime = _mbedtls_mpi_is_prime,
	.mbedtls_mpi_gen_prime = _mbedtls_mpi_gen_prime,
	// ecp
	.mbedtls_ecp_curve_list = _mbedtls_ecp_curve_list,
	.mbedtls_ecp_curve_info_from_grp_id = _mbedtls_ecp_curve_info_from_grp_id,
	.mbedtls_ecp_curve_info_from_tls_id = _mbedtls_ecp_curve_info_from_tls_id,
	.mbedtls_ecp_curve_info_from_name = _mbedtls_ecp_curve_info_from_name,
	.mbedtls_ecp_point_init = _mbedtls_ecp_point_init,
	.mbedtls_ecp_group_init = _mbedtls_ecp_group_init,
	.mbedtls_ecp_keypair_init = _mbedtls_ecp_keypair_init,
	.mbedtls_ecp_point_free = _mbedtls_ecp_point_free,
	.mbedtls_ecp_group_free = _mbedtls_ecp_group_free,
	.mbedtls_ecp_keypair_free = _mbedtls_ecp_keypair_free,
	.mbedtls_ecp_copy = _mbedtls_ecp_copy,
	.mbedtls_ecp_group_copy = _mbedtls_ecp_group_copy,
	.mbedtls_ecp_set_zero = _mbedtls_ecp_set_zero,
	.mbedtls_ecp_is_zero = _mbedtls_ecp_is_zero,
	.mbedtls_ecp_point_cmp = _mbedtls_ecp_point_cmp,
	.mbedtls_ecp_point_read_string = _mbedtls_ecp_point_read_string,
	.mbedtls_ecp_point_write_binary = _mbedtls_ecp_point_write_binary,
	.mbedtls_ecp_point_read_binary = _mbedtls_ecp_point_read_binary,
	.mbedtls_ecp_tls_read_point = _mbedtls_ecp_tls_read_point,
	.mbedtls_ecp_tls_write_point = _mbedtls_ecp_tls_write_point,
	.mbedtls_ecp_tls_read_group = _mbedtls_ecp_tls_read_group,
	.mbedtls_ecp_tls_write_group = _mbedtls_ecp_tls_write_group,
	.mbedtls_ecp_check_pubkey = _mbedtls_ecp_check_pubkey,
	.mbedtls_ecp_check_privkey = _mbedtls_ecp_check_privkey,
	.mbedtls_ecp_mul = _mbedtls_ecp_mul,
	.mbedtls_ecp_muladd = _mbedtls_ecp_muladd,
	.mbedtls_ecp_gen_keypair_base = _mbedtls_ecp_gen_keypair_base,
	.mbedtls_ecp_gen_keypair = _mbedtls_ecp_gen_keypair,
	.mbedtls_ecp_gen_key = _mbedtls_ecp_gen_key,
	.mbedtls_ecp_check_pub_priv = _mbedtls_ecp_check_pub_priv,
	// ecp_curves
	.mbedtls_ecp_group_load = _mbedtls_ecp_group_load,
	// sha1
	.mbedtls_sha1_init = _mbedtls_sha1_init,
	.mbedtls_sha1_free = _mbedtls_sha1_free,
	.mbedtls_sha1_clone = _mbedtls_sha1_clone,
	.mbedtls_sha1_starts = _mbedtls_sha1_starts,
	.mbedtls_sha1_process = _mbedtls_sha1_process,
	.mbedtls_sha1_update = _mbedtls_sha1_update,
	.mbedtls_sha1_finish = _mbedtls_sha1_finish,
	.mbedtls_sha1 = _mbedtls_sha1,
	// sha256
	.mbedtls_sha256_init = _mbedtls_sha256_init,
	.mbedtls_sha256_free = _mbedtls_sha256_free,
	.mbedtls_sha256_clone = _mbedtls_sha256_clone,
	.mbedtls_sha256_starts = _mbedtls_sha256_starts,
	.mbedtls_sha256_process = _mbedtls_sha256_process,
	.mbedtls_sha256_update = _mbedtls_sha256_update,
	.mbedtls_sha256_finish = _mbedtls_sha256_finish,
	.mbedtls_sha256 = _mbedtls_sha256,
	// sha512
	.mbedtls_sha512_init = _mbedtls_sha512_init,
	.mbedtls_sha512_free = _mbedtls_sha512_free,
	.mbedtls_sha512_clone = _mbedtls_sha512_clone,
	.mbedtls_sha512_starts = _mbedtls_sha512_starts,
	.mbedtls_sha512_process = _mbedtls_sha512_process,
	.mbedtls_sha512_update = _mbedtls_sha512_update,
	.mbedtls_sha512_finish = _mbedtls_sha512_finish,
	.mbedtls_sha512 = _mbedtls_sha512,
	// oid
	.mbedtls_oid_get_attr_short_name = _mbedtls_oid_get_attr_short_name,
	.mbedtls_oid_get_x509_ext_type = _mbedtls_oid_get_x509_ext_type,
	.mbedtls_oid_get_extended_key_usage = _mbedtls_oid_get_extended_key_usage,
	.mbedtls_oid_get_sig_alg_desc = _mbedtls_oid_get_sig_alg_desc,
	.mbedtls_oid_get_sig_alg = _mbedtls_oid_get_sig_alg,
	.mbedtls_oid_get_oid_by_sig_alg = _mbedtls_oid_get_oid_by_sig_alg,
	.mbedtls_oid_get_pk_alg = _mbedtls_oid_get_pk_alg,
	.mbedtls_oid_get_oid_by_pk_alg = _mbedtls_oid_get_oid_by_pk_alg,
	.mbedtls_oid_get_ec_grp = _mbedtls_oid_get_ec_grp,
	.mbedtls_oid_get_oid_by_ec_grp = _mbedtls_oid_get_oid_by_ec_grp,
	.mbedtls_oid_get_cipher_alg = _mbedtls_oid_get_cipher_alg,
	.mbedtls_oid_get_md_alg = _mbedtls_oid_get_md_alg,
	.mbedtls_oid_get_oid_by_md = _mbedtls_oid_get_oid_by_md,
	.mbedtls_oid_get_pkcs12_pbe_alg = _mbedtls_oid_get_pkcs12_pbe_alg,
	// md5
	.mbedtls_md5_init = _mbedtls_md5_init,
	.mbedtls_md5_free = _mbedtls_md5_free,
	.mbedtls_md5_clone = _mbedtls_md5_clone,
	.mbedtls_md5_starts = _mbedtls_md5_starts,
	.mbedtls_md5_process = _mbedtls_md5_process,
	.mbedtls_md5_update = _mbedtls_md5_update,
	.mbedtls_md5_finish = _mbedtls_md5_finish,
	.mbedtls_md5 = _mbedtls_md5,
	// aes
	.mbedtls_aes_init = _mbedtls_aes_init,
	.mbedtls_aes_free = _mbedtls_aes_free,
	.mbedtls_aes_setkey_enc = _mbedtls_aes_setkey_enc,
	.mbedtls_aes_setkey_dec = _mbedtls_aes_setkey_dec,
	.mbedtls_aes_encrypt = _mbedtls_aes_encrypt,
	.mbedtls_aes_decrypt = _mbedtls_aes_decrypt,
	.mbedtls_aes_crypt_ecb = _mbedtls_aes_crypt_ecb,
	.mbedtls_aes_crypt_cbc = _mbedtls_aes_crypt_cbc,
	.mbedtls_aes_crypt_cfb128 = _mbedtls_aes_crypt_cfb128,
	.mbedtls_aes_crypt_cfb8 = _mbedtls_aes_crypt_cfb8,
	.mbedtls_aes_crypt_ctr = _mbedtls_aes_crypt_ctr,
	// des
	.mbedtls_des_init = _mbedtls_des_init,
	.mbedtls_des_free = _mbedtls_des_free,
	.mbedtls_des3_init = _mbedtls_des3_init,
	.mbedtls_des3_free = _mbedtls_des3_free,
	.mbedtls_des_key_set_parity = _mbedtls_des_key_set_parity,
	.mbedtls_des_key_check_key_parity = _mbedtls_des_key_check_key_parity,
	.mbedtls_des_key_check_weak = _mbedtls_des_key_check_weak,
	.mbedtls_des_setkey = _mbedtls_des_setkey,
	.mbedtls_des_setkey_enc = _mbedtls_des_setkey_enc,
	.mbedtls_des_setkey_dec = _mbedtls_des_setkey_dec,
	.mbedtls_des3_set2key_enc = _mbedtls_des3_set2key_enc,
	.mbedtls_des3_set2key_dec = _mbedtls_des3_set2key_dec,
	.mbedtls_des3_set3key_enc = _mbedtls_des3_set3key_enc,
	.mbedtls_des3_set3key_dec = _mbedtls_des3_set3key_dec,
	.mbedtls_des_crypt_ecb = _mbedtls_des_crypt_ecb,
	.mbedtls_des_crypt_cbc = _mbedtls_des_crypt_cbc,
	.mbedtls_des3_crypt_ecb = _mbedtls_des3_crypt_ecb,
	.mbedtls_des3_crypt_cbc = _mbedtls_des3_crypt_cbc,
	// md
	.mbedtls_md_list = _mbedtls_md_list,
	.mbedtls_md_info_from_string = _mbedtls_md_info_from_string,
	.mbedtls_md_info_from_type = _mbedtls_md_info_from_type,
	.mbedtls_md_init = _mbedtls_md_init,
	.mbedtls_md_free = _mbedtls_md_free,
	.mbedtls_md_clone = _mbedtls_md_clone,
	.mbedtls_md_setup = _mbedtls_md_setup,
	.mbedtls_md_init_ctx = _mbedtls_md_init_ctx,
	.mbedtls_md_starts = _mbedtls_md_starts,
	.mbedtls_md_update = _mbedtls_md_update,
	.mbedtls_md_finish = _mbedtls_md_finish,
	.mbedtls_md = _mbedtls_md,
	.mbedtls_md_hmac_starts = _mbedtls_md_hmac_starts,
	.mbedtls_md_hmac_update = _mbedtls_md_hmac_update,
	.mbedtls_md_hmac_finish = _mbedtls_md_hmac_finish,
	.mbedtls_md_hmac_reset = _mbedtls_md_hmac_reset,
	.mbedtls_md_hmac = _mbedtls_md_hmac,
	.mbedtls_md_process = _mbedtls_md_process,
	.mbedtls_md_get_size = _mbedtls_md_get_size,
	.mbedtls_md_get_type = _mbedtls_md_get_type,
	.mbedtls_md_get_name = _mbedtls_md_get_name,
	// asn1parse
	.mbedtls_asn1_get_len = _mbedtls_asn1_get_len,
	.mbedtls_asn1_get_tag = _mbedtls_asn1_get_tag,
	.mbedtls_asn1_get_bool = _mbedtls_asn1_get_bool,
	.mbedtls_asn1_get_int = _mbedtls_asn1_get_int,
	.mbedtls_asn1_get_mpi = _mbedtls_asn1_get_mpi,
	.mbedtls_asn1_get_bitstring = _mbedtls_asn1_get_bitstring,
	.mbedtls_asn1_get_bitstring_null = _mbedtls_asn1_get_bitstring_null,
	.mbedtls_asn1_get_sequence_of = _mbedtls_asn1_get_sequence_of,
	.mbedtls_asn1_get_alg = _mbedtls_asn1_get_alg,
	.mbedtls_asn1_get_alg_null = _mbedtls_asn1_get_alg_null,
	.mbedtls_asn1_free_named_data = _mbedtls_asn1_free_named_data,
	.mbedtls_asn1_free_named_data_list = _mbedtls_asn1_free_named_data_list,
	.mbedtls_asn1_find_named_data = _mbedtls_asn1_find_named_data,
	// asn1write
	.mbedtls_asn1_write_len = _mbedtls_asn1_write_len,
	.mbedtls_asn1_write_tag = _mbedtls_asn1_write_tag,
	.mbedtls_asn1_write_raw_buffer = _mbedtls_asn1_write_raw_buffer,
	.mbedtls_asn1_write_mpi = _mbedtls_asn1_write_mpi,
	.mbedtls_asn1_write_null = _mbedtls_asn1_write_null,
	.mbedtls_asn1_write_oid = _mbedtls_asn1_write_oid,
	.mbedtls_asn1_write_algorithm_identifier = _mbedtls_asn1_write_algorithm_identifier,
	.mbedtls_asn1_write_bool = _mbedtls_asn1_write_bool,
	.mbedtls_asn1_write_int = _mbedtls_asn1_write_int,
	.mbedtls_asn1_write_printable_string = _mbedtls_asn1_write_printable_string,
	.mbedtls_asn1_write_ia5_string = _mbedtls_asn1_write_ia5_string,
	.mbedtls_asn1_write_bitstring = _mbedtls_asn1_write_bitstring,
	.mbedtls_asn1_write_octet_string = _mbedtls_asn1_write_octet_string,
	.mbedtls_asn1_store_named_data = _mbedtls_asn1_store_named_data,
	// base64
	.mbedtls_base64_encode = _mbedtls_base64_encode,
	.mbedtls_base64_decode = _mbedtls_base64_decode,
	// rsa
	.mbedtls_rsa_init = _mbedtls_rsa_init,
	.mbedtls_rsa_set_padding = _mbedtls_rsa_set_padding,
	.mbedtls_rsa_check_pubkey = _mbedtls_rsa_check_pubkey,
	.mbedtls_rsa_check_privkey = _mbedtls_rsa_check_privkey,
	.mbedtls_rsa_check_pub_priv = _mbedtls_rsa_check_pub_priv,
	.mbedtls_rsa_public = _mbedtls_rsa_public,
	.mbedtls_rsa_private = _mbedtls_rsa_private,
	.mbedtls_rsa_rsaes_oaep_encrypt = _mbedtls_rsa_rsaes_oaep_encrypt,
	.mbedtls_rsa_rsaes_pkcs1_v15_encrypt = _mbedtls_rsa_rsaes_pkcs1_v15_encrypt,
	.mbedtls_rsa_pkcs1_encrypt = _mbedtls_rsa_pkcs1_encrypt,
	.mbedtls_rsa_rsaes_oaep_decrypt = _mbedtls_rsa_rsaes_oaep_decrypt,
	.mbedtls_rsa_rsaes_pkcs1_v15_decrypt = _mbedtls_rsa_rsaes_pkcs1_v15_decrypt,
	.mbedtls_rsa_pkcs1_decrypt = _mbedtls_rsa_pkcs1_decrypt,
	.mbedtls_rsa_rsassa_pss_sign = _mbedtls_rsa_rsassa_pss_sign,
	.mbedtls_rsa_rsassa_pkcs1_v15_sign = _mbedtls_rsa_rsassa_pkcs1_v15_sign,
	.mbedtls_rsa_pkcs1_sign = _mbedtls_rsa_pkcs1_sign,
	.mbedtls_rsa_rsassa_pss_verify_ext = _mbedtls_rsa_rsassa_pss_verify_ext,
	.mbedtls_rsa_rsassa_pss_verify = _mbedtls_rsa_rsassa_pss_verify,
	.mbedtls_rsa_rsassa_pkcs1_v15_verify = _mbedtls_rsa_rsassa_pkcs1_v15_verify,
	.mbedtls_rsa_pkcs1_verify = _mbedtls_rsa_pkcs1_verify,
	.mbedtls_rsa_free = _mbedtls_rsa_free,
	.mbedtls_rsa_gen_key = _mbedtls_rsa_gen_key,
	.mbedtls_rsa_copy = _mbedtls_rsa_copy,
	// ctr_drbg
	.mbedtls_ctr_drbg_init = _mbedtls_ctr_drbg_init,
	.mbedtls_ctr_drbg_free = _mbedtls_ctr_drbg_free,
	.mbedtls_ctr_drbg_set_prediction_resistance = _mbedtls_ctr_drbg_set_prediction_resistance,
	.mbedtls_ctr_drbg_set_entropy_len = _mbedtls_ctr_drbg_set_entropy_len,
	.mbedtls_ctr_drbg_set_reseed_interval = _mbedtls_ctr_drbg_set_reseed_interval,
	.mbedtls_ctr_drbg_update = _mbedtls_ctr_drbg_update,
	.mbedtls_ctr_drbg_reseed = _mbedtls_ctr_drbg_reseed,
	.mbedtls_ctr_drbg_seed_entropy_len = _mbedtls_ctr_drbg_seed_entropy_len,
	.mbedtls_ctr_drbg_seed = _mbedtls_ctr_drbg_seed,
	.mbedtls_ctr_drbg_random_with_add = _mbedtls_ctr_drbg_random_with_add,
	.mbedtls_ctr_drbg_random = _mbedtls_ctr_drbg_random,
	// hmac_drbg
	.mbedtls_hmac_drbg_init = _mbedtls_hmac_drbg_init,
	.mbedtls_hmac_drbg_update = _mbedtls_hmac_drbg_update,
	.mbedtls_hmac_drbg_seed_buf = _mbedtls_hmac_drbg_seed_buf,
	.mbedtls_hmac_drbg_reseed = _mbedtls_hmac_drbg_reseed,
	.mbedtls_hmac_drbg_seed = _mbedtls_hmac_drbg_seed,
	.mbedtls_hmac_drbg_set_prediction_resistance = _mbedtls_hmac_drbg_set_prediction_resistance,
	.mbedtls_hmac_drbg_set_entropy_len = _mbedtls_hmac_drbg_set_entropy_len,
	.mbedtls_hmac_drbg_set_reseed_interval = _mbedtls_hmac_drbg_set_reseed_interval,
	.mbedtls_hmac_drbg_random_with_add = _mbedtls_hmac_drbg_random_with_add,
	.mbedtls_hmac_drbg_random = _mbedtls_hmac_drbg_random,
	.mbedtls_hmac_drbg_free = _mbedtls_hmac_drbg_free,
	// pem
	.mbedtls_pem_init = _mbedtls_pem_init,
	.mbedtls_pem_read_buffer = _mbedtls_pem_read_buffer,
	.mbedtls_pem_free = _mbedtls_pem_free,
	.mbedtls_pem_write_buffer = _mbedtls_pem_write_buffer,
	// dhm
	.mbedtls_dhm_init = _mbedtls_dhm_init,
	.mbedtls_dhm_read_params = _mbedtls_dhm_read_params,
	.mbedtls_dhm_make_params = _mbedtls_dhm_make_params,
	.mbedtls_dhm_read_public = _mbedtls_dhm_read_public,
	.mbedtls_dhm_make_public = _mbedtls_dhm_make_public,
	.mbedtls_dhm_calc_secret = _mbedtls_dhm_calc_secret,
	.mbedtls_dhm_free = _mbedtls_dhm_free,
	.mbedtls_dhm_parse_dhm = _mbedtls_dhm_parse_dhm,
	// ecjpake
	.mbedtls_ecjpake_init = _mbedtls_ecjpake_init,
	.mbedtls_ecjpake_free = _mbedtls_ecjpake_free,
	.mbedtls_ecjpake_setup = _mbedtls_ecjpake_setup,
	.mbedtls_ecjpake_check = _mbedtls_ecjpake_check,
	.mbedtls_ecjpake_read_round_one = _mbedtls_ecjpake_read_round_one,
	.mbedtls_ecjpake_write_round_one = _mbedtls_ecjpake_write_round_one,
	.mbedtls_ecjpake_read_round_two = _mbedtls_ecjpake_read_round_two,
	.mbedtls_ecjpake_write_round_two = _mbedtls_ecjpake_write_round_two,
	.mbedtls_ecjpake_derive_secret = _mbedtls_ecjpake_derive_secret,
	// arc4
	.mbedtls_arc4_init = _mbedtls_arc4_init,
	.mbedtls_arc4_free = _mbedtls_arc4_free,
	.mbedtls_arc4_setup = _mbedtls_arc4_setup,
	.mbedtls_arc4_crypt = _mbedtls_arc4_crypt,
	//ecdh
	.mbedtls_ecdh_gen_public = _mbedtls_ecdh_gen_public,
	.mbedtls_ecdh_compute_shared = _mbedtls_ecdh_compute_shared,
	.mbedtls_ecdh_init = _mbedtls_ecdh_init,
	.mbedtls_ecdh_free = _mbedtls_ecdh_free,
	.mbedtls_ecdh_make_params = _mbedtls_ecdh_make_params,
	.mbedtls_ecdh_read_params = _mbedtls_ecdh_read_params,
	.mbedtls_ecdh_get_params = _mbedtls_ecdh_get_params,
	.mbedtls_ecdh_make_public = _mbedtls_ecdh_make_public,
	.mbedtls_ecdh_read_public = _mbedtls_ecdh_read_public,
	.mbedtls_ecdh_calc_secret = _mbedtls_ecdh_calc_secret,
	// ecdsa
	.mbedtls_ecdsa_sign = _mbedtls_ecdsa_sign,
	.mbedtls_ecdsa_sign_det = _mbedtls_ecdsa_sign_det,
	.mbedtls_ecdsa_verify = _mbedtls_ecdsa_verify,
	.mbedtls_ecdsa_write_signature = _mbedtls_ecdsa_write_signature,
	.mbedtls_ecdsa_write_signature_det = _mbedtls_ecdsa_write_signature_det,
	.mbedtls_ecdsa_read_signature = _mbedtls_ecdsa_read_signature,
	.mbedtls_ecdsa_genkey = _mbedtls_ecdsa_genkey,
	.mbedtls_ecdsa_from_keypair = _mbedtls_ecdsa_from_keypair,
	.mbedtls_ecdsa_init = _mbedtls_ecdsa_init,
	.mbedtls_ecdsa_free = _mbedtls_ecdsa_free,
	// pk
	.mbedtls_pk_init = _mbedtls_pk_init,
	.mbedtls_pk_free = _mbedtls_pk_free,
	.mbedtls_pk_info_from_type = _mbedtls_pk_info_from_type,
	.mbedtls_pk_setup = _mbedtls_pk_setup,
	.mbedtls_pk_setup_rsa_alt = _mbedtls_pk_setup_rsa_alt,
	.mbedtls_pk_can_do = _mbedtls_pk_can_do,
	.mbedtls_pk_verify = _mbedtls_pk_verify,
	.mbedtls_pk_verify_ext = _mbedtls_pk_verify_ext,
	.mbedtls_pk_sign = _mbedtls_pk_sign,
	.mbedtls_pk_decrypt = _mbedtls_pk_decrypt,
	.mbedtls_pk_encrypt = _mbedtls_pk_encrypt,
	.mbedtls_pk_check_pair = _mbedtls_pk_check_pair,
	.mbedtls_pk_get_bitlen = _mbedtls_pk_get_bitlen,
	.mbedtls_pk_debug = _mbedtls_pk_debug,
	.mbedtls_pk_get_name = _mbedtls_pk_get_name,
	.mbedtls_pk_get_type = _mbedtls_pk_get_type,
	// pkwrite
	.mbedtls_pk_write_pubkey = _mbedtls_pk_write_pubkey,
	.mbedtls_pk_write_pubkey_der = _mbedtls_pk_write_pubkey_der,
	.mbedtls_pk_write_key_der = _mbedtls_pk_write_key_der,
	.mbedtls_pk_write_pubkey_pem = _mbedtls_pk_write_pubkey_pem,
	.mbedtls_pk_write_key_pem = _mbedtls_pk_write_key_pem
};
#endif
