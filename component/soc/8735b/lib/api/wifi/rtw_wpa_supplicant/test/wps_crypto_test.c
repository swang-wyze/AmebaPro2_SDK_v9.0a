#include <stddef.h>
#include <stdint.h>
#include <platform/platform_stdlib.h>

typedef uint8_t u8;

extern int crypto_mod_exp(const u8 *base, size_t base_len,
						  const u8 *power, size_t power_len,
						  const u8 *modulus, size_t modulus_len,
						  u8 *result, size_t *result_len);
extern int rom_hmac_sha256_vector(const u8 *key, size_t key_len, size_t num_elem,
								  const u8 *addr[], const size_t *len, u8 *mac);
extern int rom_aes_128_cbc_encrypt(const u8 *key, const u8 *iv, u8 *data, size_t data_len);
extern int rom_aes_128_cbc_decrypt(const u8 *key, const u8 *iv, u8 *data, size_t data_len);

static HexToData(const char *hex, uint8_t *data, int data_len)
{
	if (strlen(hex) == (data_len * 2)) {
		int i;
		for (i = 0; i < data_len; i ++) {
			char hex1 = hex[i * 2];
			char hex2 = hex[i * 2 + 1];
			data[i] = 0;

			if ((hex1 >= '0') && (hex1 <= '9')) {
				data[i] += ((hex1 - '0') * 16);
			} else if ((hex1 >= 'a') && (hex1 <= 'f')) {
				data[i] += ((hex1 - 'a' + 10) * 16);
			} else if ((hex1 >= 'A') && (hex1 <= 'F')) {
				data[i] += ((hex1 - 'A' + 10) * 16);
			}

			if ((hex2 >= '0') && (hex2 <= '9')) {
				data[i] += (hex2 - '0');
			} else if ((hex2 >= 'a') && (hex2 <= 'f')) {
				data[i] += (hex2 - 'a' + 10);
			} else if ((hex2 >= 'A') && (hex2 <= 'F')) {
				data[i] += (hex2 - 'A' + 10);
			}
		}
	} else {
		printf("%s Size Err\n", __FUNCTION__);
	}
}

typedef struct {
	const char *base;
	const char *power;
	const char *modulus;
	const char *result;
} modexp_test_vector;

static const modexp_test_vector ModExp_TestVectors[] = {
	{
		"02",
		"17fc2fc7215b6ea87c9d377ed3fcc36c26b0cf9def82e9da0de893ed9697fd2555e2443bd9faeb659e561e4a91f7d84ff515f17dcb3ef09567db42ce4bf898fa787ca0b5a9347375f60329adc7d2eaca86c7adc5380f7f2ff0c8ee7c5da1a4f5194c1f3b0bfaf7b1c549fc2e89e2f1aae3ac5965198086c31f77615748b440eba126656e35903f8389f3e2f357a63b2219fa1fc6f4a3b15774c3c3d1f60ff1c15b0802fc14a4cd2bb0525e257e84d1f62db5bdc7078b3d29692af91865d9d35e",
		"ffffffffffffffffc90fdaa22168c234c4c6628b80dc1cd129024e088a67cc74020bbea63b139b22514a08798e3404ddef9519b3cd3a431b302b0a6df25f14374fe1356d6d51c245e485b576625e7ec6f44c42e9a637ed6b0bff5cb6f406b7edee386bfb5a899fa5ae9f24117c4b1fe649286651ece45b3dc2007cb8a163bf0598da48361c55d39a69163fa8fd24cf5f83655d23dca3ad961c62f356208552bb9ed529077096966d670c354e4abc9804f1746c08ca237327ffffffffffffffff",
		"9041f9b90c6100b7c51b27893ace9da67b0811267beae9283c3890472a875acf14b8bc7c7c175e2d2b9547341a81a7ad3722884770aa3050273ba1aa0d914394b647c4bb85ffecb562e385c8733937cbc41b42f9c9c1a99ce5b0da8b256add40df5f483aee2bcf48ff55de7cdf50d37bed2bcc58ac0787e498e0f1f7a3af7067667dd6bbc08e19b860b32cb6b0c56af95fd31b233033bcc027dc7eac93ae8c5c0b0bca8ff879e4aafee59ec9fea721f2be35859b71e4acb9fd1855ee35838fe4"
	},
	{
		"d0141b15656e96b85fcead2e8e76330d2b1ac1576bb026e7a328c0e1baf8cf91664371174c08ee12ec92b0519c54879f21255be5a8770e1fa1880470ef423c90e34d7847a6fcb4924563d1af1db0c481ead9852c519bf1dd429c163951cf69181b132aea2a3684caf35bc54aca1b20c88bb3b7339ff7d56e09139d77f0ac58079097938251dbbe75e86715cc6b7c0ca945fa8dd8d661beb73b414032798dadee32b5dd61bf105f18d89217760b75c5d966a5a490472ceba9e3b4224f3d89fb2b",
		"17fc2fc7215b6ea87c9d377ed3fcc36c26b0cf9def82e9da0de893ed9697fd2555e2443bd9faeb659e561e4a91f7d84ff515f17dcb3ef09567db42ce4bf898fa787ca0b5a9347375f60329adc7d2eaca86c7adc5380f7f2ff0c8ee7c5da1a4f5194c1f3b0bfaf7b1c549fc2e89e2f1aae3ac5965198086c31f77615748b440eba126656e35903f8389f3e2f357a63b2219fa1fc6f4a3b15774c3c3d1f60ff1c15b0802fc14a4cd2bb0525e257e84d1f62db5bdc7078b3d29692af91865d9d35e",
		"ffffffffffffffffc90fdaa22168c234c4c6628b80dc1cd129024e088a67cc74020bbea63b139b22514a08798e3404ddef9519b3cd3a431b302b0a6df25f14374fe1356d6d51c245e485b576625e7ec6f44c42e9a637ed6b0bff5cb6f406b7edee386bfb5a899fa5ae9f24117c4b1fe649286651ece45b3dc2007cb8a163bf0598da48361c55d39a69163fa8fd24cf5f83655d23dca3ad961c62f356208552bb9ed529077096966d670c354e4abc9804f1746c08ca237327ffffffffffffffff",
		"d8bd22fc56a984477fc3c8f72b71c14d5bd83dab7225404f25ca00902314fb9de2f5550634f5c8e0c5e9279f54afbc573073645b29cf5b3b0aa5c92c4a8c0bc3a37d3a0945b3773f596b479d290a008ec01f1852d95d3d47da81bb6b209ca278a971745714515ffd1a326d72e5441456d738b43cc1997d041f95b2bc145b176196649a1bb39c0a4dc916562ba8998b82ae1639d25ab92d1734e998205f822c04f2bf5a0b640e95a27dd914cab0b9c9eb577bbdfe202d9d72da02e1cce8870df8"
	}
};

static int	mod_exp_test(int print)
{
	char base[192], power[192], modulus[192], result[192], out_result[192];
	int i, j, err = 0, out_result_len;

	if (print) {
		printf("\n[%s]\n", __FUNCTION__);
	}

	for (i = 0; i < (sizeof(ModExp_TestVectors) / sizeof(modexp_test_vector)); i ++)	{
		int base_len;
		modexp_test_vector *vector = &ModExp_TestVectors[i];
		memset(base, 0, 192);
		memset(power, 0, 192);
		memset(modulus, 0, 192);
		memset(result, 0, 192);
		memset(out_result, 0, 192);
		base_len = strlen(vector->base) / 2;
		HexToData(vector->base, base, base_len);
		HexToData(vector->power, power, 192);
		HexToData(vector->modulus, modulus, 192);
		HexToData(vector->result, result, 192);

		crypto_mod_exp(base, base_len, power, 192, modulus, 192, out_result, &out_result_len);

		if (print) {
			printf("base:");
			for (j = 0; j < base_len; ++j) {
				printf("%02x", base[j]);
			}
			printf(", power:");
			for (j = 0; j < 192; ++j) {
				printf("%02x", power[j]);
			}
			printf(", modulus:");
			for (j = 0; j < 192; ++j) {
				printf("%02x", modulus[j]);
			}
			printf(", result:");
			for (j = 0; j < 192; ++j) {
				printf("%02x", result[j]);
			}
			printf(", out_result:");
			for (j = 0; j < out_result_len; ++j) {
				printf("%02x", out_result[j]);
			}
			printf("\n");
		}

		if (out_result_len != 192) {
			printf("%s out_result_len Err\n", __FUNCTION__);
			err = 1;
			goto exit;
		}

		if (!(memcmp(result, out_result, 192) == 0)) {
			printf("%s out_result Err\n", __FUNCTION__);
			err = 1;
			goto exit;
		}
	}

exit:
	return (err);
}

typedef struct {
	const char *key;
	int	num;
	const char **data;
	const char *mac;
} sha_test_vector;

char *data1[] = {"30303030"};
char *data2[] = {
	"104a0001101022000109103900104ce89f4045da2d676758abf429ead99d1018004028ca5a85a4ce4f8c901c92bd520444568de19fa5b60759e456f9fae4183252bbf105f42ef0f69c21bb54f2a280f302dd4cbcc7e1fa207e762c62307a56008b1b10050008d8faba3f10c65ae1",
	"104a000110102200010a101a00106f99affa83548d48c7673c1488e555ef101800404647e8483d5fbccb2060d52c0b0b8313fd4adeac954ba3c289afbb5d2c9332aa348b4fe6f9d5000313dfda3ca2acfe86e49e7a1587d9105438d9548a9a6b9e871049000600372a000120"
};

static const sha_test_vector Sha_TestVectors[] = {
	{
		"1c063b9bbc914902bdeb276bed4b280367ec595fc08ba7a6e3ee2982e1a472f7",
		1,
		data1,
		"6ec18621d0cf4f29a1d1bc64c396ca74bdfa1bd9a84e387ab22e79f2ef3c3ce6"
	},
	{
		"1c063b9bbc914902bdeb276bed4b280367ec595fc08ba7a6e3ee2982e1a472f7",
		2,
		data2,
		"7a5ea611c455955a4477824cde9c064d326fac814549810f4c323818d15f08d9"
	}
};

static int	hmac_sha256_test(int print)
{
	char key[32], *data[2], data_buf[2][110], mac[32], out_mac[32];
	int i, j, k, err = 0, data_len[2];

	if (print) {
		printf("\n[%s]\n", __FUNCTION__);
	}

	for (i = 0; i < (sizeof(Sha_TestVectors) / sizeof(sha_test_vector)); i ++)	{
		sha_test_vector *vector = &Sha_TestVectors[i];
		int num = vector->num;
		memset(key, 0, 32);
		for (k = 0; k < num; k ++) {
			memset(&data_buf[k][0], 0, 110);
			data[k] = &data_buf[k][0];
		}
		memset(mac, 0, 32);
		memset(out_mac, 0, 32);

		HexToData(vector->key, key, 32);
		for (k = 0; k < num; k ++) {
			data_len[k] = strlen(vector->data[k]) / 2;
			HexToData(vector->data[k], data[k], data_len[k]);
		}
		HexToData(vector->mac, mac, 32);

		rom_hmac_sha256_vector(key, 32, num, data, data_len, out_mac);

		if (print) {
			printf("key:");
			for (j = 0; j < 32; ++j) {
				printf("%02x", key[j]);
			}
			for (k = 0; k < num; k ++) {
				printf(", data:");
				for (j = 0; j < data_len[k]; ++j) {
					printf("%02x", data[k][j]);
				}
			}
			printf(", mac:");
			for (j = 0; j < 32; ++j) {
				printf("%02x", mac[j]);
			}
			printf(", out:");
			for (j = 0; j < 32; ++j) {
				printf("%02x", out_mac[j]);
			}
			printf("\n");
		}

		if (!(memcmp(mac, out_mac, 32) == 0)) {
			printf("%s mac Err\n", __FUNCTION__);
			err = 1;
			goto exit;
		}
	}

exit:
	return (err);
}

#define ENC_TEST 1
#define DEC_TEST 2

typedef struct {
	int	op;
	const char *key;
	const char *iv;
	const char *data;
	const char *expect;
} aes_test_vector;

static const aes_test_vector Aes_TestVectors[] = {
	{
		ENC_TEST,
		"052340b0dad92bf4931403d628aca31e",
		"28ca5a85a4ce4f8c901c92bd52044456",
		"101600107c268cb7461fa1c77b20bd8a86deca00101e0008707dae218bc214bc10101010101010101010101010101010",
		"8de19fa5b60759e456f9fae4183252bbf105f42ef0f69c21bb54f2a280f302dd4cbcc7e1fa207e762c62307a56008b1b"
	},
	{
		ENC_TEST,
		"052340b0dad92bf4931403d628aca31e",
		"4cf292404a1321ad3404e706ed298b51",
		"1017001094bfc57e89fec5dfd021fd4e478ddbe0101e0008e4e6fc2a00ceaa3310101010101010101010101010101010",
		"33f2ffbeed7212ae21abfeb268a6aff550a1f8eee19285150f1873e049b2e0e9c764e30f2886f8b8ec77eb533e39379d"
	},
	{
		DEC_TEST,
		"052340b0dad92bf4931403d628aca31e",
		"4647e8483d5fbccb2060d52c0b0b8313",
		"fd4adeac954ba3c289afbb5d2c9332aa348b4fe6f9d5000313dfda3ca2acfe86e49e7a1587d9105438d9548a9a6b9e87",
		"104000107cbb2c781e9dc6f46c7b5756503e5d51101e000823cfd014059bef4410101010101010101010101010101010"
	},
	{
		DEC_TEST,
		"052340b0dad92bf4931403d628aca31e",
		"2261a9d8674bc6246e4fe53e5d702acd",
		"35e437c0d8d3562602b9dc6d9a359c9039b870e938ebc1b6e2c21e9e4fa81eec6ecd4936b7b7275136278610ae091264859140522f305dcefad5aa4a927d37b5e625ee9222fb6bd7e7c0498013ff5533",
		"100e003210260001011045000a4449522d3631394c4231100300020001100f000200011028000101102700001020000600e04c870000101e0008526627207b3c138c0e0e0e0e0e0e0e0e0e0e0e0e0e0e"
	}
};

static int	aes128_cbc_test(int print)
{
	char key[16], iv[16], data[80], expect[80];
	int i, j, err = 0;

	if (print) {
		printf("\n[%s]\n", __FUNCTION__);
	}

	for (i = 0; i < (sizeof(Aes_TestVectors) / sizeof(aes_test_vector)); i ++)	{
		int data_len;
		aes_test_vector *vector = &Aes_TestVectors[i];
		memset(key, 0, 16);
		memset(iv, 0, 16);
		memset(data, 0, 80);
		memset(expect, 0, 80);
		data_len = strlen(vector->data) / 2;
		HexToData(vector->key, key, 16);
		HexToData(vector->iv, iv, 16);
		HexToData(vector->data, data, data_len);
		HexToData(vector->expect, expect, data_len);

		if (print) {
			if (vector->op == ENC_TEST) {
				printf("ENC_TEST, key:");
			} else if (vector->op == DEC_TEST) {
				printf("DEC_TEST, key:");
			}
			for (j = 0; j < 16; ++j) {
				printf("%02x", key[j]);
			}
			printf(", iv:");
			for (j = 0; j < 16; ++j) {
				printf("%02x", iv[j]);
			}
			printf(", data:");
			for (j = 0; j < data_len; ++j) {
				printf("%02x", data[j]);
			}
			printf(", expect:");
			for (j = 0; j < data_len; ++j) {
				printf("%02x", expect[j]);
			}
		}

		if (vector->op == ENC_TEST) {
			rom_aes_128_cbc_encrypt(key, iv, data, data_len);
		} else if (vector->op == DEC_TEST) {
			rom_aes_128_cbc_decrypt(key, iv, data, data_len);
		}

		if (print) {
			printf(", inplace:");
			for (j = 0; j < data_len; ++j) {
				printf("%02x", data[j]);
			}
			printf("\n");
		}

		if (!(memcmp(data, expect, data_len) == 0)) {
			printf("%s data inplace Err\n", __FUNCTION__);
			err = 1;
			goto exit;
		}
	}

exit:
	return (err);
}

int	wps_crypto_test(int print)
{
	int err = 0;

	if ((err = mod_exp_test(print)) == 1) {
		goto exit;
	}
	if ((err = hmac_sha256_test(print)) == 1) {
		goto exit;
	}
	if ((err = aes128_cbc_test(print)) == 1) {
		goto exit;
	}

exit:
	printf("%s: %s\n", __FUNCTION__, !err ? "PASSED" : "FAILED");
	return err;
}
