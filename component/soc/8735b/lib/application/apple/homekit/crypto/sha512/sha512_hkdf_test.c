#include "rom_homekit_sha512.h"
#include <platform/platform_stdlib.h>

/* The tests have been derived from the original RFC 5869 test vectors, A.1. ~ A.7. */

struct hkdf_test {
	const char *ikm;
	const char *salt;
	const char *info;
	const char *okm;
};

static const struct hkdf_test hkdf_tests[] = {
	{
		"0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b",
		"000102030405060708090a0b0c",
		"f0f1f2f3f4f5f6f7f8f9",
		"832390086cda71fb47625bb5ceb168e4c8e26a1a16ed34d9fc7fe92c1481579338da362cb8d9f925d7cb"
	},
	{
		"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f404142434445464748494a4b4c4d4e4f",
		"606162636465666768696a6b6c6d6e6f707172737475767778797a7b7c7d7e7f808182838485868788898a8b8c8d8e8f909192939495969798999a9b9c9d9e9fa0a1a2a3a4a5a6a7a8a9aaabacadaeaf",
		"b0b1b2b3b4b5b6b7b8b9babbbcbdbebfc0c1c2c3c4c5c6c7c8c9cacbcccdcecfd0d1d2d3d4d5d6d7d8d9dadbdcdddedfe0e1e2e3e4e5e6e7e8e9eaebecedeeeff0f1f2f3f4f5f6f7f8f9fafbfcfdfeff",
		"ce6c97192805b346e6161e821ed165673b84f400a2b514b2fe23d84cd189ddf1b695b48cbd1c8388441137b3ce28f16aa64ba33ba466b24df6cfcb021ecff235f6a2056ce3af1de44d572097a8505d9e7a93"
	},
	{
		"0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b",
		"",
		"",
		"f5fa02b18298a72a8c23898a8703472c6eb179dc204c03425c970e3b164bf90fff22d04836d0e2343bac"
	},
	{
		"0b0b0b0b0b0b0b0b0b0b0b",
		"000102030405060708090a0b0c",
		"f0f1f2f3f4f5f6f7f8f9",
		"7413e8997e020610fbf6823f2ce14bff01875db1ca55f68cfcf3954dc8aff53559bd5e3028b080f7c068"
	},
	{
		"0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c",
		NULL,
		"",
		"1407d46013d98bc6decefcfee55f0f90b0c7f63d68eb1a80eaf07e953cfc0a3a5240a155d6e4daa965bb"
	}
};

static int str2bin(const char *str, unsigned char *buf, int size)
{
	int i, offset = strlen(str) % 2;

	if (size < (strlen(str) + 1) / 2) {
		return -1;
	}

	if (offset) {
		char hex = str[0];
		if ((hex >= '0') && (hex <= '9')) {
			buf[0] = (hex - '0');
		} else if ((hex >= 'a') && (hex <= 'f')) {
			buf[0] = (hex - 'a' + 10);
		} else if ((hex >= 'A') && (hex <= 'F')) {
			buf[0] = (hex - 'A' + 10);
		}
	}

	for (i = offset; i < offset + strlen(str) / 2; i ++) {
		char hex_high = str[i * 2];
		char hex_low = str[i * 2 + 1];
		buf[i] = 0;
		if ((hex_high >= '0') && (hex_high <= '9')) {
			buf[i] += ((hex_high - '0') * 16);
		} else if ((hex_high >= 'a') && (hex_high <= 'f')) {
			buf[i] += ((hex_high - 'a' + 10) * 16);
		} else if ((hex_high >= 'A') && (hex_high <= 'F')) {
			buf[i] += ((hex_high - 'A' + 10) * 16);
		}
		if ((hex_low >= '0') && (hex_low <= '9')) {
			buf[i] += (hex_low - '0');
		} else if ((hex_low >= 'a') && (hex_low <= 'f')) {
			buf[i] += (hex_low - 'a' + 10);
		} else if ((hex_low >= 'A') && (hex_low <= 'F')) {
			buf[i] += (hex_low - 'A' + 10);
		}
	}

	return i;
}

static void PrintBuffer(char *title, unsigned char *buf, int len)
{
	int i;
	printf("\n\r%s:", title);
	for (i = 0; i < len; i ++) {
		if (i % 32 == 0) {
			printf("\n\r");
		}
		printf(" %02x", buf[i]);
	}
	printf("\n\r");
}

int sha512_hkdf_test(int print)
{
	static const unsigned num_tests = sizeof(hkdf_tests) / sizeof(struct hkdf_test);
	int i, ret = 0;

	for (i = 0; i < num_tests && ret == 0; i ++) {
		const struct hkdf_test *test = &hkdf_tests[i];
		size_t ikm_len = 0, salt_len = 0, info_len = 0, okm_len = 0;
		unsigned char *ikm = NULL, *salt = NULL, *info = NULL, *okm = NULL, *okm_exp = NULL;

		ikm_len = strlen(test->ikm) / 2;
		ikm = (unsigned char *) malloc(ikm_len);
		str2bin(test->ikm, ikm, ikm_len);
		if (test->salt) {
			salt_len = strlen(test->salt) / 2;
			salt = (unsigned char *) malloc(salt_len);
			str2bin(test->salt, salt, salt_len);
		}
		info_len = strlen(test->info) / 2;
		info = (unsigned char *) malloc(info_len);
		str2bin(test->info, info, info_len);
		okm_len = strlen(test->okm) / 2;
		okm = (unsigned char *) malloc(okm_len);
		memset(okm, 0, okm_len);
		okm_exp = (unsigned char *) malloc(okm_len);
		str2bin(test->okm, okm_exp, okm_len);

		if (rom_sha512_hkdf(ikm, ikm_len, salt, salt_len, info, info_len, okm, okm_len) == 0) {
			if (memcmp(okm, okm_exp, okm_len) != 0) {
				printf("sha512_hkdf okm != okm_exp at test %d\n", i);
				PrintBuffer("okm", okm, okm_len);
				PrintBuffer("okm_exp", okm_exp, okm_len);
				ret = 1;
			} else if (print) {
				PrintBuffer("okm", okm, okm_len);
				PrintBuffer("okm_exp", okm_exp, okm_len);
			}
		} else {
			printf("sha512_hkdf failed at test %d\n", i);
			ret = 1;
		}

		if (ikm) {
			free(ikm);
		}
		if (salt) {
			free(salt);
		}
		if (info) {
			free(info);
		}
		if (okm) {
			free(okm);
		}
		if (okm_exp) {
			free(okm_exp);
		}
	}

	if (ret == 0) {
		printf("%s: PASSED\n", __FUNCTION__);
	}

	return ret;
}
