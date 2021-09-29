/**************************************************************************//**
 * @file     boot_load.c
 * @brief    To load the User image from the Flash.
 *
 * @version  V1.00
 * @date     2020-11-11
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2020 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 ******************************************************************************/
#include "cmsis.h"
#include "fw_img.h"
#include "fw_img_tlv.h"
#include "rtl8735b_symbns4s.h"
#include "rtl8735b_ramstart.h"
#include "otp_boot_cfg.h"
#include "memory.h"
//#include "strproc.h"
#include "hal_flash_boot.h"
#include "hal.h"
#include "hal_crypto.h"
#include "hal_sys_ctrl.h"

/* if enable Flash image load debug message */
#define BOOT_LOADER_DBG             0
#define USER_FW_IMAGE_SELECTION     0

#if BOOT_LOADER_DBG
#define BOOT_LOADER_DBG_PRINTF(...)     do {\
    dbg_printf(__VA_ARGS__);\
}while(0)
#else
#define BOOT_LOADER_DBG_PRINTF(...)
#endif

#define BOOT_SPIC_IO_MODE       (SpicQuadIOMode)

#define LS_IMG_SIZE                 (1024*4)
#define TEMP_DECRYPT_BUF_SIZE       (384)       // al least > sizeof(fw_sub_image_type_t) + sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t)
#define FLAH_IMAGE_START_ADDR       (SPI_FLASH_BASE+0x20)
#define SHARE_AES_KEY_LOG_SIZE      12
#define SHARE_HASH_KEY_LOG_SIZE     4

//#define SECTION_SBOOT_TEXT          SECTION("sboot.text")
//#define SECTION_SBOOT_DATA          SECTION("sboot.data")
//#define SECTION_SBOOT_RODATA        SECTION("sboot.rodata")
#define SECTION_SBOOT_BSS           SECTION(".sboot.bss")


// ----------------------
/**
    FIXME: PRO2_WKARD for fixing broken build
    1: mark broken build reference as comment
    0: original
 */
#define PRO2_WKARD (1)
// ----------------------


typedef int (*crypto_hmac_hash_init)(IN const u8 *key, IN const u32 keylen);
typedef int (*crypto_hash_init)(void);
typedef int (*crypto_hash_update)(IN const u8 *message, IN const u32 msglen);
typedef int (*crypto_hash_final)(OUT u8 *pDigest);
typedef int (*crypto_aes_init)(IN const u8 *key, IN const u32 keylen);
typedef int (*crypto_aes_decrypt)(IN const u8 *message, IN const u32 msglen, \
								  IN const u8 *iv, IN const u32 ivlen, OUT u8 *pResult);

extern hal_spic_func_stubs_t hal_spic_stubs;
extern hal_crypto_adapter_t g_rtl_cryptoEngine_s;

extern int32_t chk_ram_img_signature(char *sign);
extern void rom_temp_bss_clean_up(void);
extern hal_status_t hal_get_chip_id(uint32_t *pchip_id);

SECTION_SBOOT_BSS hal_spic_adaptor_t _hal_spic_adaptor;

SECTION_SBOOT_BSS fw_img_export_info_type_t *pfw_image_info;

typedef int (*boot_memcmp_t)(const void *av, const void *bv, size_t len);
int memcmp_s(const void *av, const void *bv, size_t len);

SECTION_SBOOT_BSS uint8_t tmp_img_hdr[FW_IMG_HDR_MAX_SIZE] __ALIGNED(32);
SECTION_SBOOT_BSS uint8_t tmp_sect_hdr[FW_IMG_HDR_MAX_SIZE] __ALIGNED(32);
SECTION_SBOOT_BSS uint8_t *pimg_start;


//SECTION_SBOOT_BSS uint8_t hash_priv_key_enced[PRIV_KEY_SIZE] __ALIGNED(32);  // Encrypted Priviate key for hash, get it from secure EFuse, must be cleaned before jump to RAM
//SECTION_SBOOT_BSS uint8_t hash_priv_key[PRIV_KEY_SIZE] __ALIGNED(32) ;  // Priviate key for hash, get it from secure EFuse, must be cleaned before jump to RAM
SECTION_SBOOT_BSS uint8_t pub_key[PUBLIC_KEY_SIZE * 2] __ALIGNED(32); // Public key, 1st for AES key generation, 2nd for hash key generation
SECTION_SBOOT_BSS uint8_t aes_key[AES_KEY_SIZE] __ALIGNED(32);   // AES key for image decryption
SECTION_SBOOT_BSS uint8_t hash_key[HASH_KEY_SIZE] __ALIGNED(32);   // key for image hash
SECTION_SBOOT_BSS uint8_t temp_dec_buf[TEMP_DECRYPT_BUF_SIZE] __ALIGNED(32);   // decrypted temp buffer
SECTION_SBOOT_BSS uint8_t aes_iv[AES_IV_LEN] __ALIGNED(32);

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
SECTION_SBOOT_BSS uint8_t sce_aes_key[SCE_KEY_LEN] __ALIGNED(16);
SECTION_SBOOT_BSS uint8_t sce_aes_iv[SCE_IV_LEN] __ALIGNED(16);
#endif /* FIXME: PRO2_WKARD for fixing broken build */


SECTION_SBOOT_BSS uint8_t temp_hash_out_buf[HASH_RESULT_SIZE] __ALIGNED(32);   // temp buffer for hash result
SECTION_SBOOT_BSS uint8_t image_hash_buf[HASH_RESULT_SIZE] __ALIGNED(32);   // temp buffer for hash result from image
SECTION_SBOOT_BSS uint8_t decrypt_ota_signature[OTA_SIGNATURE_SIZE] __ALIGNED(32); // the buffer to store decrypted OTG Signature
#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
SECTION_SBOOT_BSS sys_cp_fw_info_t sys_fw_info;
SECTION_SBOOT_BSS hal_sce_check_info_t sce_boot_info[2];
#endif /* FIXME: PRO2_WKARD for fixing broken build */

uint8_t export_hash_priv_key[HASH_KEY_SIZE] __ALIGNED(32);

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
hal_img1_tmp_buf_t boot_tmp_buf_rec;
#endif /* FIXME: PRO2_WKARD for fixing broken build */

uint8_t export_jtag_key[2][32] __ALIGNED(32);

//SECTION_SBOOT_BSS uint8_t sce_debug_buf[2048] __ALIGNED(32);

const char sce_cfg_sign[] = "8735bsce";
const char XIP_vrf_RamImgSignature[17] = {
	'A', 'm', 'e', 'b', 'a', 'P', 'r', 'o', '2',
	'R', 'T', 'L', '8', '7', '3', '5', 'B'
};

boot_init_flags_t boot_init_flags;
boot_memcmp_t boot_memcmp;

//SECTION_ROM_TEMP_BSS uint8_t fst_dec_buf[100];   // decrypted FST temp buffer
//SECTION_ROM_TEMP_BSS fm_image_header_t sec_header_dec; // temp decrypted secure header
//SECTION_ROM_TEMP_BSS partition_table_t partition_table;     // decrypted partition table

const uint8_t aes_init_iv[AES_IV_LEN] __ALIGNED(32) = {
	0xE7, 0x91, 0x9E, 0xE6, 0x98, 0xB1, 0xE5, 0x8D,
	0x8A, 0xE5, 0xB0, 0x8E, 0xE9, 0xAB, 0x94, 0x38
};

int memcmp_s(const void *av, const void *bv, size_t len)
{
	const uint8_t *a = av;
	const uint8_t *b = bv;
	int ret = 0;
	uint32_t i;

	for (i = 0; i < len; i++) {
		ret |= a[i] - b[i];
	}

	return ret;
}

int32_t is_key_valid(uint8_t *pkey)
{
	uint32_t i;

	for (i = 0; i < AES_KEY_SIZE; i++) {
		if (*(pkey + i) != 0xFF) {
			return 1;   // not all 0xFF, key is valid
		}
	}

	return 0;
}

#if 0
#if defined(CONFIG_BUILD_SECURE)
int32_t fw_gpio_trap_chk(gpio_pin_pwr_on_trap_t *gpio_trap)
{
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	hal_gpio_func_stubs_t *phal_gpio_stubs = symb_ns4s_stubs->phal_gpio_stubs;
	hal_gpio_comm_adapter_t gpio_comm_adp;
	hal_gpio_adapter_t gpio_adapter;
	gpio_pin_t pin;
	hal_status_t ret;
	int32_t traped = 0;
	uint32_t pin_lev;

	if ((gpio_trap->port < PORT_MAX_NUM) &&
		(gpio_trap->pin < MAX_PIN_IN_PORT) &&
		(gpio_trap->valid)) {
		// pin valid, check GPIO status
		phal_gpio_stubs->hal_gpio_comm_init(&gpio_comm_adp);
		pin.pin_name_b.port = gpio_trap->port;
		pin.pin_name_b.pin = gpio_trap->pin;
		ret = phal_gpio_stubs->hal_gpio_init(&gpio_adapter, pin);
		if (ret == HAL_OK) {
			pin_lev = phal_gpio_stubs->hal_gpio_read(&gpio_adapter);
			if (pin_lev == gpio_trap->io_lev) {
				traped = 1;
			}
		}
		phal_gpio_stubs->hal_gpio_deinit(&gpio_adapter);
		phal_gpio_stubs->hal_gpio_comm_deinit();
	}

	return traped;
}
#else
int32_t fw_gpio_trap_chk(gpio_pin_pwr_on_trap_t *gpio_trap)
{
	hal_gpio_comm_adapter_t gpio_comm_adp;
	hal_gpio_adapter_t gpio_adapter;
	gpio_pin_t pin;
	hal_status_t ret;
	int32_t traped = 0;
	uint32_t pin_lev;

	if ((gpio_trap->port < PORT_MAX_NUM) &&
		(gpio_trap->pin < MAX_PIN_IN_PORT) &&
		(gpio_trap->valid)) {
		// pin valid, check GPIO status
		hal_gpio_comm_init_rtl8710c(&gpio_comm_adp);
		pin.pin_name_b.port = gpio_trap->port;
		pin.pin_name_b.pin = gpio_trap->pin;
		ret = hal_gpio_init_rtl8710c(&gpio_adapter, pin);
		if (ret == HAL_OK) {
			pin_lev = hal_gpio_read_rtl8710c(&gpio_adapter);
			if (pin_lev == gpio_trap->io_lev) {
				traped = 1;
			}
		}
		hal_gpio_deinit_rtl8710c(&gpio_adapter);
		hal_gpio_comm_deinit_rtl8710c();
	}

	return traped;
}
#endif
#endif

/* Decrypt the header and signature, to read the sirial number in the header */
int32_t boot_get_serial(uint8_t secure_lock, uint8_t fw_rec_idx, const char *psign, uint32_t *sn)
{
	fw_part_record_t *pimg_record;
	fw_image_type_t *pfw_img;
	fm_image_header_t *pimg_hdr;
	uint32_t enc_ctrl = 0;
	char *pfw_sign;
	partition_table_t *ppartition_tbl;
	aes_key_type_t *paes_key;

	BOOT_LOADER_DBG_PRINTF("%s=>1, fw_rec_idx=%u\r\n", __FUNCTION__, fw_rec_idx);

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
	ppartition_tbl = (partition_table_t *)boot_get_partition_tbl();
#endif /* FIXME: PRO2_WKARD for fixing broken build */


	if (fw_rec_idx <= ppartition_tbl->image_info.rec_num) {
		pimg_record = (fw_part_record_t *)(((fw_part_record_t *)&ppartition_tbl->partition_record[0]) + fw_rec_idx);
		pfw_img = (fw_image_type_t *)(SPI_FLASH_BASE + pimg_record->start_addr);

		BOOT_LOADER_DBG_PRINTF("pimg_record=0x%x pfw_img=0x%x\r\n", pimg_record, pfw_img);

		if (!secure_lock) {
			enc_ctrl = pfw_img->sub_img.img_header.enc_ctrl;
			pimg_hdr = &(pfw_img->sub_img.img_header);
		}

		// Always use the 1st publick key to generate AES key for header decryption
		if (secure_lock || enc_ctrl) {
			if (fw_rec_idx == ppartition_tbl->image_info.fw1_idx) {
				// It's FW1
#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
				paes_key = (aes_key_type_t *)boot_get_fw1_key_tbl();
#endif /* FIXME: PRO2_WKARD for fixing broken build */


			} else if (fw_rec_idx == ppartition_tbl->image_info.fw2_idx) {
				// It's FW2
#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
				paes_key = (aes_key_type_t *)boot_get_fw2_key_tbl();
			} else {
#endif /* FIXME: PRO2_WKARD for fixing broken build */

				DBG_MISC_ERR("boot_get_serial: Invalid FW index(%lu)! No key table for this FW Idx\r\n", fw_rec_idx);
				return FAIL;
			}

			if (paes_key == NULL) {
//                DBG_MISC_ERR ("No Export Key Tbl\r\n");
				return FAIL;
			}

			enc_ctrl = 1;
			hal_crypto_aes_cbc_init((u8 *) & (paes_key->key[0]), AES_KEY_SIZE);
			memcpy((void *)aes_iv, (void *) aes_init_iv, AES_IV_LEN);

			// decrypt the Image header first
			if (secure_lock) {
				// The header is encrypted, decrypt it first
				// 2. decrypt the header
				if (SUCCESS != hal_crypto_aes_cbc_decrypt((uint8_t *) & (pfw_img->sub_img.img_header), \
						sizeof(fm_image_header_t), \
						aes_iv, AES_IV_LEN, temp_dec_buf)) {
					DBG_MISC_ERR("FW Img Header Decry Err!\r\n");
					return FAIL;
				}
				pimg_hdr = (fm_image_header_t *)temp_dec_buf;
			}

			// decrypt the OTA signature
			if (SUCCESS != hal_crypto_aes_cbc_decrypt((uint8_t *) & (pfw_img->ota_signature[0]), \
					OTA_SIGNATURE_SIZE, aes_iv, AES_IV_LEN, \
					temp_dec_buf + sizeof(fm_image_header_t))) {
				DBG_MISC_ERR("OTA Sign Decrypt Err!\r\n");
				return FAIL;
			}

			pfw_sign = (char *)temp_dec_buf + sizeof(fm_image_header_t);
		} else {
			// OTA Signature is plain test
			pfw_sign = (char *)(pfw_img->ota_signature);
		}

		// calculate the hash of the image header as the OTA signature
		if ((pimg_record->hkey_valid & 0x01) != 0) {
			// hash key in the partition record is valid, use it as the hash key
			if (hal_crypto_hmac_sha2_256_init(pimg_record->hash_key, HASH_KEY_SIZE) != SUCCESS) {
				DBG_MISC_ERR("%s: Hash-hmac Init Err!\r\n", __func__);
				return FAIL;
			}
			hal_crypto_hmac_sha2_256_update((uint8_t *)pimg_hdr, sizeof(fm_image_header_t));
			hal_crypto_hmac_sha2_256_final(temp_hash_out_buf);
		} else {
			// hash key in the partition record is ivalid, no hash key
			hal_crypto_sha2_256_init();
			hal_crypto_sha2_256_update((uint8_t *)pimg_hdr, sizeof(fm_image_header_t));
			hal_crypto_sha2_256_final(temp_hash_out_buf);
		}

		// Verify the OTA signature
		if (boot_memcmp((void *)pfw_sign, (void *)temp_hash_out_buf, OTA_SIGNATURE_SIZE)) {
			DBG_MISC_WARN("OTA Signature Incorrect!\r\n");
			return FAIL;
		}

		BOOT_LOADER_DBG_PRINTF("OTA Sign OK!\r\n");

		*sn = pimg_hdr->signature.serial_no;

		return SUCCESS;
	} else {
		BOOT_LOADER_DBG_PRINTF("boot_get_serial: FW Idx(%u) Invalid\r\n", fw_rec_idx);
		return FAIL;
	}
}

/**
 *  @brief To get the SN of the user selected firmware image to be loaded.
 *
 *  @param[in]  fw1_valid  To indicates the firmware 1 image is valid (signature valid) or not.
 *  @param[in]  fw1_sn     The serial number of the firmware 1 image.
 *  @param[in]  fw2_valid  To indicates the firmware 2 image is valid (signature valid) or not.
 *  @param[in]  fw2_sn     The serial number of the firmware 2 image.
 *
 *  @return     The serian number of the selected firmware image.
 *
 *  @note       If fw1_sn is identical with fw2_sn, bootloader would temporarily increase fw2_sn as fw1_sn+1. So that the fw selection feature can still be functional.
 */

__WEAK uint32_t user_boot_fw_selection(uint8_t fw1_valid, uint32_t fw1_sn, uint8_t fw2_valid, uint32_t fw2_sn)
{
	uint32_t sn;

	sn = fw1_sn + 1;
	while ((sn == fw1_sn) || (sn == fw2_sn)) {
		sn++;
	}
	return sn;
}

// To get the partition record index of the FW to be load and its key table
int32_t boot_get_load_fw_idx(uint8_t secure_lock, uint8_t *fw_idx, paes_key_type_t *ppkey_tbl)
{
	partition_table_t *ppartition_tbl;
	uint8_t fw1_rec_idx;
	uint8_t fw2_rec_idx;
	uint8_t fw1_valid = 0;
	uint8_t fw2_valid = 0;
	uint32_t fw1_sn = 0;
	uint32_t fw2_sn = 0;
	int32_t load_fw_idx;

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
	sys_dat_force_old_img_t old_img_trap;
#endif /* FIXME: PRO2_WKARD for fixing broken build */

	hal_gpio_comm_adapter_t gpio_com_adp;
	hal_gpio_adapter_t trap_pin;
#if USER_FW_IMAGE_SELECTION
	uint32_t user_fw_sn = 0;
	uint32_t fw2_sn_alien;
#endif

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
	ppartition_tbl = (partition_table_t *)boot_get_partition_tbl();
#endif /* FIXME: PRO2_WKARD for fixing broken build */

	fw1_rec_idx = ppartition_tbl->image_info.fw1_idx;
	fw2_rec_idx = ppartition_tbl->image_info.fw2_idx;

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
	pfw_image_info = get_fw_img_info_tbl();
#endif /* FIXME: PRO2_WKARD for fixing broken build */


	fw1_valid = pfw_image_info->fw1_valid;
	if (fw1_valid) {
		fw1_sn = pfw_image_info->fw1_sn;
	}

	fw2_valid = pfw_image_info->fw2_valid;
	if (fw2_valid) {
		fw2_sn = pfw_image_info->fw2_sn;
	}

	load_fw_idx = -1;
#if USER_FW_IMAGE_SELECTION
	fw2_sn_alien = fw2_sn;
	if (fw1_sn == fw2_sn_alien) {
		fw2_sn_alien++;
	}

	user_fw_sn = user_boot_fw_selection(fw1_valid, fw1_sn, fw2_valid, fw2_sn_alien);
	if ((user_fw_sn == fw1_sn) || (user_fw_sn == fw2_sn_alien)) {
		if ((user_fw_sn == fw1_sn) && fw1_valid) {
			load_fw_idx = fw1_rec_idx;
		} else if (fw2_valid) {
			load_fw_idx = fw2_rec_idx;
		}
	}

	if (load_fw_idx < 0)
#endif
	{
		if (fw1_valid && fw2_valid) {

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
			old_img_trap.word = *((uint32_t *)(SPI_FLASH_BASE + SYS_DATA_OFFSET_FORCE_OLD_IMG));
			if (((old_img_trap.word & 0x000000FF) != 0xFF) &&
				(old_img_trap.bit.port == 0) &&
				(old_img_trap.bit.pin < 24)) {
				hal_gpio_comm_init(&gpio_com_adp);
				hal_gpio_init(&trap_pin, old_img_trap.bit.pin);
				hal_gpio_set_dir(&trap_pin, GPIO_IN);
				if (hal_gpio_read(&trap_pin) == old_img_trap.bit.active) {
					if (fw2_sn < fw1_sn) {
						load_fw_idx = fw2_rec_idx;
					} else {
						load_fw_idx = fw1_rec_idx;
					}
				}
			} else {
				if (fw2_sn > fw1_sn) {
					load_fw_idx = fw2_rec_idx;
				} else {
					load_fw_idx = fw1_rec_idx;
				}
			}

#else /* FIXME: PRO2_WKARD for fixing broken build */
			load_fw_idx = fw1_rec_idx;
#endif /* FIXME: PRO2_WKARD for fixing broken build */

		} else if (fw1_valid && !fw2_valid) {
			load_fw_idx = fw1_rec_idx;
		} else if (!fw1_valid && fw2_valid) {
			load_fw_idx = fw2_rec_idx;
		}
	}

	if (load_fw_idx >= 0) {

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */

		if ((uint8_t)load_fw_idx == fw1_rec_idx) {
			*ppkey_tbl = (aes_key_type_t *)boot_get_fw1_key_tbl();
		} else if ((uint8_t)load_fw_idx == fw2_rec_idx) {
			*ppkey_tbl = (aes_key_type_t *)boot_get_fw2_key_tbl();
		}

#endif /* FIXME: PRO2_WKARD for fixing broken build */

		*fw_idx = (uint8_t)load_fw_idx;
		pfw_image_info->loaded_fw_idx = load_fw_idx;

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
		if (load_fw_idx == fw1_rec_idx) {
			boot_init_flags.b.img2_idx = 0;
		} else {
			boot_init_flags.b.img2_idx = 1;
		}
#endif /* FIXME: PRO2_WKARD for fixing broken build */

		DBG_MISC_INFO("boot_get_load_fw_idx: Load FW%u\r\n", load_fw_idx);
		return SUCCESS;
	} else {
		DBG_MISC_ERR("boot_get_load_fw_idx: failed!\r\n");
		return FAIL;
	}
}

#if 1   // Auto mode read
#define boot_flash_read(dst, src, size)         memcpy((void *)(dst), (void *)(SPI_FLASH_BASE+(src)), (uint32_t)(size))
#else   // User mode read
#define boot_flash_read(dst, src, size)         flash_burst_read (phal_spic_adaptor, (size), (src), (dst))
#endif

// make sure pheader is a 32-bytes aligned address
void load_jtag_key_from_img_hdr(fm_image_header_t *pheader)
{

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */

	//SYSON_Type *syson=SYSON;

	if ((boot_tmp_buf_rec.jtag_key_valid & (1 << S_JTAG_KEY)) == 0) {
		if ((pheader->user_key_val.w & (1 << S_JTAG_KEY))) {
			memcpy(export_jtag_key[S_JTAG_KEY], pheader->user_key_data.key_data[S_JTAG_KEY], 32);
			boot_tmp_buf_rec.jtag_key_valid |= (1 << S_JTAG_KEY);
#if BOOT_LOADER_DBG
			dbg_printf("boot-loader write s-jtag key: ");
			dump_bytes(pheader->user_key_data.key_data[S_JTAG_KEY], 32);
#endif
		}
	}

	if ((boot_tmp_buf_rec.jtag_key_valid & (1 << NS_JTAG_KEY)) == 0) {
		if ((pheader->user_key_val.w & (1 << NS_JTAG_KEY))) {
			memcpy(export_jtag_key[NS_JTAG_KEY], pheader->user_key_data.key_data[NS_JTAG_KEY], 32);
			boot_tmp_buf_rec.jtag_key_valid |= (1 << NS_JTAG_KEY);
#if BOOT_LOADER_DBG
			dbg_printf("boot-loader write ns-jtag key: ");
			dump_bytes(pheader->user_key_data.key_data[NS_JTAG_KEY], 32);
#endif
		}
	}

#endif /* FIXME: PRO2_WKARD for fixing broken build */

}

