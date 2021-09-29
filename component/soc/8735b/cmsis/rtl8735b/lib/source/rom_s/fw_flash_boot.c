/**************************************************************************//**
 * @file     fw_flash_boot.c
 * @brief    Implement the booting from flash. The ROM code will load
 *           FW image load from flash to RAM and then jump to RAM code.
 *
 * @version  V1.00
 * @date     2021-08-04
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "cmsis.h"
#include "strproc.h"
#include "fw_img.h"
#include "rtl8735b_symbns4s.h"
#include "rtl8735b_ramstart.h"
#include "otp_boot_cfg.h"
#include "fw_img_tlv.h"
#include "fw_voe_rom_boot.h"


/* if enable Flash image load debug message */
#define FLASH_FW_LOAD_DBG           (0)

#if FLASH_FW_LOAD_DBG
#define FW_LOAD_DBG_PRINTF(...)     do {\
    dbg_printf(__VA_ARGS__);\
}while(0)
#else
#define FW_LOAD_DBG_PRINTF(...)
#endif

#define TEMP_DECRYPT_BUF_SIZE       (384)       // al least > sizeof(fw_sub_image_type_t) + sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t)
#define FLAH_IMAGE_START_ADDR       (SPI_FLASH_BASE+0x20)
#define MAX_IMG_SECTON_LOAD_LOG     16

#define SECTION_BOOT_EXPORT_BUF     SECTION(".rom_boot_export.bss")
#define SECTION_SBOOT_TEXT          SECTION(".rom.sboot.text")
#define SECTION_SBOOT_DATA          SECTION(".rom.sboot.data")
#define SECTION_SBOOT_RODATA        SECTION(".rom.sboot.rodata")
#define SECTION_SBOOT_BSS           SECTION(".rom.sboot.bss")
#define SECTION_SBOOT_STUBS         SECTION(".rom.sboot.stubs")
#define SECTION_SBOOT_TLV_STUBS     SECTION(".rom.sboot_tlv.stubs")


extern uint8_t fast_boot;
extern hal_spic_func_stubs_t hal_spic_stubs;
extern hal_timer_group_adapter_t _timer_group3;
extern hal_timer_adapter_t _fcs_system_timer;
const flash_pin_sel_t boot_flash_pins[FLASH_PINS_MAX_SEL] = {
	/*  CS,      CLK,    D0,      D1,       D2,     D3 */
	{ PIN_C5,  PIN_C0, PIN_C2, PIN_C4,  PIN_C3, PIN_C1}
};

const uint8_t img_mani_ptn_rmp_chk[] = {
	'R', 'T', 'L', '8', '7', '3', '5', 'B'
};

extern int32_t chk_ram_img_signature(char *sign);
extern void rom_temp_bss_clean_up(void);

#if 0
_LONG_CALL_ extern int _memcmp(const void *av, const void *bv, size_t len);
_LONG_CALL_ extern void *_memcpy(void *s1, const void *s2, size_t n);
_LONG_CALL_ extern void *_memmove(void *destaddr, const void *sourceaddr, unsigned length);
_LONG_CALL_ extern void *_memset(void *dst0, int val, size_t length);
#endif
// Not sure whether need long jump call attribute
extern int _memcmp_s(const void *av, const void *bv, size_t len);
extern void *_memcpy(void *s1, const void *s2, size_t n);
extern void *_memmove(void *destaddr, const void *sourceaddr, unsigned length);
extern void *_memset(void *dst0, int val, size_t length);

void *get_fw1_key_tbl_addr(void);
void *get_fw2_key_tbl_addr(void);
void clear_export_partition_tbl(void);
extern void erase_boot_loader(uint32_t code_start, uint32_t code_size, uint32_t img2_entry);

void *fw_img_info_tbl_query(void);
int32_t otu_fw_download(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset);
//int32_t fw_aes_key_export (void);

SECTION_SBOOT_BSS hal_crypto_adapter_t sb_rom_crypto_adtr;
SECTION_SBOOT_BSS hal_crypto_eddsa_t sb_rom_eddsa_adtr;
SECTION_SBOOT_BSS hal_hkdf_adapter_t sb_rom_hkdf_adtr;
SECTION_SBOOT_BSS hal_flash_sec_adapter_t sb_rom_sec_adtr;
SECTION_ROM_TEMP_BSS uint8_t _hal_spic_inited;
SECTION_ROM_TEMP_BSS uint8_t _hal_snafc_inited; /** for SNAND */
SECTION_ROM_TEMP_BSS hal_spic_adaptor_t _hal_spic_adaptor;
SECTION_ROM_TEMP_BSS hal_snafc_adaptor_t _hal_snafc_adaptor; /** for SNAND */
SECTION_ROM_TEMP_BSS image_load_log_t _img_load_log[MAX_IMG_SECTON_LOAD_LOG];
//SECTION_ROM_TEMP_BSS uint8_t hmac_key_tmp[32] __attribute__((aligned(32)));


// the buffer for the ROM boot image loader to decrypt and export the partition table for the RAM boot loader to read
SECTION_BOOT_EXPORT_BUF partition_table_t export_partition_tbl __ALIGNED(32);
INFRA_ROM_BSS_SECTION fw_img_export_info_type_t fw_image_info_export; // the Flash image information will export for Image2 to do OTA

#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN)

SECTION_SBOOT_STUBS const hal_flash_boot_stubs_t hal_flash_boot_stubs = {
	/* Secure Boot API */
	.ppartition_tbl = &export_partition_tbl,
	.get_fw1_key_tbl = get_fw1_key_tbl_addr,
	.get_fw2_key_tbl = get_fw2_key_tbl_addr,
	.clear_export_partition_tbl = clear_export_partition_tbl,
	.erase_boot_loader = erase_boot_loader,
	.fw_img_info_tbl_query = fw_img_info_tbl_query,
	.otu_fw_download = otu_fw_download,
	.fast_boot = &fast_boot
};

#endif  // #if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))

#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern uint8_t Image$$_RAM_FUNC_TBL$$Base[];
#define __ram_start_table_start__ Image$$_RAM_FUNC_TBL$$Base
extern uint8_t Image$$_RAM_IMG_SIGN$$Base[];
#define __ram_img_signature__ Image$$_RAM_IMG_SIGN$$Base
#elif defined ( __GNUC__ )
extern uint8_t __ram_start_table_start__[];
extern uint8_t __ram_img_signature__[];
#endif

#if 1
void *get_fw1_key_tbl_addr(void)
{
	return (void *)NULL;
}

void *get_fw2_key_tbl_addr(void)
{
	return (void *)NULL;
}
#endif
void clear_export_partition_tbl(void)
{
	_memset((void *)&export_partition_tbl, 0, sizeof(partition_table_t));
}

void *fw_img_info_tbl_query(void)
{
	return (void *)&fw_image_info_export;
}

void img_load_log_add(uint32_t img_sz, uint32_t img_start)
{
	uint32_t i;

	for (i = 0; i < MAX_IMG_SECTON_LOAD_LOG; i++) {
		if (_img_load_log[i].img_sz == 0) {
			_img_load_log[i].img_sz = img_sz;
			_img_load_log[i].start_addr = img_start;
			break;
		}
	}
}

void erase_loaded_image(void)
{
	uint32_t i;

	for (i = 0; i < MAX_IMG_SECTON_LOAD_LOG; i++) {
		if (_img_load_log[i].img_sz != 0) {
			_memset((void *)_img_load_log[i].start_addr, 0, _img_load_log[i].img_sz);
			_img_load_log[i].img_sz = 0;
		} else {
			break;
		}
	}
}

hal_status_t fw_spic_pinmux_ctl(phal_spic_adaptor_t phal_spic_adaptor, flash_pin_sel_t *pflash_pin_sel, uint8_t ctl)
{

	//uint8_t quad_pin_sel = phal_spic_adaptor->quad_pin_sel;

	// spic pinmux codes
	if (ctl == ENABLE) {
#if CONFIG_FPGA
		/*
		      CS,      CLK,    D0,      D1,       D2,     D3
		    { PIN_C5,  PIN_C0, PIN_C2, PIN_C4,  PIN_C3, PIN_C1}
		*/
		// C0, C2, C4, C5
		hal_rtl_pinmux_register(pflash_pin_sel->pin_clk, PID_GPIO);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_d0, PID_GPIO);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_d1, PID_GPIO);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_cs, PID_GPIO);

		// C1, C3
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d3, Pin_PullUp);
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d2, Pin_PullUp);
#else
		// C0, C2, C4, C5
		hal_rtl_pinmux_register(pflash_pin_sel->pin_clk, PID_FLASH);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_d0, PID_FLASH);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_d1, PID_FLASH);
		hal_rtl_pinmux_register(pflash_pin_sel->pin_cs, PID_FLASH);

		// C1, C3
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d3, Pin_PullUp);
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d2, Pin_PullUp);
#endif
	} else {
#if CONFIG_FPGA
		// C0, C2, C4, C5
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_clk, PID_GPIO);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_d0, PID_GPIO);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_d1, PID_GPIO);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_cs, PID_GPIO);

		// C1, C3
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d3, Pin_PullNone);
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d2, Pin_PullNone);
#else
		// C0, C2, C4, C5
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_clk, PID_FLASH);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_d0, PID_FLASH);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_d1, PID_FLASH);
		hal_rtl_pinmux_unregister(pflash_pin_sel->pin_cs, PID_FLASH);

		// C1, C3
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d3, Pin_PullNone);
		hal_rtl_gpio_pull_ctrl((u32)pflash_pin_sel->pin_d2, Pin_PullNone);
#endif
	}

	return HAL_OK;
}


hal_status_t fw_spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, u8 io_pin_sel)
{
	hal_status_t ret = HAL_OK;
	flash_pin_sel_t *pflash_pin_sel;

	if (_hal_spic_inited) {
		return ret;
	}

	phal_spic_adaptor->spic_bit_mode = SpicOneIOMode;
	// Due to Pro2 rom code always using SpicOneIOMode
	spic_bit_mode = SpicOneIOMode;
	phal_spic_adaptor->flash_pin_sel = SpicQuadIOPin;   // Due to Pro2 current only has 1 pin select
	pflash_pin_sel = (flash_pin_sel_t *)&boot_flash_pins[0];

	fw_spic_pinmux_ctl(phal_spic_adaptor, pflash_pin_sel, ENABLE);

	// Todo: Wait for Flash SCE confirm
	//hal_rtl_sce_func_enable();

	if (spic_rtl_init_setting(phal_spic_adaptor, spic_bit_mode) != HAL_OK) {
		ret = HAL_ERR_HW;
		return ret;
	}

	spic_rtl_config_auto_mode(phal_spic_adaptor);

	/*Set user relevant parameters according to bit mode*/
	spic_rtl_config_user_mode(phal_spic_adaptor);
	_hal_spic_inited = 1;
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 6);
	return ret;
} /* fw_spic_init */

hal_status_t fw_spic_deinit(phal_spic_adaptor_t phal_spic_adaptor)
{
	flash_pin_sel_t *pflash_pin_sel;
	pflash_pin_sel = (flash_pin_sel_t *)&boot_flash_pins[0];  // Due to Pro2 current only has 1 pin select

	fw_spic_pinmux_ctl(phal_spic_adaptor, pflash_pin_sel, DISABLE);
	_hal_spic_inited = 0;

	return HAL_OK;
} /* fw_spic_deinit */

hal_status_t fw_snafc_init(hal_snafc_adaptor_t *pSnafcAdaptor, u8 spic_bit_mode, u8 io_pin_sel)
{
	hal_status_t ret = HAL_OK;
	flash_pin_sel_t *pflash_pin_sel;
	pflash_pin_sel = (flash_pin_sel_t *)&boot_flash_pins[0];  // Due to Pro2 current only has 1 pin select
	uint32_t regAddr;
	uint32_t regVal;

	if (_hal_snafc_inited) {
		return ret;
	}
	fw_spic_pinmux_ctl(NULL/*dummy in Pro2*/, pflash_pin_sel, ENABLE); /** Philip: pinmux as the same as spic */
	hal_rtl_snand_init(pSnafcAdaptor);

	regVal = hal_rtl_snand_issueReadIdOpCmd(pSnafcAdaptor);
	if (regVal == 0) {
		ret = HAL_ERR_UNKNOWN;
		return ret;
	}

	/* force protectBlockNone */
	regAddr = 0xa0;
	regVal = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pSnafcAdaptor, regAddr);
	regVal &= ~(0x07 << 3); /*Philip: MX35 using ~(0x7<<3), W25N using ~(0xF<<3)*/
	hal_rtl_snand_issueSetFeatureRegisterOpCmd(pSnafcAdaptor, regAddr, regVal);
	/* force protectBlockNone */

#if 0
	/* check sts */
	regAddr = 0xc0;
	regVal = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pSnafcAdaptor, regAddr);
	/* return value (regVal) sholde be 0x00, which means IDLE (available) */
	dbg_printf("%s Ln d. retVal=0x%x\r\n", __FILE__, __LINE__, regVal); /*PhilipDebug*/
#endif
	_hal_snafc_inited = 1;
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 6);
	return ret;
} /* fw_snafc_init */

hal_status_t fw_snafc_deinit(hal_snafc_adaptor_t *pSnafcAdaptor)
{
	flash_pin_sel_t *pflash_pin_sel;
	pflash_pin_sel = (flash_pin_sel_t *)&boot_flash_pins[0];  // Due to Pro2 current only has 1 pin select

	hal_rtl_snand_init(pSnafcAdaptor);
	fw_spic_pinmux_ctl(NULL/*dummy in Pro2*/, pflash_pin_sel, DISABLE); /** Philip: pinmux as the same as spic */
	_hal_snafc_inited = 0;

	return HAL_OK;
} /* fw_snafc_deinit */

#define fw_flash_read(dst, src, size)         _memcpy((void *)(dst), (void *)(SPI_FLASH_BASE+(src)), (uint32_t)(size))

extern void dump_for_one_bytes(u8 *pdata, u32 len);

/**  \brief     __dbg_mem_dump is only used to print out memory information of CRYPTO IP. \n
 *              Use \em enable to enable/disable this function which dumps \em size bytes of momery information
 *              from the \em start address.
 */
#undef __dbg_mem_dump
#define __dbg_mem_dump(start, size, str_header) do{ \
        dbg_printf(str_header "\r\n");\
        dump_for_one_bytes ((u8 *)start, size);\
}while(0)

int32_t rom_boot_load(PRAM_FUNCTION_START_TABLE *pram_start_func)
{

	partition_table_t *pPartitionTable;
	fm_image_header_t *pheader;
	fw_part_record_t *pbootimg_record;
	uint32_t image_size;
	uint32_t flash_offset;
	uint32_t i;
	uint32_t tFwIdx = -1UL;

	dbg_printf("[Start Boot ROM...]\r\n");
	pPartitionTable = (partition_table_t *)(SPI_FLASH_BASE + FLASH_CAL_PATTER_SIZE + (PUBLIC_KEY_SIZE * 2));

	pheader = (fm_image_header_t *)(SPI_FLASH_BASE + FLASH_CAL_PATTER_SIZE + (PUBLIC_KEY_SIZE * 2));
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 9);
	// Verify the partition table header
	if (pheader->img_type != FW_IMG_PT) {
		DBG_BOOT_ERR("Parttiton Table Header(Plain Text) Verification Err!\r\n");
		return FAIL;
	}

	// Decrypt the whole Partition Table and copy it to SRAM for Image1 can read it
	image_size = ((((sizeof(fm_image_header_t) + pheader->image_len) - 1) >> 5) + 1) << 5 ;
	image_size += HASH_RESULT_SIZE;
	if (image_size > sizeof(partition_table_t)) {
		DBG_BOOT_ERR("Parttiton Table Size too big!\r\n");
		return FAIL;
	}

	_memcpy((void *)&export_partition_tbl.header, (void *)pheader, sizeof(fm_image_header_t));
	rtl_dcache_clean_by_addr((uint32_t *)&export_partition_tbl, sizeof(fm_image_header_t));
	image_size -= sizeof(fm_image_header_t);

	// the whole partition table is plain text, just copy it to the export buffer
	fw_flash_read((&export_partition_tbl.image_info), \
				  ((FLASH_CAL_PATTER_SIZE + (PUBLIC_KEY_SIZE * 2)) + sizeof(fm_image_header_t)), image_size);
	rtl_dcache_clean_by_addr((uint32_t *)(&export_partition_tbl.image_info), image_size);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 10);

#if FLASH_FW_LOAD_DBG
	dbg_printf("Decrypted Partition Table:\r\n");
	dump_for_one_bytes(&export_partition_tbl, (32 * 6));
#endif

	pheader = (fm_image_header_t *)&export_partition_tbl.header;

	for (i = 0; i < export_partition_tbl.image_info.rec_num; i++) {
		pbootimg_record = &(export_partition_tbl.partition_record[i]);

		/*Load BootLoader */
		if (pbootimg_record->pt_type == FW_PT_Boot) {
			tFwIdx = i;
#if 0
			dbg_printf("[%d]=BOOTLOAD.\r\n", tFwIdx);
#endif
			break;
		}
	}

	/*Load BootLoader */
	if (pbootimg_record->pt_type == FW_PT_Boot) {
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 11);
		/* process BL1 */
		//uint8_t *pDecPubKey = NULL, *pHashPubKey = NULL, *pHashResult = NULL;
		fw_image_section_type_t *pfw_enced_img = NULL;
		raw_image_hdr_t *pSectionHdr = NULL;
		//pDecPubKey = (uint8_t *)(SPI_FLASH_BASE + pPartitionTable->partition_record[tFwIdx].start_addr);
		//pHashPubKey = (uint8_t *)(SPI_FLASH_BASE + pPartitionTable->partition_record[tFwIdx].start_addr + (PUBLIC_KEY_SIZE*1) );
		pfw_enced_img = (fw_image_section_type_t *)(SPI_FLASH_BASE + pPartitionTable->partition_record[tFwIdx].start_addr + (PUBLIC_KEY_SIZE * 2));
		pSectionHdr = &(pfw_enced_img->img_raw_header);
		//pHashResult = ((uint8_t *)(pfw_enced_img)+sizeof(/*pfw_enced_img->sec_header*/fm_image_header_t)+pfw_enced_img->sec_header.image_len);

		flash_offset = (pPartitionTable->partition_record[tFwIdx].start_addr + (PUBLIC_KEY_SIZE * 2) + sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t));

		fw_flash_read(pSectionHdr->start_addr, flash_offset, pSectionHdr->img_sz);
		rtl_dcache_clean_by_addr((uint32_t *)pSectionHdr->start_addr, pSectionHdr->img_sz);
		if (pSectionHdr->ram_start_tbl != -1UL) {
			*pram_start_func = (RAM_FUNCTION_START_TABLE *)pSectionHdr->ram_start_tbl;
			ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 12);
		}
	}

	return SUCCESS;
}

const char ImgManiValidateLbl[] = {
	'R', 'T', 'L', '8', '7', '3', '5', 'B', 0xFF
};

#define get_1byte(ie_t_ptr)               (*(ie_t_ptr + 0))
#define get_3byte(ie_l_ptr)               (((*(ie_l_ptr + 0) & 0xFF) << 0) | \
                                           ((*(ie_l_ptr + 1) & 0xFF) << 8) | \
                                           ((*(ie_l_ptr + 2) & 0xFF) << 16))
#define get_4byte(ie_l_ptr)               (((*(ie_l_ptr + 0) & 0xFF) << 0) | \
                                           ((*(ie_l_ptr + 1) & 0xFF) << 8) | \
                                           ((*(ie_l_ptr + 2) & 0xFF) << 16) | \
                                           ((*(ie_l_ptr + 3) & 0xFF) << 24))
#define RAM_FUN_TABLE_VALID_START_ADDR         (0x20100000)
#define RAM_FUN_TABLE_VALID_END_ADDR           (0x20180000)

#define CERTIFICATE_TBL_FLSH_OFFSET         (0x20)
#define PARTITION_TBL_FLSH_OFFSET           (0x1000)

#undef __mem_dump_str
#define __mem_dump_str(start,size,str) do{ \
    dbg_printf("%s\r\n",str); \
    dump_for_one_bytes((uint8_t *)start,size); \
}while(0)

#undef __mem_dump
#define __mem_dump(start,size,prefix) do{ \
    dbg_printf(prefix "\r\n"); \
    dump_for_one_bytes((uint8_t *)start,size); \
}while(0)

const ie_tlv_const_tbl_t ie_tlv_const_tbl[MANIFEST_IE_ID_MAX_NUM] = {
	{IE_PK_ID,         "Public Key",            384},
	{IE_VERSION_ID,    "Image Version",         32},
	{IE_IMGSZ_ID,      "Image Size",            4},
	{IE_TYPEID_ID,     "Type ID",               2},
	{IE_ENCALG_ID,     "Encrypt Algorithm",     1},
	{IE_ENCKN_ID,      "Encrypt Key Nonce",     32},
	{IE_ENCKS_ID,      "Encrypt Key Salt",      32},
	{IE_ENCIV_ID,      "Encrypt IV",            16},
	{IE_HSHALG_ID,     "Hash Algorithm",        1},
	{IE_HSHKN_ID,      "Hash Key Nonce",        32},
	{IE_HSHKS_ID,      "Hash Key Salt",         32},
	{IE_HASH_ID,       "Image Hash Digest",     32},
	{IE_TIMST_ID,      "Time Stamp",            8},
	{IE_VID_ID,        "Vendor ID",             4},
	{IE_PID_ID,        "Product ID",            4},
	{IE_IMGLST_ID,     "Image update List",     8},
	{IE_DEP_ID,        "Image update dependency", 8},
	{IE_RMATKN_ID,     "RMA Token",             4},
	{IE_BATLV_ID,      "Battery level",         4},
	{IE_ACPW_ID,       "AC or charger check",   1},
	{IE_IERESV_ID,     "IE reserved type",      MANIFEST_IE_RESERV_SIZE},

	{IE_NAN_ID,        "IE NON-Define type",    1},
};

const part_type_id_tbl_t part_type_const_tbl[PARTITION_TABLE_ID_MAX_NUM] = {
	{FW_PT_INI_VAL_ID,        "FW_PT_INI_VAL"},
	{FW_PT_KEY_CER_TBL_ID,    "FW_PT_KEY_CER_TBL"},
	{FW_PT_KEY_CER1_ID,       "FW_PT_KEY_CER1"},
	{FW_PT_KEY_CER2_ID,       "FW_PT_KEY_CER2"},
	{FW_PT_PT_ID,             "FW_PT_PT"},
	{FW_PT_BL_PRI_ID,         "FW_PT_BL_PRI"},
	{FW_PT_BL_SEC_ID,         "FW_PT_BL_SEC"},
	{FW_PT_FW1_ID,            "FW_PT_FW1"},
	{FW_PT_FW2_ID,            "FW_PT_FW2"},
	{FW_PT_ISP_IQ_ID,         "FW_PT_ISP_IQ"},
	{FW_PT_NN_MDL_ID,         "FW_PT_NN_MDL"},
	{FW_PT_RESV_ID,           "FW_PT_RESV"}
};

const uint16_t ld_img_type_id_list[] = {
	FW_IMG_BL_ID, FW_IMG_FWHS_S_ID, FW_IMG_FWHS_NS_ID, FW_IMG_ISP_ID, FW_IMG_VOE_ID, FW_IMG_WLAN_ID,
	FW_IMG_XIP_ID, FW_IMG_CPFW_ID, FW_IMG_WOWLN_ID, FW_IMG_CINIT_ID
};

SECTION_ROM_TEMP_BSS manif_ie_tlv_t ie_tbl_cache[MANIFEST_IE_TBL_MAX_NUM];

SECTION_BOOT_EXPORT_BUF sec_boot_keycerti_t export_sb_keycerti;
SECTION_BOOT_EXPORT_BUF certi_tbl_t certi_rec_tbl;
SECTION_BOOT_EXPORT_BUF img_manifest_ld_sel_t ld_sel_info1, ld_sel_info2;
SECTION_BOOT_EXPORT_BUF part_tbl_info_t part_tbl_info;
//SECTION_BOOT_EXPORT_BUF manif_ie_data_t manif_ie;
SECTION_BOOT_EXPORT_BUF uint8_t mani_data[MANIFEST_IEDATA_SIZE];
SECTION_BOOT_EXPORT_BUF manif_sign_t manif_sign;
SECTION_BOOT_EXPORT_BUF partition_tbl_t export_part_tbl __ALIGNED(32);
SECTION_BOOT_EXPORT_BUF uint8_t tmp_img_hdr[FW_IMG_HDR_MAX_SIZE] __ALIGNED(32);
SECTION_BOOT_EXPORT_BUF uint8_t tmp_sect_hdr[FW_IMG_HDR_MAX_SIZE] __ALIGNED(32);
SECTION_BOOT_EXPORT_BUF uint8_t *pimg_start;
SECTION_BOOT_EXPORT_BUF sec_boot_info_t sb_rom_info;
//SECTION_BOOT_EXPORT_BUF manif_sec_enc_record_t sec_enc_record[MAX_SEC_ENC_RECORD];
SECTION_BOOT_EXPORT_BUF sb_sec_enc_info_t sec_enc_info;
SECTION_BOOT_EXPORT_BUF uint8_t sb_digest_buf[IMG_HASH_CHK_DIGEST_MAX_SIZE];
SECTION_BOOT_EXPORT_BUF hal_sec_region_cfg_t sb_sec_cfg_pending __ALIGNED(32);

void clear_export_partition_tbl_info(void);
void clear_export_sb_keycerti_info(void);
int verify_manif_f(const uint8_t *img_offset, const uint8_t info_type, sec_boot_info_t *p_sb_info); //TOdo info_type may not use
void load_img_hdr_f(const uint8_t *img_offset, fw_img_hdr_t *img_hdr_buf, const uint8_t sel_img_load);
uint8_t search_available_idx(uint8_t *p_sts, const uint8_t cfg_max_size);
int xip_pending_cfg_add_rmp(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_rmp_lkp_tbl,
							uint32_t phy_addr, uint32_t remap_addr, uint32_t remap_sz);
int xip_pending_process_rmp(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info);
int xip_disable_rmp_config(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel);
int xip_pending_cfg_add_dec_key(hal_sec_region_cfg_t *psb_sec_cfg_pending, const uint8_t key_id);
int xip_pending_cfg_add_dec(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_dec_lkp_tbl,
							uint8_t cipher_sel, uint32_t dec_base, uint32_t dec_sz,
							uint32_t iv_ptn_low, uint32_t iv_ptn_high,
							uint32_t tag_base_addr, uint32_t tag_flh_addr, uint32_t total_hdr_size);
int xip_pending_process_dec(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info);
int xip_disable_dec_config(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel);
int img_rmp_and_dec_lkp_tbl_insert(img_region_lookup_t *plkp_tbl, uint8_t tbl_size, uint8_t is_xip, uint8_t *tbl_cnt);
void img_rmp_and_dec_lkp_tbl_remove(img_region_lookup_t *plkp_tbl, uint8_t *tbl_cnt);
int img_get_ld_sel_info_from_ie(const uint8_t img_obj, const uint8_t *ptr, img_manifest_ld_sel_t *pld_sel_info);
uint32_t get_ld_version(const uint8_t img_obj, uint8_t *p_img_version);
uint32_t get_ld_timst(uint8_t *p_img_timest);
int img_get_ld_sel_idx(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx, uint8_t img2_idx);
int img_get_update_sel_idx(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx,
						   uint8_t img2_idx);
int img_sel_op_idx(void *p_tbl_info, const uint8_t img_obj, const uint8_t img_sel_op);


#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN)

SECTION_SBOOT_TLV_STUBS const hal_flash_boot_tlv_stubs_t hal_flash_boot_tlv_stubs = {
	/* Secure Boot API */
	.ppartition_tbl = &export_part_tbl,
	.pkeycerti = &export_sb_keycerti,
	.clear_export_partition_tbl_info = &clear_export_partition_tbl_info,
	.clear_export_sb_keycerti_info = &clear_export_sb_keycerti_info,
	.erase_boot_loader = erase_boot_loader,
	.verify_manif_f = verify_manif_f,
	.load_img_hdr_f = load_img_hdr_f,
	.search_available_idx = search_available_idx,
	.xip_pending_cfg_add_rmp = xip_pending_cfg_add_rmp,
	.xip_pending_process_rmp = xip_pending_process_rmp,
	.xip_disable_rmp_config  = xip_disable_rmp_config,
	.xip_pending_cfg_add_dec_key = xip_pending_cfg_add_dec_key,
	.xip_pending_cfg_add_dec = xip_pending_cfg_add_dec,
	.xip_pending_process_dec = xip_pending_process_dec,
	.xip_disable_dec_config = xip_disable_dec_config,
	.img_rmp_and_dec_lkp_tbl_insert = img_rmp_and_dec_lkp_tbl_insert,
	.img_rmp_and_dec_lkp_tbl_remove = img_rmp_and_dec_lkp_tbl_remove,
	.img_get_ld_sel_info_from_ie    = img_get_ld_sel_info_from_ie,
	.get_ld_version = get_ld_version,
	.get_ld_timst   = get_ld_timst,
	.img_get_ld_sel_idx     = img_get_ld_sel_idx,
	.img_get_update_sel_idx = img_get_update_sel_idx,
	.img_sel_op_idx         = img_sel_op_idx,
};

#endif  // #if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))

void clear_export_partition_tbl_info(void)
{
	_memset((void *)&export_part_tbl, 0, sizeof(partition_tbl_t));
	_memset((void *)&part_tbl_info, 0, sizeof(part_tbl_info_t));
}

void clear_export_sb_keycerti_info(void)
{
	_memset((void *)&export_sb_keycerti, 0, sizeof(sec_boot_keycerti_t));
}

int32_t chk_manifest_lbl(char *lbl_name)
{
	uint32_t i;
	int32_t ret = 0;

	for (i = 0; i < sizeof(ImgManiValidateLbl); i++) {
		if (ImgManiValidateLbl[i] == 0xFF) {
			break;
		}

		if (ImgManiValidateLbl[i] != lbl_name[i]) {
			DBG_BOOT_ERR("Invalid Image Mani Label!\r\n");
			ret = 1;
			break;
		}
	}

	return ret;
}

void find_sb_ie_from_tbl_cache(sec_boot_info_t *p_sb_info, manif_ie_tlv_t *p_ie_tbl_cache, uint8_t ie_id)
{
	uint8_t i = 0, find_ie = 0;
	uint8_t hshalg = 0x0;
	uint8_t *p_val = NULL;
	uint32_t imgsz = 0;
	if (!p_sb_info) {
		goto find_sb_ie_from_tbl_cache_end;
	}
	if (p_ie_tbl_cache == NULL) {
		goto find_sb_ie_from_tbl_cache_end;
	}
	while (((p_ie_tbl_cache + i)->p_val) != NULL) {
		if (((p_ie_tbl_cache + i)->id) == ie_id) {
			p_val = ((p_ie_tbl_cache + i)->p_val);
			find_ie = 1;
			break;
		}
		i++;
	}
	if (find_ie == 1) {
		switch (ie_id) {
		case IE_PK_ID:
			p_sb_info->img_sign_vrf_info.p_pbk = p_val;
			break;
		case IE_IMGSZ_ID:
			//__mem_dump(p_val, 4, "imgsz_p_val");
			imgsz = get_4byte(p_val);
			p_sb_info->img_hash_chk_info.msglen = imgsz;
			break;
		case IE_HSHALG_ID:
			hshalg = get_1byte(p_val);
			p_sb_info->img_hash_chk_info.hash_alg = hshalg;
			break;
		case IE_HSHKS_ID:
			p_sb_info->img_hash_chk_info.p_key_salt = p_val;
			break;
		case IE_HSHKN_ID:
			p_sb_info->img_hash_chk_info.p_key_nonce = p_val;
			break;
		case IE_HASH_ID:
			p_sb_info->img_hash_chk_info.p_digest = p_val;
			break;
		default:
			break;
		}
	}
find_sb_ie_from_tbl_cache_end:
	return;
}

int sb_rom_img_pbk_hash_vrf_f(sec_boot_info_t *p_sb_info, const uint8_t info_type)
{
	uint8_t vrf_alg = IMG_SIGN_VRF_ALG_EDDSA_ED25519, digest_len = 0, i = 0;
	uint8_t *p_pbk_chk = NULL, *p_pbk = NULL, *p_digest = NULL;
	uint32_t vrf_pbk_len = 0;
	int pbk_vrf_ret = -1;
	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_rom_img_pbk_hash_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_pbk) == NULL) {
		DBG_BOOT_ERR("p_pbk is NULL\r\n");
		goto sb_rom_img_pbk_hash_vrf_f_end;
	}
	p_pbk = p_sb_info->img_sign_vrf_info.p_pbk;
	p_digest = &sb_digest_buf[0];
	_memset(p_digest, 0x0, IMG_HASH_CHK_DIGEST_MAX_SIZE);
	digest_len = SB_PBK_HASH_SIZE;

	vrf_alg = p_sb_info->img_sign_vrf_info.vrf_alg;
	switch (vrf_alg) {
	case IMG_SIGN_VRF_ALG_EDDSA_ED25519:
		vrf_pbk_len = 32;
		break;
	case IMG_SIGN_VRF_ALG_ECDSA256:
		// TODO
		vrf_pbk_len = 64;
		break;
	case IMG_SIGN_VRF_ALG_RSA2048:
		// TODO
		vrf_pbk_len = 256;
		break;
	case IMG_SIGN_VRF_ALG_RSA3072:
		// TODO
		vrf_pbk_len = 384;
		break;
	default:
		vrf_pbk_len = 32;
		break;
	}

	if (KEY_CERTI_INFO == info_type) {
		// ROTPK HASH from OTP
		uint32_t rotpk_hsh_addr;
		p_pbk_chk = &(p_sb_info->psb_keycerti->rotpk_hsh_data[0]);
		_memset(p_pbk_chk, 0xFF, SB_PBK_HASH_SIZE);
		if (ROTPK_IDX1 == p_sb_info->psb_keycerti->rotpk_hsh_idx) {
			rotpk_hsh_addr = (ROTPK_HSH_OTP_BASE_ADDR + 0x0);
		} else if (ROTPK_IDX2 == p_sb_info->psb_keycerti->rotpk_hsh_idx) {
			rotpk_hsh_addr = (ROTPK_HSH_OTP_BASE_ADDR + SB_PBK_HASH_SIZE);
		} else {
			rotpk_hsh_addr = ROTPK_HSH_RMA_OTP_BASE_ADDR;
		}
		// Get ROTPK HASH from OTP
		for (i = 0; i < SB_PBK_HASH_SIZE; i++) {
			p_pbk_chk[i] = hal_rtl_otp_byte_rd_syss((rotpk_hsh_addr + i));
		}
	} else {
		// PK HASH from Key certificate
		uint16_t img_type_id = FW_IMG_RESV_ID, fd_img_type_id = FW_IMG_RESV_ID;
		switch (info_type) {
		case PT_TBL_INFO:
			fd_img_type_id = FW_IMG_PT_ID;
			break;
		case BL_INFO:
			fd_img_type_id = FW_IMG_BL_ID;
			break;
		case FW_IMG_INFO:
			fd_img_type_id = FW_IMG_FWHS_S_ID;
			break;
		default:
			break;
		}
		for (i = 0; i < (p_sb_info->psb_keycerti->key_hash_rec_num); i++) {
			img_type_id = p_sb_info->psb_keycerti->img_pkhsh_info[i].key_hsh_tbl.type_id;
			if (fd_img_type_id == img_type_id) {
				p_pbk_chk = &(p_sb_info->psb_keycerti->img_pkhsh_info[i].key_pkhsh.data);
				break;
			}
		}
	}

	hal_rtl_crypto_auth_init(&sb_rom_crypto_adtr, AUTH_TYPE_SHA2_256_ALL, NULL, 0);
	hal_rtl_crypto_auth_update(&sb_rom_crypto_adtr, AUTH_TYPE_SHA2_256_ALL, p_pbk, vrf_pbk_len);
	hal_rtl_crypto_auth_final(&sb_rom_crypto_adtr, AUTH_TYPE_SHA2_256_ALL, p_digest);
#if FLASH_FW_LOAD_DBG
	__mem_dump(p_digest, digest_len, "pbk_digest");
#endif

	if (_memcmp(p_pbk_chk, p_digest, digest_len) == 0) {
		dbg_printf("[vrf pbk pass]\r\n");
		pbk_vrf_ret = SUCCESS;
	} else {
		DBG_BOOT_ERR("[vrf pbk fail]\r\n");
		pbk_vrf_ret = -1;
#if FLASH_FW_LOAD_DBG
		__mem_dump(p_pbk, vrf_pbk_len, "p_pbk");
		__mem_dump(p_pbk_chk, digest_len, "pbk_chk_digest");
		__mem_dump(p_digest, digest_len, "pbk_digest");
#endif
	}
sb_rom_img_pbk_hash_vrf_f_end:
	return pbk_vrf_ret;
}

int sb_rom_img_sign_vrf_f(sec_boot_info_t *p_sb_info)
{
	uint8_t vrf_alg = 0x0;
	uint8_t *p_vrf_msg = NULL, *p_pbk = NULL, *p_sign = NULL;
	uint32_t vrf_msg_len = 0;
	int img_vrf_ret = -1;

	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_rom_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_pbk) == NULL) {
		DBG_BOOT_ERR("p_pbk is NULL\r\n");
		goto sb_rom_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_sign) == NULL) {
		DBG_BOOT_ERR("p_sign is NULL\r\n");
		goto sb_rom_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_msg) == NULL) {
		dbg_printf("p_msg is NULL\r\n");
		goto sb_rom_img_sign_vrf_f_end;
	} else {
		if ((p_sb_info->img_sign_vrf_info.msglen) == 0) {
			DBG_BOOT_ERR("msglen is 0\r\n");
			goto sb_rom_img_sign_vrf_f_end;
		}
	}
	vrf_alg = p_sb_info->img_sign_vrf_info.vrf_alg;
	p_pbk = p_sb_info->img_sign_vrf_info.p_pbk;
	p_sign = p_sb_info->img_sign_vrf_info.p_sign;
	p_vrf_msg = p_sb_info->img_sign_vrf_info.p_msg;
	vrf_msg_len = p_sb_info->img_sign_vrf_info.msglen;
	switch (vrf_alg) {
	case IMG_SIGN_VRF_ALG_EDDSA_ED25519:
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		img_vrf_ret = SUCCESS;
#else
		img_vrf_ret = hal_rtl_eddsa_init(&sb_rom_eddsa_adtr, &sb_rom_crypto_adtr, EDDSA_HASH_CRYPTO_HW_SEL_EN);
		if (SUCCESS != img_vrf_ret) {
			DBG_BOOT_ERR("hal_rtl_eddsa_init fail,%d\r\n", img_vrf_ret);
		} else {
			img_vrf_ret = hal_rtl_eddsa_sign_verify(&sb_rom_eddsa_adtr, p_sign, p_vrf_msg, p_pbk, vrf_msg_len,
													EDDSA_FLOW_AUTOFLOW, ENABLE);
		}
		hal_rtl_eddsa_deinit(&sb_rom_eddsa_adtr);
#endif
		break;
	default:
		break;
	}
sb_rom_img_sign_vrf_f_end:
	if (img_vrf_ret == SUCCESS) {
		dbg_printf("[vrf sig pass]\r\n");
	} else {
		DBG_BOOT_ERR("[vrf sig fail,%d]\r\n", img_vrf_ret);
#if FLASH_FW_LOAD_DBG
		__mem_dump(p_pbk, 64, "p_pbk");
		__mem_dump(p_sign, 64, "p_sign");
		__mem_dump(p_vrf_msg, vrf_msg_len, "p_vrf_msg");
#endif
	}
	return img_vrf_ret;
}

int sb_rom_all_img_hash_chk_f(sec_boot_info_t *p_sb_info)
{
	int img_vrf_ret = -1;
	int ret = SUCCESS;
	hal_status_t hkdf_ret = HAL_OK;
	uint8_t hash_alg = IMG_HSH_CHK_ALG_SHA256;
	uint8_t *p_img_digest_chk = NULL, *p_msg = NULL, *p_digest = NULL, *p_hshks = NULL, *p_hshkn = NULL, *p_hmac_key = NULL;
	uint32_t msglen = 0, digest_len = 0, key_len = 0, hash_size = 0, auth_type = 0x0, sk_cfg = 0x0;
	uint8_t huk_idx = 0, extract_sk_idx, extract_wb_idx, expand_sk_idx, expand_wb_idx;
	uint32_t msglen_cnt = 0, msg_offset = 0;

	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_rom_all_img_hash_chk_f_end;
	}
	if ((p_sb_info->img_hash_chk_info.p_digest) == NULL) {
		DBG_BOOT_ERR("p_digest is NULL\r\n");
		goto sb_rom_all_img_hash_chk_f_end;
	}
	if ((p_sb_info->img_hash_chk_info.p_msg) == NULL) {
		DBG_BOOT_ERR("p_msg is NULL\r\n");
		goto sb_rom_all_img_hash_chk_f_end;
	} else {
		if ((p_sb_info->img_hash_chk_info.msglen) == 0) {
			DBG_BOOT_ERR("msglen is 0\r\n");
			goto sb_rom_all_img_hash_chk_f_end;
		}
	}
	p_msg = p_sb_info->img_hash_chk_info.p_msg;
	msglen = p_sb_info->img_hash_chk_info.msglen;
	p_digest = &sb_digest_buf[0];
	_memset(p_digest, 0x0, IMG_HASH_CHK_DIGEST_MAX_SIZE);
	p_img_digest_chk = p_sb_info->img_hash_chk_info.p_digest;
	digest_len = 32;

	hash_alg = p_sb_info->img_hash_chk_info.hash_alg;
	p_hshks  = p_sb_info->img_hash_chk_info.p_key_salt;
	p_hshkn  = p_sb_info->img_hash_chk_info.p_key_nonce;
	huk_idx  = p_sb_info->psb_keycerti->huk_idx;

	msglen_cnt = msglen;
	msg_offset = 0;
	switch (hash_alg) {
	case IMG_HSH_CHK_ALG_SHA256:
		auth_type  = AUTH_TYPE_SHA2_256_ALL;
		p_hmac_key = NULL;
		key_len    = 0;
		break;
	case IMG_HSH_CHK_ALG_HMAC_SHA256:
		auth_type = AUTH_TYPE_HMAC_SHA2_256_ALL;
		// Set key stg idx
		if (HUK_IDX1 == huk_idx) {
			extract_sk_idx = KEY_STG_HUK1;
		} else if (HUK_IDX2 == huk_idx) {
			extract_sk_idx = KEY_STG_HUK2;
		} else {
			extract_sk_idx = KEY_STG_HUK1;
		}
		extract_wb_idx = KEY_STG_IDX1;
		expand_sk_idx  = extract_wb_idx;
		expand_wb_idx  = KEY_STG_IDX2;

		// HMAC SHA256 SK FLOW
		// HKDF Flow wb into key storage return key idx
#if 0
		_memset(&hmac_key_tmp[0], 0x0, 32);
		hal_rtl_hkdf_hmac_init(&sb_rom_hkdf_adtr, &sb_rom_crypto_adtr, HKDF_CRYPTO_HW_SEL_EN, HKDF_HMAC_SHA256);
		hal_rtl_hkdf_extract_secure_set_cfg(&sb_rom_hkdf_adtr, KEY_STG_SKTYPE_LD_SK, extract_sk_idx,
											KEY_STG_WBTYPE_WB_ONLY_STG, extract_wb_idx);
		hal_rtl_hkdf_extract_secure(&sb_rom_hkdf_adtr, NULL, p_hshks, SB_HKDF_KEY_SALT_SIZE, NULL);

		hal_rtl_hkdf_expand_secure_set_cfg(&sb_rom_hkdf_adtr, KEY_STG_SKTYPE_LD_SK, expand_sk_idx,
										   KEY_STG_WBTYPE_WB_STG_BUF, expand_wb_idx);
		hal_rtl_hkdf_expand_secure(&sb_rom_hkdf_adtr, NULL, p_hshkn, SB_HKDF_KEY_NONCE_SIZE, &hmac_key_tmp[0], SB_HKDF_HMAC_SHA256_SIZE);
		p_hmac_key = &hmac_key_tmp[0];
		key_len    = SB_HKDF_HMAC_SHA256_SIZE;
#if FLASH_FW_LOAD_DBG
		__mem_dump(p_hshks, SB_HKDF_KEY_SALT_SIZE, "p_hshks");
		__mem_dump(p_hshkn, SB_HKDF_KEY_NONCE_SIZE, "p_hshkn");
		__mem_dump(p_hmac_key, digest_len, "hmac key");
#endif
#endif
		hkdf_ret = hal_rtl_hkdf_hmac_sha256_secure_init(&sb_rom_hkdf_adtr, &sb_rom_crypto_adtr, HKDF_CRYPTO_HW_SEL_EN);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac init fail,0x%x\r\n", hkdf_ret);
			goto sb_rom_all_img_hash_chk_f_end;
		}

		hkdf_ret = hal_rtl_hkdf_extract_secure_all(&sb_rom_hkdf_adtr, extract_sk_idx, extract_wb_idx, p_hshks);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac extract fail,0x%x\r\n", hkdf_ret);
			goto sb_rom_all_img_hash_chk_f_end;
		}
		hkdf_ret = hal_rtl_hkdf_expand_secure_all(&sb_rom_hkdf_adtr, expand_sk_idx, expand_wb_idx, p_hshkn);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac expand fail,0x%x\r\n", hkdf_ret);
			goto sb_rom_all_img_hash_chk_f_end;
		}
		sk_cfg = hal_rtl_crypto_get_sk_cfg(&sb_rom_crypto_adtr, KEY_STG_SKTYPE_LD_SK, expand_wb_idx, KEY_STG_WBTYPE_WB_ONLY_BUF, KEY_STG_SK_IDX_NONE);
		p_hmac_key = NULL;
		break;
	default:
		auth_type  = AUTH_TYPE_SHA2_256_ALL;
		p_hmac_key = NULL;
		key_len    = 0;
		break;

	}
	if (sk_cfg != 0x0) {
		ret = hal_rtl_crypto_auth_sk_init(&sb_rom_crypto_adtr, auth_type, p_hmac_key, sk_cfg);
	} else {
		ret = hal_rtl_crypto_auth_init(&sb_rom_crypto_adtr, auth_type, p_hmac_key, key_len);
	}
	if (ret != SUCCESS) {
		DBG_BOOT_ERR("img hash init,%d\r\n", ret);
	}
	while (msglen_cnt > 0) {
		if (msglen_cnt >= SB_CRYPTO_MAX_MSG_LENGTH) {
			hash_size = SB_CRYPTO_MAX_MSG_LENGTH;
		} else {
			hash_size = msglen_cnt;
		}
		ret = hal_rtl_crypto_auth_update(&sb_rom_crypto_adtr, auth_type, (p_msg + msg_offset), hash_size);
		if (ret != SUCCESS) {
			DBG_BOOT_ERR("img hash update fail,%d\r\n", ret);
		}
		msglen_cnt -= hash_size;
		msg_offset += hash_size;
	}
	if (sk_cfg != 0x0) {
		ret = hal_rtl_crypto_auth_sk_final(&sb_rom_crypto_adtr, auth_type, p_digest);
	} else {
		ret = hal_rtl_crypto_auth_final(&sb_rom_crypto_adtr, auth_type, p_digest);
		if (ret != SUCCESS) {
			DBG_BOOT_ERR("img hash final fail,%d\r\n", ret);
		}
	}
	if (_memcmp(p_img_digest_chk, p_digest, digest_len) == 0) {
		img_vrf_ret = SUCCESS;
		dbg_printf("[img hash chk pass]\r\n");
	} else {
		img_vrf_ret = -2;
		DBG_BOOT_ERR("[img hash chk fail]\r\n");
#if FLASH_FW_LOAD_DBG
		DBG_BOOT_ERR("img msglen = %d\r\n", msglen);
		__mem_dump(p_img_digest_chk, digest_len, "img_digest_chk");
		__mem_dump(p_digest, digest_len, "img_digest");
#endif
	}
sb_rom_all_img_hash_chk_f_end:
	return img_vrf_ret;
}

int sb_rom_img_vrf_op(const uint8_t sbl_cfg, sec_boot_info_t *p_sb_info, const uint8_t info_type)
{
	int ret = -1;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, img_hsh_en = DISABLE;
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 0);
	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 1);
	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 2);
	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 3);
	if (ENABLE == tb_en) {  // trusted boot flow
		// pbk hash chk
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 0);
		ret = sb_rom_img_pbk_hash_vrf_f(p_sb_info, info_type);
		if (ret != SUCCESS) {
			return ret;
		}

		// verify sign img
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 1);
		ret = sb_rom_img_sign_vrf_f(p_sb_info);
		if (ret != SUCCESS) {
			return ret;
		}

		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 2);
		if (KEY_CERTI_INFO != info_type) {
			// all img hash check
			ret = sb_rom_all_img_hash_chk_f(p_sb_info);
			if (ret != SUCCESS) {
				return ret;
			}
		}
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 3);
	} else {     // except key certificate need to sign vrf, only img hash check flow for others IMG
		if (KEY_CERTI_INFO == info_type) {
			// pbk hash chk
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 0);
			ret = sb_rom_img_pbk_hash_vrf_f(p_sb_info, info_type);
			if (ret != SUCCESS) {
				return ret;
			}

			// verify sign img
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 1);
			ret = sb_rom_img_sign_vrf_f(p_sb_info);
			if (ret != SUCCESS) {
				return ret;
			}
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 2);
		} else {
			// all img hash check
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 2);
			ret = sb_rom_all_img_hash_chk_f(p_sb_info);
			if (ret != SUCCESS) {
				return ret;
			}
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 3);
		}
	}
	return ret;
}


certi_tbl_t *fw_load_certi_tbl_f(uint8_t *pCerti_tbl_flh)
{
	img_keycerti_tbl_t *certi_tbl = (pimg_keycerti_tbl_t)pCerti_tbl_flh;
	uint8_t *precord;
	uint8_t record_idx;

	rtl_dcache_invalidate_by_addr(certi_tbl, sizeof(img_keycerti_tbl_t));
	_memset((void *)&certi_rec_tbl, 0x0, sizeof(certi_tbl_t));
	precord = (uint8_t *)(&certi_tbl->key_cer_tbl_rec[0]);
	for (record_idx = 0; record_idx < MAX_CERTI_TBL_RECORD; record_idx++) {
		_memcpy(&certi_rec_tbl.key_cer_tbl_rec[record_idx], precord, sizeof(certi_tbl_record_t));
#if FLASH_FW_LOAD_DBG
		dbg_printf("  #start_addr  : 0x%08x\r\n", (certi_rec_tbl.key_cer_tbl_rec[record_idx].start_addr));
		dbg_printf("  #length : %d(0x%08x)\r\n", (certi_rec_tbl.key_cer_tbl_rec[record_idx].length), (certi_rec_tbl.key_cer_tbl_rec[record_idx].length));
		dbg_printf("  #type_id : 0x%02x\r\n", (certi_rec_tbl.key_cer_tbl_rec[record_idx].type_id));
		dbg_printf("  #valid : 0x%01x\r\n", (certi_rec_tbl.key_cer_tbl_rec[record_idx].valid));
		dbg_printf("\r\n");
#endif
		precord += sizeof(certi_tbl_record_t);
	}

	// Confirm key certi1/2 index
	if (FW_PT_KEY_CER1_ID == (certi_rec_tbl.key_cer_tbl_rec[0].type_id)) {
		certi_rec_tbl.certi1_idx = 0;
		certi_rec_tbl.certi2_idx = 1;
	} else {
		certi_rec_tbl.certi1_idx = 1;
		certi_rec_tbl.certi2_idx = 0;
	}
	return (&certi_rec_tbl);
}

certi_tbl_t *fw_load_certi_tbl_direct_f()
{
	uint8_t *pcerti_tbl_flh = NULL;
	certi_tbl_t *pcerti_rec_tbl = NULL;

	pcerti_tbl_flh = (uint8_t *)(SPI_FLASH_BASE + CERTIFICATE_TBL_FLSH_OFFSET);
	pcerti_rec_tbl = fw_load_certi_tbl_f(pcerti_tbl_flh);

	return (pcerti_rec_tbl);
}

certi_tbl_t *sb_get_key_certi_tbl_rec(const uint8_t *img_offset, sec_boot_info_t *p_sb_info)
{
	img_keycerti_tbl_t *pcerti_tbl = (pimg_keycerti_tbl_t)img_offset;
	certi_tbl_t *pcerti_rec_tbl = NULL;
	uint8_t *pTmp = (uint8_t *)img_offset;

	if (!pTmp) {
		goto sb_get_key_certi_tbl_end;
	}
#if FLASH_FW_LOAD_DBG
	dbg_printf("[--- Key Certificate TBL HDR ---]\r\n");
	dbg_printf("  #type_id : 0x%04x\r\n", certi_tbl->type_id);
	dbg_printf("  #vrf_al : 0x%04x\r\n", certi_tbl->vrf_al);
#endif
	//dbg_printf("  #vrf_al : 0x%04x\r\n", certi_tbl->vrf_al);

	//p_sb_info->img_sign_vrf_info.vrf_alg = (certi_tbl->vrf_al);   // Todo
	p_sb_info->img_sign_vrf_info.vrf_alg = IMG_SIGN_VRF_ALG_EDDSA_ED25519;  //tmp sw workaround
	pcerti_rec_tbl = fw_load_certi_tbl_f(img_offset);

sb_get_key_certi_tbl_end:
	return pcerti_rec_tbl;
}


int verify_certificateInfo_f(const uint8_t *ptr, sec_boot_info_t *p_sb_info, sec_boot_keycerti_t *psb_keycerti)
{
	int ret = SUCCESS;
	img_keycerti_t *key_certi = (pimg_keycerti_t)ptr;
	key_certi_hsh_tbl_t *pkey_hsh_tbl = NULL;
	key_certi_pkhsh_t   *ppkkey_hsh = NULL;
	uint8_t *precord = NULL;
	uint8_t record_idx, i;

	if (!ptr) {
		ret = FAIL;
		goto verify_certificateInfo_f_end;
	}
#if FLASH_FW_LOAD_DBG
	dbg_printf("[Dump Key Certificate Info]:\r\n");
	dbg_printf("  #rotpk_hsh_idx : 0x%0x\r\n", key_certi->rotpk_hsh_idx);
	dbg_printf("  #huk_idx : 0x%0x\r\n", key_certi->huk_idx);
	dbg_printf("  #sec_key_idx : 0x%0x\r\n", key_certi->sec_key_idx);
	__mem_dump(key_certi->resv1, KEYCERTI_RESV1_SIZE, "#resv1");
	__mem_dump(key_certi->rotpk.data, KEYCERTI_ROTPKDATA_SIZE, "#rotpk");

	dbg_printf("  #type_id : 0x%04x\r\n", key_certi->type_id);
	dbg_printf("  #key_version : 0x%04x\r\n", key_certi->key_version);
	__mem_dump(key_certi->timest, KEYCERTI_TIMST_SIZE, "#timst");
	dbg_printf("  #imgsz : %d(0x%08x)\r\n",  key_certi->imgsz, key_certi->imgsz);
	dbg_printf("  #s_jtag_ctrl: 0x%x\r\n",  key_certi->s_jtag_ctrl);
	__mem_dump(key_certi->resv2, KEYCERTI_RESV2_SIZE, "#resv2");
	dbg_printf("[--- S_jtag_S_Nonfixed_Key_Info ---]\r\n");
	__mem_dump(key_certi->jtag_s_nonfixed_key_info.kn, KEYCERTI_SJTAG_KEY_NONCE_SIZE, "#key_nonce");
	__mem_dump(key_certi->jtag_s_nonfixed_key_info.ks, KEYCERTI_SJTAG_KEY_SALT_SIZE, "#key_salt");
	dbg_printf("[--- S_jtag_NS_Nonfixed_Key_Info ---]\r\n");
	__mem_dump(key_certi->jtag_ns_nonfixed_key_info.kn, KEYCERTI_SJTAG_KEY_NONCE_SIZE, "#key_nonce");
	__mem_dump(key_certi->jtag_ns_nonfixed_key_info.ks, KEYCERTI_SJTAG_KEY_SALT_SIZE, "#key_salt");

	dbg_printf("  #key_hash_rec_num : 0x%0x\r\n", key_certi->key_hash_rec_num);
	__mem_dump(key_certi->resv3, KEYCERTI_RESV3_SIZE, "#resv3");
#endif

	psb_keycerti->rotpk_hsh_idx = key_certi->rotpk_hsh_idx;
	psb_keycerti->huk_idx = key_certi->huk_idx;
	psb_keycerti->sec_key_idx = key_certi->sec_key_idx;
	psb_keycerti->s_jtag_ctrl = key_certi->s_jtag_ctrl;
	_memcpy(psb_keycerti->jtag_s_nonfixed_key_info.kn, key_certi->jtag_s_nonfixed_key_info.kn, KEYCERTI_SJTAG_KEY_NONCE_SIZE);
	_memcpy(psb_keycerti->jtag_s_nonfixed_key_info.ks, key_certi->jtag_s_nonfixed_key_info.ks, KEYCERTI_SJTAG_KEY_SALT_SIZE);
	_memcpy(psb_keycerti->jtag_ns_nonfixed_key_info.kn, key_certi->jtag_ns_nonfixed_key_info.kn, KEYCERTI_SJTAG_KEY_NONCE_SIZE);
	_memcpy(psb_keycerti->jtag_ns_nonfixed_key_info.ks, key_certi->jtag_ns_nonfixed_key_info.ks, KEYCERTI_SJTAG_KEY_SALT_SIZE);
	psb_keycerti->key_hash_rec_num = key_certi->key_hash_rec_num;

	precord = (uint8_t *)key_certi->key_hsh_tbl;
	for (record_idx = 0; record_idx < KEYCERTI_TBL_HDR_RESV_SIZE; record_idx++) {
		pkey_hsh_tbl = (pkey_certi_hsh_tbl_t)precord;
		_memcpy(&(psb_keycerti->img_pkhsh_info[record_idx].key_hsh_tbl), pkey_hsh_tbl, sizeof(key_certi_hsh_tbl_t));
#if FLASH_FW_LOAD_DBG
		dbg_printf("  #psb_keycerti type_id : 0x%02x\r\n", (psb_keycerti->img_pkhsh_info[record_idx].key_hsh_tbl.type_id));
		__mem_dump(pkey_hsh_tbl->resv, KEYCERTI_HSH_TBL_RESV_SIZE, "#resv");
		dbg_printf("\r\n");
#endif
		precord += sizeof(key_certi_hsh_tbl_t);
	}

	precord = (uint8_t *)key_certi->key_pkhsh;
	for (record_idx = 0; record_idx < KEYCERTI_TBL_HDR_RESV_SIZE; record_idx++) {
		ppkkey_hsh = (pkey_certi_pkhsh_t)precord;
		_memcpy(&(psb_keycerti->img_pkhsh_info[record_idx].key_pkhsh.data), ppkkey_hsh->data, sizeof(key_certi_pkhsh_t));
#if FLASH_FW_LOAD_DBG
		__mem_dump(&(psb_keycerti->img_pkhsh_info[record_idx].key_pkhsh.data), KEYCERTI_PKHASH_SIZE, "#psb_keycerti hash");
		dbg_printf("\r\n");
#endif
		precord += sizeof(key_certi_pkhsh_t);
	}
	p_sb_info->psb_keycerti = psb_keycerti;

#if FLASH_FW_LOAD_DBG
	dbg_printf("[--- Key Certificate Signature ---]\r\n");
	__mem_dump(key_certi->signature.data, KEYCERTI_ROTPKDATA_SIZE, "#signature");
#endif
	//vrf_alg set at sb_get_key_certi_tbl_rec()
	p_sb_info->img_sign_vrf_info.p_pbk = &(key_certi->rotpk.data[0]);
	p_sb_info->img_sign_vrf_info.p_sign = &(key_certi->signature.data[0]);
	p_sb_info->img_sign_vrf_info.p_msg = (uint8_t *)key_certi;
	p_sb_info->img_sign_vrf_info.msglen = (sizeof(img_keycerti_t) - sizeof(key_certi_sign_t)); // Todo (key_certi->imgsz)
verify_certificateInfo_f_end:
	return ret;
}

int fw_load_key_certi_f(const uint8_t sbl_cfg, certi_tbl_t *pcerti_tbl, sec_boot_info_t *p_sb_info, sec_boot_keycerti_t *psb_keycerti)
{
	int ret = -1;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE, img_obj = 0x0;
	uint8_t *pcerti_ld_flh = NULL;
	certi_tbl_record_t *pCerti_record = NULL;
	int ld_certi_idx = 0;

	// Get secure boot level config
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;
	sb_en      = sbl.bit.sb_en;

	// Select the latest valid key certificate to load
	// From ker_version & timst to decide which key certificate should be loaded
	img_obj = LD_SEL_IMG_KEYCERTI;
	ld_certi_idx = img_sel_op_idx(pcerti_tbl, img_obj, IMG_SEL_LD);
	if (((-1) == ld_certi_idx) || (ld_certi_idx >= MAX_CERTI_TBL_RECORD)) {
		DBG_BOOT_ERR("ld_certi_idx err:%d\r\n", ld_certi_idx);
		ret = FAIL;
		return ret;
	}
	// Get Key certificate from certi record
	pCerti_record = (certi_tbl_record_t *)(&pcerti_tbl->key_cer_tbl_rec[ld_certi_idx]);
	if (((pCerti_record->type_id) == FW_PT_KEY_CER1_ID) ||
		((pCerti_record->type_id) == FW_PT_KEY_CER2_ID)) {
		if ((pCerti_record->valid) == INFO_VALID) {
			pcerti_ld_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pCerti_record->start_addr));
		} else {
			ret = FAIL;
			return ret;
		}
	} else {
		ret = FAIL;
		return ret;
	}

#if FLASH_FW_LOAD_DBG
	dbg_printf("pcerti_ld_flh = 0x%x\r\n", pcerti_ld_flh);
#endif
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 14);
	ret = verify_certificateInfo_f(pcerti_ld_flh, p_sb_info, psb_keycerti);
	if (ret != SUCCESS) {
		DBG_BOOT_ERR("verify_certificateInfo_f fail\r\n");
		goto fw_load_key_certi_f_end;
	}

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 15);
	if ((ENABLE == tb_en) || (ENABLE == sb_en) || (ENABLE == img_hsh_en)) {
		ret = sb_rom_img_vrf_op(sbl_cfg, p_sb_info, KEY_CERTI_INFO);
	}
fw_load_key_certi_f_end:
	return ret;
}

int img_get_ld_sel_info_from_ie(const uint8_t img_obj, const uint8_t *ptr, img_manifest_ld_sel_t *pld_sel_info)
{
	int ret = SUCCESS;
	uint8_t *pTmp = (uint8_t *)ptr;
	uint8_t *p_mani_ie = NULL;
	manif_sec_enc_record_t *p_sec_enc_record = NULL;
	uint8_t i = 0, padding_size = 0, find_ie_id = 0x0;
	uint32_t ie_total_size = 0;
	uint8_t find_ie = 0;

	if (LD_SEL_IMG_KEYCERTI == img_obj) {   // Key certificate
		rtl_dcache_invalidate_by_addr(pTmp, sizeof(img_keycerti_t));
		pTmp += (KEYCERTI_HDR_SIZE + KEYCERTI_ROTPKDATA_SIZE);    // jump over key certificate hdr + rotpk
		_memset(pld_sel_info, 0xff, sizeof(img_manifest_ld_sel_t));
		_memcpy(&(pld_sel_info->type_id), pTmp, KEYCERTI_TYPEID_SIZE);
		_memcpy(&(pld_sel_info->version[0]), (pTmp + KEYCERTI_TYPEID_SIZE), KEYCERTI_VERSION_SIZE);
		_memcpy(&(pld_sel_info->timest[0]), (pTmp + (KEYCERTI_TYPEID_SIZE + KEYCERTI_VERSION_SIZE)), KEYCERTI_TIMST_SIZE);
	} else {    // Boot & FW IMG
		rtl_dcache_invalidate_by_addr(pTmp, sizeof(img_manifest_t));
		pTmp += sizeof(manif_hdr_t);    // jump over manifest hdr
		pTmp += (MAX_SEC_ENC_RECORD * sizeof(manif_sec_enc_record_t));  // jump over sec_enc_record content
		_memset(pld_sel_info, 0xff, sizeof(img_manifest_ld_sel_t));
		// memcpy from flash to internal SRAM
		// validate the SRAM buffer not flash!!
		p_mani_ie = &mani_data[0];
		_memcpy((void *)p_mani_ie, (void *)pTmp, MANIFEST_IEDATA_SIZE);

		// add into IE cache table for later use
		while (ie_total_size < MANIFEST_IEDATA_SIZE) {
			uint32_t ie_size = 0;
			ie_size = get_3byte(p_mani_ie + IE_TLV_TYPE_ID_SIZE);
			if (IE_TYPEID_ID == get_1byte(p_mani_ie)) {
				_memcpy(&(pld_sel_info->type_id), (p_mani_ie + IE_TLV_TL_TOTAL_SIZE), ie_size);
				find_ie++;
			} else if (IE_VERSION_ID == get_1byte(p_mani_ie)) {
				_memcpy(&(pld_sel_info->version[0]), (p_mani_ie + IE_TLV_TL_TOTAL_SIZE), ie_size);
				find_ie++;
			} else if (IE_TIMST_ID == get_1byte(p_mani_ie)) {
				_memcpy(&(pld_sel_info->timest[0]), (p_mani_ie + IE_TLV_TL_TOTAL_SIZE), ie_size);
				find_ie++;
			}
			padding_size = (MANIFEST_IE_ALIGNED_SIZE - ((ie_size) % MANIFEST_IE_ALIGNED_SIZE));
			if (padding_size < MANIFEST_IE_ALIGNED_SIZE) {
				// padding condition
				ie_size = ((uint32_t)IE_TLV_TL_TOTAL_SIZE + (ie_size) + padding_size);
			} else {
				// without padding
				ie_size = ((uint32_t)IE_TLV_TL_TOTAL_SIZE + (ie_size));
			}
			if (IMG_MANIFEST_LD_SEL_MAX_IE_CNT == find_ie) {
				ret = SUCCESS;
				break;
			}

			p_mani_ie += ie_size;
			ie_total_size += ie_size;
		}
	}
	return ret;
}

uint32_t get_ld_version(const uint8_t img_obj, uint8_t *p_img_version)
{
	uint32_t version = 0x0, byte_offset = 0x0;
	uint8_t bit_offset = 0x0;

	if (LD_SEL_IMG_KEYCERTI == img_obj) {
		byte_offset = KEYCERTI_VERSION_SIZE;
	} else {
		byte_offset = IMG_MANIFEST_IE_VERSION_SIZE;
	}
	while ((version == 0x0) && (byte_offset != 0x0)) {
		for (bit_offset = 8; bit_offset >= 1; bit_offset--) {
			if (0x0 == (*(p_img_version + (byte_offset - 1)) & (0x1 << (bit_offset - 1)))) {
				//dbg_printf("p_img_version=0x%x\r\n", *(p_img_version + (byte_offset - 1)));
				version = (((byte_offset - 1) * 8) + bit_offset);
				//dbg_printf("version = %d,byte_offset=%d, bit_offset=%d\r\n", version, byte_offset, bit_offset);
				break;
			}
		}
		byte_offset--;
	}
	return version;
}

uint32_t get_ld_timst(uint8_t *p_img_timest)
{
	uint32_t timest = 0x0;
	timest = get_4byte(p_img_timest);
	return timest;
}

int img_get_ld_sel_idx(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx, uint8_t img2_idx)
{
	int img_ld_idx = -1;
	uint8_t img1_valid = DISABLE, img2_valid = DISABLE;
	uint32_t img1_ver = 0x0, img2_ver = 0x0, img1_timst = 0x0, img2_timst = 0x0;
	uint16_t img_type_id = FW_IMG_NOTSET_ID;

	// Confirm IMG OBJ type ID
	if (LD_SEL_IMG_FW == img_obj) {
		img_type_id = FW_IMG_FWHS_S_ID;
	} else if (LD_SEL_IMG_BOOT == img_obj) {
		img_type_id = FW_IMG_BL_ID;
	} else if (LD_SEL_IMG_KEYCERTI == img_obj) {
		img_type_id = FW_IMG_KEY_CER_ID;
	} else {
		img_ld_idx = -1;
		return img_ld_idx;
	}

	if (img_type_id == (pld_sel_info1->type_id)) {
		img1_valid = ENABLE;
	}
	if (img_type_id == (pld_sel_info2->type_id)) {
		img2_valid = ENABLE;
	}
	img1_ver = get_ld_version(img_obj, &(pld_sel_info1->version[0]));
	img2_ver = get_ld_version(img_obj, &(pld_sel_info2->version[0]));
	img1_timst = get_ld_timst(&(pld_sel_info1->timest[0]));
	img2_timst = get_ld_timst(&(pld_sel_info2->timest[0]));
#if FLASH_FW_LOAD_DBG
	dbg_printf("img1_valid=%d,img2_valid=%d\r\n", img1_valid, img2_valid);
	dbg_printf("img1_ver=%d,img2_ver=%d\r\n", img1_ver, img2_ver);
	dbg_printf("img1_timst=0x%x,img2_timst=0x%x\r\n", img1_timst, img2_timst);
#endif
	img_ld_idx = -1;
	if (img1_valid && img2_valid) {

		if (img2_ver > img1_ver) {
			img_ld_idx = img2_idx;
		} else if (img2_ver < img1_ver) {
			img_ld_idx = img1_idx;
		} else {

			if (img2_timst >= img1_timst) {
				img_ld_idx = img2_idx;
			} else {
				img_ld_idx = img1_idx;
			}
		}
	} else if (img1_valid && !img2_valid) {
		img_ld_idx = img1_idx;
	} else if (!img1_valid && img2_valid) {
		img_ld_idx = img2_idx;
	}
	//dbg_printf("img_ld_idx=%d\r\n", img_ld_idx);
	return img_ld_idx;
}

int img_get_update_sel_idx(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx,
						   uint8_t img2_idx)
{
	int img_update_idx = -1;
	uint8_t img1_valid = DISABLE, img2_valid = DISABLE;
	uint32_t img1_ver = 0x0, img2_ver = 0x0, img1_timst = 0x0, img2_timst = 0x0;
	uint16_t img_type_id = FW_IMG_NOTSET_ID;

	// Confirm IMG OBJ type ID
	if (LD_SEL_IMG_FW == img_obj) {
		img_type_id = FW_IMG_FWHS_S_ID;
	} else if (LD_SEL_IMG_BOOT == img_obj) {
		img_type_id = FW_IMG_BL_ID;
	} else if (LD_SEL_IMG_KEYCERTI == img_obj) {
		img_type_id = FW_IMG_KEY_CER_ID;
	} else {
		img_update_idx = -1;
		return img_update_idx;
	}

	if (img_type_id == (pld_sel_info1->type_id)) {
		img1_valid = ENABLE;
	}
	if (img_type_id == (pld_sel_info2->type_id)) {
		img2_valid = ENABLE;
	}
	img1_ver = get_ld_version(img_obj, &(pld_sel_info1->version[0]));
	img2_ver = get_ld_version(img_obj, &(pld_sel_info2->version[0]));
	img1_timst = get_ld_timst(&(pld_sel_info1->timest[0]));
	img2_timst = get_ld_timst(&(pld_sel_info2->timest[0]));
	img_update_idx = -1;
	if (img1_valid && img2_valid) {

		if (img2_ver > img1_ver) {
			img_update_idx = img1_idx;
		} else if (img2_ver < img1_ver) {
			img_update_idx = img2_idx;
		} else {

			if (img2_timst >= img1_timst) {
				img_update_idx = img1_idx;
			} else {
				img_update_idx = img2_idx;
			}
		}
	} else if (img1_valid && !img2_valid) {
		img_update_idx = img1_idx;
	} else if (!img1_valid && img2_valid) {
		img_update_idx = img2_idx;
	}
	return img_update_idx;
}

int confirm_manif_hdr_f(const uint8_t *ptr, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	manif_hdr_t *pmani_hdr = (pmanif_hdr_t)ptr;
	char lbl_name[MANIFEST_MAX_LABEL_SIZE + 1] = "";
	if (!ptr) {
		ret = FAIL;
		goto confirm_manif_hdr_f_end;
	}

	/*
	 * Parse manifest HDR
	 *            | label  | size   | vrf_al | resv |
	 * -----------+--------+--------+--------+------+
	 * size(Bytes)|  8     | 2      | 2      | 4    |
	 * ----------------------------------------------
	 */
	_memset(lbl_name, '\0', MANIFEST_MAX_LABEL_SIZE + 1);
	_strncpy(lbl_name, (void *)(pmani_hdr->lbl), MANIFEST_MAX_LABEL_SIZE);
	// label confirm
#if FLASH_FW_LOAD_DBG
	dbg_printf("  #label : %s\r\n", lbl_name);
	__mem_dump(lbl_name, MANIFEST_MAX_LABEL_SIZE, "label_hex");
#endif
	if (chk_manifest_lbl(lbl_name)) {
		// Invalid Label
		ret = FAIL;
	} else {
#if FLASH_FW_LOAD_DBG
		// Valid Label
		dbg_printf("  #size  : %d\r\n", pmani_hdr->size);
		dbg_printf("  #vrf_al: 0x%04x\r\n", pmani_hdr->vrf_al);
		dbg_printf("  #enc_rmp_base_addr: 0x%08x\r\n", pmani_hdr->enc_rmp_base_addr);
		__mem_dump(pmani_hdr->resv2, MANIFEST_HDR_RESERV2_SIZE, "#resv2");
#endif
		//p_sb_info->img_sign_vrf_info.vrf_alg = (pmani_hdr->vrf_al);   // Todo
		p_sb_info->img_sign_vrf_info.vrf_alg = IMG_SIGN_VRF_ALG_EDDSA_ED25519; //tmp sw workaround
		p_sb_info->img_sign_vrf_info.p_msg = ((uint8_t *)ptr);
		p_sb_info->img_sign_vrf_info.msglen = ((pmani_hdr->size));
		_memset(&sec_enc_info, 0xff, sizeof(sb_sec_enc_info_t));
		sec_enc_info.enc_rmp_base_addr = (pmani_hdr->enc_rmp_base_addr);
		p_sb_info->pimg_sec_enc_info = &sec_enc_info;

		ret = SUCCESS;
	}
confirm_manif_hdr_f_end:
	return ret;
}

int confirm_manif_ie_f(const uint8_t *ptr, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	uint8_t *pTmp = (uint8_t *)ptr;
	uint8_t *p_mani_ie = NULL;
	manif_ie_tlv_t *p_ie_tbl_cache = NULL;
	manif_sec_enc_record_t *p_sec_enc_record = NULL;
	uint8_t i = 0, padding_size = 0, find_ie_id = 0x0;
	uint32_t ie_total_size = 0;
	char *ie_name;

	if (!pTmp) {
		ret = FAIL;
		goto confirm_manif_ie_f_end;
	}

	// sec_enc_record content
#if 0
	_memcpy((void *)&sec_enc_record[0], (void *)pTmp, MAX_SEC_ENC_RECORD * sizeof(manif_sec_enc_record_t));
	p_sec_enc_record = &sec_enc_record[0];
	pTmp += (MAX_SEC_ENC_RECORD * sizeof(manif_sec_enc_record_t));
#endif
	for (i = 0; i < MAX_SEC_ENC_RECORD; i++) {
		p_sec_enc_record = (manif_sec_enc_record_t *)pTmp;
		_memcpy(&(p_sb_info->pimg_sec_enc_info->sec_enc_record[i]), p_sec_enc_record, sizeof(manif_sec_enc_record_t));
		p_sec_enc_record = &(p_sb_info->pimg_sec_enc_info->sec_enc_record[i]);
		p_sb_info->pimg_sec_enc_info->enc_sts &= (~(ENABLE << i) & 0xFF);   // clear enc_sts related bit
		p_sb_info->pimg_sec_enc_info->enc_sts |= (((p_sec_enc_record->enc_en) & ENABLE) << i);  // set enc_sts related bit
		pTmp += sizeof(manif_sec_enc_record_t);

#if FLASH_FW_LOAD_DBG
		dbg_printf("sec_enc%d_record\r\n", i);
		dbg_printf("  #enc_en  : 0x%x\r\n", p_sec_enc_record->enc_en);
		dbg_printf("  #encalg_sel: 0x%x\r\n", p_sec_enc_record->encalg_sel);
		dbg_printf("  #add_ref: 0x%x\r\n", p_sec_enc_record->add_ref);
		dbg_printf("  #xip_en: 0x%x\r\n", p_sec_enc_record->xip_en);
		dbg_printf("  #tag_base_addr: 0x%08x\r\n", p_sec_enc_record->tag_base_addr);
		dbg_printf("  #iv_ptn_low: 0x%08x\r\n", p_sec_enc_record->iv_ptn_low);
		dbg_printf("  #iv_ptn_high: 0x%08x\r\n", p_sec_enc_record->iv_ptn_high);
		dbg_printf("  #base_addr: 0x%08x\r\n", p_sec_enc_record->base_addr);
		dbg_printf("  #end_addr: 0x%08x\r\n", p_sec_enc_record->end_addr);
#endif
	}

	_memset(&ie_tbl_cache[0], 0x0, sizeof(manif_ie_tlv_t)*MANIFEST_IE_TBL_MAX_NUM);
	p_ie_tbl_cache = &ie_tbl_cache[0];
	// memcpy from flash to internal SRAM
	// validate the SRAM buffer not flash!!
	p_mani_ie = &mani_data[0];
	_memcpy((void *)p_mani_ie, (void *)pTmp, MANIFEST_IEDATA_SIZE);

	// add into IE cache table for later use
	i = 0;
	while (ie_total_size < MANIFEST_IEDATA_SIZE) {
		uint32_t ie_size = 0;
		ie_tbl_cache[i].id    = get_1byte(p_mani_ie);
		ie_tbl_cache[i].size  = get_3byte(p_mani_ie + IE_TLV_TYPE_ID_SIZE);
		ie_tbl_cache[i].p_val = (p_mani_ie + IE_TLV_TL_TOTAL_SIZE);
#if FLASH_FW_LOAD_DBG
		dbg_printf("id:0x%08x\r\n", ie_tbl_cache[i].id);
		dbg_printf("size:%d\r\n", ie_tbl_cache[i].size);
		dbg_printf("p_val=0x%08x\r\n", ie_tbl_cache[i].p_val);
#endif
		padding_size = (MANIFEST_IE_ALIGNED_SIZE - ((ie_tbl_cache[i].size) % MANIFEST_IE_ALIGNED_SIZE));
		if (padding_size < MANIFEST_IE_ALIGNED_SIZE) {
			// padding condition
			ie_size = ((uint32_t)IE_TLV_TL_TOTAL_SIZE + (ie_tbl_cache[i].size) + padding_size);

#if FLASH_FW_LOAD_DBG
			char *ie_name = NULL;
			uint8_t j = 0;
			// Debug dump padding ie name
			for (j = 0; j < MANIFEST_IE_ID_MAX_NUM; j++) {
				if (ie_tbl_cache[i].id == ie_tlv_const_tbl[j].ie_id) {
					ie_name = ie_tlv_const_tbl[j].ie_name;
					break;
				}
			}
			if (j == MANIFEST_IE_ID_MAX_NUM) {
				ie_name = ie_tlv_const_tbl[(j - 1)].ie_name;
			}
#endif
		} else {
			// without padding
			ie_size = ((uint32_t)IE_TLV_TL_TOTAL_SIZE + (ie_tbl_cache[i].size));
		}

		p_mani_ie += ie_size;
		ie_total_size += ie_size;
		i++;
#if FLASH_FW_LOAD_DBG
		dbg_printf("update pTmp=0x%08x\r\n", p_mani_ie);
		dbg_printf("i=%d,ie_size=%d\r\n", i, ie_size);
		dbg_printf("ie_total_size=%d\r\n", ie_total_size);
#endif
	}
	if (ie_total_size != MANIFEST_IEDATA_SIZE) {
		DBG_BOOT_ERR("manifest ie_total_size invalid!");
		ret = FAIL;
	} else {
#if FLASH_FW_LOAD_DBG
		dbg_printf("manifest ie_total_size valid!\r\n");
		dbg_printf("Check all IE member:\r\n");

		// Debug dump
		i = 0;
		while (ie_tbl_cache[i].p_val != NULL) {
			uint8_t j = 0;
			// Find ie name
			for (j = 0; j < MANIFEST_IE_ID_MAX_NUM; j++) {
				if (ie_tbl_cache[i].id == ie_tlv_const_tbl[j].ie_id) {
					ie_name = ie_tlv_const_tbl[j].ie_name;
					break;
				}
			}
			if (j == MANIFEST_IE_ID_MAX_NUM) {
				ie_name = ie_tlv_const_tbl[(j - 1)].ie_name;
			}
			dbg_printf(" #IE_Name(%s) #IE_id(0x%x) : size : %d\r\n", ie_name, ie_tbl_cache[i].id, ie_tbl_cache[i].size);
			if (IE_IMGSZ_ID == ie_tbl_cache[i].id) {
				uint32_t imgsz = get_4byte(ie_tbl_cache[i].p_val);
				dbg_printf("  #IE_IMGSZ:(0x%x, %d)\r\n", imgsz, imgsz);
			}
			__mem_dump_str(ie_tbl_cache[i].p_val, ie_tbl_cache[i].size, ie_name);
			i++;
		}
#endif
		ret = SUCCESS;
	}

	// Get public key
	find_ie_id = IE_PK_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

	// Get huk, msglen, hash_alg, hash key salt, key nonce, hash digest
	find_ie_id = IE_IMGSZ_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

	find_ie_id = IE_HSHALG_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

	find_ie_id = IE_HSHKS_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

	find_ie_id = IE_HSHKN_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

	find_ie_id = IE_HASH_ID;
	find_sb_ie_from_tbl_cache(p_sb_info, p_ie_tbl_cache, find_ie_id);

confirm_manif_ie_f_end:
	return ret;
}

int confirm_manif_sign_f(const uint8_t *ptr, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;

	// validate the SRAM buffer not flash!!
	_memcpy((void *) & (manif_sign.data[0]), (void *)ptr, sizeof(manif_sign_t));
	p_sb_info->img_sign_vrf_info.p_sign = (uint8_t *) & (manif_sign.data[0]);
#if FLASH_FW_LOAD_DBG
	dump_for_one_bytes(&(manif_sign.data[0]), sizeof(manif_sign_t));
#endif
	return ret;
}

int verify_manif_f(const uint8_t *img_offset, const uint8_t info_type, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	uint8_t *p_img_hdr = (uint8_t *)img_offset;
	uint32_t cur_offset = 0x0;

	//Confirm Partition Tbl Manifest HDR
	ret = confirm_manif_hdr_f(ptr, p_sb_info);
	if (ret != SUCCESS) {
		goto confirm_manif_f_end;
	}

	ptr += sizeof(manif_hdr_t);
	ret = confirm_manif_ie_f(ptr, p_sb_info);   // included sec_enc_record
	if (ret != SUCCESS) {
		goto confirm_manif_f_end;
	}

	ptr += (MAX_SEC_ENC_RECORD * sizeof(manif_sec_enc_record_t));
	ptr += sizeof(manif_ie_data_t);
	ret = confirm_manif_sign_f(ptr, p_sb_info);
	if (ret != SUCCESS) {
		goto confirm_manif_f_end;
	}

	cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	p_img_hdr += cur_offset;
	p_sb_info->img_hash_chk_info.p_msg = p_img_hdr;
confirm_manif_f_end:
	return ret;
}

partition_tbl_t *fw_load_partition_tbl_f(uint8_t *pPar_tbl_flh)
{
	uint8_t *pPar_tbl_data = NULL;
	uint32_t cur_offset = 0x0;

	cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	pPar_tbl_flh += cur_offset;
	pPar_tbl_data = pPar_tbl_flh;
	rtl_dcache_invalidate_by_addr(pPar_tbl_data, sizeof(partition_tbl_t));
	_memset((void *)&export_part_tbl, 0x0, sizeof(partition_tbl_t));
	// Confirm Partition Tbl FST & records
	_memcpy((void *)&export_part_tbl, (void *)pPar_tbl_data, sizeof(partition_tbl_t));
	rtl_dcache_clean_by_addr((void *)&export_part_tbl, sizeof(partition_tbl_t));
#if FLASH_FW_LOAD_DBG
	dump_for_one_bytes((&export_part_tbl), sizeof(partition_tbl_t));
#endif
	return (&export_part_tbl);
}

partition_tbl_t *fw_load_partition_tbl_direct_f()
{
	uint8_t *pPar_tbl_flh = NULL;
	uint32_t cur_offset = 0x0;
	partition_tbl_t *pPar_tbl_data = NULL;

	pPar_tbl_flh = (uint8_t *)(SPI_FLASH_BASE + PARTITION_TBL_FLSH_OFFSET);
	pPar_tbl_data = fw_load_partition_tbl_f(pPar_tbl_flh);

	return (pPar_tbl_data);
}

hal_status_t sec_enable_remap_region(uint8_t rmp_region_sel,
									 uint32_t va_base, uint32_t pa_base, uint32_t region_size)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_enable_remap_region(&sb_rom_sec_adtr, rmp_region_sel, va_base, pa_base, region_size);
#endif
	return sec_ret;
}

hal_status_t sec_disable_rmp_region(uint8_t rmp_region_sel)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_disable_rmp_region(&sb_rom_sec_adtr, rmp_region_sel);
#endif
	return sec_ret;
}

hal_status_t sec_default_decrypt_init(const uint8_t key_sel)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_default_decrypt_init(&sb_rom_sec_adtr, key_sel);
#endif
	return sec_ret;
}

hal_status_t sec_decrypt_region_init(const uint32_t iv_low_ptn, const uint32_t iv_high_ptn, uint8_t dec_region_sel)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_decrypt_region_init(&sb_rom_sec_adtr, iv_low_ptn, iv_high_ptn, dec_region_sel);
#endif
	return sec_ret;
}

hal_status_t sec_decrypt_region_enable(const uint8_t cipher_sel, uint32_t dec_base, uint32_t dec_size, uint32_t tag_base)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_decrypt_region_enable(&sb_rom_sec_adtr, cipher_sel, dec_base, dec_size, tag_base);
#endif
	return sec_ret;
}

hal_status_t sec_disable_dec_region(uint8_t dec_region_sel)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_disable_dec_region(&sb_rom_sec_adtr, dec_region_sel);
#endif
	return sec_ret;
}

void sec_aes_disable(void)
{
	hal_rtl_flash_sec_aes_disable();
}

uint32_t sec_set_word_from_byte_bigen(const unsigned char *s_value)
{
	uint32_t val;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	val = hal_rtl_flash_sec_set_word_from_byte_bigen(s_value);
#endif
	return val;
}

hal_status_t sec_default_calculate_tag_base(uint32_t flash_addr, uint32_t region_size, uint32_t tag_region_addr,
		uint32_t *tag_base, uint32_t *tag_region_size)
{
	hal_status_t sec_ret = HAL_OK;
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	sec_ret = hal_rtl_flash_sec_default_calculate_tag_base(flash_addr, region_size, tag_region_addr,
			  tag_base, tag_region_size);
#endif
	return sec_ret;
}

int sec_cfg_f_init(hal_sec_region_cfg_t *psb_sec_cfg_pending)
{
	int ret = SUCCESS;
	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}
	psb_sec_cfg_pending->sec_cfg_f.enable_remap_region   = sec_enable_remap_region;
	psb_sec_cfg_pending->sec_cfg_f.disable_rmp_region    = sec_disable_rmp_region;
	psb_sec_cfg_pending->sec_cfg_f.default_decrypt_init  = sec_default_decrypt_init;
	psb_sec_cfg_pending->sec_cfg_f.decrypt_region_init   = sec_decrypt_region_init;
	psb_sec_cfg_pending->sec_cfg_f.decrypt_region_enable = sec_decrypt_region_enable;
	psb_sec_cfg_pending->sec_cfg_f.disable_dec_region    = sec_disable_dec_region;
	psb_sec_cfg_pending->sec_cfg_f.aes_disable           = sec_aes_disable;
	psb_sec_cfg_pending->sec_cfg_f.set_word_from_byte_bigen = sec_set_word_from_byte_bigen;
	psb_sec_cfg_pending->sec_cfg_f.default_calculate_tag_base = sec_default_calculate_tag_base;
	return ret;
}

uint8_t search_available_idx(uint8_t *p_sts, const uint8_t cfg_max_size)
{
	uint8_t idx = 0x0;
	for (idx = 0; idx < cfg_max_size ; idx++) {
		if (DISABLE == (*p_sts & (ENABLE << idx))) {
			break;
		}
	}
	return idx;
}

int xip_pending_cfg_add_rmp(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_rmp_lkp_tbl,
							uint32_t phy_addr, uint32_t remap_addr, uint32_t remap_sz)
{
	uint8_t idx = 0x0;
	hal_sec_rmp_region_cfg_t *prmp_region_cfg = NULL;
	uint8_t *prmp_sts = NULL;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}

	if (pimg_rmp_lkp_tbl == NULL) {
		DBG_MISC_ERR("img_rmp_lkp_tbl is NULL\r\n");
		return FAIL;
	}

	if (psb_sec_cfg_pending->rmp_cfg_cnt >= MAX_PENDING_SEC_RMP_REGION_CFG) {
		DBG_MISC_ERR("boot_load: No free SEC RMP Cfg pending handler\r\n");
		return FAIL;
	}

	// remap idx using search from rmp_sts
	prmp_sts = &(psb_sec_cfg_pending->rmp_sts);

	idx = search_available_idx(prmp_sts, MAX_PENDING_SEC_RMP_REGION_CFG);
	if (MAX_PENDING_SEC_RMP_REGION_CFG <= idx) {
		DBG_MISC_ERR("boot_load: free SEC RMP Cfg search err!\r\n");
		return FAIL;
	}
	prmp_region_cfg = &(psb_sec_cfg_pending->sec_rmp_region_cfg[idx]);
	pimg_rmp_lkp_tbl->rng_idx   = idx;
	prmp_region_cfg->is_used    = ENABLE;
	prmp_region_cfg->phy_addr   = phy_addr;
	prmp_region_cfg->remap_addr = remap_addr;
	prmp_region_cfg->remap_size = remap_sz;

#if FLASH_FW_LOAD_DBG
	dbg_printf("SEC Remap info:\r\n");
	dbg_printf("va_base = 0x%08x, pa_base = 0x%08x, remap_region_size = %d, remap_region_sel = %d\r\n", prmp_region_cfg->remap_addr,
			   prmp_region_cfg->phy_addr,
			   prmp_region_cfg->remap_size,
			   (pimg_rmp_lkp_tbl->rng_idx));
#endif
	(*prmp_sts) |= (ENABLE << idx);
	(psb_sec_cfg_pending->rmp_not_set_sts) |= (ENABLE << idx);
	(psb_sec_cfg_pending->rmp_cfg_cnt)++;

	return SUCCESS;
}

int xip_pending_cfg_add_dec_key(hal_sec_region_cfg_t *psb_sec_cfg_pending, const uint8_t key_id)
{
	int ret = SUCCESS;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}

	uint8_t sec_key_top_sel = 0x0;
	switch (key_id) {
	case SEC_KEY_IDX1:
		sec_key_top_sel = TOP_SEC_KEY1_LD_SEC_KEY_ALL;
		break;
	case SEC_KEY_IDX2:
		sec_key_top_sel = TOP_SEC_KEY2_LD_SEC_KEY_ALL;
		break;
	default:
		sec_key_top_sel = TOP_SEC_KEY1_LD_SEC_KEY_ALL;
		break;
	}
	psb_sec_cfg_pending->key_id = sec_key_top_sel;
	return ret;
}

int xip_pending_cfg_add_dec(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_dec_lkp_tbl,
							uint8_t cipher_sel, uint32_t dec_base, uint32_t dec_sz,
							uint32_t iv_ptn_low, uint32_t iv_ptn_high,
							uint32_t tag_base_addr, uint32_t tag_flh_addr, uint32_t total_hdr_size)
{
	uint32_t idx = 0x0;
	hal_sec_dec_region_cfg_t *pdec_region_cfg = NULL;
	uint8_t *pdec_sts = NULL;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}

	if (pimg_dec_lkp_tbl == NULL) {
		DBG_MISC_ERR("img_dec_lkp_tbl is NULL\r\n");
		return FAIL;
	}

	if ((psb_sec_cfg_pending->dec_cfg_cnt) >= MAX_PENDING_SEC_DEC_REGION_CFG) {
		DBG_MISC_ERR("boot_load: No free SEC DEC Cfg pending handler\r\n");
		return FAIL;
	}

	// decrypt idx using search from dec_sts
	pdec_sts = &(psb_sec_cfg_pending->dec_sts);

	idx = search_available_idx(pdec_sts, MAX_PENDING_SEC_DEC_REGION_CFG);
	if (MAX_PENDING_SEC_DEC_REGION_CFG <= idx) {
		DBG_MISC_ERR("boot_load: free SEC DEC Cfg search err!\r\n");
		return FAIL;
	}

	pdec_region_cfg = &(psb_sec_cfg_pending->sec_dec_region_cfg[idx]);
	pimg_dec_lkp_tbl->rng_idx = idx;
	pdec_region_cfg->is_used     = ENABLE;
	pdec_region_cfg->cipher_sel  = cipher_sel;
	pdec_region_cfg->dec_base    = dec_base;
	pdec_region_cfg->dec_size    = dec_sz;
	pdec_region_cfg->iv_ptn_low  = iv_ptn_low;
	pdec_region_cfg->iv_ptn_high = iv_ptn_high;
	pdec_region_cfg->tag_base_addr = tag_base_addr;
	pdec_region_cfg->tag_flh_addr  = tag_flh_addr;
	pdec_region_cfg->hdr_total_size = total_hdr_size;

#if FLASH_FW_LOAD_DBG
	dbg_printf("SEC Decrypt info:\r\n");
	dbg_printf("dec_base = 0x%08x, dec_end = 0x%08x, dec_region_size = %d, dec_region_sel = %d\r\n", pdec_region_cfg->dec_base,
			   ((pdec_region_cfg->dec_base) + (pdec_region_cfg->dec_size)),
			   (pdec_region_cfg->dec_size),
			   (pimg_dec_lkp_tbl->rng_idx));
	dbg_printf("tag_base = 0x%08x, tag_flh_addr = 0x%08x\r\n", pdec_region_cfg->tag_base_addr, pdec_region_cfg->tag_flh_addr);

	dbg_printf("dump cfg pIv_ptn_low:0x%08x\r\n", (pdec_region_cfg->iv_ptn_low));
	dbg_printf("dump cfg pIv_ptn_high:0x%08x\r\n", (pdec_region_cfg->iv_ptn_high));
#endif
	(*pdec_sts) |= (ENABLE << idx);
	(psb_sec_cfg_pending->dec_not_set_sts) |= (ENABLE << idx);
	(psb_sec_cfg_pending->dec_cfg_cnt)++;

	return SUCCESS;
}

int xip_pending_process_rmp(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	hal_status_t sec_ret = HAL_OK;
	uint32_t i = 0x0;
	hal_sec_rmp_region_cfg_t *prmp_region_cfg = NULL;
	uint8_t *prmp_sts = NULL, *prmp_not_set_sts = NULL;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}
	if (p_sb_info == NULL) {
		ret = FAIL;
		return ret;
	}

	prmp_sts = &(psb_sec_cfg_pending->rmp_sts);
	prmp_not_set_sts = &(psb_sec_cfg_pending->rmp_not_set_sts);
	for (i = 0; i < MAX_PENDING_SEC_RMP_REGION_CFG; i++) {
		if (*prmp_not_set_sts & (ENABLE << i)) {
			prmp_region_cfg = &(psb_sec_cfg_pending->sec_rmp_region_cfg[i]);
			if (prmp_region_cfg->is_used) {
				// do remapping only
				//dbg_printf("do remapping only\r\n");
				// ---------- Start setting remap -------------------- //
				sec_ret = psb_sec_cfg_pending->sec_cfg_f.enable_remap_region(i, prmp_region_cfg->remap_addr, prmp_region_cfg->phy_addr,
						  prmp_region_cfg->remap_size);
				if (sec_ret != HAL_OK) {
					DBG_MISC_ERR("[Set RMP fail...]\r\n");
					ret = FAIL;
					return ret;
				} else {
					ret = SUCCESS;
					(*prmp_not_set_sts) &= (uint8_t)(~(ENABLE << i));
					dbg_printf("[Set RMP ok]\r\n");
#if 0
					// FW IMG manifest can simple check ptn
					if (0 != (memcmp(&img_mani_ptn_rmp_chk[0], prmp_region_cfg->remap_addr, sizeof(img_mani_ptn_rmp_chk)))) {
						ret = FAIL;
						return ret;
					} else {
						ret = SUCCESS;
						(*prmp_sts) |= (ENABLE << i);
						dbg_printf("[Set remap ok]\r\n");
					}
#endif
				}
				rtl_dcache_invalidate_by_addr(prmp_region_cfg->remap_addr, prmp_region_cfg->remap_size);
#if FLASH_FW_LOAD_DBG
				dbg_printf("phy dump:\r\n");
				dump_for_one_bytes((prmp_region_cfg->phy_addr), 128);
				dbg_printf("vir dump:\r\n");
				dump_for_one_bytes((prmp_region_cfg->remap_addr), 128);
#endif
			}
		}
	}
	return ret;
}

int xip_disable_rmp_config(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel)
{
	int ret = SUCCESS;
	hal_status_t sec_ret = HAL_OK;
	uint8_t i = 0x0;
	hal_sec_rmp_region_cfg_t *prmp_region_cfg = NULL;
	uint8_t disable_all_rmp = DISABLE;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}

	if (region_sel >= MAX_PENDING_SEC_RMP_REGION_CFG) {
		disable_all_rmp = ENABLE;
	}

	if (ENABLE == disable_all_rmp) {
		for (i = 0; i < MAX_PENDING_SEC_RMP_REGION_CFG; i++) {
			prmp_region_cfg = &(psb_sec_cfg_pending->sec_rmp_region_cfg[i]);
			if (prmp_region_cfg->is_used) {
				// disable remapping
				dbg_printf("[Disable RMP rgn_%d]\r\n", i);
				// ---------- Disable remap -------------------- //
				sec_ret = psb_sec_cfg_pending->sec_cfg_f.disable_rmp_region(i);
				if (sec_ret != HAL_OK) {
					ret = FAIL;
					return ret;
				}
				prmp_region_cfg->is_used    = DISABLE;
				prmp_region_cfg->phy_addr   = SB_SEC_INIT_ADDR;
				prmp_region_cfg->remap_addr = SB_SEC_INIT_ADDR;
				prmp_region_cfg->remap_size = 0;
			}
		}
		psb_sec_cfg_pending->rmp_cfg_cnt = DISABLE;
		psb_sec_cfg_pending->rmp_sts = DISABLE;
	} else {
		i = region_sel;
		prmp_region_cfg = &(psb_sec_cfg_pending->sec_rmp_region_cfg[i]);
		dbg_printf("[Disable RMP rgn_%d]\r\n", i);
		// ---------- Disable remap -------------------- //
		sec_ret = psb_sec_cfg_pending->sec_cfg_f.disable_rmp_region(i);
		if (sec_ret != HAL_OK) {
			ret = FAIL;
			return ret;
		}
		prmp_region_cfg->is_used    = DISABLE;
		prmp_region_cfg->phy_addr   = SB_SEC_INIT_ADDR;
		prmp_region_cfg->remap_addr = SB_SEC_INIT_ADDR;
		prmp_region_cfg->remap_size = 0;
		(psb_sec_cfg_pending->rmp_sts) &= (uint8_t)(~(ENABLE << i));
		(psb_sec_cfg_pending->rmp_cfg_cnt)--;
	}
	return SUCCESS;
}

int xip_pending_process_dec(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	hal_status_t sec_ret = HAL_OK;
	uint8_t i = 0x0;
	hal_sec_dec_region_cfg_t *pdec_region_cfg = NULL;
	uint8_t sec_cipher_sel = FLASH_SEC_CIPHER_GCM;
	fw_img_hdr_t *img_dec_hdr = NULL;
	uint16_t img_dec_type_id = FW_IMG_BL_ID;
	uint8_t *pdec_sts = NULL, *pdec_not_set_sts = NULL;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}
	if (p_sb_info == NULL) {
		ret = FAIL;
		return ret;
	}

	pdec_sts = &(psb_sec_cfg_pending->dec_sts);
	pdec_not_set_sts = &(psb_sec_cfg_pending->dec_not_set_sts);

	if (DISABLE == (psb_sec_cfg_pending->key_set)) {
		sec_ret = psb_sec_cfg_pending->sec_cfg_f.default_decrypt_init((psb_sec_cfg_pending->key_id));
		if (sec_ret != HAL_OK) {
			ret = FAIL;
			return;
		} else {
			psb_sec_cfg_pending->key_set = ENABLE;
		}
	}
	for (i = 0; i < MAX_PENDING_SEC_DEC_REGION_CFG; i++) {
		if (*pdec_not_set_sts & (ENABLE << i)) {
			pdec_region_cfg = &(psb_sec_cfg_pending->sec_dec_region_cfg[i]);
			if (pdec_region_cfg->is_used) {
				// do decrypt only
				//dbg_printf("do decrypt only\r\n");
				// ---------- Start setting decrypt -------------------- //


				sec_ret = psb_sec_cfg_pending->sec_cfg_f.decrypt_region_init(pdec_region_cfg->iv_ptn_low, pdec_region_cfg->iv_ptn_high, i);
				if (sec_ret != HAL_OK) {
					DBG_MISC_ERR("[Set DEC init fail...]\r\n");
					ret = FAIL;
					return ret;
				}
				switch (pdec_region_cfg->cipher_sel) {
				case IMG_SEC_ENC_ALG_AES256_GCM:
					sec_cipher_sel = FLASH_SEC_CIPHER_GCM;
					break;
				case IMG_SEC_ENC_ALG_AES256_CTR:
					sec_cipher_sel = FLASH_SEC_CIPHER_CTR;
					break;
				case IMG_SEC_ENC_ALG_AES256_ECB_MIX:
					sec_cipher_sel = FLASH_SEC_CIPHER_MIX;
					break;
				default:
					ret = FAIL;
					return ret;
					break;
				}
				sec_ret = psb_sec_cfg_pending->sec_cfg_f.decrypt_region_enable(sec_cipher_sel, pdec_region_cfg->dec_base,
						  pdec_region_cfg->dec_size, pdec_region_cfg->tag_base_addr);
				if (sec_ret != HAL_OK) {
					DBG_MISC_ERR("[Set DEC fail...]\r\n");
					ret = FAIL;
					return ret;
				} else {
					ret = SUCCESS;
					(*pdec_not_set_sts) &= (uint8_t)(~(ENABLE << i));
					dbg_printf("[Set DEC ok]\r\n");
				}
				rtl_dcache_invalidate_by_addr((pdec_region_cfg->dec_base), pdec_region_cfg->dec_size);
			}
#if FLASH_FW_LOAD_DBG
			dbg_printf("dec dump:\r\n");
			dump_for_one_bytes((pdec_region_cfg->dec_base), 128);
#endif
		}
	}
	return ret;
}

int xip_disable_dec_config(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel)
{
	int ret = SUCCESS;
	hal_status_t sec_ret = HAL_OK;
	uint8_t i = 0x0;
	hal_sec_dec_region_cfg_t *pdec_region_cfg = NULL;
	uint8_t disable_all_dec = DISABLE;

	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}

	if (region_sel >= MAX_PENDING_SEC_DEC_REGION_CFG) {
		disable_all_dec = ENABLE;
	}

	if (ENABLE == disable_all_dec) {
		for (i = 0; i < MAX_PENDING_SEC_DEC_REGION_CFG; i++) {
			pdec_region_cfg = &(psb_sec_cfg_pending->sec_dec_region_cfg[i]);
			if (pdec_region_cfg->is_used) {
				// disable decrypt setting
				dbg_printf("[Disable DEC rgn_%d]\r\n", i);
				// ---------- Disable decrypt -------------------- //
				sec_ret = psb_sec_cfg_pending->sec_cfg_f.disable_dec_region(i);
				if (sec_ret != HAL_OK) {
					ret = FAIL;
					return ret;
				}
				pdec_region_cfg->is_used    = DISABLE;
				pdec_region_cfg->cipher_sel  = 0x0;
				pdec_region_cfg->dec_base    = SB_SEC_INIT_ADDR;
				pdec_region_cfg->dec_size    = 0x0;
				pdec_region_cfg->iv_ptn_low  = SB_SEC_INIT_ADDR;
				pdec_region_cfg->iv_ptn_high = SB_SEC_INIT_ADDR;
				pdec_region_cfg->tag_base_addr = SB_SEC_INIT_ADDR;
				pdec_region_cfg->tag_flh_addr  = SB_SEC_INIT_ADDR;
				pdec_region_cfg->hdr_total_size = 0x0;
			}
		}
		psb_sec_cfg_pending->dec_cfg_cnt = DISABLE;
		psb_sec_cfg_pending->dec_sts = DISABLE;
	} else {
		i = region_sel;
		pdec_region_cfg = &(psb_sec_cfg_pending->sec_dec_region_cfg[i]);
		dbg_printf("[Disable DEC rgn_%d]\r\n", i);
		// ---------- Disable decrypt -------------------- //
		sec_ret = psb_sec_cfg_pending->sec_cfg_f.disable_dec_region(i);
		if (sec_ret != HAL_OK) {
			ret = FAIL;
			return ret;
		}
		pdec_region_cfg->is_used    = DISABLE;
		pdec_region_cfg->cipher_sel  = 0x0;
		pdec_region_cfg->dec_base    = SB_SEC_INIT_ADDR;
		pdec_region_cfg->dec_size    = 0x0;
		pdec_region_cfg->iv_ptn_low  = SB_SEC_INIT_ADDR;
		pdec_region_cfg->iv_ptn_high = SB_SEC_INIT_ADDR;
		pdec_region_cfg->tag_base_addr = SB_SEC_INIT_ADDR;
		pdec_region_cfg->tag_flh_addr  = SB_SEC_INIT_ADDR;
		pdec_region_cfg->hdr_total_size = 0x0;
		(psb_sec_cfg_pending->dec_sts) &= (uint8_t)(~(ENABLE << i));
		(psb_sec_cfg_pending->dec_cfg_cnt)--;
	}
	if ((DISABLE == (psb_sec_cfg_pending->dec_sts)) &&
		(DISABLE == (psb_sec_cfg_pending->dec_cfg_cnt))) {
		psb_sec_cfg_pending->key_set = DISABLE;
		psb_sec_cfg_pending->sec_cfg_f.aes_disable();
	}
	return SUCCESS;
}

uint8_t chk_rom_img_type_id_invalid(const uint16_t type_id)
{
	uint8_t i = 0x0, find_cnt = 0x0, invalid_sts = ENABLE;
	uint8_t img_type_id_list_size = sizeof(ld_img_type_id_list);
	for (i = 0; i < img_type_id_list_size; i++) {
		if (type_id == ld_img_type_id_list[i]) {
			invalid_sts = DISABLE;
		} else {
			find_cnt++;
		}
	}
	return invalid_sts;
}

int img_rmp_and_dec_lkp_tbl_insert(img_region_lookup_t *plkp_tbl, uint8_t tbl_size, uint8_t is_xip, uint8_t *tbl_cnt)
{
	int ret = SUCCESS;
	uint32_t i = 0x0;
	if (NULL == plkp_tbl) {
		return;
	}
	if (*tbl_cnt >= tbl_size) {
		ret = FAIL;
	} else {
		plkp_tbl->is_xip = is_xip;
		(*tbl_cnt)++;
	}
	return ret;
}

void img_rmp_and_dec_lkp_tbl_remove(img_region_lookup_t *plkp_tbl, uint8_t *tbl_cnt)
{
	uint32_t i = 0x0;
	if (NULL == plkp_tbl) {
		return;
	}
	plkp_tbl->is_xip  = DISABLE;
	plkp_tbl->rng_idx = RGN_IDX_INITVAL;
	(*tbl_cnt)--;
}

int sb_rom_img_dec_rmp(uint8_t *img_phy_addr, uint8_t *enc_rmp_base_addr, sec_boot_info_t *p_sb_info, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	uint8_t *pFw_img_flh = NULL, *pFw_rmp_img_flh = NULL, *pFw_data = NULL;
	uint32_t map_size = 0x0, dec_size = 0x0;
	uint32_t mani_align_offset = 0x0, map_addr = 0xFFFFFFFF, dec_base = 0xFFFFFFFF;
	uint32_t base_offset = 0xFFFFFFFF, end_offset = 0xFFFFFFFF;
	uint32_t tag_flash_mem = 0xFFFFFFFF, tag_base = 0xFFFFFFFF;
	uint32_t total_hdr_size = 0x0;
	uint32_t img_fh_phy_offset = 0xFFFFFFFF;
	uint8_t i = 0, ld_sec_enc_idx = 0x0, nxt_sec_enc_idx = 0x0, set_dec = DISABLE, aes_gcm_used = DISABLE;
	uint8_t sec_key_idx = SEC_KEY_IDX1, enc_en = DISABLE, encalg_sel = IMG_SEC_ENC_ALG_AES256_GCM;
	uint8_t xip_img_ld = DISABLE, nxt_xip_img_ld = DISABLE, xip_en = DISABLE;
	uint32_t iv_ptn_low = 0xFFFFFFFF, iv_ptn_high = 0xFFFFFFFF;
	manif_sec_enc_record_t *p_sec_enc_rd_tbl = NULL;
	hal_sec_region_cfg_t *psb_sec_cfg_pending = NULL;
	uint32_t cur_offset = 0x0;
	fw_img_hdr_t *pfw_hdr = NULL;
	const uint8_t sel_img_load = IMG_LOAD_BL;
	sect_hdr_t *psect_hdr = NULL;
	uint16_t nxt_img_type_id = FW_IMG_NOTSET_ID;
	uint8_t img_ld_idx = 0x0, rmp_lkp_cnt = 0x0, dec_lkp_cnt = 0x0;
	img_region_lookup_t img_rmp_lkp_tbl[IMG_REGION_LOOKUP_TBL_MAX_SIZE], img_dec_lkp_tbl[IMG_REGION_LOOKUP_TBL_MAX_SIZE];

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 4);

	dbg_printf("=== SB IMG Process ===\r\n");
	/* Init IMG region lookup table */
	for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE; i++) {
		img_rmp_lkp_tbl[i].is_xip  = DISABLE;
		img_rmp_lkp_tbl[i].rng_idx = RGN_IDX_INITVAL;
		img_dec_lkp_tbl[i].is_xip  = DISABLE;
		img_dec_lkp_tbl[i].rng_idx = RGN_IDX_INITVAL;
	}

	/* Get sec region control table */
	psb_sec_cfg_pending = (p_sb_info->psec_region_ctrl);

	/* Set Boot IMG remap region */
	pFw_img_flh = img_phy_addr;
	img_fh_phy_offset = (uint32_t)img_phy_addr;
	pFw_rmp_img_flh = enc_rmp_base_addr;
	map_addr = (uint32_t)enc_rmp_base_addr;
	mani_align_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);
	map_size = (mani_align_offset + (p_sb_info->img_hash_chk_info.msglen));  // Manifest 4K bytes-aligned + imgsz
	if (map_size & PAGE_OFFSET_MASK) {
		map_size = (((map_size / PAGE_SIZE) + 1) * PAGE_SIZE);
	}
	img_rmp_and_dec_lkp_tbl_insert(&img_rmp_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &rmp_lkp_cnt);
	ret = xip_pending_cfg_add_rmp(psb_sec_cfg_pending, &img_rmp_lkp_tbl[img_ld_idx], img_fh_phy_offset, map_addr, map_size);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = xip_pending_process_rmp(psb_sec_cfg_pending, p_sb_info);
	if (ret != SUCCESS) {
		return ret;
	}
	ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 4);
	/* offset img manifest, get FW IMG header */
	//dbg_printf("[--- Boot loader ---]\r\n");
	cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	pFw_data = pFw_rmp_img_flh + cur_offset;

	while ((uint8_t *)pFw_data != (uint8_t *)FW_HDR_NXTOFFSET_NULL) {
		/*
		   Based on enc_sts & nxt_sec_enc_idx, search from sec_enc_record, find which one base_addr match the pFw_data
		   [Note] FW IMG at least one IMG is Non-XIP IMGs (because .data section) and it always the 1st one IMG
		    - Check set decrypt region or not
		        1. if ld_sec_enc_idx == nxt_sec_enc_idx
		            > check ld_sec_enc_idx enc_sts,
		                $ if enc_en = 0x1, then record set decrypt region
		                $ if enc_en = 0x0, then record not set decrypt region
		        2. if ld_sec_enc_idx != nxt_sec_enc_idx
		            > ld_sec_enc_idx = nxt_sec_enc_idx
		            > check ld_sec_enc_idx enc_sts,
		                $ if enc_en = 0x1, then record set decrypt region
		                $ if enc_en = 0x0, then record not set decrypt region
		    - No need to set decrypt region:
		        goto Chk IMG valid or not flow
		    - Need to set decrypt region:
		        goto set decrypt region flow
		*/
		if (ld_sec_enc_idx != nxt_sec_enc_idx) {
			ld_sec_enc_idx = nxt_sec_enc_idx;
			//dbg_printf("ld_sec_enc_idx=%d\r\n", ld_sec_enc_idx);
		}
		// check set decrypt region or not, plaintext img do not set decrypt region
		if ((p_sb_info->pimg_sec_enc_info->enc_sts) & (ENABLE << ld_sec_enc_idx)) {
			if ((p_sb_info->psec_region_ctrl->dec_sts) & (ENABLE << ld_sec_enc_idx)) {
				set_dec = DISABLE;  // decrypt region already set!
			} else {
				set_dec = ENABLE;   // decrypt region not set before!
			}
		} else {
			set_dec = DISABLE;
		}
		xip_img_ld = nxt_xip_img_ld;
		//dbg_printf("set_dec=%d\r\n", set_dec);
		//dbg_printf("xip_img_ld=%d\r\n", xip_img_ld);

		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 5);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 5);
		if (ENABLE == set_dec) { // Cipher IMG handles

			// Check key load or not
			if (DISABLE == (psb_sec_cfg_pending->key_set)) {
				ret = xip_pending_cfg_add_dec_key(psb_sec_cfg_pending, sec_key_idx);
				if (ret != SUCCESS) {
					return ret;
				}
			}
			/*
			   Set Decrypt region, before setting check ready to load IMG is XIP IMG or not and check AES GCM set or not
			    - Non XIP IMG:
			        1. confirm again this record is not for XIP IMG(xip_en)
			            > if not for XIP IMG, continue set decrypt region
			            > if it's for XIP IMG, goto boot fail(load wrong sec_enc_record info fail)
			        2. if select AES GCM, then use it directly and record AES GCM already set
			    - XIP IMG:
			        1. confirm again this record is for XIP IMG(xip_en)
			            > if not for XIP IMG, goto boot fail(load wrong sec_enc_record info fail)
			            > if it's for XIP IMG, then set remap region first, based on decrypt region(should same as remap region)
			              $ base_addr & 0xFFFFF000, make sure the remap region start from the IMG HDR start(4K-byte aligned)
			              $ record already set XIP remap region
			        2. if AES GCM already set by before XIP IMG, then boot fail(Cannot allow multiple XIP IMGs use AES GCM)
			           else , if select AES GCM, then use it directly and record AES GCM already set
			*/

			/* Set decrypt region */
			p_sec_enc_rd_tbl = &(p_sb_info->pimg_sec_enc_info->sec_enc_record[ld_sec_enc_idx]);
			enc_en = (p_sec_enc_rd_tbl->enc_en);
			xip_en = (p_sec_enc_rd_tbl->xip_en);

			if (ENABLE == enc_en) {
				if (DISABLE == xip_img_ld) {    // Non-XIP IMG
					if (DISABLE == xip_en) {
						base_offset = (p_sec_enc_rd_tbl->base_addr);
						end_offset  = (p_sec_enc_rd_tbl->end_addr);
						if (end_offset & IMG_HDR_START_ALIGN_MASK) {
							end_offset = (((end_offset >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
						}
						dec_base = map_addr + base_offset;  // Reference RMP IMG manifest start
						dec_size = (end_offset - base_offset);
					} else {
						ret = FAIL; // (load wrong sec_enc_record info fail)
						goto sb_rom_img_dec_rmp_end;
					}
				} else { // XIP IMG
					ret = FAIL; // (boot img non-support XIP IMG load)
					goto sb_rom_img_dec_rmp_end;
				}

				/* Set encalg_sel */
				encalg_sel  = (p_sec_enc_rd_tbl->encalg_sel);
				if (IMG_SEC_ENC_ALG_AES256_GCM == encalg_sel) {
					if (DISABLE == aes_gcm_used) {
						aes_gcm_used = ENABLE;
					} else {
						if (ENABLE == xip_img_ld) {
							ret = FAIL; // (Cannot allow multiple XIP IMGs use AES GCM)
							goto sb_rom_img_dec_rmp_end;
						}
					}
				} else {
					aes_gcm_used = DISABLE;
				}

				/* Set iv pattern */
				iv_ptn_low  = psb_sec_cfg_pending->sec_cfg_f.set_word_from_byte_bigen(&(p_sec_enc_rd_tbl->iv_ptn_low));
				iv_ptn_high = psb_sec_cfg_pending->sec_cfg_f.set_word_from_byte_bigen(&(p_sec_enc_rd_tbl->iv_ptn_high));

				if (ENABLE == aes_gcm_used) {
					tag_flash_mem = (img_fh_phy_offset + (p_sec_enc_rd_tbl->tag_base_addr));  // AES GCM Tags reference img manifest physical addr
					ret = psb_sec_cfg_pending->sec_cfg_f.default_calculate_tag_base(dec_base, dec_size, tag_flash_mem, &tag_base, NULL);
					if (ret != SUCCESS) {
						return ret;
					}
				} else {
					tag_base = 0xFFFFFFFF;
					tag_flash_mem = 0xFFFFFFFF;
				}
				img_rmp_and_dec_lkp_tbl_insert(&img_dec_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &dec_lkp_cnt);
				ret = xip_pending_cfg_add_dec(psb_sec_cfg_pending, &img_dec_lkp_tbl[img_ld_idx],
											  encalg_sel, dec_base, dec_size, iv_ptn_low, iv_ptn_high,
											  tag_base, tag_flash_mem, sizeof(fw_img_hdr_t));
				if (ret != SUCCESS) {
					return ret;
				}
				ret = xip_pending_process_dec(psb_sec_cfg_pending, p_sb_info);
				if (ret != SUCCESS) {
					return ret;
				}
			}
		}
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 6);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 6);
		/*
		   Chk IMG valid or not from imt hdr type id
		   - valid , prepare load sections and check next img or not and next img type id is XIP IMG or not
		     > if XIP IMG, then record XIP IMG is the next IMG load
		   - invalid, goto boot fail(decrypt fail)
		*/
		//dbg_printf("[--- IMG HDR ---]\r\n");
		pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
		if (ENABLE == xip_img_ld) { // XIP cipher IMG already remap
			ret = FAIL; // (boot img non-support XIP IMG load)
			goto sb_rom_img_dec_rmp_end;
		}
		load_img_hdr_f((const uint8_t *)pFw_data, pfw_hdr, sel_img_load);
		if (chk_rom_img_type_id_invalid((pfw_hdr->type_id))) {
			ret = FAIL; // (invalid, colud get wrong addr or decrypt fail)
			goto sb_rom_img_dec_rmp_end;
		} else {
			if ((pfw_hdr->nxtoffset) != (uint32_t)FW_HDR_NXTOFFSET_NULL) {
				nxt_img_type_id = (pfw_hdr->nxt_type_id);
				if (FW_IMG_XIP_ID == (pfw_hdr->nxt_type_id)) {
					nxt_xip_img_ld = ENABLE;
				} else {
					nxt_xip_img_ld = DISABLE;
				}
			} else {
				nxt_img_type_id = FW_IMG_NOTSET_ID;
				nxt_xip_img_ld = DISABLE;
			}
		}
		/*
		   Load sections:
		    - Non XIP IMG:
		        > Load all section data into assign memory
		    - XIP IMG:
		        > XIP cipher IMG ,already set-up remap, then simple check XIP IMG Section type id & section nxtoffset
		        > XIP plaintext IMG,
		          only set XIP IMG Remap
		          follow section hdr #nxtoffset
		          $ nxtoffset & 0xFFFFF000, make sure the remap region start from the IMG HDR start(4K-byte aligned)
		*/

		if (((pfw_hdr->str_tbl) != 0x0) &&
			((pfw_hdr->str_tbl) != 0xFFFFFFFF)) {
			if (pfw_hdr->type_id == FW_IMG_BL_ID) {
				*pram_start_func = (RAM_FUNCTION_START_TABLE *)(pfw_hdr->str_tbl);
				ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 19);
			}
			dbg_printf("[Image Start Table @ 0x%x]\r\n", *pram_start_func);
		}
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 7);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 7);
		if (pfw_hdr->type_id != FW_IMG_XIP_ID) {
			pFw_data += sizeof(fw_img_hdr_t);
			load_img_sect_f((const uint8_t *)pFw_data, pfw_hdr);
		}

		/*
		   Check current IMG is XIP IMG or not, and check next img load or not
		   - Non XIP IMG:
		       > if next IMG exist & it's not XIP IMG,
		           # (nxtoffset != 0xFFFF_FFFF)Next IMG load prepare:
		              1. check nxtoffset if locate at current decrypt region
		                   $  if yes, then keep the current decrypt region setting and assign nxt_sec_enc_idx.(continue next img load)
		                   $  if no,  then disable current decrypt region setting and assign nxt_sec_enc_idx.(continue next img load)
		              2. pFw_data jump to (map_addr + nxtoffset)[Remap IMG manifest base + offset]
		       > if next IMG exist & it's XIP IMG,
		           # (nxtoffset != 0xFFFF_FFFF)Next IMG load prepare:
		              1. No nxtoffset check
		                   $  disable current decrypt region setting and current remap setting
		                   $  assign nxt_sec_enc_idx
		              2. pFw_data jump to (img_fh_phy_offset + nxtoffset)[Physical IMG manifest base + offset]

		      (Note:Disable AES GCM decrypt region setting, modify the record as well)

		       > else (no more next IMG exist), then break while

		   - XIP IMG:
		       > if next IMG exist & it's not XIP IMG, then boot fail(Cannot allow other Non-XIP IMGs after XIP IMGs)
		       > if next IMG exist & it's XIP IMG,
		           # (nxtoffset != 0xFFFF_FFFF)Next IMG load prepare:
		              1. No nxtoffset check
		                   $  keep the current decrypt region setting and current remap setting.
		                   $  assign nxt_sec_enc_idx
		              2. pFw_data jump to (img_fh_phy_offset + nxtoffset)[Physical IMG manifest base + offset]
		      (Note: If use AES GCM, then still keep the AES GCM already set)
		       > else (no more next IMG exist), then break while
		*/
		if (FW_IMG_NOTSET_ID != nxt_img_type_id) {
			if (DISABLE == xip_img_ld) {    // current IMG is Non XIP IMG
				if (nxt_img_type_id != FW_IMG_XIP_ID) { // next IMG is Non XIP IMG
					if (ENABLE == set_dec) {
						// check nxtoffset if locate at current decrypt region
						if (((pfw_hdr->nxtoffset) >= base_offset) &&    // locate current decrypt region
							((pfw_hdr->nxtoffset) < end_offset)) {
							nxt_sec_enc_idx = ld_sec_enc_idx;
						} else {    // other decrypt region
							xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[img_ld_idx].rng_idx));
							img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[img_ld_idx], &dec_lkp_cnt);
							if (ENABLE == aes_gcm_used) {
								aes_gcm_used = DISABLE;
							}
							nxt_sec_enc_idx = (++ld_sec_enc_idx);
						}
					} else {
						nxt_sec_enc_idx = (++ld_sec_enc_idx);
					}
				} else {    // next IMG is XIP IMG
					ret = FAIL; // (boot img non-support XIP IMG load)
					if (ENABLE == set_dec) {
						// disable all decrypt region setting and current remap setting
						for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE; i++) {
							if ((img_dec_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_DEC_REGION_CFG) &&
								(DISABLE == (img_dec_lkp_tbl[i].is_xip))) {
								xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[i].rng_idx));
								img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[i], &dec_lkp_cnt);
							}
							if ((img_rmp_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_RMP_REGION_CFG) &&
								(DISABLE == (img_rmp_lkp_tbl[i].is_xip))) {
								xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
								img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
							}
						}
						if (ENABLE == aes_gcm_used) {
							aes_gcm_used = DISABLE;
						}
						nxt_sec_enc_idx = (++ld_sec_enc_idx);
					} else {
						// Disable all remap only
						for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE; i++) {
							if ((img_rmp_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_RMP_REGION_CFG) &&
								(DISABLE == (img_rmp_lkp_tbl[i].is_xip))) {
								xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
								img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
							}
						}
						nxt_sec_enc_idx = (++ld_sec_enc_idx);
					}
					goto sb_rom_img_dec_rmp_end;
				}
			}
		}


		/*
		    (nxtoffset != 0xFFFF_FFFF)Next IMG load prepare:
		        - Next(Non XIP IMG):
		            > pFw_data jump to (map_addr + nxtoffset)[Remap IMG manifest base + offset]
		        - Next(XIP IMG):
		            > pFw_data jump to (img_fh_phy_offset + nxtoffset)[Physical IMG manifest base + offset]
		*/
		if ((pfw_hdr->nxtoffset) != (uint32_t)FW_HDR_NXTOFFSET_NULL) {
			img_ld_idx++;
			pFw_data = (uint8_t *)pFw_rmp_img_flh + (uint32_t)(pfw_hdr->nxtoffset);
			//dbg_printf("nxt_xip_img_ld not xip img\r\n");
		} else {
			break;
		}
	}

	// Disable all current decrypt region setting and current remap setting except XIP IMGs
	for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE ; i++) {
		if ((img_dec_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_DEC_REGION_CFG) &&
			(DISABLE == (img_dec_lkp_tbl[i].is_xip))) {
			xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[i].rng_idx));
			img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[i], &dec_lkp_cnt);
		}
		if ((img_rmp_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_RMP_REGION_CFG) &&
			(DISABLE == (img_rmp_lkp_tbl[i].is_xip))) {
			xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
			img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
		}
	}
sb_rom_img_dec_rmp_end:

	return ret;
}

void load_img_hdr_f(const uint8_t *img_offset, fw_img_hdr_t *img_hdr_buf, const uint8_t sel_img_load)
{
	uint8_t *ptr = (uint8_t *)img_offset;
	fw_img_hdr_t *pimg_hdr = img_hdr_buf;
	// set img hdr data
	_memcpy((void *)pimg_hdr, (void *)ptr, sizeof(fw_img_hdr_t));
	/*
	 +-------------------------------------------------------------------+
	 | Firmware IMG HDR Basic format                                     |
	 +------+--------+-----------+---------+-------+-------------+-------+
	 |  0x0 | imglen | nxtoffset | type_id | resv1 | s_jtag_ctrl | resv2 |
	 +------+--------+-----------+---------+-------+-------------+-------+
	 | 0x10 | resv3  | resv4     | str_tbl | resv5                       |
	 +------+--------+-----------+---------+-----------------------------+
	 | 0x20 |                        s_jtag_s_key                        |
	 +------+------------------------------------------------------------+
	 | 0x40 |                        s_jtag_ns_key                       |
	 +------+------------------------------------------------------------+
	 | 0x60 |                            resv6                           |
	 +------+------------------------------------------------------------+
	 | 0x70 |                            resv7                           |
	 +------+------------------------------------------------------------+
	 */
#if FLASH_FW_LOAD_DBG
	dbg_printf("  #imglen : %d(0x%08x)\r\n", pimg_hdr->imglen, pimg_hdr->imglen);
	dbg_printf("  #nxtoffset : 0x%08x\r\n", pimg_hdr->nxtoffset);
	dbg_printf("  #type_id: 0x%04x\r\n", pimg_hdr->type_id);
	dbg_printf("  #resv1: 0x%04x\r\n", pimg_hdr->resv1);

	dbg_printf("  #s_jtag_ctrl : 0x%0x\r\n", pimg_hdr->s_jtag_ctrl);
	__mem_dump(pimg_hdr->resv2, FW_HDR_RESV2_SIZE, "#resv2");
	__mem_dump(pimg_hdr->resv3, FW_HDR_RESV3_SIZE, "#resv3");
	__mem_dump(pimg_hdr->resv4, FW_HDR_RESV4_SIZE, "#resv4");
	dbg_printf("  #str_tbl : 0x%08x\r\n", pimg_hdr->str_tbl);
	__mem_dump(pimg_hdr->resv5, FW_HDR_RESV5_SIZE, "#resv5");
	__mem_dump(pimg_hdr->jtag_key.s_jtag_s_key, FW_JTAG_KEY_SIZE, "#img_jtag_s_key");
	__mem_dump(pimg_hdr->jtag_key.s_jtag_ns_key, FW_JTAG_KEY_SIZE, "#img_jtag_ns_key");
	__mem_dump(pimg_hdr->resv6, FW_HDR_RESV6_SIZE, "#resv6");
	__mem_dump(pimg_hdr->resv7, FW_HDR_RESV7_SIZE, "#resv7");
#endif
}

void load_img_sect_f(const uint8_t *img_offset, fw_img_hdr_t *pfw_hdr)
{
	uint8_t *ptr = (uint8_t *)img_offset;
	sect_hdr_t *psect_hdr = NULL;
	uint8_t *pimg_load_dest = NULL;
	uint32_t img_load_size = 0;

	while ((uint8_t *)ptr != (uint8_t *)SECT_HDR_NXTOFFSET_NULL) {
		//dbg_printf("[--- IMG Section HDR ---]\r\n");
		psect_hdr = (psect_hdr_t)&tmp_sect_hdr[0];
		_memcpy((void *)psect_hdr, (void *)ptr, sizeof(sect_hdr_t));

#if FLASH_FW_LOAD_DBG
		dbg_printf("[--- Section HDR ---]\r\n");
		dbg_printf("  #seclen : %d(0x%08x)\r\n", psect_hdr->seclen, psect_hdr->seclen);
		dbg_printf("  #nxtoffset : 0x%08x\r\n", psect_hdr->nxtoffset);
		dbg_printf("  #type_id: 0x%04x\r\n", psect_hdr->type_id);
		dbg_printf("  #resv1: 0x%04x\r\n", psect_hdr->resv1);

		dbg_printf("  #sec_ctrl : 0x%08x\r\n", psect_hdr->sec_ctrl);
		__mem_dump(psect_hdr->resv2, SECT_HDR_RESV2_SIZE, "#resv2");
		dbg_printf("  #dest : 0x%08x\r\n", psect_hdr->dest);
		__mem_dump(psect_hdr->resv3, SECT_HDR_RESV3_SIZE, "#resv3");
		__mem_dump(psect_hdr->resv4, SECT_HDR_RESV4_SIZE, "#resv4");

		__mem_dump(psect_hdr->sec_info.sec_info1, SECT_HDR_SEC_INFO_SIZE, "#sec_info1");
		__mem_dump(psect_hdr->sec_info.sec_info2, SECT_HDR_SEC_INFO_SIZE, "#sec_info2");
		__mem_dump(psect_hdr->resv5, SECT_HDR_RESV5_SIZE, "#resv5");
		__mem_dump(psect_hdr->resv6, SECT_HDR_RESV6_SIZE, "#resv6");
#endif
		ptr += sizeof(sect_hdr_t);
		pimg_load_dest = (psect_hdr->dest);
		img_load_size = (psect_hdr->seclen);
		// load section data from flash to assigned memory destination
		//dbg_printf("[--- IMG Section Raw Data ---]\r\n");
		switch (psect_hdr->type_id) {
		case FW_SIMG_DTCM_ID:
		case FW_SIMG_ITCM_ID:
		case FW_SIMG_SRAM_ID:
			_memcpy((void *)(pimg_load_dest), (void *)ptr, img_load_size);
			rtl_dcache_clean_by_addr((uint32_t *)pimg_load_dest, img_load_size);
#if FLASH_FW_LOAD_DBG
			uint8_t *pTmp = (uint8_t *)(psect_hdr->dest);
			if ((get_4byte((pTmp + 4)) >= RAM_FUN_TABLE_VALID_START_ADDR) &&
				(get_4byte((pTmp + 4)) < RAM_FUN_TABLE_VALID_END_ADDR)) {
				dbg_printf("[--- Section Data Pre-Parse ---]\r\n");
				dbg_printf("[--- Section Data(#gRamStartFun) ---]\r\n");
				dbg_printf("  #Signature addr : 0x%08x\r\n", get_4byte((pTmp)));
				dbg_printf("  #RamStartFun addr : 0x%08x\r\n", get_4byte((pTmp + 4)));
				dbg_printf("  #RamWakeupFun addr : 0x%08x\r\n", get_4byte((pTmp + 8)));
				dbg_printf("  #RamPatchFun0 addr : 0x%08x\r\n", get_4byte((pTmp + 12)));
				dbg_printf("  #RamPatchFun1 addr : 0x%08x\r\n", get_4byte((pTmp + 16)));
			}
			__mem_dump((psect_hdr->dest), 64, "#Section raw data");
#endif
			break;
		case FW_SIMG_DDR_ID:
			// No init DRAM codes in ROM
			break;
		case FW_SIMG_XIP_ID:
			break;
		}

		if ((psect_hdr->nxtoffset) != (uint32_t)SECT_HDR_NXTOFFSET_NULL) {
			ptr = (uint8_t *)pimg_start + (uint32_t)(psect_hdr->nxtoffset);
		} else {
			break;
		}
	}
}

int fw_load_f(const uint8_t *img_offset, const uint8_t sel_img_load, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	fw_img_hdr_t *pfw_hdr = NULL;
	uint32_t entry_img_type_id = FW_IMG_INI_VAL_ID;

	if (!ptr) {
		ret = FAIL;
		goto fw_load_f_end;
	}
	if (IMG_LOAD_BL == sel_img_load) {
		entry_img_type_id = FW_IMG_BL_ID;
	}


	// Confirm boot loader raw data
	while ((uint8_t *)ptr != (uint8_t *)FW_HDR_NXTOFFSET_NULL) {
		//dbg_printf("[--- IMG HDR ---]\r\n");
		pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
		load_img_hdr_f((const uint8_t *)ptr, pfw_hdr, sel_img_load);

		if ((pfw_hdr->str_tbl) != 0xFFFFFFFF) {
			if (pfw_hdr->type_id == entry_img_type_id) {
				*pram_start_func = (RAM_FUNCTION_START_TABLE *)(pfw_hdr->str_tbl);
				ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 19);
			}
			dbg_printf("[Image Start Table @ 0x%x]\r\n", *pram_start_func);
		}

		ptr += sizeof(fw_img_hdr_t);
		load_img_sect_f((const uint8_t *)ptr, pfw_hdr);

		if ((pfw_hdr->nxtoffset) != (uint32_t)FW_HDR_NXTOFFSET_NULL) {
			ptr = (uint8_t *)pimg_start + (uint32_t)(pfw_hdr->nxtoffset);
		} else {
			break;
		}
	}

fw_load_f_end:
	return ret;
}

int fw_load_bl_f(const uint8_t *img_offset, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	if (!ptr) {
		ret = FAIL;
		goto fw_load_bl_f_end;
	}

	ret = fw_load_f(img_offset, IMG_LOAD_BL, pram_start_func);
	if (ret != SUCCESS) {
		goto fw_load_bl_f_end;
	}

fw_load_bl_f_end:
	return ret;
}

int boot_rom_ld_key_certi(const uint8_t sbl_cfg, uint8_t *pcerti_tbl_flh, sec_boot_keycerti_t *psb_keycerti, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	certi_tbl_t *pcerti_tbl = NULL;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE;
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;
	sb_en      = sbl.bit.sb_en;

	if ((ENABLE == tb_en) || (ENABLE == sb_en) || (ENABLE == img_hsh_en)) {
		//DBG_INFO_MSG_ON(_DBG_EDDSA_);
		// Confirm Key Certificate Tbl
		//dbg_printf("[--- Key Certificate TBL ---]\r\n");
		pcerti_tbl = sb_get_key_certi_tbl_rec(pcerti_tbl_flh, p_sb_info);

		ret = fw_load_key_certi_f(sbl_cfg, pcerti_tbl, p_sb_info, psb_keycerti);
	}
	return ret;
}

int boot_rom_ld_parti_tbl(const uint8_t sbl_cfg, uint8_t *pPar_tbl_flh, part_tbl_info_t *pPar_tbl, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	uint32_t cur_offset = 0x0;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE;
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;
	sb_en      = sbl.bit.sb_en;

	// Confirm Partition Tbl Manifest
	//dbg_printf("[--- Partition TBL Manifest ---]\r\n");
	ret = verify_manif_f(pPar_tbl_flh, PT_TBL_INFO, p_sb_info);
	if (ret != SUCCESS) {
		return ret;
	}

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 16);
	if ((ENABLE == tb_en) || (ENABLE == img_hsh_en)) {
		ret = sb_rom_img_vrf_op(sbl_cfg, p_sb_info, PT_TBL_INFO);
		if (SUCCESS != ret) {
			return ret;
		}
	}

	// Get Partition Tbl raw data
	//dbg_printf("[--- Partition Table ---]\r\n");
	pPar_tbl->part_tbl_raw_data = fw_load_partition_tbl_f(pPar_tbl_flh);
	return ret;
}

int img_sel_op_idx(void *p_tbl_info, const uint8_t img_obj, const uint8_t img_sel_op)
{
	part_fst_info_t *pPar_fst = NULL;
	partition_tbl_t *pPar_tbl_raw_data = NULL;
	certi_tbl_t *pCerti_rec_tbl = NULL;
	part_record_t *pimg1_record = NULL, *pimg2_record = NULL;
	certi_tbl_record_t *pcert1_record = NULL, *pcert2_record = NULL;
	uint16_t img1_part_type_id = FW_PT_INI_VAL_ID, img2_part_type_id = FW_PT_INI_VAL_ID;
	uint8_t *pimg1_flh = NULL, *pimg2_flh = NULL;
	uint8_t  img1_idx = 0x0, img2_idx = 0x0;

	int img_sel_idx = -1;

	// Confirm IMG OBJ type ID
	if (LD_SEL_IMG_FW == img_obj) {
		pPar_tbl_raw_data = (partition_tbl_t *)p_tbl_info;
		img1_part_type_id = FW_PT_FW1_ID;
		img2_part_type_id = FW_PT_FW2_ID;
		pPar_fst = (part_fst_info_t *)(&pPar_tbl_raw_data->fst);
		img1_idx = pPar_fst->fw1_idx;
		img2_idx = pPar_fst->fw2_idx;
		pimg1_record = (part_record_t *)(&pPar_tbl_raw_data->partition_record[img1_idx]);
		pimg2_record = (part_record_t *)(&pPar_tbl_raw_data->partition_record[img2_idx]);
	} else if (LD_SEL_IMG_BOOT == img_obj) {
		pPar_tbl_raw_data = (partition_tbl_t *)p_tbl_info;
		img1_part_type_id = FW_PT_BL_PRI_ID;
		img2_part_type_id = FW_PT_BL_SEC_ID;
		pPar_fst = (part_fst_info_t *)(&pPar_tbl_raw_data->fst);
		img1_idx = pPar_fst->bl_p_idx;
		img2_idx = pPar_fst->bl_s_idx;
		pimg1_record = (part_record_t *)(&pPar_tbl_raw_data->partition_record[img1_idx]);
		pimg2_record = (part_record_t *)(&pPar_tbl_raw_data->partition_record[img2_idx]);
	} else if (LD_SEL_IMG_KEYCERTI == img_obj) {
		pCerti_rec_tbl = (certi_tbl_t *)p_tbl_info;
		img1_part_type_id = FW_PT_KEY_CER1_ID;
		img2_part_type_id = FW_PT_KEY_CER2_ID;
		img1_idx = pCerti_rec_tbl->certi1_idx;
		img2_idx = pCerti_rec_tbl->certi2_idx;
		pcert1_record = (certi_tbl_record_t *)(&pCerti_rec_tbl->key_cer_tbl_rec[img1_idx]);
		pcert2_record = (certi_tbl_record_t *)(&pCerti_rec_tbl->key_cer_tbl_rec[img2_idx]);
	} else {
		img_sel_idx = -1;
		return img_sel_idx;
	}

	if (LD_SEL_IMG_KEYCERTI == img_obj) {
		// Select the latest & valid key certificate
		if (((pcert1_record->type_id) == img1_part_type_id) &&
			((pcert1_record->valid) == INFO_VALID)) {
			pimg1_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pcert1_record->start_addr));
			//dbg_printf("pimg1_flh=0x%08x,img1_idx=%d\r\n", pimg1_flh, img1_idx);
			img_get_ld_sel_info_from_ie(img_obj, pimg1_flh, &ld_sel_info1);
		}

		if (((pcert2_record->type_id) == img2_part_type_id) &&
			((pcert2_record->valid) == INFO_VALID)) {
			pimg2_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pcert2_record->start_addr));
			//dbg_printf("pimg2_flh=0x%08x,img2_idx=%d\r\n", pimg2_flh, img2_idx);
			img_get_ld_sel_info_from_ie(img_obj, pimg2_flh, &ld_sel_info2);
		}
	} else {
		// Select the latest & valid img
		if (((pimg1_record->type_id) == img1_part_type_id) &&
			((pimg1_record->valid) == INFO_VALID)) {
			pimg1_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pimg1_record->start_addr));
			img_get_ld_sel_info_from_ie(img_obj, pimg1_flh, &ld_sel_info1);
		}

		if (((pimg2_record->type_id) == img2_part_type_id) &&
			((pimg2_record->valid) == INFO_VALID)) {
			pimg2_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pimg2_record->start_addr));
			img_get_ld_sel_info_from_ie(img_obj, pimg2_flh, &ld_sel_info2);
		}
	}
	if (IMG_SEL_LD == img_sel_op) {
		img_sel_idx = img_get_ld_sel_idx(img_obj, &ld_sel_info1, &ld_sel_info2, img1_idx, img2_idx);
	} else {
		img_sel_idx = img_get_update_sel_idx(img_obj, &ld_sel_info1, &ld_sel_info2, img1_idx, img2_idx);
	}
	return img_sel_idx;
}

int boot_rom_ld_bl(const uint8_t sbl_cfg, part_tbl_info_t *pPar_tbl, sec_boot_info_t *p_sb_info, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	part_fst_info_t *pPar_fst = NULL;
	int bl_idx = 0;
	uint8_t *pBl_flh = NULL, *pBl_data = NULL;
	part_record_t *pBl_record = NULL;
	uint32_t cur_offset = 0x0;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE, img_obj = 0x0;
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;
	sb_en      = sbl.bit.sb_en;

	// Select the latest valid boot img to load
	img_obj = LD_SEL_IMG_BOOT;
	bl_idx = img_sel_op_idx((pPar_tbl->part_tbl_raw_data), img_obj, IMG_SEL_LD);
	if (((-1) == bl_idx) || (bl_idx >= PARTITION_RECORD_MAX)) {
		DBG_BOOT_ERR("bl_idx err:%d\r\n", bl_idx);
		ret = FAIL;
		return ret;
	}
	// Get Boot loader img from partition record
	pBl_record = (part_record_t *)(&pPar_tbl->part_tbl_raw_data->partition_record[bl_idx]);
	if (((pBl_record->type_id) == FW_PT_BL_PRI_ID) ||
		((pBl_record->type_id) == FW_PT_BL_SEC_ID)) {
		if ((pBl_record->valid) == INFO_VALID) {
			pBl_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + (pBl_record->start_addr));
		} else {
			ret = FAIL;
			return ret;
		}
	} else {
		ret = FAIL;
		return ret;
	}

	// Confirm Boot Loader(Image1) Manifest
	//dbg_printf("[--- Boot Loader Manifest ---]\r\n");
	pimg_start = pBl_flh;
	//dbg_printf("pBl_flh=0x%08x\r\n", pimg_start);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 17);
	ret = verify_manif_f(pBl_flh, BL_INFO, p_sb_info);
	if (ret != SUCCESS) {
		return ret;
	}

	if ((ENABLE == tb_en) || (ENABLE == img_hsh_en)) {
		ret = sb_rom_img_vrf_op(sbl_cfg, p_sb_info, BL_INFO);
		if (SUCCESS != ret) {
			return ret;
		}
	}

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 18);
	if (ENABLE == sb_en) {
		/* Set remap region base */
		if ((p_sb_info->pimg_sec_enc_info->enc_rmp_base_addr) != 0xFFFFFFFF) {
			pimg_start = (p_sb_info->pimg_sec_enc_info->enc_rmp_base_addr);
		} else {
			ret = FAIL;
			return ret;
		}
		/* Set remap region & decrypt region */
		ret = sb_rom_img_dec_rmp(pBl_flh, pimg_start, p_sb_info, pram_start_func);
		if (SUCCESS != ret) {
			dbg_printf("=== SB BL fail ===\r\n");
			goto boot_rom_ld_bl_end;
		} else {
			dbg_printf("=== SB BL pass ===\r\n");
		}
	} else {
		// Get Boot loader plaintext raw data
		//dbg_printf("[--- Boot loader ---]\r\n");
		//pBl_flh += sizeof(img_manifest_t);
		cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
		pBl_flh += cur_offset;
		pBl_data = pBl_flh;
		ret = fw_load_bl_f(pBl_data, pram_start_func);
		if (ret != SUCCESS) {
			goto boot_rom_ld_bl_end;
		}
	}
boot_rom_ld_bl_end:
	return ret;
}

int fw_load_isp_iq_f(const uint8_t *img_offset, uint8_t fcs_id)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	fw_img_hdr_t *pfw_hdr = NULL;
	uint16_t entry_img_type_id = FW_IMG_ISP_ID;
	uint32_t flash_img_offset = SPI_FLASH_BASE;
	uint8_t sel_img_load = IMG_LOAD_ISP_IQ;

	if (!ptr) {
		ret = FAIL;
		goto fw_load_isp_iq_f_end;
	}

	// Confirm isp iq raw data
	while ((uint8_t *)ptr != (uint8_t *)FW_HDR_NXTOFFSET_NULL) {
		//dbg_printf("[--- IMG HDR ---]\r\n");
		pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
		load_img_hdr_f((const uint8_t *)ptr, pfw_hdr, sel_img_load);

		if (pfw_hdr->type_id == FW_IMG_ISP_ID) {
			ptr += sizeof(fw_img_hdr_t);
			flash_img_offset = (uint32_t)ptr;
			ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 8);
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 8);
			dbg_printf("ISP_IQ @ 0x%x, 0x%x, 0x%x\r\n", flash_img_offset, (pfw_hdr->imglen), fcs_id);
			// Debug ISP_IQ Flash IMG data
			//__mem_dump(ptr, 128, "ISP_IQ flash_img_offset_dump:");
			ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 9);
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 9);
			hal_rtl_voe_fcs_process((voe_cpy_t)_memcpy, flash_img_offset, fcs_id, (pfw_hdr->imglen));
			ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 10);
			ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 10);
			dbg_printf("=== Process ISP_IQ ===\r\n");
		}
		if ((pfw_hdr->nxtoffset) != (uint32_t)FW_HDR_NXTOFFSET_NULL) {
			ptr = (uint8_t *)pimg_start + (uint32_t)(pfw_hdr->nxtoffset);
		} else {
			break;
		}
	}

fw_load_isp_iq_f_end:
	return ret;
}

int verify_fcs_isp_iq_manif_f(const uint8_t *img_offset, const uint8_t info_type, uint8_t *pfcs_id)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	uint8_t *p_unpt_ie = (uint8_t *)img_offset;
	uint8_t *p_unpt_ie_msg = NULL;
	uint32_t unpt_ie_size = 0x0;
	uint32_t cur_unprot_area = 0x0;
	manif_hdr_t *pmani_hdr = NULL;
	char lbl_name[MANIFEST_MAX_LABEL_SIZE + 1] = "";
	uint32_t crc_chk = 0x0, unpt_ie_chk = 0x0, crc_chk_valid = DISABLE;

	//Confirm ISP_IQ Manifest HDR
	/*
	 * Parse manifest HDR
	 *            | label  | size   | vrf_al | resv |
	 * -----------+--------+--------+--------+------+
	 * size(Bytes)|  8     | 2      | 2      | 4    |
	 * ----------------------------------------------
	 */
	pmani_hdr = (pmanif_hdr_t)ptr;
	_memset(lbl_name, '\0', MANIFEST_MAX_LABEL_SIZE + 1);
	_strncpy(lbl_name, (void *)(pmani_hdr->lbl), MANIFEST_MAX_LABEL_SIZE);
	// label confirm
	if (chk_manifest_lbl(lbl_name)) {
		// Invalid Label
		ret = FAIL;
		goto confirm_fcs_isp_iq_manif_f_end;
	}

	// FCS fast verify
	cur_unprot_area = MANIFEST_UNIE_START_ALIGN_SIZE;  // 2K bytes-aligned for UNIE start
	p_unpt_ie += cur_unprot_area;
#if FLASH_FW_LOAD_DBG
	__mem_dump(p_unpt_ie, 16, "unprtect ie area");
#endif
	p_unpt_ie_msg = (p_unpt_ie + IE_TLV_TL_TOTAL_SIZE);
	unpt_ie_size  = get_3byte((p_unpt_ie + IE_TLV_TYPE_ID_SIZE));
	unpt_ie_chk   = get_4byte((p_unpt_ie + 8));
	hal_rtl_crypto_crc32_dma(&sb_rom_crypto_adtr, p_unpt_ie_msg, unpt_ie_size);
	hal_rtl_crypto_crc_wait_done(&sb_rom_crypto_adtr);
	crc_chk = hal_rtl_crypto_crc_get_result(&sb_rom_crypto_adtr);
	crc_chk_valid = DISABLE;

	if (_memcmp(&unpt_ie_chk, &crc_chk, MANIFEST_UNIE_CHK_SIZE) == 0) {
		ret = SUCCESS;
		dbg_printf("[fcs chk pass]\r\n");
		crc_chk_valid = ENABLE;
	} else {
		ret = -2;
		DBG_BOOT_ERR("[fcs chk fail]\r\n");
#if FLASH_FW_LOAD_DBG
		DBG_BOOT_ERR("unpt_ie_size = %d\r\n", unpt_ie_size);
		__mem_dump(&unpt_ie_chk, MANIFEST_UNIE_CHK_SIZE, "unpt_ie_chk");
		__mem_dump(&crc_chk, MANIFEST_UNIE_CHK_SIZE, "crc_chk");
#endif
	}
	if (ENABLE == crc_chk_valid) {
		_memcpy(pfcs_id, p_unpt_ie_msg, unpt_ie_size);
	}

confirm_fcs_isp_iq_manif_f_end:
	return ret;
}

int boot_rom_fcs_timer_init(hal_timer_group_adapter_t *ptimer_g_adptr, hal_timer_adapter_t *pfcs_system_timer)
{
	int ret = SUCCESS;
	if (NULL == ptimer_g_adptr) {
		ret = FAIL;
		return ret;
	}
	if (NULL == pfcs_system_timer) {
		ret = FAIL;
		return ret;
	}
	// Start a G-Timer as system tick
	hal_rtl_timer_clock_init(3, ENABLE);
	hal_rtl_timer_group_init(ptimer_g_adptr, 3); // time group 3
	hal_rtl_timer_group_sclk_sel(ptimer_g_adptr, GTimerSClk_31_25M);   // Group3 Sclk:4M
	hal_rtl_misc_start_systimer(pfcs_system_timer, CONFIG_SYS_TIMER_ID, GTimerCountUp, CONFIG_SYS_TICK_TIME, 3);
	return ret;
}

int boot_rom_ld_isp_iq(const uint8_t sbl_cfg, part_record_t *pISP_record, sec_boot_info_t *p_sb_info)
{
	int ret = SUCCESS;
	part_fst_info_t *pPar_fst = NULL;
	int isp_iq_idx = 0;
	uint8_t *pISP_iq_img_flh = NULL, *pISP_iq_data = NULL;
	uint32_t cur_offset = 0x0;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_obj = 0x0, fcs_id = 0x0;
	tb_en = sbl.bit.tb_en;
	sb_en = sbl.bit.sb_en;

	// Get Boot loader img from partition record
	pISP_iq_img_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + pISP_record->start_addr);
	pimg_start = pISP_iq_img_flh;
	//dbg_printf("pISP_iq_img_flh=0x%08x\r\n", pISP_iq_img_flh);
	// Verify fw img manifest
	ret = verify_fcs_isp_iq_manif_f(pISP_iq_img_flh, FW_ISP_INFO, &fcs_id);
	if (ret != SUCCESS) {
		return ret;
	}

	// Init fcs need timer
	ret = boot_rom_fcs_timer_init(&_timer_group3, &_fcs_system_timer);
	if (ret != SUCCESS) {
		return ret;
	}

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 12);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 12);
	// Get ISP_IQ raw data
	cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	pISP_iq_img_flh += cur_offset;
	pISP_iq_data = pISP_iq_img_flh;
	ret = fw_load_isp_iq_f(pISP_iq_data, fcs_id);
	if (ret != SUCCESS) {
		goto boot_rom_ld_isp_iq_end;
	}
boot_rom_ld_isp_iq_end:
	return ret;
}

int32_t rom_boot_load_tlv(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	volatile uint8_t sbl_cfg = 0x0;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE, fcs_rom_flow_dis = DISABLE;
	uint8_t *pcerti_tbl_flh = NULL;
	part_tbl_info_t *pPar_tbl = NULL;
	uint8_t *pPar_tbl_flh = NULL;
	sec_boot_info_t *p_sb_info = NULL;
	sec_boot_keycerti_t *psb_keycerti = NULL;
	hal_sec_region_cfg_t *psb_sec_cfg_pending = NULL;
	int ret = SUCCESS;

	dbg_printf("[Start Boot ROM...]\r\n");
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 9);

	// Get secure boot level config
	sbl_cfg    = potp_boot_cfg->byte.cfg3.byte;
	img_hsh_en = potp_boot_cfg->byte.cfg3.bit.img_hsh_en;
	tb_en      = potp_boot_cfg->byte.cfg3.bit.tb_en;
	sb_en      = potp_boot_cfg->byte.cfg3.bit.sb_en;
	fcs_rom_flow_dis = potp_boot_cfg7->bit.fcs_rom_flow_dis;

	p_sb_info = &sb_rom_info;
	_memset(p_sb_info, 0x0, sizeof(sec_boot_info_t));
	psb_keycerti = &export_sb_keycerti;
	_memset(psb_keycerti, 0x0, sizeof(sec_boot_keycerti_t));
	psb_sec_cfg_pending = &sb_sec_cfg_pending;
	_memset(psb_sec_cfg_pending, 0x0, sizeof(hal_sec_region_cfg_t));
	sec_cfg_f_init(psb_sec_cfg_pending);
	p_sb_info->psec_region_ctrl = psb_sec_cfg_pending;

	// Get Key Certificate table flash memory
	pcerti_tbl_flh = (uint8_t *)(SPI_FLASH_BASE + CERTIFICATE_TBL_FLSH_OFFSET);

	// Get Partition table flash memory
	pPar_tbl_flh = (uint8_t *)(SPI_FLASH_BASE + PARTITION_TBL_FLSH_OFFSET);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 10);

	if ((ENABLE == tb_en) || (ENABLE == sb_en) || (ENABLE == img_hsh_en)) {
		/*
		   Load Key certificate TBL & Key certificate
		 */
		dbg_printf("=== Load CERTI ===\r\n");
		ret = boot_rom_ld_key_certi(sbl_cfg, pcerti_tbl_flh, psb_keycerti, p_sb_info);
		if (ret != SUCCESS) {
			goto rom_boot_load_new_end;
		}
		dbg_printf("=== Load Done ===\r\n");
	}

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 11);
	/*
	   Load Partition TBL
	 */
	dbg_printf("=== Load PARTBL ===\r\n");
	pPar_tbl = &part_tbl_info;
	_memset((void *)pPar_tbl, 0x0, sizeof(part_tbl_info_t));
	ret = boot_rom_ld_parti_tbl(sbl_cfg, pPar_tbl_flh, pPar_tbl, p_sb_info);
	if (ret != SUCCESS) {
		goto rom_boot_load_new_end;
	}
	dbg_printf("=== Load Done ===\r\n");

	// Get ISP_IQ img record from partition table
	uint8_t ld_isp_iq_idx = 0;
	part_record_t *pISP_record = NULL;

	ld_isp_iq_idx = pPar_tbl->part_tbl_raw_data->fst.iq_idx;
	pISP_record = (part_record_t *)(&pPar_tbl->part_tbl_raw_data->partition_record[ld_isp_iq_idx]);
	if ((DISABLE == fcs_rom_flow_dis) &&
		(ENABLE == (pISP_record->valid))) {
		/*
		    Load ISP-IQ record loader
		*/
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 11);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 11);
		dbg_printf("=== Load ISP_IQ ===\r\n");
		ret = boot_rom_ld_isp_iq(sbl_cfg, pISP_record, p_sb_info);
		if (ret != SUCCESS) {
			goto rom_boot_load_new_end;
		}
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 13);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 13);
		dbg_printf("=== Load Done ===\r\n");
	}
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 12);
	/*
	   Load Boot loader
	 */
	dbg_printf("=== Load BL ===\r\n");
	ret = boot_rom_ld_bl(sbl_cfg, pPar_tbl, p_sb_info, pram_start_func);
	if (ret != SUCCESS) {
		goto rom_boot_load_new_end;
	}
	dbg_printf("=== Load Done ===\r\n");
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 13);
rom_boot_load_new_end:
	return ret;
}

int32_t flash_boot(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int32_t ret;
	int32_t crypt_ret;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	volatile uint8_t imgft_cfg = LD_IMF_FT_TLV;

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 8);


	hal_rtl_otp_init();
	crypt_ret = hal_rtl_crypto_engine_init(&sb_rom_crypto_adtr);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	ret = rom_boot_load(pram_start_func);
#else
	imgft_cfg = potp_boot_cfg7->bit.ntlv_img_ld_en;

	if (LD_IMF_FT_TLV == imgft_cfg) {
		ret = rom_boot_load_tlv(pram_start_func);
	} else {
		ret = rom_boot_load(pram_start_func);
	}
#endif
	crypt_ret = hal_rtl_crypto_engine_deinit(&sb_rom_crypto_adtr);
	hal_rtl_otp_deinit();
	return ret;
}

