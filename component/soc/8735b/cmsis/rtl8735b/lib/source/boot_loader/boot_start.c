/**************************************************************************//**
 * @file     boot_start.c
 * @brief    The RAM code entry function for the boot loader.
 * @version  V1.00
 * @date     2021-08-04
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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
#include "fw_img.h"
#include "fw_img_tlv.h"
#include "fw_snand_boot.h"
#include "otp_boot_cfg.h"
#include "mpu.h"
#include "rtl8735b_symbns4s.h"
#include "rtl8735b_ramstart.h"
#include "shell.h"
#include "hal.h"
#include "rt_printf.h"
#include "stdio_port_func.h"
#include "memory.h"
#include "diag.h"
#include "hal_flash_boot.h"
#include "shell.h"
#include "hal_sys_ctrl.h"
#include "hal_flash_sec.h"
#include "hal_dram_init.h"
#include "hal_crypto.h"
#include "hal_eddsa.h"
#include "hal_flash_sec.h"
#include "hal_hkdf.h"
#include "hal_otp.h"
#include "hal_spic.h"
#include "hal_flash.h"

#if CONFIG_BOOT_LD_VOE_CTRL
#include "hal_video.h"
#include "fw_voe_rom_boot.h"
#endif
#if defined(CONFIG_BUILD_BOOT) && defined(CONFIG_BUILD_RAM)

#include <arm_cmse.h>   /* Use CMSE intrinsics */

extern void stdio_port_deinit(void);
extern void shell_cmd_init(void);
extern void hal_pinmux_manager_init(hal_pin_mux_mang_t *pinmux_manag);
s32 snand_boot_loader(PRAM_FUNCTION_START_TABLE *ram_start_func);

SECTION_RAM_VECTOR_TABLE int_vector_t ram_vector_table[MAX_VECTOR_TABLE_NUM] __ALIGNED(256);

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
extern u8 Image$$_BSS$$ZI$$Limit[];
#define __bss_end__ Image$$_BSS$$ZI$$Limit
extern u8 Image$$_BSS$$ZI$$Base[];
#define __bss_start__ Image$$_BSS$$ZI$$Base
extern uint32_t Image$$_DATA$$Base;
#define __data_start__ Image$$_DATA$$Base
extern uint32_t Image$$_DATA$$Limit;
#define __data_end__ Image$$_DATA$$Limit
extern uint32_t Load$$_DATA$$Base;
#define __etext Load$$_DATA$$Base
extern u8 Image$$_RAM_VECTOR$$Base[];
#define __ram_vector_start__ Image$$_RAM_VECTOR$$Base
extern u8 Image$$_RAM_VECTOR$$Limit[];
#define __ram_vector_end__ Image$$_RAM_VECTOR$$Limit
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
extern u8 Image$$_BSS$$ZI$$Limit[];
#define __bss_end__ Image$$_BSS$$ZI$$Limit
extern u8 Image$$_BSS$$ZI$$Base[];
#define __bss_start__ Image$$_BSS$$ZI$$Base
extern uint32_t Image$$_DATA$$Base;
#define __data_start__ Image$$_DATA$$Base
extern uint32_t Image$$_DATA$$Limit;
#define __data_end__ Image$$_DATA$$Limit
extern uint32_t Load$$_DATA$$Base;
#define __etext Load$$_DATA$$Base
extern u8 Image$$_RAM_VECTOR$$Base[];
#define __ram_vector_start__ Image$$_RAM_VECTOR$$Base
extern u8 Image$$_RAM_VECTOR$$Limit[];
#define __ram_vector_end__ Image$$_RAM_VECTOR$$Limit
#elif defined ( __GNUC__ )
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
extern u8 __bss_end__[];
extern u8 __bss_start__[];
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __etext;
extern u8 __ram_vector_start__[];
extern u8 __ram_vector_end__[];
extern uint32_t __fw_img_start__;
extern uint32_t __ram_code_rodata_end__;
extern u8 __boot_sboot_bss_start__[];
extern u8 __boot_sboot_bss_end__[];
#elif defined( __ICCARM__ )
extern uint32_t RAM_STACK$$Base;
#define __StackLimit RAM_STACK$$Base
extern uint32_t RAM_STACK$$Limit;
#define __StackTop RAM_STACK$$Limit
extern u8 RAM_BSS$$Limit[];
#define __bss_end__ RAM_BSS$$Limit
extern u8 RAM_BSS$$Base[];
#define __bss_start__ RAM_BSS$$Base
extern uint32_t RAM_DATA$$Base;
#define __data_start__ RAM_DATA$$Base
extern uint32_t RAM_DATA$$Limit;
#define __data_end__ RAM_DATA$$Limit
#define __etext RAM_DATA$$Base
extern uint32_t RAM_VECTOR$$Base[];
#define __ram_vector_start__ RAM_VECTOR$$Base
extern uint32_t RAM_VECTOR$$Limit[];
#define __ram_vector_end__ RAM_VECTOR$$Limit
extern uint32_t RAM_DATA$$Base;
#define __fw_img_start__ RAM_DATA$$Base
extern uint32_t RAM_RODATA$$Limit;
#define __ram_code_rodata_end__ RAM_RODATA$$Limit
extern u8 SBOOT_BSS$$Base[];
extern u8 SBOOT_BSS$$Limit[];
#define __boot_sboot_bss_start__ SBOOT_BSS$$Base
#define __boot_sboot_bss_end__ SBOOT_BSS$$Limit
#endif

#if !defined(PRO2_WKARD) || (PRO2_WKARD==0) /* FIXME: PRO2_WKARD for fixing broken build */
extern sys_cp_fw_info_t sys_fw_info;
#endif /* FIXME: PRO2_WKARD for fixing broken build */

extern hal_img1_tmp_buf_t boot_tmp_buf_rec;

START_RAM_SIGNATURE_SECTION const char RamImgSignature[] = {
	'A', 'm', 'e', 'b', 'a', 'P', 'r', 'o', '2', 0xFF
};

const uint16_t ld_img_type_id_list[] = {
	FW_IMG_BL_ID, FW_IMG_FWHS_S_ID, FW_IMG_FWHS_NS_ID, FW_IMG_ISP_ID, FW_IMG_VOE_ID, FW_IMG_WLAN_ID,
	FW_IMG_XIP_ID, FW_IMG_CPFW_ID, FW_IMG_WOWLN_ID, FW_IMG_CINIT_ID
};

void __attribute__((noreturn)) boot_start(void);

START_RAM_FUN_A_SECTION
RAM_FUNCTION_START_TABLE gRamStartFun = {
	.Signature = (char *)RamImgSignature,
	.RamStartFun = boot_start,
	.RamWakeupFun = boot_start,
	.RamPatchFun0 = boot_start,
	.RamPatchFun1 = boot_start,
	.boot_cfg_w = 0,
	.msp_start = 0,
	.msp_limit = 0
};

#ifndef SECTION_SBOOT_BSS
#define SECTION_SBOOT_BSS           SECTION(".sboot.bss")
#endif

#define FLASH_FW_BOOTLOAD_DBG                 (0)
#define DDR_EN_PIN                            (PIN_F17)
#define DDR_FOR_VOE_IMG_LOAD_ADDR             (0x72E00000)
//uint32_t SystemCoreClock;
PRAM_FUNCTION_START_TABLE pRamStartFun;
hal_uart_group_adapter_t uart_gadapter;
hal_uart_adapter_t log_uart;
hal_timer_group_adapter_t timer_group1;
hal_timer_adapter_t system_timer;
hal_gdma_group_t hal_gdma_group;
hal_pin_mux_mang_t pinmux_manager;
hal_gpio_comm_adapter_t gpio_comm;
hal_gpio_adapter_t ddr_en_gpio;
SECTION_SBOOT_BSS sec_boot_info_t sb_ram_info;
SECTION_SBOOT_BSS uint8_t sb_digest_buf[IMG_HASH_CHK_DIGEST_MAX_SIZE];
SECTION_SBOOT_BSS hal_flash_sec_adapter_t sec_adtr;
SECTION_SBOOT_BSS hal_xip_sce_cfg_t xip_sce_cfg_pending __ALIGNED(32);
SECTION_SBOOT_BSS hal_sec_region_cfg_t sb_ram_sec_cfg_pending __ALIGNED(32);
SECTION_SBOOT_BSS uint8_t sec_digest_buf[2][20];
SECTION_SBOOT_BSS uint32_t fw1_xip_img_phy_offset; // simu
//SECTION_SBOOT_BSS uint8_t hmac_key_tmp[32] __attribute__((aligned(32)));


int32_t chk_fw_img_signature(char *sign)
{
	uint32_t i;
	int32_t ret = 0;

	for (i = 0; i < sizeof(RamImgSignature); i++) {
		if (RamImgSignature[i] == (char)0xFF) {
			break;
		}

		if (RamImgSignature[i] != sign[i]) {
			dbg_printf("Invalid FW Image Signature!\r\n");
			/*            dbg_printf ("Sing @0x%x: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
			                (uint32_t)sign, sign[0], sign[1],
			                sign[2], sign[3], sign[4],
			                sign[5], sign[6], sign[7]);
			*/
			ret = 1;
			break;
		}
	}

	return ret;
}

void log_uart_port_init(int log_uart_tx, int log_uart_rx, uint32_t baud_rate)
{
	hal_status_t ret;
#if defined(CONFIG_BUILD_SECURE) && (CONFIG_BUILD_SECURE == 1)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
#endif

	//* Init the UART port hadware
	ret = hal_uart_init(&log_uart, log_uart_tx, log_uart_rx, NULL);
	if (ret == HAL_OK) {
		hal_uart_set_baudrate(&log_uart, baud_rate);
		hal_uart_set_format(&log_uart, 8, UartParityNone, 1);
	}
	// hook the putc function to xprintf
#if defined(CONFIG_BUILD_SECURE) && (CONFIG_BUILD_SECURE == 1)
	stdio_port_init_s((void *)&log_uart, (stdio_putc_t)hal_uart_stubs.hal_uart_wputc, \
					  (stdio_getc_t)hal_uart_stubs.hal_uart_rgetc);

	symb_ns4s_stubs->stdio_port_init_ns((void *)&log_uart, (stdio_putc_t)hal_uart_stubs.hal_uart_wputc, \
										(stdio_getc_t)hal_uart_stubs.hal_uart_rgetc);
#else
	stdio_port_init_s((void *)&log_uart, (stdio_putc_t)hal_uart_stubs.hal_uart_wputc, \
					  (stdio_getc_t)hal_uart_stubs.hal_uart_rgetc);

	stdio_port_init_ns((void *)&log_uart, (stdio_putc_t)hal_uart_stubs.hal_uart_wputc, \
					   (stdio_getc_t)hal_uart_stubs.hal_uart_rgetc);
#endif
}

/* main function does not take arguments */
#if !defined(__MICROLIB) && defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(" .global __ARM_use_no_argv\n");
#endif

void boot_start_secure_bss_clean(void)
{
	uint32_t bss_len;

	/* clear secure boot BSS */
	bss_len = ((u32)__boot_sboot_bss_end__ - (u32)__boot_sboot_bss_start__);
	memset((void *)__boot_sboot_bss_start__, 0, bss_len);
	dcache_clean_by_addr((uint32_t *)__boot_sboot_bss_start__, bss_len);
}

/**
 *  @brief The RAM code BSS and Data initialization. All BSS data will be reset as 0.
 *         And load initial value to Data section. For Boot from flash case, the Data
 *         section should be loaded by boot loader.
 *
 *  @returns void
 */
void boot_start_bss_data_init(void)
{
	uint32_t bss_len;

	/* clear BSS */
	bss_len = ((u32)__bss_end__ - (u32)__bss_start__);
	memset((void *)__bss_start__, 0, bss_len);
	dcache_clean_by_addr((uint32_t *)__bss_start__, bss_len);
	__DSB();
	__ISB();
}

#define boot_flash_read(dst, src, size)         memcpy((void *)(dst), (void *)(SPI_FLASH_BASE+(src)), (uint32_t)(size))

/* temp solution for FPGA */
inline static uint32_t bootload_dram_init(void)
{
#if CONFIG_FPGA
	hal_sys_peripheral_en(DDR_SYS, ENABLE);   // Enable DDR clock and power on SYSON_S
#else
	hal_init_dram();
#endif
	return SUCCESS;
}

extern hal_spic_adaptor_t _hal_spic_adaptor;
extern boot_init_flags_t boot_init_flags;
extern uint8_t temp_dec_buf[];
extern uint8_t image_hash_buf[];

int32_t xip_pending_cfg_add(uint8_t is_remap, uint8_t is_enc, uint8_t key_id,
							uint32_t phy_addr, uint32_t remap_addr, uint32_t remap_sz, uint32_t total_hdr_size)
{
	uint32_t idx;
	hal_xip_page_cfg_t *ppage_cfg;

	if (xip_sce_cfg_pending.cfg_cnt > MAX_PENDING_XIP_PAGE_CFG) {
		DBG_MISC_ERR("boot_load: No free XIP Cfg pending handler\r\n");
		return FAIL;
	}

	idx = xip_sce_cfg_pending.cfg_cnt;
	ppage_cfg = &(xip_sce_cfg_pending.xip_page_cfg[idx]);
	ppage_cfg->is_used = 1;
	ppage_cfg->is_remap = is_remap;
	ppage_cfg->is_encrypted = is_enc;
	ppage_cfg->key_id = key_id;
	ppage_cfg->phy_addr = phy_addr;
	ppage_cfg->remap_addr = remap_addr;
	ppage_cfg->remap_size = remap_sz;
	ppage_cfg->hdr_total_size = total_hdr_size;

	dbg_printf("SEC Remap info:\r\n");
	dbg_printf("va_base = 0x%08x, pa_base = 0x%08x, remap_region_size = %d, remap_region_sel = %d\r\n", ppage_cfg->remap_addr,
			   ppage_cfg->phy_addr,
			   ppage_cfg->remap_size,
			   idx);
	xip_sce_cfg_pending.cfg_cnt++;

	return SUCCESS;
}

void xip_pending_process(void)
{
	uint32_t i, j;
	hal_xip_page_cfg_t *pxip_cfg;
	uint8_t xip_pg_sz;
	uint8_t xip_bk_sz;
	uint8_t *p_msg = NULL, *p_digest = NULL, *p_phy_dig = NULL, *p_vir_dig = NULL;
	uint32_t msglen = 0;

	for (i = 0; i < MAX_PENDING_XIP_PAGE_CFG; i++) {
		pxip_cfg = &(xip_sce_cfg_pending.xip_page_cfg[i]);
		if (pxip_cfg->is_used) {
			if (pxip_cfg->is_remap) {
				// only do re-map
				sec_adtr.va_base = pxip_cfg->remap_addr;
				sec_adtr.pa_base = pxip_cfg->phy_addr;
				sec_adtr.remap_region_size = pxip_cfg->remap_size;
				sec_adtr.remap_region_sel = i;
				hal_flash_sec_enable_remap_region(&sec_adtr);
#if 0
				dbg_printf("phy dump:\r\n");
				dump_bytes((u8 *)sec_adtr.pa_base, 1024);
				dbg_printf("vir dump:\r\n");
				dump_bytes((u8 *)sec_adtr.va_base, 1024);
#endif
				// Use SHA1 to verify XIP Flash remap First 4K bytes
				for (j = 0; j < 2; j++) {
					if (j == 0) {
						p_msg = pxip_cfg->phy_addr;
						p_phy_dig = &sec_digest_buf[0][0];
						p_digest = p_phy_dig;
					} else {
						p_msg = pxip_cfg->remap_addr;
						p_vir_dig = &sec_digest_buf[1][0];
						p_digest = p_vir_dig;
					}
					msglen = PAGE_SIZE;
					hal_crypto_sha1_init();
					hal_crypto_sha1_update(p_msg, msglen);
					hal_crypto_sha1_final(p_digest);
				}
				if (memcmp((void *)p_phy_dig, (void *)p_vir_dig, 20) != 0) {
					dbg_printf("\r\nXIP Remap: 0x%08x -> 0x%08x fail\r\n", pxip_cfg->remap_addr, pxip_cfg->phy_addr);
				} else {
					dbg_printf("\r\nXIP Remap: 0x%08x -> 0x%08x pass\r\n", pxip_cfg->remap_addr, pxip_cfg->phy_addr);
				}
			}
		} else {
			break;
		}
	}
}


int32_t boot_load_no_sb(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	hal_status_t ret;
	int32_t crypt_ret = SUCCESS;
	int32_t boot_ret = SUCCESS;
	fm_image_header_t *pheader;
	uint8_t load_fw_idx;
	uint8_t fw1_rec_idx;
	uint32_t image_size;
	fw_part_record_t *pfw_img_record;
	paes_key_type_t pkey_tbl;
	partition_table_t *ppartition_tbl;
	fw_image_type_t *pfw_enced_img; // encrypted sub-image start address
	fw_sub_image_type_t *penced_sub_image_header;   // encrypted sub-image header start address
	fw_sub_image_type_t *psub_image_header;
	fw_fst_t *pfst;
	uint32_t next_section_offset;
	uint32_t flash_img_offset;
	uint32_t sub_img_cnt;
	fm_image_header_t *psection_header;
	raw_image_hdr_t *psection_img_hdr;
	uint8_t *pimg_load_dest;
	uint32_t img_load_size;

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 23);
	crypt_ret = hal_crypto_engine_init();
	if (crypt_ret != SUCCESS) {
		DBG_MISC_ERR("boot_load: Crypto Init Failed!\r\n");
		boot_ret = FAIL;
		goto boot_load_no_sb_end;
	}

	ppartition_tbl = (partition_table_t *)boot_get_partition_tbl();

	fw1_rec_idx = ppartition_tbl->image_info.fw1_idx;
	load_fw_idx = fw1_rec_idx;
	dbg_printf("boot_get_load_fw_idx: Load FW%u\r\n", load_fw_idx);

	pfw_img_record = (fw_part_record_t *)(((fw_part_record_t *)&ppartition_tbl->partition_record[0]) + load_fw_idx);
	pfw_enced_img = (fw_image_type_t *)(SPI_FLASH_BASE + pfw_img_record->start_addr);
	penced_sub_image_header = &(pfw_enced_img->sub_img);

	psub_image_header = (fw_sub_image_type_t *)temp_dec_buf;
	psection_header = (fm_image_header_t *)((uint8_t *)temp_dec_buf + sizeof(fw_sub_image_type_t));
	psection_img_hdr = (raw_image_hdr_t *)((uint8_t *)psection_header + sizeof(fm_image_header_t));
	pheader = &psub_image_header->img_header;
	pfst = &psub_image_header->fst;

	sub_img_cnt = 0;
	dbg_printf("[Start Parse IMG...]\r\n");
	while ((uint32_t)penced_sub_image_header != IMG_LINK_LIST_END) {

		// image header is plain text
		boot_flash_read(pheader, (uint32_t) & (penced_sub_image_header->img_header) - SPI_FLASH_BASE, sizeof(fm_image_header_t));

		// FST is plain text
		boot_flash_read(pfst, (uint32_t) & (penced_sub_image_header->fst) - SPI_FLASH_BASE, sizeof(fw_fst_t));

		if (sub_img_cnt == 0) {
			// the 1st sub-image hash calculation is start from OTA signature
			image_size = OTA_SIGNATURE_SIZE + (PUBLIC_KEY_SIZE * MAX_PUBLIC_KEY_NUM) + pheader->image_len + sizeof(fm_image_header_t);
			flash_img_offset = (uint32_t)pfw_enced_img;
		} else {
			// 2nd and after sub-image hash calculation is start from image header
			image_size = pheader->image_len + sizeof(fm_image_header_t);
			flash_img_offset = (uint32_t)penced_sub_image_header;
		}

		// Decrypt image
		next_section_offset = (uint32_t)((uint8_t *)penced_sub_image_header + sizeof(fw_sub_image_type_t));

		// Image is plain text, just copy image data from flash to RAM
		while (next_section_offset != 0xFFFFFFFF) {
			dbg_printf("[Start Parse Section @ 0x%x]\r\n", next_section_offset);
			psection_header = (fm_image_header_t *)(next_section_offset);

			if ((psection_header->image_len == 0)) {
				// it's an null end section
				goto _boot_load_no_sb_end;
			}
#if 0
			// Validate Section header signature
			if (boot_memcmp(pfst->validate, psection_header->validation_patt, 8)) {
				DBG_MISC_ERR("Sub_img Section Hdr Validate Err!\r\n");
				goto _boot_load_no_sb_end;
			}
#endif
			psection_img_hdr = (raw_image_hdr_t *)((uint8_t *)psection_header + sizeof(fm_image_header_t));
			flash_img_offset = next_section_offset + sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t);
			pimg_load_dest = (uint8_t *)psection_img_hdr->start_addr;
			img_load_size = psection_img_hdr->img_sz;
			dbg_printf("img_load_size = %d\r\n", img_load_size);
			dbg_printf("flash_img_offset = 0x%x\r\n", flash_img_offset);

			if (img_load_size > 0) {
				if (pheader->img_type == FW_IMG_XIP) {
					if ((uint32_t)pimg_load_dest != flash_img_offset) {
						// it need to do XIP re-mapping
						uint32_t map_size;
						uint32_t map_addr;
						uint32_t total_hdr_size;
						dbg_printf("Todo:Handles FW_IMG_XIP\r\n");

						total_hdr_size = sizeof(fw_sub_image_type_t) + sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t);
						if (sub_img_cnt == 0) {
							// this XIP image is the 1st image, so there is a OTA signature and 6 * public key are prepended
							total_hdr_size += (OTA_SIGNATURE_SIZE + sizeof(public_key_type_t) * MAX_PUBLIC_KEY_NUM);
						}
						flash_img_offset -= total_hdr_size;

						map_size = image_size + total_hdr_size;
						map_addr = (uint32_t)pimg_load_dest - total_hdr_size;

						if (map_size & PAGE_OFFSET_MASK) {
							map_size = (((map_size / PAGE_SIZE) + 1) * PAGE_SIZE);
						}

						// cannot do XIP re-map or encryption now, since flash loading still running
						// add it as pending, do it later
						if (xip_pending_cfg_add(1 /*is_remap*/, 0 /*is_enc*/, 0 /*key_id*/, flash_img_offset, map_addr, map_size,
												total_hdr_size) == FAIL) {
							boot_ret = FAIL;
							goto boot_load_no_sb_end;
						}
					}
				} else if (pheader->img_type == FW_IMG_VOE) {
					//dbg_printf("VOE Load dest:0x%08x\r\n",pimg_load_dest);
					dbg_printf("VOE flash offset = 0x%x\r\n", flash_img_offset);
					dbg_printf("VOE img_load_size = %d\r\n", img_load_size);
#if CONFIG_BOOT_LD_VOE_CTRL
					// Debug VOE Flash IMG data
					//dbg_printf("VOE flash_img_offset_dump:");
					//dump_bytes ((u8 *)flash_img_offset, 32);
//					hal_video_load_fw((int *)flash_img_offset, (int *)DDR_FOR_VOE_IMG_LOAD_ADDR);
					hal_video_load_fw((voe_cpy_t)memcpy, (int *)flash_img_offset, (int *)DDR_FOR_VOE_IMG_LOAD_ADDR);

					dbg_printf("[VOE IMG Load]\r\n");
#endif
				} else {
					if (psection_header->img_type == FW_SIMG_PSRAM) {
						if (boot_init_flags.b.psram_inited == 0) {
							bootload_dram_init();
							boot_init_flags.b.psram_inited = 1;
						}
					}
					dbg_printf("pimg_load_dest = 0x%x\r\n", pimg_load_dest);
					dbg_printf("psection_img_hdr->ram_start_tbl = 0x%x\r\n", psection_img_hdr->ram_start_tbl);
					boot_flash_read(pimg_load_dest, (flash_img_offset - SPI_FLASH_BASE), img_load_size);
					dcache_clean_by_addr((uint32_t *)pimg_load_dest, img_load_size);
					// Debug ram img data
					//dbg_printf("flash_img_offset_dump:");
					//dump_bytes ((u8 *)flash_img_offset, img_load_size);
					if (psection_img_hdr->ram_start_tbl != 0xFFFFFFFF) {
						if (pheader->img_type == FW_IMG_FWHS_S) {
							*pram_start_func = (PRAM_FUNCTION_START_TABLE)psection_img_hdr->ram_start_tbl;
							RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 28);
						}
						dbg_printf("[Image Start Table @ 0x%x]\r\n", *pram_start_func);
					}
				}
			}
_boot_load_no_sb_end:
			// load next section
			if (psection_header->nxt_img_offset != IMG_LINK_LIST_END) {
				next_section_offset += psection_header->nxt_img_offset;
				dbg_printf("[Load next section @ 0x%x]\r\n", next_section_offset);
			} else {
				// no next section
				next_section_offset = IMG_LINK_LIST_END;
				//dbg_printf ("no next section\r\n");
			}
		}
		// load next sub-image
		if ((pheader->nxt_img_offset != IMG_LINK_LIST_END) && (pheader->nxt_img_offset != 0)) {
			penced_sub_image_header = (fw_sub_image_type_t *)((uint8_t *)penced_sub_image_header + pheader->nxt_img_offset);
			sub_img_cnt++;
		} else {
			penced_sub_image_header = (fw_sub_image_type_t *)IMG_LINK_LIST_END;
		}
	}
	(*pram_start_func)->init_flags.w = boot_init_flags.w;

	xip_pending_process();
boot_load_no_sb_end:
	hal_crypto_engine_deinit();
	return SUCCESS;
}

extern uint8_t tmp_img_hdr[];
extern uint8_t tmp_sect_hdr[];
extern uint8_t *pimg_start;

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


#undef __mem_dump_str
#define __mem_dump_str(start,size,str) do{ \
    dbg_printf("%s\r\n",str); \
    dump_bytes((uint8_t *)start,size); \
}while(0)

#undef __mem_dump
#define __mem_dump(start,size,prefix) do{ \
    dbg_printf(prefix "\r\n"); \
    dump_bytes((uint8_t *)start,size); \
}while(0)

int sb_ram_img_pbk_hash_vrf_f(sec_boot_info_t *p_sb_info, const uint8_t info_type)
{
	uint8_t vrf_alg = IMG_SIGN_VRF_ALG_EDDSA_ED25519, digest_len = 0, i = 0;
	uint8_t *p_pbk_chk = NULL, *p_pbk = NULL, *p_digest = NULL;
	uint32_t vrf_pbk_len = 0;
	int pbk_vrf_ret = -1;
	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_ram_img_pbk_hash_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_pbk) == NULL) {
		DBG_BOOT_ERR("p_pbk is NULL\r\n");
		goto sb_ram_img_pbk_hash_vrf_f_end;
	}
	p_pbk = p_sb_info->img_sign_vrf_info.p_pbk;
	p_digest = &sb_digest_buf[0];
	memset(p_digest, 0x0, IMG_HASH_CHK_DIGEST_MAX_SIZE);
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
		pbk_vrf_ret = SUCCESS;
		goto sb_ram_img_pbk_hash_vrf_f_end;
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

	hal_crypto_sha2_256_init();
	hal_crypto_sha2_256_update(p_pbk, vrf_pbk_len);
	hal_crypto_sha2_256_final(p_digest);
	dcache_invalidate_by_addr((void *)p_digest, digest_len);
	//__mem_dump(p_digest, digest_len, "pbk_digest");

	if (memcmp(p_pbk_chk, p_digest, digest_len) == 0) {
		dbg_printf("[vrf pbk pass]\r\n");
		pbk_vrf_ret = SUCCESS;
	} else {
		DBG_BOOT_ERR("[vrf pbk fail]\r\n");
		pbk_vrf_ret = -1;
		__mem_dump(p_pbk_chk, digest_len, "pbk_chk_digest");
		__mem_dump(p_digest, digest_len, "pbk_digest");
	}

sb_ram_img_pbk_hash_vrf_f_end:
	return pbk_vrf_ret;
}


int sb_ram_img_sign_vrf_f(sec_boot_info_t *p_sb_info)
{
	uint8_t vrf_alg = 0x0;
	uint8_t *p_vrf_msg = NULL, *p_pbk = NULL, *p_sign = NULL;
	uint32_t vrf_msg_len = 0;
	int img_vrf_ret = -1;

	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_ram_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_pbk) == NULL) {
		DBG_BOOT_ERR("p_pbk is NULL\r\n");
		goto sb_ram_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_sign) == NULL) {
		DBG_BOOT_ERR("p_sign is NULL\r\n");
		goto sb_ram_img_sign_vrf_f_end;
	}
	if ((p_sb_info->img_sign_vrf_info.p_msg) == NULL) {
		DBG_BOOT_ERR("p_msg is NULL\r\n");
		goto sb_ram_img_sign_vrf_f_end;
	} else {
		if ((p_sb_info->img_sign_vrf_info.msglen) == 0) {
			DBG_BOOT_ERR("msglen is 0\r\n");
			goto sb_ram_img_sign_vrf_f_end;
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
		img_vrf_ret = hal_eddsa_engine_init(EDDSA_HASH_CRYPTO_HW_SEL_EN);
		if (SUCCESS != img_vrf_ret) {
			dbg_printf("hal_rtl_eddsa_init fail\r\n");
		} else {
			img_vrf_ret = hal_eddsa_sign_verify(p_sign, p_vrf_msg, p_pbk, vrf_msg_len,
												EDDSA_FLOW_AUTOFLOW, ENABLE);
		}
		//hal_rtl_eddsa_deinit(&sb_rom_eddsa_adtr);
#endif
		break;
	default:
		break;
	}
sb_ram_img_sign_vrf_f_end:
	if (img_vrf_ret == SUCCESS) {
		dbg_printf("[vrf sig pass]\r\n");
	} else {
		DBG_BOOT_ERR("[vrf sig fail]\r\n");
	}
	return img_vrf_ret;
}

typedef int (*img_hsh_init_f_t)(const uint8_t *key, const uint32_t key_len);
typedef int (*img_hsh_update_f_t)(const uint8_t *msg, const uint32_t msg_len);
typedef int (*img_hsh_final_f_t)(uint8_t *digest);

int sb_img_hash_init_wrap_f(const uint8_t *key, const uint32_t key_len)
{
	int ret = SUCCESS;
	uint32_t sk_cfg = 0x0;
	if ((NULL == key) && (0 == key_len)) {
		ret = hal_crypto_sha2_256_init();

	} else if ((NULL == key) && (0x0 != key_len)) {
		sk_cfg = key_len;
		ret = hal_crypto_hmac_sha2_256_sk_init(key, sk_cfg);
	} else {
		ret = hal_crypto_hmac_sha2_256_init(key, key_len);
	}
	return ret;
}

int sb_ram_all_img_hash_chk_f(sec_boot_info_t *p_sb_info)
{
	int img_vrf_ret = -1;
	int ret = SUCCESS;
	hal_status_t hkdf_ret = HAL_OK;
	uint8_t hash_alg = IMG_HSH_CHK_ALG_SHA256;
	uint8_t *p_img_digest_chk = NULL, *p_msg = NULL, *p_digest = NULL, *p_hshks = NULL, *p_hshkn = NULL, *p_hmac_key = NULL;
	uint32_t msglen = 0, digest_len = 0, key_len = 0, hash_size = 0, auth_type = 0x0;
	uint8_t huk_idx = 0, extract_sk_idx, extract_wb_idx, expand_sk_idx, expand_wb_idx;
	uint32_t msglen_cnt = 0, msg_offset = 0, sk_cfg = 0x0;
	img_hsh_init_f_t   img_hsh_init_f   = NULL;
	img_hsh_update_f_t img_hsh_update_f = NULL;
	img_hsh_final_f_t  img_hsh_final_f  = NULL;

	if (!p_sb_info) {
		DBG_BOOT_ERR("p_sb_info is NULL\r\n");
		goto sb_ram_all_img_hash_chk_f_end;
	}
	if ((p_sb_info->img_hash_chk_info.p_digest) == NULL) {
		DBG_BOOT_ERR("p_digest is NULL\r\n");
		goto sb_ram_all_img_hash_chk_f_end;
	}
	if ((p_sb_info->img_hash_chk_info.p_msg) == NULL) {
		DBG_BOOT_ERR("p_msg is NULL\r\n");
		goto sb_ram_all_img_hash_chk_f_end;
	} else {
		if ((p_sb_info->img_hash_chk_info.msglen) == 0) {
			DBG_BOOT_ERR("msglen is 0\r\n");
			goto sb_ram_all_img_hash_chk_f_end;
		}
	}
	p_msg = p_sb_info->img_hash_chk_info.p_msg;
	msglen = p_sb_info->img_hash_chk_info.msglen;
	p_digest = &sb_digest_buf[0];
	memset(p_digest, 0x0, IMG_HASH_CHK_DIGEST_MAX_SIZE);
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
		auth_type = AUTH_TYPE_SHA2_256_ALL;
		p_hmac_key = NULL;
		key_len    = 0;
		img_hsh_init_f   = &sb_img_hash_init_wrap_f;
		img_hsh_update_f = &hal_crypto_sha2_256_update;
		img_hsh_final_f  = &hal_crypto_sha2_256_final;
		break;
	case IMG_HSH_CHK_ALG_HMAC_SHA256:
		auth_type = AUTH_TYPE_HMAC_SHA2_256_ALL;
		img_hsh_init_f = &sb_img_hash_init_wrap_f;
		img_hsh_update_f = &hal_crypto_hmac_sha2_256_update;
		//img_hsh_final_f  = &hal_crypto_hmac_sha2_256_final;
		img_hsh_final_f  = &hal_crypto_hmac_sha2_256_sk_final;
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
		memset(&hmac_key_tmp[0], 0x0, 32);
		hal_hkdf_hmac_init(HKDF_CRYPTO_HW_SEL_EN, HKDF_HMAC_SHA256);
		hal_hkdf_extract_secure_set_cfg(KEY_STG_SKTYPE_LD_SK, extract_sk_idx,
										KEY_STG_WBTYPE_WB_ONLY_STG, extract_wb_idx);
		hal_hkdf_extract_secure(NULL, p_hshks, SB_HKDF_KEY_SALT_SIZE, NULL);        // bug here

		hal_hkdf_expand_secure_set_cfg(KEY_STG_SKTYPE_LD_SK, expand_sk_idx,
									   KEY_STG_WBTYPE_WB_STG_BUF, expand_wb_idx);
		hal_hkdf_expand_secure(NULL, p_hshkn, SB_HKDF_KEY_NONCE_SIZE, &hmac_key_tmp[0], SB_HKDF_HMAC_SHA256_SIZE);
		p_hmac_key = &hmac_key_tmp[0];
		key_len    = SB_HKDF_HMAC_SHA256_SIZE;
#if 0
		__mem_dump(p_hshks, SB_HKDF_KEY_SALT_SIZE, "p_hshks");
		__mem_dump(p_hshkn, SB_HKDF_KEY_NONCE_SIZE, "p_hshkn");
		__mem_dump(p_hmac_key, digest_len, "hmac key");
#endif
#endif
		hkdf_ret = hal_hkdf_hmac_sha256_secure_init(HKDF_CRYPTO_HW_SEL_EN);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac init fail,0x%x\r\n", hkdf_ret);
			goto sb_ram_all_img_hash_chk_f_end;
		}
		hkdf_ret = hal_hkdf_extract_secure_all(extract_sk_idx, extract_wb_idx, p_hshks);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac extract fail,0x%x\r\n", hkdf_ret);
			goto sb_ram_all_img_hash_chk_f_end;
		}
		hkdf_ret = hal_hkdf_expand_secure_all(expand_sk_idx, expand_wb_idx, p_hshkn);
		if (HAL_OK != hkdf_ret) {
			img_vrf_ret = -1;
			DBG_BOOT_ERR("hkdf_hmac expand fail,0x%x\r\n", hkdf_ret);
			goto sb_ram_all_img_hash_chk_f_end;
		}
		sk_cfg = hal_crypto_hmac_sha2_256_get_sk_cfg(KEY_STG_SKTYPE_LD_SK, expand_wb_idx, KEY_STG_WBTYPE_WB_ONLY_BUF, KEY_STG_SK_IDX_NONE);
		p_hmac_key = NULL;
		break;
	default:
		auth_type = AUTH_TYPE_SHA2_256_ALL;
		p_hmac_key = NULL;
		key_len    = 0;
		img_hsh_init_f   = &sb_img_hash_init_wrap_f;
		img_hsh_update_f = &hal_crypto_sha2_256_update;
		img_hsh_final_f  = &hal_crypto_sha2_256_final;
		break;
	}

	if (0x0 != sk_cfg) {
		ret = img_hsh_init_f(p_hmac_key, sk_cfg);
	} else {
		ret = img_hsh_init_f(p_hmac_key, key_len);
	}
	if (ret != SUCCESS) {
		DBG_BOOT_ERR("img hash init,%d\r\n", ret);
		goto sb_ram_all_img_hash_chk_f_end;
	}
	while (msglen_cnt > 0) {
		if (msglen_cnt >= SB_CRYPTO_MAX_MSG_LENGTH) {
			hash_size = SB_CRYPTO_MAX_MSG_LENGTH;
		} else {
			hash_size = msglen_cnt;
		}
		ret = img_hsh_update_f((p_msg + msg_offset), hash_size);
		if (ret != SUCCESS) {
			DBG_BOOT_ERR("img hash update fail,%d\r\n", ret);
			goto sb_ram_all_img_hash_chk_f_end;
		}
		msglen_cnt -= hash_size;
		msg_offset += hash_size;
	}
	ret = img_hsh_final_f(p_digest);
	if (ret != SUCCESS) {
		DBG_BOOT_ERR("img hash final fail,%d\r\n", ret);
	}
	dcache_invalidate_by_addr((void *)p_digest, digest_len);

	if (memcmp(p_img_digest_chk, p_digest, digest_len) == 0) {
		img_vrf_ret = SUCCESS;
		dbg_printf("[img hash chk pass]\r\n");
	} else {
		img_vrf_ret = -2;
		DBG_BOOT_ERR("[img hash chk fail]\r\n");
#if 0
		DBG_BOOT_ERR("img msglen = %d\r\n", msglen);
		__mem_dump(p_img_digest_chk, digest_len, "img_digest_chk");
		__mem_dump(p_digest, digest_len, "img_digest");
#endif
	}
sb_ram_all_img_hash_chk_f_end:
	return img_vrf_ret;
}

int sb_ram_img_vrf_op(const uint8_t sbl_cfg, sec_boot_info_t *p_sb_info, const uint8_t info_type)
{
	int ret = -1;
	otp_boot_cfg3_t sbl = (otp_boot_cfg3_t)sbl_cfg;
	uint8_t tb_en = DISABLE, img_hsh_en = DISABLE;
	img_hsh_en = sbl.bit.img_hsh_en;
	tb_en      = sbl.bit.tb_en;

	RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 0);
	RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 1);
	RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 2);
	RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 3);
	if (ENABLE == tb_en) {  // trusted boot flow
		// pbk hash chk
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 0);
		ret = sb_ram_img_pbk_hash_vrf_f(p_sb_info, info_type);
		if (ret != SUCCESS) {
			return ret;
		}

		// verify sign img
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 1);
		ret = sb_ram_img_sign_vrf_f(p_sb_info);
		if (ret != SUCCESS) {
			return ret;
		}

		// all img hash check
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 2);
		ret = sb_ram_all_img_hash_chk_f(p_sb_info);
		if (ret != SUCCESS) {
			return ret;
		}
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 3);
	} else {
		// all img hash check
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 2);
		ret = sb_ram_all_img_hash_chk_f(p_sb_info);
		if (ret != SUCCESS) {
			return ret;
		}
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 3);
	}
	return ret;
}

uint8_t chk_img_type_id_invalid(const uint16_t type_id)
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

int sec_ram_cfg_f_init(hal_sec_region_cfg_t *psb_sec_cfg_pending)
{
	int ret = SUCCESS;
	if (psb_sec_cfg_pending == NULL) {
		DBG_MISC_ERR("sec_cfg_tbl is NULL\r\n");
		return FAIL;
	}
	psb_sec_cfg_pending->sec_cfg_f.enable_remap_region   = hal_flash_sec_enable_rmp_region;
	psb_sec_cfg_pending->sec_cfg_f.disable_rmp_region    = hal_flash_sec_disable_rmp_region;
	psb_sec_cfg_pending->sec_cfg_f.default_decrypt_init  = hal_flash_sec_default_decrypt_init;
	psb_sec_cfg_pending->sec_cfg_f.decrypt_region_init   = hal_flash_sec_decrypt_region_init;
	psb_sec_cfg_pending->sec_cfg_f.decrypt_region_enable = hal_flash_sec_decrypt_region_enable;
	psb_sec_cfg_pending->sec_cfg_f.disable_dec_region    = hal_flash_sec_disable_dec_region;
	psb_sec_cfg_pending->sec_cfg_f.aes_disable           = hal_flash_sec_aes_disable;
	psb_sec_cfg_pending->sec_cfg_f.set_word_from_byte_bigen = hal_flash_sec_set_word_from_byte_bigen;
	psb_sec_cfg_pending->sec_cfg_f.default_calculate_tag_base = hal_flash_sec_default_calculate_tag_base;
	return ret;
}

int sb_ram_img_dec_rmp(uint8_t *img_phy_addr, uint8_t *enc_rmp_base_addr, sec_boot_info_t *p_sb_info, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	uint8_t *pFw_img_flh = NULL, *pFw_rmp_img_flh = NULL, *pFw_data = NULL;
	uint32_t map_size = 0x0, xip_map_size = 0x0, dec_size = 0x0;
	uint32_t mani_align_offset = 0x0, map_addr = 0xFFFFFFFF, dec_base = 0xFFFFFFFF;
	uint32_t xip_img_fh_phy_offset = 0xFFFFFFFF, xip_map_addr = 0xFFFFFFFF;
	uint32_t base_offset = 0xFFFFFFFF, end_offset = 0xFFFFFFFF;
	uint32_t tag_flash_mem = 0xFFFFFFFF, tag_base = 0xFFFFFFFF;
	uint32_t total_hdr_size = 0x0;
	uint32_t img_fh_phy_offset = 0xFFFFFFFF;
	uint8_t i = 0, ld_sec_enc_idx = 0x0, nxt_sec_enc_idx = 0x0, set_dec = DISABLE, aes_gcm_used = DISABLE;
	uint8_t sec_key_idx = SEC_KEY_IDX1, enc_en = DISABLE, encalg_sel = IMG_SEC_ENC_ALG_AES256_GCM;
	uint8_t xip_img_ld = DISABLE, nxt_xip_img_ld = DISABLE, xip_en = DISABLE, xip_rmp = DISABLE;
	uint32_t iv_ptn_low = 0xFFFFFFFF, iv_ptn_high = 0xFFFFFFFF;
	manif_sec_enc_record_t *p_sec_enc_rd_tbl = NULL;
	hal_sec_region_cfg_t *psb_sec_cfg_pending = NULL;
	uint32_t cur_offset = 0x0;
	fw_img_hdr_t *pfw_hdr = NULL;
	const uint8_t sel_img_load = IMG_LOAD_FW;
	sect_hdr_t *psect_hdr = NULL;
	uint16_t nxt_img_type_id = FW_IMG_NOTSET_ID;
	uint32_t flash_img_offset = SPI_FLASH_BASE;
	uint8_t img_ld_idx = 0x0, rmp_lkp_cnt = 0x0, dec_lkp_cnt = 0x0;
	img_region_lookup_t img_rmp_lkp_tbl[IMG_REGION_LOOKUP_TBL_MAX_SIZE], img_dec_lkp_tbl[IMG_REGION_LOOKUP_TBL_MAX_SIZE];

	RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 4);
	dbg_printf("=== SB IMG Process ===\r\n");
	for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE; i++) {
		img_rmp_lkp_tbl[i].is_xip  = DISABLE;
		img_rmp_lkp_tbl[i].rng_idx = RGN_IDX_INITVAL;
		img_dec_lkp_tbl[i].is_xip  = DISABLE;
		img_dec_lkp_tbl[i].rng_idx = RGN_IDX_INITVAL;
	}
	/* Get sec region control table */
	psb_sec_cfg_pending = (p_sb_info->psec_region_ctrl);

	/* Set FW IMG remap region */
	pFw_img_flh = img_phy_addr;
	img_fh_phy_offset = (uint32_t)img_phy_addr;
	pFw_rmp_img_flh = enc_rmp_base_addr;
	map_addr = (uint32_t)enc_rmp_base_addr;
	mani_align_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);
	map_size = (mani_align_offset + (p_sb_info->img_hash_chk_info.msglen));  // Manifest 4K bytes-aligned + imgsz
	if (map_size & PAGE_OFFSET_MASK) {
		map_size = (((map_size / PAGE_SIZE) + 1) * PAGE_SIZE);
	}
	boot_img_rmp_and_dec_lkp_tbl_insert(&img_rmp_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &rmp_lkp_cnt);
	ret = boot_sec_xip_pending_cfg_add_rmp(psb_sec_cfg_pending, &img_rmp_lkp_tbl[img_ld_idx], img_fh_phy_offset, map_addr, map_size);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = boot_sec_xip_pending_process_rmp(psb_sec_cfg_pending, p_sb_info);
	if (ret != SUCCESS) {
		return ret;
	}
	RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 4);
	/* offset img manifest, get FW IMG header */
	//dbg_printf("[--- FW IMG ---]\r\n");
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
		RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 5);
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 5);
		if (ENABLE == set_dec) { // Cipher IMG handles

			// Check key load or not
			if (DISABLE == (psb_sec_cfg_pending->key_set)) {
				ret = boot_sec_xip_pending_cfg_add_dec_key(psb_sec_cfg_pending, sec_key_idx);
				//xip_ram_pending_cfg_add_dec_key(psb_sec_cfg_pending, sec_key_idx);
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
						goto sb_ram_img_dec_rmp_end;
					}
				} else { // XIP IMG
					if (DISABLE == xip_en) {
						ret = FAIL; // (load wrong sec_enc_record info fail)
						goto sb_ram_img_dec_rmp_end;
					} else {
						/* XIP IMG Remap first
						   XIP Decrypt & remap condition, remap region based on decrypt region(should same as remap region)
						*/

						/* XIP IMG Decrypt region setting */
						base_offset = (p_sec_enc_rd_tbl->base_addr);
						end_offset  = (p_sec_enc_rd_tbl->end_addr);
						if (end_offset & IMG_HDR_START_ALIGN_MASK) {
							end_offset = (((end_offset >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
						}

						dec_base = SPI_FLASH_BASE + base_offset;  // Reference flash base addr start
						dec_size = (end_offset - base_offset);

						/* XIP Remap region setting */
						xip_img_fh_phy_offset = (uint32_t)pFw_data;
						xip_map_addr = (uint32_t)dec_base;
						xip_map_size = dec_size;
						boot_img_rmp_and_dec_lkp_tbl_insert(&img_rmp_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &rmp_lkp_cnt);
						ret = boot_sec_xip_pending_cfg_add_rmp(psb_sec_cfg_pending, &img_rmp_lkp_tbl[img_ld_idx], xip_img_fh_phy_offset, xip_map_addr, xip_map_size);
						if (ret != SUCCESS) {
							return ret;
						}
						ret = boot_sec_xip_pending_process_rmp(psb_sec_cfg_pending, p_sb_info);
						if (ret != SUCCESS) {
							return ret;
						}
					}
				}

				/* Set encalg_sel */
				encalg_sel  = (p_sec_enc_rd_tbl->encalg_sel);
				if (IMG_SEC_ENC_ALG_AES256_GCM == encalg_sel) {
					if (DISABLE == aes_gcm_used) {
						aes_gcm_used = ENABLE;
					} else {
						if (ENABLE == xip_img_ld) {
							ret = FAIL; // (Cannot allow multiple XIP IMGs use AES GCM)
							goto sb_ram_img_dec_rmp_end;
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
				boot_img_rmp_and_dec_lkp_tbl_insert(&img_dec_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &dec_lkp_cnt);
				ret = boot_sec_xip_pending_cfg_add_dec(psb_sec_cfg_pending, &img_dec_lkp_tbl[img_ld_idx], encalg_sel, dec_base, dec_size, iv_ptn_low, iv_ptn_high,
													   tag_base, tag_flash_mem, sizeof(fw_img_hdr_t));
				if (ret != SUCCESS) {
					return ret;
				}
				ret = boot_sec_xip_pending_process_dec(psb_sec_cfg_pending, p_sb_info);
				//xip_ram_pending_process_dec(psb_sec_cfg_pending, p_sb_info);
				if (ret != SUCCESS) {
					return ret;
				}
			}

		}

		RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 6);
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 6);
		/*
		   Chk IMG valid or not from imt hdr type id
		   - valid , prepare load sections and check next img or not and next img type id is XIP IMG or not
		     > if XIP IMG, then record XIP IMG is the next IMG load
		   - invalid, goto boot fail(decrypt fail)
		*/
		//dbg_printf("[--- IMG HDR ---]\r\n");
		pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
		if (ENABLE == xip_img_ld) { // XIP cipher IMG already remap
			pFw_data = (uint8_t *)xip_map_addr;
			boot_load_img_hdr_f((const uint8_t *)pFw_data, pfw_hdr, sel_img_load);

		}
		boot_load_img_hdr_f((const uint8_t *)pFw_data, pfw_hdr, sel_img_load);
		if (chk_img_type_id_invalid((pfw_hdr->type_id))) {
			ret = FAIL; // (invalid, colud get wrong addr or decrypt fail)
			goto sb_ram_img_dec_rmp_end;
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
		if (pfw_hdr->type_id == FW_IMG_VOE_ID) {
			pFw_data += sizeof(fw_img_hdr_t);
			flash_img_offset = (uint32_t)pFw_data;

			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 11);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 11);
			// Init DDR for VOE memcpy
			if (boot_init_flags.b.psram_inited == 0) {
				bootload_dram_init();
				boot_init_flags.b.psram_inited = 1;
			}
			dbg_printf("VOE flash @ 0x%x, 0x%x\r\n", pFw_data, (pfw_hdr->imglen));
#if CONFIG_BOOT_LD_VOE_CTRL
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 12);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 12);
			// Debug VOE Flash IMG data
			// __mem_dump (pFw_data, 128, "VOE flash_img_offset_dump:");
			hal_video_load_fw((voe_cpy_t)memcpy, (int *)flash_img_offset, (int *)DDR_FOR_VOE_IMG_LOAD_ADDR);
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 14);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 14);
			dbg_printf("=== Process VOE IMG ===\r\n");
#endif
		} else if (pfw_hdr->type_id == FW_IMG_XIP_ID) {
			dbg_printf("=== Process FW_IMG_XIP ===\r\n");
			if (DISABLE == xip_en) {
				/* only set XIP IMG Remap */
				xip_img_fh_phy_offset = (uint32_t)pFw_data;
				psect_hdr = (sect_hdr_t *)(pFw_data + sizeof(fw_img_hdr_t));
				xip_map_addr = (uint32_t)(psect_hdr->dest);
				if (xip_map_addr & PAGE_OFFSET_MASK) {
					xip_map_addr = (xip_map_addr & PAGE_ALIGN_MASK);  // move from the img hdr start(4K byte-aligned)
				}
				xip_map_size = ((pfw_hdr->imglen) + sizeof(fw_img_hdr_t));
				if (xip_map_size & PAGE_OFFSET_MASK) {
					xip_map_size = (((xip_map_size / PAGE_SIZE) + 1) * PAGE_SIZE);
				}
				boot_img_rmp_and_dec_lkp_tbl_insert(&img_rmp_lkp_tbl[img_ld_idx], IMG_REGION_LOOKUP_TBL_MAX_SIZE, xip_img_ld, &rmp_lkp_cnt);
				ret = boot_sec_xip_pending_cfg_add_rmp(psb_sec_cfg_pending, &img_rmp_lkp_tbl[img_ld_idx], xip_img_fh_phy_offset, xip_map_addr, xip_map_size);
				if (ret != SUCCESS) {
					return ret;
				}
				ret = boot_sec_xip_pending_process_rmp(psb_sec_cfg_pending, p_sb_info);
				//xip_ram_pending_process_rmp(psb_sec_cfg_pending, p_sb_info);
				if (ret != SUCCESS) {
					return ret;
				}
			}

		} else {
			if (((pfw_hdr->str_tbl) != 0x0) &&
				((pfw_hdr->str_tbl) != 0xFFFFFFFF)) {
				if (pfw_hdr->type_id == FW_IMG_FWHS_S_ID) {
					*pram_start_func = (RAM_FUNCTION_START_TABLE *)(pfw_hdr->str_tbl);
					RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 19);
				}
				dbg_printf("[Image Start Table @ 0x%x]\r\n", *pram_start_func);
			}
		}

		RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 7);
		RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 7);
		if ((pfw_hdr->type_id != FW_IMG_XIP_ID) &&
			(pfw_hdr->type_id != FW_IMG_ISP_ID) &&
			(pfw_hdr->type_id != FW_IMG_VOE_ID)) {
			pFw_data += sizeof(fw_img_hdr_t);
			fw_load_img_sect_f((const uint8_t *)pFw_data, pfw_hdr);
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
							boot_sec_xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[img_ld_idx].rng_idx));
							boot_img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[img_ld_idx], &dec_lkp_cnt);
							if (ENABLE == aes_gcm_used) {
								aes_gcm_used = DISABLE;
							}
							nxt_sec_enc_idx = (++ld_sec_enc_idx);
						}
					} else {
						nxt_sec_enc_idx = (++ld_sec_enc_idx);
					}
				} else {    // next IMG is XIP IMG
					if (ENABLE == set_dec) {
						// disable all decrypt region setting and current remap setting
						for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE; i++) {
							if ((img_dec_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_DEC_REGION_CFG) &&
								(DISABLE == (img_dec_lkp_tbl[i].is_xip))) {
								boot_sec_xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[i].rng_idx));
								boot_img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[i], &dec_lkp_cnt);
							}
							if ((img_rmp_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_RMP_REGION_CFG) &&
								(DISABLE == (img_rmp_lkp_tbl[i].is_xip))) {
								boot_sec_xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
								boot_img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
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
								boot_sec_xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
								boot_img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
							}
						}
						//xip_ram_disable_rmp_config(psb_sec_cfg_pending, ld_sec_enc_idx);
						nxt_sec_enc_idx = (++ld_sec_enc_idx);
					}
				}
			} else {    // current IMG is XIP IMG
				if (nxt_img_type_id != FW_IMG_XIP_ID) { // next IMG is Non XIP IMG
					// Cannot allow other Non-XIP IMGs after XIP IMGs
					ret = FAIL;
					goto sb_ram_img_dec_rmp_end;
				} else {
					nxt_sec_enc_idx = (++ld_sec_enc_idx);
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
			if (ENABLE == nxt_xip_img_ld) {
				pFw_data = (uint8_t *)pFw_img_flh + (uint32_t)(pfw_hdr->nxtoffset);
				//dbg_printf("nxt_xip_img_ld:%d\r\n", nxt_xip_img_ld);
			} else {
				pFw_data = (uint8_t *)pFw_rmp_img_flh + (uint32_t)(pfw_hdr->nxtoffset);
				//dbg_printf("nxt_xip_img_ld not xip img\r\n");
			}
		} else {
			break;
		}
	}

	// Disable all current decrypt region setting and current remap setting except XIP IMGs
	for (i = 0; i < IMG_REGION_LOOKUP_TBL_MAX_SIZE ; i++) {
		if ((img_dec_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_DEC_REGION_CFG) &&
			(DISABLE == (img_dec_lkp_tbl[i].is_xip))) {
			boot_sec_xip_disable_dec_config(psb_sec_cfg_pending, (img_dec_lkp_tbl[i].rng_idx));
			boot_img_rmp_and_dec_lkp_tbl_remove(&img_dec_lkp_tbl[i], &dec_lkp_cnt);
		}
		if ((img_rmp_lkp_tbl[i].rng_idx < MAX_PENDING_SEC_RMP_REGION_CFG) &&
			(DISABLE == (img_rmp_lkp_tbl[i].is_xip))) {
			boot_sec_xip_disable_rmp_config(psb_sec_cfg_pending, (img_rmp_lkp_tbl[i].rng_idx));
			boot_img_rmp_and_dec_lkp_tbl_remove(&img_rmp_lkp_tbl[i], &rmp_lkp_cnt);
		}
	}

sb_ram_img_dec_rmp_end:
	return ret;
}

int verify_fw_manif_f(const uint8_t *img_offset, const uint8_t info_type, sec_boot_info_t *p_sb_info)
//const uint8_t *img_offset, sec_boot_keycerti_t *psb_keycerti)
{
	int ret = SUCCESS;
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	volatile uint8_t sbl_cfg = 0x0;
	uint8_t tb_en = DISABLE, img_hsh_en = DISABLE;

	// Get secure boot level config
	sbl_cfg     = potp_boot_cfg->byte.cfg3.byte;
	img_hsh_en  = potp_boot_cfg->byte.cfg3.bit.img_hsh_en;
	tb_en       = potp_boot_cfg->byte.cfg3.bit.tb_en;

	// ROM codes re-used, RAM codes can patch
	ret = boot_verify_manif_f(img_offset, info_type, p_sb_info);

	if (((ENABLE == tb_en) || (ENABLE == img_hsh_en)) &&
		(FW_ISP_INFO != info_type)) {
		//DBG_INFO_MSG_ON(_DBG_EDDSA_);

		ret = sb_ram_img_vrf_op(sbl_cfg, p_sb_info, info_type);
		if (SUCCESS != ret) {
			goto verify_fw_manif_f_end;
		}
	}

verify_fw_manif_f_end:
	return ret;
}

// Flash SEC simu
uint8_t flash_sector_erase(hal_spic_adaptor_t *adtr, uint32_t flash_mem_erase)
{
	if (flash_mem_erase & PAGE_OFFSET_MASK) {
		dbg_printf("Error erase addr not aligned: %x, skipped\r\n", flash_mem_erase);
		return FAIL;
	}

	// No write to first and 2nd page
	if (flash_mem_erase == 0x08000000 || flash_mem_erase == 0x8001000) {
		dbg_printf("Do not erase flash calibration data! 0x%x skipped\r\n");
		return FAIL;
	}
	if (flash_mem_erase < FLASH_ACCESS_BASE || flash_mem_erase > FLASH_MAX_END) {
		dbg_printf("Do not erase non-flash mem! 0x%x\r\n", flash_mem_erase);
		return FAIL;
	}
	dbg_printf("Erased flash addr:0x%08x !\r\n", flash_mem_erase);

	hal_flash_sector_erase(adtr, flash_mem_erase);
	return SUCCESS;
}

// Flash SEC simu
static int sec_flash_burst_write_buf(uint32_t flash_mem, uint32_t size, const uint8_t *w_buf)
{
	hal_status_t ret = HAL_OK;
	uint32_t flash_mem_aligned, flash_mem_erase, erase_cnt, i;
	uint32_t erase_size;

	// start to program Flash
	// Erase Flash memory first, must 4k bytes-aligned
	flash_mem_aligned = flash_mem & PAGE_ALIGN_MASK;

	if (flash_mem_aligned != flash_mem) {
		dbg_printf("Try to write flash with non-aligned addr 0x%x\r\n", flash_mem);
	}
	erase_size = size + flash_mem - flash_mem_aligned;
	if (erase_size & PAGE_OFFSET_MASK) {
		erase_cnt = (erase_size / (4 * 1024)) + 1;
		dbg_printf("Try to write flash with non-page size 0x%x\r\n", erase_size);
	} else {
		erase_cnt = (erase_size / (4 * 1024));
	}
	for (i = 0; i < erase_cnt; i++) {
		flash_mem_erase = (flash_mem_aligned + PAGE_SIZE * i) & PAGE_ALIGN_MASK;
		// erase func will guard the writing range
		ret = flash_sector_erase(&_hal_spic_adaptor, flash_mem_erase);
		if (SUCCESS != ret) {
			dbg_printf("Failed to erase 0x%x\r\n", flash_mem_erase);
			return FAIL;
		}
	}

#if 0
	if (size > FLASH_SEC_TEST_MSG_SIZE_MAX) {
		size = FLASH_SEC_TEST_MSG_SIZE_MAX;
	}
#endif
	hal_flash_burst_write(&_hal_spic_adaptor, size, flash_mem, (u8 *)w_buf);

	// Clean dcache
	dcache_invalidate_by_addr((void *)flash_mem, size);
	return ret;
}

// Flash SEC simu
static void calculate_tag_base(uint32_t flash_addr, uint32_t region_size, uint32_t tag_region_addr,
							   uint32_t cache_line_size, uint32_t tag_size, uint32_t *tag_region_size, uint32_t *tag_base)
{
	// check divide of cache line and tag
	if (cache_line_size % tag_size != 0) {
		dbg_printf("calculate_tag_base arg error: cache %% tag != 0");
		return;
	}
	uint32_t ratio = cache_line_size / tag_size;
	if (NULL != tag_region_size) {
		*tag_region_size = region_size / ratio;
	}

	uint32_t tag_offset = (flash_addr & FLASH_OFFSET_MASK) / ratio;
	if (NULL != tag_base) {
		*tag_base = tag_region_addr - tag_offset;
	}
}

void fw_load_img_sect_f(const uint8_t *img_offset, fw_img_hdr_t *pfw_hdr)
{
	uint8_t *ptr = (uint8_t *)img_offset;
	sect_hdr_t *psect_hdr = NULL;
	uint8_t *pimg_load_dest = NULL;
	uint32_t img_load_size = 0;
	uint32_t flash_img_offset = SPI_FLASH_BASE;

	while ((uint8_t *)ptr != (uint8_t *)SECT_HDR_NXTOFFSET_NULL) {
		//dbg_printf("[--- IMG Section HDR ---]\r\n");
		psect_hdr = (psect_hdr_t)&tmp_sect_hdr[0];
		memcpy((void *)psect_hdr, (void *)ptr, sizeof(sect_hdr_t));

#if FLASH_FW_BOOTLOAD_DBG
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
		flash_img_offset = (uint32_t *)(ptr);
		pimg_load_dest = (psect_hdr->dest);
		img_load_size = (psect_hdr->seclen);
		// load section data from flash to assigned memory destination
		//dbg_printf("[--- IMG Section Raw Data ---]\r\n");
		switch (psect_hdr->type_id) {
		case FW_SIMG_DTCM_ID:
		case FW_SIMG_ITCM_ID:
		case FW_SIMG_SRAM_ID:
			boot_flash_read(pimg_load_dest, (flash_img_offset - SPI_FLASH_BASE), img_load_size);
			dcache_clean_by_addr((uint32_t *)pimg_load_dest, img_load_size);
			dbg_printf("RAM Load @ 0x%x->0x%x, 0x%x\r\n", flash_img_offset, pimg_load_dest, img_load_size);
#if FLASH_FW_BOOTLOAD_DBG
			if ((get_4byte((pimg_load_dest + 4)) >= RAM_FUN_TABLE_VALID_START_ADDR) &&
				(get_4byte((pimg_load_dest + 4)) < RAM_FUN_TABLE_VALID_END_ADDR)) {
				dbg_printf("[--- Section Data Pre-Parse ---]\r\n");
				dbg_printf("[--- Section Data(#gRamStartFun) ---]\r\n");
				dbg_printf("  #Signature addr : 0x%08x\r\n", get_4byte((pimg_load_dest)));
				dbg_printf("  #RamStartFun addr : 0x%08x\r\n", get_4byte((pimg_load_dest + 4)));
				dbg_printf("  #RamWakeupFun addr : 0x%08x\r\n", get_4byte((pimg_load_dest + 8)));
				dbg_printf("  #RamPatchFun0 addr : 0x%08x\r\n", get_4byte((pimg_load_dest + 12)));
				dbg_printf("  #RamPatchFun1 addr : 0x%08x\r\n", get_4byte((pimg_load_dest + 16)));
			}
			__mem_dump(pimg_load_dest, 64, "#Section raw data");
#endif
			break;
		case FW_SIMG_DDR_ID:
			// Init DDR
			if (boot_init_flags.b.psram_inited == 0) {
				bootload_dram_init();
				boot_init_flags.b.psram_inited = 1;
			}
			dbg_printf("DDR Load @ 0x%x->0x%x, 0x%x\r\n", flash_img_offset, pimg_load_dest, img_load_size);
			boot_flash_read(pimg_load_dest, (flash_img_offset - SPI_FLASH_BASE), img_load_size);
			dcache_clean_by_addr((uint32_t *)pimg_load_dest, img_load_size);
			break;
		case FW_SIMG_XIP_ID:
			dbg_printf("XIP_IMG Set @ 0x%x->0x%x, 0x%x\r\n", flash_img_offset, pimg_load_dest, pimg_load_dest);
			break;
		}
		if ((psect_hdr->nxtoffset) != (uint32_t)SECT_HDR_NXTOFFSET_NULL) {
			ptr = (uint8_t *)pimg_start + (uint32_t)(psect_hdr->nxtoffset);
		} else {
			break;
		}
	}
}

int fw_load_fw_f(const uint8_t *img_offset, PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = SUCCESS;
	uint8_t *ptr = (uint8_t *)img_offset;
	fw_img_hdr_t *pfw_hdr = NULL;
	const uint8_t sel_img_load = IMG_LOAD_FW;
	uint32_t flash_img_offset = SPI_FLASH_BASE;

	if (!ptr) {
		ret = FAIL;
		goto fw_load_fw_f_end;
	}

	// Confirm FW raw data
	while ((uint8_t *)ptr != (uint8_t *)FW_HDR_NXTOFFSET_NULL) {
		//dbg_printf("[--- IMG HDR ---]\r\n");
		pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
		boot_load_img_hdr_f((const uint8_t *)ptr, pfw_hdr, sel_img_load);

		if (pfw_hdr->type_id == FW_IMG_XIP_ID) {
			dbg_printf("=== Handle FW_IMG_XIP ===\r\n");
		} else if (pfw_hdr->type_id == FW_IMG_ISP_ID) {
			ptr += sizeof(fw_img_hdr_t);
			flash_img_offset = (uint32_t)ptr;
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 8);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 8);
			dbg_printf("ISP_IQ @ 0x%x, 0x%x\r\n", flash_img_offset, (pfw_hdr->imglen));
#if CONFIG_BOOT_LD_VOE_CTRL
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 9);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 9);
			// Debug ISP_IQ Flash IMG data
			//__mem_dump (ptr, 128, "ISP_IQ flash_img_offset_dump:");
			// Todo: boot loader handle sensor bin
			//hal_rtl_voe_fcs_process((voe_cpy_t)memcpy, flash_img_offset);
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 10);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 10);
			dbg_printf("=== Process ISP_IQ ===\r\n");
#endif
		} else if (pfw_hdr->type_id == FW_IMG_VOE_ID) {
			ptr += sizeof(fw_img_hdr_t);
			flash_img_offset = (uint32_t)ptr;

			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 11);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 11);
			// Init DDR for VOE memcpy
			if (boot_init_flags.b.psram_inited == 0) {
				bootload_dram_init();
				boot_init_flags.b.psram_inited = 1;
			}
			dbg_printf("VOE flash @ 0x%x, 0x%x\r\n", flash_img_offset, (pfw_hdr->imglen));
#if CONFIG_BOOT_LD_VOE_CTRL
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 12);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 12);
			// Debug VOE Flash IMG data
			// __mem_dump (ptr, 128, "VOE flash_img_offset_dump:");
			while (1) {
				if (hal_voe_fcs_check_km_run_done()) {
					break;
				}
			}
			dbg_printf("=== ISP FCS Done ===\r\n");
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 13);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 13);
			hal_video_load_fw((voe_cpy_t)memcpy, (int *)flash_img_offset, (int *)DDR_FOR_VOE_IMG_LOAD_ADDR);
			//hal_voe_fcs_set_voe_fm_load_flag_final();
			RAM_FOOTPH_CLR(BOOT_INFO_IDX1, 14);
			RAM_FOOTPH_STORE(BOOT_INFO_IDX1, 14);
			dbg_printf("=== Process VOE IMG ===\r\n");
#endif
		} else {
			if ((pfw_hdr->str_tbl) != 0xFFFFFFFF) {
				if (pfw_hdr->type_id == FW_IMG_FWHS_S_ID) {
					*pram_start_func = (RAM_FUNCTION_START_TABLE *)(pfw_hdr->str_tbl);
					RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 28);
				}
				dbg_printf("[Image Start Table @ 0x%x]\r\n", *pram_start_func);
			}
		}
		// Tmp ignore XIP IMG
		if ((pfw_hdr->type_id != FW_IMG_XIP_ID) &&
			(pfw_hdr->type_id != FW_IMG_ISP_ID) &&
			(pfw_hdr->type_id != FW_IMG_VOE_ID)) {
			ptr += sizeof(fw_img_hdr_t);
			fw_load_img_sect_f((const uint8_t *)ptr, pfw_hdr);
		}
		if ((pfw_hdr->nxtoffset) != (uint32_t)FW_HDR_NXTOFFSET_NULL) {
			ptr = (uint8_t *)pimg_start + (uint32_t)(pfw_hdr->nxtoffset);
		} else {
			break;
		}
	}

fw_load_fw_f_end:
	return ret;
}

void chk_load_fw_idx(part_record_t *pfw_img_record, uint8_t *pld_fw_idx_user)
{

	uint16_t fw_pt_type_id = FW_PT_FW1_ID;
	fw_pt_type_id = (pfw_img_record->type_id);
	if (FW_PT_FW2_ID == fw_pt_type_id) {
		*pld_fw_idx_user = 2;
	} else {
		*pld_fw_idx_user = 1;
	}
}

int32_t boot_load_tlv(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	hal_status_t ret = HAL_OK;
	int32_t crypt_ret = SUCCESS;
	int32_t boot_ret = SUCCESS;
	int load_fw_idx = 0;
	uint8_t ld_fw_idx_user = 0;
	sec_boot_keycerti_t *pkey_certi = NULL;
	partition_tbl_t *ppartition_tbl = NULL;
	sec_boot_info_t *p_sb_info = NULL;
	hal_sec_region_cfg_t *psb_sec_cfg_pending = NULL;
	part_record_t *pfw_img_record = NULL;
	uint8_t *pFw_img_flh = NULL, *pFw_data = NULL;
	uint32_t cur_offset = 0x0;
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	volatile uint8_t sbl_cfg = 0x0;
	uint8_t tb_en = DISABLE, sb_en = DISABLE, img_hsh_en = DISABLE, img_obj = 0x0;

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 23);
	// Get secure boot level config
	sbl_cfg    = potp_boot_cfg->byte.cfg3.byte;
	img_hsh_en = potp_boot_cfg->byte.cfg3.bit.img_hsh_en;
	tb_en      = potp_boot_cfg->byte.cfg3.bit.tb_en;
	sb_en      = potp_boot_cfg->byte.cfg3.bit.sb_en;

	crypt_ret = hal_crypto_engine_init();
	if (crypt_ret != SUCCESS) {
		DBG_MISC_ERR("boot_load: Crypto Init Failed!%d\r\n", crypt_ret);
		boot_ret = FAIL;
		goto boot_load_tlv_end;
	}

	// Init sb info buffer
	p_sb_info = &sb_ram_info;
	memset(p_sb_info, 0x0, sizeof(sec_boot_info_t));

	// Get export Key certificate ram buffer
	pkey_certi = (sec_boot_keycerti_t *)boot_get_key_certi();
	p_sb_info->psb_keycerti = pkey_certi;

	// Get export partition table ram buffer
	ppartition_tbl = (partition_tbl_t *)boot_get_tlv_partition_tbl();
	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 24);

	// Init sb_sec_cfg_pending
	psb_sec_cfg_pending = &sb_ram_sec_cfg_pending;
	memset(psb_sec_cfg_pending, 0x0, sizeof(hal_sec_region_cfg_t));
	sec_ram_cfg_f_init(psb_sec_cfg_pending);
	p_sb_info->psec_region_ctrl = psb_sec_cfg_pending;

	// Get ISP_IQ img record from partition table
	uint8_t load_isp_iq_idx;
	load_isp_iq_idx = ppartition_tbl->fst.iq_idx;
	pfw_img_record = (part_record_t *)(((part_record_t *)&ppartition_tbl->partition_record[0]) + load_isp_iq_idx);
	if (ENABLE == pfw_img_record->valid) {
		pFw_img_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + pfw_img_record->start_addr);
		pimg_start = pFw_img_flh;
		//dbg_printf("pFw_img_flh=0x%08x\r\n", pFw_img_flh);
		// Need a simple verify ISP IQ img manifest
		boot_ret = verify_fw_manif_f(pFw_img_flh, FW_ISP_INFO, p_sb_info);
		if (boot_ret != SUCCESS) {
			goto boot_load_tlv_end;
		}
		RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 25);
		// Get ISP_IQ raw data
		dbg_printf("=== Load ISP_IQ Sensor ===\r\n");
		cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
		pFw_img_flh += cur_offset;
		pFw_data = pFw_img_flh;
		ret = fw_load_fw_f(pFw_data, pram_start_func);
		if (ret != SUCCESS) {
			goto boot_load_tlv_end;
		}
		dbg_printf("=== Load Done ===\r\n");
	}

	// Select the latest valid boot img to load
	img_obj = LD_SEL_IMG_FW;
	load_fw_idx = boot_img_sel_op_idx(ppartition_tbl, img_obj, IMG_SEL_LD);
	if (((-1) == load_fw_idx) || (load_fw_idx > PARTITION_RECORD_MAX)) {
		DBG_BOOT_ERR("load_fw_idx err:%d\r\n", load_fw_idx);
		boot_ret = FAIL;
		goto boot_load_tlv_end;
	}

	// Get FW img from partition table record
	pfw_img_record = (part_record_t *)(((part_record_t *)&ppartition_tbl->partition_record[0]) + load_fw_idx);
	if (((pfw_img_record->type_id) == FW_PT_FW1_ID) ||
		((pfw_img_record->type_id) == FW_PT_FW2_ID)) {
		if ((pfw_img_record->valid) == INFO_VALID) {
			// Get fw img flash memory
			pFw_img_flh = (uint8_t *)((uint32_t)SPI_FLASH_BASE + pfw_img_record->start_addr);
		} else {
			boot_ret = FAIL;
			goto boot_load_tlv_end;
		}
	} else {
		boot_ret = FAIL;
		goto boot_load_tlv_end;
	}

	chk_load_fw_idx(pfw_img_record, &ld_fw_idx_user);
	pimg_start = pFw_img_flh;
	//dbg_printf("pFw_img_flh=0x%08x\r\n", pFw_img_flh);

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 26);
	dbg_printf("=== Load FW%u ===\r\n", ld_fw_idx_user);
	// Verify fw img manifest
	boot_ret = verify_fw_manif_f(pFw_img_flh, FW_IMG_INFO, p_sb_info);
	if (boot_ret != SUCCESS) {
		goto boot_load_tlv_end;
	}

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 27);
	// Decrypt fw img
	if (ENABLE == sb_en) {
		/* Set remap region base */
		if ((p_sb_info->pimg_sec_enc_info->enc_rmp_base_addr) != 0xFFFFFFFF) {
			pimg_start = (p_sb_info->pimg_sec_enc_info->enc_rmp_base_addr);
		} else {
			boot_ret = FAIL;
			dbg_printf("=== SB FW fail ===\r\n");
			goto boot_load_tlv_end;
		}

		/* Set remap region & decrypt region */
		boot_ret = sb_ram_img_dec_rmp(pFw_img_flh, pimg_start, p_sb_info, pram_start_func);
		if (SUCCESS != boot_ret) {
			goto boot_load_tlv_end;
		} else {
			dbg_printf("=== SB FW pass ===\r\n");
		}
	} else {
		// Get FW plaintext raw data
		//dbg_printf("[--- FW IMG ---]\r\n");
		cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
		pFw_img_flh += cur_offset;
		pFw_data = pFw_img_flh;
		ret = fw_load_fw_f(pFw_data, pram_start_func);
		if (ret != SUCCESS) {
			goto boot_load_tlv_end;
		}
	}
	dbg_printf("=== FW Load Done ===\r\n");
boot_load_tlv_end:
	hal_crypto_engine_deinit();
	return boot_ret;
}

/**
 *  @brief The boot loader entry function. It will do memory initialization,
 *         load FW image from flash to RAM and then jump
 *         to the application start function.
 *
 *  @returns void
 */
void boot_start(void)
{
	uint32_t sys_timer_id;
	int32_t ret;
	hal_spic_adaptor_t *phal_spic_adaptor = &_hal_spic_adaptor;
#if CONFIG_FPGA
	uint8_t spic_io_mode = SpicOneIOMode;   // To avoid DD bitfile Flash timing not stable issue
#else   // pxp & ASIC
	uint8_t spic_io_mode = SpicQpiMode;
#endif
	uint8_t boot_pin_sel = SpicQuadIOPin;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	volatile uint8_t imgft_cfg = LD_IMF_FT_TLV;
	u8 rom_snand_boot = 0;
//    symb_ns4s_t *symb_ns4s_stubs=(symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);

	boot_start_bss_data_init();
	boot_start_secure_bss_clean();

	// TODO: get CPU clock for register
//    SystemCoreClock = CONFIG_CPU_CLK;

	hal_vector_table_init((uint32_t)&__StackTop, ram_vector_table);

	boot_init_flags.w = 0;

	// Disable AON WDT & VNDR WDT(VNDR can be operated untill ram start)
	hal_wdt_all_disable();
	boot_init_flags.b.wdt_disabled = 1;

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 22);
	/* Configure MPU regions: Regions configuration is in "mpu_config.h"
	// Region0: RAM as non-cacheable for DMA buffer
	// Region1: Flash calibration data cannot be cacheable
	*/
	mpu_init();
#if CONFIG_BOOTLOAD_DCACHE_EN
	dcache_enable();
#endif

	// re-initial system timer
	hal_timer_clock_init(1, ENABLE);
	hal_timer_group_init(&timer_group1, 1);  // Use TimerGroup 1
	hal_timer_group_sclk_sel(&timer_group1, GTimerSClk_4M);
	hal_start_systimer(&system_timer, CONFIG_SYS_TIMER_ID, GTimerCountUp, CONFIG_SYS_TICK_TIME, 1);

	// for ignore-secure condition, default use non-secure region G-Timer HAL, since PWM needs it,
	// but the secure region ROM code still needs the system timer to be initialed
	*(__rom_stubs_hal_timer_s.ppsys_timer) = &system_timer;

	hal_pinmux_manager_init(&pinmux_manager);
	// Re-initial UART for debugging message
	log_uart_port_init(STDIO_UART_TX_PIN, STDIO_UART_RX_PIN, (uint32_t)115200);
	ConfigDebugErr = 0xFFFFFFFF;
	dbg_printf("\r\n== Boot Loader ==\r\n");
	dbg_printf("%s:%s\r\n", __DATE__, __TIME__);

	hal_gdma_group_init(&hal_gdma_group);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	// Enable DDR Power
	hal_pon_gpio_comm_init(&gpio_comm);
	memset((void *)&ddr_en_gpio, 0, sizeof(hal_gpio_adapter_t));
	hal_gpio_init(&ddr_en_gpio, DDR_EN_PIN);
	hal_gpio_set_dir(&ddr_en_gpio, GPIO_OUT);
	hal_gpio_write(&ddr_en_gpio, 1);
	hal_delay_ms(2);
#endif

#if ! IS_CUT_TEST(CONFIG_CHIP_VER)
	rom_snand_boot = *(hal_snand_boot_stubs.rom_snand_boot);
	imgft_cfg = potp_boot_cfg7->bit.ntlv_img_ld_en;     //tlv not complete
#endif

	if (rom_snand_boot == 1) {
		dbg_printf("BOOT_FROM_NANDFLASH\r\n");
		if (LD_IMF_FT_TLV == imgft_cfg) {
			ret = snand_boot_loader(&pRamStartFun);
		} else {
			ret = FAIL;
		}
	} else {
		ret = spic_init(phal_spic_adaptor, spic_io_mode, boot_pin_sel /*SpicDualIOPin*/);
		if (ret != HAL_OK) {
			DBG_MISC_ERR("boot_load: SPIC Init Failed!\r\n");
			goto __boot_start_end;
		}
		boot_init_flags.b.flash_inited = 1;

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		ret = boot_load_no_sb(&pRamStartFun);
#else
		if (LD_IMF_FT_TLV == imgft_cfg) {
			ret = boot_load_tlv(&pRamStartFun);
		} else {
			ret = boot_load_no_sb(&pRamStartFun);
		}
#endif

#if IS_CUT_TEST(CONFIG_CHIP_VER) // FIXME remove this condition check
		// Deinit spic, it will be re-init at ram_start
		hal_flash_return_spi(&_hal_spic_adaptor);
#endif
	}

	RAM_FOOTPH_STORE(BOOT_INFO_IDX0, 29);
	if (SUCCESS == ret) {
		boot_start_secure_bss_clean();
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		boot_clear_partition_tbl();
#else
		if (LD_IMF_FT_TLV == imgft_cfg) {
			boot_clear_partition_tbl_info();
			boot_clear_keycerti_info();
		} else {
			boot_clear_partition_tbl();
		}
#endif
		if (chk_fw_img_signature(pRamStartFun->Signature)) {
			DBG_BOOT_ERR("Invalid FW Image Sigature\r\n");
		} else {
			pRamStartFun->boot_cfg_w = gRamStartFun.boot_cfg_w;
			pRamStartFun->boot_status.byte = gRamStartFun.boot_status.byte;

			const uint32_t er_code_start = &__fw_img_start__, er_code_end = &__ram_code_rodata_end__;
			const uint32_t er_size = (er_code_end - er_code_start);

			// Erase boot loader code and then jum to Image2, Jump to ROM code to do this
//          dbg_printf("Erase Img1 0x%x ~ 0x%x\r\n", er_code_start, er_code_end);
			dbg_printf("\r\nBoot Loader <==\r\n");
			DBG_ERR_MSG_OFF(0xFFFFFFFF);
			hal_uart_deinit(&log_uart);
			stdio_port_deinit();
			hal_pinmux_rom_info_manage(&pinmux_manager, OP_BACKUP, ALL_LOG);

			/* Disable MPU */
			MPU->CTRL = 0;
			__DSB();
			__ISB();
			boot_loader_erase(er_code_start, er_size, (uint32_t)pRamStartFun);
		}
	} else {
		boot_start_secure_bss_clean();
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		boot_clear_partition_tbl();
#else
		if (LD_IMF_FT_TLV == imgft_cfg) {
			boot_clear_partition_tbl_info();
			boot_clear_keycerti_info();
		} else {
			boot_clear_partition_tbl();
		}
#endif
		boot_clear_partition_tbl();
		DBG_BOOT_ERR("Boot Load Err!\r\n");
	}

__boot_start_end:
#if 1
	// for boot loader debugging only
	shell_cmd_init();
	shell_set_prompt("$boot>");

	while (1) {
		shell_task();
	}
#else
	while (1) {
		__NOP();
	}
#endif
}

#endif  // end of "#if defined(CONFIG_BUILD_BOOT) && defined(CONFIG_BUILD_RAM)"

