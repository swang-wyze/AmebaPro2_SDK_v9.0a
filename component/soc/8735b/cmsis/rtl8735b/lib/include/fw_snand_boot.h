/**************************************************************************//**
 * @file     fw_snand_boot.h
 * @brief    Declare the booting from nand flash.
 *
 * @version  V1.00
 * @date     2021-07-26
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

#ifndef _FW_SNAND_BOOT_H_
#define _FW_SNAND_BOOT_H_

#include "fw_img_tlv.h"
#include "rtl8735b_ramstart.h"

#define NAND_PAGE_MAX_LEN           (2 * 0x800)

#define IMG_HDR_MAX                 64
#define SECT_HDR_MAX                64

/// Fixed ctrl info start blk
#define NAND_CTRL_INFO_START_BLK    0
/// Fixed ctrl info backup count
#define NAND_CTRL_INFO_DUP_CNT      8
/// Max count of virtual block map
#define SNAND_VMAP_MAX              48
/// SNAND start block for debug
#define DBG_SNAND_START_BLK         0

#define SNAND_PAR_RECORD_MAG_NUM    0xFF35FF87

#define NAND_SPARE_TYPE_IDX         4

#define SNAND_ADDR_INIT(PAGE, COL)  { .page = PAGE, .col = COL}
#define SNAND_ADDR_BLK_INIT(BLK, COL)  { .page = (BLK) * NAND_PAGE_PER_BLK, .col = COL}

#define SNAND_ADDR_SET(addr_ptr, PAGE, COL) {(addr_ptr)->page = PAGE; (addr_ptr)->col = COL; }
#define SNAND_ADDR_BLK_SET(addr_ptr, BLK, COL) \
    {(addr_ptr)->page = (BLK) * NAND_PAGE_PER_BLK; \
     (addr_ptr)->col = COL; }

#define SNAND_VADDR_SET(addr_ptr, PAGE, COL, SIZE, MAP) \
    {(addr_ptr)->vpage = PAGE; \
    (addr_ptr)->col = COL; \
    (addr_ptr)->map_size = SIZE; \
    (addr_ptr)->vmap = MAP;}

#define NAND_VBLK_START             (DBG_SNAND_START_BLK + NAND_CTRL_INFO.vblks_start)
/// NAND flash page size in byte
#define NAND_PAGE_LEN               (NAND_CTRL_INFO.page_size)
/// NAND flash page count per block
#define NAND_PAGE_PER_BLK           (NAND_CTRL_INFO.page_per_blk)
/// Get offset of address inside page
#define NAND_PAGE_OFST(addr)        ((addr) & NAND_CTRL_INFO.page_mask)
/// Get offset of page inside block
#define NAND_BLK_OFST(page_addr)    ((page_addr) & NAND_CTRL_INFO.blk_mask)
/// Convert page to block
#define NAND_PAGE2BLK(page_addr)    ((page_addr) >> NAND_CTRL_INFO.page2blk_shift)
/// Convert address to page
#define NAND_ADDR2PAGE(addr)        ((addr) >> NAND_CTRL_INFO.addr2page_shift)
/// Convert page to address
#define NAND_PAGE2ADDR(page_addr)   ((page_addr) << NAND_CTRL_INFO.addr2page_shift)
/// Convert block to page
#define NAND_BLK2PAGE(blk_idx)      ((blk_idx) << NAND_CTRL_INFO.page2blk_shift)
/// Base address of simulated snand address
#define NAND_SIM_ADDR_BASE          SPI_FLASH_BASE
/// Convert page & col to simulated address, do not directly access the address
#define NAND_SIM_ADDR(PAGE, COL)    ((void *)(NAND_PAGE2ADDR(PAGE) + COL + NAND_SIM_ADDR_BASE))

typedef struct snand_raw_ctrl_info {
	u32 blk_cnt;
	u32 page_per_blk;
	u32 page_size;
	u32 spare_size;
	u32 rsvd1[4];
	u32 par_tbl_start;
	u32 par_tbl_dup_cnt;
	u32 vblks_start;
	u32 vblks_cnt;
	u32 rsvd2[10];
} snand_raw_ctrl_info_t;

typedef struct snand_ctrl_info {
	u32 blk_cnt;
	u32 page_per_blk;
	u32 page_size;
	u32 spare_size;
	u32 page_mask;
	u32 blk_mask;
	u8 page2blk_shift;
	u8 addr2page_shift;
	u16 cache;
	// cache line size
	u32 par_tbl_start;
	u32 par_tbl_dup_cnt;
	u32 vblks_start;
	u32 vblks_cnt;
	u32 rsvd2[18];
} snand_ctrl_info_t;

typedef struct snand_addr {
	u32 page;
	u16 col;
} snand_addr_t;

typedef struct snand_vaddr {
	u16 *vmap;          // virtual block to physical block mapping
	u32 vpage;
	u16 col;
	u8 map_size;
} snand_vaddr_t;

// Partition record in NAND flash layout
typedef struct snand_part_record {
	u32 magic_num;
	u16 type_id;
	u16 blk_cnt;
	u32 rsvd[6];
	u16 vblk[48];
} snand_part_record_t;

// partition table info
typedef struct snand_partition_entry {
	u16 type_id;
	u16 blk_cnt;
	snand_addr_t vmap_addr;
	u32 rsvd[6];
} snand_partition_entry_t;

typedef struct snand_partition_tbl {
	part_fst_info_t mfst;
	snand_partition_entry_t entrys[PARTITION_RECORD_MAX];
} snand_partition_tbl_t;

typedef struct hal_snand_boot_stubs {
	u8 *rom_snand_boot;
	snand_ctrl_info_t *ctrl_info;
	snand_partition_tbl_t *part_tbl;

	hal_status_t (*snafc_init)(hal_snafc_adaptor_t *snafc_adpt, u8 spic_bit_mode, u8 io_pin_sel);
	hal_status_t (*snafc_deinit)(hal_snafc_adaptor_t *snafc_adpt);
	hal_status_t (*snand_memcpy_update)(hal_snafc_adaptor_t *snafc_adpt, void *dest, snand_addr_t *snand_addr, u32 size);
	hal_status_t (*snand_memcpy)(hal_snafc_adaptor_t *snafc_adpt, void *dest, const snand_addr_t *snand_addr, u32 size);
	hal_status_t (*snand_offset)(snand_addr_t *snand_addr, u32 offset);
	u32 rsvd[40];
} hal_snand_boot_stubs_t;


#if defined(ROM_REGION)
#define NAND_CTRL_INFO              rom_snand_ctrl_info
extern snand_ctrl_info_t rom_snand_ctrl_info;
extern hal_crypto_adapter_t sb_rom_crypto_adtr;
#else
#define NAND_CTRL_INFO              snand_ctrl_info
extern snand_ctrl_info_t snand_ctrl_info;
#endif

extern const hal_snand_boot_stubs_t hal_snand_boot_stubs;

hal_status_t snand_memcpy(hal_snafc_adaptor_t *snafc_adpt, void *dest, const snand_addr_t *snand_addr, u32 size);
hal_status_t snand_memcpy_update(hal_snafc_adaptor_t *snafc_adpt, void *dest, snand_addr_t *snand_addr, u32 size);
u32 snand_flash_boot(PRAM_FUNCTION_START_TABLE *pram_start_func);

hal_status_t fw_snafc_init(hal_snafc_adaptor_t *snafc_adpt, u8 spic_bit_mode, u8 io_pin_sel);
hal_status_t fw_snafc_deinit(hal_snafc_adaptor_t *pSnafcAdaptor);
hal_status_t snand_memcpy_update(hal_snafc_adaptor_t *snafc_adpt, void *dest, snand_addr_t *snand_addr, u32 size);
hal_status_t snand_memcpy(hal_snafc_adaptor_t *snafc_adpt, void *dest, const snand_addr_t *snand_addr, u32 size);
hal_status_t snand_offset(snand_addr_t *snand_addr, u32 offset);

#endif  // end of "#define _FW_SNAND_BOOT_H_"
