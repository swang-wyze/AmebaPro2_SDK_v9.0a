/**************************************************************************//**
 * @file     snand_boot_loader.c
 * @brief    The boot loader for nand flash boot.
 * @version  V1.00
 * @date     2021-07-27
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
#include "hal_sys_ctrl.h"
#include "fw_img_tlv.h"
#include "fw_snand_boot.h"
#include "hal_flash_boot.h"
#include "hal_snand.h"

// TODO Add FOOTPH

/* if enable Flash image load debug message */
#define FLASH_FW_BOOTLOAD_DBG                 (1)

// FIXME fixed value
#define RAM_FUN_TABLE_VALID_START_ADDR         (0x20100000)
#define RAM_FUN_TABLE_VALID_END_ADDR           (0x20180000)

extern boot_init_flags_t boot_init_flags;

hal_snafc_adaptor_t snafc_adpt;
snand_ctrl_info_t snand_ctrl_info __ALIGNED(32);

// FIXME duplicated
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

// FIXME hal_snand_pio_read replace
// TODO bad block check?
hal_status_t snand_vmemcpy_update(hal_snafc_adaptor_t *snafc_adpt, void *dest, snand_vaddr_t *snand_vaddr, u32 size)
{
	dbg_printf("snand_vmemcpy dst: %x src_vpage: %x col: %x size: %x\r\n", dest, snand_vaddr->vpage, snand_vaddr->col, size);

	u32 size_left = 0;
	u32 cur_col = snand_vaddr->col;
	u32 cur_vpage = snand_vaddr->vpage;
	u32 cur_page, cur_vblk, cur_blk;
	u8 *cur_dst = (u8 *)dest;

	if (size % 4) {
		dbg_printf("snand_memcpy size not 4 byte align\r\n");
		return HAL_ERR_PARA;
	}

	// Prevent over-map copy
	if ((NAND_PAGE2ADDR(cur_vpage) + cur_col + size) > NAND_PAGE2ADDR(NAND_BLK2PAGE(snand_vaddr->map_size))) {
		dbg_printf("snand_memcpy size will cause vmap overflow\r\n");
		return HAL_ERR_PARA;
	}

	// Get vblock
	cur_vblk = NAND_PAGE2BLK(cur_vpage);

	cur_blk = snand_vaddr->vmap[cur_vblk];
	dbg_printf("vblk 0x%x -> pblk 0x%x\r\n", cur_vblk, cur_blk);
	cur_page = NAND_BLK2PAGE(cur_blk) + NAND_BLK_OFST(cur_vpage);

	if (cur_col != 0) {
		if (cur_col > NAND_PAGE_LEN) {
			dbg_printf("snand_memcpy col > page size\r\n");
			return HAL_ERR_PARA;
		}
		if (cur_col % 4) {
			dbg_printf("snand_memcpy col address not 4 byte align\r\n");
			return HAL_ERR_PARA;
		}
		snafc_adpt->col_addr = cur_col;
		size_left = NAND_PAGE_LEN - cur_col;
		if (size_left < size) { // FIXME
			hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_page += 1;
			cur_col = 0;

			// check cur_page
			if (NAND_BLK_OFST(cur_page) == 0) {
				cur_vblk++;
				cur_page = NAND_BLK2PAGE(snand_vaddr->vmap[cur_vblk]);
#if FLASH_FW_BOOTLOAD_DBG
				dbg_printf("Start reading vblk: 0x%x, -> pblk: 0x%x\r\n", cur_vblk, snand_vaddr->vmap[cur_vblk]);
#endif
			}
		} else {
			size_left = size;
			hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_col += size_left;
			// end of memcpy
		}
	}

	// Reset col_addr
	snafc_adpt->col_addr = 0;
	size_left = size - size_left;

	while (size_left > NAND_PAGE_LEN) {
		hal_snand_pio_read(snafc_adpt, cur_dst, NAND_PAGE_LEN, cur_page);
		size_left -= NAND_PAGE_LEN;
		cur_dst += NAND_PAGE_LEN;
		cur_page += 1;

		// check cur_page
		if (NAND_BLK_OFST(cur_page) == 0) {
			cur_vblk++;
			cur_page = NAND_BLK2PAGE(snand_vaddr->vmap[cur_vblk]);
#if FLASH_FW_BOOTLOAD_DBG
			dbg_printf("Start reading vblk: 0x%x, -> pblk: 0x%x\r\n", cur_vblk, snand_vaddr->vmap[cur_vblk]);
#endif
		}
	}

	// handle left size
	if (size_left > 0) {
		hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
		cur_col += size_left;
	}

	snand_vaddr->vpage = NAND_BLK2PAGE(cur_vblk) + NAND_BLK_OFST(cur_page);
	snand_vaddr->col = cur_col;

	return HAL_OK;
}

hal_status_t snand_vmemcpy(hal_snafc_adaptor_t *snafc_adpt, void *dest, const snand_vaddr_t *snand_vaddr, u32 size)
{
	snand_vaddr_t tmp_vaddr = *snand_vaddr;
	return snand_vmemcpy_update(snafc_adpt, dest, &tmp_vaddr, size);
}

hal_status_t snand_memcpy_update(hal_snafc_adaptor_t *snafc_adpt, void *dest, snand_addr_t *snand_addr, u32 size)
{
	dbg_printf("snand_memcpy dst: %x src_page: %x col: %x size: %x\r\n", dest, snand_addr->page, snand_addr->col, size);

	u32 size_left = 0;
	u32 cur_col = snand_addr->col;
	u32 cur_page = snand_addr->page;
	u8 *cur_dst = (u8 *)dest;

	if (size % 4) {
		dbg_printf("snand_memcpy size not 4 byte align\r\n");
		return HAL_ERR_PARA;
	}

	// Prevent cross-block copy
	if ((NAND_PAGE2ADDR(NAND_BLK_OFST(cur_page)) + cur_col + size) > NAND_PAGE2ADDR(NAND_PAGE_PER_BLK)) {
		dbg_printf("snand_memcpy size too big, cross block boundary\r\n");
		return HAL_ERR_PARA;
	}

	if (cur_col != 0) {
		if (cur_col > NAND_PAGE_LEN) {
			dbg_printf("snand_memcpy col > page size\r\n");
			return HAL_ERR_PARA;
		}
		if (cur_col % 4) {
			dbg_printf("snand_memcpy col address not 4 byte align\r\n");
			return HAL_ERR_PARA;
		}
		snafc_adpt->col_addr = cur_col;
		size_left = NAND_PAGE_LEN - cur_col;
		if (size_left <= size) {
			hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_page += 1;
			cur_col = 0;
		} else {
			size_left = size;
			hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_col += size_left;
		}
	}

	// Reset col_addr
	snafc_adpt->col_addr = 0;
	size_left = size - size_left;

	while (size_left > NAND_PAGE_LEN) {
		hal_snand_pio_read(snafc_adpt, cur_dst, NAND_PAGE_LEN, cur_page);
		size_left -= NAND_PAGE_LEN;
		cur_dst += NAND_PAGE_LEN;
		cur_page += 1;
	}

	// Handle left size
	if (size_left > 0) {
		hal_snand_pio_read(snafc_adpt, cur_dst, size_left, cur_page);
		cur_col += size_left;
	}

	snand_addr->page = cur_page;
	snand_addr->col = cur_col;

	return HAL_OK;
}

hal_status_t snand_memcpy(hal_snafc_adaptor_t *snafc_adpt, void *dest, const snand_addr_t *snand_addr, u32 size)
{
	snand_addr_t tmp_addr = *snand_addr;
	return snand_memcpy_update(snafc_adpt, dest, &tmp_addr, size);
}

hal_status_t snand_voffset(snand_vaddr_t *snand_vaddr, u32 offset)
{
	hal_status_t ret;
	snand_addr_t addr = SNAND_ADDR_INIT(snand_vaddr->vpage, snand_vaddr->col);
	ret = snand_offset(&addr, offset);
	snand_vaddr->vpage = addr.page;
	snand_vaddr->col = addr.col;
	return ret;
}

hal_status_t snand_offset(snand_addr_t *snand_addr, u32 offset)
{
	u32 page, col;
	page = snand_addr->page;
	col = snand_addr->col;
	u32 cur_page = snand_addr->page;
	u32 cur_col = snand_addr->col;
	if (offset % 4) {
		dbg_printf("snand_offset size not 4 byte align\r\n");
		return HAL_ERR_PARA;
	}

	if (cur_col != 0) {
		if (cur_col > NAND_PAGE_LEN) {
			dbg_printf("snand_offset col > page size\r\n");
			return HAL_ERR_PARA;
		}
		if (cur_col % 4) {
			dbg_printf("snand_offset col address not 4 byte align\r\n");
			return HAL_ERR_PARA;
		}
	}
	cur_page += NAND_ADDR2PAGE(offset);
	cur_col += NAND_PAGE_OFST(offset);
	if (cur_col > NAND_PAGE_LEN) {
		cur_col -= NAND_PAGE_LEN;
		cur_page += 1;
	}

	snand_addr->page = cur_page;
	snand_addr->col = cur_col;
	dbg_printf("snand_offset 0x%x:0x%x > offset: 0x%x > 0x%x:0x%x\r\n", page, col, offset, snand_addr->page, snand_addr->col);
	return HAL_OK;
}

u32 snand_load_sect(snand_vaddr_t *cur_addr, const snand_vaddr_t *fw_base_addr)
{
	u32 ret = SUCCESS;
	sect_hdr_t sect_hdr;
	ret = snand_vmemcpy_update(&snafc_adpt, &sect_hdr, cur_addr, sizeof(sect_hdr_t));
	if (ret != HAL_OK) {
		return FAIL;
	}

	for (u32 sect_idx = 0; sect_idx < SECT_HDR_MAX; sect_idx++) {
#if FLASH_FW_BOOTLOAD_DBG
		dbg_printf("[--- Section HDR ---]\r\n");
		dbg_printf("  #seclen : %d(0x%08x)\r\n", sect_hdr.seclen, sect_hdr.seclen);
		dbg_printf("  #nxtoffset : 0x%08x\r\n", sect_hdr.nxtoffset);
		dbg_printf("  #type_id: 0x%04x\r\n", sect_hdr.type_id);
		dbg_printf("  #sec_ctrl : 0x%08x\r\n", sect_hdr.sec_ctrl);
		dbg_printf("  #dest : 0x%08x\r\n", sect_hdr.dest);

		dbg_printf("Load from snand:0x%x:0x%x\r\n", cur_addr->vpage, cur_addr->col);
#endif

		switch (sect_hdr.type_id) {
		case FW_SIMG_DTCM_ID:
		case FW_SIMG_ITCM_ID:
		case FW_SIMG_SRAM_ID:
			ret = snand_vmemcpy_update(&snafc_adpt, (u8 *)sect_hdr.dest, cur_addr, sect_hdr.seclen);
			if (ret != HAL_OK) {
				return FAIL;
			}
#if FLASH_FW_BOOTLOAD_DBG

			if (((u32)((u32 *)sect_hdr.dest + 1) >= RAM_FUN_TABLE_VALID_START_ADDR) &&
				((u32)((u32 *)sect_hdr.dest + 1) < RAM_FUN_TABLE_VALID_END_ADDR)) {
				u32 *pTmp = (u32 *)sect_hdr.dest;
				dbg_printf("[--- Section Data(#gRamStartFun) ---]\r\n");
				dbg_printf("  #Signature addr : 0x%08x\r\n", pTmp);
				dbg_printf("  #RamStartFun addr : 0x%08x\r\n", pTmp + 1);
				dbg_printf("  #RamWakeupFun addr : 0x%08x\r\n", pTmp + 2);
				dbg_printf("  #RamPatchFun0 addr : 0x%08x\r\n", pTmp + 3);
				dbg_printf("  #RamPatchFun1 addr : 0x%08x\r\n", pTmp + 4);
				dump_bytes(((u8 *)sect_hdr.dest), 64);
			}
#endif
			break;
		case FW_SIMG_DDR_ID:
			if (boot_init_flags.b.psram_inited == 0) {
				bootload_dram_init();
				boot_init_flags.b.psram_inited = 1;
			}
			ret = snand_vmemcpy_update(&snafc_adpt, (u8 *)sect_hdr.dest, cur_addr, sect_hdr.seclen);
			if (ret != HAL_OK) {
				return FAIL;
			}
			break;
		case FW_SIMG_XIP_ID:
			dbg_printf("Warning: XIP IMG does not work for NAND flash\r\n");
			break;
		}

		if (sect_hdr.nxtoffset == SECT_HDR_NXTOFFSET_NULL) {
			dbg_printf("out of section header\r\n");
			break;
		}
		*cur_addr = *fw_base_addr;
		ret = snand_voffset(cur_addr, sect_hdr.nxtoffset);
		if (ret != HAL_OK) {
			return FAIL;
		}

		ret = snand_vmemcpy_update(&snafc_adpt, &sect_hdr, cur_addr, sizeof(sect_hdr_t));
		if (ret != HAL_OK) {
			return FAIL;
		}
	}
	return ret;
}

u32 snand_load_img(snand_vaddr_t *cur_addr, const snand_vaddr_t *fw_base_addr, PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	u32 ret;
	fw_img_hdr_t img_hdr;

	ret = snand_vmemcpy_update(&snafc_adpt, &img_hdr, cur_addr, sizeof(fw_img_hdr_t));
	if (ret != HAL_OK) {
		return FAIL;
	}

	for (u32 img_idx = 0; img_idx < IMG_HDR_MAX; img_idx++) {
		dbg_printf("FW img header \r\n");

		switch (img_hdr.type_id) {
		case FW_IMG_XIP_ID:
			dbg_printf("=== Handle FW_IMG_XIP ===\r\n");
			break;
		case FW_IMG_ISP_ID:
			// TODO
			dbg_printf("=== Handle ISP IMG ===\r\n");
			break;
		case FW_IMG_VOE_ID:
			dbg_printf("=== Handle VOE IMG ===\r\n");
			// TODO
			break;
		default:
			dbg_printf("=== Handle Other IMG ===\r\n");
			if (img_hdr.str_tbl != 0xFFFFFFFF && img_hdr.type_id == FW_IMG_FWHS_S_ID) {
				*ram_start_func = (RAM_FUNCTION_START_TABLE *)img_hdr.str_tbl;
				dbg_printf("[Image Start Table @ 0x%x]\r\n", *ram_start_func);
			}
			// Load section header
			ret = snand_load_sect(cur_addr, fw_base_addr);
			if (ret != SUCCESS) {
				return FAIL;
			}
			break;
		}

		if (img_hdr.nxtoffset == FW_HDR_NXTOFFSET_NULL) {
			dbg_printf("out of img header\r\n");
			break;
		}

		*cur_addr = *fw_base_addr;
		ret = snand_voffset(cur_addr, img_hdr.nxtoffset);
		if (ret != HAL_OK) {
			return FAIL;
		}

		ret = snand_vmemcpy_update(&snafc_adpt, &img_hdr, cur_addr, sizeof(fw_img_hdr_t));
		if (ret != HAL_OK) {
			return FAIL;
		}
	}

	return SUCCESS;
}

u32 snand_load_fw(snand_partition_entry_t *entry, PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	img_manifest_t mfst;
	u32 ret, mfst_offset;
	u16 vmap[SNAND_VMAP_MAX];
	snand_vaddr_t fw_addr, cur_addr;
	sec_boot_info_t dummy_sb_info;

	memset(&dummy_sb_info, 0, sizeof(sec_boot_info_t));

	if (entry->type_id != FW_PT_FW1_ID && entry->type_id != FW_PT_FW2_ID) {
		dbg_printf("Partition entry: type id invalid\r\n");
		return FAIL;
	}

	if (entry->blk_cnt == 0 || entry->blk_cnt == 0xFF) {
		dbg_printf("Partition entry: count invalid\r\n");
		return FAIL;
	}

	// get vmap
	ret = snand_memcpy(&snafc_adpt, (void *) vmap, &entry->vmap_addr, sizeof(vmap));
	if (ret != SUCCESS) {
		return FAIL;
	}

	// Init vmap
	for (u32 i = 0; i < entry->blk_cnt; i++) {
		vmap[i] += NAND_VBLK_START;
	}

	dump_bytes((u8 *)vmap, sizeof(vmap));

	// Init vaddr
	SNAND_VADDR_SET(&fw_addr, 0, 0, entry->blk_cnt, vmap);

	// Load manifest
	ret = snand_vmemcpy(&snafc_adpt, (void *) &mfst, &fw_addr, sizeof(img_manifest_t));
	if (ret != SUCCESS) {
		return FAIL;
	}

	dbg_printf("manifest: \r\n");
	dump_bytes((u8 *)&mfst, 64);

	ret = boot_verify_manif_f((u8 *)&mfst, FW_IMG_INFO, &dummy_sb_info);
	if (ret != SUCCESS) {
		dbg_printf("Verify manif failed\r\n");
		return FAIL;
	}
	dbg_printf("Verify manif pass\r\n");

	// Load img header
	dbg_printf("[--- FW IMG ---]\r\n");
	mfst_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	cur_addr = fw_addr;
	ret = snand_voffset(&cur_addr, mfst_offset);
	if (ret != HAL_OK) {
		return FAIL;
	}

	ret = snand_load_img(&cur_addr, &fw_addr, ram_start_func);
	if (ret != HAL_OK) {
		return FAIL;
	}

	return SUCCESS;
}

static u8 count_set_bits(u32 val)
{
	u8 cnt = 0;
	while (val) {
		cnt += val & 1;
		val >>= 1;
	}
	return cnt;
}

u32 snand_boot_loader(PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	u32 ret;
	snand_partition_tbl_t *part_tbl = hal_snand_boot_stubs.part_tbl;
	u8 fw1_entry_idx;

	// Init snand_ctrl_info
	memcpy(&snand_ctrl_info, hal_snand_boot_stubs.ctrl_info, sizeof(snand_ctrl_info));

	// Verify snand info
	if (count_set_bits(NAND_PAGE_LEN) != 1 || count_set_bits(NAND_PAGE_PER_BLK) != 1) {
		dbg_printf("Invalid ctrl info\r\n");
		return FAIL;
	}

#ifdef FLASH_FW_BOOTLOAD_DBG
	dbg_printf("page size: 0x%x\r\n", NAND_PAGE_LEN);
	dbg_printf("page per blk: 0x%x\r\n", NAND_PAGE_PER_BLK);

	// dump partition table
	dbg_printf("part table size: %d\r\n", part_tbl->mfst.rec_num);
	dump_bytes((u8 *)&part_tbl->mfst, sizeof(part_fst_info_t));

	for (u32 i = 0; i < part_tbl->mfst.rec_num; i++) {
		snand_partition_entry_t *entry = &part_tbl->entrys[i];
		dbg_printf("idx: %d, type: 0x%x, cnt: %u, addr: 0x%x, 0x%x\r\n",
				   i, entry->type_id, entry->blk_cnt, entry->vmap_addr.page,
				   entry->vmap_addr.col);
	}
#endif

	ret = hal_snand_boot_stubs.snafc_init(&snafc_adpt, SpicOneIOMode, SpicDualIOPin);
	if (ret != HAL_OK) {
		dbg_printf("snafc init failed\r\n");
		ret = FAIL;
		goto snand_bl_end;
	}
	dbg_printf("snafc init pass\r\n");

	// Load ISP_IQ
	dbg_printf("=== Load ISP_IQ ===\r\n");
	dbg_printf("=== Load Done ===\r\n");

	// Load FW1
	dbg_printf("=== Load FW1 ===\r\n");
	fw1_entry_idx = part_tbl->mfst.fw1_idx;

	if (fw1_entry_idx != 0xFF) {
		ret = snand_load_fw(&part_tbl->entrys[fw1_entry_idx], ram_start_func);
		if (ret != SUCCESS) {
			goto snand_bl_end;
		}
	}

	dbg_printf("=== FW Load Done ===\r\n");

	ret = hal_snand_boot_stubs.snafc_deinit(&snafc_adpt);
	if (ret != HAL_OK) {
		dbg_printf("snafc deinit failed\r\n");
		ret = FAIL;
		goto snand_bl_end;
	}
	dbg_printf("snafc deinit pass\r\n");

snand_bl_end:
#if 0 // FIXME
	dbg_printf("fw_snafc_init freeze at %s:%d\r\n", __FILE__, __LINE__);
	while (1);
#endif

	return ret;
}

