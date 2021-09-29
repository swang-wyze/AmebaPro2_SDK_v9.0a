/**************************************************************************//**
 * @file     fw_snand_boot.c
 * @brief    Implement the booting from nand flash. The ROM code will load
 *           FW image load from flash to RAM and then jump to RAM code.
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

#include "cmsis.h"
#include "memory.h"
#include "otp_boot_cfg.h"
#include "rtl8735b_ramstart.h"
#include "fw_img_tlv.h"
#include "fw_snand_boot.h"
#include "fw_voe_rom_boot.h"

// TODO move snafc init here

// FIXME add snand api return value handling

#define FLASH_FW_LOAD_DBG           1

#define SECTION_SNAND_BOOT_STUBS     SECTION(".rom.snand_boot.stubs")

u8 rom_snand_boot = 0;

snand_ctrl_info_t rom_snand_ctrl_info __ALIGNED(32) = {
	.blk_cnt = 0x800,           // 2048 blks
	.page_per_blk = 0x40,
	.page_size = 0x800,
	.spare_size = 0x40,
	.page_mask = 0x800 - 1,
	.blk_mask = 0x40 - 1,
	.addr2page_shift = 11,
	.page2blk_shift = 6,

	.par_tbl_start = 9,
	.par_tbl_dup_cnt = 7,

	.vblks_start = 16,
	.vblks_cnt = 0x700,
};

snand_partition_tbl_t snand_part_tbl;

SECTION_SNAND_BOOT_STUBS
const hal_snand_boot_stubs_t hal_snand_boot_stubs = {
	.rom_snand_boot = &rom_snand_boot,
	.ctrl_info = &rom_snand_ctrl_info,
	.part_tbl = &snand_part_tbl,
	.snafc_init = fw_snafc_init,
	.snafc_deinit = fw_snafc_deinit,
	.snand_memcpy_update = snand_memcpy_update,
	.snand_memcpy = snand_memcpy,
	.snand_offset = snand_offset,
};

// FIXME
extern hal_snafc_adaptor_t _hal_snafc_adaptor;
extern sec_boot_info_t sb_rom_info;

// FIXME remove me
SECTION_ROM_TEMP_BSS u8 snand_r_buf[NAND_PAGE_MAX_LEN] __attribute((aligned(32)));

#define ATTR_CPY(s1, s2, attr)      {(s1)->attr = (s2)->attr;}

static void init_ctrl_info(snand_ctrl_info_t *info, snand_raw_ctrl_info_t *raw)
{
	u32 target_bit, val;
	ATTR_CPY(info, raw, blk_cnt);
	ATTR_CPY(info, raw, page_per_blk);
	ATTR_CPY(info, raw, page_size);
	ATTR_CPY(info, raw, spare_size);

	ATTR_CPY(info, raw, par_tbl_start);
	ATTR_CPY(info, raw, par_tbl_dup_cnt);
	ATTR_CPY(info, raw, vblks_start);
	ATTR_CPY(info, raw, vblks_cnt);

	// Set useful values
	info->page_mask     = raw->page_size - 1;
	info->blk_mask      = raw->page_per_blk - 1;

	target_bit = info->page_size;
	val = 1;
	for (u32 bit = 0; bit < 32; bit++, val <<= 1) {
		if (val & target_bit) {
			info->addr2page_shift = bit;
		}
	}

	target_bit = info->page_per_blk;
	for (u32 bit = 0; bit < 32; bit++, val <<= 1) {
		if (val & target_bit) {
			info->page2blk_shift = bit;
		}
	}
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
			hal_rtl_snand_pageRead(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_page += 1;
			cur_col = 0;
		} else {
			size_left = size;
			hal_rtl_snand_pageRead(snafc_adpt, cur_dst, size_left, cur_page);
			cur_dst += size_left;
			cur_col += size_left;
		}
	}

	// Reset col_addr
	snafc_adpt->col_addr = 0;
	size_left = size - size_left;

	while (size_left > NAND_PAGE_LEN) {
		hal_rtl_snand_pageRead(snafc_adpt, cur_dst, NAND_PAGE_LEN, cur_page);
		size_left -= NAND_PAGE_LEN;
		cur_dst += NAND_PAGE_LEN;
		cur_page += 1;
	}

	// Handle left size
	if (size_left > 0) {
		hal_rtl_snand_pageRead(snafc_adpt, cur_dst, size_left, cur_page);
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

hal_status_t snand_sim_memcpy(void *dst, const void *src, u32 size)
{
	u32 offset;
	snand_addr_t addr;
	hal_status_t ret;
	if ((u32)src < NAND_SIM_ADDR_BASE || (u32)src % 4) {
		dbg_printf("snand_memcpy_wrap invalid dst\r\n");
		return HAL_ERR_PARA;
	}

	if (size % 4) {
		dbg_printf("snand_memcpy_wrap size not 4 byte align\r\n");
		return HAL_ERR_PARA;
	}

	offset = (u32)src - NAND_SIM_ADDR_BASE;
	SNAND_ADDR_SET(&addr, NAND_ADDR2PAGE(offset), NAND_PAGE_OFST(offset));

	ret = snand_memcpy(&_hal_snafc_adaptor, dst, &addr, size);
	if (ret != HAL_OK) {
		return ret;
	}
	return HAL_OK;
}

BOOLEAN snand_is_bblk(hal_snafc_adaptor_t *snafc_adpt, u32 blk_id)
{
	u8 spare[16];
	snafc_adpt->col_addr = NAND_PAGE_LEN;
	hal_rtl_snand_pageRead(snafc_adpt, spare, sizeof(spare), NAND_BLK2PAGE(blk_id));
	snafc_adpt->col_addr = 0;
	if (spare[0] != 0xFF) {
		dbg_printf("blk %u is bb\r\n", blk_id);
		// FIXME
		dump_for_one_bytes(spare, sizeof(spare));
		return TRUE;
	}
	return FALSE;
}

u32 snand_load_par_tbl(void)
{
	u32 ret = SUCCESS;
	snand_addr_t par_tbl_addr;

	// Init partition table mfst
	snand_part_tbl.mfst.bl_p_idx = 0xFF;
	snand_part_tbl.mfst.bl_s_idx = 0xFF;
	snand_part_tbl.mfst.fw1_idx = 0xFF;
	snand_part_tbl.mfst.fw2_idx = 0xFF;
	snand_part_tbl.mfst.iq_idx = 0xFF;
	snand_part_tbl.mfst.nn_m_idx = 0xFF;
	snand_part_tbl.mfst.mp_idx = 0xFF;

	dbg_printf("=== Load PARTBL ===\r\n");

	u32 blk_id = rom_snand_ctrl_info.par_tbl_start;
	u32 blk_end = blk_id + rom_snand_ctrl_info.par_tbl_dup_cnt;
	for (; blk_id < blk_end; blk_id++) {
		if (snand_is_bblk(&_hal_snafc_adaptor, blk_id)) {
			continue;
		}
		dbg_printf("Load partition page from 0x%x block\r\n", blk_id);
		SNAND_ADDR_BLK_SET(&par_tbl_addr, DBG_SNAND_START_BLK + blk_id, 0);
		break;
	}

	if (blk_id == blk_end) {
		dbg_printf("No valid partition table block found\r\n");
		return FAIL;
	}


	for (u32 page_idx = 0; page_idx < NAND_PAGE_PER_BLK; page_idx += 1) {
		u32 tbl_idx;
		u32 out_of_record = 0;
		u32 record_idx = 0;
		snand_addr_t saved_tbl_addr = par_tbl_addr;
		snand_partition_entry_t *entry;
		ret = snand_memcpy_update(&_hal_snafc_adaptor, snand_r_buf, &par_tbl_addr, NAND_PAGE_LEN);
		if (ret != HAL_OK) {
			return FAIL;
		}

		snand_part_record_t *par_records = (snand_part_record_t *) snand_r_buf;
		u32 record_cnt = NAND_PAGE_LEN / sizeof(snand_part_record_t);

		for (u32 idx = 0; idx < record_cnt; idx++, record_idx++) {

			if (par_records[idx].magic_num == 0xFFFFFFFF) {
				dbg_printf("End of record\r\n");
				out_of_record = 1;
				break;
			}

			if (par_records[idx].magic_num != SNAND_PAR_RECORD_MAG_NUM) {
				// Skip invalid record
				continue;
			}

			dbg_printf("record idx: %u, size: %u\r\n", record_idx, par_records[idx].blk_cnt);

			tbl_idx = snand_part_tbl.mfst.rec_num;

			if (tbl_idx >= PARTITION_RECORD_MAX) {
				break;
			}

			entry = &snand_part_tbl.entrys[tbl_idx];
			entry->type_id = par_records[idx].type_id;
			entry->blk_cnt = par_records[idx].blk_cnt;
			entry->vmap_addr = saved_tbl_addr;
			// Calculate offset of vmap
			ret = snand_offset(&entry->vmap_addr, (u32)par_records[idx].vblk - (u32)par_records);
			if (ret != HAL_OK) {
				return FAIL;
			}
			snand_part_tbl.mfst.rec_num++;

			switch (par_records[idx].type_id) {
			case FW_PT_BL_PRI_ID:
				dbg_printf("Bootloader entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.bl_p_idx = tbl_idx;
				break;
			case FW_PT_BL_SEC_ID:
				dbg_printf("Bootloader 2nd entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.bl_s_idx = tbl_idx;
				break;
			case FW_PT_FW1_ID:
				dbg_printf("FW1 entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.fw1_idx = tbl_idx;
				break;
			case FW_PT_FW2_ID:
				dbg_printf("FW2 entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.fw2_idx = tbl_idx;
				break;
			case FW_PT_ISP_IQ_ID:
				dbg_printf("ISP entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.iq_idx = tbl_idx;
				break;
			case FW_PT_NN_MDL_ID:
				dbg_printf("NN entry idx: %u\r\n", tbl_idx);
				snand_part_tbl.mfst.nn_m_idx = tbl_idx;
				break;
			default:
				dbg_printf("Unknown partition table type id\r\n");
				break;
			}
		}
		if (out_of_record == 1) {
			break;
		}
	}

	if (snand_part_tbl.mfst.bl_p_idx == 0xFF) {
		return FAIL;
	}
	return SUCCESS;
}

// TODO
#if 0
u16 snand_get_blk_type(hal_snafc_adaptor_t *snafc_adpt, u32 blk_id)
{
	u8 spare[16];
	u16 ret;
	snafc_adpt->col_addr = NAND_PAGE_LEN;
	hal_rtl_snand_pageRead(snafc_adpt, spare, sizeof(spare), NAND_BLK2PAGE(blk_id));
	snafc_adpt->col_addr = 0;
	ret = *(u16 *) &spare[NAND_SPARE_TYPE_IDX];
	return ret;
}
#endif

static u8 count_set_bits(u32 val)
{
	u8 cnt = 0;
	while (val) {
		cnt += val & 1;
		val >>= 1;
	}
	return cnt;
}

u32 snand_load_ctrl_info(void)
{
	u32 ret = SUCCESS;
	snand_raw_ctrl_info_t raw_info;
	snand_addr_t raw_ctrl_info_addr;
	u32 blk_id = NAND_CTRL_INFO_START_BLK;
	u32 blk_end = blk_id + NAND_CTRL_INFO_DUP_CNT;
	for (; blk_id < blk_end; blk_id++) {
		if (snand_is_bblk(&_hal_snafc_adaptor, blk_id)) {
			continue;
		}
		dbg_printf("Load snand ctrl info from %u blk\r\n", blk_id);
		SNAND_ADDR_BLK_SET(&raw_ctrl_info_addr, DBG_SNAND_START_BLK + blk_id, 0);
		break;
	}
	if (blk_id == blk_end) {
		dbg_printf("No valid snand ctrl info block found\r\n");
		return FAIL;
	}

	ret = snand_memcpy(&_hal_snafc_adaptor, &raw_info, &raw_ctrl_info_addr, sizeof(snand_raw_ctrl_info_t));

	// Verify snand info
	if (count_set_bits(raw_info.page_size) != 1 || count_set_bits(raw_info.page_per_blk) != 1) {
		dbg_printf("Invalid raw ctrl info\r\n");
		return FAIL;
	}

	if (SUCCESS != ret) {
		return FAIL;
	}

	init_ctrl_info(&rom_snand_ctrl_info, &raw_info);

	// Check valid ctrl_info

#if FLASH_FW_LOAD_DBG
	dbg_printf("page_per_blk: %x\r\n", rom_snand_ctrl_info.page_per_blk);
	dbg_printf("page_size: %x\r\n", rom_snand_ctrl_info.page_size);
	dbg_printf("spare_size: %x\r\n", rom_snand_ctrl_info.spare_size);
	dbg_printf("par tbl start: %x\r\n", rom_snand_ctrl_info.par_tbl_start);
	dbg_printf("vblks start: %x\r\n", rom_snand_ctrl_info.vblks_start);
#endif
	return SUCCESS;
}

u32 snand_load_sect(snand_addr_t *cur_addr, const snand_addr_t *fw_base_addr)
{
	u32 ret = SUCCESS;
	sect_hdr_t sect_hdr;

	for (u32 sect_idx = 0; sect_idx < SECT_HDR_MAX; sect_idx++) {
		ret = snand_memcpy_update(&_hal_snafc_adaptor, &sect_hdr, cur_addr, sizeof(sect_hdr_t));
		if (ret != HAL_OK) {
			return FAIL;
		}

#if FLASH_FW_LOAD_DBG
		dbg_printf("[--- Section HDR ---]\r\n");
		dbg_printf("  #seclen : %d(0x%08x)\r\n", sect_hdr.seclen, sect_hdr.seclen);
		dbg_printf("  #nxtoffset : 0x%08x\r\n", sect_hdr.nxtoffset);
		dbg_printf("  #type_id: 0x%04x\r\n", sect_hdr.type_id);
		dbg_printf("  #sec_ctrl : 0x%08x\r\n", sect_hdr.sec_ctrl);
		dbg_printf("  #dest : 0x%08x\r\n", sect_hdr.dest);
#endif

		switch (sect_hdr.type_id) {
		case FW_SIMG_DTCM_ID:
		case FW_SIMG_ITCM_ID:
		case FW_SIMG_SRAM_ID:
			ret = snand_memcpy_update(&_hal_snafc_adaptor, (u8 *)sect_hdr.dest, cur_addr, sect_hdr.seclen);
			if (ret != HAL_OK) {
				return FAIL;
			}

#if FLASH_FW_LOAD_DBG
			{
				u32 *pTmp = (u32 *)sect_hdr.dest;
				dbg_printf("[--- Section Data(#gRamStartFun) ---]\r\n");
				dbg_printf("  #Signature addr : 0x%08x\r\n", pTmp);
				dbg_printf("  #RamStartFun addr : 0x%08x\r\n", pTmp + 1);
				dbg_printf("  #RamWakeupFun addr : 0x%08x\r\n", pTmp + 2);
				dbg_printf("  #RamPatchFun0 addr : 0x%08x\r\n", pTmp + 3);
				dbg_printf("  #RamPatchFun1 addr : 0x%08x\r\n", pTmp + 4);
				//dump_for_one_bytes(((u8 *)sect_hdr.dest), 64);
			}
#endif
			break;
		case FW_SIMG_DDR_ID:
			// Todo: no init DRAM codes in ROM
			break;
		case FW_SIMG_XIP_ID:
			break;
		}

		if (sect_hdr.nxtoffset == SECT_HDR_NXTOFFSET_NULL) {
			dbg_printf("End of section header\r\n");
			break;
		}
		*cur_addr = *fw_base_addr;
		ret = snand_offset(cur_addr, sect_hdr.nxtoffset);
		if (ret != HAL_OK) {
			return FAIL;
		}
	}

	return SUCCESS;
}

u32 snand_load_img(snand_addr_t *cur_addr, const snand_addr_t *fw_base_addr,
				   PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	u32 ret;
	fw_img_hdr_t img_hdr;

	dbg_printf("BL IMG header\r\n");
	for (u32 img_idx = 0; img_idx < IMG_HDR_MAX; img_idx++) {
		ret = snand_memcpy_update(&_hal_snafc_adaptor, &img_hdr, cur_addr, sizeof(fw_img_hdr_t));
		if (ret != HAL_OK) {
			return FAIL;
		}

		dbg_printf("FW img header \r\n");
		dbg_printf("%x:%d %d\r\n", cur_addr->page, cur_addr->col, sizeof(fw_img_hdr_t));

		if ((img_hdr.type_id == FW_IMG_BL_ID) && (img_hdr.str_tbl != 0xFFFFFFFF)) {
			*ram_start_func = (RAM_FUNCTION_START_TABLE *)img_hdr.str_tbl;
			dbg_printf("[Image Start Table @ 0x%x]\r\n", *ram_start_func);
		}

		// Load section header
		ret = snand_load_sect(cur_addr, fw_base_addr);
		if (ret != SUCCESS) {
			return FAIL;
		}

		if (img_hdr.nxtoffset == FW_HDR_NXTOFFSET_NULL) {
			dbg_printf("End of img header\r\n");
			break;
		}
		*cur_addr = *fw_base_addr;
		ret = snand_offset(cur_addr, img_hdr.nxtoffset);
		if (ret != HAL_OK) {
			return FAIL;
		}
	}

	return SUCCESS;
}

u32 snand_load_bl(PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	u32 ret = SUCCESS;
	u8 vmap[SNAND_VMAP_MAX];
	img_manifest_t mfst __ALIGNED(32);
	snand_addr_t bl_addr;
	snand_addr_t *vmap_addr;
	u8 bl_idx;

	dbg_printf("=== Load BL ===\r\n");

	// load vmap
	bl_idx = snand_part_tbl.mfst.bl_p_idx;
	vmap_addr = &snand_part_tbl.entrys[bl_idx].vmap_addr;
	ret = snand_memcpy(&_hal_snafc_adaptor, &vmap, vmap_addr, sizeof(vmap));
	if (ret != HAL_OK) {
		return FAIL;
	}

	SNAND_ADDR_BLK_SET(&bl_addr, NAND_VBLK_START + vmap[0], 0);
	dbg_printf("boot loader page addr: 0x%x\r\n", bl_addr.page);

	// Get manifest
	snand_addr_t cur_addr = bl_addr;
	ret = snand_memcpy_update(&_hal_snafc_adaptor, (void *) &mfst, &cur_addr, sizeof(img_manifest_t));
	if (ret != HAL_OK) {
		return FAIL;
	}

	ret = verify_manif_f((void *) &mfst, BL_INFO, &sb_rom_info);
	if (ret != SUCCESS) {
		dbg_printf("verify mani failed!!!\r\n");
		return ret;
	}
	dbg_printf("verify mani pass\r\n");

	// Load img header
	u32 cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	cur_addr = bl_addr;
	ret = snand_offset(&cur_addr, cur_offset);
	if (ret != HAL_OK) {
		return FAIL;
	}

	ret = snand_load_img(&cur_addr, &bl_addr, ram_start_func);
	if (ret != SUCCESS) {
		return FAIL;
	}

	return SUCCESS;
}

u32 snand_load_iq_raw(snand_addr_t *cur_addr, const snand_addr_t *iq_addr, u8 fcs_id)
{
	u32 ret;
	void *sim_addr;
	fw_img_hdr_t img_hdr;

	for (u32 img_idx = 0; img_idx < IMG_HDR_MAX; img_idx++) {
		ret = snand_memcpy_update(&_hal_snafc_adaptor, &img_hdr, cur_addr, sizeof(fw_img_hdr_t));
		if (ret != HAL_OK) {
			return FAIL;
		}

		dbg_printf("[--- IMG HDR ---]\r\n");
		dbg_printf("%x:%d %d\r\n", cur_addr->page, cur_addr->col, sizeof(fw_img_hdr_t));

		if (img_hdr.type_id != FW_IMG_ISP_ID) {
			continue;
		}

		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 8);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 8);
		sim_addr = NAND_SIM_ADDR(cur_addr->page, cur_addr->col);
		dbg_printf("ISP_IQ @ (snand) 0x%x, size: 0x%x, id: 0x%x\r\n", sim_addr, (img_hdr.imglen), fcs_id);
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 9);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 9);
		hal_rtl_voe_fcs_process((voe_cpy_t)snand_sim_memcpy, (u32)sim_addr, fcs_id, img_hdr.imglen);
		ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 10);
		ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 10);
		dbg_printf("=== Process ISP_IQ ===\r\n");

		if (img_hdr.nxtoffset == FW_HDR_NXTOFFSET_NULL) {
			dbg_printf("End of img header\r\n");
			break;
		}

		*cur_addr = *iq_addr;
		ret = snand_offset(cur_addr, img_hdr.nxtoffset);
		if (ret != HAL_OK) {
			return FAIL;
		}
	}

	return SUCCESS;
}

u32 snand_load_isp_iq(void)
{
	u8 fcs_rom_flow_dis, fcs_id = 0;
	u32 ret;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	u8 vmap[SNAND_VMAP_MAX];
	img_manifest_t mfst __ALIGNED(32);
	snand_addr_t iq_addr;
	snand_addr_t *vmap_addr;

	fcs_rom_flow_dis = potp_boot_cfg7->bit.fcs_rom_flow_dis;
	if (DISABLE == fcs_rom_flow_dis) {
		return SUCCESS;
	}

	dbg_printf("=== Load ISP_IQ ===\r\n");
	u32 isp_iq_idx = snand_part_tbl.mfst.iq_idx;
	if (isp_iq_idx == 0xFF) {
		return FAIL;
	}

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 11);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 11);

	// load vmap
	vmap_addr = &snand_part_tbl.entrys[isp_iq_idx].vmap_addr;
	ret = snand_memcpy(&_hal_snafc_adaptor, &vmap, vmap_addr, sizeof(vmap));
	if (ret != HAL_OK) {
		return FAIL;
	}

	SNAND_ADDR_BLK_SET(&iq_addr, NAND_VBLK_START + vmap[0], 0);
	dbg_printf("isp iq page addr: 0x%x\r\n", iq_addr.page);

	// Get manifest
	snand_addr_t cur_addr = iq_addr;
	ret = snand_memcpy_update(&_hal_snafc_adaptor, (void *) &mfst, &cur_addr, sizeof(img_manifest_t));
	if (ret != HAL_OK) {
		return FAIL;
	}

	// FIXME init crypto before verify
	ret = verify_fcs_isp_iq_manif_f((void *) &mfst, FW_ISP_INFO, &fcs_id);
	if (ret != SUCCESS) {
		dbg_printf("verify mani failed!!!\r\n");
		return ret;
	}
	dbg_printf("verify mani pass\r\n");

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 12);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 12);

	// Get raw data
	u32 cur_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
	cur_addr = iq_addr;
	ret = snand_offset(&cur_addr, cur_offset);
	if (ret != HAL_OK) {
		return FAIL;
	}

	ret = snand_load_iq_raw(&cur_addr, &iq_addr, fcs_id);
	if (ret != SUCCESS) {
		return FAIL;
	}

	ROM_FOOTPH_CLR(BOOT_INFO_IDX1, 13);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX1, 13);
	dbg_printf("=== Load Done ===\r\n");
	return SUCCESS;
}

u32 snand_flash_boot(PRAM_FUNCTION_START_TABLE *ram_start_func)
{
	// TLV format only
	u32 ret;

	dbg_printf("NAND Flash Boot\r\n");

	// Init
	hal_rtl_otp_init();
	ret = hal_rtl_crypto_engine_init(&sb_rom_crypto_adtr);

	// Get flash ID
	u32 flash_id = hal_rtl_snand_issueReadIdOpCmd(&_hal_snafc_adaptor);
	dbg_printf("Flash ID: 0x%08x\r\n", flash_id);

	// TODO Search ID in supported flash table to load ctrl info

	// Load ctrl info
	ret = snand_load_ctrl_info();
	if (SUCCESS != ret) {
		goto end;
	}

	// Load partition table
	ret = snand_load_par_tbl();
	if (SUCCESS != ret) {
		goto end;
	}

	// TODO Load ISP IQ
	ret = snand_load_isp_iq();
#if 0 // Ignore result
	if (SUCCESS != ret) {
		goto end;
	}
#endif

	// Load boot loader
	ret = snand_load_bl(ram_start_func);
	if (SUCCESS != ret) {
		goto end;
	}

end:
	ret = hal_rtl_crypto_engine_deinit(&sb_rom_crypto_adtr);
	hal_rtl_otp_deinit();

	return ret;
}

