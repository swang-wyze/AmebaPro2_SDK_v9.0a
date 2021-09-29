/**************************************************************************//**
 * @file     fw_uart_boot.c
 * @brief    Implement the FW image load over UART X-Modem.
 *
 * @version  V1.00
 * @date     2021-07-29
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
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
#include "memory.h"
#include "otp_boot_cfg.h"
#include "xmport_uart.h"
#include "xmodem.h"
#include "strproc.h"
#include "fw_uart_boot.h"

/* if enable Flash image load debug message */
#define UART_BOOT_DBG               0
#define UART_BOOT_LS_IMG_DMA        0

#define UART_BOOT_TX_PKT_SIZE       (2048)
#define TEMP_DECRYPT_BUF_SIZE       (384)


#if UART_BOOT_DBG
#define UART_BOOT_DBG_PRINTF(...)     do {\
     dbg_printf(__VA_ARGS__);\
 }while(0)
#else
#define UART_BOOT_DBG_PRINTF(...)
#endif


extern hal_status_t otu_uart_port_open(uint32_t tx_pin, uint32_t rx_pin,
									   uint32_t baud_rate, uint32_t parity, uint32_t flow_ctrl,
									   stdio_port_backup_t *stdio_uart_bkup);
extern hal_status_t otu_uart_port_close(stdio_port_backup_t *stdio_uart_bkup);
extern void img_load_log_add(uint32_t img_sz, uint32_t img_start);
extern int32_t chk_manifest_lbl(char *lbl_name);

extern xmodem_uart_port_handler_t xm_uart_port;
extern XMODEM_CTRL xmodem_ctrl;
extern uint8_t xm_rx_buf[];    // the buffer for UART RX FIFO

extern uint8_t xmodem_frame_buf[];

extern hal_uart_adapter_t otu_uart;
extern uint32_t boot_uart_baud_sel_map[];
extern const uint32_t rom_log_uart_rx_pins[2][4];
extern const uint32_t rom_log_uart_tx_pins[2][4];
extern const char otu_ok[];
extern const char otu_err[];

//SECTION_ROM_TEMP_BSS uint8_t uart_boot_temp_buf[XM_BUFFER_SIZE*2] __ALIGNED(32);     // the temp buffer for packet decryption
// SDIO boot also use UART boot packet parser, so the packet buffer must be able to store 2 SDIO packets
SECTION_ROM_TEMP_BSS uint8_t uart_boot_temp_buf[UART_BOOT_TX_PKT_SIZE * 2] __ALIGNED(32);   // the temp buffer for packet decryption
SECTION_ROM_TEMP_BSS uboot_xm_frame_handler_t uboot_frame_handle;
extern uint8_t boot_imgft;
extern uint8_t tmp_img_hdr[];
extern uint8_t tmp_sect_hdr[];

// temp_dec_buf can externed from flash boot SECTION_SBOOT_BSS
SECTION_ROM_TEMP_BSS uint8_t temp_dec_buf[TEMP_DECRYPT_BUF_SIZE] __ALIGNED(32);   // decrypted temp buffer

void uart_boot_store_frame(uint8_t *pframe_data, uint32_t remain_len)
{
	_memcpy((void *)uart_boot_temp_buf, pframe_data, remain_len);
	uboot_frame_handle.tmp_buf_offset = remain_len;
}

void uart_boot_store_frame_append(uint8_t *pframe_data, uint32_t remain_len)
{
	_memcpy((void *)((uint32_t)uart_boot_temp_buf + uboot_frame_handle.tmp_buf_offset), pframe_data, remain_len);
	uboot_frame_handle.tmp_buf_offset += remain_len;
}

int32_t uart_boot_process_image_header(uint32_t *pimg_hdr)
{
	fw_sub_image_type_t *penced_sub_image_header;   // encrypted sub-image header start address
	fw_sub_image_type_t *psub_image_header;
	fm_image_header_t *pheader;
	//fw_fst_t *pfst;
	int ret = SUCCESS;

	penced_sub_image_header = (fw_sub_image_type_t *)pimg_hdr;

	psub_image_header = (fw_sub_image_type_t *)temp_dec_buf;
	pheader = &psub_image_header->img_header;
	//pfst = &psub_image_header->fst;
	// image header is plain text
	_memcpy(pheader, &(penced_sub_image_header->img_header), sizeof(fm_image_header_t));

	uboot_frame_handle.next_img_offset = pheader->nxt_img_offset;
	uboot_frame_handle.offset_2_next_img = pheader->nxt_img_offset;

	if (pheader->img_type == FW_IMG_XIP) {
		// Always skip XIP FW image loading
		uboot_frame_handle.skip_sub_img_load = 1;
		return ret;
	}
	uboot_frame_handle.skip_sub_img_load = 0;

	// FST is plain text
	//UART_BOOT_DBG_PRINTF("Read Plain FST from 0x%x\r\n", ((uint32_t)&(penced_sub_image_header->fst) - SPI_FLASH_BASE));
	//_memcpy ((void *)pfst, (void *)&(penced_sub_image_header->fst), sizeof(fw_fst_t));

#if UART_BOOT_DBG
	dbg_printf("ImgHde:");
	dump_for_one_bytes((u8 *)pheader, sizeof(fm_image_header_t));

	//dbg_printf("FST:");
	//dump_for_one_bytes ((u8 *)pfst, sizeof(fw_fst_t));
#endif

	return ret;
}

int32_t uart_boot_process_section_header(uint32_t *pimg_hdr)
{
	fw_sub_image_type_t *psub_image_header;
	fm_image_header_t *psection_header;
	raw_image_hdr_t *psection_img_hdr;
	fm_image_header_t *pheader;
	//fw_fst_t *pfst;

	psub_image_header = (fw_sub_image_type_t *)temp_dec_buf;
	psection_header = (fm_image_header_t *)((uint8_t *)temp_dec_buf + sizeof(fw_sub_image_type_t));
	psection_img_hdr = (raw_image_hdr_t *)((uint8_t *)psection_header + sizeof(fm_image_header_t));
	pheader = &psub_image_header->img_header;
	//pfst = &psub_image_header->fst;

	_memcpy((void *)psection_header, (void *)pimg_hdr, sizeof(fm_image_header_t) + sizeof(raw_image_hdr_t));

	uboot_frame_handle.next_section_offset = psection_header->nxt_img_offset;
	uboot_frame_handle.offset_2_next_section = psection_header->nxt_img_offset;
	uboot_frame_handle.ram_img_offset = 0;
	uboot_frame_handle.ram_img_size = psection_img_hdr->img_sz;

	// add image loading log, so we can erase them when load failed

	uboot_frame_handle.ram_img_start_addr = psection_img_hdr->start_addr;
	if (psection_img_hdr->ram_start_tbl != 0xFFFFFFFF) {
		if (pheader->img_type == FW_IMG_FWHS_S) {
			uboot_frame_handle.ram_start_tbl = psection_img_hdr->ram_start_tbl;
			//DBG_MISC_INFO ("HS Image Start Table @ 0x%x\r\n", uboot_frame_handle.ram_start_tbl);
		}
	}

	return SUCCESS;
}

int32_t uart_boot_process_img_payload(uint32_t *pimg_payload, uint32_t remain_len, uint32_t *pconsume_len, uint8_t *load_done)
{
	//fw_sub_image_type_t *psub_image_header;
	//fw_fst_t *pfst;
	uint32_t tmp_size;
	int ret = SUCCESS;

	if (uboot_frame_handle.ram_img_size == 0) {
		*pconsume_len = 0;
		*load_done = 1;
		return ret;
	}

	//psub_image_header = (fw_sub_image_type_t *)temp_dec_buf;
	//pfst = &psub_image_header->fst;

	// Image is plain text, just copy image data from temp memory buffer to assigned RAM
	if (remain_len <= uboot_frame_handle.ram_img_size) {
		tmp_size = remain_len;
	} else {
		tmp_size = uboot_frame_handle.ram_img_size;
	}
	*pconsume_len = tmp_size;
	_memcpy((void *)(uboot_frame_handle.ram_img_start_addr), (uint8_t *)(pimg_payload), tmp_size);
	//dcache_clean_by_addr_rtl8710c ((uint32_t *)(uboot_frame_handle.ram_img_start_addr), tmp_size);
	uboot_frame_handle.ram_img_start_addr += tmp_size;
	uboot_frame_handle.ram_img_size -= tmp_size;

	if (uboot_frame_handle.ram_img_size == 0) {
		*load_done = 1;
	} else {
		*load_done = 0;
	}

	return ret;
}

int32_t uart_boot_tlv_process_image_header(uint8_t *pimg_hdr)
{
	uint8_t *ptr = (uint8_t *)pimg_hdr;
	fw_img_hdr_t *pfw_hdr = NULL;
	int ret = SUCCESS;

	if (!ptr) {
		ret = FAIL;
		return ret;
	}

	pfw_hdr = (fw_img_hdr_t *)&tmp_img_hdr[0];
	_memcpy((void *)pfw_hdr, (void *)pimg_hdr, sizeof(fw_img_hdr_t));

	uboot_frame_handle.next_img_offset = pfw_hdr->nxtoffset;
	uboot_frame_handle.offset_2_next_img = pfw_hdr->nxtoffset;
	if (pfw_hdr->type_id == FW_IMG_XIP_ID) {
		// Always skip XIP FW image loading
		uboot_frame_handle.skip_sub_img_load = 1;
		return ret;
	}
	uboot_frame_handle.skip_sub_img_load = 0;
	if (pfw_hdr->str_tbl != 0xFFFFFFFF) {
		if ((pfw_hdr->type_id == FW_IMG_BL_ID) ||
			(pfw_hdr->type_id == FW_IMG_FWHS_S_ID) ||
			(pfw_hdr->type_id == FW_IMG_FWHS_S_ID) ||
			(pfw_hdr->type_id == FW_IMG_CPFW_ID) ||
			(pfw_hdr->type_id == FW_IMG_FWUPD_ID)) {
			uboot_frame_handle.ram_start_tbl = (pfw_hdr->str_tbl);
		}
	}
#if UART_BOOT_DBG
	dbg_printf("ImgHde:");
	dump_for_one_bytes((u8 *)pheader, sizeof(fm_image_header_t));
#endif
	return ret;
}

int32_t uart_boot_tlv_process_section_header(uint8_t *pimg_hdr)
{
	uint8_t *ptr = (uint8_t *)pimg_hdr;
	sect_hdr_t *psect_hdr = NULL;
	int ret = SUCCESS;

	if (!ptr) {
		ret = FAIL;
		return ret;
	}
	psect_hdr = (psect_hdr_t)&tmp_sect_hdr[0];
	_memcpy((void *)psect_hdr, (void *)ptr, sizeof(sect_hdr_t));
	uboot_frame_handle.next_section_offset = psect_hdr->nxtoffset;
	uboot_frame_handle.offset_2_next_section = psect_hdr->nxtoffset;
	uboot_frame_handle.ram_img_offset = 0;
	uboot_frame_handle.ram_img_size = psect_hdr->seclen;

	// add image loading log, so we can erase them when load failed
	uboot_frame_handle.ram_img_start_addr = psect_hdr->dest;
	return SUCCESS;
}

int32_t uart_boot_tlv_process_img_payload(uint32_t *pimg_payload, uint32_t remain_len, uint32_t *pconsume_len, uint8_t *load_done)
{
	uint32_t tmp_size;
	int ret = SUCCESS;

	if (uboot_frame_handle.ram_img_size == 0) {
		*pconsume_len = 0;
		*load_done = 1;
		return ret;
	}

	// Image is plain text, just copy image data from temp memory buffer to assigned RAM
	if (remain_len <= uboot_frame_handle.ram_img_size) {
		tmp_size = remain_len;
	} else {
		tmp_size = uboot_frame_handle.ram_img_size;
	}
	*pconsume_len = tmp_size;
	_memcpy((void *)(uboot_frame_handle.ram_img_start_addr), (uint8_t *)(pimg_payload), tmp_size);
	rtl_dcache_clean_by_addr((uint32_t *)(uboot_frame_handle.ram_img_start_addr), tmp_size);
	uboot_frame_handle.ram_img_start_addr += tmp_size;
	uboot_frame_handle.ram_img_size -= tmp_size;

	if (uboot_frame_handle.ram_img_size == 0) {
		*load_done = 1;
	} else {
		*load_done = 0;
	}
	return ret;
}


int32_t uart_boot_frame_handler(char *frame_ptr,  uint32_t frame_num, uint32_t frame_size)
{
	uint32_t tmp_offset;
	uint32_t min_needed_len;    // minimum needed data length for processing
	uint32_t remain_len;
	uint8_t *pframe_offset;
	int32_t ret;

	if (uboot_frame_handle.tmp_buf_offset != 0) {
		uart_boot_store_frame_append((uint8_t *)frame_ptr, frame_size);
		remain_len = uboot_frame_handle.tmp_buf_offset;
		pframe_offset = uart_boot_temp_buf;
		frame_ptr = (char *)pframe_offset;
		uboot_frame_handle.tmp_buf_offset = 0;
	} else {
		remain_len = frame_size;
		pframe_offset = (uint8_t *)frame_ptr;
	}

	//dcahce clean
	rtl_dcache_clean_by_addr((uint32_t *)pframe_offset, remain_len);

	while (remain_len > 0) {
		UART_BOOT_DBG_PRINTF("remain_len=%lu, offset=%lu, state=%u\r\n", remain_len, (uint32_t)pframe_offset - (uint32_t)frame_ptr, uboot_frame_handle.state);

		switch (uboot_frame_handle.state) {
		case UART_BOOT_STATE_KEY_GEN:
			if (frame_num != 1) {
				//DBG_BOOT_ERR ("uart_boot_frame_handler: Didn't got the Image header\r\n");
				uboot_frame_handle.state = UART_BOOT_STATE_ERR;
				return -1;
			}
			min_needed_len = (OTA_SIGNATURE_SIZE + sizeof(public_key_type_t) * MAX_PUBLIC_KEY_NUM);
			// Because secure boot redefine, so Ignore zii OTA signature and 6 x public key [32 + (6 x 32)] bytes and fst info
			//ret = uart_boot_key_gen((uint32_t *)pframe_offset);
			ret = SUCCESS;
			remain_len -= min_needed_len;
			pframe_offset += min_needed_len;

			if (ret == SUCCESS) {
				uboot_frame_handle.state = UART_BOOT_STATE_IMG_HDR;
			} else {
				uboot_frame_handle.state = UART_BOOT_STATE_ERR;
				remain_len = 0;
			}
			break;

		case UART_BOOT_STATE_IMG_HDR:
			min_needed_len = sizeof(fw_sub_image_type_t);
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				ret = uart_boot_process_image_header((uint32_t *)pframe_offset);
				remain_len -= min_needed_len;
				pframe_offset += min_needed_len;
				if (ret == SUCCESS) {
					uboot_frame_handle.offset_2_next_img -= min_needed_len;
					if (uboot_frame_handle.skip_sub_img_load == 0) {
						uboot_frame_handle.state = UART_BOOT_STATE_SECTION_HDR;
					} else {
						uboot_frame_handle.state = UART_BOOT_STATE_W4_SUBIMG_HDR;
					}
				} else {
					uboot_frame_handle.state = UART_BOOT_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_STATE_SECTION_HDR:
			min_needed_len = sizeof(fw_image_section_type_t);
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				ret = uart_boot_process_section_header((uint32_t *)pframe_offset);
				remain_len -= min_needed_len;
				pframe_offset += min_needed_len;
				if (ret == SUCCESS) {
					uboot_frame_handle.offset_2_next_section -= min_needed_len;
					uboot_frame_handle.offset_2_next_img -= min_needed_len;
					uboot_frame_handle.state = UART_BOOT_STATE_IMG_DATA;
				} else {
					uboot_frame_handle.state = UART_BOOT_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_STATE_IMG_DATA:
			min_needed_len = 16;    // decryption unit
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				uint32_t consume_len;
				uint8_t load_done;

				ret = uart_boot_process_img_payload((uint32_t *)pframe_offset, remain_len, &consume_len, &load_done);
				uboot_frame_handle.offset_2_next_section -= consume_len;
				uboot_frame_handle.offset_2_next_img -= consume_len;
				if (consume_len < remain_len) {
					remain_len -= consume_len;
					pframe_offset += consume_len;
				} else {
					remain_len = 0;
				}

				if (ret == SUCCESS) {
					if (load_done) {
#if 0
						// Set SCE from write mode to read mode
						if (uboot_frame_handle.sce_switch_rd_mode) {
							sce_sce_ctrl_t sce_ctrl;

							// Image load done, switch SCE Accsee mode from write to read
							sce_ctrl.w = hal_sce_read_reg_rtl8710c(REG_SCE_CTRL);
							sce_ctrl.b.sce_mode_type = ReadMode;
							hal_sce_write_reg_rtl8710c(REG_SCE_CTRL, sce_ctrl.w);
							uboot_frame_handle.sce_switch_rd_mode = 0;
						}
#endif
						if (uboot_frame_handle.next_section_offset != IMG_LINK_LIST_END) {
							// Load next section
							if (uboot_frame_handle.offset_2_next_section > 0) {
								uboot_frame_handle.state = UART_BOOT_STATE_W4_SECTION_HDR;
							} else {
								uboot_frame_handle.state = UART_BOOT_STATE_SECTION_HDR;
							}
						} else {
							if (uboot_frame_handle.next_img_offset != IMG_LINK_LIST_END) {
								// Load next Sub-image
								if (uboot_frame_handle.offset_2_next_img > 0) {
									uboot_frame_handle.state = UART_BOOT_STATE_W4_SUBIMG_HDR;
								} else {
									uboot_frame_handle.state = UART_BOOT_STATE_IMG_HDR;
								}
							} else {
								uboot_frame_handle.state = UART_BOOT_STATE_W_DISCON;
							}
						}
					}
				} else {
					uboot_frame_handle.state = UART_BOOT_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_STATE_W4_SECTION_HDR:
			tmp_offset = uboot_frame_handle.offset_2_next_section;
			if (tmp_offset <= remain_len) {
				remain_len -= tmp_offset;
				pframe_offset += tmp_offset;
				uboot_frame_handle.state = UART_BOOT_STATE_SECTION_HDR;
				uboot_frame_handle.offset_2_next_img -= tmp_offset;
			} else {
				uboot_frame_handle.offset_2_next_section -= remain_len;
				uboot_frame_handle.offset_2_next_img -= remain_len;
				remain_len = 0;
			}
			break;

		case UART_BOOT_STATE_W4_SUBIMG_HDR:
			tmp_offset = uboot_frame_handle.offset_2_next_img;
			if (tmp_offset <= remain_len) {
				remain_len -= tmp_offset;
				pframe_offset += tmp_offset;
				if (uboot_frame_handle.next_img_offset != IMG_LINK_LIST_END) {
					uboot_frame_handle.state = UART_BOOT_STATE_IMG_HDR;
				} else {
					uboot_frame_handle.state = UART_BOOT_STATE_W_DISCON;
				}
			} else {
				uboot_frame_handle.offset_2_next_img -= remain_len;
				remain_len = 0;
			}
			break;

		case UART_BOOT_STATE_W_DISCON:
			// image load finished, wait for disconnect
			// do not thing
			remain_len = 0;
			break;

		case UART_BOOT_STATE_ERR:
		default:
			remain_len = 0;
			break;
		}
	}

	return frame_size;
}

int32_t uart_boot_tlv_frame_handler(char *frame_ptr,  uint32_t frame_num, uint32_t frame_size)
{
	int32_t ret;
	uint32_t tmp_offset;
	uint32_t min_needed_len;    // minimum needed data length for processing
	uint32_t remain_len;
	uint8_t *pframe_offset;
	manif_hdr_t *pmani_hdr = NULL;
	char lbl_name[MANIFEST_MAX_LABEL_SIZE + 1] = "";

	if (uboot_frame_handle.tmp_buf_offset != 0) {   // previous data in tmp buffer, new data append it
		uart_boot_store_frame_append((uint8_t *)frame_ptr, frame_size);
		remain_len = uboot_frame_handle.tmp_buf_offset; // update remain length, so later parsing loop can handle
		pframe_offset = uart_boot_temp_buf;     // parsing obj from tmp buf
		frame_ptr = (char *)pframe_offset;
		uboot_frame_handle.tmp_buf_offset = 0;
	} else {    // no previous data in tmp buffer, new data directly handle
		remain_len = frame_size;
		pframe_offset = (uint8_t *)frame_ptr;
	}

	//dcahce clean
	//rtl_dcache_clean_by_addr ((uint32_t *)pframe_offset, remain_len);
	while (remain_len > 0) {
		UART_BOOT_DBG_PRINTF("remain_len=%lu, offset=%lu, state=%u\r\n", remain_len, (uint32_t)pframe_offset - (uint32_t)frame_ptr, uboot_frame_handle.state);

		switch (uboot_frame_handle.state) {
		case UART_BOOT_TLV_STATE_IMG_MANIF:
			if (frame_num != 1) {
				//DBG_BOOT_ERR ("uart_boot_frame_handler: Didn't got the Image header\r\n");
				uboot_frame_handle.state = UART_BOOT_STATE_ERR;
				return -1;
			}
			pmani_hdr = (pmanif_hdr_t)pframe_offset;
			_memset(lbl_name, '\0', MANIFEST_MAX_LABEL_SIZE + 1);
			_strncpy(lbl_name, (void *)(pmani_hdr->lbl), MANIFEST_MAX_LABEL_SIZE);
			if (chk_manifest_lbl(lbl_name)) {
				// Invalid Label
				ret = FAIL;
			} else {
				ret = SUCCESS;
				//min_needed_len = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);  // 4K bytes-aligned for IMG HDR start
				uboot_frame_handle.next_img_offset = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);
				uboot_frame_handle.offset_2_next_img = (((sizeof(img_manifest_t) >> IMG_HDR_START_ALIGN_SHIFT) + 1) * IMG_HDR_START_ALIGN_SIZE);
				min_needed_len = XM_BUFFER_SIZE;
				uboot_frame_handle.offset_2_next_img -= min_needed_len;
				remain_len -= min_needed_len;
				pframe_offset += min_needed_len;
			}

			if (ret == SUCCESS) {
				uboot_frame_handle.state = UART_BOOT_TLV_STATE_W4_SUBIMG_HDR;
				if (remain_len != 0) {    // should consume a complete 4KB buffer, if not (something happen), return error
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_ERR;
					remain_len = 0;
				}
			} else {
				uboot_frame_handle.state = UART_BOOT_TLV_STATE_ERR;
				remain_len = 0;
			}
			break;

		case UART_BOOT_TLV_STATE_IMG_HDR:
			min_needed_len = sizeof(fw_img_hdr_t);
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				ret = uart_boot_tlv_process_image_header((uint8_t *)pframe_offset);
				remain_len -= min_needed_len;
				pframe_offset += min_needed_len;
				if (ret == SUCCESS) {
					uboot_frame_handle.offset_2_next_img -= min_needed_len;
					if (uboot_frame_handle.skip_sub_img_load == 0) {
						uboot_frame_handle.state = UART_BOOT_TLV_STATE_SECTION_HDR;
					} else {
						uboot_frame_handle.state = UART_BOOT_TLV_STATE_W4_SUBIMG_HDR;
					}
				} else {
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_TLV_STATE_SECTION_HDR:
			min_needed_len = sizeof(sect_hdr_t);
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				ret = uart_boot_tlv_process_section_header((uint32_t *)pframe_offset);
				remain_len -= min_needed_len;
				pframe_offset += min_needed_len;
				if (ret == SUCCESS) {
					uboot_frame_handle.offset_2_next_section -= min_needed_len;
					uboot_frame_handle.offset_2_next_img -= min_needed_len;
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_IMG_DATA;
				} else {
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_STATE_IMG_DATA:
			min_needed_len = 32;    // decryption unit
			if (remain_len < min_needed_len) {
				uart_boot_store_frame(pframe_offset, remain_len);
				remain_len = 0;
			} else {
				uint32_t consume_len;
				uint8_t load_done;

				ret = uart_boot_tlv_process_img_payload((uint32_t *)pframe_offset, remain_len, &consume_len, &load_done);
				uboot_frame_handle.offset_2_next_section -= consume_len;
				uboot_frame_handle.offset_2_next_img -= consume_len;
				if (consume_len < remain_len) {
					remain_len -= consume_len;
					pframe_offset += consume_len;
				} else {
					remain_len = 0;
				}

				if (ret == SUCCESS) {
					if (load_done) {
						if (uboot_frame_handle.next_section_offset != IMG_LINK_LIST_END) {
							// Load next section
							if (uboot_frame_handle.offset_2_next_section > 0) {
								uboot_frame_handle.state = UART_BOOT_TLV_STATE_W4_SECTION_HDR;
							} else {
								uboot_frame_handle.state = UART_BOOT_TLV_STATE_SECTION_HDR;
							}
						} else {
							if (uboot_frame_handle.next_img_offset != IMG_LINK_LIST_END) {
								// Load next Sub-image
								if (uboot_frame_handle.offset_2_next_img > 0) {
									uboot_frame_handle.state = UART_BOOT_TLV_STATE_W4_SUBIMG_HDR;
								} else {
									uboot_frame_handle.state = UART_BOOT_TLV_STATE_IMG_HDR;
								}
							} else {
								uboot_frame_handle.state = UART_BOOT_TLV_STATE_W_DISCON;
							}
						}
					}
				} else {
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_ERR;
					remain_len = 0;
				}
			}
			break;

		case UART_BOOT_TLV_STATE_W4_SECTION_HDR:
			tmp_offset = uboot_frame_handle.offset_2_next_section;
			if (tmp_offset <= remain_len) {
				remain_len -= tmp_offset;
				pframe_offset += tmp_offset;
				uboot_frame_handle.state = UART_BOOT_TLV_STATE_SECTION_HDR;
				uboot_frame_handle.offset_2_next_img -= tmp_offset;
			} else {
				uboot_frame_handle.offset_2_next_section -= remain_len;
				uboot_frame_handle.offset_2_next_img -= remain_len;
				remain_len = 0;
			}
			break;

		case UART_BOOT_TLV_STATE_W4_SUBIMG_HDR:
			tmp_offset = uboot_frame_handle.offset_2_next_img;
			if (tmp_offset <= remain_len) {
				remain_len -= tmp_offset;
				pframe_offset += tmp_offset;
				if (uboot_frame_handle.next_img_offset != IMG_LINK_LIST_END) {
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_IMG_HDR;
				} else {
					uboot_frame_handle.state = UART_BOOT_TLV_STATE_W_DISCON;
				}
			} else {
				uboot_frame_handle.offset_2_next_img -= remain_len;
				remain_len = 0;
			}
			break;

		case UART_BOOT_TLV_STATE_W_DISCON:
			// image load finished, wait for disconnect
			// do not thing
			remain_len = 0;
			break;

		case UART_BOOT_TLV_STATE_ERR:
		default:
			remain_len = 0;
			break;
		}
	}

	return frame_size;
}


void uart_boot_init(void)
{
	// Initial Cryptal engine HW

	// read priviate key from super secure eFuse

	_memset((void *)&uboot_frame_handle, 0, sizeof(uboot_xm_frame_handler_t));
}

int32_t uart_boot_load_imag(hal_uart_adapter_t *potu_uart)
{
	int32_t fw_dnload_size;

	//dbg_printf("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);
	uart_boot_init();

	// <NOTE> Cannot use DDR resources, because DDR is not init yet!!!

	// Start XModem transfer
	//dbg_printf("Start XModem..\r\n");
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)uart_boot_frame_handler);
#else
	if (LD_IMF_FT_TLV == boot_imgft) {
		xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)uart_boot_tlv_frame_handler);
	} else {
		xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)uart_boot_frame_handler);
	}
#endif

#if CONFIG_FPGA // for FPGA workaround pg_service.py can't recv NAK delay 2ms By Raymond
	int loop_cnt;
	for (loop_cnt = 0; loop_cnt < 20000; loop_cnt++) {
		__NOP();
	}
#endif

	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (16 * 1024 * 1024)); // maximum 16M
	//dbg_printf("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);
	return fw_dnload_size;
}

int32_t uart_boot(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg1_t cfg1;
	uint32_t tx_pin;
	uint32_t rx_pin;
	uint32_t baud_rate;
	int32_t ret;
	stdio_port_backup_t stdio_uart_bkup;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	volatile uint8_t imgft_cfg = LD_IMF_FT_TLV;

	cfg1.byte = potp_boot_cfg->byte.cfg1.byte;
	rx_pin = rom_log_uart_rx_pins[cfg1.bit.uart_boot.rx_pin_sel][cfg1.bit.uart_boot.port_sel];
	tx_pin = rom_log_uart_tx_pins[cfg1.bit.uart_boot.tx_pin_sel][cfg1.bit.uart_boot.port_sel];
	baud_rate = boot_uart_baud_sel_map[potp_boot_cfg->byte.cfg2.bit.uart_boot_baud_sel];

	dbg_printf("Image Load over UART%u baud=%lu\r\n", hal_rtl_uart_rx_pin_to_idx(rx_pin), baud_rate);
	otu_uart_port_open(tx_pin, rx_pin, baud_rate, 0, 0, &stdio_uart_bkup);
	imgft_cfg = potp_boot_cfg7->bit.ntlv_img_ld_en;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	boot_imgft = LD_IMF_FT_NTLV;
#else
	boot_imgft = imgft_cfg;
#endif

	ret = uart_boot_load_imag(&otu_uart);
	if ((xmodem_ctrl.result != 0) || (ret <= 0)) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}
	otu_uart_port_close(&stdio_uart_bkup);

	if (ret > 0) {
		dbg_printf(" Load Image over UART: ram_start_tbl @0x%x\r\n", uboot_frame_handle.ram_start_tbl);
		*pram_start_func = (PRAM_FUNCTION_START_TABLE)(uboot_frame_handle.ram_start_tbl);
		return SUCCESS;
	} else {
		dbg_printf(" Load Image over UART Failed\r\n");
		return FAIL;
	}
}

