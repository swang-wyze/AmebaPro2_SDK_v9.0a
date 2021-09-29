/**************************************************************************//**
 * @file     fw_otu.c
 * @brief    Implement the FW image download to flash over the UART XModem.
 *
 * @version  V1.00
 * @date     2017-02-20
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
#include "xmport_uart.h"
#include "xmodem.h"
#include "strproc.h"
#include "memory.h"
#include "rtl8195bhp_symbns4s.h"
#include "efuse_boot_cfg.h"
#include "fw_uart_boot.h"

#define SHELL_ROM_CMD_LIST_SIZE               13

//extern const char _img_hdr_sign[];
extern const u32 spic_calibration_pattern[4];
extern uint8_t uart_tr_pin[];
extern u8 boot_uart_sel_map[];
extern const u8 boot_uart_pin_sel_map[];
extern uint32_t boot_uart_baud_sel_map[];
extern shell_command_entry_t shell_rom_cmd_list[];
extern partition_table_t export_partition_tbl;
extern char ota_signature[];

extern hal_uart_adapter_t stdio_uart;
extern hal_spic_adaptor_t _hal_spic_adaptor;
extern uboot_xm_frame_handler_t uboot_frame_handle;
extern uint8_t temp_hash_out_buf[];   // temp buffer for hash result
extern uint8_t ss_priv_key[];  // Priviate key for AES, get it from super secure EFuse, must be cleaned before jump to RAM
extern hal_crypto_adapter_t _crypto_engine;

extern hal_status_t fw_spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, u8 io_pin_sel);
extern hal_status_t fw_spic_deinit(phal_spic_adaptor_t phal_spic_adaptor);

extern _LONG_CALL_ int32_t fw_decrypt_partition_tbl(void);
extern _LONG_CALL_ void rom_sboot_bss_clean_up(void);
extern _LONG_CALL_ int32_t fw_get_serial(uint8_t secure_lock, uint8_t fw_rec_idx, uint32_t *sn);
extern int32_t uart_boot_load_imag(hal_uart_adapter_t *potu_uart);
extern int32_t uart_boot_notify_ls_img_rdy(void);
extern int rtl_crypto_sha2_256_init_s(void);
extern int rtl_crypto_sha2_256_process_s(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);
extern int rtl_crypto_sha2_256_update_s(IN const u8 *message, IN const u32 msglen);
extern int rtl_crypto_sha2_256_final_s(OUT u8 *pDigest);
extern int rtl_cryptoEngine_init_s(void *pIE1);
extern int rtl_cryptoEngine_deinit_s(void *pIE1);

typedef struct otu_xm_frame_handler_s {
	uint8_t got_1st_frame;  // is the 1st frame of the XModem transfer
	uint8_t update_partition;    // the partition to be update
	uint32_t flash_start;   // the flash start address to be write
	uint32_t flash_offset;   // the flash offset address to be write
} otu_xm_frame_handler_t, *potu_xm_frame_handler_t;

SECTION_ROM_TEMP_BSS hal_uart_adapter_t otu_uart;
SECTION_ROM_TEMP_BSS xmodem_uart_port_handler_t xm_uart_port;
SECTION_ROM_TEMP_BSS uint8_t xm_rx_buf[XM_BUF_SIZE] __ALIGNED(32);    // the buffer for UART RX FIFO
SECTION_ROM_TEMP_BSS XMODEM_CTRL xmodem_ctrl;
SECTION_ROM_TEMP_BSS uint8_t xmodem_frame_buf[XM_BUFFER_SIZE] __ALIGNED(32);     // the buffer for XModem RX Frame buffer
SECTION_ROM_TEMP_BSS otu_xm_frame_handler_t frame_handler;
SECTION_ROM_TEMP_BSS uint8_t flah_read_temp_buf[XM_BUFFER_SIZE];     // the buffer for flash data read back for the flash write varification
SECTION_ROM_TEMP_BSS volatile uint8_t otu_disconnect;     // OTU disconnect flag
SECTION_ROM_TEMP_BSS static uint32_t flash_pg_offset;     // to record the latest PG command flash offset

const char otu_ok[] = "OK";
const char otu_err[] = "ER";
const char otu_hash_ret[] = "hashs ";
const char otu_ping[] = "ping";

#if 0
// For XModem test
int32_t otu_rx_frame_handler_test(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	dbg_printf(".");
	return frame_size;
}
#endif

int32_t otu_rx_frame_handler(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	fm_image_header_t *ppt_header;
	uint32_t retry;
	uint32_t retry_lim = 3;

//    dbg_printf("%s==> frame_num=%u frame_size=%u\r\n", __FUNCTION__, frame_num, frame_size);
//    dbg_printf(".");

	if ((frame_num == 1) && (frame_handler.got_1st_frame == 0)) {
		// It's the 1st frame, verify the format
		// check the signature
		if (!_memcmp(ptr, spic_calibration_pattern, 16)) {
			// Start with Flash calibration data, it means download the whole flash image
			ppt_header = (fm_image_header_t *)((char *)ptr + 32);
			frame_handler.update_partition = FW_PT_PT;
			frame_handler.flash_start = SPI_FLASH_BASE;
			frame_handler.flash_offset = 0;
			frame_handler.got_1st_frame = 1;
		} else {
			ppt_header = (fm_image_header_t *)ptr;

#if 0
			if (_memcmp(_img_hdr_sign, ppt_header->signature.sign, 4)) {
				DBG_MISC_ERR("otu_rx_frame_handler: PlainHdr.Sign Incorrect\r\n");
				return -1;
			}
#endif
			frame_handler.got_1st_frame = 1;
			if (ppt_header->img_type == FW_PT_PT) {
				// Start with partition table, so it goining to update the entiry flash image
				frame_handler.update_partition = FW_PT_PT;
				frame_handler.flash_start = SPI_FLASH_BASE;
				frame_handler.flash_offset = 0;
			} else if ((ppt_header->img_type == FW_PT_FW1) || (ppt_header->img_type == FW_PT_FW2)) {
				// TODO: other type of partition
				// TODO: search the partition of old FW to be update
			}
		}
	}

	if (frame_handler.got_1st_frame == 0) {
		DBG_MISC_ERR("otu_rx_frame_handler: Didn't got the Image header\r\n");
		return 0;
	}
	// TODO: parse image and check hash
	// start to program Flash
	if ((frame_handler.flash_offset & 0xFFF) == 0) {
		// sector boundary, erase flash sector first
//        dbg_printf("Sector Erase 0x%x\r\n", frame_handler.flash_offset);
		flash_sector_erase_rtl8195bhp(&_hal_spic_adaptor, frame_handler.flash_offset);
	}

//    dbg_printf("Burst write 0x%x\r\n", frame_handler.flash_offset);
	for (retry = 0; retry < retry_lim; retry++) {
		flash_burst_write_rtl8195bhp(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, (u8 *)ptr);
		flash_burst_read_rtl8195bhp(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, flah_read_temp_buf);
		if (!_memcmp((void *)ptr, (void *) flah_read_temp_buf, frame_size)) {
			break;
		} else {
			DBG_MISC_WARN("otu_rx_frame_handler: flash write read-back incorrect\r\n");
		}
	}

	if (retry >= retry_lim) {
		DBG_MISC_ERR("otu_rx_frame_handler: Flash write err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
}

int32_t otu_rx_frame_direct_wite(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	uint32_t retry;
	uint32_t retry_lim = 3;

//    dbg_printf(".");

	// start to program Flash
	if ((frame_handler.flash_offset & 0xFFF) == 0) {
		// sector boundary, erase flash sector first
//        dbg_printf("Sector Erase 0x%x\r\n", frame_handler.flash_offset);
		flash_sector_erase_rtl8195bhp(&_hal_spic_adaptor, frame_handler.flash_offset);
	}

//    dbg_printf("Burst write 0x%x\r\n", frame_handler.flash_offset);
	for (retry = 0; retry < retry_lim; retry++) {
		flash_burst_write_rtl8195bhp(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, (u8 *)ptr);
		flash_burst_read_rtl8195bhp(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, flah_read_temp_buf);
		if (!_memcmp((void *)ptr, (void *) flah_read_temp_buf, frame_size)) {
			break;
		} else {
			DBG_MISC_WARN("otu_rx_frame_handler: flash write read-back incorrect\r\n");
		}
	}

	if (retry >= retry_lim) {
		DBG_MISC_ERR("otu_rx_frame_handler: Flash write err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
}

#define INTERNAL_HS_RAM_START          0x20100000UL
#define INTERNAL_HS_RAM_END            0x20179FFFUL
#define INTERNAL_LS_RAM_START          0x20200000UL
#define INTERNAL_LS_RAM_END            0x2020BFFFUL

int32_t otu_rx_frame_ram_wite(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	uint32_t retry;
	uint32_t retry_lim = 3;
	uint8_t *pdest;
	uint32_t end_addr;

	pdest = (uint8_t *)(frame_handler.flash_start + frame_handler.flash_offset);
	end_addr = (uint32_t)pdest + frame_size;

	if ((frame_handler.flash_start >> 20) == (INTERNAL_HS_RAM_START >> 20)) {
		// Load HS Image
		if (((uint32_t)pdest < INTERNAL_HS_RAM_START) || ((uint32_t)pdest > INTERNAL_HS_RAM_END)) {
			return -1;
		}

		if ((end_addr < INTERNAL_HS_RAM_START) || (end_addr > INTERNAL_HS_RAM_END)) {
			return -1;
		}
	} else if ((frame_handler.flash_start >> 20) == (INTERNAL_LS_RAM_START >> 20)) {
		// Load LS Image
		if (((uint32_t)pdest < INTERNAL_LS_RAM_START) || ((uint32_t)pdest > INTERNAL_LS_RAM_END)) {
			return -1;
		}

		if ((end_addr < INTERNAL_LS_RAM_START) || (end_addr > INTERNAL_LS_RAM_END)) {
			return -1;
		}

		// LS platform RAM Address 0x20200000 is map to HS platform Address 0xB0200000
		pdest = (uint8_t *)(((uint32_t)pdest & 0x00FFFFFFUL) | 0xB0000000UL);
	} else {
		return -1;
	}

	_memcpy((void *)pdest, (void *)ptr, frame_size);
	for (retry = 0; retry < retry_lim; retry++) {
		_memcpy((void *)pdest, ptr, frame_size);
		if (!_memcmp((void *)ptr, (void *) pdest, frame_size)) {
			break;
		}
	}

	if (retry >= retry_lim) {
		DBG_MISC_ERR("otu_rx_frame_ram_wite: RAM img load err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;
	uboot_frame_handle.ls_img_size = frame_handler.flash_offset;

	return frame_size;
}

hal_status_t otu_uart_port_open(uint32_t uart_idx, uint32_t uart_pin_sel,
								uint32_t baud_rate, uint32_t parity, uint32_t flow_ctrl,
								stdio_port_backup_t *stdio_uart_bkup)
{
	uint8_t *pin_list;
	hal_status_t ret;

	if (uart_idx == stdio_uart.uart_idx) {
		// use the same UART as Log UART
		stdio_uart_bkup->cfg_err = ConfigDebugErr;
		stdio_uart_bkup->cfg_info = ConfigDebugInfo;
		stdio_uart_bkup->cfg_warn = ConfigDebugWarn;
		stdio_uart_bkup->is_inited = stdio_uart.is_inited;
		ConfigDebugErr = 0;
		ConfigDebugInfo = 0;
		ConfigDebugWarn = 0;
		if (stdio_uart.is_inited) {
			stdio_uart_bkup->baud_rate = stdio_uart.baudrate;
			stdio_uart_bkup->parity = stdio_uart.parity_type;
			stdio_uart_bkup->flow_ctrl = stdio_uart.flow_ctrl;
			hal_uart_deinit_rtl8195bhp(&stdio_uart);
			pin_list = &uart_tr_pin[(stdio_uart.uart_idx * 12) + (stdio_uart.pin_mux_sel * 3)];
			hal_pinmux_unregister_rtl8195bhp((io_pin_t *)pin_list, (PID_UART0 + stdio_uart.uart_idx));
		}
	}

	pin_list = &uart_tr_pin[(uart_idx * 12) + (uart_pin_sel * 3)];
	hal_pinmux_register_rtl8195bhp((io_pin_t *)pin_list, (PID_UART0 + uart_idx));

	// uart_sel: [15:8]= UART index, [7:0]= UART Pin Sel
	// flash_sel: [15:8]= Flash IO mode, [7:0]= Flash Pin Sel
	ret = hal_uart_init_rtl8195bhp(&otu_uart, uart_idx, uart_pin_sel, NULL);
	ret |= hal_uart_set_baudrate_rtl8195bhp(&otu_uart, baud_rate);
	ret |= hal_uart_set_format_rtl8195bhp(&otu_uart, 8, parity, 1);
	hal_irq_enable_rtl8195bhp(UART_IRQn);

	if (flow_ctrl == 0) {
		flow_ctrl = UartFlowCtlNone;
	} else {
		flow_ctrl = UartFlowCtlRTSCTS;
	}
	ret |= hal_uart_set_flow_control_rtl8195bhp(&otu_uart, flow_ctrl);

	if ((uart_idx == stdio_uart.uart_idx) && (ret != HAL_OK)) {
		ConfigDebugErr = stdio_uart_bkup->cfg_err;
		ConfigDebugInfo = stdio_uart_bkup->cfg_info;
		ConfigDebugWarn = stdio_uart_bkup->cfg_warn;
		DBG_MISC_ERR("OTU UART(Idx=%u Sel=%u) Open Err\r\n", uart_idx, uart_pin_sel);
	}

	return ret;
}

hal_status_t otu_uart_port_close(stdio_port_backup_t *stdio_uart_bkup)
{
	uint8_t *pin_list;
	uint32_t uart_idx;
	hal_status_t ret = HAL_OK;

	uart_idx = otu_uart.uart_idx;
	hal_uart_deinit_rtl8195bhp(&otu_uart);
	pin_list = &uart_tr_pin[(uart_idx * 12) + (otu_uart.pin_mux_sel * 3)];
	hal_pinmux_unregister_rtl8195bhp((io_pin_t *)pin_list, (PID_UART0 + uart_idx));

	if (uart_idx == stdio_uart.uart_idx) {
		if (stdio_uart_bkup->is_inited) {
			pin_list = &uart_tr_pin[(uart_idx * 12) + (stdio_uart.pin_mux_sel * 3)];
			ret = hal_pinmux_register_rtl8195bhp((io_pin_t *)pin_list, (PID_UART0 + uart_idx));
			ret |= hal_uart_init_rtl8195bhp(&stdio_uart, uart_idx, stdio_uart.pin_mux_sel, NULL);
			ret |= hal_uart_set_baudrate_rtl8195bhp(&stdio_uart, stdio_uart_bkup->baud_rate);
			ret |= hal_uart_set_format_rtl8195bhp(&stdio_uart, 8, stdio_uart_bkup->parity, 1);
			ret |= hal_uart_set_flow_control_rtl8195bhp(&stdio_uart, stdio_uart_bkup->flow_ctrl);
		}

		ConfigDebugErr = stdio_uart_bkup->cfg_err;
		ConfigDebugInfo = stdio_uart_bkup->cfg_info;
		ConfigDebugWarn = stdio_uart_bkup->cfg_warn;
	}

	return ret;
}

int32_t otu_fw_download(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset)
{
	hal_status_t ret;
	uint32_t fw_dnload_size;

	dbg_printf("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Initial Flash controller
	dbg_printf("Init Flash BitMod=%u PinSel=%u...\r\n", (flash_sel >> 8) & 0x0F, flash_sel & 0x0F);
	ret = fw_spic_init(&_hal_spic_adaptor, (flash_sel >> 8) & 0x0F, flash_sel & 0x0F);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU Flash(BitMod=%u Sel=%u) Init Err\r\n", (flash_sel >> 8) & 0x0F, flash_sel & 0x0F);
		return 0;
	}

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_offset = flash_offset;  // it must 4K aligned
	dbg_printf("Start XModem..\r\n");
//    xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_handler);
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_direct_wite);
	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (16 * 1024 * 1024)); // maximum 8M
	dbg_printf("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);
	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_uart_send_rtl8195bhp(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_uart_send_rtl8195bhp(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}
	flash_return_spi_rtl8195bhp(&_hal_spic_adaptor);

	return fw_dnload_size;
}

int32_t otu_fw_download_cmd(int argc, char **argv)
{
	uint8_t uart_id = 1;
	uint8_t uart_pin_sel = 0;
	uint8_t parity = 0;
	uint8_t flow_ctrl = 0;
	uint32_t baud = 2000000;
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	int32_t ret;
	stdio_port_backup_t stdio_uart_bkup;

//    DBG_INFO_MSG_ON(_DBG_UART_);
//    DBG_WARN_MSG_ON(_DBG_MISC_);
//    DBG_ERR_MSG_ON(_DBG_MISC_);

	if (argc >= 1) {
		uart_id = (uint8_t)_strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		uart_pin_sel = (uint8_t)_strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		baud = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	if (argc >= 4) {
		parity = (uint8_t)_strtoul((const char *)(argv[3]), (char **)NULL, 10);
	}

	if (argc >= 5) {
		flow_ctrl = (uint8_t)_strtoul((const char *)(argv[4]), (char **)NULL, 10);
	}

	if (argc >= 6) {
		flash_pin = _strtoul((const char *)(argv[5]), (char **)NULL, 10);
	}

	if (argc >= 7) {
		flash_io = _strtoul((const char *)(argv[6]), (char **)NULL, 10);
	}

	if (argc >= 8) {
		flash_offset = _strtoul((const char *)(argv[7]), (char **)NULL, 16);
	}

	dbg_printf("Image Downlaod: UART[Idx%u,Sel%u], baud=%lu, parity=%u, flow_ctrl=%u Flash[IO%u,Sel%u], Offset=0x%x\r\n",
			   uart_id, uart_pin_sel, baud, parity, flow_ctrl, flash_io, flash_pin, flash_offset);

	otu_uart_port_open(uart_id, uart_pin_sel, baud, parity, flow_ctrl, &stdio_uart_bkup);

	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);

	otu_uart_port_close(&stdio_uart_bkup);

	return ret;
}

int32_t otu_ram_fw_download(hal_uart_adapter_t *potu_uart, uint32_t start_addr)
{
//    hal_status_t ret;
	uint32_t fw_dnload_size;

	dbg_printf("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_start = start_addr;
	frame_handler.flash_offset = 0;
	dbg_printf("Start XModem..\r\n");
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_ram_wite);
	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (1024 * 1024)); // maximum 1M
	dbg_printf("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);
	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_uart_send_rtl8195bhp(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_uart_send_rtl8195bhp(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	return fw_dnload_size;
}

#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))

int32_t test_mode_ping_cmd(int argc, char **argv)
{
	hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ping, _strlen(otu_ping), 100);

	return 0;
}

int32_t test_mode_uart_cfg_cmd(int argc, char **argv)
{
	uint32_t baud_rate = 2000000;
	uint8_t parity = UartParityNone;
	uint8_t flow_ctrl = UartFlowCtlNone;
	hal_status_t ret;

	if (argc >= 1) {
		baud_rate = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		parity = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		flow_ctrl = _strtoul((const char *)(argv[2]), (char **)NULL, 16);
	}

	hal_uart_wait_tx_done_rtl8195bhp(&otu_uart, 100);

	ret = hal_uart_set_baudrate_rtl8195bhp(&otu_uart, baud_rate);
	ret |= hal_uart_set_format_rtl8195bhp(&otu_uart, 8, parity, 1);

	if (flow_ctrl == 0) {
		flow_ctrl = UartFlowCtlNone;
	} else {
		flow_ctrl = UartFlowCtlRTSCTS;
	}
	ret |= hal_uart_set_flow_control_rtl8195bhp(&otu_uart, flow_ctrl);

	hal_delay_ms(500);
	if (ret == HAL_OK) {
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	} else {
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	}

	return ret;
}

int32_t test_mode_img_download_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	int32_t ret;

	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		flash_offset = _strtoul((const char *)(argv[2]), (char **)NULL, 16);
	}

	flash_pg_offset = flash_offset;

	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;
}

int32_t test_mode_img1_update_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	int32_t ret;

//    dbg_printf("test_mode_otu_img1_update_cmd=>\r\n");
	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	ret = fw_decrypt_partition_tbl();
	rom_sboot_bss_clean_up();
	if (SUCCESS != ret) {
		DBG_MISC_ERR("test_mode_img1_update_cmd: rd pt tbl err\r\n");
		return -1;
	}

	flash_offset = export_partition_tbl.partition_record[0].start_addr;
	_memset((void *)&export_partition_tbl, 0, sizeof(partition_table_t));
	flash_pg_offset = flash_offset;
	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;

}

int32_t test_mode_img2_update_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	int32_t  ret;
	int32_t  update_fw_idx = 1;
	uint8_t  fw1_rec_idx;
	uint8_t  fw2_rec_idx;

	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		update_fw_idx = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("test_mode_img2_update_cmd: flash init err %d\r\n", ret);
	}

	ret = fw_decrypt_partition_tbl();
	rom_sboot_bss_clean_up();
	if (SUCCESS != ret) {
		DBG_MISC_ERR("test_mode_img2_update_cmd: rd pt tbl err\r\n");
		return -1;
	}

	DBG_MISC_INFO("test_mode_img2_update_cmd update fw%d\r\n", update_fw_idx);

	fw1_rec_idx = export_partition_tbl.image_info.fw1_idx;
	fw2_rec_idx = export_partition_tbl.image_info.fw2_idx;

	if (update_fw_idx == 1) {
		flash_offset = export_partition_tbl.partition_record[fw1_rec_idx].start_addr;
	} else {
		flash_offset = export_partition_tbl.partition_record[fw2_rec_idx].start_addr;
	}
	_memset((void *)&export_partition_tbl, 0, sizeof(partition_table_t));
	flash_pg_offset = flash_offset;
	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;
}

int32_t test_mode_img2x_update_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	int32_t  ret;
	uint32_t fw1_sn = 0;
	uint32_t fw2_sn = 0;
	uint8_t  fw1_valid = 0;
	uint8_t  fw2_valid = 0;
	int32_t  update_fw_idx;
	uint8_t  secure_lock;
	uint8_t  fw1_rec_idx;
	uint8_t  fw2_rec_idx;

	secure_lock = BootCfg2Reg->bit.secure_lock;

//    dbg_printf("test_mode_otu_img1_update_cmd=>\r\n");
	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (SUCCESS != fw_decrypt_partition_tbl()) {
		rom_sboot_bss_clean_up();
		DBG_MISC_ERR("test_mode_img2_update_cmd: rd pt tbl err\r\n");
		return -1;
	}

	fw1_rec_idx = export_partition_tbl.image_info.fw1_idx;
	fw2_rec_idx = export_partition_tbl.image_info.fw2_idx;

	if (fw_get_serial(secure_lock, fw1_rec_idx, &fw1_sn) == SUCCESS) {
		fw1_valid = 1;
	}

	if (fw_get_serial(secure_lock, fw2_rec_idx, &fw2_sn) == SUCCESS) {
		fw2_valid = 1;
	}

	update_fw_idx = -1;
	if (fw1_valid && fw2_valid) {
		if (fw2_sn >=  fw1_sn) {
			update_fw_idx = fw1_rec_idx;
		} else {
			update_fw_idx = fw2_rec_idx;
		}
	} else if (fw1_valid && !fw2_valid) {
		update_fw_idx = fw1_rec_idx;
	} else if (!fw1_valid && fw2_valid) {
		update_fw_idx = fw2_rec_idx;
	}

//    dbg_printf("test_mode_img2_update_cmd update fw%d\r\n", update_fw_idx);

	if (update_fw_idx >= 0) {
		flash_offset = export_partition_tbl.partition_record[update_fw_idx].start_addr;
	} else {
		DBG_MISC_ERR("test_mode_img2_update_cmd: no valid img\r\n");
	}
	rom_sboot_bss_clean_up();
	_memset((void *)&export_partition_tbl, 0, sizeof(partition_table_t));
	flash_pg_offset = flash_offset;
	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;
}

int32_t test_mode_uart_boot_cmd(int argc, char **argv)
{
	int32_t ret;

	ret = uart_boot_load_imag(&otu_uart);
	if ((xmodem_ctrl.result != 0) || (ret <= 0)) {
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	return ret;
}

int32_t test_mode_ram_img_download_cmd(int argc, char **argv)
{
	uint32_t start_addr;
	int32_t ret;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_ram_img_download_cmd: no start addr\r\n");
		return 0;
	}

	start_addr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	ret = otu_ram_fw_download(&otu_uart, start_addr);
	return ret;
}

int32_t test_mode_hash_req_cmd(int argc, char **argv)
{
	uint32_t flash_offset;
	int32_t size;
	int32_t hash_size;
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	hal_status_t ret;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_hash_req_cmd: no size\r\n");
		return 0;
	}

	flash_offset = SPI_FLASH_BASE + flash_pg_offset;
	size = _strtoul((const char *)(argv[0]), (char **)NULL, 10);

	// Initial Cryptal engine HW
	rtl_cryptoEngine_init_s(&_crypto_engine);
	_crypto_engine.arch_clean_dcache_by_size = (void(*)(uint32_t, int32_t))dcache_clean_by_addr_rtl8195bhp;
	_crypto_engine.arch_invalidate_dcache_by_size = (void(*)(uint32_t, int32_t))dcache_invalidate_by_addr_rtl8195bhp;

	if (argc >= 2) {
		flash_io = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		flash_pin = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("test_mode_hash_req_cmd SPIC(io=%u pin=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	if (rtl_crypto_sha2_256_init_s() != SUCCESS) {
		DBG_MISC_ERR("test_mode_hash_req_cmd: Hash Init Err!\r\n");
		return 0;
	}

	// Hash length seems cannot over 16K, or hash result is incorrect
	// we need to calculate hash block by block
	while (size > 0) {
		if (size > BOOT_IMG_HASH_BLK_SIZE) {
			hash_size = BOOT_IMG_HASH_BLK_SIZE;
		} else {
			hash_size = size;
		}
		rtl_crypto_sha2_256_update_s((uint8_t *)flash_offset, hash_size);
		size -= hash_size;
		flash_offset += hash_size;
	}
	rtl_crypto_sha2_256_final_s(temp_hash_out_buf);
	flash_return_spi_rtl8195bhp(&_hal_spic_adaptor);

#if 0//CONFIG_FPGA
	dbg_printf("hash result:\r\n");
	dump_for_one_bytes(temp_hash_out_buf, 32);
#endif

	// send back the hash result over UART
	hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_hash_ret, _strlen(otu_hash_ret), 100);
	hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)temp_hash_out_buf, HASH_RESULT_SIZE, 500);

	return 0;
}

int32_t test_mode_flash_chip_erase_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	int32_t ret;

	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("Flash chip erase init err: BitMod=%u Sel=%u\r\n", flash_io, flash_pin);
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	flash_chip_erase_rtl8195bhp(&_hal_spic_adaptor);
//    flash_return_spi_rtl8195bhp (&_hal_spic_adaptor);
	fw_spic_deinit(&_hal_spic_adaptor);
	hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	return ret;
}

int32_t test_mode_flash_sector_erase_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = SpicDualIOPin;
	uint32_t flash_offset = 0;
	uint32_t len = 0;
	uint32_t erase_offset;
	int32_t ret;

	if (argc < 2) {
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
		return 0;
	}

	flash_offset = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	len = _strtoul((const char *)(argv[1]), (char **)NULL, 16);

	if (argc >= 3) {
		flash_io = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	if (argc >= 4) {
		flash_pin = _strtoul((const char *)(argv[3]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("Flash sector erase init err: BitMod=%u Sel=%u\r\n", flash_io, flash_pin);
		hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	erase_offset = flash_offset & 0xFFFFF000;
	while (erase_offset < (flash_offset + len)) {
		flash_sector_erase_rtl8195bhp(&_hal_spic_adaptor, erase_offset);
		erase_offset += 0x1000;
	}
	flash_return_spi_rtl8195bhp(&_hal_spic_adaptor);
	hal_uart_send_rtl8195bhp(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	return ret;
}

int32_t test_mode_uart_discon_cmd(int argc, char **argv)
{
	uint32_t start_addr;

	otu_disconnect = 1;

	if (argc >= 1) {
		start_addr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
		if (((uint32_t)start_addr >= INTERNAL_HS_RAM_START) && ((uint32_t)start_addr < INTERNAL_HS_RAM_END)) {
			uboot_frame_handle.ram_start_tbl = start_addr;
		}
	}

	if (argc >= 2) {
		start_addr = _strtoul((const char *)(argv[1]), (char **)NULL, 16);
		if (((uint32_t)start_addr >= INTERNAL_LS_RAM_START) && ((uint32_t)start_addr < INTERNAL_LS_RAM_END)) {
			uboot_frame_handle.ls_ram_start_tbl = start_addr;
			uart_boot_notify_ls_img_rdy();
		}
	}

	return 0;
}

void test_mode_img_download(void)
{
	uint8_t uart_idx;
	uint8_t uart_pin_sel;
//    uint8_t baud_rate_sel;
	uint32_t baud_rate;
	int32_t ret;
	int32_t cryp_ret;
	stdio_port_backup_t stdio_uart_bkup;
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const cmd_shell_func_stubs_t *pcmd_shell_stubs = symb_ns4s_stubs->pcmd_shell_stubs;
#if !CONFIG_FPGA
	uint32_t i;
#endif

#if !CONFIG_FPGA
	uart_idx = boot_uart_sel_map[BootDbgMsgCfgReg->bit.port_sel];
	uart_pin_sel = boot_uart_pin_sel_map[BootDbgMsgCfgReg->bit.pin_sel];
#else

#if 1
	uart_idx = boot_uart_sel_map[BootDbgMsgCfgReg->bit.port_sel];
	uart_pin_sel = boot_uart_pin_sel_map[BootDbgMsgCfgReg->bit.pin_sel];
#else
	uart_idx = 1;
	uart_pin_sel = 1;
#endif

#endif

//    baud_rate_sel = BootCfg2Reg->bit.cfg;
//    baud_rate = boot_uart_baud_sel_map[baud_rate_sel];
	if (uart_idx >= MAX_UART_PORT) {
		uart_idx = 1;
	}

	if ((uart_idx == 0) && (uart_pin_sel > UartPinSel3)) {
		uart_pin_sel = UartPinSel0;
	} else if (uart_pin_sel > UartPinSel2) {
		uart_pin_sel = UartPinSel0;
	}

	// Initial Cryptal engine HW
	cryp_ret = rtl_cryptoEngine_init_s(&_crypto_engine);
	if (cryp_ret != SUCCESS) {
		DBG_MISC_ERR("img_download: Crypto init failed(%d)!\r\n", cryp_ret);
	}
	_crypto_engine.arch_clean_dcache_by_size = (void (*)(uint32_t, int32_t))dcache_clean_by_addr_rtl8195bhp;
	_crypto_engine.arch_invalidate_dcache_by_size = (void (*)(uint32_t, int32_t))dcache_invalidate_by_addr_rtl8195bhp;

	baud_rate = 115200;
	// default enable UART with baud rate 115200, no parity, no flow control to wait UART Cfg command
	dbg_printf("Download Image over UART%u_S%u\r\n", uart_idx, uart_pin_sel);
	ret = otu_uart_port_open(uart_idx, uart_pin_sel, baud_rate, 0, 0, &stdio_uart_bkup);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU UART port open failed!\r\n");
		if (cryp_ret == SUCCESS) {
			cryp_ret = rtl_cryptoEngine_deinit_s(&_crypto_engine);
		}
		return;
	}

	// Initial NS region stdio port
	symb_ns4s_stubs->stdio_port_init_ns((void *)&otu_uart, (stdio_putc_t)&hal_uart_wputc_rtl8195bhp, \
										(stdio_getc_t)&hal_uart_rgetc_rtl8195bhp);
	// Initial command shell
	((void(*)(void))symb_ns4s_stubs->shell_cmd_task_init)();
	pcmd_shell_stubs->shell_set_cmd_list(pcmd_shell_stubs->shell_cmd_hdl, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	pcmd_shell_stubs->shell_unregister_all(pcmd_shell_stubs->shell_cmd_hdl);
	// register command for FW download over UART
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_ping_cmd, "ping", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_uart_cfg_cmd, "ucfg", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_img_download_cmd, "fwd", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_img1_update_cmd, "otu1", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_img2_update_cmd, "otu2", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_img2x_update_cmd, "otu2x", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_hash_req_cmd, "hashq", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_uart_discon_cmd, "disc", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_flash_chip_erase_cmd, "ceras", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_flash_sector_erase_cmd, "seras", NULL);

	// read priviate key from super secure eFuse
#if !CONFIG_FPGA
	// Load RAM image function only available when the secure efuse is empty (means secure boot is not enabled)
	hal_susec_key_get_rtl8195bhp(SYSON->hs_efuse_ctrl1, ss_priv_key, 7);
	for (i = 0; i < PRIV_KEY_SIZE; i++) {
		if (ss_priv_key[i] != 0xFF) {
			break;
		}
	}

	if (i == PRIV_KEY_SIZE) {
		// no secure efuse key, so load image to RAM function is available
		pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_uart_boot_cmd, "uboot", NULL);
		pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_ram_img_download_cmd, "fwdram", NULL);
	}
#else
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_uart_boot_cmd, "uboot", NULL);
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_program_t)test_mode_ram_img_download_cmd, "fwdram", NULL);
#endif
	hal_misc_wdt_disable_rtl8195bhp();

	otu_disconnect = 0;
	while (1) {
		ret = pcmd_shell_stubs->shell_parse_one_cmd(pcmd_shell_stubs->shell_cmd_hdl);
		if (otu_disconnect != 0) {
			break;  // break the while loop
		}
	}
	otu_uart_port_close(&stdio_uart_bkup);

	// De-Initial Cryptal engine HW
	if (cryp_ret == SUCCESS) {
		cryp_ret = rtl_cryptoEngine_deinit_s(&_crypto_engine);
		if (cryp_ret != SUCCESS) {
			DBG_MISC_WARN("img_download: Crypto deinit failed(%d)!\r\n", cryp_ret);
		}
	}

	hal_misc_wdt_refresh_rtl8195bhp();
	hal_misc_wdt_enable_rtl8195bhp();
}
#endif  // end of "#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))"

