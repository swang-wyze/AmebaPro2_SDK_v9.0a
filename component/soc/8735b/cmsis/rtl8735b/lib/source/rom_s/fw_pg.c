/**************************************************************************//**
 * @file     fw_pg.c
 * @brief    Implement the FW image program to flash over the UART XModem.
 *
 * @version  V1.00
 * @date     2021-07-28
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
#include "xmport_uart.h"
#include "xmodem.h"
#include "strproc.h"
#include "memory.h"
#include "rtl8735b_symbns4s.h"
#include "otp_boot_cfg.h"
#include "fw_uart_boot.h"
#include "shell.h"

#define PG_HASH_RESULT_SIZE                     (64)
#define SECTION_PG_SECURE_BSS                   SECTION(".rom.pg.secure.bss")

const char otu_ok[] = "OK";
const char otu_err[] = "ER";
const char otu_hash_ret[] = "hashs ";
const char otu_ping[] = "ping";

typedef struct otu_xm_frame_handler_s {
	uint8_t got_1st_frame;  // is the 1st frame of the XModem transfer
	uint8_t update_partition;    // the partition to be update
	uint32_t flash_start;   // the flash start address to be write
	uint32_t flash_offset;   // the flash offset address to be write
} otu_xm_frame_handler_t, *potu_xm_frame_handler_t;

enum {
	PG_HASH_ALG_SEL_SHA256  = 0,
	PG_HASH_ALG_SEL_SHA224  = 1,
	PG_HASH_ALG_SEL_SHA1    = 2,
	PG_HASH_ALG_SEL_MD5     = 3,
	PG_HASH_ALG_SEL_SHA384  = 4,
	PG_HASH_ALG_SEL_SHA512  = 5
};

SECTION_ROM_TEMP_BSS hal_uart_adapter_t otu_uart;
SECTION_ROM_TEMP_BSS volatile uint8_t otu_disconnect;
SECTION_ROM_TEMP_BSS uint8_t boot_imgft;
SECTION_ROM_TEMP_BSS xmodem_uart_port_handler_t xm_uart_port;
SECTION_ROM_TEMP_BSS uint8_t xm_rx_buf[XM_BUF_SIZE] __ALIGNED(32);    // the buffer for UART RX FIFO
SECTION_ROM_TEMP_BSS XMODEM_CTRL xmodem_ctrl;
SECTION_ROM_TEMP_BSS uint8_t xmodem_frame_buf[XM_BUFFER_SIZE] __ALIGNED(32);     // the buffer for XModem RX Frame buffer
SECTION_ROM_TEMP_BSS otu_xm_frame_handler_t frame_handler;
SECTION_ROM_TEMP_BSS uint8_t flah_read_temp_buf[XM_BUFFER_SIZE];

#define SNAND_PG_PINGPONG_BUF_SIZE (2112) /* 2048+64 */
SECTION_ROM_TEMP_BSS uint8_t mSnandPageBufA[SNAND_PG_PINGPONG_BUF_SIZE];
SECTION_ROM_TEMP_BSS uint8_t mSnandPageBufB[SNAND_PG_PINGPONG_BUF_SIZE];
SECTION_ROM_TEMP_BSS uint8_t *mpSnandPageBufAHead;
SECTION_ROM_TEMP_BSS uint8_t *mpSnandPageBufBHead;
SECTION_ROM_TEMP_BSS uint16_t mBufARemainSz;
SECTION_ROM_TEMP_BSS uint16_t mBufBRemainSz;

SECTION_ROM_TEMP_BSS static uint32_t flash_pg_offset;     // to record the latest PG command flash offset

// Need to be the secure bss declaration
SECTION_PG_SECURE_BSS hal_crypto_adapter_t _crypto_engine;   // temp used SECTION_ROM_TEMP_BSS
SECTION_PG_SECURE_BSS uint8_t temp_hash_out_buf[PG_HASH_RESULT_SIZE] __ALIGNED(32); // temp used SECTION_ROM_TEMP_BSS

typedef struct {
	int (*pg_crypto_auth_init)(hal_crypto_adapter_t *pcrypto_adapter, const u32 auth_type,
							   const u8 *pAuthKey, const u32 lenAuthKey);
	int (*pg_crypto_auth_update)(hal_crypto_adapter_t *pcrypto_adapter, const u32 auth_type,
								 const u8 *message, const u32 msglen);
	int (*pg_crypto_auth_final)(hal_crypto_adapter_t *pcrypto_adapter, const u32 auth_type, u8 *pDigest);
} pg_crypto_auth_t;

extern hal_uart_adapter_t stdio_uart;
extern shell_command_t shell_cmd_hdl_rom;
extern shell_command_entry_t shell_rom_cmd_list[];
extern uboot_xm_frame_handler_t uboot_frame_handle;
extern hal_spic_adaptor_t _hal_spic_adaptor;
extern hal_snafc_adaptor_t _hal_snafc_adaptor;
extern const uint32_t rom_log_uart_rx_pins[2][4];
extern const uint32_t rom_log_uart_tx_pins[2][4];

#define INTERNAL_TM_RAM_START          0x20100000UL
#define INTERNAL_TM_RAM_END            0x2017FFFFUL

extern hal_status_t fw_spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, u8 io_pin_sel);
extern hal_status_t fw_spic_deinit(phal_spic_adaptor_t phal_spic_adaptor);
extern int img_sel_op_idx(void *p_tbl_info, const uint8_t img_obj, const uint8_t img_sel_op);
#if 1 /** for SNAND */
extern hal_status_t fw_snafc_init(hal_snafc_adaptor_t  *pSnafcAdaptor, u8 spic_bit_mode, u8 io_pin_sel);
extern hal_status_t fw_snafc_deinit(hal_snafc_adaptor_t  *pSnafcAdaptor);
#endif

extern int32_t uart_boot_load_imag(hal_uart_adapter_t *potu_uart);

int32_t test_mode_ping_cmd(int argc, char **argv);
int32_t test_mode_uart_cfg_cmd(int argc, char **argv);
int32_t test_mode_uart_discon_cmd(int argc, char **argv);
int32_t test_mode_ram_img_download_cmd(int argc, char **argv);
int32_t test_mode_uart_boot_cmd(int argc, char **argv);
int32_t test_mode_img_download_cmd(int argc, char **argv);
int32_t test_mode_flash_chip_erase_cmd(int argc, char **argv);
int32_t test_mode_flash_sector_erase_cmd(int argc, char **argv);
int32_t test_mode_flash_write_ext_cmd(int argc, char **argv);
int32_t test_mode_hash_req_cmd(int argc, char **argv);
int32_t test_mode_set_imgft_cmd(int argc, char **argv);
int32_t test_mode_img_update_cmd(int argc, char **argv);
int32_t test_mode_flash_basic_op_cmd(int argc, char **argv);
int32_t test_mode_flash_nand_read_cmd(int argc, char **argv);
int32_t test_mode_flash_block_erase_cmd(int argc, char **argv);
int32_t test_mode_flash_nand_write_cmd(int argc, char **argv);
int32_t test_mode_img_download_snand_cmd(int argc, char **argv);

const shell_command_entry_t pg_cmd_info[SHELL_ROM_CMD_LIST_SIZE] = {
	{
		"PG Tool", (const char *)"ping", (shell_program_t)test_mode_ping_cmd,
		(const char *)"\tping\r\n"
	},
	{
		"PG Tool", (const char *)"ucfg", (shell_program_t)test_mode_uart_cfg_cmd,
		(const char *)"\tucfg\r\n"
	},
	{
		"PG Tool", (const char *)"disc", (shell_program_t)test_mode_uart_discon_cmd,
		(const char *)"\tdisc\r\n"
	},
	{
		"PG Tool", (const char *)"fwdram", (shell_program_t)test_mode_ram_img_download_cmd,
		(const char *)"\tfwdram\r\n"
	},
	{
		"PG Tool", (const char *)"uboot", (shell_program_t)test_mode_uart_boot_cmd,
		(const char *)"\tuboot\r\n"
	},
	{
		"PG Tool", (const char *)"fwd", (shell_program_t)test_mode_img_download_cmd,
		(const char *)"\tfwd\r\n"
	},
	{
		"PG Tool", (const char *)"ceras", (shell_program_t)test_mode_flash_chip_erase_cmd,
		(const char *)"\tceras\r\n"
	},
	{
		"PG Tool", (const char *)"seras", (shell_program_t)test_mode_flash_sector_erase_cmd,
		(const char *)"\tseras\r\n"
	},
	{
		"PG Tool", (const char *)"flashwr", (shell_program_t)test_mode_flash_write_ext_cmd,
		(const char *)"\tflashwr\r\n"
	},
	{
		"PG Tool", (const char *)"hashq", (shell_program_t)test_mode_hash_req_cmd,
		(const char *)"\thash check flash mem\r\n"
	},
	{
		"PG Tool", (const char *)"imgft", (shell_program_t)test_mode_set_imgft_cmd,
		(const char *)"\tset imgft\r\n"
	},
	{
		"PG Tool", (const char *)"otu_img", (shell_program_t)test_mode_img_update_cmd,
		(const char *)"\tota update img\r\n"
	},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)"snb", (shell_program_t)test_mode_flash_basic_op_cmd, (const char *)"\tsnand basic op\r\n"},
	{"PG Tool", (const char *)"snr", (shell_program_t)test_mode_flash_nand_read_cmd, (const char *)"\tsnand pageread\r\n"},
	{"PG Tool", (const char *)"sne", (shell_program_t)test_mode_flash_block_erase_cmd, (const char *)"\tsnan blockerase\r\n"},
	{"PG Tool", (const char *)"snw", (shell_program_t)test_mode_flash_nand_write_cmd, (const char *)"\tsnand pagewrite\r\n"},
	{"PG Tool", (const char *)"sfwd", (shell_program_t)test_mode_img_download_snand_cmd, (const char *)"\tsnand flash.bin\r\n"},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"PG Tool", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL}      // end of table
};

int32_t otu_rx_frame_ram_write(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	uint32_t retry;
	uint32_t retry_lim = 3;
	uint8_t *pdest;
	uint32_t end_addr;

	pdest = (uint8_t *)(frame_handler.flash_start + frame_handler.flash_offset);
	end_addr = (uint32_t)pdest + frame_size;

	if ((frame_handler.flash_start >> 20) == (INTERNAL_TM_RAM_START >> 20)) {
		// Load Image to Bus RAM
		if (((uint32_t)pdest < INTERNAL_TM_RAM_START) || ((uint32_t)pdest > INTERNAL_TM_RAM_END)) {
			return -1;
		}

		if ((end_addr < INTERNAL_TM_RAM_START) || (end_addr > INTERNAL_TM_RAM_END)) {
			return -1;
		}
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
		//DBG_MISC_ERR ("otu_rx_frame_ram_write: RAM img load err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
}

hal_status_t otu_uart_port_open(uint32_t tx_pin, uint32_t rx_pin,
								uint32_t baud_rate, uint32_t parity, uint32_t flow_ctrl,
								stdio_port_backup_t *stdio_uart_bkup)
{
	hal_status_t ret;
	uint8_t uart_idx;
	int32_t uart_irqn;

	uart_idx = hal_rtl_uart_rx_pin_to_idx(rx_pin);

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
			hal_rtl_uart_deinit(&stdio_uart);
			hal_rtl_pinmux_unregister(stdio_uart.tx_pin, (PID_UART0 + stdio_uart.uart_idx));
			hal_rtl_pinmux_unregister(stdio_uart.rx_pin, (PID_UART0 + stdio_uart.uart_idx));
		}
	}

	hal_rtl_pinmux_register(tx_pin, (PID_UART0 + uart_idx));
	hal_rtl_pinmux_register(rx_pin, (PID_UART0 + uart_idx));
	hal_rtl_gpio_pull_ctrl(rx_pin, Pin_PullUp);

	ret = hal_rtl_uart_init(&otu_uart, tx_pin, rx_pin, NULL);
	ret |= hal_rtl_uart_set_baudrate(&otu_uart, baud_rate);
	ret |= hal_rtl_uart_set_format(&otu_uart, 8, parity, 1);

	// need select other UART irq
	switch (uart_idx) {
	case Uart0:
		uart_irqn = UART0_IRQn;
		break;
	case Uart1:
		uart_irqn = UART1_IRQn;
		break;
	case Uart2:
		uart_irqn = UART2_IRQn;
		break;
	case Uart3:
		uart_irqn = UART3_IRQn;
		break;
	default:
		uart_irqn = UART1_IRQn;
		break;
	}
	hal_rtl_irq_enable(uart_irqn);

	if (flow_ctrl == 0) {
		flow_ctrl = UartFlowCtlNone;
	} else {
		flow_ctrl = UartFlowCtlRTSCTS;
	}
	ret |= hal_rtl_uart_set_flow_control(&otu_uart, flow_ctrl);
	// TODO: pinmux for CTS/RTS
	if ((uart_idx == stdio_uart.uart_idx) && (ret != HAL_OK)) {
		ConfigDebugErr = stdio_uart_bkup->cfg_err;
		ConfigDebugInfo = stdio_uart_bkup->cfg_info;
		ConfigDebugWarn = stdio_uart_bkup->cfg_warn;
		DBG_MISC_ERR("OTU UART(Idx=%u[tx=%u, rx=%u]) Open Err\r\n", uart_idx, tx_pin, rx_pin);
	}

	return ret;
}

hal_status_t otu_uart_port_close(stdio_port_backup_t *stdio_uart_bkup)
{
	uint32_t uart_idx;
	hal_status_t ret = HAL_OK;

	uart_idx = otu_uart.uart_idx;
	hal_rtl_uart_deinit(&otu_uart);
	hal_rtl_pinmux_unregister(otu_uart.tx_pin, (PID_UART0 + uart_idx));
	hal_rtl_pinmux_unregister(otu_uart.rx_pin, (PID_UART0 + uart_idx));

	if (uart_idx == stdio_uart.uart_idx) {
		if (stdio_uart_bkup->is_inited) {
			ret = hal_rtl_pinmux_register(stdio_uart.tx_pin, (PID_UART0 + uart_idx));
			ret |= hal_rtl_pinmux_register(stdio_uart.rx_pin, (PID_UART0 + uart_idx));
			hal_rtl_gpio_pull_ctrl(stdio_uart.rx_pin, Pin_PullUp);
			ret |= hal_rtl_uart_init(&stdio_uart, stdio_uart.tx_pin, stdio_uart.rx_pin, NULL);
			ret |= hal_rtl_uart_set_baudrate(&stdio_uart, stdio_uart_bkup->baud_rate);
			ret |= hal_rtl_uart_set_format(&stdio_uart, 8, stdio_uart_bkup->parity, 1);
			ret |= hal_rtl_uart_set_flow_control(&stdio_uart, stdio_uart_bkup->flow_ctrl);
		}

		ConfigDebugErr = stdio_uart_bkup->cfg_err;
		ConfigDebugInfo = stdio_uart_bkup->cfg_info;
		ConfigDebugWarn = stdio_uart_bkup->cfg_warn;
	}
	return ret;
}


int32_t test_mode_ping_cmd(int argc, char **argv)
{
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ping, _strlen(otu_ping), 100);

	return 0;
}

int32_t test_mode_uart_cfg_cmd(int argc, char **argv)
{
	uint32_t baud_rate = 1000000;
	uint8_t parity = UartParityNone;
	uint8_t flow_ctrl = UartFlowCtlNone;
	hal_status_t ret;
	int32_t uart_irqn;

	if (argc >= 1) {
		baud_rate = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
		if (baud_rate == 0) {
			baud_rate = 1000000;
		}
	}

	if (argc >= 2) {
		parity = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		flow_ctrl = _strtoul((const char *)(argv[2]), (char **)NULL, 16);
	}

	hal_rtl_uart_wait_tx_done(&otu_uart, 100);

	ret = hal_rtl_uart_set_baudrate(&otu_uart, baud_rate);
	ret |= hal_rtl_uart_set_format(&otu_uart, 8, parity, 1);

	// Set flow control & flow control pins
	if (flow_ctrl == 0) {
		flow_ctrl = UartFlowCtlNone;
	} else {
		uint32_t rts_pin;
		uint32_t cts_pin;
		uint8_t uart_idx = otu_uart.uart_idx;

		flow_ctrl = UartFlowCtlRTSCTS;
		switch (uart_idx) {
		case Uart0:
			rts_pin = (PIN_NC & 0xFF);
			cts_pin = (PIN_NC & 0xFF);
			break;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		case Uart1:
			rts_pin = PIN_F3;
			cts_pin = PIN_F2;
			break;
		case Uart2:
			rts_pin = PIN_E4;
			cts_pin = PIN_E3;
			break;
		case Uart3:
			rts_pin = PIN_E7;
			cts_pin = PIN_E8;
			break;
#else
		case Uart1:
			rts_pin = PIN_F2;
			cts_pin = PIN_F1;
			break;
		case Uart2:
			rts_pin = PIN_D18;
			cts_pin = PIN_D17;
			break;
		case Uart3:
			rts_pin = PIN_E3;
			cts_pin = PIN_E4;
			break;
#endif
		default:
			ret |= HAL_NOT_READY;
			break;
		}
		if (ret == HAL_OK) {
			ret = hal_rtl_pinmux_register(rts_pin, PID_UART0 + otu_uart.uart_idx);
			ret |= hal_rtl_pinmux_register(cts_pin, PID_UART0 + otu_uart.uart_idx);
		}
	}
	ret |= hal_rtl_uart_set_flow_control(&otu_uart, flow_ctrl);

	// need select other UART irq
	switch (otu_uart.uart_idx) {
	case Uart0:
		uart_irqn = UART0_IRQn;
		break;
	case Uart1:
		uart_irqn = UART1_IRQn;
		break;
	case Uart2:
		uart_irqn = UART2_IRQn;
		break;
	case Uart3:
		uart_irqn = UART3_IRQn;
		break;
	default:
		uart_irqn = UART1_IRQn;
		break;
	}
	hal_rtl_irq_enable(uart_irqn);
	hal_delay_ms(500);      // give some time for the host side to change its baud rate

	if (ret == HAL_OK) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	} else {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	}

	return ret;
}

int32_t test_mode_uart_discon_cmd(int argc, char **argv)
{
	uint32_t start_addr;

	otu_disconnect = 1;
	if (argc >= 1) {
		start_addr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
#if 0
		// Basic check start_addr is valid or not
		if ((((uint32_t)start_addr >= DTCM_RAM_START) && ((uint32_t)start_addr < DTCM_RAM_END)) ||
			(((uint32_t)start_addr >= PS_RAM_START) && ((uint32_t)start_addr < PS_RAM_END))) {
			uboot_frame_handle.ram_start_tbl = start_addr;
		}
#endif
		uboot_frame_handle.ram_start_tbl = start_addr;
	}

	return 0;
}

int32_t otu_ram_fw_download(hal_uart_adapter_t *potu_uart, uint32_t start_addr)
{
//    hal_status_t ret;
	uint32_t fw_dnload_size;

	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_start = start_addr;
	frame_handler.flash_offset = 0;
	//dbg_printf("Start XModem..\r\n");
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_ram_write);
	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (1024 * 1024)); // maximum 1M
	//dbg_printf("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);

	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);
	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	return fw_dnload_size;
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

int32_t test_mode_set_imgft_cmd(int argc, char **argv)
{
	int32_t ret;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	uint8_t imgft = LD_IMF_FT_NTLV;
#else
	uint8_t imgft = LD_IMF_FT_TLV;
#endif
	if (argc >= 1) {
		imgft = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	imgft = LD_IMF_FT_NTLV;
#else
	if (LD_IMF_FT_TLV == imgft) {
		imgft = LD_IMF_FT_TLV;
	} else {
		imgft = LD_IMF_FT_NTLV;
	}
#endif
	boot_imgft = imgft;
	ret = SUCCESS;
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	return ret;
}

int32_t test_mode_uart_boot_cmd(int argc, char **argv)
{
	int32_t ret;

	ret = uart_boot_load_imag(&otu_uart);
	if ((xmodem_ctrl.result != 0) || (ret <= 0)) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}
	return ret;
}

int32_t otu_rx_frame_direct_write(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	// Todo: Flash controller in secure domain
	uint32_t retry = 0;
	uint32_t retry_lim = 3;

	// start to program Flash
	if ((frame_handler.flash_offset & 0xFFF) == 0) {
		//dbg_printf("Sector Erase 0x%x\r\n", frame_handler.flash_offset);
		// 1. sector boundary, erase flash sector first
		// Todo: Flash controller in secure domain

		flash_rtl_sector_erase(&_hal_spic_adaptor, frame_handler.flash_offset);
	}
	//dbg_printf("Burst write 0x%x\r\n", frame_handler.flash_offset);
	/* Write data comes from PC into flash */
#if CONFIG_FPGA
	char *pTmp;
	uint32_t tmpOffset;
	uint32_t remain;

	/* multi-shot (64 byte each trigger) */

	pTmp = ptr;
	tmpOffset = frame_handler.flash_offset;
	remain = frame_size;
	while (remain >= 60) {
		flash_rtl_burst_write(&_hal_spic_adaptor, 60, tmpOffset, (u8 *)pTmp);
		tmpOffset += 60;
		pTmp += 60;
		remain -= 60;
	}
	if (remain) {
		flash_rtl_burst_write(&_hal_spic_adaptor, remain, tmpOffset, (u8 *)pTmp);
	}
#else
	/* 1 shot */
	for (retry = 0; retry < retry_lim; retry++) {
		flash_rtl_burst_write(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, (u8 *)ptr);
		flash_rtl_burst_read(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, flah_read_temp_buf);
		if (!_memcmp((void *)ptr, (void *) flah_read_temp_buf, frame_size)) {
			break;
		} else {
			DBG_MISC_WARN("otu_rx_frame_handler: flash write read-back incorrect\r\n");
		}
	}
#endif

	if (retry >= retry_lim) {
		DBG_MISC_ERR("otu_rx_frame_direct_write: Flash write err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
}



int32_t otu_fw_download(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset)
{
	hal_status_t ret;
	uint32_t fw_dnload_size;
	uint8_t flash_io;
	uint8_t flash_pin;
	// Todo: Flash controller in secure domain

	flash_io = (flash_sel >> 8) & 0xFF;
	flash_pin = flash_sel & 0xFF;
	DBG_MISC_INFO("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Initial Flash controller
	DBG_MISC_INFO("Init Flash IOMod=%u PinSel=%u...\r\n", flash_io, flash_pin);
	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU Flash(BitMod=%u Sel=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_offset = flash_offset;  // it must 4K aligned
	DBG_MISC_INFO("Start XModem..\r\n");
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_direct_write);

	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (16 * 1024 * 1024)); // maximum 16-MB

	DBG_MISC_INFO("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);


	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	// Todo: Flash controller in secure domain
	flash_rtl_return_spi(&_hal_spic_adaptor);

	return fw_dnload_size;
}

int32_t test_mode_img_download_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0; //Confirm flash pin select 0
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

int32_t test_mode_img_update_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0; //Confirm flash pin select 0
	uint32_t flash_offset = 0;
	int32_t  ret;
	int32_t  update_fw_idx = -1;
	uint8_t  img_obj = OTA_UPDATE_IMG_FW;
	uint8_t  img1_rec_idx, img2_rec_idx;
	partition_tbl_t *ppartition_tbl = NULL;
	certi_tbl_t *pcerti_rec_tbl = NULL;
	uint8_t record_idx = 0;

	if (argc >= 1) {
		flash_io = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		flash_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}

	if (argc >= 3) {
		img_obj = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	if (argc >= 4) {
		update_fw_idx = _strtoul((const char *)(argv[3]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU Flash(BitMod=%u Sel=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	if (OTA_UPDATE_IMG_KEYCERTI == img_obj) {  // For key certificate update, ref. key certificate table
		// get key certificate table
		pcerti_rec_tbl = fw_load_certi_tbl_direct_f();

		// get key certificate 1,2 record idx
		img1_rec_idx = pcerti_rec_tbl->certi1_idx;
		img2_rec_idx = pcerti_rec_tbl->certi2_idx;
	} else {    // For boot/ fw img update, ref part_tbl
		// get partition table
		ppartition_tbl = fw_load_partition_tbl_direct_f();
		if (OTA_UPDATE_IMG_BOOT == img_obj) {
			// get boot img 1,2 record idx
			img1_rec_idx = ppartition_tbl->fst.bl_p_idx;
			img2_rec_idx = ppartition_tbl->fst.bl_s_idx;
		} else {
			// get fw 1,2 record idx
			img1_rec_idx = ppartition_tbl->fst.fw1_idx;
			img2_rec_idx = ppartition_tbl->fst.fw2_idx;
		}
	}
	DBG_MISC_INFO("test_mode_img_update_cmd update fw%d\r\n", update_fw_idx);
	if (OTA_UPDATE_IMG_NO_SPECI_IDX == update_fw_idx) {
		// update into the valid oldest version((record valid+type_id), version, timestamp)

		// chk valid, version, timestamp, then get update flash offset
		if (OTA_UPDATE_IMG_KEYCERTI == img_obj) {
			update_fw_idx = img_sel_op_idx(pcerti_rec_tbl, img_obj, IMG_SEL_UPDATE);
			if (((-1) != update_fw_idx) && (update_fw_idx < MAX_CERTI_TBL_RECORD)) {
				flash_offset = pcerti_rec_tbl->key_cer_tbl_rec[update_fw_idx].start_addr;
			}
		} else {
			update_fw_idx = img_sel_op_idx(ppartition_tbl, img_obj, IMG_SEL_UPDATE);
			if (((-1) != update_fw_idx) && (update_fw_idx < PARTITION_RECORD_MAX)) {
				flash_offset = ppartition_tbl->partition_record[update_fw_idx].start_addr;
			}
		}
	} else {
		if (OTA_UPDATE_IMG_KEYCERTI == img_obj) {
			if (OTA_UPDATE_IMG_SPECI_IDX1 == update_fw_idx) {
				flash_offset = pcerti_rec_tbl->key_cer_tbl_rec[img1_rec_idx].start_addr;
			} else {
				flash_offset = pcerti_rec_tbl->key_cer_tbl_rec[img2_rec_idx].start_addr;
			}
		} else {
			if (OTA_UPDATE_IMG_SPECI_IDX1 == update_fw_idx) {
				flash_offset = ppartition_tbl->partition_record[img1_rec_idx].start_addr;
			} else {
				flash_offset = ppartition_tbl->partition_record[img2_rec_idx].start_addr;
			}
		}
	}

	if (OTA_UPDATE_IMG_KEYCERTI == img_obj) {  // For key certificate update, ref. key certificate table
		_memset((void *)pcerti_rec_tbl, 0, sizeof(certi_tbl_t));
	} else {    // For boot/ fw img update, ref part_tbl
		_memset((void *)ppartition_tbl, 0, sizeof(partition_tbl_t));
	}
	//rom_sboot_export_bss_clean_up();
	flash_pg_offset = flash_offset;
	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;
}

int32_t otu_fw_download_cmd(int argc, char **argv)
{
	uint32_t tx_pin = STDIO_UART_TX_PIN;
	uint32_t rx_pin = STDIO_UART_RX_PIN;
	uint8_t parity = 0;
	uint8_t flow_ctrl = 0;
	uint8_t uart_idx = 1;
	uint32_t baud = 1000000;
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	uint32_t flash_offset = 0;
	int32_t ret;
	stdio_port_backup_t stdio_uart_bkup;

	if (argc >= 1) {
		tx_pin = _strtoul((const char *)(argv[0]), (char **)NULL, 10);
	}

	if (argc >= 2) {
		rx_pin = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
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
	uart_idx = hal_rtl_uart_rx_pin_to_idx(rx_pin);
	dbg_printf("Image Downlaod: UART%u[tx_pin=%u,rx_pin=%u], baud=%lu, parity=%u, flow_ctrl=%u Flash[IO%u,Sel%u], Offset=0x%x\r\n",
			   uart_idx, PIN_NAME_2_PIN(tx_pin), PIN_NAME_2_PIN(rx_pin), baud, parity, flow_ctrl, flash_io, flash_pin, flash_offset);

	otu_uart_port_open(tx_pin, rx_pin, baud, parity, flow_ctrl, &stdio_uart_bkup);

	ret = otu_fw_download(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);

	otu_uart_port_close(&stdio_uart_bkup);

	return ret;
}

int32_t test_mode_flash_chip_erase_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode; /* 0 */
	uint32_t flash_pin = BootFlash_DualIOS0; /* 0 */
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
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	flash_rtl_chip_erase(&_hal_spic_adaptor);

	fw_spic_deinit(&_hal_spic_adaptor);
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	return ret;
} /* test_mode_flash_chip_erase_cmd */

int32_t test_mode_flash_sector_erase_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	uint32_t flash_offset = 0;
	uint32_t len = 0;
	uint32_t erase_offset;
	int32_t ret;

	if (argc < 2) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
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
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	erase_offset = flash_offset & 0xFFFFF000;
	while (erase_offset < (flash_offset + len)) {
		flash_rtl_sector_erase(&_hal_spic_adaptor, erase_offset);
		erase_offset += 0x1000;
	}

	flash_rtl_return_spi(&_hal_spic_adaptor);
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);

	return ret;
} /* test_mode_flash_sector_erase_cmd */


int32_t otu_rx_frame_direct_write_ext(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	// Todo: Flash controller in secure domain
	uint32_t retry = 0;
	uint32_t retry_lim = 3;

	// start to program Flash
	//dbg_printf("Burst write 0x%x\r\n", frame_handler.flash_offset);
	/* Write data comes from PC into flash */
#if CONFIG_FPGA
	/* multi-shot (64 byte each trigger) */
	uint32_t remain;
	uint32_t tmpOffset;
	char *pTmp;

	pTmp = ptr;
	tmpOffset = frame_handler.flash_offset;
	remain = frame_size;
	while (remain >= 60) {
		flash_rtl_burst_write(&_hal_spic_adaptor, 60, tmpOffset, (u8 *)pTmp);
		tmpOffset += 60;
		pTmp += 60;
		remain -= 60;
	}
	if (remain) {
		flash_rtl_burst_write(&_hal_spic_adaptor, remain, tmpOffset, (u8 *)pTmp);
	}
#else
	/* 1 shot */
	for (retry = 0; retry < retry_lim; retry++) {
		flash_rtl_burst_write(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, (u8 *)ptr);
		flash_rtl_burst_read(&_hal_spic_adaptor, frame_size, frame_handler.flash_offset, flah_read_temp_buf);
		if (!_memcmp((void *)ptr, (void *) flah_read_temp_buf, frame_size)) {
			break;
		} else {
			DBG_MISC_WARN("otu_rx_frame_handler: flash write read-back incorrect\r\n");
		}
	}
#endif

	if (retry >= retry_lim) {
		DBG_MISC_ERR("otu_rx_frame_direct_write_ext: Flash write err\r\n");
		return -1;
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
} /* otu_rx_frame_direct_write_ext */


int32_t otu_fw_download_ext(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset, uint32_t buf_length)
{
	hal_status_t ret;
	uint32_t fw_dnload_size;
	uint8_t flash_io;
	uint8_t flash_pin;
	// Todo: Flash controller in secure domain

	flash_io = (flash_sel >> 8) & 0xFF;
	flash_pin = flash_sel & 0xFF;
	DBG_MISC_INFO("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Initial Flash controller
	DBG_MISC_INFO("Init Flash IOMod=%u PinSel=%u...\r\n", flash_io, flash_pin);
	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU Flash(BitMod=%u Sel=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_offset = flash_offset;  // it must 4K aligned
	DBG_MISC_INFO("Start XModem..\r\n");
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_direct_write_ext);

	if (buf_length > (16 * 1024 * 1024)) {
		buf_length = (16 * 1024 * 1024);
	}

	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, buf_length);   // maximum 16-MB

	DBG_MISC_INFO("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);


	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	// Todo: Flash controller in secure domain
	flash_rtl_return_spi(&_hal_spic_adaptor);

	return fw_dnload_size;
}


int32_t test_mode_flash_write_ext_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	uint32_t flash_offset = 0;
	uint32_t len = 0;
	int32_t ret;

	if (argc < 2) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
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


	flash_pg_offset = flash_offset;

	ret = otu_fw_download_ext(&otu_uart, (flash_io << 8) | flash_pin, flash_offset, len);
	return ret;

}/* test_mode_flash_write_ext_cmd */

int32_t test_mode_hash_req_cmd(int argc, char **argv)
{
	uint32_t flash_offset = flash_pg_offset;
	uint32_t flash_mem = SPI_FLASH_BASE;
	int32_t size;
	int32_t hash_size;
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	uint32_t hash_alg_sel = PG_HASH_ALG_SEL_SHA256, crypto_hash_type;
	pg_crypto_auth_t pg_auth_f;
	hal_status_t ret;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_hash_req_cmd: no size\r\n");
		return 0;
	}
	size = _strtoul((const char *)(argv[0]), (char **)NULL, 10);

	// Initial Cryptal engine HW
	hal_rtl_crypto_engine_init(&_crypto_engine);
	_crypto_engine.arch_clean_dcache_by_size = (void (*)(uint32_t, int32_t))rtl_dcache_clean_by_addr;
	_crypto_engine.arch_invalidate_dcache_by_size = (void (*)(uint32_t, int32_t))rtl_dcache_invalidate_by_addr;
	_memset(&temp_hash_out_buf[0], 0x0, PG_HASH_RESULT_SIZE);

	if (argc >= 2) {
		flash_offset = _strtoul((const char *)(argv[1]), (char **)NULL, 16);
	}

	if (argc >= 3) {
		hash_alg_sel = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}

	if (argc >= 4) {
		flash_io = _strtoul((const char *)(argv[3]), (char **)NULL, 10);
	}

	if (argc >= 5) {
		flash_pin = _strtoul((const char *)(argv[4]), (char **)NULL, 10);
	}

	ret = fw_spic_init(&_hal_spic_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("test_mode_hash_req_cmd SPIC(io=%u pin=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	switch (hash_alg_sel) {
	case PG_HASH_ALG_SEL_SHA256:
		crypto_hash_type = AUTH_TYPE_SHA2_256_ALL;
		break;
	case PG_HASH_ALG_SEL_SHA224:
		crypto_hash_type = AUTH_TYPE_SHA2_224_ALL;
		break;
	case PG_HASH_ALG_SEL_SHA1:
		crypto_hash_type = AUTH_TYPE_SHA1;
		break;
	case PG_HASH_ALG_SEL_MD5:
		crypto_hash_type = AUTH_TYPE_MD5;
		break;
	case PG_HASH_ALG_SEL_SHA384:
		crypto_hash_type = AUTH_TYPE_SHA2_384_ALL;
		break;
	case PG_HASH_ALG_SEL_SHA512:
		crypto_hash_type = AUTH_TYPE_SHA2_512_ALL;
		break;
	default:
		// Using SHA256
		crypto_hash_type = AUTH_TYPE_SHA2_256_ALL;
		break;
	}
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if ((hash_alg_sel == PG_HASH_ALG_SEL_SHA384) || (hash_alg_sel == PG_HASH_ALG_SEL_SHA512)) {
		pg_auth_f.pg_crypto_auth_init = &hal_rtl_crypto_auth_init_SHA512;
		pg_auth_f.pg_crypto_auth_update = &hal_rtl_crypto_auth_update_SHA512;
		pg_auth_f.pg_crypto_auth_final = &hal_rtl_crypto_auth_final_SHA512;
	} else {
		pg_auth_f.pg_crypto_auth_init = &hal_rtl_crypto_auth_init;
		pg_auth_f.pg_crypto_auth_update = &hal_rtl_crypto_auth_update;
		pg_auth_f.pg_crypto_auth_final = &hal_rtl_crypto_auth_final;
	}
#else
	pg_auth_f.pg_crypto_auth_init = &hal_rtl_crypto_auth_init;
	pg_auth_f.pg_crypto_auth_update = &hal_rtl_crypto_auth_update;
	pg_auth_f.pg_crypto_auth_final = &hal_rtl_crypto_auth_final;
#endif
	if (pg_auth_f.pg_crypto_auth_init(&_crypto_engine, crypto_hash_type, NULL, 0) != SUCCESS) {
		DBG_MISC_ERR("test_mode_hash_req_cmd: Hash Init Err!\r\n");
		return 0;
	}

	flash_mem = (SPI_FLASH_BASE + flash_offset);
	// Hash length seems cannot over 16K, or hash result is incorrect
	// we need to calculate hash block by block
	while (size > 0) {
		if (size > BOOT_IMG_HASH_BLK_SIZE) {
			hash_size = BOOT_IMG_HASH_BLK_SIZE;
		} else {
			hash_size = size;
		}
		pg_auth_f.pg_crypto_auth_update(&_crypto_engine, crypto_hash_type, (uint8_t *)flash_mem, hash_size);

		size -= hash_size;
		flash_mem += hash_size;
	}
	pg_auth_f.pg_crypto_auth_final(&_crypto_engine, crypto_hash_type, temp_hash_out_buf);

	flash_rtl_return_spi(&_hal_spic_adaptor);

#if 0
	// Only FPGA Debug, otu UART & dbg UART not use the same one would be better!
	dbg_printf("hash result:\r\n");
	dump_for_one_bytes(temp_hash_out_buf, HASH_RESULT_SIZE);
#endif

	hal_rtl_crypto_engine_deinit(&_crypto_engine);

	// send back the hash result over UART
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_hash_ret, _strlen(otu_hash_ret), 100);
	hal_rtl_uart_send(&otu_uart, (uint8_t *)temp_hash_out_buf, PG_HASH_RESULT_SIZE, 500);

	return 0;
}



void test_mode_img_download(void)
{
	uint32_t tx_pin;
	uint32_t rx_pin;
	uint8_t uart_idx;
//    uint8_t uart_pin_sel;
//    uint8_t baud_rate_sel;
	uint32_t baud_rate;
	int32_t ret;
	uint32_t cmd_idx;
	int32_t cryp_ret;
#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const cmd_shell_func_stubs_t *pcmd_shell_stubs = symb_ns4s_stubs->pcmd_shell_stubs;
#endif
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg2_t dbg_cfg;
	stdio_port_backup_t stdio_uart_bkup;

	dbg_printf("test_mode_img_download\r\n");
	// PG download UART re-init
#if CONFIG_PXP  // PXP
	rx_pin = STDIO_UART_RX_PIN;
	tx_pin = STDIO_UART_TX_PIN;
#else   // FPGA/ ASIC
	dbg_cfg.byte = potp_boot_cfg->byte.cfg2.byte;
	rx_pin = rom_log_uart_rx_pins[dbg_cfg.bit.rom_log_rx_pin_sel][dbg_cfg.bit.rom_log_port_sel];
	tx_pin = rom_log_uart_tx_pins[dbg_cfg.bit.rom_log_tx_pin_sel][dbg_cfg.bit.rom_log_port_sel];
#endif
	// default enable UART with baud rate 115200, no parity, no flow control to wait UART Cfg command

	uart_idx = hal_rtl_uart_rx_pin_to_idx(rx_pin);
	baud_rate = 115200;
	dbg_printf("Download Image over UART%u[tx=%u,rx=%u] baud=%lu\r\n", uart_idx,
			   PIN_NAME_2_PIN(tx_pin),
			   PIN_NAME_2_PIN(rx_pin), baud_rate);

	ret = otu_uart_port_open(tx_pin, rx_pin, baud_rate, 0, 0, &stdio_uart_bkup);
	if (ret != HAL_OK) {
		dbg_printf("OTU UART port open failed!\r\n");
		return;
	}
	// Initial NS region stdio port, this is very important for shell task parsing cmds!!!
#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_stubs->stdio_port_init_ns((void *)&otu_uart, (stdio_putc_t)&hal_rtl_uart_wputc, \
										(stdio_getc_t)&hal_rtl_uart_rgetc);
#else
	_stdio_port_init((void *)&otu_uart, (stdio_putc_t)&hal_rtl_uart_wputc, \
					 (stdio_getc_t)&hal_rtl_uart_rgetc);
#endif


	// Initial Cryptal engine HW
	cryp_ret = hal_rtl_crypto_engine_init(&_crypto_engine);
	if (cryp_ret != SUCCESS) {
		DBG_MISC_ERR("img_download: Crypto init failed(%d)!\r\n", cryp_ret);
	}
	_crypto_engine.arch_clean_dcache_by_size = (void (*)(uint32_t, int32_t))rtl_dcache_clean_by_addr;
	_crypto_engine.arch_invalidate_dcache_by_size = (void (*)(uint32_t, int32_t))rtl_dcache_invalidate_by_addr;

	/* Initial command shell */
#if defined(CONFIG_BUILD_SECURE)
	pcmd_shell_stubs->shell_cmd_task_init();
	pcmd_shell_stubs->shell_set_cmd_list(pcmd_shell_stubs->shell_cmd_hdl, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	pcmd_shell_stubs->shell_unregister_all(pcmd_shell_stubs->shell_cmd_hdl);
#else
	shell_cmd_task_init();
	_shell_set_cmd_list(&shell_cmd_hdl_rom, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	_shell_unregister_all(&shell_cmd_hdl_rom);
#endif

#if defined(CONFIG_BUILD_SECURE)
	for (cmd_idx = 0; cmd_idx < SHELL_ROM_CMD_LIST_SIZE; cmd_idx++) {
		pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_command_entry_t *)&pg_cmd_info[cmd_idx]);
	}
#else
	for (cmd_idx = 0; cmd_idx < SHELL_ROM_CMD_LIST_SIZE; cmd_idx++) {
		_shell_register(&shell_cmd_hdl_rom, &pg_cmd_info[cmd_idx]);
	}
#endif

	otu_disconnect = 0;
	while (1) {
#if defined(CONFIG_BUILD_SECURE)
		ret = pcmd_shell_stubs->shell_parse_one_cmd(pcmd_shell_stubs->shell_cmd_hdl);
#else
		ret = _shell_parse_one_cmd(&shell_cmd_hdl_rom);
#endif
		if (otu_disconnect != 0) {
			break;  // break the while loop
		}
	}

	// PG Tool uart port close
	otu_uart_port_close(&stdio_uart_bkup);

	// De-Initial Cryptal engine HW
	if (cryp_ret == SUCCESS) {
		cryp_ret = hal_rtl_crypto_engine_deinit(&_crypto_engine);
		if (cryp_ret != SUCCESS) {
			DBG_MISC_WARN("img_download: Crypto deinit failed(%d)!\r\n", cryp_ret);
		}
	}
}

int32_t test_mode_flash_basic_op_cmd(int argc, char **argv)
{
	uint32_t tCmdIdx;
	uint32_t tAddr;
	uint32_t tVal;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_flash_basic_op_cmd: no cmd\r\n");
		return 0;
	}
	tCmdIdx = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	if (argc >= 2) {
		tAddr = _strtoul((const char *)(argv[1]), (char **)NULL, 16);
	} else {
		tAddr = 0xc0;
	}

	if (argc >= 3) {
		tVal = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	} else {
		tVal = 0;
	}

	hal_rtl_snand_init(NULL);
	switch (tCmdIdx) {
	case 0x10:
		/* rdid */
		tVal = hal_rtl_snand_issueReadIdOpCmd(NULL);
		hal_rtl_uart_send(&otu_uart, (uint8_t *)&tVal, 3, 100/*mSec*/);
		break;
	case 0x11:
		/* reset */
		hal_rtl_snand_issusResetOpCmd(NULL);
		break;
	case 0x12:
		/* setf <regAddr> <regVal> */
		hal_rtl_snand_issueSetFeatureRegisterOpCmd(NULL, tAddr, tVal);
		break;
	case 0x13:
		/* getf <regAddr> */
		tVal = hal_rtl_snand_issueGetFeatureRegisterOpCmd(NULL, tAddr);
		hal_rtl_uart_send(&otu_uart, (uint8_t *)&tVal, 1, 100/*mSec*/);
		break;
	}
	hal_rtl_snand_deinit(NULL);
	return tVal;
}

int32_t test_mode_flash_nand_read_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	int32_t ret;


	uint32_t tPageAddr;
	uint32_t tColAddr;
	uint32_t tBufSize;
	uint32_t tDmaMode;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_flash_nand_read_cmd: no cmd\r\n");
		return 0;
	}
	tPageAddr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	if (argc >= 2) {
		tColAddr = _strtoul((const char *)(argv[1]), (char **)NULL, 16);
	} else {
		tColAddr = 0x0;
	}

	if (argc >= 3) {
		tBufSize = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	} else {
		tBufSize = SNAND_PG_PINGPONG_BUF_SIZE - tColAddr;
	}

	if (argc >= 4) {
		tDmaMode = _strtoul((const char *)(argv[3]), (char **)NULL, 10);
	} else {
		tDmaMode = 1;
	}

	_hal_snafc_adaptor.dma_en = tDmaMode;
	_hal_snafc_adaptor.col_addr = tColAddr;
	mpSnandPageBufAHead = &mSnandPageBufA[0];

	ret = fw_snafc_init(&_hal_snafc_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("Flash page read init err: BitMod=%u Sel=%u\r\n", flash_io, flash_pin);
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	hal_rtl_snand_pageRead(&_hal_snafc_adaptor, mpSnandPageBufAHead, tBufSize, tPageAddr);
	fw_snafc_deinit(&_hal_snafc_adaptor);
	hal_rtl_uart_send(&otu_uart, (uint8_t *)mpSnandPageBufAHead, tBufSize, 500/*mSec*/);

	return SUCCESS;
}

int32_t test_mode_flash_block_erase_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	uint32_t flash_offset = 0;
	uint32_t len = 0;
	uint32_t erase_offset;
	int32_t ret;

	if (argc < 2) {
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
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

	ret = fw_snafc_init(&_hal_snafc_adaptor, flash_io, flash_pin);

	if (ret != HAL_OK) {
		DBG_MISC_ERR("Flash block erase init err: BitMod=%u Sel=%u\r\n", flash_io, flash_pin);
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	erase_offset = flash_offset / 0x21000; /* flash_offset unit is byte, blockIdx unit is 132 KBytes. */
	while ((erase_offset * 0x21000) < (flash_offset + len)) {
		hal_rtl_snand_issueBlockEraseOpCmd(&_hal_snafc_adaptor, erase_offset * (0x40));
		erase_offset++;
	}

	fw_snafc_deinit(&_hal_snafc_adaptor);
	hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);

	return ret;
} /* test_mode_flash_block_erase_cmd */


int32_t test_mode_flash_nand_write_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0;
	int32_t ret;

	uint32_t tPageAddr;
	uint32_t tColAddr;
	uint32_t tBufSize;
	uint32_t tDmaMode;

	if (argc < 1) {
		DBG_MISC_ERR("test_mode_flash_nand_write_cmd: no cmd\r\n");
		return 0;
	}
	tPageAddr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	if (argc >= 2) {
		tColAddr = _strtoul((const char *)(argv[1]), (char **)NULL, 16);
	} else {
		tColAddr = 0x0;
	}

	if (argc >= 3) {
		tBufSize = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	} else {
		tBufSize = SNAND_PG_PINGPONG_BUF_SIZE - tColAddr;
	}

	if (argc >= 4) {
		tDmaMode = _strtoul((const char *)(argv[3]), (char **)NULL, 10);
	} else {
		tDmaMode = 1;
	}

	_hal_snafc_adaptor.dma_en = tDmaMode;
	_hal_snafc_adaptor.col_addr = tColAddr;
	mpSnandPageBufAHead = &mSnandPageBufA[0];

	ret = fw_snafc_init(&_hal_snafc_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("Flash page write init err: BitMod=%u Sel=%u\r\n", flash_io, flash_pin);
		hal_rtl_uart_send(&otu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
		return 0;
	}

	hal_rtl_snand_pageWrite(&_hal_snafc_adaptor, mpSnandPageBufAHead, tBufSize, tPageAddr);

	fw_snafc_deinit(&_hal_snafc_adaptor);

	return SUCCESS;
}


int32_t otu_rx_frame_direct_write_snand(char *ptr,  uint32_t frame_num, uint32_t frame_size)
{
	uint32_t tSel = 0;

	// start to program Flash
	/* For s-nor flash, Erase every 4KB alignment */
	/* For s-nand flash, Erase every 64 pages ((2K+64)x64) alignment */
	if ((frame_handler.flash_offset % 0x21000) == 0) {
		//dbg_printf("Block Erase 0x%x\r\n", frame_handler.flash_offset);
		// 1. Block boundary, block flash sector first
		hal_rtl_snand_issueBlockEraseOpCmd(&_hal_snafc_adaptor, (frame_handler.flash_offset / 0x21000) * (0x40));
	}

	/* Write data comes from PC into flash */
	//dbg_printf("Burst write 0x%x\r\n", frame_handler.flash_offset);

	/* Handle write buffer */
	/* 1. Collect data from XMODEM 1KB buffer to S-NAND page buffer (2KB+64B) (ping-pong buffer) */
	/* 2. Trigger S-NAND pageWrite while page buffer available */
	if ((mBufARemainSz == SNAND_PG_PINGPONG_BUF_SIZE) && (mBufBRemainSz == SNAND_PG_PINGPONG_BUF_SIZE)) {
		tSel = 0;
	} else if ((mBufARemainSz != SNAND_PG_PINGPONG_BUF_SIZE) && (mBufBRemainSz == SNAND_PG_PINGPONG_BUF_SIZE)) {
		tSel = 0;
	} else {
		tSel = 1;
	}

	if (tSel == 0) {
		if (mBufARemainSz >= frame_size) {
			_memcpy(mpSnandPageBufAHead + (SNAND_PG_PINGPONG_BUF_SIZE - mBufARemainSz), ptr, frame_size);
			mBufARemainSz -= frame_size;
		} else {
			_memcpy(mpSnandPageBufAHead + (SNAND_PG_PINGPONG_BUF_SIZE - mBufARemainSz), ptr, mBufARemainSz);
			/* hook on XMODEM 1KB RX callback handler */
			_memcpy(mpSnandPageBufBHead, ptr + mBufARemainSz, (1024 - mBufARemainSz));
			mBufBRemainSz -= (1024 - mBufARemainSz);
			mBufARemainSz = 0;
		}

		if (mBufARemainSz == 0) {
			hal_rtl_snand_pioPageWrite(&_hal_snafc_adaptor, mpSnandPageBufAHead, SNAND_PG_PINGPONG_BUF_SIZE, (frame_handler.flash_offset / SNAND_PG_PINGPONG_BUF_SIZE));
			mBufARemainSz = SNAND_PG_PINGPONG_BUF_SIZE;
		}
	} else {
		/* if (tSel==1) */
		if (mBufBRemainSz >= frame_size) {
			_memcpy(mpSnandPageBufBHead + (SNAND_PG_PINGPONG_BUF_SIZE - mBufBRemainSz), ptr, frame_size);
			mBufBRemainSz -= frame_size;
		} else {
			_memcpy(mpSnandPageBufBHead + (SNAND_PG_PINGPONG_BUF_SIZE - mBufBRemainSz), ptr, mBufBRemainSz);
			/* hook on XMODEM 1KB RX callback handler */
			_memcpy(mpSnandPageBufAHead, ptr + mBufBRemainSz, (1024 - mBufBRemainSz));
			mBufARemainSz -= (1024 - mBufBRemainSz);
			mBufBRemainSz = 0;
		}
		if (mBufBRemainSz == 0) {
			hal_rtl_snand_pioPageWrite(&_hal_snafc_adaptor, mpSnandPageBufBHead, SNAND_PG_PINGPONG_BUF_SIZE, (frame_handler.flash_offset / SNAND_PG_PINGPONG_BUF_SIZE));
			mBufBRemainSz = SNAND_PG_PINGPONG_BUF_SIZE;
		}
	}

	frame_handler.flash_offset += frame_size;

	return frame_size;
}

int32_t otu_fw_download_snand(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset)
{
	hal_status_t ret;
	uint32_t fw_dnload_size;
	uint8_t flash_io;
	uint8_t flash_pin;

	flash_io = (flash_sel >> 8) & 0xFF;
	flash_pin = flash_sel & 0xFF;
	DBG_MISC_INFO("Init XModem...\r\n");
	xmodem_uart_init(&xm_uart_port, potu_uart, xm_rx_buf, XM_BUF_SIZE);
	_memset((void *)&xmodem_ctrl, 0, sizeof(XMODEM_CTRL));
	xmodem_uart_func_hook(&(xmodem_ctrl.ComPort), &xm_uart_port);

	// Initial Flash controller
	DBG_MISC_INFO("Init Flash IOMod=%u PinSel=%u...\r\n", flash_io, flash_pin);

	ret = fw_snafc_init(&_hal_snafc_adaptor, flash_io, flash_pin);
	if (ret != HAL_OK) {
		DBG_MISC_ERR("OTU Flash(BitMod=%u Sel=%u) Init Err\r\n", flash_io, flash_pin);
		return 0;
	}

	// Start XModem transfer
	_memset((void *)&frame_handler, 0, sizeof(frame_handler));
	frame_handler.flash_offset = flash_offset;  // it must 4K aligned
	DBG_MISC_INFO("Start XModem..\r\n");
	mpSnandPageBufAHead = &mSnandPageBufA[0];
	mpSnandPageBufBHead = &mSnandPageBufB[0];
	mBufARemainSz = SNAND_PG_PINGPONG_BUF_SIZE;
	mBufBRemainSz = SNAND_PG_PINGPONG_BUF_SIZE;
	xModemStart(&xmodem_ctrl, (char *)xmodem_frame_buf, (RxFrameHandler_t)otu_rx_frame_direct_write_snand);

	fw_dnload_size = xModemRxBuffer(&xmodem_ctrl, (16 * 1024 * 1024)); // maximum 16-MB

	DBG_MISC_INFO("\r\nXModem transfer end, %lu bytess transfered\r\n", fw_dnload_size);
	xModemEnd(&xmodem_ctrl);
	xmodem_uart_deinit(&xm_uart_port);

	if ((xmodem_ctrl.result != 0) || (fw_dnload_size <= 0)) {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_err, _strlen(otu_err), 100);
	} else {
		hal_rtl_uart_send(potu_uart, (uint8_t *)otu_ok, _strlen(otu_ok), 100);
	}

	fw_snafc_deinit(&_hal_snafc_adaptor);

	return fw_dnload_size;
}

int32_t test_mode_img_download_snand_cmd(int argc, char **argv)
{
	uint32_t flash_io = SpicOneIOMode;
	uint32_t flash_pin = BootFlash_DualIOS0; //Confirm flash pin select 0
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

	ret = otu_fw_download_snand(&otu_uart, (flash_io << 8) | flash_pin, flash_offset);
	return ret;
}
