/**************************************************************************//**
 * @file     startup.c
 * @brief    The System startup and HAL initialization for rtl8735b.
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
#include "rtl8735b_ramstart.h"
#include "rtl8735b_symbns4s.h"
#include "stdio_port.h"
#include <arm_cmse.h>
#include "otp_boot_cfg.h"
#include "memory.h"
#include "fw_img.h"
#include "fw_img_tlv.h"
#include "fw_uart_boot.h"
#include "fw_snand_boot.h"
#include "shell.h"

#define SECTION_SYSSTART_TEXT                 SECTION(".sysstart.text")
#define SECTION_SYSSTART_BSS                  SECTION(".sysstart.bss")

#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip
#define PG_TRAP_PIN                            (PIN_E8)
#define CPFT_TRAP_PIN                          (PIN_E7)
#else   // After A-cut
#define PG_TRAP_PIN                            (PIN_E5)
#define CPFT_TRAP_PIN                          (PIN_E6)
#endif
#define TST_GPIO_PIN_NUM                       (4)
#define NORMAL_MODE_PG_PTN                     (0x35)

#define RAM_FUN_TABLE_VALID_ITCM_START_ADDR    (0x00010000)
#define RAM_FUN_TABLE_VALID_ITCM_END_ADDR      (0x00030000)
#define RAM_FUN_TABLE_VALID_START_ADDR         (0x20100000)
#define RAM_FUN_TABLE_VALID_END_ADDR           (0x20180000)


enum sic_pins_e {
	PIN_SIC_SCL   = PIN_A2,
	PIN_SIC_SDA   = PIN_A3
};

enum {
	TEST_MODE_MISC_OPT0  = 0x00,
	TEST_MODE_MISC_OPT1  = 0x01,
	TEST_MODE_MISC_OPT2  = 0x02,
	TEST_MODE_MISC_OPT3  = 0x03,
	TEST_MODE_MISC_OPT4  = 0x04,
	TEST_MODE_MISC_OPT5  = 0x05,
	TEST_MODE_MISC_OPT6  = 0x06,
	TEST_MODE_MISC_OPT7  = 0x07,
	TEST_MODE_MISC_OPT8  = 0x08,
	TEST_MODE_MISC_OPT9  = 0x09,
	TEST_MODE_MISC_OPT10 = 0x0A,
	TEST_MODE_MISC_OPT11 = 0x0B,
	TEST_MODE_MISC_OPT12 = 0x0C,
	TEST_MODE_MISC_OPT13 = 0x0D,
	TEST_MODE_MISC_OPT14 = 0x0E,
	TEST_MODE_MISC_OPT15 = 0x0F
};

enum {
	BOOT_MOD_OTP_W = 0x0,
	BOOT_MOD_OTP_R = 0x1
};

typedef void __attribute__((cmse_nonsecure_call)) nsfunc(void);

extern uint32_t ConfigDebugErr;
extern uint32_t ConfigDebugWarn;
extern uint32_t ConfigDebugInfo;

extern uint32_t __Vectors;      // ROM Vector table
extern int_vector_t ram_vector_table[];
extern hal_spic_adaptor_t _hal_spic_adaptor;
extern hal_snafc_adaptor_t _hal_snafc_adaptor;
extern uboot_xm_frame_handler_t uboot_frame_handle;
extern hal_gpio_func_stubs_t hal_gpio_stubs;

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
extern uint8_t Image$$_RAM_FUNC_TBL$$Base[];
#define __ram_start_table_start__ Image$$_RAM_FUNC_TBL$$Base
extern uint8_t Image$$_ROM_BSS$$Base[];
#define __rom_bss_start__ Image$$_ROM_BSS$$Base
extern uint8_t Image$$_ROM_BSS$$ZI$$Limit[];
#define __rom_bss_end__ Image$$_ROM_BSS$$ZI$$Limit
extern uint32_t Load$$_ROM_DATA$$Base;
#define __rom_etext Load$$_ROM_DATA$$Base
extern uint32_t Image$$_ROM_DATA$$Base;
#define __rom_data_start__ Image$$_ROM_DATA$$Base
extern uint32_t Image$$_ROM_DATA$$Limit;
#define __rom_data_end__ Image$$_ROM_DATA$$Limit
extern uint8_t Image$$_ROM_TEMP_BSS$$Base[];
#define __rom_temp_bss_start__ Image$$_ROM_TEMP_BSS$$Base
extern uint8_t Image$$_ROM_TEMP_BSS$$ZI$$Limit[];
#define __rom_temp_bss_end__ Image$$_ROM_TEMP_BSS$$ZI$$Limit
extern uint8_t Image$$_RAM_IMG_SIGN$$Base[];
#define __ram_img_signature__ Image$$_RAM_IMG_SIGN$$Base
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
extern uint8_t Image$$_RAM_FUNC_TBL$$Base[];
#define __ram_start_table_start__ Image$$_RAM_FUNC_TBL$$Base
extern uint8_t Image$$_ROM_BSS$$ZI$$Base[];
#define __rom_bss_start__ Image$$_ROM_BSS$$ZI$$Base
extern uint8_t Image$$_ROM_BSS$$ZI$$Limit[];
#define __rom_bss_end__ Image$$_ROM_BSS$$ZI$$Limit
extern uint32_t Load$$_ROM_DATA$$Base;
#define __rom_etext Load$$_ROM_DATA$$Base
extern uint32_t Image$$_ROM_DATA$$Base;
#define __rom_data_start__ Image$$_ROM_DATA$$Base
extern uint32_t Image$$_ROM_DATA$$Limit;
#define __rom_data_end__ Image$$_ROM_DATA$$Limit
extern uint8_t Image$$_ROM_TEMP_BSS$$Base[];
#define __rom_temp_bss_start__ Image$$_ROM_TEMP_BSS$$Base
extern uint8_t Image$$_ROM_TEMP_BSS$$ZI$$Limit[];
#define __rom_temp_bss_end__ Image$$_ROM_TEMP_BSS$$ZI$$Limit
extern uint8_t Image$$_RAM_IMG_SIGN$$Base[];
#define __ram_img_signature__ Image$$_RAM_IMG_SIGN$$Base
extern uint8_t Image$$_ROM_SBOOT_BSS$$Base[];
#define __rom_sboot_bss_start__ Image$$_ROM_SBOOT_BSS$$Base
extern uint8_t Image$$_ROM_SBOOT_BSS$$ZI$$Limit[];
#define __rom_sboot_bss_end__ Image$$_ROM_SBOOT_BSS$$ZI$$Limit
#elif defined ( __GNUC__ )
extern uint32_t __StackTop;
extern uint8_t __ram_start_table_start__[];
extern uint8_t __rom_bss_start__[];
extern uint8_t __rom_bss_end__[];
extern uint32_t __rom_etext;
extern uint32_t __rom_data_start__;
extern uint32_t __rom_data_end__;
extern uint8_t __rom_temp_bss_start__[];
extern uint8_t __rom_temp_bss_end__[];
extern uint8_t __ram_img_signature__[];
extern uint8_t __rom_sboot_bss_start__[];
extern uint8_t __rom_sboot_bss_end__[];
extern uint8_t __rom_boot_loader_tmp_buf_start__[];
extern uint8_t __rom_boot_loader_tmp_buf_end__[];
#endif

SECTION_SYSSTART_BSS uint8_t fast_boot;
SECTION_SYSSTART_BSS PRAM_FUNCTION_START_TABLE pRamStartFun;
SECTION_SYSSTART_BSS PRAM_FUNCTION_START_TABLE pFastBootStartFun;

#if !CONFIG_BOOT_TRAP_CTRL
SECTION_SYSSTART_BSS uint32_t test_mode;
SECTION_SYSSTART_BSS uint8_t test_pg_trap_gpio;
SECTION_SYSSTART_BSS uint8_t test_gpio_port;
SECTION_SYSSTART_BSS uint32_t boot_mode;
#endif

SECTION_SYSSTART_BSS uint8_t fast_reboot_rdy;

#if CONFIG_FPGA // FPGA TEST ONLY
#if CONFIG_BOOT_SETMOD_EN
SECTION_SYSSTART_BSS uint8_t exit_set_mode;
SECTION_SYSSTART_BSS shell_command_entry_t cmd_info[5];
#endif
#endif // FPGA TEST ONLY

extern int32_t otu_fw_download_cmd(int argc, char **argv);
extern int32_t test_mode_flash_chip_erase_cmd(int argc, char **argv);
extern int32_t test_mode_flash_sector_erase_cmd(int argc, char **argv);
extern int32_t otu_fw_download(uint32_t uart_sel, uint32_t baud_rate, uint32_t flash_sel, uint32_t flash_offset);
int32_t otp_operate_cmd(int argc, char **argv);
#if CONFIG_FPGA  // FPGA
int32_t otp_dump_all_cmd(int argc, char **argv);
#endif
extern hal_status_t fw_spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, u8 io_pin_sel);
extern int32_t flash_boot(PRAM_FUNCTION_START_TABLE *pram_start_func);
extern int32_t uart_boot(PRAM_FUNCTION_START_TABLE *pram_start_func);
extern void shell_cmd_task_init(void);
extern void rom_cmd_shell_set_prompt(void);
extern void shell_cmd_task(void);
extern _LONG_CALL_ void test_mode_img_download(void);
extern void _LONG_CALL_ *_memset(void *dst0, int Val, SIZE_T length);
extern void clear_export_partition_tbl(void);
extern void erase_loaded_image(void);
extern int32_t is_key_valid(uint8_t *pkey);

_LONG_CALL_ extern int rtl_cryptoEngine_init_s(void *pIE1);
_LONG_CALL_ extern int rtl_cryptoEngine_deinit_s(void *pIE1);
_LONG_CALL_ extern int rtl_crypto_aes_cbc_init_s(IN const u8 *key, IN const u32 keylen);
_LONG_CALL_ extern int rtl_crypto_aes_cbc_decrypt_s(
	IN const u8 *message,   IN const u32 msglen,
	IN const u8 *iv,        IN const u32 ivlen,
	OUT u8 *pResult);
_LONG_CALL_ extern int rtl_crypto_aes_cbc_encrypt_s(
	IN const u8 *message,   IN const u32 msglen,
	IN const u8 *iv,        IN const u32 ivlen,
	OUT u8 *pResult);
_LONG_CALL_ extern int rtl_crypto_hmac_sha2_256_init_s(IN const u8 *key, IN const u32 keylen);
_LONG_CALL_ extern int rtl_crypto_hmac_sha2_256_update_s(IN const u8 *message, IN const u32 msglen);
_LONG_CALL_ extern int rtl_crypto_hmac_sha2_256_final_s(OUT u8 *pDigest);
_LONG_CALL_ u32 _strtoul(const char *nptr, char **endptr, int base);
#if 1
extern void ram_start(void) __attribute__((noreturn));
extern uint8_t uart_tr_pin[];
extern shell_command_t shell_cmd_hdl_rom;

extern hal_crypto_adapter_t _crypto_engine;
extern uint8_t export_hash_priv_key[];
extern uint8_t ss_priv_key[PRIV_KEY_SIZE];
extern uint8_t aes_iv[];
extern uint8_t aes_key[AES_KEY_SIZE];
extern const uint8_t aes_init_iv[];
extern uint8_t hash_priv_key_enced[PRIV_KEY_SIZE];
extern uint8_t hash_priv_key[PRIV_KEY_SIZE];
extern uint8_t hash_key[HASH_KEY_SIZE];
extern uint8_t temp_dec_buf[];
#endif
extern shell_command_t shell_cmd_hdl_rom;

//typedef void __attribute__((cmse_nonsecure_call)) (hal_uart_adapter_init) (phal_uart_adapter_t puart_adapter, u8 uart_idx, phal_uart_defconfig_t pconfig);

// fp can point to a secure function or a non-secure function
//nsfunc *fp_hal_uart_adapter_init = cmse_nsfptr_create ((nsfunc *) hal_uart_adapter_init_rtl8195bhp));

void rom_bss_data_init(void);

const char RamImgValidatePat[] = {
	'A', 'm', 'e', 'b', 'a', 'P', 'r', 'o', '2', 0xFF
};

const u8 cmd_shell_pwd[] = {
	'R', 'T', 'K', '8', '7', '3', '5', 'B'
};

const uint32_t rom_log_uart_rx_pins[2][4] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip    
	/*           UART1,    UART2,  UART3    UART1    */
	/* S0 */    {PIN_F4,  PIN_D16, PIN_E6, PIN_F4},
	/* S1 */    {PIN_F12,  PIN_E2, PIN_E6, PIN_F12}
#else   // After A-cut
	/*           UART1,    UART2,  UART3    UART1    */
	/* S0 */    {PIN_F3,  PIN_D16, PIN_E2, PIN_F3},
	/* S1 */    {PIN_F12,  PIN_D20, PIN_E2, PIN_F12}
#endif
};

const uint32_t rom_log_uart_tx_pins[2][4] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip       
	/*           UART1,    UART2,  UART3    UART1    */
	/* S0 */    {PIN_F5,  PIN_D15, PIN_E5, PIN_F5},
	/* S1 */    {PIN_F13,  PIN_E1, PIN_E5, PIN_F13}
#else   // After A-cut
	/*           UART1,    UART2,  UART3    UART1    */
	/* S0 */    {PIN_F4,  PIN_D15, PIN_E1, PIN_F4},
	/* S1 */    {PIN_F13,  PIN_E0, PIN_E1, PIN_F13}
#endif
};

const uint32_t boot_uart_baud_sel_map[8] = {
	115200,  230400,  460800,  1000000,
	2000000, 3000000, 4000000, 6000000
};

const uint32_t _dbg_port_tms_io_pin_sel[2] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip      
	/* S0 */    PIN_A0,
	/* S1 */    PIN_F11
#else   // After A-cut
	/* S0 */    PIN_A0,
	/* S1 */    PIN_F10
#endif
};

const uint32_t _dbg_port_clk_pin_sel[2] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip      
	/* S0 */    PIN_A1,
	/* S1 */    PIN_F10
#else   // After A-cut
	/* S0 */    PIN_A1,
	/* S1 */    PIN_F9
#endif
};

const uint32_t _dbg_port_jtag_fixed_pins[3] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip       
	/* pin_tdo, pin_tdi, pin_trst */
	PIN_F6, PIN_F7, PIN_F12
#else   // After A-cut
	/* pin_tdo, pin_tdi, pin_trst */
	PIN_F6, PIN_F7, PIN_F11
#endif
};

const char sce_cfg_sign[] = "8735bsce";

const shell_command_entry_t boot_cmd_tbl[SHELL_BOOT_ROM_CMD_LIST_SIZE] = {
	{
		"BOOT_ROM", (const char *)"fwd", (shell_program_t)otu_fw_download_cmd,
		(const char *)" fwd <uart_pin_tx, Dec> <uart_pin_rx, Dec> <baud_rate, Dec> <parity> <flow ctrl> \r\n"
		"\t\t\t\t    <flash_io> <flash_pin> <flash_offset_4K_aligned>:\r\n"
		"\t\t\t\tTo download Flash image over UART\r\n"
	},
	{
		"BOOT_ROM", (const char *)"ceras", (shell_program_t)test_mode_flash_chip_erase_cmd,
		(const char *)" ceras <io_mode> <pin_sel> \r\n"
		"\t\t\t\tFlash chip erase\r\n"
	},
	{
		"BOOT_ROM", (const char *)"seras", (shell_program_t)test_mode_flash_sector_erase_cmd,
		(const char *)" seras <offset> <len> <io_mode> <pin_sel> \r\n"\
		"\t\t\t\tFlash sector erase\r\n"
	},
	{
		"BOOT_ROM", (const char *)"otp_op", (shell_program_t)otp_operate_cmd,
		(const char *)" otp_op <write(0x0)/read(0x1)> <otp_mem_address, Hex> <w_Value, Hex> \r\n"
		"\t\t\t\tOTP Read/Write one byte operate\r\n"
	},

#if CONFIG_FPGA  // FPGA
	{
		"BOOT_ROM", (const char *)"otp_dp_all", (shell_program_t)otp_dump_all_cmd,
		(const char *)" otp_dp_all \r\n"
		"\t\t\t\tOTP Dump all bytes for rom simu\r\n"
	},
#else   // PXP & ASIC
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
#endif
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL},
	{"BOOT_ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL}   // end of table
};

const uint8_t otp_rma1_mgn[RMA1_MGN_PTN_SIZE] = {
	0x38, 0x37, 0x33, 0x35, 0x42, 0x72, 0x6D, 0x61
};

SECTION_ROM_TEMP_BSS hal_uart_adapter_t stdio_uart;
SECTION_ROM_TEMP_BSS shell_command_entry_t shell_rom_cmd_list[SHELL_ROM_CMD_LIST_SIZE];
SECTION_ROM_TEMP_BSS hal_timer_group_adapter_t _timer_group1;
SECTION_ROM_TEMP_BSS hal_timer_adapter_t _system_timer;
SECTION_ROM_TEMP_BSS hal_timer_group_adapter_t _timer_group3;
SECTION_ROM_TEMP_BSS hal_timer_adapter_t _fcs_system_timer;
SECTION_ROM_TEMP_BSS hal_pin_mux_mang_t _pinmux_manager;
SECTION_ROM_TEMP_BSS hal_gpio_comm_adapter_t gpio_comm;
SECTION_ROM_TEMP_BSS hal_gpio_adapter_t test_mode_trap_gpio;
SECTION_ROM_TEMP_BSS uint32_t dbg_uart_rx_pin;

SECTION_SYSSTART_TEXT
BOOLEAN ram_start_addr_valid(uint32_t addr)
{

	if (((addr >= RAM_FUN_TABLE_VALID_ITCM_START_ADDR) && (addr < RAM_FUN_TABLE_VALID_ITCM_END_ADDR)) ||
		((addr >= RAM_FUN_TABLE_VALID_START_ADDR) && (addr < RAM_FUN_TABLE_VALID_END_ADDR))) {
		return 1;
	} else {
		return 0;
	}
}

SECTION_SYSSTART_TEXT
void stdio_uart_port_init(void)
{
	uint32_t tx_pin;
	uint32_t rx_pin;
	uint8_t uart_idx;
	uint32_t i;

#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
#endif
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg2_t dbg_cfg;
	/*
	 *   Normal path:
	 * - directly using UART1 as log UART
	 *
	 */
#if CONFIG_PXP  // PXP
	rx_pin = STDIO_UART_RX_PIN;
	tx_pin = STDIO_UART_TX_PIN;
#else   // FPGA/ ASIC
	dbg_cfg.byte = potp_boot_cfg->byte.cfg2.byte;
	rx_pin = rom_log_uart_rx_pins[dbg_cfg.bit.rom_log_rx_pin_sel][dbg_cfg.bit.rom_log_port_sel];
	tx_pin = rom_log_uart_tx_pins[dbg_cfg.bit.rom_log_tx_pin_sel][dbg_cfg.bit.rom_log_port_sel];
#endif
	uart_idx = hal_rtl_uart_rx_pin_to_idx(rx_pin);
	hal_rtl_pinmux_register(rx_pin, (PID_UART0 + uart_idx));
	hal_rtl_pinmux_register(tx_pin, (PID_UART0 + uart_idx));

	// RX Pin pull-high to prevent this folating on this pin
	dbg_uart_rx_pin = rx_pin;
	hal_rtl_gpio_pull_ctrl(dbg_uart_rx_pin, Pin_PullUp);
	// wait ready, delay ~4us
	for (i = 0; i < 45; i++) {
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
	hal_rtl_uart_init(&stdio_uart, tx_pin, rx_pin, NULL);
	hal_rtl_uart_set_baudrate(&stdio_uart, 115200);
	hal_rtl_uart_set_format(&stdio_uart, 8, UartParityNone, 1);

	// hook the putc function to stdio
	_stdio_port_init((void *)&stdio_uart, (stdio_putc_t)&hal_rtl_uart_wputc,
					 (stdio_getc_t)&hal_rtl_uart_rgetc);
#if defined(CONFIG_BUILD_SECURE)
	// Initial NS region stdio port
	symb_ns4s_stubs->stdio_port_init_ns((void *)&stdio_uart, (stdio_putc_t)&hal_rtl_uart_wputc, \
										(stdio_getc_t)&hal_rtl_uart_rgetc);
#endif
	ConfigDebugErr = 0xFFFFFFFF;
}

SECTION_SYSSTART_TEXT
int32_t chk_ram_img_signature(char *sign)
{
	uint32_t i;
	int32_t ret = 0;

	for (i = 0; i < sizeof(RamImgValidatePat); i++) {
		if (RamImgValidatePat[i] == 0xFF) {
			break;
		}

		if (RamImgValidatePat[i] != sign[i]) {
//            dbg_printf ("Invalid RAM Image Signature!\r\n");
			ret = 1;
			break;
		}
	}

	return ret;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void get_chip_ver_info(uint8_t *vndr_id, uint8_t *chip_ver)
{

	/*
	  AON 0x4000_90C4[11:8] -> vendor id
	  AON 0x4000_90C4[7:4] -> chip version
	 */
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;
	reg_val = aon->AON_REG_AON_SYS_INFO0;
	*vndr_id   = (reg_val & AON_MASK_VENDOR_ID);
	*chip_ver  = (reg_val & AON_MASK_CHIP_VER);
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void get_rom_ver_info(uint8_t *ver_main, uint8_t *ver_sub)
{
	/*
	  AON 0x4000_90C8[7:4] -> Main Number
	  AON 0x4000_90C8[3:0] -> Sub Number
	 */
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;
	reg_val = aon->AON_REG_AON_SYS_INFO1;
	*ver_main = ((reg_val & AON_MASK_SYSCFG_ROM_INFO) & 0xF0) >> 4;
	*ver_sub  = ((reg_val & AON_MASK_SYSCFG_ROM_INFO) & 0xF);
}

SECTION_SYSSTART_TEXT
void show_welcome_message(void)
{

	uint8_t vndor_id, chip_ver, rom_ver_main, rom_ver_sub;
	dbg_printf("\r\n== Rtl8735b IoT Platform ==\r\n");
	get_chip_ver_info(&vndor_id, &chip_ver);
	get_rom_ver_info(&rom_ver_main, &rom_ver_sub);
	dbg_printf("Chip VID: %u, Ver: %u\r\n", vndor_id, chip_ver);
	dbg_printf("ROM Version: v%d.%d\r\n", rom_ver_main, rom_ver_sub);
#if CONFIG_FPGA || CONFIG_PXP
	dbg_printf("ROM Build @ %s, %s\r\n", __TIME__, __DATE__);
#endif
}

SECTION_SYSSTART_TEXT
void rom_temp_bss_clean_up(void)
{
	u32 bss_len;

	bss_len = (__rom_temp_bss_end__ - __rom_temp_bss_start__);
	_memset((void *)__rom_temp_bss_start__, 0, bss_len);
}

#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN)
#define IMG2_START_ADDR_REG                 (0x50000828)  // in S region

SECTION_SYSSTART_TEXT
void erase_boot_loader(uint32_t code_start, uint32_t code_size, uint32_t img2_entry)
{
	PRAM_FUNCTION_START_TABLE pRamStartFun = (PRAM_FUNCTION_START_TABLE)0;
	if (code_size > 0) {
		_memset((void *)code_start, 0, code_size);
		rtl_dcache_clean_by_addr((uint32_t *)code_start, code_size);
	}
	pRamStartFun = (PRAM_FUNCTION_START_TABLE)img2_entry;
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 30);
	if (chk_ram_img_signature(pRamStartFun->Signature)) {
		dbg_printf("erase_boot_loader==> code_start=0x%x code_size=0x%x pRamStartFun=0x%x\r\n", \
				   code_start, code_size, img2_entry);
		dbg_printf("erase_boot_loader: Invalid Image2 Signature!\r\n");
		goto __erase_boot_loader_err;
	} else {
		// ram img simple signature check pass
		rtl_dcache_disable();
		pFastBootStartFun = (PRAM_FUNCTION_START_TABLE)img2_entry;
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 31);

		if ((pRamStartFun->msp_start != 0) && (pRamStartFun->msp_start > pRamStartFun->msp_limit)) {
			// switch the main stack memory as the application defined
			__disable_irq();
			__ISB();
			__set_MSP((uint32_t)pRamStartFun->msp_start);
			__set_MSPLIM((uint32_t)pRamStartFun->msp_limit);
			// !!! Don't access local variable after this line... !!!
			__enable_irq();
			__DSB();
			__ISB();
		}

		pRamStartFun->RamStartFun();
	}
__erase_boot_loader_err:
	//boot_fail_log_uart_resume();
	//boot_command_shell();
	return;
}


SECTION_SYSSTART_TEXT
void rom_sboot_bss_clean_up(void)
{
	u32 bss_len;

	// Clean secure boot memory to prevent securite leakedge
	bss_len = (__rom_sboot_bss_end__ - __rom_sboot_bss_start__);
	_memset((void *)__rom_sboot_bss_start__, 0, bss_len);
	rtl_dcache_clean_by_addr((void *)__rom_sboot_bss_start__, bss_len);
}

SECTION_SYSSTART_TEXT
void rom_sboot_export_bss_clean_up(void)
{
	u32 bss_len;

	// Clean secure boot memory to prevent securite leakedge
	bss_len = (__rom_boot_loader_tmp_buf_end__ - __rom_boot_loader_tmp_buf_start__);
	_memset((void *)__rom_boot_loader_tmp_buf_start__, 0, bss_len);
	rtl_dcache_clean_by_addr((void *)__rom_boot_loader_tmp_buf_start__, bss_len);


}

SECTION_SYSSTART_TEXT
void fast_boot_xip_sce_restore(hal_xip_sce_cfg_t *pxip_sce_cfg, uint32_t key_rdy)
{

}

SECTION_SYSSTART_TEXT
int32_t fast_boot_img2(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	int ret = 0;
	return ret;
}

#endif  // #if defined(CONFIG_BUILD_SECURE)

SECTION_SYSSTART_TEXT
void super_secure_otp_lock(void)
{

}

SECTION_SYSSTART_TEXT
int32_t otp_operate_cmd(int argc, char **argv)
{
	u8 read_data = 0xff;
	u8 argc_idx;
	u32 InPutData[5];

	if (argc > 5) {
		argc = 5;   // prevent stack corrupting
	}

	for (argc_idx = 0; argc_idx < argc; argc_idx++) {
		InPutData[argc_idx] = _strtoul((const char *)(argv[argc_idx]), (char **)NULL, 16);
	}
	hal_rtl_otp_init();
	switch (InPutData[0]) {
	case BOOT_MOD_OTP_W:
		// Write OTP one byte
		dbg_printf("\n ==== write OTP one byte ====\r\n");
		dbg_printf("write 0x%x = 0x%x\r\n", InPutData[1], InPutData[2]);
		//hal_rtl_otp_byte_wr_syss(InPutData[1], (uint8_t)InPutData[2]);
		hal_rtl_otp_byte_wr_marr_syss(InPutData[1], (uint8_t)InPutData[2], DISABLE);
		break;
	case BOOT_MOD_OTP_R:
		// Read OTP one byte
		dbg_printf("\n ==== read OTP one byte ====\r\n");
		read_data = hal_rtl_otp_byte_rd_syss(InPutData[1]);
		dbg_printf("read 0x%x = 0x%x\r\n", InPutData[1], read_data);
		break;
	default:
		break;
	}
	hal_rtl_otp_deinit();
	return 0;
}

#if CONFIG_FPGA  // FPGA
SECTION_SYSSTART_TEXT
int32_t otp_dump_all_cmd(int argc, char **argv)
{
	uint8_t argc_idx;
	uint32_t InPutData[5], j = 0x0;
	uint32_t otp_addr = 0x0;
	const uint32_t otp_mem_max_size = 2048;
	uint8_t mem_buf[2048];

	if (argc > 5) {
		argc = 5;   // prevent stack corrupting
	}

	for (argc_idx = 0; argc_idx < argc; argc_idx++) {
		InPutData[argc_idx] = _strtoul((const char *)(argv[argc_idx]), (char **)NULL, 16);
	}
	_memset(&mem_buf[0], 0xFF, otp_mem_max_size);
	hal_rtl_otp_init();
	for (j = 0; j < otp_mem_max_size; j++) {
		mem_buf[j] = hal_rtl_otp_byte_rd_syss((otp_addr + j));
	}
	hal_rtl_otp_deinit();
	dbg_printf("------------- Dump all OTP Mem -------------\r\n");
	for (j = 0; j < otp_mem_max_size; j = j + 16) {
		dbg_printf("[addr:0x%03x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				   (otp_addr + j), mem_buf[0 + j], mem_buf[1 + j], mem_buf[2 + j], mem_buf[3 + j], mem_buf[4 + j], mem_buf[5 + j], mem_buf[6 + j], mem_buf[7 + j],
				   mem_buf[8 + j], mem_buf[9 + j], mem_buf[10 + j], mem_buf[11 + j], mem_buf[12 + j], mem_buf[13 + j],
				   mem_buf[14 + j], mem_buf[15 + j]);
	}
	dbg_printf("\r\n");
	return 0;
}
#endif

void boot_command_shell(void) __attribute__((noreturn));

SECTION_SYSSTART_TEXT
void boot_command_shell(void)
{
	uint32_t cmd_idx;
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;

	// password confirm
	char pwd[9];

	// Inhibit Supre-Secure OTP access
	//super_secure_otp_lock();

	uint32_t i;

	if ((BootDbgMsgOff == (potp_boot_cfg->byte.cfg2.bit.boot_rom_dbg_msg_dis)) &&
		(DISABLE == stdio_uart.is_inited)) {
		stdio_uart_port_init();
		ConfigDebugErr = 0xFFFFFFFF;
	}

	for (i = 0; i < 8; i++) {
		while (!_stdio_port_getc(&pwd[i]));
	}
	if (_memcmp_s((void *)pwd, cmd_shell_pwd, 8)) {
		// Wrong pass word
		while (1);
	}

	// wdt(AON/VNDR) disable
	hal_rtl_wdt_ctrl_all_disable();

	dbg_printf("<Boot command shell on-going>\r\n");
#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const cmd_shell_func_stubs_t *pcmd_shell_stubs = symb_ns4s_stubs->pcmd_shell_stubs;
	// Initial NS region stdio port
	symb_ns4s_stubs->stdio_port_init_ns((void *)&stdio_uart, (stdio_putc_t)&hal_rtl_uart_wputc, \
										(stdio_getc_t)&hal_rtl_uart_rgetc);

	// Initial command shell
	pcmd_shell_stubs->shell_cmd_task_init();
	pcmd_shell_stubs->shell_rom_cmd_set_prompt();
	pcmd_shell_stubs->shell_set_cmd_list(pcmd_shell_stubs->shell_cmd_hdl, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	pcmd_shell_stubs->shell_unregister_all(pcmd_shell_stubs->shell_cmd_hdl);
	for (cmd_idx = 0; cmd_idx < SHELL_BOOT_ROM_CMD_LIST_SIZE; cmd_idx++) {
		pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, (shell_command_entry_t *) &boot_cmd_tbl[cmd_idx]);
	}

	while (1) {
		pcmd_shell_stubs->shell_task(pcmd_shell_stubs->shell_cmd_hdl);
	}
#else
	// Initial command shell
	shell_cmd_task_init();
	rom_cmd_shell_set_prompt();
	_shell_set_cmd_list(&shell_cmd_hdl_rom, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	_shell_unregister_all(&shell_cmd_hdl_rom);
	for (cmd_idx = 0; cmd_idx < SHELL_BOOT_ROM_CMD_LIST_SIZE; cmd_idx++) {
		_shell_register(&shell_cmd_hdl_rom, &boot_cmd_tbl[cmd_idx]);
	}

	while (1) {
		shell_cmd_task();
	}
#endif
}

#if CONFIG_FPGA  // FPGA TEST ONLY

#if !CONFIG_BOOT_TRAP_CTRL
#if CONFIG_BOOT_SETMOD_EN
SECTION_SYSSTART_TEXT
int32_t set_test_mode_pg_cmd(int argc, char **argv)
{
	test_mode = 1;
	test_pg_trap_gpio = 1;
	exit_set_mode = 1;
	dbg_printf("<Set test mode PG>\r\n");
	return 0;
}

SECTION_SYSSTART_TEXT
int32_t set_flash_boot_cmd(int argc, char **argv)
{
	exit_set_mode = 1;
	boot_mode = BootFromNORFlash;
	dbg_printf("<Set BootFromNORFlash>\r\n");
	dbg_printf("<Leave Boot set mode shell>\r\n");
	test_mode = 1;
	test_pg_trap_gpio = 0;
	return 0;
}

SECTION_SYSSTART_TEXT
int32_t exit_setmode_cmd(int argc, char **argv)
{
	exit_set_mode = 1;
	boot_mode = BootFromJTAG_FPGALOAD;
	dbg_printf("<Set BootFromJTAG_FPGALOAD>\r\n");
	dbg_printf("<Leave Boot set mode shell>\r\n");
	return 0;
}

SECTION_SYSSTART_TEXT
void boot_setmode_shell(void)
{
	dbg_printf("<Boot set mode shell on-going>\r\n");

#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const cmd_shell_func_stubs_t *pcmd_shell_stubs = symb_ns4s_stubs->pcmd_shell_stubs;

	// Initial command shell
	pcmd_shell_stubs->shell_cmd_task_init();
	pcmd_shell_stubs->shell_rom_cmd_set_prompt();
	pcmd_shell_stubs->shell_set_cmd_list(pcmd_shell_stubs->shell_cmd_hdl, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	pcmd_shell_stubs->shell_unregister_all(pcmd_shell_stubs->shell_cmd_hdl);
	cmd_info[0].shell_program = &set_test_mode_pg_cmd;
	cmd_info[0].shell_command_string = "pg";
	cmd_info[0].shell_command_type = "BOOT";
	cmd_info[0].help_string = "\tenable test mode pg cmd.\r\n";
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, &cmd_info[0]);
	cmd_info[1].shell_program = &exit_setmode_cmd;
	cmd_info[1].shell_command_string = "q";
	cmd_info[1].shell_command_type = "BOOT";
	cmd_info[1].help_string = "\texit.\r\n";
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, &cmd_info[1]);
	cmd_info[2].shell_program = &set_flash_boot_cmd;
	cmd_info[2].shell_command_string = "bff";
	cmd_info[2].shell_command_type = "BOOT";
	cmd_info[2].help_string = "\tset boot from flash.\r\n";
	pcmd_shell_stubs->shell_register(pcmd_shell_stubs->shell_cmd_hdl, &cmd_info[2]);

	while (1) {
		if (1 == exit_set_mode) {
			break;
		}
		pcmd_shell_stubs->shell_task(pcmd_shell_stubs->shell_cmd_hdl);
	}
#else
	shell_cmd_task_init();
	rom_cmd_shell_set_prompt();
	_shell_set_cmd_list(&shell_cmd_hdl_rom, shell_rom_cmd_list, SHELL_ROM_CMD_LIST_SIZE);
	_shell_unregister_all(&shell_cmd_hdl_rom);
	cmd_info[0].shell_program = &set_test_mode_pg_cmd;
	cmd_info[0].shell_command_string = "pg";
	cmd_info[0].shell_command_type = "BOOT";
	cmd_info[0].help_string = "\tenable test mode pg cmd.\r\n";
	_shell_register(&shell_cmd_hdl_rom, &cmd_info[0]);
	cmd_info[1].shell_program = &exit_setmode_cmd;
	cmd_info[1].shell_command_string = "q";
	cmd_info[1].shell_command_type = "BOOT";
	cmd_info[1].help_string = "\texit.\r\n";
	_shell_register(&shell_cmd_hdl_rom, &cmd_info[1]);
	cmd_info[2].shell_program = &set_flash_boot_cmd;
	cmd_info[2].shell_command_string = "bff";
	cmd_info[2].shell_command_type = "BOOT";
	cmd_info[2].help_string = "\tset boot from flash.\r\n";
	_shell_register(&shell_cmd_hdl_rom, &cmd_info[2]);

	while (1) {
		if (1 == exit_set_mode) {
			break;
		}
		shell_cmd_task();
	}
#endif
}
#endif
#endif

#endif  // FPGA TEST ONLY

SECTION_SYSSTART_TEXT
void rom_bss_data_init(void)
{
	uint32_t bss_len;

	// Clear BSS for ROM code
	bss_len = (__rom_bss_end__ - __rom_bss_start__);
	_memset((void *)__rom_bss_start__, 0, bss_len);

	rom_temp_bss_clean_up();
#if !(CONFIG_FPGA && (defined ( __CC_ARM ) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))))
	uint32_t *pSrc, *pDest;
	// For FPGA verification and load image from ELF, the ARMCC has no section for RW data initialization
	// initial ROM .data
	pSrc  = &__rom_etext;
	pDest = &__rom_data_start__;

	for (; pDest < &__rom_data_end__ ;) {
		*pDest++ = *pSrc++;
	}
#endif
}

SECTION_SYSSTART_TEXT
uint8_t chk_normal_mode_pg(void)
{
	char pg_get[3];
	uint8_t chk_fail_sts, pg_ptn_chk;
	uint32_t loopWait;
	uint32_t i;

	if (hal_rtl_uart_readable(&stdio_uart)) {

		loopWait = 500000;
		for (i = 0; i < 3; i++) {
			while ((!_stdio_port_getc(&pg_get[i])) && (loopWait != 0)) {
				loopWait--;
			}
			if (i == 0) {
				pg_ptn_chk = (uint8_t)(pg_get[i] ^ NORMAL_MODE_PG_PTN);
				//dbg_printf ("0 pg_ptn_chk 0x%x\r\n",pg_ptn_chk);
				if (pg_ptn_chk != 0x0) {
					chk_fail_sts++;
				} else {
					chk_fail_sts += 0;
				}
			} else if (i == 1) {
				pg_ptn_chk ^= (uint8_t)pg_get[i];
				//dbg_printf ("1 pg_ptn_chk 0x%x\r\n",pg_ptn_chk);
				if (pg_ptn_chk != NORMAL_MODE_PG_PTN) {
					chk_fail_sts++;
				} else {
					chk_fail_sts += 0;
				}
			} else {
				pg_ptn_chk ^= (uint8_t)pg_get[i];
				//dbg_printf ("2 pg_ptn_chk 0x%x\r\n",pg_ptn_chk);
				if (pg_ptn_chk != 0x0) {
					chk_fail_sts++;
				} else {
					chk_fail_sts += 0;
				}
			}
		}
		//dbg_printf ("chk_normal_mode_pg sts=0x%x\r\n",chk_fail_sts);
		if (chk_fail_sts > 0) {
			return DISABLE;
		} else {
			return ENABLE;
		}
	} else {
		return DISABLE;
	}
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void chk_img_to_ram_direct(uint32_t *pboot_mode)
{
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	if (uboot_frame_handle.ram_start_tbl != 0) {
		PRAM_FUNCTION_START_TABLE pTempRamStartFun;

		pTempRamStartFun = (PRAM_FUNCTION_START_TABLE)uboot_frame_handle.ram_start_tbl;

		// simple signature check
		if (!chk_ram_img_signature(pTempRamStartFun->Signature)) {
			*pboot_mode = BootFromTestModeUART;
			pRamStartFun = (PRAM_FUNCTION_START_TABLE)uboot_frame_handle.ram_start_tbl;
		}
	} else {
		//dbg_printf("uboot_frame_handle.ram_start_table is NULL!!\r\n");
		//*pboot_mode = BootFromFlash;
		*pboot_mode = potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_sel;
	}
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void timer_init_rom(void)
{
	// Start a G-Timer as system tick
	hal_rtl_timer_clock_init(1, ENABLE);
	hal_rtl_timer_group_init(&_timer_group1, 1); // time group 1
	hal_rtl_timer_group_sclk_sel(&_timer_group1, GTimerSClk_4M);   // Group1 Sclk:4M
	hal_rtl_misc_start_systimer(&_system_timer, CONFIG_SYS_TIMER_ID, GTimerCountUp, CONFIG_SYS_TICK_TIME, 1);
#if 0
#if defined(CONFIG_BUILD_SECURE)
	// share the G-Timer with NS
	*symb_ns4s_stubs->ppsystem_timer = &_system_timer;
#endif

#endif
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void pinmux_init_rom(void)
{
	_pinmux_manager.pin_reg_func = hal_rtl_pin_register;
	_pinmux_manager.pin_unreg_func = hal_rtl_pin_unregister;
	_pinmux_manager.pin_mux_cfg_func = hal_rtl_pin_mux_cfg;
	_pinmux_manager.ioport_pwrup_ctrl_func = hal_rtl_pin_pwrup;
	_pinmux_manager.ioport_pwrdn_ctrl_func = hal_rtl_pin_pwrdwn;
	_pinmux_manager.pin_validat_func = NULL;
	_pinmux_manager.ppinmux_reg_rec = NULL;
	hal_rtl_pinmux_manager_init(&_pinmux_manager);
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void wdt_init_rom(void)
{
	// VNDR wdt init setting
	hal_rtl_wdt_reset(0x17);   // If reset, need to clear  AON/ PON/ WLON/ SYSON block
	hal_rtl_wdt_init(10000000);  // 10sec
	hal_rtl_wdt_enable();
}

#if CONFIG_FPGA
// Simulate writing of S/NS Vendor Key to VNDR_S for Secure JTAG Non-Fixed Key Mode

const uint8_t vndr_s_jtag_key[32] = { // secure key; non-fixed key mode
	0xAA, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F

	/*0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF*/
};

const uint8_t vndr_ns_jtag_key[32] = { // non-secure key; non-fixed key mode
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
};

// Start: Declare test functions for Secure JTAG Non-Fixed Key Mode
void non_fixed_secure_jtag_test(void)
{

	// Not going via bootloader; write to VNDR reg directly here for testing purposes.
	hal_rtl_vdr_s_jtag_key_write((uint8_t *)vndr_s_jtag_key);

	// Bypass OTP autoload - manually write to AON register
	HAL_WRITE32(0x40009000, 0xBC, 0x1F4F); // Bit [7:6] = 2'b01 - Password Secure World Enable
}

void non_fixed_non_secure_jtag_test(void)
{

	// Not going via bootloader; write to VNDR reg directly here for testing purposes.
	hal_rtl_vdr_ns_jtag_key_write((uint8_t *)vndr_ns_jtag_key);

	// Bypass OTP autoload - manually write to AON register
	HAL_WRITE32(0x40009000, 0xBC, 0x1DCF); // Bit [9:8] = 2'b01 - Password Non-Secure World Enable
}

void non_fixed_secure_cum_non_secure_jtag_test(void)
{

	// Not going via bootloader; write to VNDR reg directly here for testing purposes.
	hal_rtl_vdr_s_jtag_key_write((uint8_t *)vndr_s_jtag_key);
	hal_rtl_vdr_ns_jtag_key_write((uint8_t *)vndr_ns_jtag_key);

	// Bypass OTP autoload - manually write to AON register
	HAL_WRITE32(0x40009000, 0xBC, 0x1D4F); // Bit [7:6] = 2'b01; Bit [9:8] = 2'b01 - Password Secure/Non-Secure World Enable
}

#endif

#if CONFIG_FPGA
#if CONFIG_BOOT_OTP_SIMU_AUTO_EN
// OTP load simulation related codes only for FPGA

#define OTP_ROM_SIMU_ID_0               (0x35)
#define OTP_ROM_SIMU_ID_1               (0x87)
#define OTP_ROM_SIMU_SIZE               (144)

SECTION_SYSSTART_BSS uint8_t vry_lgl_data_buf[OTP_ROM_SIMU_SIZE];
SECTION_SYSSTART_BSS uint8_t vry_phy_data_buf[OTP_ROM_SIMU_SIZE];

const uint8_t otp_rma0_ptn[RMA0_PTN_SIZE] = {
	0xFE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
};

const uint8_t otp_rotpk_hsh_ptn[3][SB_PBK_HASH_SIZE] = {
	{
		0xae, 0xc1, 0x16, 0x6f, 0xba, 0x2b, 0xfe, 0x8d,
		0x48, 0x71, 0x67, 0xc9, 0x8b, 0x57, 0x8d, 0xa9,
		0x46, 0x4c, 0x01, 0x72, 0xcb, 0x8e, 0x06, 0x4f,
		0x8c, 0xf7, 0x85, 0x30, 0xd1, 0xac, 0x9e, 0x90
	},
	{
		0xb3, 0x8b, 0x61, 0x33, 0x4c, 0x0d, 0x9e, 0xe1,
		0x25, 0xf4, 0x28, 0x05, 0xbd, 0xdd, 0xdd, 0xc8,
		0x91, 0xfe, 0x82, 0x91, 0x96, 0xdf, 0x43, 0xf2,
		0x20, 0x3a, 0x99, 0x32, 0x56, 0x30, 0xf4, 0x1d
	},
	{
		0xae, 0xc1, 0x16, 0x6f, 0xba, 0x2b, 0xfe, 0x8d,
		0x48, 0x71, 0x67, 0xc9, 0x8b, 0x57, 0x8d, 0xa9,
		0x46, 0x4c, 0x01, 0x72, 0xcb, 0x8e, 0x06, 0x4f,
		0x8c, 0xf7, 0x85, 0x30, 0xd1, 0xac, 0x9e, 0x90
	},
};
const uint8_t otp_huk_ptn[3][SB_HUK_SIZE] = {
	{
		0xcd, 0xe3, 0xc1, 0xf5, 0x5f, 0xc1, 0xa7, 0x01,
		0x43, 0x3f, 0xf0, 0x20, 0x34, 0xbd, 0xc9, 0xdc,
		0x6a, 0x1e, 0xda, 0x4f, 0x63, 0x7c, 0x26, 0x7e,
		0xe3, 0x19, 0x32, 0xe8, 0x3a, 0x99, 0x69, 0x5e
	},
	{
		0xc5, 0x6f, 0xbf, 0x14, 0xb7, 0x96, 0x95, 0xf9,
		0x19, 0x81, 0xdc, 0x44, 0x10, 0x54, 0x7e, 0x4f,
		0x5f, 0x54, 0xf9, 0xa9, 0x2c, 0x9c, 0x4d, 0x67,
		0x07, 0x13, 0x19, 0xba, 0x28, 0x9a, 0x01, 0xc7
	},
	{
		0xcd, 0xe3, 0xc1, 0xf5, 0x5f, 0xc1, 0xa7, 0x01,
		0x43, 0x3f, 0xf0, 0x20, 0x34, 0xbd, 0xc9, 0xdc,
		0x6a, 0x1e, 0xda, 0x4f, 0x63, 0x7c, 0x26, 0x7e,
		0xe3, 0x19, 0x32, 0xe8, 0x3a, 0x99, 0x69, 0x5e
	},
};

const uint8_t otp_sec_key_ptn[3][SB_SEC_KEY_SIZE] = {
	{
		0x10, 0x86, 0xb3, 0x32, 0x8e, 0x21, 0xe6, 0xcb,
		0x73, 0xa1, 0x13, 0x77, 0x10, 0x29, 0xa3, 0x91,
		0xa4, 0x08, 0x78, 0x02, 0xc5, 0x8c, 0x73, 0xb4,
		0x01, 0x1b, 0x3a, 0x4a, 0xc6, 0x7f, 0x38, 0x0e
	},
	{
		0x7f, 0x82, 0xe5, 0x2a, 0x00, 0xbb, 0x47, 0x1c,
		0x24, 0xa7, 0xb9, 0x64, 0xb4, 0xfc, 0xeb, 0x7f,
		0x8b, 0x39, 0x2a, 0x19, 0xf8, 0x82, 0xe1, 0x7f,
		0x64, 0xab, 0x60, 0x53, 0x35, 0x93, 0xdd, 0x83
	},
	{
		0x10, 0x86, 0xb3, 0x32, 0x8e, 0x21, 0xe6, 0xcb,
		0x73, 0xa1, 0x13, 0x77, 0x10, 0x29, 0xa3, 0x91,
		0xa4, 0x08, 0x78, 0x02, 0xc5, 0x8c, 0x73, 0xb4,
		0x01, 0x1b, 0x3a, 0x4a, 0xc6, 0x7f, 0x38, 0x0e
	},
};

const uint8_t otp_s_jtag_key_ptn[2][SB_SJTAG_KEY_SIZE] = { // secure key 1; fixed key mode // NOTE!! starting is 0xDD NOTE!!
	{
		0xDD, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
	},
	{
		0xDE, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
	}
};

const uint8_t otp_ns_jtag_key_ptn[2][SB_SJTAG_KEY_SIZE] = { // non-secure key 1; fixed key mode // NOTE!! ending is 0x40 NOTE!!
	{
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
		0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
		0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x40
	},
	{
		0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
		0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
		0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
		0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x41
	}
};

const uint8_t otp_adc_param_ptn[10] = {
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0x87,
	0x3B, 0x5A
};


SECTION_SYSSTART_TEXT
void otp_rom_simu_dump_map(uint8_t *mem_buf)
{
	int j;
	dbg_printf("------------- Dump otp_map -------------\r\n");
	for (j = 0; j < OTP_ROM_SIMU_SIZE; j = j + 16) {
		dbg_printf("[addr:0x%03x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				   j, mem_buf[0 + j], mem_buf[1 + j], mem_buf[2 + j], mem_buf[3 + j], mem_buf[4 + j], mem_buf[5 + j], mem_buf[6 + j], mem_buf[7 + j],
				   mem_buf[8 + j], mem_buf[9 + j], mem_buf[10 + j], mem_buf[11 + j], mem_buf[12 + j], mem_buf[13 + j],
				   mem_buf[14 + j], mem_buf[15 + j]);
	}
	dbg_printf("\r\n");
}

SECTION_SYSSTART_TEXT
void otp_rom_simu_key_dump(uint32_t otp_addr, uint8_t *mem_buf)
{
	int j;
	dbg_printf("------------- Dump key -------------\r\n");
	for (j = 0; j < 32; j = j + 16) {
		dbg_printf("[addr:0x%03x] %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
				   (otp_addr + j), mem_buf[0 + j], mem_buf[1 + j], mem_buf[2 + j], mem_buf[3 + j], mem_buf[4 + j], mem_buf[5 + j], mem_buf[6 + j], mem_buf[7 + j],
				   mem_buf[8 + j], mem_buf[9 + j], mem_buf[10 + j], mem_buf[11 + j], mem_buf[12 + j], mem_buf[13 + j],
				   mem_buf[14 + j], mem_buf[15 + j]);
	}
	dbg_printf("\r\n");
}


SECTION_SYSSTART_TEXT
int32_t otp_rom_simu_fill_lgl_map(uint8_t *lgl_map_buf, uint32_t lgl_addr, uint8_t data)
{
	if (lgl_addr > 0x7FF) {
		return -1;
	} else if ((lgl_addr >= 0x000) && (lgl_addr <= 0x003)) {
		return 0;
	} else {
		if (((lgl_addr == 0x004) && (data != OTP_ROM_SIMU_ID_0)) ||
			((lgl_addr == 0x005) && (data != OTP_ROM_SIMU_ID_1))) {
			return -1;
		} else {
			lgl_map_buf[lgl_addr] = data;
			return 0;
		}
	}
}

SECTION_SYSSTART_TEXT
int32_t otp_rom_simu_lgl_buf_to_phy_eny(uint32_t strt_addr, uint8_t *lgl_map_buf, uint8_t *otp_mem_buf)
{
	/* log_map_buf should be a continuous 8 bytes buffer */
	uint32_t tmp_cnt;
	uint32_t lgl_off;
	uint8_t word_en;
	int32_t phy_eny_len;

	word_en = 0;
	phy_eny_len = 0;
	lgl_off = strt_addr / OTP_LGL_FULL_ENY_DAT_LEN;
	for (tmp_cnt = 0; tmp_cnt < OTP_LGL_WE_NUM; tmp_cnt++) {
		if ((*(lgl_map_buf + tmp_cnt * OTP_LGL_ONE_DAT_LEN) == 0xFF) &&
			(*(lgl_map_buf + tmp_cnt * OTP_LGL_ONE_DAT_LEN + 1) == 0xFF)) {
			// set WE to 1
			word_en |= (BIT0 << tmp_cnt);
		} else {
			phy_eny_len++;
		}
	}
	if ((word_en & 0xF) != 0xF) {
		/* create entry header prefix */
		*otp_mem_buf = (OTP_LGL_PFX_VAL << OTP_SHIFT_LGL_PFX);
		*otp_mem_buf |= ((lgl_off & OTP_LGL_OFFH_MSK) >> OTP_SHIFT_LGL_OFFH);
		*(otp_mem_buf + OTP_LGL_ENY_HDR0_OFF) = ((lgl_off & OTP_LGL_OFFL_MSK) << OTP_SHIFT_LGL_OFFL);

		/* use lgl_off to record otp mem data pointer */
		lgl_off = 0;
		*(otp_mem_buf + OTP_LGL_ENY_HDR0_OFF) |= word_en;
		for (tmp_cnt = 0; tmp_cnt < OTP_LGL_WE_NUM; tmp_cnt++) {
			if ((word_en & (BIT0 << tmp_cnt)) == 0) {
				/* copy data from logical to otp physical memory buffer */
				*(otp_mem_buf + OTP_LGL_ENY_DATA_OFF + lgl_off * OTP_LGL_ONE_DAT_LEN) =
					*(lgl_map_buf + tmp_cnt * OTP_LGL_ONE_DAT_LEN);
				*(otp_mem_buf + OTP_LGL_ENY_DATA_OFF + lgl_off * OTP_LGL_ONE_DAT_LEN + 1) =
					*(lgl_map_buf + tmp_cnt * OTP_LGL_ONE_DAT_LEN + 1);
				lgl_off++;
			}
		}
		/* calculate physical entry length and return it */
		phy_eny_len = OTP_LGL_ENY_DATA_OFF + phy_eny_len * OTP_LGL_ONE_DAT_LEN;
	}
	return phy_eny_len;
}

SECTION_SYSSTART_TEXT
int32_t otp_rom_simu_trans_lgl_map_to_otp_phy_map(uint8_t *lgl_map_buf, uint8_t *phy_map_buf)
{
	uint32_t tmp_cnt;
	uint32_t phy_idx;   /* logical index */
	int32_t phy_eny_len;   /* physical index */
	int32_t total_phy_len;
	uint8_t phy_eny_buf_tmp[OTP_LGL_ENTY_LEN];

	total_phy_len = 0;
	phy_idx = OTP_PHY_RSVD_LEN;
	for (tmp_cnt = 0; tmp_cnt < OTP_ROM_SIMU_SIZE; tmp_cnt += OTP_LGL_FULL_ENY_DAT_LEN) {
		_memset(&phy_eny_buf_tmp[0], 0xFF, OTP_LGL_ENTY_LEN);

		phy_eny_len = otp_rom_simu_lgl_buf_to_phy_eny(tmp_cnt, (lgl_map_buf + tmp_cnt), &phy_eny_buf_tmp[0]);
		/* copy physical entry into physical map buffer */
		//copy data to physical map
		_memcpy((phy_map_buf + phy_idx), &phy_eny_buf_tmp[0], phy_eny_len);
		phy_idx += phy_eny_len;
		total_phy_len += phy_eny_len;
	}
	return total_phy_len;
}

SECTION_SYSSTART_TEXT
void otp_pg_rma0_pattern()
{
	uint32_t rma_ptn_buf[RMA0_PTN_SIZE];
	uint8_t *p_ptn_buf = NULL, *p_high_val_ptn = NULL;;
	uint32_t otp_idx, rma_high_val_addr = RMA0_PTN_BASE_ADDR;
	uint8_t otp_not_burn = 0, rma_high_val_size = RMA0_PTN_SIZE;

	p_ptn_buf = &rma_ptn_buf[0];
	_memset(p_ptn_buf, 0xFF, rma_high_val_size);
	p_high_val_ptn = &otp_rma0_ptn[0];

	// Get High value assets from OTP
	for (otp_idx = 0; otp_idx < rma_high_val_size; otp_idx++) {
		p_ptn_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((rma_high_val_addr + otp_idx));
		if ((p_ptn_buf[otp_idx] != p_high_val_ptn[otp_idx]) && (p_ptn_buf[otp_idx] == 0xFF)) {
			otp_not_burn++;
		}
	}

	if ((rma_high_val_size / 2) < otp_not_burn) {
		for (otp_idx = 0; otp_idx < rma_high_val_size; otp_idx++) {
			//hal_rtl_otp_byte_wr_syss((rma_high_val_addr + otp_idx), p_high_val_ptn[otp_idx]);
			hal_rtl_otp_byte_wr_marr_syss((rma_high_val_addr + otp_idx), p_high_val_ptn[otp_idx], DISABLE);
		}
	}
}

SECTION_SYSSTART_TEXT
void otp_pg_key_patterns(uint8_t sb_high_val_obj)
{
	uint32_t otp_idx, i;
	uint8_t otp_not_burn = 0, idx_start, idx_end, sb_high_val_size;
	uint32_t sb_high_val_addr, sb_high_val_base, sb_high_val_rma_base;
	uint32_t key_buf[32];
	uint8_t *p_key_buf = NULL, *p_high_val_ptn = NULL;

	switch (sb_high_val_obj) {
	case SB_HIGH_VAL_RTOPK_HSH:
		idx_start = SB_HIGH_VAL_IDX1;
		idx_end   = SB_HIGH_VAL_RMA_IDX;
		sb_high_val_base     = ROTPK_HSH_OTP_BASE_ADDR;
		sb_high_val_rma_base = ROTPK_HSH_RMA_OTP_BASE_ADDR;
		sb_high_val_size     = SB_PBK_HASH_SIZE;
		p_high_val_ptn       = &otp_rotpk_hsh_ptn[0][0];
		break;
	case SB_HIGH_VAL_HUK:
		idx_start = SB_HIGH_VAL_IDX1;
		idx_end   = SB_HIGH_VAL_RMA_IDX;
		sb_high_val_base     = HUK_OTP_BASE_ADDR;
		sb_high_val_rma_base = HUK_RMA_OTP_BASE_ADDR;
		sb_high_val_size     = SB_HUK_SIZE;
		p_high_val_ptn       = &otp_huk_ptn[0][0];
		break;
	case SB_HIGH_VAL_SEC_KEY:
		idx_start = SB_HIGH_VAL_IDX1;
		idx_end   = SB_HIGH_VAL_RMA_IDX;
		sb_high_val_base     = SEC_KEY_OTP_BASE_ADDR;
		sb_high_val_rma_base = SEC_KEY_RMA_OTP_BASE_ADDR;
		sb_high_val_size     = SB_SEC_KEY_SIZE;
		p_high_val_ptn       = &otp_sec_key_ptn[0][0];
		break;
	case SB_HIGH_VAL_SJTAG_S_KEY:
		idx_start = SB_HIGH_VAL_IDX1;
		idx_end   = SB_HIGH_VAL_IDX2;
		sb_high_val_base     = SJTAG_S_KEY_OTP_BASE_ADDR;
		sb_high_val_rma_base = 0x0;
		sb_high_val_size     = SB_SJTAG_KEY_SIZE;
		p_high_val_ptn       = &otp_s_jtag_key_ptn[0][0];
		break;
	case SB_HIGH_VAL_SJTAG_NS_KEY:
		idx_start = SB_HIGH_VAL_IDX1;
		idx_end   = SB_HIGH_VAL_IDX2;
		sb_high_val_base     = SJTAG_NS_KEY_OTP_BASE_ADDR;
		sb_high_val_rma_base = 0x0;
		sb_high_val_size     = SB_SJTAG_KEY_SIZE;
		p_high_val_ptn       = &otp_s_jtag_key_ptn[0][0];
		break;
	default:
		goto otp_pg_key_patterns_end;
		break;
	}

	for (i = idx_start; i <= idx_end; i++) {
		otp_not_burn = 0;
		if (SB_HIGH_VAL_IDX1 == i) {
			sb_high_val_addr = (sb_high_val_base + 0x0);
		} else if (SB_HIGH_VAL_IDX2 == i) {
			sb_high_val_addr = (sb_high_val_base + sb_high_val_size);
		} else {
			sb_high_val_addr = sb_high_val_rma_base;
		}
		p_key_buf = &key_buf[0];
		_memset(p_key_buf, 0xFF, sb_high_val_size);
		// Get High value assets from OTP
		for (otp_idx = 0; otp_idx < sb_high_val_size; otp_idx++) {
			p_key_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((sb_high_val_addr + otp_idx));
			if ((p_key_buf[otp_idx] != p_high_val_ptn[((i - 1)*sb_high_val_size) + otp_idx]) && (p_key_buf[otp_idx] == 0xFF)) {
				otp_not_burn++;
			}
		}

		if ((sb_high_val_size / 2) < otp_not_burn) {
			for (otp_idx = 0; otp_idx < sb_high_val_size; otp_idx++) {
				//hal_rtl_otp_byte_wr_syss((sb_high_val_addr + otp_idx), p_high_val_ptn[((i - 1)*sb_high_val_size) + otp_idx]);
				hal_rtl_otp_byte_wr_marr_syss((sb_high_val_addr + otp_idx), p_high_val_ptn[((i - 1)*sb_high_val_size) + otp_idx], DISABLE);
			}
		}
#if 0
		// Dbg otp High value assets
		for (otp_idx = 0; otp_idx < sb_high_val_size; otp_idx++) {
			p_key_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((sb_high_val_addr + otp_idx));
		}
		otp_rom_simu_key_dump(sb_high_val_addr, p_key_buf);
#endif
	}
otp_pg_key_patterns_end:
	return;
}

SECTION_SYSSTART_TEXT
void otp_adc_param_patterns()
{
	uint32_t otp_idx, i;
	uint8_t otp_not_burn = 0;
	uint32_t adc_param_buf[10];
	uint8_t *p_adc_param_buf = NULL, *p_high_val_ptn = NULL;;
	p_high_val_ptn = &otp_adc_param_ptn[0];
	p_adc_param_buf = &adc_param_buf[0];
	_memset(p_adc_param_buf, 0xFF, 10);
	for (otp_idx = 0; otp_idx < 10; otp_idx++) {
		p_adc_param_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((HP_ADC_GAIN_DENO_ADDR + otp_idx));
		if ((p_adc_param_buf[otp_idx] != p_high_val_ptn[otp_idx]) && (p_adc_param_buf[otp_idx] == 0xFF)) {
			otp_not_burn++;
		}
	}

	if ((10 / 2) < otp_not_burn) {
		for (otp_idx = 0; otp_idx < 10; otp_idx++) {
			hal_rtl_otp_byte_wr_marr_syss((HP_ADC_GAIN_DENO_ADDR + otp_idx), p_high_val_ptn[otp_idx], DISABLE);
		}
	}
}

SECTION_SYSSTART_TEXT
void otp_load_simulation_rom(uint8_t *p_autoload_not_set)
{
	otp_boot_cfg1_t otp_boot_cfg1_info;
	otp_boot_cfg2_t otp_boot_cfg2_info;
	otp_boot_cfg3_t otp_boot_cfg3_info;
	otp_boot_cfg4_t otp_boot_cfg4_info;
	uint8_t *vry_lgl_data = &vry_lgl_data_buf[0];
	uint8_t *vry_phy_data = &vry_phy_data_buf[0];
	uint8_t otp_val = 0xFF, otp_not_burn = 0;
	uint32_t otp_idx, i;
	int32_t total_phy_burn_len;

	_memset(vry_lgl_data, 0xFF, OTP_ROM_SIMU_SIZE);
	_memset(vry_phy_data, 0xFF, OTP_ROM_SIMU_SIZE);

	/* fill in OTP logical map */
	otp_rom_simu_fill_lgl_map(vry_lgl_data,  0x4, OTP_ROM_SIMU_ID_0);
	otp_rom_simu_fill_lgl_map(vry_lgl_data,  0x5, OTP_ROM_SIMU_ID_1);

	otp_boot_cfg1_info.byte = 0;
	otp_boot_cfg1_info.bit.boot_comm.boot_sel = BootFromNORFlash;
	//otp_boot_cfg1_info.bit.boot_comm.boot_sel = BootFromUART;
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg1_LOGL_Offset, otp_boot_cfg1_info.byte);

	otp_boot_cfg2_info.byte = 0;
	otp_boot_cfg2_info.bit.boot_rom_dbg_msg_dis = BootDbgMsgOn;
	otp_boot_cfg2_info.bit.rom_log_port_sel = BootDbgUART1;
	otp_boot_cfg2_info.bit.rom_log_rx_pin_sel = BootUART_S0;
	otp_boot_cfg2_info.bit.rom_log_tx_pin_sel = BootUART_S0;
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg2_LOGL_Offset, otp_boot_cfg2_info.byte);

	otp_boot_cfg3_info.byte = 0;
#if LOAD_FLAH_IMG_NEW_FORMAT_IMGHSH_EN
	otp_boot_cfg3_info.bit.img_hsh_en = ENABLE;
#else
	otp_boot_cfg3_info.bit.img_hsh_en = DISABLE;
#endif

#if LOAD_FLAH_IMG_NEW_FORMAT_TB_EN
	otp_boot_cfg3_info.bit.tb_en = ENABLE;
#else
	otp_boot_cfg3_info.bit.tb_en = DISABLE;
#endif

#if LOAD_FLAH_IMG_NEW_FORMAT_SB_EN
	otp_boot_cfg3_info.bit.sb_en = ENABLE;
#else
	otp_boot_cfg3_info.bit.sb_en = DISABLE;
#endif
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg3_LOGL_Offset, otp_boot_cfg3_info.byte);

	// Debug port select(Default set TM using SW0, S0 for SWD IO pin & CLK pin)
	otp_boot_cfg4_info.byte = 0;
	otp_boot_cfg4_info.bit.dbg_port_mod_sel = DbgPorSWD;
	//otp_boot_cfg4_info.bit.dbg_port_mod_sel = DbgPorJTag;     // secure jtag need to set this
	otp_boot_cfg4_info.bit.dbg_port_tms_io_pin_sel = DbgPort_pin_S0;
	otp_boot_cfg4_info.bit.dbg_port_clk_pin_sel = DbgPort_pin_S0;
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg4_LOGL_Offset, otp_boot_cfg4_info.byte);

#if 1
	otp_boot_cfg5_t otp_boot_cfg5_info;
	// Enable Normal mode SIC
	otp_boot_cfg5_info.byte = 0;
	otp_boot_cfg5_info.bit.boot_normal_sic_en = DISABLE;
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg5_LOGL_Offset, otp_boot_cfg5_info.byte);

	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg6_LOGL_Offset, 0x1E); // A-cut default val
#endif

	otp_boot_cfg7_t otp_boot_cfg7_info;
	// Enable Control NON-TLV IMG format load, fcs ROM flow disable control
	otp_boot_cfg7_info.byte = 0;
	//otp_boot_cfg7_info.bit.rom_dcache_dis = ENABLE;
#if LOAD_FLAH_IMG_NEW_FORMAT
	otp_boot_cfg7_info.bit.ntlv_img_ld_en = DISABLE;
#else
	otp_boot_cfg7_info.bit.ntlv_img_ld_en = ENABLE;
#endif

#if CONFIG_BOOT_LD_VOE_CTRL
	otp_boot_cfg7_info.bit.fcs_rom_flow_dis = DISABLE;
#else
	otp_boot_cfg7_info.bit.fcs_rom_flow_dis = ENABLE;
#endif
	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg7_LOGL_Offset, otp_boot_cfg7_info.byte);

	otp_rom_simu_fill_lgl_map(vry_lgl_data, OTPBootCfg8_LOGL_Offset, 0x0); // A-cut default val

#if 0
	// Dbg otp logical map, make sure UART already init!!!
	dbg_printf("Logical map fill:\r\n");
	otp_rom_simu_dump_map(vry_lgl_data);
#endif
	/* transform from logical to physical OTP map */
	total_phy_burn_len = otp_rom_simu_trans_lgl_map_to_otp_phy_map(vry_lgl_data, vry_phy_data);
#if 0
	// Dbg otp physical map, make sure UART already init!!!
	dbg_printf("Physical map fill:\r\n");
	otp_rom_simu_dump_map(vry_phy_data);
#endif

	//dbg_printf("total_phy_burn_len = %u\r\n",total_phy_burn_len);
	// 1st boot, OTP did not burn value;
	hal_rtl_otp_init();
	otp_val = hal_rtl_otp_byte_rd_syss(0x6);
	if ((otp_val != 0x35) && (otp_val == 0xFF)) {
		otp_not_burn++;
	}
	otp_val = hal_rtl_otp_byte_rd_syss(0x7);
	if ((otp_val != 0x87) && (otp_val == 0xFF)) {
		otp_not_burn++;
	}
	if (otp_not_burn > 0) {
		for (otp_idx = OTP_PHY_RSVD_LEN; otp_idx < OTP_PHY_RSVD_LEN + total_phy_burn_len; otp_idx++) {
			//hal_rtl_otp_byte_wr_syss(otp_idx, vry_phy_data[otp_idx]);
			hal_rtl_otp_byte_wr_marr_syss(otp_idx, vry_phy_data[otp_idx], DISABLE);
		}
		*p_autoload_not_set = ENABLE;
	}

	// write rma0 ptn
#if LOAD_RMA0_PTN_EN
	otp_pg_rma0_pattern();
#endif

	// write ROTPK HSH
	otp_pg_key_patterns(SB_HIGH_VAL_RTOPK_HSH);

	// write HUK
	otp_pg_key_patterns(SB_HIGH_VAL_HUK);

	// write SEC Keys
	otp_pg_key_patterns(SB_HIGH_VAL_SEC_KEY);

	// write SJTAG OTP AON Ctrl
#if 0
	otp_not_burn = 0;
	otp_val = hal_rtl_otp_byte_rd_syss(0x0);
	if ((otp_val != 0x5F) && (otp_val == 0xFF)) {
		otp_not_burn++;
	}
	otp_val = hal_rtl_otp_byte_rd_syss(0x1);
	if ((otp_val != 0xFD) && (otp_val == 0xFF)) {
		otp_not_burn++;
	}
	if (otp_not_burn > 0) {
		hal_rtl_otp_byte_wr_marr_syss(0x0, 0x5F, DISABLE);  // 0x5F NF Key S world pw en, 0x7F Fixed Key S world pw en
		hal_rtl_otp_byte_wr_marr_syss(0x1, 0xFD, DISABLE);  // 0xFD NS world pw en
	}

	// write SJTAG Keys
	otp_pg_key_patterns(SB_HIGH_VAL_SJTAG_S_KEY);
	otp_pg_key_patterns(SB_HIGH_VAL_SJTAG_NS_KEY);
#endif

	// write ADC PARAM ptn
	otp_adc_param_patterns(SB_HIGH_VAL_RTOPK_HSH);

	hal_rtl_otp_deinit();
}
#endif

SECTION_SYSSTART_TEXT
__STATIC_INLINE void autold_fail_self_cfg(void)
{
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;

	if (ENABLE != hal_rtl_otp_rd_al_sts(OTPAlAonSts)) {
#if LOAD_FLAH_IMG_NEW_FORMAT_IMGHSH_EN
		potp_boot_cfg->byte.cfg3.bit.img_hsh_en = ENABLE;
#else
		potp_boot_cfg->byte.cfg3.bit.img_hsh_en = DISABLE;
#endif

#if LOAD_FLAH_IMG_NEW_FORMAT_TB_EN
		potp_boot_cfg->byte.cfg3.bit.tb_en = ENABLE;
#else
		potp_boot_cfg->byte.cfg3.bit.tb_en = DISABLE;
#endif

#if LOAD_FLAH_IMG_NEW_FORMAT_SB_EN
		potp_boot_cfg->byte.cfg3.bit.sb_en = ENABLE;
#else
		potp_boot_cfg->byte.cfg3.bit.sb_en = DISABLE;
#endif

#if LOAD_FLAH_IMG_NEW_FORMAT
		potp_boot_cfg7->bit.ntlv_img_ld_en = DISABLE;
#else
		potp_boot_cfg7->bit.ntlv_img_ld_en = ENABLE;
#endif

#if CONFIG_BOOT_LD_VOE_CTRL
		potp_boot_cfg7->bit.fcs_rom_flow_dis = DISABLE;
#else
		potp_boot_cfg7->bit.fcs_rom_flow_dis = ENABLE;
#endif
	}
}
#endif

SECTION_SYSSTART_TEXT
__STATIC_INLINE void otp_pg_rma1_mgn()
{
	uint32_t rma_ptn_buf[RMA1_MGN_PTN_SIZE];
	uint8_t *p_ptn_buf = NULL, *p_high_val_ptn = NULL;;
	uint32_t otp_idx, rma_high_val_addr = RMA1_MGNPTN_BASE_ADDR;
	uint8_t otp_not_burn = 0, rma_high_val_size = RMA1_MGN_PTN_SIZE;

	hal_rtl_otp_init();

	// PG enable
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	hal_rtl_sys_en_rma_mgn_pg();
#endif

	p_ptn_buf = &rma_ptn_buf[0];
	_memset(p_ptn_buf, 0xFF, rma_high_val_size);
	p_high_val_ptn = &otp_rma1_mgn[0];

	// Get High value assets from OTP
	for (otp_idx = 0; otp_idx < rma_high_val_size; otp_idx++) {
		p_ptn_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((rma_high_val_addr + otp_idx));
		if ((p_ptn_buf[otp_idx] != p_high_val_ptn[otp_idx]) && (p_ptn_buf[otp_idx] == 0xFF)) {
			otp_not_burn++;
		}
	}

	if ((rma_high_val_size / 2) < otp_not_burn) {
		for (otp_idx = 0; otp_idx < rma_high_val_size; otp_idx++) {
			//hal_rtl_otp_byte_wr_syss((rma_high_val_addr + otp_idx), p_high_val_ptn[otp_idx]);
			hal_rtl_otp_byte_wr_marr_syss((rma_high_val_addr + otp_idx), p_high_val_ptn[otp_idx], DISABLE);
		}
	}

#if 0
	// dbg rma1 ptn
	for (otp_idx = 0; otp_idx < rma_high_val_size; otp_idx++) {
		p_ptn_buf[otp_idx] = hal_rtl_otp_byte_rd_syss((rma_high_val_addr + otp_idx));
		dbg_printf("0x%x ", *(p_ptn_buf + otp_idx));
	}
	dbg_printf("\r\n");
#endif

	hal_rtl_otp_deinit();
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t chk_op_rma_flow()
{
	uint8_t rma_state = RMA_NORMAL_STATE;
	int vrf_sts = FAIL;
	// Get RMA state
#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
	rma_state = hal_rtl_sys_get_rma_state();
#endif

	if (RMA_NORMAL_STATE == rma_state) {
		goto chk_op_rma_flow_end;
	} else if (RMA_STATE0 == rma_state) {
		/*
		    prepare enter rma1
		    1. UART init check (todo)
		    2. WDT all disable (todo)
		    3. Wait for get pbk ptn (todo)

		    go enter rma1
		    1. use pbk ptn vrf related rma keys signature (todo)
		    2. write enable & pg mgn ptn (done)
		    3. WDT enable [option](todo)
		*/
		dbg_printf("[RMA_STATE0]\r\n");
		// wdt(AON/VNDR) disable
		hal_rtl_wdt_ctrl_all_disable();
		vrf_sts = SUCCESS;
		if (SUCCESS == vrf_sts) {
			otp_pg_rma1_mgn();
		}
		hal_rtl_wdt_ctrl_aon_enable();
	} else if (RMA_STATE1 == rma_state) {
		/*
		    go enter rma2
		    1. chk uuid

		    if (uuid not burn)

		    else (uuid burn, set inv(uuid))

		    final get rma state
		*/
		dbg_printf("[RMA_STATE1]\r\n");
	}

	if ((RMA_STATE1 == rma_state) ||
		(RMA_STATE2 == rma_state)) {
		/*
		    OTP secure & super secure zone lock ctrl enable
		    - SSZ RMA lock
		    - SZ RMA lock
		*/

	}
chk_op_rma_flow_end:
	return rma_state;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void pwr_seq(void)
{
#if 0
	// Power flow workaround
	HAL_WRITE32(0x400090B4, 0x0, 0xFFE);    // Disable WDT --> move to Bootloader
	HAL_WRITE32(0x40009008, 0x0, 0x2);      // Enable PON block (AON_SYS_PON_FEN) from AON --> HW bootflow
	HAL_WRITE32(0x4000982C, 0x0, 0x307);    // Enable UART0 Sclock (SCLK_UART0_EN) & pclock (PCLK_UART0_EN) on PON_REG --> HW bootflow
	// Enable TIMER0 Sclock (SCLK_TIMER0_EN) & pclock (PCLK_TIMER0_EN) on PON_REG
	// Enable UART Bus domain (UART0_BD_EN) on PON_REG
	HAL_WRITE32(0x40009800, 0x0, 0x1FFFF);  // CPU and platform config + enable SYSON from PON --> HW bootflow
#endif
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void rom_info_store(void)
{
	/*
	  AON 0x4000_90C8[7:4] -> Main Number
	  AON 0x4000_90C8[3:0] -> Sub Number
	 */
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;
	reg_val = aon->AON_REG_AON_SYS_INFO1;
	reg_val &= (~AON_MASK_SYSCFG_ROM_INFO);
	aon->AON_REG_AON_SYS_INFO1 = (reg_val | (((ROM_VER_MAIN << 4) & 0xF0) | (ROM_VER_SUB & 0xF)));
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void vendor_init(void)
{
	/*
	    Enable vendor register clock:PON 0x4000_9800[5]
	    Enable vendor registers : PON 0x4000_9800[6]
	 */
	PON_TypeDef *pon = PON;
	volatile uint32_t reg_val;
	reg_val = pon->PON_REG_PON_SYSON_CTRL;
	reg_val |= PON_BIT_VENDOR_CLK_EN;
	pon->PON_REG_PON_SYSON_CTRL = reg_val;

	reg_val = (pon->PON_REG_PON_SYSON_CTRL);
	reg_val |= PON_BIT_VENDOR_REG_EN;
	pon->PON_REG_PON_SYSON_CTRL = reg_val;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void pon_sram_init(void)
{
	/*
	    PON SRAM Normal Mode:PON 0x4000_983C[3:0]
	    OSC 128K LDO ON
	 */
	PON_TypeDef *pon = PON;
	AON_TypeDef *aon = AON;
	pon->PON_REG_PON_BUS_CTRL = 0;
	//aon->AON_REG_AON_OSC32K_CTRL0 &= ~(AON_BIT_POW_OSC32K);
	aon->AON_REG_AON_OSC32K_CTRL0 |= AON_BIT_POW_OSC32K;
	aon->AON_REG_AON_CLK_CTRL |= AON_BIT_REG_CLK_SEL;
	aon->AON_REG_AON_CLK_CTRL |= AON_BIT_INT_CLK_SEL;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint32_t get_test_mode_status(void)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;
	uint8_t test_mode_status;
	reg_val = aon->AON_REG_AON_SYS_INFO1;
	test_mode_status = ((reg_val & AON_BIT_SYSCFG_TRP_TEST_MODE_SEL) >> AON_SHIFT_SYSCFG_TRP_TEST_MODE_SEL);
	return test_mode_status;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t get_pg_trap_status(const hal_gpio_func_stubs_t *pgpio_stubs)
{
	uint8_t test_pg_trap_gpio_status;
	hal_rtl_pinmux_register(PG_TRAP_PIN, PID_GPIO);
	_memset((void *)&test_mode_trap_gpio, 0, sizeof(hal_gpio_adapter_t));
	pgpio_stubs->hal_gpio_init(&test_mode_trap_gpio, PG_TRAP_PIN);
	pgpio_stubs->hal_gpio_set_dir(&test_mode_trap_gpio, GPIO_IN);
	test_pg_trap_gpio_status = pgpio_stubs->hal_gpio_read(&test_mode_trap_gpio);
	pgpio_stubs->hal_gpio_deinit(&test_mode_trap_gpio);
	hal_rtl_pinmux_unregister(PG_TRAP_PIN, PID_GPIO);
	return test_pg_trap_gpio_status;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t get_cpft_sic_trap_status(const hal_gpio_func_stubs_t *pgpio_stubs)
{
	uint8_t test_cpft_sic_trap_gpio_status = 0x0;
	hal_rtl_pinmux_register(CPFT_TRAP_PIN, PID_GPIO);
	_memset((void *)&test_mode_trap_gpio, 0, sizeof(hal_gpio_adapter_t));

	pgpio_stubs->hal_gpio_init(&test_mode_trap_gpio, CPFT_TRAP_PIN);
	pgpio_stubs->hal_gpio_set_dir(&test_mode_trap_gpio, GPIO_IN);
	test_cpft_sic_trap_gpio_status = pgpio_stubs->hal_gpio_read(&test_mode_trap_gpio);
	pgpio_stubs->hal_gpio_deinit(&test_mode_trap_gpio);

	hal_rtl_pinmux_unregister(CPFT_TRAP_PIN, PID_GPIO);
	return test_cpft_sic_trap_gpio_status;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t get_test_mode_trap_status(const hal_gpio_func_stubs_t *pgpio_stubs)
{
	uint8_t test_mode_trap_gpio_status = 0x0;
	hal_rtl_pinmux_register(PIN_A5, PID_GPIO);
	_memset((void *)&test_mode_trap_gpio, 0, sizeof(hal_gpio_adapter_t));

	pgpio_stubs->hal_gpio_init(&test_mode_trap_gpio, PIN_A5);
	pgpio_stubs->hal_gpio_set_dir(&test_mode_trap_gpio, GPIO_IN);
	test_mode_trap_gpio_status = pgpio_stubs->hal_gpio_read(&test_mode_trap_gpio);
	pgpio_stubs->hal_gpio_deinit(&test_mode_trap_gpio);

	hal_rtl_pinmux_unregister(PIN_A5, PID_GPIO);
	return test_mode_trap_gpio_status;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t get_autoload_trap_status(const hal_gpio_func_stubs_t *pgpio_stubs)
{
	uint8_t test_mode_trap_gpio_status = 0x0;
	hal_rtl_pinmux_register(PIN_A4, PID_GPIO);
	_memset((void *)&test_mode_trap_gpio, 0, sizeof(hal_gpio_adapter_t));

	pgpio_stubs->hal_gpio_init(&test_mode_trap_gpio, PIN_A4);
	pgpio_stubs->hal_gpio_set_dir(&test_mode_trap_gpio, GPIO_IN);
	test_mode_trap_gpio_status = pgpio_stubs->hal_gpio_read(&test_mode_trap_gpio);
	pgpio_stubs->hal_gpio_deinit(&test_mode_trap_gpio);

	hal_rtl_pinmux_unregister(PIN_A4, PID_GPIO);
	return test_mode_trap_gpio_status;
}


SECTION_SYSSTART_TEXT
__STATIC_INLINE uint8_t get_test_mode_misc_option_trap_status(const hal_gpio_func_stubs_t *pgpio_stubs)
{
	const uint32_t tst_gpio_pins_qfn[(TST_GPIO_PIN_NUM + 1)] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		PIN_D11, PIN_D13, PIN_E5, PIN_E6, 0xFF
#else
		PIN_E3, PIN_E4, PIN_F5, PIN_E0, 0xFF
#endif
	};
	uint32_t opt_pin_idx;
	const uint32_t *ptst_opt_pin;
	uint32_t tst_opt_pin_num;
	tst_opt_pin_num = TST_GPIO_PIN_NUM;
	ptst_opt_pin = tst_gpio_pins_qfn;
	uint8_t test_gpio_port_status;

	for (opt_pin_idx = 0; opt_pin_idx < tst_opt_pin_num; opt_pin_idx++) {
		if (ptst_opt_pin[opt_pin_idx] == 0xFF) {
			break;
		}
		hal_rtl_pinmux_register(ptst_opt_pin[opt_pin_idx], PID_GPIO);
		_memset((void *)&test_mode_trap_gpio, 0, sizeof(hal_gpio_adapter_t));
		pgpio_stubs->hal_gpio_init(&test_mode_trap_gpio, ptst_opt_pin[opt_pin_idx]);
		pgpio_stubs->hal_gpio_set_dir(&test_mode_trap_gpio, GPIO_IN);
		test_gpio_port_status = (test_gpio_port_status | (pgpio_stubs->hal_gpio_read(&test_mode_trap_gpio) << opt_pin_idx));
		pgpio_stubs->hal_gpio_deinit(&test_mode_trap_gpio);
		hal_rtl_pinmux_unregister(ptst_opt_pin[opt_pin_idx], PID_GPIO);
	}
	return test_gpio_port_status;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void test_mode_pinmux_init(void)
{
	AON_TypeDef *aon = AON;
	// Disable pinmux limit in testmode(0x400090B0[31:0]=0)
	aon->AON_REG_AON_TEST_MUX_CTRL = 0x0;
}

SECTION_SYSSTART_TEXT
__STATIC_INLINE void test_mode_sic_pinmux_init(void)
{
	AON_TypeDef *aon = AON;
	volatile uint32_t reg_val;

	// SIC Enable
	reg_val = aon->AON_REG_AON_FUNC_CTRL;
	reg_val |= AON_BIT_SYS_FEN_SIC;

	// SIC pinmux setting
	// it pins conflict with UART0
	uint8_t uart_idx = stdio_uart.uart_idx;
	if ((Uart0 == uart_idx) && (stdio_uart.is_inited)) {
		hal_rtl_gpio_pull_ctrl(dbg_uart_rx_pin, Pin_PullNone);
		hal_rtl_pinmux_unregister(stdio_uart.rx_pin, (PID_UART0 + uart_idx));
		hal_rtl_pinmux_unregister(stdio_uart.tx_pin, (PID_UART0 + uart_idx));
	}
	// 0x40009098[3:0]=1 ; 0x40009098[19:16]=1
	hal_rtl_pinmux_register(PIN_SIC_SCL, PID_SIC);
	hal_rtl_pinmux_register(PIN_SIC_SDA, PID_SIC);
	//dbg_printf("0x40009098=0x%x\r\n",HAL_READ32(0x40009098,0x0));
}


SECTION_SYSSTART_TEXT
__STATIC_INLINE void enable_tm_dbg_port(void)
{
	AON_TypeDef *aon = AON;
	SYSON_TypeDef *syson = SYSON;
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg4_t dbg_port_cfg;
	volatile uint32_t reg_val;
	uint8_t s_jtag_swd_sts = 0x0, ns_jtag_swd_sts = 0x0;
	hal_jtag_pin_t _jtag_pins;
	volatile uint8_t dbg_port_mod_sel;

	//hal_rtl_otp_init();
	//hal_rtl_otpc_ctrl(ENABLE, ENABLE, ENABLE); // Enable OTP controller, RW clocks
#if 0
	//NON-Fixed SKEY load
	HAL_WRITE32(0x50002C00, 0x30, 0x1C1D1E1F);
	HAL_WRITE32(0x50002C00, 0x34, 0x18191A1B);
	HAL_WRITE32(0x50002C00, 0x38, 0x14151617);
	HAL_WRITE32(0x50002C00, 0x3C, 0x10111213);

	HAL_WRITE32(0x50002C00, 0x40, 0x0C0D0E0F);
	HAL_WRITE32(0x50002C00, 0x44, 0x08090A0B);
	HAL_WRITE32(0x50002C00, 0x48, 0x04050607);
	HAL_WRITE32(0x50002C00, 0x4C, 0xDD010203);
	// Set SJTAG Secure unlock wait signal
	HAL_WRITE32(0x50002C00, 0x70, 0x1);
#endif

	//DBG_PORT status check
	reg_val = aon->AON_REG_AON_ATLOAD_CTRL1;
	s_jtag_swd_sts  = ((reg_val & AON_MASK_S_JTAG_SWD_MODE) >> AON_SHIFT_S_JTAG_SWD_MODE);
	ns_jtag_swd_sts = ((reg_val & AON_MASK_NS_JTAG_SWD_MODE) >> AON_SHIFT_NS_JTAG_SWD_MODE);

	if ((s_jtag_swd_sts != 0) ||
		(ns_jtag_swd_sts != 0)) {
		// either secure or non-secure JTAG/SWD is enabled

		dbg_port_mod_sel = potp_boot_cfg->byte.cfg4.bit.dbg_port_mod_sel;

		/*
		    JTAG/SWD Control Handles
		*/
		if (dbg_port_mod_sel == DbgPorSWD) {
			// SWD
			reg_val = 0x0;
			// DD confirm TM9 SWD/JTAG select is inside detect, do not need to set pin enable to DbgPorSWD
			reg_val |= (SYSON_BIT_RM5_DBG_PIN_EN | (DbgPorJTag << SYSON_SHIFT_RM5_DBG_MD_SEL) | (DISABLE << SYSON_SHIFT_DBG_CHAIN_EN));
			reg_val |= (DISABLE << SYSON_SHIFT_RM3_DBG_PIN_EN);
			reg_val |= (DbgPorSWD << SYSON_SHIFT_RM3_DBG_MD_SEL);
			syson->SYSON_REG_SYS_JTAG_SWD_CTRL = reg_val;
		} else {
			// JTAG
			reg_val = 0x0;
			reg_val |= (SYSON_BIT_RM5_DBG_PIN_EN | (DbgPorJTag << SYSON_SHIFT_RM5_DBG_MD_SEL) | (DISABLE << SYSON_SHIFT_DBG_CHAIN_EN));
			reg_val |= (DISABLE << SYSON_SHIFT_RM3_DBG_PIN_EN);
			reg_val |= (DbgPorSWD << SYSON_SHIFT_RM3_DBG_MD_SEL);
			syson->SYSON_REG_SYS_JTAG_SWD_CTRL = reg_val;
		}

		/*
		    JTAG/SWD PINMUX Handles
		*/
		dbg_port_cfg.byte = potp_boot_cfg->byte.cfg4.byte;
		if (dbg_port_mod_sel == DbgPorSWD) {
			// SWD
			_jtag_pins.pin_swdio = _dbg_port_tms_io_pin_sel[dbg_port_cfg.bit.dbg_port_tms_io_pin_sel];
			_jtag_pins.pin_swclk = _dbg_port_clk_pin_sel[dbg_port_cfg.bit.dbg_port_clk_pin_sel];
			hal_rtl_pinmux_register(_jtag_pins.pin_swclk, PID_JTAG);
			hal_rtl_pinmux_register(_jtag_pins.pin_swdio, PID_JTAG);
			hal_rtl_gpio_pull_ctrl(_jtag_pins.pin_swclk, Pin_PullUp);
			hal_rtl_gpio_pull_ctrl(_jtag_pins.pin_swdio, Pin_PullUp);
		} else {
			// JTAG
			uint32_t *pin;
			uint32_t i;
			_jtag_pins.pin_tms  = _dbg_port_tms_io_pin_sel[dbg_port_cfg.bit.dbg_port_tms_io_pin_sel];
			_jtag_pins.pin_tclk = _dbg_port_clk_pin_sel[dbg_port_cfg.bit.dbg_port_clk_pin_sel];
			_jtag_pins.pin_tdo  = _dbg_port_jtag_fixed_pins[0];
			_jtag_pins.pin_tdi  = _dbg_port_jtag_fixed_pins[1];
			_jtag_pins.pin_trst = _dbg_port_jtag_fixed_pins[2];
			pin = (uint32_t *)&_jtag_pins;
			for (i = 0; i < JTAG_PINS_SIZE; i++) {
				hal_rtl_pinmux_register(*(pin + i), PID_JTAG);
				hal_rtl_gpio_pull_ctrl(*(pin + i), Pin_PullUp);
			}
		}

	}
	//hal_rtl_otp_deinit();
}


SECTION_SYSSTART_TEXT
void _start_rtl8735b(void)
{
	uint8_t normal_pg_en;
	uint8_t spic_io_mode = SpicOneIOMode, boot_pin_sel = BootFlash_DualIOS0;
	otp_boot_cfg_t *potp_boot_cfg = otpBootCfg;
	otp_boot_cfg5_t *potp_boot_cfg5 = otpBootCfg5;
	otp_boot_cfg7_t *potp_boot_cfg7 = otpBootCfg7;
	uint8_t rma_state = RMA_NORMAL_STATE;
#if CONFIG_BOOT_TRAP_CTRL
#if (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))
	uint32_t test_mode;
	uint32_t boot_mode = BootFromNORFlash;
	uint8_t test_pg_trap_gpio;
	uint8_t test_gpio_port;

#endif  // end of "#if (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))"
#endif

#if CONFIG_FPGA || CONFIG_PXP
	pwr_seq(); // Workaround for Power sequence not ready
#endif
#if defined(CONFIG_BUILD_SECURE)
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
#endif
	const hal_gpio_func_stubs_t *pgpio_stubs = &hal_gpio_stubs;
	ROM_FOOTPH_INIT(BOOT_INFO_IDX0);
	ROM_FOOTPH_INIT(BOOT_INFO_IDX1);
	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 0);

	// ROM version number
	rom_info_store();

	// Clean BSS before enable D-Cache
	// Clear ROM use BSS and initial ROM DATA
	rom_bss_data_init();
#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))
	rom_sboot_bss_clean_up();
	rom_sboot_export_bss_clean_up();
#endif
	hal_rtl_irq_api_init(NULL);
#if defined(CONFIG_BUILD_SECURE)
	// Initial data,BSS for NS ROM
	if (NULL != symb_ns4s_stubs->startup_ns) {
		((void(*)(void))(symb_ns4s_stubs->startup_ns))();
	}
#endif


	SystemInit();

	// Enable vendor control
	vendor_init();
	// Set PON SRAM to Normal Mode
	pon_sram_init();

#if CONFIG_ROM_ICACHE_EN
	// Enable I-Cache
	rtl_icache_enable();
#endif
	// re-direct vector table to RAM
	hal_rtl_vector_table_init((uint32_t)&__StackTop, ram_vector_table);

#if CONFIG_ROM_DCACHE_EN
	// Enable D-Cache
	if (DISABLE == (potp_boot_cfg7->bit.rom_dcache_dis)) {
		rtl_dcache_enable();
	}
#endif

	ConfigDebugErr = 0;
	ConfigDebugWarn = 0;
	ConfigDebugInfo = 0;

	timer_init_rom();                       // maintain by John
	pinmux_init_rom();

#if CONFIG_FPGA
#if CONFIG_BOOT_OTP_SIMU_AUTO_EN
	uint8_t autoload_not_set = DISABLE;
	// OTP simulation setting value only for FPGA
	otp_load_simulation_rom(&autoload_not_set);
#endif
	autold_fail_self_cfg();
	//Non-Fixed Key Mode Tests here - Select **ONLY ONE** out of 3 tests below for testing by uncommenting the desired function

	//non_fixed_secure_jtag_test();
	//non_fixed_non_secure_jtag_test();
	//non_fixed_secure_cum_non_secure_jtag_test();
#endif

	ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 1);

#if CONFIG_PXP  // PXP
	// Init debug printf
	ConfigDebugErr = 0xFFFFFFFF;
	stdio_uart_port_init();
#else   // FPGA & ASIC

	// Confirm rom dbg msg enable or not
	if (BootDbgMsgOn == (potp_boot_cfg->byte.cfg2.bit.boot_rom_dbg_msg_dis)) {
		// Init debug printf
		ConfigDebugErr = 0xFFFFFFFF;
		stdio_uart_port_init();
	}
#if CONFIG_BOOT_OTP_SIMU_AUTO_EN
	if (ENABLE == autoload_not_set) {
		// Enable VNDR WDT, for autoload AON/ keys to engine
		wdt_init_rom();
		dbg_printf("FPGA autoload fail! Wait for WDT Reset...\r\n");
		while (1);
	}
#endif

#endif

#if CONFIG_PXP & (LOAD_FLAH_IMG_EN == 0)
	pRamStartFun = (PRAM_FUNCTION_START_TABLE) __ram_start_table_start__;
	show_welcome_message();
#else
	// FPGA, asic, (pxp with LOAD_FLAH_IMG_EN ==1)

#if CONFIG_BOOT_TRAP_CTRL
	// CONFIG_BOOT_TRAP_CTRL enable

	// Check operate RMA flow
	rma_state = chk_op_rma_flow();

	test_mode = get_test_mode_status();

	if (test_mode) {
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 2);
		//dbg_printf("trap pin Test mode\r\n");

		// Disable AON WDT, before enter to the test_mode
		hal_rtl_wdt_ctrl_aon_disable();

		test_mode_pinmux_init();     // Disable pinmux testmode, before enter to the test_mode. Default test mode cannot use pinmux

		test_mode_sic_pinmux_init(); // Confirm enter test mode, sic pinmux init

		// Test mode
		// GPIO Group init

		pgpio_stubs->hal_gpio_comm_init(&gpio_comm);

		test_pg_trap_gpio = get_pg_trap_status(pgpio_stubs);
		if (!test_pg_trap_gpio) {
			// Not in Flash PG mode
			// Check CP_FT SW trap pin for SIC
			uint8_t test_cpft_trap_gpio;
			test_cpft_trap_gpio = get_cpft_sic_trap_status(pgpio_stubs);

			// check CP/FT trap pin to park CPU: if E7 == 1, then park CPU here.
			if (test_cpft_trap_gpio == 1) {
				// Make CPU park here to prevent interfer SIC operation
				// Disable CPU via SIC interface to operate
				dbg_printf("CPU park here\r\n");
				while (1);
			}

			// In test mode, read GPIO pins (D11, D13, E5, E6) as the option of test mode
			test_gpio_port = 0;
			test_gpio_port = get_test_mode_misc_option_trap_status(pgpio_stubs);
			//dbg_printf("trap pin option of test mode\r\n");
		} else {
			//dbg_printf("trap pin PG\r\n");
		}
		// GPIO Group deinit
		pgpio_stubs->hal_gpio_comm_deinit();

	} else {
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 3);
		//dbg_printf("trap pin Normal mode\r\n");
		// 1. Enable Vendor wdt, AON wdt default hw already enable
		wdt_init_rom();

		// 2. confirm boot_mode from OTP autoload
		boot_mode = potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_sel;

		/*
		  3. check boot mode pin conflict or not
		*/
	}
#else
	// CONFIG_BOOT_TRAP_CTRL Disable

#if CONFIG_BOOT_SETMOD_EN
	hal_rtl_wdt_ctrl_aon_disable();
	boot_setmode_shell();
#else
	test_mode = 0;
	test_pg_trap_gpio = 0;
#endif  // end of CONFIG_BOOT_SETMOD_EN

	wdt_init_rom();
#endif  // end of CONFIG_BOOT_TRAP_CTRL

	show_welcome_message();
	if (!test_mode) {
		dbg_printf("\r\n[Normal mode]\r\n");
		if (potp_boot_cfg5->bit.boot_normal_sic_en == ENABLE) {
			test_mode_sic_pinmux_init(); // Confirm Normal mode can use SIC, sic pinmux init
		}
		if ((BootDbgMsgOn == (potp_boot_cfg->byte.cfg2.bit.boot_rom_dbg_msg_dis)) &&
			(ENABLE == stdio_uart.is_inited)) {
			normal_pg_en = chk_normal_mode_pg();
		}
		if (ENABLE == normal_pg_en) {
			// Disable AON & VNDRT WDT, before enter to the PG mode
			hal_rtl_wdt_ctrl_all_disable();

			uboot_frame_handle.ram_start_tbl = 0;
			test_mode_img_download();

			// Enable VNDR WDT, after exit to the PG mode
			hal_rtl_wdt_enable();

			// Check if the image is downloaded to RAM directly
			chk_img_to_ram_direct(&boot_mode);
		} else {
#if !CONFIG_BOOT_TRAP_CTRL
			boot_mode = BootFromJTAG_FPGALOAD;
#else
			boot_mode = potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_sel;
			//boot_mode = BootFromNORFlash;
#endif
			ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 4);
		}
	} else {
		// Test mode
		// Confirm otp autoload boot config, because some test mode will triger operate device boot flow
		dbg_printf("Test Mode: boot_cfg1=0x%x\r\n", potp_boot_cfg->byte.cfg1.byte);
		boot_mode    = potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_sel;
		boot_pin_sel = potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_pin_sel;

		if (test_pg_trap_gpio) {

			dbg_printf("\r\n[test mode PG]\r\n");

			uboot_frame_handle.ram_start_tbl = 0;
			test_mode_img_download();

			// Check if the image is downloaded to RAM directly
			chk_img_to_ram_direct(&boot_mode);
		} else {
			dbg_printf("\r\n[MISC option mode]\r\n");
#if IS_CUT_TEST(CONFIG_CHIP_VER)
			dbg_printf("Test Mode GPIO[E6, E5, D13, D11] = 0x%x\r\n", test_gpio_port);
#else
			dbg_printf("Test Mode GPIO[E0, F5, E4, E3] = 0x%x\r\n", test_gpio_port);
#endif
			switch (test_gpio_port) {
			case TEST_MODE_MISC_OPT0:
				dbg_printf("TEST_MODE_MISC_OPT0\r\n");
				break;

			case TEST_MODE_MISC_OPT1:
				dbg_printf("TEST_MODE_MISC_OPT1\r\n");
				break;

			case TEST_MODE_MISC_OPT2:
				dbg_printf("TEST_MODE_MISC_OPT2\r\n");
				break;

			case TEST_MODE_MISC_OPT3:
				dbg_printf("TEST_MODE_MISC_OPT3\r\n");
				break;

			case TEST_MODE_MISC_OPT4:
				dbg_printf("TEST_MODE_MISC_OPT4\r\n");
				break;

			case TEST_MODE_MISC_OPT5:
				dbg_printf("TEST_MODE_MISC_OPT5\r\n");
				break;

			case TEST_MODE_MISC_OPT6:
				dbg_printf("TEST_MODE_MISC_OPT6\r\n");
				break;

			case TEST_MODE_MISC_OPT7:
				dbg_printf("TEST_MODE_MISC_OPT7\r\n");
				break;

			case TEST_MODE_MISC_OPT8:
				dbg_printf("TEST_MODE_MISC_OPT8\r\n");
				break;

			case TEST_MODE_MISC_OPT9:
				dbg_printf("TEST_MODE_MISC_OPT9\r\n");
				break;

			case TEST_MODE_MISC_OPT10:
				dbg_printf("TEST_MODE_MISC_OPT10\r\n");
				break;

			case TEST_MODE_MISC_OPT11:
				dbg_printf("TEST_MODE_MISC_OPT11\r\n");
				break;

			case TEST_MODE_MISC_OPT12:
				dbg_printf("TEST_MODE_MISC_OPT12\r\n");
				break;

			case TEST_MODE_MISC_OPT13:
				dbg_printf("TEST_MODE_MISC_OPT13\r\n");
				// Enable pin-mux for TM JTAG/SWD
				enable_tm_dbg_port();
#if 0
				// Dbg TM JTAG/SWD control and pinmux
				dbg_printf("0x400090BC:0x%x\r\n", HAL_READ32(0x400090BC, 0x0));
				dbg_printf("0x40009094:0x%x\r\n", HAL_READ32(0x40009094, 0x0));
				dbg_printf("0x40000070:0x%x\r\n", HAL_READ32(0x40000070, 0x0));
				dbg_printf("0x4000985C:0x%x\r\n", HAL_READ32(0x4000985C, 0x0));
				dbg_printf("0x40009864:0x%x\r\n", HAL_READ32(0x40009864, 0x0));
				dbg_printf("0x40009868:0x%x\r\n", HAL_READ32(0x40009868, 0x0));
#endif
				// CPU Park here leave JTAG contorl
				while (1);
				break;

			case TEST_MODE_MISC_OPT14:
				dbg_printf("TEST_MODE_MISC_OPT14\r\n");
				boot_command_shell();
				break;

			case TEST_MODE_MISC_OPT15:
				dbg_printf("TEST_MODE_MISC_OPT15\r\n");
				// Note: Check device life cycle state to allow or not
				uboot_frame_handle.ram_start_tbl = 0;
				test_mode_img_download();
				// Check if the image is downloaded to RAM directly
				chk_img_to_ram_direct(&boot_mode);
				break;

			default:
				boot_command_shell();
				break;
			}
			potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_sel = boot_mode;
			potp_boot_cfg->byte.cfg1.bit.boot_comm.boot_pin_sel = boot_pin_sel;
		}
	}

#if CONFIG_FPGA // FPGA only
#if CONFIG_BOOT_DBG_PORT_AUTO_SEL_EN    // if not enable, not chagned default swd/jtag sw control setting(using FPGA JTAG dedicated pins)
	// Enable pin-mux for TM JTAG/SWD using OTP autoload
	enable_tm_dbg_port();
#endif
#else // PXP, ASIC
	// Enable pin-mux for TM JTAG/SWD using OTP autoload
	enable_tm_dbg_port();
#endif

	if (BootFromNORFlash == boot_mode) {
		spic_io_mode = potp_boot_cfg->byte.cfg1.bit.norflash_boot.io_mode_sel;
		boot_pin_sel = potp_boot_cfg->byte.cfg1.bit.norflash_boot.pin_sel;
	}

	if (!fast_reboot_rdy) {
		if (test_mode) {
			// Test mode, after PG or other test mdoe misc options
			// Init VNDR WDT, after exit to the PG or other test mdoe misc options
			wdt_init_rom();

		} else {
			// Normal mode
			// Refresh VNDR WDT
			hal_rtl_wdt_refresh();
		}
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 5);
		if (boot_mode == BootFromNORFlash) {
			hal_status_t ret;
			uint8_t spic_set_io_mode = spic_io_mode;

			dbg_printf("BootFromNORFlash\r\n");
			// Due to Pro2 rom code always using SpicOneIOMode
			spic_set_io_mode = SpicOneIOMode;
			// Due to Pro2 current only has 1 pin select
			boot_pin_sel = BootFlash_DualIOS0;

			ret = fw_spic_init(&_hal_spic_adaptor, spic_set_io_mode, boot_pin_sel);
			if (ret == HAL_OK) {
				ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 7);
				ret = flash_boot(&pRamStartFun);
				if (ret != SUCCESS) {
					pRamStartFun = (PRAM_FUNCTION_START_TABLE)NULL;
				} else {
					if (test_mode) {
						// Enable sys_fen_sic
					}
				}
				flash_rtl_return_spi(&_hal_spic_adaptor);
				fw_spic_deinit(&_hal_spic_adaptor);
				ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 20);
				// Refresh VNDR WDT
				hal_rtl_wdt_refresh();
			} else {
				pRamStartFun = (PRAM_FUNCTION_START_TABLE)NULL;
				DBG_BOOT_ERR("Flash init error (io_mod=%u, pin_sel=%u)\r\n", spic_set_io_mode, boot_pin_sel);
				fw_spic_deinit(&_hal_spic_adaptor);

				// Fallback to SNAND flash
				dbg_printf("Fallback SNAND Flash\r\n");
				boot_mode = BootFromNANDFlash;
				goto NAND_FLASH_BOOT;
			}
		} else if (boot_mode == BootFromNANDFlash) {
			hal_status_t ret;

NAND_FLASH_BOOT:

			dbg_printf("BootFromNANDFlash\r\n");
			// TODO fpga may error
			*(hal_snand_boot_stubs.rom_snand_boot) = 1;

			// FIXME IO mode and Pin is not used by fw_snafc_init
			ret = fw_snafc_init(&_hal_snafc_adaptor, SpicOneIOMode, BootFlash_DualIOS0);

			if (ret == HAL_OK) {
				dbg_printf("snafc init pass\r\n");
				ret = snand_flash_boot(&pRamStartFun);
				if (ret != HAL_OK) {
					pRamStartFun = (PRAM_FUNCTION_START_TABLE) NULL;
				}
				fw_snafc_deinit(&_hal_snafc_adaptor);
				dbg_printf("snafc deinit pass\r\n");
			} else {
				pRamStartFun = (PRAM_FUNCTION_START_TABLE) NULL;
				dbg_printf("snafc init error\r\n");
				fw_snafc_deinit(&_hal_snafc_adaptor);
			}

			// Refresh VNDR WDT
			hal_rtl_wdt_refresh();
#if 0
			dbg_printf("fw_snafc_init freeze at %s:%d\r\n", __FILE__, __LINE__);
			while (1);
#endif

		} else if (boot_mode == BootFromUART) {
			dbg_printf("BootFromUART\r\n");
			// Refresh VNDR WDT
			//hal_rtl_wdt_refresh();
			// Disable AON & VNDRT WDT, before enter to the PG mode
			hal_rtl_wdt_ctrl_all_disable();
			uart_boot(&pRamStartFun);
		} else if (boot_mode == BootFromTestModeUART) {
			dbg_printf("BootFromTestModeUART\r\n");
			// Refresh VNDR WDT
			hal_rtl_wdt_refresh();
		} else if (boot_mode == BootFromJTAG_FPGALOAD) {    // Only for FPGA development load
#if CONFIG_FPGA // FPGA only
			dbg_printf("BootFromJTAG_FPGALOAD\r\n");
			pRamStartFun = (PRAM_FUNCTION_START_TABLE) __ram_start_table_start__;
#endif
		} else {
			DBG_BOOT_ERR("Unknow boot mode(0x%x); otp boot cfg = 0x%x\r\n", boot_mode, potp_boot_cfg->byte.cfg1.byte);
		}
	} else {
		dbg_printf("Fast Reboot to 0x%x\r\n", pRamStartFun->RamStartFun);
	}
#endif // end of CONFIG_PXP & (LOAD_FLAH_IMG_EN == 0)

	// check the RAM image signature
	if (pRamStartFun != NULL && (!chk_ram_img_signature(pRamStartFun->Signature))) {
		RAM_START_FUNCTION ram_start_func;
		if (stdio_uart.is_inited) {
			uint8_t uart_idx;

			ConfigDebugErr = 0;
			uart_idx = stdio_uart.uart_idx;
			hal_rtl_uart_deinit(&stdio_uart);
			_stdio_port_deinit();

			// Todo release UART RX Pin pull control
			hal_rtl_gpio_pull_ctrl(dbg_uart_rx_pin, Pin_PullNone);
			hal_rtl_pinmux_unregister(stdio_uart.rx_pin, (PID_UART0 + uart_idx));
			hal_rtl_pinmux_unregister(stdio_uart.tx_pin, (PID_UART0 + uart_idx));
		}

		// pinmux info backup
		hal_rtl_pinmux_rom_info_manage(&_pinmux_manager, OP_BACKUP, ALL_LOG);

		// Todo vendor wdt disable or refresh
		// Refresh VNDR WDT
		hal_rtl_wdt_refresh();

		// Todo D-cache disable
		rtl_dcache_disable();

		pRamStartFun->boot_cfg_w = potp_boot_cfg->word;

		ram_start_func.RamStartFun = pRamStartFun->RamStartFun;
		ROM_FOOTPH_STORE(BOOT_INFO_IDX0, 21);
		if (ram_start_addr_valid((uint32_t)ram_start_func.RamStartFun)) {
			if ((pRamStartFun->msp_start != 0) && (pRamStartFun->msp_start > pRamStartFun->msp_limit)) {
				// switch the main stack memory as the application defined
				__disable_irq();
				__ISB();
				__set_MSP((uint32_t)pRamStartFun->msp_start);
				__set_MSPLIM((uint32_t)pRamStartFun->msp_limit);
				// !!! Don't access local variable after this line... !!!
				__enable_irq();
				__DSB();
				__ISB();
			}
			ram_start_func.RamStartFun();
		} else {
			goto _start_invalid_start_tbl;
		}
	} else {
_start_invalid_start_tbl:
		if (ConfigDebugErr == 0) {
			stdio_uart_port_init();
			ConfigDebugErr = 0xFFFFFFFF;
		}
#if defined(CONFIG_BUILD_SECURE) && (LOAD_FLAH_IMG_EN || !(CONFIG_FPGA))
		rom_sboot_bss_clean_up();
		rom_sboot_export_bss_clean_up();
#endif
		dbg_printf("StartUp@0x%x: Invalid RAM Img Signature!\r\n", pRamStartFun);
		boot_command_shell();
		while (1);
	}
}

