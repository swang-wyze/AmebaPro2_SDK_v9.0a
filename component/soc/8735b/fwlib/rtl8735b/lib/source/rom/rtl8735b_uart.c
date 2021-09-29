/**************************************************************************//**
 * @file     rtl8735b_uart.c
 * @brief    This file implements the UART IP power, clock, baud rate
 *           configuration functions.
 * @version  V1.00
 * @date     2021-06-29
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation. All rights reserved.
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
#include "rtl8735b_uart_type_unify.h"
#include "rtl8735b.h"

#if CONFIG_UART_EN

#if !defined(CONFIG_BUILD_NONSECURE)
#undef UART0_REG_BASE
#define UART0_REG_BASE (UART0_S_BASE)
#undef UART1_REG_BASE
#define UART1_REG_BASE (UART1_S_BASE)
#undef UART2_REG_BASE
#define UART2_REG_BASE (UART2_S_BASE)
#undef UART3_REG_BASE
#define UART3_REG_BASE (UART3_S_BASE)
#undef UART4_REG_BASE
#define UART4_REG_BASE (UART4_S_BASE)
#else

#undef UART0_REG_BASE
#define UART0_REG_BASE (UART0_BASE)
#undef UART1_REG_BASE
#define UART1_REG_BASE (UART1_BASE)
#undef UART2_REG_BASE
#define UART2_REG_BASE (UART2_BASE)
#undef UART3_REG_BASE
#define UART3_REG_BASE (UART3_BASE)
#undef UART4_REG_BASE
#define UART4_REG_BASE (UART4_BASE)
#endif


#define SECTION_UART_TEXT           SECTION(".rom.hal_uart.text")
#define SECTION_UART_DATA           SECTION(".rom.hal_uart.data")
#define SECTION_UART_RODATA         SECTION(".rom.hal_uart.rodata")
#define SECTION_UART_BSS            SECTION(".rom.hal_uart.bss")
#define SECTION_UART_STUBS          SECTION(".rom.hal_uart.stubs")

extern void *_memset(void *dst0, int Val, SIZE_T length);

void _uart_irq_handler(phal_uart_adapter_t puart_adapter);
void _uart_tx_dma_irq_handler(phal_uart_adapter_t puart_adapter);
void _uart_rx_dma_irq_handler(phal_uart_adapter_t puart_adapter);
void uart_tx_isr(phal_uart_adapter_t puart_adapter);
void uart_rx_isr(phal_uart_adapter_t puart_adapter);
void uart_iir_isr(phal_uart_adapter_t puart_adapter);

/**
  * @brief Baud rate table. List out the pre-defined supported baud rate.
  *        If the baud rate setting is not list in this table, the divisor
  *        value is caculated at run time.
  */
SECTION_UART_RODATA const uint32_t def_baudrate_table[] = {
	110,     300,     600,    1200,
	2400,    4800,    9600,   14400,
	19200,   28800,   38400,   57600,
	76800,  115200,  128000,  153600,
	230400,  380400,  460800,  500000,
	921600, 1000000, 1382400, 1444400,
	1500000, 1843200, 2000000, 2100000,
	2764800, 3000000, 3250000, 3692300,
	3750000, 4000000, 6000000,

	// End of the table
	0xffffffff
};

/**
  * @brief Over sampling table. The over sampling value for pre-defined
  *        baud rate. It is base on the UART0 SCLK = 4M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_4m[] = {
	20, 17, 17, 17,
	17, 17, 13, 12,
	13, 17, 13, 17,
	13, 17, 10, 13,
	17,  5,  8,  8,
	4,  4,  2,  2,
	2,  2,  2,  1,
	1,  1,  1,  1,
	1,  1,  0
};

/**
  * @brief Divisor table. The divisor value for pre-defined
  *        baud rate. It is base on the UART0 SCLK = 4M.
  */
SECTION_UART_RODATA const uint16_t def_div_4m[] = {
	1818,    784,    392,    196,
	98,     49,     32,     23,
	16,      8,      8,      4,
	4,      2,      3,      2,
	1,      2,      1,      1,
	1,      1,      1,      1,
	1,      1,      1,      1,
	1,      1,      1,      1,
	1,      1,      1
};

/**
  * @brief Bit adjustment table index for 10-bits frame.
  *             It is base on the UART0 SCLK = 4M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_10b_4m[] = {
	0,   0,   0,   0,
	0,   0,   0,   1,
	0,   4,   0,   4,
	0,   4,   4,   0,
	4,   2,   7,   0,
	3,   0,   9,   8,
	7,   2,   0,   9,
	4,   3,   2,   1,
	1,   0,   7
};

/**
  * @brief Bit adjustment table index for 9-bits frame.
  *             It is base on the UART0 SCLK = 4M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_9b_4m[] = {
	0,   0,   0,   0,
	0,   0,   0,   1,
	0,   3,   0,   3,
	0,   3,   4,   0,
	3,   2,   6,   0,
	3,   0,   8,   7,
	6,   1,   0,   8,
	4,   3,   2,   1,
	1,   0,   6
};

/**
  * @brief Bit adjustment table index for 8-bits frame.
  *             It is base on the UART0 SCLK = 4M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_8b_4m[] = {
	0,   0,   0,   0,
	0,   0,   0,   1,
	0,   3,   0,   3,
	0,   3,   3,   0,
	3,   2,   5,   0,
	3,   0,   7,   6,
	5,   1,   0,   7,
	3,   3,   2,   1,
	0,   0,   5
};

/**
  * @brief Over sampling table. The over sampling value for pre-defined
  *        baud rate. It is base on the UART SCLK = 26M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_26m[] = {
	20, 20, 20, 20,
	19, 19, 11, 19,
	11, 11, 13, 11,
	13, 15, 10, 13,
	14, 17, 14, 13,
	14, 13,  9, 18,
	17,  7, 13,  6,
	9,  8,  8,  7,
	6,  6,  4
};

/**
  * @brief Divisor table. The divisor value for pre-defined
  *        baud rate. It is base on the UART SCLK = 26M.
  */
SECTION_UART_RODATA const uint16_t def_div_26m[] = {
	11813,  4332,  2166,  1083,
	570,   285,   246,    95,
	123,    82,    52,    41,
	26,    15,    20,    13,
	8,     4,     4,     4,
	2,     2,     2,     1,
	1,     2,     1,     2,
	1,     1,     1,     1,
	1,     1,     1
};

/**
  * @brief Bit adjustment table index for 10-bits frame.
  *             It is base on the UART0 SCLK = 26M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_10b_26m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   0,   2,
	1,   0,   3,   7,
	0,   3,   3,   4,
	3,   2,   3
};

/**
  * @brief Bit adjustment table index for 9-bits frame.
  *             It is base on the UART0 SCLK = 26M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_9b_26m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   0,   3,
	1,   0,   3,   8,
	0,   3,   3,   5,
	3,   2,   3
};

/**
  * @brief Bit adjustment table index for 8-bits frame.
  *             It is base on the UART0 SCLK = 26M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_8b_26m[] = {
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  1,  0,  0,
	2,  0,  3,  0,
	4,  0,  4,  2,
	3,  2,  0,  0,
	2,  3,  1,  3,
	3,  0,  5
};


/**
  * @brief Over sampling table. The over sampling value for pre-defined
  *        baud rate. It is base on the UART0 SCLK = 40M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_40m[] = {
	20, 20, 20, 20,
	20, 17, 17, 15,
	10, 11, 10, 11,
	10, 15, 12, 10,
	10, 15, 17, 20,
	14, 20, 14,  9,
	13,  7, 20, 19,
	7, 13,  6,  5,
	5, 10,  6
};

/**
  * @brief Divisor table. The divisor value for pre-defined
  *        baud rate. It is base on the UART0 SCLK = 40M.
  */
SECTION_UART_RODATA const uint16_t def_div_40m[] = {
	18173,  6664,  3332,  1666,
	833,   490,   245,   185,
	208,   126,   104,    63,
	52,    23,    26,    26,
	17,     7,     5,     4,
	3,     2,     2,     3,
	2,     3,     1,     1,
	2,     1,     2,     2,
	2,     1,     1
};

/**
  * @brief Bit adjustment table index for 10-bits frame.
  *             It is base on the UART0 SCLK = 40M
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_10b_40m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	2,   0,   4,   0,
	5,   0,   5,   2,
	3,   2,   0,   0,
	2,   3,   1,   4,
	3,   0,   7
};

/**
  * @brief Bit adjustment table index for 9-bits frame.
  *             It is base on the UART0 SCLK = 40M
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_9b_40m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	2,   0,   3,   0,
	4,   0,   4,   2,
	3,   2,   0,   0,
	2,   3,   1,   4,
	3,   0,   6
};

/**
  * @brief Bit adjustment table index for 8-bits frame.
  *             It is base on the UART0 SCLK = 40M
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_8b_40m[] = {
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  1,  0,  0,
	2,  0,  3,  0,
	4,  0,  4,  2,
	3,  2,  0,  0,
	2,  3,  1,  3,
	3,  0,  5
};


/**
  * @brief Over sampling table. The over sampling value for pre-defined
  *        baud rate. It is base on the UART1/2/3/4 SCLK = 50M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_50m[] = {
	20, 20, 20, 20,
	19, 19, 19, 16,
	19, 14, 14, 14,
	10, 14, 10, 13,
	12, 10, 12, 20,
	9, 10,  9, 17,
	11,  9,  8, 11,
	9,  8, 15, 13,
	13,  6,  8
};

/**
  * @brief Divisor table. The divisor value for pre-defined
  *        baud rate. It is base on the UART1/2/3/4 SCLK = 50M.
  */
SECTION_UART_RODATA const uint16_t def_div_50m[] = {
	22716,  8330,  4165,  2083,
	1096,   548,   274,   217,
	137,   124,    93,    62,
	65,    31,    39,    25,
	18,    13,     9,     5,
	6,     5,     4,     2,
	3,     3,     3,     2,
	2,     2,     1,     1,
	1,     2,     1
};

/**
  * @brief Bit adjustment table index for 10-bits frame.
  *             It is base on the UART1/2/3/4 SCLK = 50M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_10b_50m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   0,   2,
	1,   0,   3,   7,
	0,   3,   3,   4,
	3,   2,   3
};

/**
  * @brief Bit adjustment table index for 9-bits frame.
  *             It is base on the UART1/2/3/4 SCLK = 50M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_9b_50m[] = {
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   0,   0,   0,
	0,   1,   0,   0,
	0,   0,   0,   3,
	1,   0,   3,   8,
	0,   3,   3,   5,
	3,   2,   3
};

/**
  * @brief Bit adjustment table index for 8-bits frame.
  *             It is base on the UART1/2/3/4 SCLK = 50M.
  */
SECTION_UART_RODATA const uint8_t def_ovsr_adj_bit_8b_50m[] = {
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  0,  0,  0,
	0,  1,  0,  0,
	2,  0,  3,  0,
	4,  0,  4,  2,
	3,  2,  0,  0,
	2,  3,  1,  3,
	3,  0,  5
};


/**
  * @brief Bit adjustment table for 10-bits frame.
  */
SECTION_UART_RODATA const uint16_t ovsr_adj_table_10bit[10] = {
	0x000, 0x020, 0x044, 0x124, 0x294, 0x2AA, 0x16B, 0x2DB, 0x3BB, 0x3EF
};

/**
  * @brief Bit adjustment table for 9-bits frame.
  */
SECTION_UART_RODATA const uint16_t ovsr_adj_table_9bit[9] = {
	0x000, 0x010, 0x044, 0x92, 0xAA, 0x155, 0x1B6, 0x1BB, 0x1EF
};

/**
  * @brief Bit adjustment table for 8-bits frame.
  */
SECTION_UART_RODATA const uint16_t ovsr_adj_table_8bit[8] = {
	0x000, 0x010, 0x044, 0x92, 0xAA, 0xB5, 0xBB, 0xEF
};

/**
  * @brief Pin map of UART RX pin.
  */
SECTION_UART_RODATA const hal_pin_map uart_rx_pin_map[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_A3,  PID_UART0},
	{PIN_F4,  PID_UART1},
	{PIN_F12, PID_UART1},
	{PIN_D16, PID_UART2},
	{PIN_E2,  PID_UART2},
	{PIN_E6,  PID_UART3},

	{PIN_UART4_RX, PID_UART4},

	{0xFF,    0xFF} // end of table
#else
	{PIN_A3,  PID_UART0},
	{PIN_F3,  PID_UART1},
	{PIN_F12, PID_UART1},
	{PIN_D16, PID_UART2},
	{PIN_D20, PID_UART2},
	{PIN_D16, PID_UART2},
	{PIN_E2,  PID_UART3},

	{PIN_UART4_RX, PID_UART4},

	{0xFF,    0xFF} // end of table
#endif
};

/**
  * @brief Pin map of UART TX pin.
  */
SECTION_UART_RODATA const hal_pin_map uart_tx_pin_map[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_A2,  PID_UART0},
	{PIN_F5,  PID_UART1},
	{PIN_F13, PID_UART1},
	{PIN_D15, PID_UART2},
	{PIN_E1,  PID_UART2},
	{PIN_E5,  PID_UART3},

	{PIN_UART4_TX, PID_UART4},

	{0xFF,    0xFF} // end of table
#else
	{PIN_A2,  PID_UART0},
	{PIN_F4,  PID_UART1},
	{PIN_F13, PID_UART1},
	{PIN_D15, PID_UART2},
	{PIN_D19, PID_UART2},
	{PIN_E0,  PID_UART2},
	{PIN_E1,  PID_UART3},

	{PIN_UART4_TX, PID_UART4},

	{0xFF,    0xFF} // end of table
#endif
};

/**
  * @brief Pin map of UART RTS pin.
  */
SECTION_UART_RODATA const hal_pin_map uart_rts_pin_map[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_NC,  PID_UART0},//UART0 NONE
	{PIN_F3,  PID_UART1},
	{PIN_E4,  PID_UART2},
	{PIN_E7,  PID_UART3},

	{PIN_UART4_RTS, PID_UART4},

	{0xFF,    0xFF} // end of table
#else
	{PIN_NC,  PID_UART0},//UART0 NONE
	{PIN_F2,  PID_UART1},
	{PIN_D18, PID_UART2},
	{PIN_E3,  PID_UART3},

	{PIN_UART4_RTS, PID_UART4},

	{0xFF,    0xFF} // end of table
#endif
};

/**
  * @brief Pin map of UART CTS pin.
  */
SECTION_UART_RODATA const hal_pin_map uart_cts_pin_map[] = {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_NC,  PID_UART0},//UART0 NONE
	{PIN_F2,  PID_UART1},
	{PIN_E3,  PID_UART2},
	{PIN_E8,  PID_UART3},

	{PIN_UART4_CTS, PID_UART4},

	{0xFF,    0xFF} // end of table
#else
	{PIN_NC,  PID_UART0},//UART0 NONE
	{PIN_F1,  PID_UART1},
	{PIN_D17, PID_UART2},
	{PIN_E4,  PID_UART3},

	{PIN_UART4_CTS, PID_UART4},

	{0xFF,    0xFF} // end of table
#endif
};

/**
  * @brief RTS pin list for each UART index.
  */
SECTION_UART_RODATA const uint32_t uart_rts_pin_tbl[] = {
	PIN_NC,         // RTS pin of UART0
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	PIN_F3,         // RTS pin of UART1
	PIN_E4,         // RTS pin of UART2
	PIN_E7,         // RTS pin of UART3
#else
	PIN_F2,         // RTS pin of UART1
	PIN_D18,        // RTS pin of UART2
	PIN_E3,         // RTS pin of UART3
#endif
	PIN_UART4_RTS   // RTS pin of UART4
};

/**
  * @brief CTS pin list for each UART index.
  */
SECTION_UART_RODATA const uint32_t uart_cts_pin_tbl[] = {
	PIN_NC,         // CTS pin of UART0
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	PIN_F2,         // CTS pin of UART1
	PIN_E3,         // CTS pin of UART2
	PIN_E8,         // CTS pin of UART3
#else
	PIN_F1,         // CTS pin of UART1
	PIN_D17,        // CTS pin of UART2
	PIN_E4,         // CTS pin of UART3
#endif
	PIN_UART4_CTS   // RTS pin of UART4
};

/**
  * @brief GDMA handshake interface ID for UART TX.
  */
SECTION_UART_RODATA const uint8_t uart_tx_gdma_hsk_id_tbl[] = {
	GDMA_HANDSHAKE_UART0_TX,        // GDMA hardware handshake number for UART0 TX
	GDMA_HANDSHAKE_UART1_TX,        // GDMA hardware handshake number for UART1 TX
	GDMA_HANDSHAKE_UART2_TX,        // GDMA hardware handshake number for UART2 TX
	GDMA_HANDSHAKE_UART3_TX,        // GDMA hardware handshake number for UART3 TX
	GDMA_HANDSHAKE_BT_UART_TX       // GDMA hardware handshake number for UART4 TX
};

/**
  * @brief GDMA handshake interface ID for UART RX.
  */
SECTION_UART_RODATA const uint8_t uart_rx_gdma_hsk_id_tbl[] = {
	GDMA_HANDSHAKE_UART0_RX,        // GDMA hardware handshake number for UART0 RX
	GDMA_HANDSHAKE_UART1_RX,        // GDMA hardware handshake number for UART1 RX
	GDMA_HANDSHAKE_UART2_RX,        // GDMA hardware handshake number for UART2 RX
	GDMA_HANDSHAKE_UART3_RX,        // GDMA hardware handshake number for UART3 RX
	GDMA_HANDSHAKE_BT_UART_RX       // GDMA hardware handshake number for UART4 RX
};

/**
  * @brief The default UART adapter setting. This structure can be used to initial a new
  *        UART adapter for UART0.
  */
SECTION_UART_RODATA const hal_uart_defconfig_t hal_uart_default_setting_40m = {
	.baudrate = 115200,                 /*!< the baud rate: 115200  */
	.flow_ctrl = UartFlowCtlNone,       /*!< flow control setting */
	.word_len = 8,                      /*!< frame length: 8 bits */
	.stop_bit = 1,                      /*!< stop bit: 1 stop bit */
	.parity_type = UartParityNone,      /*!< parity check: none  */
	.pdef_baudrate_tbl = def_baudrate_table,            /*!< the pre-defined support baud rate table */
	.pdef_ovsr_tbl = def_ovsr_40m,                      /*!< OVSR table for pre-defined baud rate */
	.pdef_div_tbl = def_div_40m,                        /*!< the table of DIV for pre-defined baud rate */
	.pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_40m,  /*!< the table of OVSR-Adj bits for 10 bits frame */
	.pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_40m,    /*!< the table of OVSR-Adj bits for 9 bits frame */
	.pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_40m,    /*!< the table of OVSR-Adj bits for 8 bits frame */
	.pdef_ovsradj_tbl10 = ovsr_adj_table_10bit,         /*!< the table of OVSR-Adj for for 10 bits frame */
	.pdef_ovsradj_tbl9 = ovsr_adj_table_9bit,           /*!< the table of OVSR-Adj for for 9 bits frame */
	.pdef_ovsradj_tbl8 = ovsr_adj_table_8bit            /*!< the table of OVSR-Adj for for 8 bits frame */
};

/**
  * @brief The default UART adapter setting. This structure can be used to initial a new
  *        UART adapter for UART 1/2/3/4.
  */
SECTION_UART_RODATA const hal_uart_defconfig_t hal_uart_default_setting_50m = {
	.baudrate = 115200,                 /*!< the baud rate: 115200  */
	.flow_ctrl = UartFlowCtlNone,       /*!< flow control setting */
	.word_len = 8,                      /*!< frame length: 8 bits */
	.stop_bit = 1,                      /*!< stop bit: 1 stop bit */
	.parity_type = UartParityNone,      /*!< parity check: none  */
	.pdef_baudrate_tbl = def_baudrate_table,            /*!< the pre-defined support baud rate table */
	.pdef_ovsr_tbl = def_ovsr_50m,                      /*!< OVSR table for pre-defined baud rate */
	.pdef_div_tbl = def_div_50m,                        /*!< the table of DIV for pre-defined baud rate */
	.pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_50m,  /*!< the table of OVSR-Adj bits for 10 bits frame */
	.pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_50m,    /*!< the table of OVSR-Adj bits for 9 bits frame */
	.pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_50m,    /*!< the table of OVSR-Adj bits for 8 bits frame */
	.pdef_ovsradj_tbl10 = ovsr_adj_table_10bit,         /*!< the table of OVSR-Adj for for 10 bits frame */
	.pdef_ovsradj_tbl9 = ovsr_adj_table_9bit,           /*!< the table of OVSR-Adj for for 9 bits frame */
	.pdef_ovsradj_tbl8 = ovsr_adj_table_8bit            /*!< the table of OVSR-Adj for for 8 bits frame */
};


/**
  * @brief The global common data structure to store common resource
  *        for all UART adapters.
  */
SECTION_UART_BSS hal_uart_adapter_t *_puart_adapter[5];

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_uart_rom_func UART HAL ROM APIs.
 * @ingroup hs_hal_uart
 * @{
 * @brief UART HAL ROM API. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of UART HAL APIs in the RAM space is provided for the user application.
 */

/**
  * @brief The stubs functions table to exports UART HAL functions in ROM.
  */
SECTION_UART_STUBS const hal_uart_func_stubs_t hal_uart_stubs = {
	.hal_uart_reset_rx_fifo = hal_rtl_uart_reset_rx_fifo,
	.hal_uart_gen_baudrate = hal_rtl_uart_gen_baudrate,
	.hal_uart_set_baudrate = hal_rtl_uart_set_baudrate,
	.hal_uart_set_format = hal_rtl_uart_set_format,
	.hal_uart_set_flow_control = hal_rtl_uart_set_flow_control,
	.hal_uart_init = hal_rtl_uart_init,
	.hal_uart_deinit = hal_rtl_uart_deinit,
	.uart_irq_handler = _uart_irq_handler,
	.hal_uart_tx_gdma_init = hal_rtl_uart_tx_gdma_init,
	.hal_uart_tx_gdma_deinit = hal_rtl_uart_tx_gdma_deinit,
	.hal_uart_rx_gdma_init = hal_rtl_uart_rx_gdma_init,
	.hal_uart_rx_gdma_deinit = hal_rtl_uart_rx_gdma_deinit,
	.uart_tx_dma_irq_handler = _uart_tx_dma_irq_handler,
	.uart_rx_dma_irq_handler = _uart_rx_dma_irq_handler,
	.hal_uart_writeable = hal_rtl_uart_writeable,
	.hal_uart_putc = hal_rtl_uart_putc,
	.hal_uart_wputc = hal_rtl_uart_wputc,
	.hal_uart_wait_tx_done = hal_rtl_uart_wait_tx_done,
	.hal_uart_send = hal_rtl_uart_send,
	.hal_uart_int_send = hal_rtl_uart_int_send,
	.hal_uart_dma_send = hal_rtl_uart_dma_send,
	.hal_uart_send_abort = hal_rtl_uart_send_abort,
	.hal_uart_readable = hal_rtl_uart_readable,
	.hal_uart_getc = hal_rtl_uart_getc,
	.hal_uart_rgetc = hal_rtl_uart_rgetc,
	.hal_uart_recv = hal_rtl_uart_recv,
	.hal_uart_int_recv = hal_rtl_uart_int_recv,
	.hal_uart_dma_recv = hal_rtl_uart_dma_recv,
	.hal_uart_recv_abort = hal_rtl_uart_recv_abort,
	.hal_uart_set_rts = hal_rtl_uart_set_rts,
	.hal_uart_tx_pause = hal_rtl_uart_tx_pause,
	.hal_uart_reg_irq = hal_rtl_uart_reg_irq,
	.hal_uart_unreg_irq = hal_rtl_uart_unreg_irq,
	.hal_uart_adapter_init = hal_rtl_uart_adapter_init,
	.hal_uart_line_sts_hook = hal_rtl_uart_line_sts_hook,
	.hal_uart_txtd_hook = hal_rtl_uart_txtd_hook,
	.hal_uart_rxind_hook = hal_rtl_uart_rxind_hook,
	.hal_uart_txdone_hook = hal_rtl_uart_txdone_hook,
	.hal_uart_rxdone_hook = hal_rtl_uart_rxdone_hook,
	.hal_uart_set_rx_filter_pattern = hal_rtl_uart_set_rx_filter_pattern,
	.hal_uart_set_rx_filter_op = hal_rtl_uart_set_rx_filter_op,
	.hal_uart_set_rx_filter_timeout = hal_rtl_uart_set_rx_filter_timeout,
	.hal_uart_rx_filter_en = hal_rtl_uart_rx_filter_en,
	.hal_uart_rx_filter_dis = hal_rtl_uart_rx_filter_dis,
	.hal_uart_reset_receiver = hal_rtl_uart_reset_receiver,
	.hal_uart_set_tx_fifo_level = hal_rtl_uart_set_tx_fifo_level,
	.hal_uart_set_rx_fifo_level = hal_rtl_uart_set_rx_fifo_level,
	.hal_uart_rx_idle_timeout_en = hal_rtl_uart_rx_idle_timeout_en,
	.hal_uart_rx_idle_timeout_dis = hal_rtl_uart_rx_idle_timeout_dis,
	.hal_uart_en_ctrl = hal_rtl_uart_en_ctrl,
	.hal_uart_tx_fifo_low_hook = hal_rtl_uart_tx_fifo_low_hook,
	.hal_uart_pin_to_idx = hal_rtl_uart_pin_to_idx,
	.hal_uart_tx_isr = uart_tx_isr,
	.hal_uart_rx_isr = uart_rx_isr,
	.hal_uart_iir_isr = uart_iir_isr,
	.hal_uart_baud_rate_table_dump = hal_rtl_uart_baud_rate_table_dump,
	.hal_uart_lp_sclk_select = hal_rtl_uart_lp_sclk_select,
	.hal_uart_set_default_state = hal_rtl_uart_set_default_state,
	.hal_uart_load_default_state = hal_rtl_uart_load_default_state
};

/**
 *  @brief The TX interrupt handler for the interrupt mode transmission.
 *         This interrupt will be triggered whenever the TX FIFO level
 *         lower than threshold.
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void uart_tx_isr(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart;
	uint32_t buf_size;

	if (puart_adapter == NULL) {
		return;
	}

	puart = puart_adapter->base_addr;

	while (puart_adapter->tx_count > 0) {
		buf_size = Uart_Tx_FIFO_Size - puart->tflvr_b.tx_fifo_lv;
		if (buf_size > 0) {
			if (buf_size > puart_adapter->tx_count) {
				buf_size = puart_adapter->tx_count;
			}
			for (; buf_size > 0; buf_size--) {
				puart->thr = (uint8_t)(*(puart_adapter->ptx_buf));
				puart_adapter->tx_count--;
				puart_adapter->ptx_buf++;
			}
		} else {
			break;
		}
	}

	if (0 == puart_adapter->tx_count) {
		// Disable Tx FIFO low interrupt
		puart->vier_b.tx_fifo_lv_int_en = 0;
		// call TX FIFO low event callback
		if (puart_adapter->tx_fifo_low_callback != NULL) {
			(puart_adapter->tx_fifo_low_callback)(puart_adapter->tx_fifo_low_cb_para);
		}
		// Enable TX FIFO empty interrupt (TX totally done)
		puart->ier_b.etbei = 1;
	}
}

/**
 *  @brief The RX interrupt handler for the interrupt mode transmission
 *         This interrupt will be triggeren whenever the RX FIFO level
 *         over the thread or data ready timeout.
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void uart_rx_isr(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;
	volatile uint32_t bytes_in_fifo;
	uart_lsr_t lsr;
	uint8_t lsr_err;
	uint8_t lsr_err_sum;

	while (puart_adapter->rx_count > 0) {
		bytes_in_fifo = puart->rflvr_b.rx_fifo_lv;
		if (bytes_in_fifo > 0) {
			if (bytes_in_fifo > puart_adapter->rx_count) {
				bytes_in_fifo = puart_adapter->rx_count;
			}

			lsr.w = puart->lsr;
			lsr.w |= puart_adapter->lsr;
			puart_adapter->lsr = 0;
			if (lsr.b.rxfifo_err == 0) {
				for (; bytes_in_fifo > 0; bytes_in_fifo--) {
					*(puart_adapter->prx_buf) = (uint8_t)puart->rbr;
					puart_adapter->rx_count--;
					puart_adapter->prx_buf++;
				}
			} else {
				// line status error: pop RX FIFO and read LSR for every RX byte
				lsr_err_sum = HAL_UART_STATUS_ERR_RXFIFO;
				for (; bytes_in_fifo > 0; bytes_in_fifo--) {
					*(puart_adapter->prx_buf) = (uint8_t)puart->rbr;
					// read LSR after every RBR reading, to get the LSR for the read RX byte
					lsr_err = puart->lsr & UART_LSR_ERR;
					if (lsr_err != 0) {
						if (puart_adapter->lsr_callback != NULL) {
							puart_adapter->lsr_callback(lsr_err, puart_adapter->lsr_cb_para);
						}
						lsr_err_sum |= lsr_err;
					}
					puart_adapter->rx_count--;
					puart_adapter->prx_buf++;
				}
				puart_adapter->rx_status |= lsr_err_sum;
			}
		} else {
			break;
		}
	}

	if (puart_adapter->rx_count == 0) {
		// Disable RX trigger Interrupt
		if (puart_adapter->rx_dr_callback == NULL) {
			puart->ier_b.erbi = 0;
		}

		uart_clear_state(puart_adapter, HAL_UART_STATE_RX_BUSY);

		// Call User Rx complete callback
		if (puart_adapter->rx_done_callback != NULL) {
			puart_adapter->rx_done_callback(puart_adapter->rx_done_cb_para);
		}
	}
}



/**
 *  @brief The ISR for the IIR events processing.
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void uart_iir_isr(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;
	volatile uart_iir_t reg_iir;
	uint8_t lsr;
	uint8_t int_id;
	int i;

	for (i = 0; i < 5; i++) {
		// Maximum process 5 different interrupt events
		reg_iir.w = puart->iir;
		if (reg_iir.b.int_pend != 0) {
			// No pending IRQ
			break;
		}

		int_id = reg_iir.b.int_id;

		switch (int_id) {
		case ModemStatus:
			puart_adapter->modem_status = (uint8_t)puart->msr;
			// Call the Modem Status handler
			if (puart_adapter->modem_status_ind != NULL) {
				puart_adapter->modem_status_ind(puart_adapter);
			}
			break;

		case TxFifoEmpty:
			if (puart->ier_b.etbei == 0) {
				// got TX FIFO empty IRQ while this IRQ is disabled
				// It could be caused by multiple-issue when etbei = 1, so just ignore it for this case
				break;
			}

			if (puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) {
				uart_clear_state(puart_adapter, (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY));
				// Call user TX complete callback
				if (NULL != puart_adapter->tx_done_callback) {
					puart_adapter->tx_done_callback(puart_adapter->tx_done_cb_para);
				}
				// Disable TX FIFO empty interrupt (TX totally done)
				puart->ier_b.etbei = 0;
			} else {
				// Call Tx done callback
				if (puart_adapter->tx_td_callback != NULL) {
					puart_adapter->tx_td_callback(puart_adapter->tx_td_cb_id, puart_adapter->tx_td_cb_ev);
				} else {
					// Disable TX FIFO empty interrupt
					puart->ier_b.etbei = 0;
				}
			}
			break;

		case ReceiverDataAvailable:
		case TimeoutIndication:
			if (puart_adapter->state & HAL_UART_STATE_RX_BUSY) {
				uart_rx_isr(puart_adapter);
			} else if ((puart_adapter->state & HAL_UART_STATE_DMARX_BUSY) == 0) {
				// Call Rx data ready callback
				if (puart_adapter->rx_dr_callback != NULL) {
					if (puart->rflvr_b.rx_fifo_lv > 0) {
						puart_adapter->rx_dr_callback(puart_adapter->rx_dr_cb_id, puart_adapter->rx_dr_cb_ev);
					}
				}
			}
			break;

		case ReceivLineStatus:
			lsr = puart->lsr & UART_LSR_ERR;
			puart_adapter->rx_status |= lsr;
			if (puart_adapter->lsr_callback != NULL) {
				puart_adapter->lsr_callback(lsr, puart_adapter->lsr_cb_para);
			}
			break;

		default:
			DBG_UART_ERR(ANSI_COLOR_RED"Unknown Interrupt Type\n"ANSI_COLOR_RESET);
			break;
		}
	}
}

/**
 *  @brief The default UART interrupt handler. It process all interrupt events.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void _uart_irq_handler(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;
	uart_visr_t visr;

	if (puart->iir_b.int_pend == 0) {
		uart_iir_isr(puart_adapter);
	}

	visr.w = puart->visr & puart->vier;
	if (visr.w != 0) {
		if (visr.b.rx_idle_timeout) {
			// RX idle timeout
			puart->visr_b.rx_idle_timeout_int_sts = 1;   // clear interrupt
			if (puart_adapter->rx_idle_timeout_callback) {
				(puart_adapter->rx_idle_timeout_callback)(puart_adapter->rx_idle_timeout_cb_arg);
			}
		}

		if (visr.b.rf_match_patt) {
			puart->visr_b.rf_patt_match_int_sts = 1;   // clear interrupt
			if (puart_adapter->rx_flt_matched_callback) {
				(puart_adapter->rx_flt_matched_callback)(puart_adapter->rx_flt_matched_cb_arg);
			}
		}

		if (visr.b.rf_timeout) {
			puart->visr_b.rf_idle_timeout_int_sts = 1;   // clear interrupt
			if (puart_adapter->rx_flt_timeout_callback) {
				(puart_adapter->rx_flt_timeout_callback)(puart_adapter->rx_flt_timeout_cb_arg);
			}
		}

		if (visr.b.tx_fifo_lv) {
			puart->visr_b.tx_fifo_lv_int_sts = 1;   // clear interrupt
			if (puart_adapter->state & HAL_UART_STATE_TX_BUSY) {
				uart_tx_isr(puart_adapter);
			} else {
				// Disable Tx FIFO low interrupt
				puart->vier_b.tx_fifo_lv_int_en = 0;
			}
		}
	}

	if (puart->iir_b.int_pend == 0) {
		uart_iir_isr(puart_adapter);
	}

}


SECTION_UART_TEXT
void UART0_IRQHandler(void)
{

	hal_rtl_irq_clear_pending(UART0_IRQn);
	_puart_adapter[0]->irq_fun((void *)_puart_adapter[0]);
	__ISB();
}

SECTION_UART_TEXT
void UART1_IRQHandler(void)
{

	hal_rtl_irq_clear_pending(UART1_IRQn);
	_puart_adapter[1]->irq_fun((void *)_puart_adapter[1]);
	__ISB();
}

SECTION_UART_TEXT
void UART2_IRQHandler(void)
{

	hal_rtl_irq_clear_pending(UART2_IRQn);
	_puart_adapter[2]->irq_fun((void *)_puart_adapter[2]);
	__ISB();
}

SECTION_UART_TEXT
void UART3_IRQHandler(void)
{

	hal_rtl_irq_clear_pending(UART3_IRQn);
	_puart_adapter[3]->irq_fun((void *)_puart_adapter[3]);
	__ISB();
}

SECTION_UART_TEXT
void BTUART_IRQHandler(void)
{

	hal_rtl_irq_clear_pending(BTUART_IRQn);
	_puart_adapter[4]->irq_fun((void *)_puart_adapter[4]);
	__ISB();
}

/**
 *  @brief The TX DMA interrupt handler. This interrupt will be triggered when
 *         a TX DMA mode transfer is done.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void _uart_tx_dma_irq_handler(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;

	puart->miscr_b.txdma_en = 0; // Disable UART TX DMA

	// update TX Buffer adddr
	puart_adapter->ptx_buf = (uint8_t *)((uint32_t)(puart_adapter->ptx_buf_sar) + puart_adapter->tx_count);

	// Wait all data in TX FIFO to be transfered
	// Enable TX FIFO empty interrupt (TX totally done)
	if (puart->tflvr_b.tx_fifo_lv == 0) {
		// Call user TX complete callback
		if (NULL != puart_adapter->tx_done_callback) {
			puart_adapter->tx_done_callback(puart_adapter->tx_done_cb_para);
		}
		uart_clear_state(puart_adapter, HAL_UART_STATE_DMATX_BUSY);
	} else {
		puart->ier_b.etbei = 1;
	}
}

/**
 *  @brief The RX DMA interrupt handler. This interrupt will be triggered when
 *         a RX DMA mode transfer is done.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void _uart_rx_dma_irq_handler(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;

	puart->miscr_b.rxdma_en = 0; // Disable UART RX DMA
	uart_clear_state(puart_adapter, HAL_UART_STATE_DMARX_BUSY);

	// D-Cache sync (Invalidate)
	if (puart_adapter->dcache_invalidate_by_addr != NULL) {
		puart_adapter->dcache_invalidate_by_addr((uint32_t *)puart_adapter->prx_buf_dar, (int32_t)puart_adapter->rx_count);
	}

	puart_adapter->prx_buf = (uint8_t *)((uint32_t)(puart_adapter->prx_buf_dar) + puart_adapter->rx_count);

	// Call User Rx complete callback
	if (puart_adapter->rx_done_callback != NULL) {
		puart_adapter->rx_done_callback(puart_adapter->rx_done_cb_para);
	}
}


/**
 *  @brief To enable or disable an UART device. It controls the
 *         hardware function, bus domain, PCLK and SCLK of the
 *         specified UART.
 *
 *  @param[in]  uart_idx The UART index. The value can be 0 .. 2.
 *  @param[in]  en  Enable control:
 *                    - 0: Fully disable the UART hardware(function and clock).
 *                    - 1: Fully enable the UART hardware(function and clock).
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_en_ctrl(uint8_t uart_idx, BOOL en)
{
#if !defined(CONFIG_BUILD_NONSECURE)
	//enable UART fun/BUS/SCLK/PCLK
	switch (uart_idx) {
	case Uart0:
		hal_rtl_sys_peripheral_en(UART0_SYS, en);
		break;

	case Uart1:
		hal_rtl_sys_peripheral_en(UART1_SYS, en);
		break;

	case Uart2:
		hal_rtl_sys_peripheral_en(UART2_SYS, en);
		break;

	case Uart3:
		hal_rtl_sys_peripheral_en(UART3_SYS, en);
		break;

	case Uart4:
		hal_rtl_sys_peripheral_en(UART4_SYS, en);    //BTUART
		break;

	default:
		// error, invalid UART Index
		return;
	}
#endif
	return;
}

/**
 *  @brief To set a state flag of the UART.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  state  The state flag going to be set.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void uart_set_state(phal_uart_adapter_t puart_adapter, uint32_t state)
{
	puart_adapter->state |= state;
}

/**
 *  @brief To register a interrupt handler for the given UART port.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  handler  The interrupt handle function.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_reg_irq(phal_uart_adapter_t puart_adapter, irq_handler_t handler)
{
	uint8_t uart_idx;
	uint8_t uart_irq = 0xFF;
	uint8_t uart_irq_pri = 0xFF;
	uint32_t vector = 0x0;

	uart_idx = puart_adapter->uart_idx;

	switch (uart_idx) {
	case Uart0:
		uart_irq = UART0_IRQn;
		uart_irq_pri = UART0_IRQPri;
		vector = (uint32_t)&UART0_IRQHandler;
		break;

	case Uart1:
		uart_irq = UART1_IRQn;
		uart_irq_pri = UART1_IRQPri;
		vector = (uint32_t)&UART1_IRQHandler;
		break;

	case Uart2:
		uart_irq = UART2_IRQn;
		uart_irq_pri = UART2_IRQPri;
		vector = (uint32_t)&UART2_IRQHandler;
		break;

	case Uart3:
		uart_irq = UART3_IRQn;
		uart_irq_pri = UART3_IRQPri;
		vector = (uint32_t)&UART3_IRQHandler;
		break;

	case Uart4:
		uart_irq = BTUART_IRQn;
		uart_irq_pri = BTUART_IRQPri;
		vector = (uint32_t)&BTUART_IRQHandler;
		break;

	default:
		// error, invalid UART Index
		break;
	}

	hal_rtl_irq_disable(uart_irq);
	__ISB();

	// Register UART    IRQ  to Vector table
	hal_rtl_irq_set_vector(uart_irq, vector);
	hal_rtl_irq_clear_pending(uart_irq);
	hal_rtl_irq_set_priority(uart_irq, uart_irq_pri);

	hal_rtl_irq_enable(uart_irq);

	//hook irq handler
	puart_adapter->irq_fun = handler;
}

/**
 *  @brief To un-register the interrupt handler of the given UART port.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_unreg_irq(phal_uart_adapter_t puart_adapter)
{
	uint8_t uart_idx = puart_adapter->uart_idx;
	uint8_t uart_irq = 0xFF;

	switch (uart_idx) {
	case Uart0:
		uart_irq = UART0_IRQn;
		break;

	case Uart1:
		uart_irq = UART1_IRQn;
		break;

	case Uart2:
		uart_irq = UART2_IRQn;
		break;

	case Uart3:
		uart_irq = UART3_IRQn;
		break;

	case Uart4:
		uart_irq = BTUART_IRQn;
		break;

	default:
		// error, invalid UART Index
		break;
	}

	if (uart_idx < MaxUartNum) {
		_puart_adapter[uart_idx]->irq_fun = (irq_handler_t)NULL;
		hal_rtl_irq_disable(uart_irq);
	}
}

/**
 *  @brief Resets the RX FIFO and the receiver.
 *         The reset steps are:
 *           1. Assert receiver reset hardware bit.
 *           2. Assert clear_rxfifo hardware bit.
 *           3. De-assert receiver reset hardware bit.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    Always return HAL_OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_reset_rx_fifo(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart;

	puart = puart_adapter->base_addr;

	/* Step 1: Enable Reset_rcv */
	puart->stsr_b.reset_rcv = 1;

	/* clear RX fifo */
	puart->fcr_b.clear_rxfifo = 1;
	__NOP();
	__NOP();

	/* Step 3: Disable Reset_rcv */
	puart->stsr_b.reset_rcv = 0;

	return HAL_OK;
}

/**
 *  @brief Calculates the divisor, over sampling for a given baud rate.
 *
 *  @param[in]  pbaud_setting The needed parameters, baud rate and system clock
 *                                for the calculation. The result also will be passed by
 *                                this structure.
 *
 *  @returns        Always return HAL_OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_gen_baudrate(uart_speed_setting_t *pbaud_setting)
{
	float divid_f;
	float ovsr_f;
	float ovsr_mod_f;
	float bit_adj_f;
	float best_ovsr_f;
	float best_ovsr_mod_f;
	uint32_t divisor;
	uint32_t best_divisor;
	uint32_t ovsr_d;
	uint32_t adj_bits;
	uint32_t ovsr_adj;

	divid_f = (float)pbaud_setting->sclk / (float)pbaud_setting->baudrate;
	best_ovsr_f = 20.0f;
	best_divisor = (uint32_t)(divid_f / 20.0f);

	//DBG_UART_INFO("divid_f = %f\r\n", divid_f);

	divisor = 1;
	//ovsr_f = 100.0f;
	best_ovsr_mod_f = 1.0f;
	while (1) {
		ovsr_f = divid_f / (float)divisor;
		if (ovsr_f  < 21.0f) {
			ovsr_d = (uint32_t)(ovsr_f / 1.00f);
			ovsr_mod_f = ovsr_f - (float)ovsr_d;
			if (ovsr_mod_f < 0.01f) {
				//best_ovsr_mod_f = ovsr_mod_f;
				best_ovsr_f = ovsr_f;
				best_divisor = divisor;
				break;
			} else if (ovsr_mod_f < best_ovsr_mod_f) {
				//best_ovsr_mod_f = ovsr_mod_f;
				best_ovsr_f = ovsr_f;
				best_divisor = divisor;
				if (ovsr_f <= 10.0f) {
					break;
				}
			} else if (ovsr_f <= 10.0f) {
				if (ovsr_mod_f < best_ovsr_mod_f) {
					//best_ovsr_mod_f = ovsr_mod_f;
					best_ovsr_f = ovsr_f;
					best_divisor = divisor;
				}
				break;
			}
		}
		divisor++;
	}

	//DBG_UART_INFO("divisor = %u ovsr_f=%f\r\n",divisor, ovsr_f);

	ovsr_d = (uint32_t)(best_ovsr_f / 1.00f);
	bit_adj_f = 1.0f / (float)pbaud_setting->ovsr_adj_max_bits;
	adj_bits = (uint32_t)(((best_ovsr_f - (float)ovsr_d) * 10.0f) / bit_adj_f);
	if ((adj_bits % 10) > 5) {
		adj_bits = adj_bits / 10 + 1;
	} else {
		adj_bits = adj_bits / 10;
	}
	//DBG_UART_INFO("adj_bits = %u\r\n",adj_bits);

	if (adj_bits > (pbaud_setting->ovsr_adj_max_bits - 1)) {
		//DBG_UART_WARN("HalRuartGenBaudRateRtl8195a: adj_bits=%d\r\n", adj_bits);
		ovsr_d++;
		adj_bits = 0;
	}

	ovsr_adj = pbaud_setting->ovsr_adj_map[adj_bits];

	pbaud_setting->ovsr = ovsr_d;
	pbaud_setting->div = best_divisor;
	pbaud_setting->ovsr_adj = ovsr_adj;
	pbaud_setting->ovsr_adj_bits = (uint8_t)adj_bits;

	return HAL_OK;
}


/**
 *  @brief Configures the baud rate setting.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  baudrate  The baud rate setting. If the baud rate value is invalid then
 *                        baud rate 9600 will be the used for the configuration.
 *
 *  @returns    Always return HAL_OK
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_set_baudrate(phal_uart_adapter_t puart_adapter, uint32_t baudrate)
{
	uart_speed_setting_t uart_speed_setting;
	uint32_t dll;
	uint32_t dlm;
	uint32_t divisor = 490;
	uint32_t ovsr = 17;
	uint32_t ovsr_adj = 0;
	uint32_t i;
	uint8_t is_defined_baud;
	uint8_t frame_bits;
	uint8_t adj_bits;
	UART_Type *puart;

	frame_bits = puart_adapter->word_len + 1; // 1 start bit + data bits
	if (puart_adapter->parity_type != UartParityNone) {
		frame_bits++;   // 1 parity bit
	}

	is_defined_baud = 0;

	if (puart_adapter->pdef_baudrate_tbl != NULL) {
		i = 0;

		while (puart_adapter->pdef_baudrate_tbl[i] < 0xffffffff) {
			if (puart_adapter->pdef_baudrate_tbl[i] == baudrate) {
				divisor = puart_adapter->pdef_div_tbl[i];
				ovsr = puart_adapter->pdef_ovsr_tbl[i];
				switch (frame_bits) {
				case 9:
					adj_bits = puart_adapter->pdef_ovsradjbit_tbl9[i];
					ovsr_adj = puart_adapter->pdef_ovsradj_tbl9[adj_bits];
					break;
				case 10:
					adj_bits = puart_adapter->pdef_ovsradjbit_tbl10[i];
					ovsr_adj = puart_adapter->pdef_ovsradj_tbl10[adj_bits];
					break;
				case 8:
					adj_bits = puart_adapter->pdef_ovsradjbit_tbl8[i];
					ovsr_adj = puart_adapter->pdef_ovsradj_tbl8[adj_bits];
					break;

				default:
					adj_bits = puart_adapter->pdef_ovsradjbit_tbl9[i];
					ovsr_adj = puart_adapter->pdef_ovsradj_tbl9[adj_bits];
					break;
				}
				is_defined_baud = 1;
				break;  // break the while loop
			} else {
				i++;
			}
		}
	}

	if (is_defined_baud == 0) {
		switch (frame_bits) {
		case 9:
			uart_speed_setting.ovsr_adj_map = puart_adapter->pdef_ovsradj_tbl9;
			break;

		case 10:
			uart_speed_setting.ovsr_adj_map = puart_adapter->pdef_ovsradj_tbl10;
			break;

		case 8:
			uart_speed_setting.ovsr_adj_map = puart_adapter->pdef_ovsradj_tbl8;
			break;

		default:
			frame_bits = 9;
			uart_speed_setting.ovsr_adj_map = puart_adapter->pdef_ovsradj_tbl9;
			break;
		}
		DBG_UART_WARN("BaudRate(%u) not in the Lookup table \n", baudrate);
		uart_speed_setting.ovsr_adj_max_bits = frame_bits;
		uart_speed_setting.max_err = 3;
		uart_speed_setting.ovsr_min = UART_OVSR_POOL_MIN;
		uart_speed_setting.ovsr_max = UART_OVSR_POOL_MAX;
		uart_speed_setting.divisor_resolution = DIVISOR_RESOLUTION;
		uart_speed_setting.jitter_lim = JITTER_LIMIT;
		uart_speed_setting.baudrate = baudrate;
		switch (puart_adapter->uart_sclk) {
		case UartSCLK_4M:
			uart_speed_setting.sclk = UART_SCLK_4M;
			break;
		case UartSCLK_26M:
			uart_speed_setting.sclk = UART_SCLK_26M;
			break;

		case UartSCLK_40M:
			uart_speed_setting.sclk = UART_SCLK_40M;
			break;

		case UartSCLK_50M:
			uart_speed_setting.sclk = UART_SCLK_50M;
			break;

		default:
			if (puart_adapter->uart_idx == Uart0) {
				//for uart0
				uart_speed_setting.sclk = UART_SCLK_40M;
			} else {
				//for uart1~4
				uart_speed_setting.sclk = UART_SCLK_50M;
			}
			break;
		}

		if (hal_rtl_uart_gen_baudrate(&uart_speed_setting) == HAL_OK) {
			divisor = uart_speed_setting.div;
			ovsr = uart_speed_setting.ovsr;
			ovsr_adj = uart_speed_setting.ovsr_adj;
		} else {
			DBG_UART_ERR("Invalid BaudRate(%d), Force Baud Rateit as 9600\n", baudrate);
			if (puart_adapter->uart_idx == Uart0) {
				//for uart0
				divisor = 490;
				ovsr = 17;
				ovsr_adj = 0;
				baudrate = 9600;
			} else {
				//for uart1~4
				divisor = 548;
				ovsr = 19;
				ovsr_adj = 0;
				baudrate = 9600;
			}
		}
	}

	DBG_UART_INFO("uart_set_baudrate: BaudRate:%d Divisor:%d Ovsr:%d Ovsr_ADj:0x%x\r\n",
				  baudrate, divisor, ovsr, ovsr_adj);

	puart = puart_adapter->base_addr;

	dll = divisor & 0xFF;
	dlm = (divisor & 0xFF00) >> 8;

	/* Set DLAB bit to 1 to access DLL/DLM */
	puart->lcr_b.dlab = 1;

#if CONFIG_PXP
	/* set divisor */
	puart->dllr = 1;
#else
	/* set divisor */
	puart->dllr = dll;
#endif

	puart->dlmr = dlm;

	/**
	  * Clean Rx break signal interrupt status at initial stage.
	  */
	puart->scr_b.rx_break_int_sts = 1;

#if CONFIG_PXP
	/* Set OVSR(xfactor) */
	puart->stsr_b.xfactor = 0xB;

	/* Set OVSR_ADJ[10:0] (xfactor_adj[26:16]) */
	puart->scr_b.xfactor_adj = 0;
#else
	/* Set OVSR(xfactor) */
	puart->stsr_b.xfactor = ovsr - 5;

	/* Set OVSR_ADJ[10:0] (xfactor_adj[26:16]) */
	puart->scr_b.xfactor_adj = ovsr_adj;
#endif
	/* clear DLAB bit */
	puart->lcr_b.dlab = 0;

	puart_adapter->baudrate = baudrate;
	puart_adapter->frame_bits = frame_bits;

	return HAL_OK;
}

SECTION_UART_TEXT
void hal_rtl_uart_baud_rate_table_dump(uint32_t sclk, uint32_t *baudrate_tbl)
{
	uart_speed_setting_t uart_speed_setting;
	uint32_t i, j;
	uint32_t best_divisor;
	uint32_t ovsr_d;
	uint32_t adj_bits;
	uint32_t ovsr_adj;
	uint32_t actural_baud;
	uint32_t baudrate;
	float act_divid_f;

	uart_speed_setting.max_err = 3;
	uart_speed_setting.ovsr_min = UART_OVSR_POOL_MIN;
	uart_speed_setting.ovsr_max = UART_OVSR_POOL_MAX;
	uart_speed_setting.divisor_resolution = DIVISOR_RESOLUTION;
	uart_speed_setting.jitter_lim = JITTER_LIMIT;
	uart_speed_setting.sclk = sclk;

	for (j = 8; j < 11; j++) {
		uart_speed_setting.ovsr_adj_max_bits = (uint8_t)j;
		if (j == 8) {
			uart_speed_setting.ovsr_adj_map = ovsr_adj_table_8bit;
		} else if (j == 9) {
			uart_speed_setting.ovsr_adj_map = ovsr_adj_table_9bit;
		} else {
			uart_speed_setting.ovsr_adj_map = ovsr_adj_table_10bit;
		}

		DBG_UART_INFO("===== %u frame bits ====\r\n", j);
		i = 0;
		while (baudrate_tbl[i] != 0xffffffff) {
			baudrate = baudrate_tbl[i];
			uart_speed_setting.baudrate = baudrate;
			hal_rtl_uart_gen_baudrate(&uart_speed_setting);

			ovsr_d = uart_speed_setting.ovsr;
			best_divisor = uart_speed_setting.div;
			ovsr_adj = uart_speed_setting.ovsr_adj;
			adj_bits = uart_speed_setting.ovsr_adj_bits;

			DBG_UART_INFO("BaudRate=%8u ovsr=%2u divisor=%6u ovsr_adj=0x%04x adj_bits=%4u ",
						  baudrate, ovsr_d, best_divisor, ovsr_adj, adj_bits);

			act_divid_f = ((float)ovsr_d + ((float)adj_bits / (float)uart_speed_setting.ovsr_adj_max_bits)) * (float)best_divisor;
			actural_baud = (uint32_t)((float)sclk / (act_divid_f));
			if (actural_baud > baudrate) {
				DBG_UART_INFO("actural_baud=%8u err=%f\r\n", actural_baud, (float)((float)(actural_baud - baudrate) * 100.0f / (float)baudrate));
			} else {
				DBG_UART_INFO("actural_baud=%8u err=%f\r\n", actural_baud, (float)((float)(baudrate - actural_baud) * 100.0f / (float)baudrate));
			}

			i++;
		}

	}
}


/**
 *  @brief Configures the UART frame format.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  data_bis  The number of bits in a UART word. This value can be 7 or 8.
 *  @param[in]  parity  The parity type.
 *                      - 0: No parity bit.
 *                      - 1: Odd parity.
 *                      - 2: Even parity.
 *                      - 3: Parity bit value always is 1.
 *                      - 4: Parity bit value always is 0.
 *  @param[in]  stop_bits  The number of stop bits. This value can be 1 or 2.
 *
 *  @returns    Always return HAL_OK
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_set_format(phal_uart_adapter_t puart_adapter, uint32_t data_bits,
									 uint32_t parity, uint32_t stop_bits)
{
	UART_Type *puart;
	uart_lcr_t lcr;
	uint32_t parity_en = 0;
	uint32_t parity_type = 0;
	uint32_t stick_parity = 0;
	uint32_t frame_bits = 0;

	puart = puart_adapter->base_addr;

	lcr.w = puart->lcr;

	/* set frame length: 0/1: 7/8 */
	if (data_bits >= 8) {
		// 8 bits
		lcr.b.wls0 = 1;
		data_bits = 8;
	} else {
		// 7 bits
		lcr.b.wls0 = 0;
		data_bits = 7;
	}

	/* set stop bit: 0/1: 1/2 */
	if (stop_bits < 2) {
		// stop bit = 1
		lcr.b.stb = 0;
		stop_bits = 1;
	} else {
		// stop bit = 2
		lcr.b.stb = 1;
		stop_bits = 2;
	}

	switch (parity) {
	case UartParityNone:
		parity_en = UartParityDisable;
		stick_parity = 0;
		break;

	case UartParityOdd:
		parity_en = UartParityEnable;
		parity_type = 0;
		stick_parity = 0;
		break;

	case UartParityEven:
		parity_en = UartParityEnable;
		parity_type = 1;
		stick_parity = 0;
		break;

	case UartParityForced1:
		parity_en = UartParityEnable;
		parity_type = 1;
		stick_parity = 1;
		break;

	case UartParityForced0:
		parity_en = UartParityEnable;
		parity_type = 0;
		stick_parity = 1;
		break;

	default:
		parity_en = UartParityDisable;
		stick_parity = 0;
		parity = UartParityDisable;
		break;
	}

	/* set parity check enable: 0/1: no parity/with parity */
	lcr.b.parity_en = parity_en;
	/* set parity check type: 0/1: odd/even */
	lcr.b.even_parity_sel = parity_type;
	/* stick parity (set parity bit fixed as): 0/1 */
	lcr.b.stick_parity_en = stick_parity;

	puart->lcr = lcr.w;

	puart_adapter->word_len = data_bits;
	puart_adapter->stop_bit = stop_bits;
	puart_adapter->parity_type = parity;

	frame_bits = data_bits + 1;  // 1 start bit + data bits
	if (parity != UartParityDisable) {
		// enable line status interrupt
		puart_adapter->base_addr->ier_b.elsi = 1;
		frame_bits++;   // 1 parity bit
	}

	if ((puart_adapter->frame_bits != frame_bits) && (puart_adapter->baudrate != 0)) {
		// The frame bis changed, need to re-calculate the baud rate
		hal_rtl_uart_set_baudrate(puart_adapter, puart_adapter->baudrate);
	}
	return HAL_OK;
}

/**
 *  @brief Configures the UART hardware auto flow-control setting.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  flow_ctrl  The flow control setting.
 *                           - 0: no hardware flow control.
 *                           - 1: enable RX (RTS) flow control.
 *                           - 2: enable TX (CTS) flow control.
 *                           - 3: enable RTS and CTS hardware flow control.
 *  @param[in]  rts_pin  The RTS pin for the RX flow control.
 *  @param[in]  rts_pin  The CTS pin for the TX flow control.
 *
 *  @returns    Always return HAL_OK
 */
/*
To-Do: pin assign need to fix
*/
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_set_flow_control(phal_uart_adapter_t puart_adapter, uint32_t flow_ctrl)
{
	uart_mcr_t mcr;
	uint8_t uart_idx;

	uart_idx = puart_adapter->uart_idx;
	mcr.w = puart_adapter->base_addr->mcr;
	switch (flow_ctrl) {
	case UartFlowCtlNone:
		mcr.b.cts_en = 0;
		mcr.b.rts_en = 0;
		break;

	case UartFlowCtlRTSCTS:
		mcr.b.cts_en = 1;
		mcr.b.rts_en = 1;
		// get RTS pin and CTS pin by the UART index, RTS/CTS pin has no option (pin number is fixed for each UART)
		puart_adapter->rts_pin = uart_rts_pin_tbl[uart_idx];
		puart_adapter->cts_pin = uart_cts_pin_tbl[uart_idx];
		break;

	case UartFlowCtlRTS:
		mcr.b.cts_en = 0;
		mcr.b.rts_en = 1;
		puart_adapter->rts_pin = uart_rts_pin_tbl[uart_idx];
		break;

	case UartFlowCtlCTS:
		mcr.b.cts_en = 1;
		mcr.b.rts_en = 0;
		puart_adapter->cts_pin = uart_cts_pin_tbl[uart_idx];
		break;

	default:
		mcr.b.cts_en = 0;
		mcr.b.rts_en = 0;
		flow_ctrl = UartFlowCtlNone;
		break;
	}

	puart_adapter->base_addr->mcr = mcr.w;
	puart_adapter->flow_ctrl = flow_ctrl;

	return HAL_OK;
}

/**
 *  @brief To get the UART port TX FIFO writable status.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     value 1   TX FIFO is writable (not full).
 *  @return     value 0   TX FIFO is full.
 */
SECTION_UART_TEXT
BOOL hal_rtl_uart_writeable(phal_uart_adapter_t puart_adapter)
{
	if (puart_adapter->base_addr->tflvr_b.tx_fifo_lv < Uart_Tx_FIFO_Size) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/**
 *  @brief To send a char.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[in]  tx_data The data(1 byte) to be send.
 *
 *  @returns    void.
 */
SECTION_UART_TEXT
void hal_rtl_uart_putc(phal_uart_adapter_t puart_adapter, uint8_t tx_data)
{
	puart_adapter->base_addr->thr = tx_data;
}

/**
 *  @brief To wait TX FIFO is writable and then send a char.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[in]  tx_data The data(1 byte) to be send.
 *
 *  @returns    void.
 */
SECTION_UART_TEXT
void hal_rtl_uart_wputc(phal_uart_adapter_t puart_adapter, uint8_t tx_data)
{
	while (puart_adapter->base_addr->tflvr_b.tx_fifo_lv >= Uart_Tx_FIFO_Size);
	puart_adapter->base_addr->thr = tx_data;
}

/**
 *  @brief Waits all data in the TX FIFO are transfered.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_ms  The maximum period(in mini-second) to wait the transfer done.
 *
 *  @returns    void.
 */
SECTION_UART_TEXT
void hal_rtl_uart_wait_tx_done(phal_uart_adapter_t puart_adapter, uint32_t timeout_ms)
{
	uint32_t start_us = 0;
	uart_lsr_t lsr;

	if (timeout_ms > 0) {
		start_us = hal_read_cur_time();
	}

	lsr.w = puart_adapter->base_addr->lsr;
	puart_adapter->lsr |= lsr.w & UART_LSR_ERR;
	while (lsr.b.txfifo_empty == 0) {
		// check timeout
		if ((timeout_ms > 0) && (timeout_ms != UART_WAIT_FOREVER)) {
			if (hal_is_timeout(start_us, timeout_ms * 1000)) {
				break;
			}
		} else if (timeout_ms == 0) {
			// No wait
			break;
		}
		lsr.w = puart_adapter->base_addr->lsr;
		puart_adapter->lsr |= lsr.w & UART_LSR_ERR;
	}
}

/**
 *  @brief To send a block of data.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be transmitted.
 *  @param[in]  timeout_ms  The maximum period(in mini-secand) to wait the data transmission is finished.
 *
 *  @returns    The length of data in byte has been sent.
 */
SECTION_UART_TEXT
uint32_t hal_rtl_uart_send(phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len, uint32_t timeout_ms)
{
	UART_Type *puart = puart_adapter->base_addr;
	uint32_t start_us = 0;
	uint32_t tx_bytes = 0;
	uart_lsr_t lsr;

	if (puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) {
		DBG_UART_WARN("hal_uart_send: on Busy, State=%d\n", puart_adapter->state);
		return 0;
	}

	if ((ptx_buf == NULL) || (len == 0)) {
		puart_adapter->tx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_send: Err: ptx_buf=0x%x,  len=%d\n", ptx_buf, len);
		return 0;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_TX_BUSY);

	puart_adapter->tx_status = HAL_UART_STATUS_OK;
	if (timeout_ms > 0) {
		start_us = hal_read_cur_time();
	}

	while (tx_bytes < len) {
		if (puart->tflvr_b.tx_fifo_lv < Uart_Tx_FIFO_Size) {
			puart->thr = *((uint8_t *)(ptx_buf + tx_bytes));
			tx_bytes++;
		} else {
			// check timeout
			if ((timeout_ms > 0) && (timeout_ms != UART_WAIT_FOREVER)) {
				if (hal_is_timeout(start_us, timeout_ms * 1000)) {
					puart_adapter->tx_status = HAL_UART_STATUS_TIMEOUT;
					DBG_UART_WARN("hal_uart_send: Tx Timeout, TxCount=%d\n", tx_bytes);
					break;
				}
			} else if (timeout_ms == 0) {
				// No wait
				puart_adapter->tx_status = HAL_UART_STATUS_TIMEOUT;
				break;
			}
		}
	}

	if (tx_bytes == len) {
		// wait all data in TX FIFO be transmitted
		while (1) {
			lsr.w = puart->lsr;
			puart_adapter->lsr |= lsr.w & UART_LSR_ERR;
			if (lsr.b.txfifo_empty) {
				break;
			} else {
				if ((timeout_ms > 0) && (timeout_ms != UART_WAIT_FOREVER)) {
					if (hal_is_timeout(start_us, timeout_ms * 1000)) {
						puart_adapter->tx_status = HAL_UART_STATUS_TIMEOUT;
						break;
					}
				}
			}
		}
	}

	uart_clear_state(puart_adapter, HAL_UART_STATE_TX_BUSY);

	return tx_bytes;
}

/**
 *  @brief To send a block of data by interrupt mode.
 *         The TX FIFO will be refilled by the TX FIFO level low interrupt handler.
 *         This function returns with no waiting of the transmission done.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be send.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART TX is in busy state, previous transmission is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_int_send(phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len)
{
	UART_Type *puart = puart_adapter->base_addr;
	uint32_t fifo_size;

	if (puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) {
		DBG_UART_WARN("HalRuartIntSendRtl8195a: on Busy, State=%d\n", puart_adapter->state);
		return HAL_BUSY;
	}

	if ((ptx_buf == NULL) || (len == 0)) {
		puart_adapter->tx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_int_send: Err: pTxData=0x%x,  Length=%d\n", ptx_buf, len);
		return HAL_ERR_PARA;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_TX_BUSY);

	puart_adapter->tx_status = HAL_UART_STATUS_OK;
	puart_adapter->ptx_buf = ptx_buf;
	puart_adapter->ptx_buf_sar = ptx_buf;
	puart_adapter->tx_count = len;

	fifo_size = Uart_Tx_FIFO_Size - puart->tflvr_b.tx_fifo_lv;
	if (fifo_size > puart_adapter->tx_count) {
		fifo_size = puart_adapter->tx_count;
	}

	for (; fifo_size > 0; fifo_size--) {
		puart->thr = *((uint8_t *)(puart_adapter->ptx_buf));
		puart_adapter->tx_count--;
		puart_adapter->ptx_buf++;
	}

	if (0 == puart_adapter->tx_count) {
		// Enable Tx FIFO empty interrupt for TX done callback
		puart->ier_b.etbei = 1;
	} else {
		// Disable Tx FIFO empty interrupt
		puart->ier_b.etbei = 0;
		// Enable Tx FIFO level low interrupt to trigger re-fill TX FIFO
		puart->vier_b.tx_fifo_lv_int_en = 1;
	}

	return HAL_OK;
}


/**
 *  @brief To send a block of data by the DMA transmission mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  ptx_buf  The buffer of data to be transmitted.
 *  @param[in]  len  The length of data in bytes to be transmitted.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART TX is in busy state, previous transmission is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: Multiple-block DMA channel allocation failed.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_dma_send(phal_uart_adapter_t puart_adapter, uint8_t *ptx_buf, uint32_t len)
{
	uint32_t block_size;
	hal_gdma_adaptor_t *pgdma_chnl = puart_adapter->ptx_gdma;

	if (puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) {
		DBG_UART_WARN("hal_uart_dma_send: on Busy, State=%d\n", puart_adapter->state);
		return HAL_BUSY;
	}

	if ((ptx_buf == NULL) || (len == 0) || (pgdma_chnl == NULL)) {
		puart_adapter->tx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_dma_send: Err: pTxData=0x%x,  Length=%d GDMA_Chnl=0x%x\n", ptx_buf, len, pgdma_chnl);
		return HAL_ERR_PARA;
	}

	if (((len & 0x03) == 0) && (((uint32_t)(ptx_buf) & 0x03) == 0) &&
		(puart_adapter->tx_dma_width_1byte == 0)) {
		// 4-bytes aligned, move 4 bytes each transfer
		block_size = len >> 2;
	} else {
		block_size = len;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_DMATX_BUSY);

	puart_adapter->tx_status = HAL_UART_STATUS_OK;
	puart_adapter->ptx_buf = ptx_buf;
	puart_adapter->ptx_buf_sar = ptx_buf;
	puart_adapter->tx_count = len;

	pgdma_chnl->ch_sar = (uint32_t) ptx_buf;
	if (block_size == len) {
		// move 1 byte each transfer
		pgdma_chnl->gdma_ctl.src_msize   = MsizeFour;
		pgdma_chnl->gdma_ctl.src_tr_width = TrWidthOneByte;
		pgdma_chnl->gdma_ctl.block_size = len;
	} else {
		// 4-bytes aligned, move 4 bytes each transfer
		pgdma_chnl->gdma_ctl.src_msize   = MsizeOne;
		pgdma_chnl->gdma_ctl.src_tr_width = TrWidthFourBytes;
		pgdma_chnl->gdma_ctl.block_size = len >> 2;
	}
	pgdma_chnl->gdma_ctl.dest_msize  = MsizeFour;
	pgdma_chnl->gdma_ctl.dst_tr_width = TrWidthOneByte;

	if (puart_adapter->dcache_clean_by_addr != NULL) {
		puart_adapter->dcache_clean_by_addr((uint32_t *)ptx_buf, (int32_t)len);
	}
	if (block_size <= MAX_DMA_BLOCK_SIZE) {
		puart_adapter->base_addr->miscr_b.txdma_en = 1; // Enable UART TX DMA
		hal_rtl_gdma_transfer_start(pgdma_chnl, MultiBlkDis);
	} else if (block_size <= MAX_DMA_BLOCK_SIZE * 32) {
		// Need to use multiple block DMA
		if (pgdma_chnl->ch_num < 4) {
			// Current GDMA Channel didn't support multiple block DMA, re-allocate another one
			uart_clear_state(puart_adapter, HAL_UART_STATE_DMATX_BUSY);
			DBG_UART_ERR("hal_uart_dma_send: Err! TX %u Blk, Current DMA Chnl not support Multi-Blk\r\n", block_size);
			return HAL_NO_RESOURCE;
		}

		puart_adapter->base_addr->miscr_b.txdma_en = 1; // Enable UART TX DMA
		hal_rtl_gdma_transfer_start(pgdma_chnl, MultiBlkEn);
	} else {
		uart_clear_state(puart_adapter, HAL_UART_STATE_DMATX_BUSY);
		DBG_UART_ERR("hal_uart_dma_send: Err: TX length too big(%lu)\r\n", len);
		return HAL_ERR_PARA;
	}

	return HAL_OK;
}

/**
 *  @brief To stop and skip current on going interrupt mode or
 *         DMA mode data transmission.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns The length of data, in byte, has been transmitted.
 *           Return 0 for the case of there is no on going data transmittion.
 */
SECTION_UART_TEXT
uint32_t hal_rtl_uart_send_abort(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;
	BOOLEAN dma_tx = 0;
	uint32_t dma_tx_bytes = 0;
	hal_gdma_adaptor_t *ptx_gdma;

	if ((puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) == 0) {
		DBG_UART_INFO("hal_uart_send_abort: Not in TX state, State=%d\n", puart_adapter->state);
		return 0;
	}

	// Disable Tx FIFO empty interrupt
	puart->ier_b.etbei = 0;
	puart->vier_b.tx_fifo_lv_int_en = 0;
	if ((puart_adapter->state & HAL_UART_STATE_DMATX_BUSY) != 0) {
		dma_tx = 1;
	}

	if (dma_tx) {
		puart_adapter->base_addr->miscr_b.txdma_en = 0; // Disable UART TX DMA
		ptx_gdma = puart_adapter->ptx_gdma;
		hal_rtl_gdma_abort(ptx_gdma);
#if 0
		dma_tx_bytes = ptx_gdma->abort_recv_byte;
#else
		dma_tx_bytes = hal_rtl_gdma_query_send_bytes(ptx_gdma);
#endif
		DBG_UART_INFO("dma_tx_bytes: %lu\r\n", dma_tx_bytes);

		if (dma_tx_bytes > puart_adapter->tx_count) {
			DBG_UART_WARN("hal_uart_send_abort: DMA TXed bytes(%lu) over expected(%lu)\n", \
						  dma_tx_bytes, puart_adapter->tx_count);
			dma_tx_bytes = puart_adapter->tx_count;
		}
		puart_adapter->ptx_buf += dma_tx_bytes;
		puart_adapter->tx_count -= dma_tx_bytes;
	}

	uart_clear_state(puart_adapter, (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY));

	return (uint32_t)puart_adapter->ptx_buf - (uint32_t)puart_adapter->ptx_buf_sar;
}

/**
 *  @brief To check if any data is ready in the RX FIFO.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     TRUE    at least 1 byte of data is ready in the RX FIFO.
 *  @return     FALSE   no data in the RX FIFO.
 */
SECTION_UART_TEXT
BOOL hal_rtl_uart_readable(phal_uart_adapter_t puart_adapter)
{
	UART_Type *puart = puart_adapter->base_addr;

	if (puart->rflvr_b.rx_fifo_lv > 0) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/**
 *  @brief To read a byte of data from the RX FIFO.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    The the received data.
 */
SECTION_UART_TEXT
char hal_rtl_uart_getc(phal_uart_adapter_t puart_adapter)
{
	return (puart_adapter->base_addr->rbr_b.rxdata);
}

/**
 *  @brief To check if any data is ready in RX FIFO and read 1
 *         byte of data if data is available.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[out] data  The gotten character.
 *
 *  @return     TRUE    read RX FIFO OK.
 *  @return     FALSE   no data in the RX FIFO.
 */
SECTION_UART_TEXT
int hal_rtl_uart_rgetc(phal_uart_adapter_t puart_adapter, char *data)
{
	UART_Type *puart = puart_adapter->base_addr;

	if (puart->rflvr_b.rx_fifo_lv > 0) {
		*data = (char)puart->rbr;
		return TRUE;
	} else {
		return FALSE;
	}
}

/**
 *  @brief To receives a block of data by the polling mode.
 *
 *  @param[in]   puart_adapter  The UART adapter.
 *  @param[out]  prx_buf  The buffer for the data receiving.
 *  @param[in]   len  The length of data in byte to be received.
 *  @param[in]   timeout_ms  The maximum period(in ms) for the waiting of data receiving.
 *
 *  @returns     The length, in byte, of data has been received. Return 0 for the case of
 *               UART RX state is busy (previous receiving not didn't finished yet)
 */
SECTION_UART_TEXT
uint32_t hal_rtl_uart_recv(phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len, uint32_t timeout_ms)
{
	UART_Type *puart = puart_adapter->base_addr;
	uint32_t rx_bytes = 0;
	uint32_t start_us = 0;
	volatile uint32_t bytes_in_fifo = 0;

	if (puart_adapter->state & (HAL_UART_STATE_RX_BUSY | HAL_UART_STATE_DMARX_BUSY)) {
		DBG_UART_WARN("hal_uart_recv: on Busy, State=%d\n", puart_adapter->state);
		return 0;
	}

	if ((prx_buf == NULL) || (len == 0)) {
		puart_adapter->rx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_recv: Err: prx_buf=0x%x, len=%d\n", prx_buf, len);
		return 0;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_RX_BUSY);

	puart_adapter->rx_status = HAL_UART_STATUS_OK;

	if (timeout_ms > 0) {
		start_us = hal_read_cur_time();
	}

	while (len > 0) {
		bytes_in_fifo = puart->rflvr_b.rx_fifo_lv;
		if (bytes_in_fifo > 0) {
			if (bytes_in_fifo > len) {
				bytes_in_fifo = len;
			}

			// update LSR by interupt
			for (; bytes_in_fifo > 0; bytes_in_fifo--) {
				*(prx_buf) = (uint8_t)puart->rbr;
				prx_buf++;
				len--;
				rx_bytes++;
			}
		} else {
			// check timeout
			if ((timeout_ms > 0) && (timeout_ms != UART_WAIT_FOREVER)) {
				if (hal_is_timeout(start_us, timeout_ms * 1000)) {
					puart_adapter->rx_status = HAL_UART_STATUS_TIMEOUT;
					DBG_UART_WARN("hal_uart_recv: Rx Timeout, RxCount=%d\n", rx_bytes);
					break;
				}
			} else if (timeout_ms == 0) {
				// No wait
				puart_adapter->rx_status = HAL_UART_STATUS_TIMEOUT;
				break;
			}
		}
	}

	uart_clear_state(puart_adapter, HAL_UART_STATE_RX_BUSY);

	return rx_bytes;
}

/**
 *  @brief To receive a block of data by the interrupt mode.
 *         It read the RX FIFO in the RX FIFO level interrupr handler.
 *         This function returns without waiting of data receiving done.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[out] prx_buf The buffer for the data receiving.
 *  @param[in]  len  The length of data, in byte, are going to receive.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART RX is in busy state, previous receiving is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_int_recv(phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len)
{
	UART_Type *puart = puart_adapter->base_addr;
	uint32_t bytes_in_fifo = 0;

	if (puart_adapter->state & (HAL_UART_STATE_RX_BUSY | HAL_UART_STATE_DMARX_BUSY)) {
		DBG_UART_WARN("hal_uart_int_recv: on Busy, State=%d\n", puart_adapter->state);
		return HAL_BUSY;
	}

	if ((prx_buf == NULL) || (len == 0)) {
		puart_adapter->rx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_int_recv: Err: prx_buf=0x%x, len=%u\n", prx_buf, len);
		return HAL_ERR_PARA;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_RX_BUSY);

	puart_adapter->rx_status = HAL_UART_STATUS_OK;
	puart_adapter->prx_buf = prx_buf;
	puart_adapter->prx_buf_dar = prx_buf;
	puart_adapter->rx_count = len;

	// Could be the RX FIFO has some data already
	bytes_in_fifo = puart->rflvr;
	if (bytes_in_fifo > 0) {
		if (bytes_in_fifo > len) {
			bytes_in_fifo = len;
		}

		// update LSR by interrupt
		for (; bytes_in_fifo > 0; bytes_in_fifo--) {
			*(puart_adapter->prx_buf) = (uint8_t)puart->rbr;
			puart_adapter->rx_count--;
			puart_adapter->prx_buf++;
		}
	}

	if (puart_adapter->rx_count == 0) {
		uart_clear_state(puart_adapter, HAL_UART_STATE_RX_BUSY);

		// Call User Rx complete callback
		if (puart_adapter->rx_done_callback != NULL) {
			puart_adapter->rx_done_callback(puart_adapter->rx_done_cb_para);
		}
	} else {
		// Enable RX trigger Interrupt: data ready and line status changed(RX error)
		puart->ier_b.erbi = 1;
	}

	return HAL_OK;
}

/**
 *  @brief To receive a block of data by the DMA mode.
 *         This function returns without waiting of data receiving done.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *  @param[out] prx_buf The buffer for the data receiving.
 *  @param[in]  len  The length of data, in byte, are going to receive.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: UART RX is in busy state, previous receiving is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: Multiple-block DMA channel allocation failed.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_dma_recv(phal_uart_adapter_t puart_adapter, uint8_t *prx_buf, uint32_t len)
{
	hal_gdma_adaptor_t *pgdma_chnl = puart_adapter->prx_gdma;

	if (puart_adapter->state & (HAL_UART_STATE_RX_BUSY | HAL_UART_STATE_DMARX_BUSY)) {
		DBG_UART_WARN("hal_uart_dma_recv: on Busy, State=%d\n", puart_adapter->state);
		return HAL_BUSY;
	}

	if ((prx_buf == NULL) || (len == 0) || (pgdma_chnl == NULL)) {
		puart_adapter->rx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_UART_ERR("hal_uart_dma_recv: Err: prx_buf=0x%x, len=%u, GDMA_Chnl=0x%x\n", prx_buf, len, pgdma_chnl);
		return HAL_ERR_PARA;
	}

	uart_set_state(puart_adapter, HAL_UART_STATE_DMARX_BUSY);
	puart_adapter->rx_status = HAL_UART_STATUS_OK;
	puart_adapter->prx_buf = prx_buf;
	puart_adapter->prx_buf_dar = prx_buf;
	puart_adapter->rx_count = len;

	pgdma_chnl->ch_dar = (uint32_t) prx_buf;
	pgdma_chnl->gdma_ctl.src_msize   = MsizeFour;
	pgdma_chnl->gdma_ctl.src_tr_width = TrWidthOneByte;
	pgdma_chnl->gdma_ctl.block_size = len;



//    if (((len & 0x03) == 0) && (((uint32_t)(prx_buf) & 0x03) == 0)) {
	if ((len >= 4) && (puart_adapter->rx_dma_width_1byte == 0)) {
		// 4-bytes aligned, move 4 bytes each transfer
		pgdma_chnl->gdma_ctl.dest_msize  = MsizeOne;
		pgdma_chnl->gdma_ctl.dst_tr_width = TrWidthFourBytes;
	} else {
		// move 1 byte each transfer
		pgdma_chnl->gdma_ctl.dest_msize  = MsizeFour;
		pgdma_chnl->gdma_ctl.dst_tr_width = TrWidthOneByte;
	}

	if (len <= MAX_DMA_BLOCK_SIZE) {
		puart_adapter->base_addr->miscr_b.rxdma_en = 1; // Enable UART RX DMA
		hal_rtl_gdma_transfer_start(pgdma_chnl, MultiBlkDis);
	} else if (len <= MAX_DMA_BLOCK_SIZE * 32) {
		// Need to use multiple block DMA
		if (pgdma_chnl->ch_num < 4) {
			// Current GDMA Channel didn't support multiple block DMA, re-allocate another one
			uart_clear_state(puart_adapter, HAL_UART_STATE_DMARX_BUSY);
			DBG_UART_ERR("hal_uart_dma_recv: Err! RX %u, Current DMA Chnl not support Multi-Blk!\r\n", len);
			return HAL_NO_RESOURCE;
		}

		puart_adapter->base_addr->miscr_b.rxdma_en = 1; // Enable UART RX DMA
		hal_rtl_gdma_transfer_start(pgdma_chnl, MultiBlkEn);
	} else {
		uart_clear_state(puart_adapter, HAL_UART_STATE_DMARX_BUSY);
		DBG_UART_ERR("hal_uart_dma_recv: Err: RX Len(%lu) too big\n", len);
		return HAL_ERR_PARA;
	}

	return HAL_OK;
}

/**
 *  @brief To abort an on going UART RX transfer.
 *
 *  @param[in]  puart_adapter The UART adapter.
 *
 *  @return     value 0  There is no on going UART data receiving.
 *  @return     value > 0  The data length, in byte, has been received.
 */
SECTION_UART_TEXT
uint32_t hal_rtl_uart_recv_abort(phal_uart_adapter_t puart_adapter)
{
//    PUART_DMA_CONFIG pUartGdmaConfig;
	UART_Type *puart = puart_adapter->base_addr;
	uint32_t dma_rx_bytes;
	uint32_t bytes_in_fifo;
	BOOLEAN dma_rx = 0;
	hal_gdma_adaptor_t *prx_gdma;

	if ((puart_adapter->state & (HAL_UART_STATE_RX_BUSY | HAL_UART_STATE_DMARX_BUSY)) == 0) {
		DBG_UART_INFO("hal_uart_recv_abort: Not in RX state, State=%d\n",  puart_adapter->state);
		return 0;
	}

	if ((puart_adapter->state & HAL_UART_STATE_DMARX_BUSY) != 0) {
		dma_rx = 1;
	}

	// Disable Rx interrupt
	if (puart_adapter->rx_dr_callback == NULL) {
		puart->ier_b.erbi = 0;
	}

	// TODO: DMA RX Abort
	if (dma_rx) {
		puart_adapter->base_addr->miscr_b.rxdma_en = 0; // Disable UART RX DMA
		prx_gdma = puart_adapter->prx_gdma;
		hal_rtl_gdma_abort(prx_gdma);
#if 0
		dma_rx_bytes = prx_gdma->abort_recv_byte;
#else
		dma_rx_bytes = hal_rtl_gdma_query_recv_bytes(prx_gdma);
#endif

		if (dma_rx_bytes > puart_adapter->rx_count) {
			DBG_UART_WARN("hal_uart_recv_abort: DMA RXed bytes(%lu) over expected(%lu)\n", \
						  dma_rx_bytes, puart_adapter->rx_count);
			dma_rx_bytes = puart_adapter->rx_count;
		}
		// D-Cache sync (Invalidate)
		if (puart_adapter->dcache_invalidate_by_addr != NULL) {
			puart_adapter->dcache_invalidate_by_addr((uint32_t *)puart_adapter->prx_buf_dar, (int32_t)dma_rx_bytes);
		}

		puart_adapter->prx_buf += dma_rx_bytes;
		puart_adapter->rx_count -= dma_rx_bytes;

		if (puart_adapter->rx_count > 0) {
			bytes_in_fifo = puart->rflvr;
			if (bytes_in_fifo > 0) {
				if (bytes_in_fifo > puart_adapter->rx_count) {
					bytes_in_fifo = puart_adapter->rx_count;
				}

				// update LSR by interupt
				for (; bytes_in_fifo > 0; bytes_in_fifo--) {
					*(puart_adapter->prx_buf) = (uint8_t)puart->rbr;
					puart_adapter->rx_count--;
					puart_adapter->prx_buf++;
				}
			}
		}
	}

	uart_clear_state(puart_adapter, (HAL_UART_STATE_RX_BUSY | HAL_UART_STATE_DMARX_BUSY));

	return ((uint32_t)puart_adapter->prx_buf - (uint32_t)puart_adapter->prx_buf_dar);
}

/**
 *  @brief Query the UART index by a given pin name and a pin map table.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *  @param[in]  pin_map  The pin map table.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t _hal_uart_pin_to_idx(uint32_t pin_name, phal_pin_map pin_map)
{
	uint8_t uart_idx;

	uart_idx = 0xFF;
	while (pin_map->pin_name != 0xFF) {
		if (pin_name == pin_map->pin_name) {
			uart_idx = pin_map->peripheral_id - PID_UART0;
			break;
		} else {
			pin_map++;
		}
	}

	return uart_idx;
}

/**
 *  @brief Query the UART index by a given pin name and its pin function.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *  @param[in]  pin_type  The function type of the pin.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t hal_rtl_uart_pin_to_idx(uint32_t pin_name, uart_pin_func_t pin_type)
{
	phal_pin_map pmap_tbl;

	switch (pin_type) {
	case UART_Pin_TX:
		pmap_tbl = (phal_pin_map)&uart_tx_pin_map[0];
		break;

	case UART_Pin_RX:
		pmap_tbl = (phal_pin_map)&uart_rx_pin_map[0];
		break;

	case UART_Pin_RTS:
		pmap_tbl = (phal_pin_map)&uart_rts_pin_map[0];
		break;

	case UART_Pin_CTS:
		pmap_tbl = (phal_pin_map)&uart_cts_pin_map[0];
		break;

	default:
		// invalid pin type
		return 0xFF;
	}

	return _hal_uart_pin_to_idx(pin_name, pmap_tbl);
}

/**
 *  @brief Query the UART index by a given pin name of an UART RX pin.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t hal_rtl_uart_rx_pin_to_idx(uint32_t pin_name)
{
	return _hal_uart_pin_to_idx(pin_name, (phal_pin_map)&uart_rx_pin_map[0]);
}

/**
 *  @brief Query the UART index by a given pin name of an UART TX pin.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t hal_rtl_uart_tx_pin_to_idx(uint32_t pin_name)
{
	return _hal_uart_pin_to_idx(pin_name, (phal_pin_map)&uart_tx_pin_map[0]);
}

/**
 *  @brief Query the UART index by a given pin name of an UART RTS pin.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t hal_rtl_uart_rts_pin_to_idx(uint32_t pin_name)
{
	return _hal_uart_pin_to_idx(pin_name, (phal_pin_map)&uart_rts_pin_map[0]);
}

/**
 *  @brief Query the UART index by a given pin name of an UART CTS pin.
 *
 *  @param[in]  pin_name  The pin name be used to query the corresponding UART index.
 *
 *  @returns    The UART index. If the given pin name didn't map to a valid UART, the return value is 0xFF.
 */
SECTION_UART_TEXT
uint8_t hal_rtl_uart_cts_pin_to_idx(uint32_t pin_name)
{
	return _hal_uart_pin_to_idx(pin_name, (phal_pin_map)&uart_cts_pin_map[0]);
}

/**
 *  @brief Init default UART register state.
 *
 *  @param[in]  puart_adapter   The UART adapter.
 *  @param[in]  pconfig  The extra UART port setting for the initial configuration.
 *                       This is an UART adapter initial value. If this value is not NULL,
 *                       the initialization function will initial the new UART adapter by
 *                       this initial value. And also will do further configure, configures
 *                       the bard rate, hardware flow control and the frame format.
 *  @return     HAL_OK: UART set    default state OK!
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_set_default_state(phal_uart_adapter_t puart_adapter, phal_uart_defconfig_t pconfig)
{

	/* Get UART reg type*/
	UART_Type *puart = puart_adapter->base_addr;
	hal_status_t ret = HAL_OK;

	/* Reset RX FIFO */
	hal_rtl_uart_reset_rx_fifo(puart_adapter);

	/* Disable all interrupts */
	puart->ier = 0;
//    puart->ier_b.elsi = 1;  // always enable line status error interrupt

	/* clear vendor interrupt status */
	puart->visr = 0xFF; // clear all
	puart->vier = 0; // disable all

	/* Set Baudrate Division */
	if ((!puart_adapter->is_inited) && (pconfig != NULL)) {
		hal_rtl_uart_set_format(puart_adapter, puart_adapter->word_len,
								puart_adapter->parity_type, puart_adapter->stop_bit);
		hal_rtl_uart_set_baudrate(puart_adapter, puart_adapter->baudrate);
		/* Configure FlowControl */
		ret = hal_rtl_uart_set_flow_control(puart_adapter, puart_adapter->flow_ctrl);
	}

	/**
	  * Clean Rx break signal interrupt status at initial stage.
	  */
	puart->scr_b.rx_break_int_sts = 1;

	/* Need to assert RTS during initial stage. */
	hal_rtl_uart_set_rts(puart_adapter, 1);

	return ret;
}

/**
 *  @brief Select low-power UART SCLK, and setting default baud-rate table.
 *
 *  @param[in]  puart_adapter   The UART adapter.
 *  @param[in]  sclk_sel   The low-power UART SCLK source.
 *
 *  @return     HAL_ERR_PARA:  Input UART ID are invalid.
 *  @return     HAL_OK: Low-power UART SCLK select OK!
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_lp_sclk_select(phal_uart_adapter_t puart_adapter, uint8_t sclk_sel)
{

	hal_status_t ret = HAL_OK;

#if !defined(CONFIG_BUILD_NONSECURE)
	AON_TypeDef *aon_obj = AON;

	if (Uart0 != puart_adapter->uart_idx) {
		DBG_UART_ERR("%s: UART%d can not select SCLK\r\n", __func__, puart_adapter->uart_idx);
		return HAL_ERR_PARA;
	}

	/* Disable func/bus/pclk/sclk */
	hal_rtl_uart_en_ctrl(puart_adapter->uart_idx, OFF);

	switch (sclk_sel) {
	case UART_IRC_4M:
		puart_adapter->pdef_ovsr_tbl = def_ovsr_4m;
		puart_adapter->pdef_div_tbl = def_div_4m;
		puart_adapter->pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_4m;
		puart_adapter->pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_4m;
		puart_adapter->pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_4m;
		puart_adapter->uart_sclk = UartSCLK_4M;
		hal_rtl_sys_set_clk(UART0_SYS, UART_IRC_4M);
		break;

	case UART_XTAL:
		if (0x0 == aon_obj->AON_REG_AON_OTP_SYSCFG5) {
			//Xtal = 40MHz
			puart_adapter->pdef_ovsr_tbl = def_ovsr_40m;
			puart_adapter->pdef_div_tbl = def_div_40m;
			puart_adapter->pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_40m;
			puart_adapter->pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_40m;
			puart_adapter->pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_40m;
			puart_adapter->uart_sclk = UartSCLK_40M;
		} else if (0x5 == aon_obj->AON_REG_AON_OTP_SYSCFG5) {
			//Xtal = 26MHz
			puart_adapter->pdef_ovsr_tbl = def_ovsr_26m;
			puart_adapter->pdef_div_tbl = def_div_26m;
			puart_adapter->pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_26m;
			puart_adapter->pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_26m;
			puart_adapter->pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_26m;
			puart_adapter->uart_sclk = UartSCLK_26M;
		}
		hal_rtl_sys_set_clk(UART0_SYS, UART_XTAL);
		break;

	case UART_PERI_40M:
	default:
		puart_adapter->pdef_ovsr_tbl = def_ovsr_40m;
		puart_adapter->pdef_div_tbl = def_div_40m;
		puart_adapter->pdef_ovsradjbit_tbl10 = def_ovsr_adj_bit_10b_40m;
		puart_adapter->pdef_ovsradjbit_tbl9 = def_ovsr_adj_bit_9b_40m;
		puart_adapter->pdef_ovsradjbit_tbl8 = def_ovsr_adj_bit_8b_40m;
		puart_adapter->uart_sclk = UartSCLK_40M;
		hal_rtl_sys_set_clk(UART0_SYS, UART_PERI_40M);
		break;
	}

	/* Enable func/bus/pclk/sclk */
	hal_rtl_uart_en_ctrl(puart_adapter->uart_idx, ON);

	/*  Set UART register default state*/
	hal_rtl_uart_set_default_state(puart_adapter, NULL);

	DBG_UART_INFO("%s: UART%d SCLK= %d\r\n", __func__, puart_adapter->uart_idx, puart_adapter->uart_sclk);
#else
	ret = HAL_NO_RESOURCE;
#endif
	return ret;
}

SECTION_UART_TEXT
hal_status_t hal_rtl_uart_load_default_state(phal_uart_adapter_t puart_adapter, phal_uart_defconfig_t pconfig)
{

	hal_status_t ret = HAL_OK;
	/*  Set UART register default state*/
	ret = hal_rtl_uart_set_default_state(puart_adapter, pconfig);

	puart_adapter->state = 0;
	puart_adapter->is_inited = 1;
	return ret;
}


/**
 *  @brief To initial a UART port adapter. This function must be called before any UART port
 *         operation. This function will do:
 *           - enable the UART hardware.
 *           - register the interrupt handler.
 *           - configures the pin mux.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  uart_idx The UART index. The value can be 0 .. 2.
 *  @param[in]  pin_sel  The pin mux selection.
 *  @param[in]  pconfig  The extra UART port setting for the initial configuration.
 *                       This is an UART adapter initial value. If this value is not NULL,
 *                       the initialization function will initial the new UART adapter by
 *                       this initial value. And also will do further configure, configures
 *                       the bard rate, hardware flow control and the frame format.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  UART port initialization OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_init(phal_uart_adapter_t puart_adapter, uint32_t tx_pin, uint32_t rx_pin,
							   phal_uart_defconfig_t pconfig)
{
	/* Get UART reg type*/
	UART_Type *puart;
	hal_status_t ret = HAL_OK;
	uint8_t uart_idx = 0xFF;

	/* Set UART ID*/
	if (tx_pin != PIN_NC) {
		uart_idx = hal_rtl_uart_pin_to_idx(tx_pin, UART_Pin_TX);
		if (uart_idx >= MAX_UART_PORT) {
			DBG_UART_ERR("%s: pin(0x%x) is not for UART TX\r\n", __func__, tx_pin);
			return HAL_ERR_PARA;
		}
	}

	if (rx_pin != PIN_NC) {
		if (uart_idx != 0xFF) {
			if (uart_idx != hal_rtl_uart_pin_to_idx(rx_pin, UART_Pin_RX)) {
				DBG_UART_ERR("%s:tx_pin(0x%x) & rx_pin(0x%x) is not on the same UART\r\n", __func__, tx_pin, rx_pin);
				return HAL_ERR_PARA;
			}
		} else {
			uart_idx = hal_rtl_uart_pin_to_idx(rx_pin, UART_Pin_RX);
			if (uart_idx >= MAX_UART_PORT) {
				DBG_UART_ERR("%s: pin(0x%x) is not for UART RX\r\n", __func__, rx_pin);
				return HAL_ERR_PARA;
			}
		}
	}

	DBG_UART_INFO("hal_uart_init: UART%u\r\n", uart_idx);
	if (uart_idx >= MaxUartNum) {
		return HAL_ERR_PARA;
	}

	hal_rtl_uart_adapter_init(puart_adapter, (uint8_t)uart_idx, pconfig);
	puart_adapter->tx_pin = tx_pin;
	puart_adapter->rx_pin = rx_pin;
	puart_adapter->rts_pin = PIN_NC;
	puart_adapter->cts_pin = PIN_NC;

	puart = puart_adapter->base_addr;
	if (puart == (UART_Type *)0) {
		// error: the adapter did not initial yet!!
		return HAL_ERR_PARA;
	}

	/* Register UART IRQ handler*/
	hal_rtl_uart_reg_irq(puart_adapter, (irq_handler_t)_uart_irq_handler);

#if !defined(CONFIG_BUILD_NONSECURE)
	if (Uart0 == puart_adapter->uart_idx) {
		//UART0 default SCLK= 40MHz
		hal_rtl_sys_set_clk(UART0_SYS, UART_PERI_40M);
	}
	/* Enable func/bus/pclk/sclk */
	hal_rtl_uart_en_ctrl(puart_adapter->uart_idx, ON);
	hal_rtl_uart_load_default_state(puart_adapter, pconfig);
#else
	ret = HAL_NOT_READY;
#endif
	return ret;
}

/**
 *  @brief To initial a GDMA channel for the UART TX DMA mode transfer.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pgdma_chnl The GDMA channel adapter. It is use to control
 *              the GDMA channel transfer.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_tx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl)
{
	uint32_t dst_per = 0;

	if ((NULL == puart_adapter) || (NULL == pgdma_chnl)) {
		DBG_UART_ERR("hal_uart_tx_gdma_init: puart_adapter=0x%x pgdma_chnl=0x%x\n", puart_adapter, pgdma_chnl);
		return HAL_ERR_PARA;
	}

//    _memset((void *)pgdma_chnl, 0, sizeof(hal_gdma_adaptor_t));

	// set burst size: how many space in TX FIFO will trigger DMA req
	if (puart_adapter->tx_dma_burst_size == 0) {
		// didn't assign the TX DMA burst size, use TX FIFO trigger level to configure the TX burst size
		if (puart_adapter->base_addr->stsr_b.txfifo_low_level_status == 0) {
			puart_adapter->tx_dma_burst_size = Uart_Tx_FIFO_Size - 4;
		} else {
			puart_adapter->tx_dma_burst_size = Uart_Tx_FIFO_Size - 8;
		}
	}
	puart_adapter->base_addr->miscr_b.txdma_burstsize = puart_adapter->tx_dma_burst_size;
	puart_adapter->base_addr->fcr_b.dma_mode = 1;
	puart_adapter->base_addr->miscr_b.txdma_en = 0; // disable TX DMA at initial

	if (puart_adapter->uart_idx > Uart4) {
		DBG_UART_ERR("hal_uart_tx_gdma_init: Invalid UART Idx(%u), didn't initial? \r\n", puart_adapter->uart_idx);
		return HAL_ERR_PARA;
	}
	dst_per = uart_tx_gdma_hsk_id_tbl[puart_adapter->uart_idx];

	pgdma_chnl->gdma_ctl.tt_fc      = TTFCMemToPeri;
	//pgdma_chnl->gdma_cfg.reload_dst = 1;
	pgdma_chnl->gdma_cfg.dest_per   = (u8)dst_per;
	pgdma_chnl->ch_dar = (u32) & (puart_adapter->base_addr->thr);
	pgdma_chnl->gdma_isr_type = (TransferType | ErrType);
	pgdma_chnl->gdma_ctl.int_en      = 1;
	pgdma_chnl->gdma_ctl.dinc = NoChange;
	pgdma_chnl->gdma_ctl.sinc = IncType;
	pgdma_chnl->have_chnl = 0;

	hal_rtl_gdma_handshake_init(pgdma_chnl, (uint8_t)dst_per);
	hal_rtl_gdma_irq_reg(pgdma_chnl, (irq_handler_t)_uart_tx_dma_irq_handler, puart_adapter);
	puart_adapter->ptx_gdma = pgdma_chnl;
	return HAL_OK;
}

/**
 *  @brief To de-initial the UART TX GDMA channel.
 *         Also will disable the UART TX DMA transfer mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel de-initialization OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_tx_gdma_deinit(phal_uart_adapter_t puart_adapter)
{
	if (NULL == puart_adapter) {
		DBG_UART_ERR("hal_uart_tx_gdma_deinit: puart_adapter = NULL\n");
		return HAL_ERR_PARA;
	}

	if (NULL == puart_adapter->ptx_gdma) {
		DBG_UART_ERR("hal_uart_tx_gdma_deinit: ptx_gdma = NULL\n");
		return HAL_ERR_PARA;
	}

	puart_adapter->ptx_gdma = NULL;
	puart_adapter->base_addr->miscr_b.txdma_en = 0; // disable TX DMA at initial

	return HAL_OK;
}

/**
 *  @brief To initial a GDMA channel for the UART RX DMA mode transfer.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pgdma_chnl The GDMA channel adapter. It is use to control
 *              the GDMA channel transfer.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_rx_gdma_init(phal_uart_adapter_t puart_adapter, phal_gdma_adaptor_t pgdma_chnl)
{
	uint32_t src_per;
	uint32_t burst_size;

	if ((NULL == puart_adapter) || (NULL == pgdma_chnl)) {
		DBG_UART_ERR("hal_uart_rx_gdma_init: puart_adapter=0x%x pgdma_chnl=0x%x\n", puart_adapter, pgdma_chnl);
		return HAL_ERR_PARA;
	}

	if (puart_adapter->rx_dma_burst_size == 0) {
		// didn't assign the TX DMA burst size
		// use rx FIFO trigger level to set the DMA burst size
		switch (puart_adapter->base_addr->stsr_b.rxfifo_trigger_level_status) {
		case UartRxFifoLev_1byte:
			burst_size = 4;
			break;

		case UartRxFifoLev_8bytes:
			burst_size = 8;
			break;

		case UartRxFifoLev_16bytes:
			burst_size = 16;
			break;

		case UartRxFifoLev_28bytes:
			burst_size = 28;
			break;
		}
		puart_adapter->rx_dma_burst_size = (uint8_t)burst_size;
	}
	// how many bytes in RX FIFO will trigger DMA to read RX FIFO
	puart_adapter->base_addr->miscr_b.rxdma_burstsize = puart_adapter->rx_dma_burst_size;
	puart_adapter->base_addr->fcr_b.dma_mode = 1;
	puart_adapter->base_addr->miscr_b.rxdma_en = 0; // disable RX DMA at initial

	if (puart_adapter->uart_idx > Uart4) {
		DBG_UART_ERR("hal_uart_rx_gdma_init: Invalid UART Idx(%u), didn't initial? \r\n", puart_adapter->uart_idx);
		return HAL_ERR_PARA;
	}
	src_per = uart_rx_gdma_hsk_id_tbl[puart_adapter->uart_idx];

	pgdma_chnl->gdma_ctl.tt_fc = TTFCPeriToMem;
	//pgdma_chnl->gdma_cfg.reload_src = 1;
	pgdma_chnl->gdma_cfg.src_per = (u8)src_per;
	pgdma_chnl->ch_sar = (u32) & (puart_adapter->base_addr->rbr);
	pgdma_chnl->gdma_isr_type = (TransferType | ErrType);
	pgdma_chnl->gdma_ctl.int_en = 1;
	pgdma_chnl->gdma_ctl.dinc = IncType;
	pgdma_chnl->gdma_ctl.sinc = NoChange;
	pgdma_chnl->have_chnl = 0;

	hal_rtl_gdma_handshake_init(pgdma_chnl, (uint8_t)src_per);
	hal_rtl_gdma_irq_reg(pgdma_chnl, (irq_handler_t)_uart_rx_dma_irq_handler, puart_adapter);
	puart_adapter->prx_gdma = pgdma_chnl;

	return HAL_OK;
}

/**
 *  @brief To de-initial the UART RX GDMA channel.
 *         Also will disable the UART RX DMA transfer mode.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel de-initialization OK.
 */
SECTION_UART_TEXT
hal_status_t hal_rtl_uart_rx_gdma_deinit(phal_uart_adapter_t puart_adapter)
{
	if (NULL == puart_adapter) {
		DBG_UART_ERR("hal_uart_rx_gdma_deinit: puart_adapter = NULL\n");
		return HAL_ERR_PARA;
	}

	if (NULL == puart_adapter->prx_gdma) {
		DBG_UART_ERR("hal_uart_rx_gdma_deinit: rx_gdma = NULL\n");
		return HAL_ERR_PARA;
	}

	puart_adapter->prx_gdma = NULL;
	puart_adapter->base_addr->miscr_b.rxdma_en = 0; // disable RX DMA

	return HAL_OK;
}


/**
 *  @brief Disable the given UART port. It will do:
 *           - disable UART hardware function.
 *           - disable UART GDMA channel.
 *           - disable UART pins.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_deinit(phal_uart_adapter_t puart_adapter)
{
	uint32_t i;

	if ((puart_adapter->is_inited == 0) ||
		(puart_adapter->uart_idx >= MaxUartNum)) {
		return;
	}

	// Wait all TX data transfered
	for (i = 0; i < 100; i++) {
		if (puart_adapter->base_addr->lsr_b.txfifo_empty) {
			break;
		} else {
			hal_delay_us(100);
		}
	}

	// Disable Interrupt
	puart_adapter->base_addr->ier = 0x00;
	puart_adapter->base_addr->vier = 0x00;
	puart_adapter->base_addr->visr = 0xFF;  // clear all pending status
	hal_rtl_uart_unreg_irq(puart_adapter);

	if (puart_adapter->ptx_gdma != NULL) {
		hal_rtl_uart_tx_gdma_deinit(puart_adapter);
	}

	if (puart_adapter->prx_gdma != NULL) {
		hal_rtl_uart_rx_gdma_deinit(puart_adapter);
	}

#if !defined(CONFIG_BUILD_NONSECURE)
	hal_rtl_uart_en_ctrl(puart_adapter->uart_idx, OFF);
#else
	if (NULL != puart_adapter->hal_sys_peripheral_en_cb) {
		puart_adapter->hal_sys_peripheral_en_cb(puart_adapter->uart_idx, OFF);
	} else {
		DBG_UART_ERR("UART%d: enable ctrl fail! fun_ptr == NULL\r\n", puart_adapter->uart_idx);
		//return HAL_ERR_PARA;
	}
#endif

	puart_adapter->is_inited = 0;
}

/**
 *  @brief Controls the RTS signal.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  rts_ctrl  The RTS signal control value.
 *                        - 1: the RTS signal will go low(notify peer can start to send data).
 *                        - 0: the RTS signal will go high(notify peer stop data sending).
 *
 *  @returns    void.
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_rts(phal_uart_adapter_t puart_adapter, BOOLEAN rts_ctrl)
{
	UART_Type *puart;

	puart = puart_adapter->base_addr;
	if (rts_ctrl) {
		// write rts bit 1 -> the RTS signal will go low -> notify peer can start to send data
		puart->mcr_b.rts = 1;
	} else {
		// write rts bit 0 -> the RTS signal will go high -> notify peer stop data sending
		puart->mcr_b.rts = 0;
	}
}

/**
 *  @brief To pause/resume the TX by control the internal CTS.
 *         It can be used to implement the software flow control(XOn/XOff).
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  cts_ctrl The internal CTS signal control value.
 *                         - 1: the TX will be paused just like the external CTS goes high.
 *                         - 0: the TX will be resumed just like the externel CTS goes low.
 *
 *  @returns    void.
 */
SECTION_UART_TEXT
void hal_rtl_uart_tx_pause(phal_uart_adapter_t puart_adapter, BOOLEAN cts_ctrl)
{
	puart_adapter->base_addr->mcr_b.sw_cts = (cts_ctrl == 0) ? 0 : 1;
}

/**
 *  @brief Initials a UART adapter contents by a given UART port configuration structure.
 *
 *  @param[in]  puart_adapter The UART adapter to be initialed.
 *  @param[in]  uart_idx  The UART index. The value can be 0 ~ 2.
 *  @param[in]  pconfig  The UART adapter configuration to be used to initial this UART adapter.
 *                       If this value is NULL, a default configuration will be applied for
 *                       the initialization.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_adapter_init(phal_uart_adapter_t puart_adapter, uint8_t uart_idx, phal_uart_defconfig_t pconfig)
{
	if (NULL == puart_adapter) {
		return;
	}

	_memset((void *)puart_adapter, 0, sizeof(hal_uart_adapter_t));

	switch (uart_idx) {
	case Uart0:
		puart_adapter->base_addr = (UART_Type *)UART0_REG_BASE;
		break;

	case Uart1:
		puart_adapter->base_addr = (UART_Type *)UART1_REG_BASE;
		break;

	case Uart2:
		puart_adapter->base_addr = (UART_Type *)UART2_REG_BASE;
		break;

	case Uart3:
		puart_adapter->base_addr = (UART_Type *)UART3_REG_BASE;
		break;

	case Uart4:
		puart_adapter->base_addr = (UART_Type *)UART4_REG_BASE;   //BTUART
		break;

	default:
		return;
	}

	puart_adapter->uart_idx = uart_idx;

	if (pconfig == NULL) {
		if (uart_idx == Uart0) {
			pconfig = (phal_uart_defconfig_t) &hal_uart_default_setting_40m;
			puart_adapter->uart_sclk = UartSCLK_40M;
		} else {
			//UART1/2/3/4 default SCLK= 50MHz
			pconfig = (phal_uart_defconfig_t) &hal_uart_default_setting_50m;
			puart_adapter->uart_sclk = UartSCLK_50M;
		}
	}

	puart_adapter->baudrate = pconfig->baudrate;
	puart_adapter->flow_ctrl = pconfig->flow_ctrl;
	puart_adapter->parity_type = pconfig->parity_type;
	puart_adapter->stop_bit = pconfig->stop_bit;
	puart_adapter->word_len = pconfig->word_len;

	puart_adapter->pdef_baudrate_tbl = pconfig->pdef_baudrate_tbl;
	puart_adapter->pdef_ovsr_tbl = pconfig->pdef_ovsr_tbl;
	puart_adapter->pdef_div_tbl = pconfig->pdef_div_tbl;
	puart_adapter->pdef_ovsradjbit_tbl10 = pconfig->pdef_ovsradjbit_tbl10;
	puart_adapter->pdef_ovsradjbit_tbl9 = pconfig->pdef_ovsradjbit_tbl9;
	puart_adapter->pdef_ovsradjbit_tbl8 = pconfig->pdef_ovsradjbit_tbl8;
	puart_adapter->pdef_ovsradj_tbl10 = pconfig->pdef_ovsradj_tbl10;
	puart_adapter->pdef_ovsradj_tbl9 = pconfig->pdef_ovsradj_tbl9;
	puart_adapter->pdef_ovsradj_tbl8 = pconfig->pdef_ovsradj_tbl8;

	_puart_adapter[uart_idx] = puart_adapter;

}

/**
 *  @brief Hooks a callback function for the UART line status error interrupt.
 *         This function will enable the line status error interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  pdata  The argument of the callback function. It is an application
 *                     priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_line_sts_hook(phal_uart_adapter_t puart_adapter, uart_lsr_callback_t pcallback, void *pdata)
{
	puart_adapter->lsr_callback = pcallback;
	puart_adapter->lsr_cb_para = pdata;

	// enable line status interrupt
	puart_adapter->base_addr->ier_b.elsi = 1;
}

/**
 *  @brief Hooks a callback function for data TX done interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  id  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *  @param[in]  event  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_txtd_hook(phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback,
							uint32_t id, uint32_t event)
{
	puart_adapter->tx_td_callback = pcallback;
	puart_adapter->tx_td_cb_id = id;
	puart_adapter->tx_td_cb_ev = event;

	// enable/disable TX SFR empty interrupt
	if (pcallback != NULL) {
		puart_adapter->base_addr->ier_b.etbei = 1;
	} else {
		if ((puart_adapter->state & (HAL_UART_STATE_TX_BUSY | HAL_UART_STATE_DMATX_BUSY)) == 0) {
			puart_adapter->base_addr->ier_b.etbei = 0;
		}
	}
}

/**
 *  @brief Hooks a callback function for RX data ready interrupt.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  id  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *  @param[in]  event  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rxind_hook(phal_uart_adapter_t puart_adapter, uart_irq_callback_t pcallback,
							 uint32_t id, uint32_t event)
{
	puart_adapter->rx_dr_callback = pcallback;
	puart_adapter->rx_dr_cb_id = id;
	puart_adapter->rx_dr_cb_ev = event;

	// enable RX data ready interrupt
	if (pcallback != NULL) {
		puart_adapter->base_addr->ier_b.erbi = 1;
	} else {
		if ((puart_adapter->state & (HAL_UART_STATE_RX_BUSY)) == 0) {
			puart_adapter->base_addr->ier_b.erbi = 0;
		}
	}
}

/**
 *  @brief Hooks a callback function for interrupt mode or DMA mode
 *         data transmission finished event.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  parg  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_txdone_hook(phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg)
{
	puart_adapter->tx_done_callback = pcallback;
	puart_adapter->tx_done_cb_para = parg;
}

/**
 *  @brief Hooks a callback function for interrupt mode or DMA mode
 *         data receiving finished event.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  parg  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rxdone_hook(phal_uart_adapter_t puart_adapter, uart_callback_t pcallback, void *parg)
{
	puart_adapter->rx_done_callback = pcallback;
	puart_adapter->rx_done_cb_para = parg;
}

/**
 *  @brief Setups and enable the UART RX match filter.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pattern  The match pattern.
 *                       - [7:0]: byte0.
 *                       - [15:8]: byte1.
 *  @param[in]  mask  The mask of the match pattern.
 *  @param[in]  mask_en The mask enable control.
 *                      value = 0   mask is disabled.
 *                      value > 0   mask is enabled.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_rx_filter_pattern(phal_uart_adapter_t puart_adapter, uint32_t pattern,
										uint32_t mask, uint32_t mask_en)
{
	UART_Type *puart = puart_adapter->base_addr;

	puart->rfmpr_b.rf_mp1 = (uint8_t)(pattern & 0xFF);
	puart->rfmpr_b.rf_mp2 = (uint8_t)((pattern >> 8) & 0xFF);
	puart->rfmvr_b.rf_mv1 = (uint8_t)(mask & 0xFF);
	puart->rfmvr_b.rf_mv2 = (uint8_t)((mask >> 8) & 0xFF);
	puart->rfcr_b.rf_mask_en = (mask_en != 0) ? 1 : 0;
}

/**
 *  @brief Configures the UART RX match filter option.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pattern_len  The match pattern length:
 *                             - 1: 1 byte.
 *                             - 2: 2 bytes.
 *  @param[in]  match_op  The match filter operation mode
 *                          - 0: AND.
 *                          - 1: OR.
 *                          - 2: XOR.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_rx_filter_op(phal_uart_adapter_t puart_adapter, uint32_t pattern_len, uint32_t match_op)
{
	UART_Type *puart = puart_adapter->base_addr;

	puart->rfcr_b.rf_len = (pattern_len == 1) ? UartRxFilter1Byte : UartRxFilter2Bytes;
	puart->rfcr_b.rf_cmp_op = match_op & 0x03;
}

/**
 *  @brief Configures the UART RX match filter timeout value.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_ms  The period to wait next incoming RX byte, unit is mini-second.
 *  @param[in]  callback  The callback function to handle the timeout event of RX filter wait next byte.
 *  @param[in]  cb_arg  The argument of the RX filter timeout callback function. It is an application
 *                      priviate data to be passed by this callback function.
 *
 *  @returns void
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_rx_filter_timeout(phal_uart_adapter_t puart_adapter, uint32_t timeout_ms,
										uart_callback_t callback, void *cb_arg)
{
	uint64_t ticks;

	ticks = (uint64_t)puart_adapter->baudrate * (uint64_t)timeout_ms / 1000;
	if (ticks > 0xFFFFF) {
		ticks = 0xFFFFF;
	} else if (ticks < 12) {
		ticks = 12;
	}
	puart_adapter->base_addr->rftor_b.rf_timeout = ticks;

	puart_adapter->rx_flt_timeout_callback = callback;
	puart_adapter->rx_flt_timeout_cb_arg = cb_arg;
	if (callback) {
		puart_adapter->base_addr->vier_b.rf_idle_timeout_int_en = 1;
	} else {
		puart_adapter->base_addr->vier_b.rf_idle_timeout_int_en = 0;
	}
}

/**
 *  @brief Enables the UART RX match filter. The RX filter should be configured
 *         before enable it. The software need to re-enable the RX filter for every
 *         RX data matching. The hardware will disable this RX filter on a matched event hit.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  callback  The callback function to handle the RX filter matched event.
 *  @param[in]  cb_arg  The argument of the RX filter matched callback function.
 *                      It is an application priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rx_filter_en(phal_uart_adapter_t puart_adapter, uart_callback_t callback, void *cb_arg)
{
	if (callback) {
		puart_adapter->rx_flt_matched_callback = callback;
		puart_adapter->rx_flt_matched_cb_arg = cb_arg;
		puart_adapter->base_addr->rfcr_b.rf_en = 1;
		puart_adapter->base_addr->vier_b.rf_patt_match_int_en = 1;   // enable RX filter matched interrupt
	} else {
		DBG_UART_ERR("hal_uart_rx_filter_en: Callback is NULL\r\n");
	}
}

/**
 *  @brief Disables the UART RX match filter.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rx_filter_dis(phal_uart_adapter_t puart_adapter)
{
	puart_adapter->base_addr->rfcr_b.rf_en = 0;
	puart_adapter->base_addr->vier_b.rf_patt_match_int_en = 0;   // disable RX filter matched interrupt
}

/**
 *  @brief Resets the UART receiver hardware state machine.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_reset_receiver(phal_uart_adapter_t puart_adapter)
{
	puart_adapter->base_addr->stsr_b.reset_rcv = 1;
	__NOP();
	__NOP();
	puart_adapter->base_addr->stsr_b.reset_rcv = 0;
}

/**
 *  @brief Setups the TX FIFO low level. The TX FIFO low interrupt will be triggered
 *         when the the data count in the TX FIFO is lower than the low level.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  low_level  The low level selection:
 *                           - value 0: 4 bytes.
 *                           - value 1: 8 bytes.
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_tx_fifo_level(phal_uart_adapter_t puart_adapter, uint32_t low_level)
{
	if (low_level <= 4) {
		puart_adapter->base_addr->fcr_b.txfifo_low_level = UartTxFifoLow_4bytes;
	} else {
		puart_adapter->base_addr->fcr_b.txfifo_low_level = UartTxFifoLow_8bytes;
	}
}

/**
 *  @brief Configures the RX FIFO high level. The RX FIFO high interrupt will be triggered
 *         when the the data count in the RX FIFO is higher than the configured level.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  level  The high level selection:
 *                     - 0: 1 byte.
 *                     - 1: 8 bytes.
 *                     - 2: 16 bytes.
 *                     - 3: 28 bytes.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_set_rx_fifo_level(phal_uart_adapter_t puart_adapter, uint32_t level)
{
	if (level >= 28) {
		puart_adapter->base_addr->fcr_b.rxfifo_trigger_level = UartRxFifoLev_28bytes;
	} else if (level >= 16) {
		puart_adapter->base_addr->fcr_b.rxfifo_trigger_level = UartRxFifoLev_16bytes;
	} else if (level >= 8) {
		puart_adapter->base_addr->fcr_b.rxfifo_trigger_level = UartRxFifoLev_8bytes;
	} else {
		puart_adapter->base_addr->fcr_b.rxfifo_trigger_level = UartRxFifoLev_1byte;
	}
}

/**
 *  @brief Enables the RX Idle timeout function. The RX idle timer will be start/re-start on
 *         every new byte receiving. If there is no new data be received in the given period
 *         the callback function will be called.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  timeout_us  The timeout period, in micro-second.
 *  @param[in]  pcallback  The call back function.
 *  @param[in]  parg  The argument for the callback function. It is an application priviate
 *                    data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rx_idle_timeout_en(phal_uart_adapter_t puart_adapter, uint32_t timeout_us,
									 uart_callback_t pcallback, void *parg)
{
	uint32_t bit_time;
	uint32_t byte_time;
	uint32_t frame_bits;
	uint32_t timeout_bytes;
	uint32_t timeout_bytes_2pwr;

	if ((puart_adapter == NULL) || (pcallback == NULL)) {
		DBG_UART_ERR("hal_uart_rx_idle_timeout_en: puart_adapter=0x%x, pcallback=0x%x\r\n", puart_adapter, pcallback);
		return;
	}

	bit_time = 1000000000 / puart_adapter->baudrate + 1; // bit time in ns
	frame_bits = puart_adapter->word_len + 1; // 1 start bit + data bits
	if (puart_adapter->parity_type != UartParityNone) {
		frame_bits++;   // 1 parity bit
	}
	byte_time = bit_time * frame_bits;
	// convert byte_time to uint us
	if ((byte_time % 1000) > 500) {
		byte_time = byte_time / 1000 + 1;
	} else {
		byte_time = byte_time / 1000;
	}
	timeout_bytes = timeout_us / byte_time;
	if (timeout_bytes == 0) {
		timeout_bytes = 1;
	}

	// timeout bytes = 2**bits_field
	timeout_bytes_2pwr = 0;
	while (timeout_bytes > 1) {
		timeout_bytes = timeout_bytes >> 1;
		timeout_bytes_2pwr++;
	}

	if (timeout_bytes_2pwr > 15) {
		timeout_bytes_2pwr = 15;
	}

	// hook call-back
	puart_adapter->rx_idle_timeout_callback = pcallback;
	puart_adapter->rx_idle_timeout_cb_arg = parg;
	// Enable RX timeout interrupt
	puart_adapter->base_addr->visr_b.rx_idle_timeout_int_sts = 1;   // clear interrrupt pending status first
	puart_adapter->base_addr->vier_b.rx_idle_timeout_int_en = 1;   // enable RX idle timeout interrupt
	// configure RX idle timeout register
	puart_adapter->base_addr->ritor_b.rxidle_timeout_value = timeout_bytes_2pwr;
	puart_adapter->base_addr->ritor_b.rx_idle_timeout_en = 1;   // enable RX idle timeout function
}

/**
 *  @brief Disables the RX idle timeout function.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_rx_idle_timeout_dis(phal_uart_adapter_t puart_adapter)
{
	if (puart_adapter == NULL) {
		DBG_UART_ERR("hal_uart_rx_idle_timeout_dis: puart_adapter = NULL\r\n");
		return;
	}

	// disable RX idle timeout function
	puart_adapter->base_addr->ritor_b.rx_idle_timeout_en = 0;   // disable RX idle timeout function

	// Disable RX timeout interrupt
	puart_adapter->base_addr->visr_b.rx_idle_timeout_int_sts = 1;   // clear interrrupt pending status first
	puart_adapter->base_addr->vier_b.rx_idle_timeout_int_en = 0;   // disable RX idle timeout interrupt

	// clear call-back
	puart_adapter->rx_idle_timeout_callback = NULL;
	puart_adapter->rx_idle_timeout_cb_arg = NULL;
}

/**
 *  @brief Hooks a callback function for interrupt mode TX FIFO level low event.
 *
 *  @param[in]  puart_adapter  The UART adapter.
 *  @param[in]  pcallback  The callback function.
 *  @param[in]  parg  The argument of the callback function. It is an application
 *                    priviate data to be passed by this callback function.
 *
 *  @returns    void
 */
SECTION_UART_TEXT
void hal_rtl_uart_tx_fifo_low_hook(phal_uart_adapter_t puart_adapter, uart_callback_t pcallback,
								   void *parg)
{
	puart_adapter->tx_fifo_low_callback = pcallback;
	puart_adapter->tx_fifo_low_cb_para = parg;
}
#endif // end of "#if CONFIG_UART_EN"
