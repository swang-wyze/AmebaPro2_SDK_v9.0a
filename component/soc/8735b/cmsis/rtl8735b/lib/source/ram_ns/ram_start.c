/**************************************************************************//**
 * @file     ram_start.c
 * @brief    The RAM code entry function. It initial the RAM memory and the platform.
 * @version  V1.00
 * @date     2021-07-17
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
#undef ROM_REGION
#include "cmsis.h"
#include "mpu.h"
#include "rtl8735b_ramstart.h"
#include "stdio_port_func.h"
#include "shell.h"
#include "cmsis_os.h"               // CMSIS RTOS header file
#include "hal.h"
#include "memory.h"
#include "diag.h"

extern hal_uart_adapter_t stdio_uart;
RAM_BSS_NOINIT_SECTION hal_spic_adaptor_t hal_spic_adaptor;
hal_spic_adaptor_t *pglob_spic_adaptor;

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
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
extern uint32_t Image$$_STACK$$Limit;
#define __StackTop Image$$_STACK$$Limit
extern uint32_t Image$$_STACK$$Base;
#define __StackLimit Image$$_STACK$$Base
extern u8 Image$$RAM_BSS$$ZI$$Limit[];
#define __bss_end__ Image$$RAM_BSS$$ZI$$Limit
extern u8 Image$$RAM_BSS$$ZI$$Base[];
#define __bss_start__ Image$$RAM_BSS$$ZI$$Base
extern uint32_t Image$$RAM_DATA$$Base;
#define __data_start__ Image$$RAM_DATA$$Base
extern uint32_t Image$$RAM_DATA$$Limit;
#define __data_end__ Image$$RAM_DATA$$Limit
extern uint32_t Load$$RAM_DATA$$Base;
#define __etext Load$$RAM_DATA$$Base
extern u8 Image$$_RAM_VECTOR$$Base[];
#define __ram_vector_start__ Image$$_RAM_VECTOR$$Base
extern u8 Image$$_RAM_VECTOR$$Limit[];
#define __ram_vector_end__ Image$$_RAM_VECTOR$$Limit
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
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
extern uint32_t Image$$_STACK$$Limit;
#define __StackTop Image$$_STACK$$Limit
extern uint32_t Image$$_STACK$$Base;
#define __StackLimit Image$$_STACK$$Base
extern u8 Image$$RAM_BSS$$ZI$$Limit[];
#define __bss_end__ Image$$RAM_BSS$$ZI$$Limit
extern u8 Image$$RAM_BSS$$ZI$$Base[];
#define __bss_start__ Image$$RAM_BSS$$ZI$$Base
extern uint32_t Image$$RAM_DATA$$Base;
#define __data_start__ Image$$RAM_DATA$$Base
extern uint32_t Image$$RAM_DATA$$Limit;
#define __data_end__ Image$$RAM_DATA$$Limit
extern uint32_t Load$$RAM_DATA$$Base;
#define __etext Load$$RAM_DATA$$Base
extern u8 Image$$_RAM_VECTOR$$Base[];
#define __ram_vector_start__ Image$$_RAM_VECTOR$$Base
extern u8 Image$$_RAM_VECTOR$$Limit[];
#define __ram_vector_end__ Image$$_RAM_VECTOR$$Limit
extern u8 Image$$_XIP_CODE$$Base[];
#define __xip_code_text_start__ Image$$_XIP_CODE$$Base
extern u8 Image$$_XIP_CODE$$Limit[];
#define __xip_code_text_end__ Image$$_XIP_CODE$$Limit
extern u8 Image$$_LPDDR_CODE$$Base[];
#define __lpddr_code_text_start__ Image$$_LPDDR_CODE$$Base
extern u8 Image$$_LPDDR_CODE$$Limit[];
#define __lpddr_code_text_end__ Image$$_LPDDR_CODE$$Limit
extern u8 Image$$_RAM_CODE$$Base[];
#define __ram_code_text_start__ Image$$_RAM_CODE$$Base
extern u8 Image$$_RAM_CODE$$Limit[];
#define __ram_code_text_end__ Image$$_RAM_CODE$$Limit
#elif defined ( __GNUC__ )
extern uint8_t __rom_bss_start__[];
extern uint8_t __rom_bss_end__[];
extern uint32_t __rom_etext;
extern uint32_t __rom_data_start__;
extern uint32_t __rom_data_end__;
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
extern u8 __bss_end__[];
extern u8 __bss_start__[];
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __etext;
extern u8 __ram_vector_start__[];
extern u8 __ram_vector_end__[];
extern u8 __eram_bss_end__[];
extern u8 __eram_bss_start__[];
extern u8 __lpddr_bss_end__[];
extern u8 __lpddr_bss_start__[];
extern u8 __psram_bss_start__[];
extern u8 __psram_bss_end__[];
extern const uint32_t __xip_code_text_start__;
extern const uint32_t __xip_code_text_end__;
extern const uint32_t __lpddr_code_text_start__;
extern const uint32_t __lpddr_code_text_end__;
extern const uint32_t __ram_code_text_start__;
extern const uint32_t __ram_code_text_end__;
#elif defined( __ICCARM__ )
extern uint32_t RAM_STACK$$Base;
#define __StackLimit RAM_STACK$$Base
extern uint32_t RAM_STACK$$Limit;
#define __StackTop RAM_STACK$$Limit
extern u8 RAM_BSS$$Limit[];
#define __bss_end__ RAM_BSS$$Limit
extern u8 RAM_BSS$$Base[];
#define __bss_start__ RAM_BSS$$Base
extern u8 ERAM_BSS$$Limit[];
#define __eram_bss_end__ ERAM_BSS$$Limit
extern u8 ERAM_BSS$$Base[];
#define __eram_bss_start__ ERAM_BSS$$Base
extern uint32_t RAM_DATA$$Base;
#define __data_start__ RAM_DATA$$Base
extern uint32_t RAM_DATA$$Limit;
#define __data_end__ RAM_DATA$$Limit
#define __etext RAM_DATA$$Base
extern uint32_t RAM_VECTOR$$Base[];
#define __ram_vector_start__ RAM_VECTOR$$Base
extern uint32_t RAM_VECTOR$$Limit[];
#define __ram_vector_end__ RAM_VECTOR$$Limit
#endif

extern void *_memset(void *dst0, int Val, SIZE_T length);
extern void app_start(void) __attribute__((noreturn));

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void os_idle_callback(void);
extern void os_error_callback(uint32_t error_code);
void hal_setup_ns_fault_back_trace_nsc(pfault_handler_back_trace_t trace_hdl);

void ram_start(void) __attribute__((noreturn));

START_RAM_FUN_A_SECTION
RAM_FUNCTION_START_TABLE gRamStartFun = {
	.RamStartFun = ram_start,
	.RamWakeupFun = ram_start,
	.RamPatchFun0 = ram_start
};

SECTION_RAM_VECTOR_TABLE int_vector_t ram_vector_table[MAX_VECTOR_TABLE_NUM] __ALIGNED(256) = {
	(int_vector_t)((uint32_t)&__StackTop),
	(int_vector_t) &gRamStartFun
};

#if defined(CONFIG_BUILD_RAM) && (CONFIG_BUILD_RAM == 1)
#ifndef __STACK_SIZE
#define   __STACK_SIZE  0x00001000
#endif
static uint8_t stack[__STACK_SIZE] __attribute__((aligned(8), used, section(".stack")));
#endif  // end of "#if defined(CONFIG_BUILD_RAM) && (CONFIG_BUILD_RAM == 1)"

hal_uart_group_adapter_t uart_gadapter;
hal_uart_adapter_t log_uart;
hal_timer_group_adapter_t timer_group3;
hal_timer_adapter_t system_timer;
hal_gdma_group_t hal_gdma_group;
//hal_syson_adapter_t syson_adapter;
hal_misc_adapter_t misc_adapter;

#if defined(HARD_FAULT_BACK_TRACE) && (HARD_FAULT_BACK_TRACE != 0)
#define FAULT_BACK_TRACE_BUF_DEPTH      16
uint32_t hard_fault_back_trace_buf[FAULT_BACK_TRACE_BUF_DEPTH];

const uint32_t hard_fault_back_trace_txt_range[] = {
	/* code start,              code end */
	0x000000c0,                 0x00007FFF,               // code in ITCM ROM
	0x10002000,                 0x100AEC00,               // cdde in ROM
	(uint32_t) &__xip_code_text_start__, (uint32_t) &__xip_code_text_end__,   // code in XIP
	(uint32_t) &__lpddr_code_text_start__, (uint32_t) &__lpddr_code_text_end__,  // code in LPDDR
	(uint32_t) &__ram_code_text_start__, (uint32_t) &__ram_code_text_end__,      // code in Internal SRAM

	0xFFFFFFFF                                          // end of list
};

const fault_handler_back_trace_t hard_fault_back_trace = {
	.msp_top = (uint32_t) &__StackTop,
	.msp_limit = (uint32_t) &__StackLimit,
	.psp_max_size = 0x2000, // the maximum PS size of all task
	.ptxt_range_list = (uint32_t *)hard_fault_back_trace_txt_range,
	.trace_depth = FAULT_BACK_TRACE_BUF_DEPTH,
	.ptrace_buf = hard_fault_back_trace_buf
};
#endif  // end of "#if defined(HARD_FAULT_BACK_TRACE) && (HARD_FAULT_BACK_TRACE != 0)"

__WEAK void log_uart_flush_wait(void)
{
	if (log_uart.is_inited == 1) {
		hal_uart_wait_tx_done(&log_uart, 10);
	}
}

__WEAK void log_uart_wputc(void *adapter, const char tx_data)
{
	hal_uart_wputc((phal_uart_adapter_t)adapter, (uint8_t) tx_data);
	if (tx_data == '\n') {
		hal_uart_wputc((phal_uart_adapter_t)adapter, (uint8_t) '\r');
	}
}

__WEAK int log_uart_rgetc(void *adapter, char *data)
{
	return (hal_uart_rgetc((phal_uart_adapter_t)adapter, (char *)data));
}

__WEAK void log_uart_port_init_ns(int log_uart_tx, int log_uart_rx, uint32_t baud_rate)
{
	hal_status_t ret;
	uint8_t uart_idx;

	/* Since the log UART pin may has been registered at Secure RAM Start, so unregister it first to prevent
	   pin confliction */
	uart_idx = hal_uart_pin_to_idx(log_uart_rx, UART_Pin_RX);
	hal_pinmux_unregister(log_uart_rx, (PID_UART0 + uart_idx));
	hal_pinmux_unregister(log_uart_tx, (PID_UART0 + uart_idx));

	//* Init the UART port hadware
	ret = hal_uart_init(&log_uart, log_uart_tx, log_uart_rx, NULL);
	if (ret == HAL_OK) {
		hal_uart_set_baudrate(&log_uart, baud_rate);
		hal_uart_set_format(&log_uart, 8, UartParityNone, 1);

		// hook the putc function to stdio port for printf
		stdio_port_init((void *)&log_uart, (stdio_putc_t)&hal_uart_wputc, (stdio_getc_t)&hal_uart_rgetc);
	}
}

/**
 *  @brief The following log_uart_port_init function is a wrapper function that contains the
 *         log_uart_port_init_ns function above. Its purpose is to make sure that it is
 *         consistent with the non-trustzone log_uart_port_init function., so that user
 *         can just call log_uart_port_init for both trustzone and non-trustzone.
 *  @returns void
 */
__WEAK void log_uart_port_init(int log_uart_tx, int log_uart_rx, uint32_t baud_rate)
{
	log_uart_port_init_ns(log_uart_tx, log_uart_rx, baud_rate);
}

/**
 *  @brief The RAM code BSS and Data initialization. All BSS data will be reset as 0.
 *         And load initial value to Data section. For Boot from flash case, the Data
 *         section should be loaded by boot loader.
 *
 *  @returns void
 */
void ram_start_bss_data_init(void)
{
	uint32_t bss_len;

	/* clear SRAM BSS */
	bss_len = ((u32)__bss_end__ - (u32)__bss_start__);
	memset((void *)__bss_start__, 0, bss_len);

	__DSB();
	__ISB();
}

#if defined (CONFIG_BUILD_RAM) || defined (CONFIG_BUILD_ALL)
/**
 *  @brief The external DRAM (PSRAM or LPDDR) code BSS and Data initialization. All BSS data will be reset as 0.
 *         And load initial value to Data section. For Boot from flash case, the Data
 *         section should be loaded by boot loader.
 *
 *  @returns void
 */
void ram_start_exram_bss_init(void)
{
	u32 bss_len, bss_start;

	(void)bss_len;


#if defined (CONFIG_EXRAM_LPDDR_EN) && (CONFIG_EXRAM_LPDDR_EN == 1)
	/* clear DDR BSS */
	bss_len = ((u32)__eram_bss_end__ - (u32)__eram_bss_start__);
	if (bss_len > 0) {
		memset((void *)__eram_bss_start__, 0, bss_len);
		dcache_clean_by_addr((uint32_t *)__eram_bss_start__, bss_len);
	}
#endif

#if defined (CONFIG_BUILD_ALL)
#if !(defined (CONFIG_BUILD_SECURE) || defined (CONFIG_BUILD_NONSECURE))
	// Clear DDR BSS for VOE
	bss_start = (u32)__eram_voe_bss_start__;
	bss_len = (u32)__eram_voe_bss_end__ - bss_start;
	if (bss_len > 0) {
		memset((void *)bss_start, 0, bss_len);
		dcache_clean_by_addr((u32 *)bss_start, bss_len);
	}
#endif
#endif
	__DSB();
	__ISB();
}
#endif  // end of "#if defined (CONFIG_BUILD_RAM)"

/**
 *  @brief The RAM code entry function. It will do memory initialization,
 *         peripheral initializaion, application initialization and then jump
 *         to the application start function.
 *
 *  @returns void
 */
void ram_start(void)
{
	//uint32_t sys_timer_id;

	SystemInit();

#if CONFIG_RAM_START_CACHE_EN
	// enable NS I-cache
	icache_enable();
#endif

	ram_start_bss_data_init();
	ram_start_exram_bss_init();

	// Initial IRQ function table: NULL: to use default IRQ API
	hal_irq_api_init(NULL);
	hal_vector_table_init((uint32_t)&__StackTop, ram_vector_table);
	//dcache_clean_by_addr((uint32_t *)ram_vector_table, sizeof(ram_vector_table));

	// re-initial system timer
	hal_timer_clock_init(3, ENABLE);
	hal_timer_group_init(&timer_group3, 3);
	hal_timer_group_sclk_sel(&timer_group3, GTimerSClk_31_25M);
	hal_start_systimer(&system_timer, CONFIG_SYS_TIMER_ID, GTimerCountUp, CONFIG_SYS_TICK_TIME, 3);

	// Re-initial UART for debugging message (Todo:Make users define baudrate from Flash sys_data)
	log_uart_port_init(STDIO_UART_TX_PIN, STDIO_UART_RX_PIN, (uint32_t)115200);
	dbg_printf("RAM Start(Non-Secure), Build @ %s, %s\r\n", __TIME__, __DATE__);
#if 1
	// GDMA init
	hal_gdma_group_init(&hal_gdma_group);
#endif

#if 0
	// syson or misc init
	hal_syson_init(&syson_adapter);
	hal_misc_init(&misc_adapter);
#endif

	/* Configure MPU regions: Regions configuration is in "mpu_config_ns.h"
	*/
	mpu_init();

#if CONFIG_RAM_START_CACHE_EN
	// enable NS D-Cache
	dcache_enable();
#endif


	// initial global variable
	ConfigDebugErr = 0xFFFFFFFF;
	ConfigDebugWarn = 0;
	ConfigDebugInfo = 0;

	hal_irq_set_vector(SVCall_IRQn, (uint32_t)vPortSVCHandler);
	hal_irq_set_vector(PendSV_IRQn, (uint32_t)xPortPendSVHandler);
	hal_irq_set_vector(SysTick_IRQn, (uint32_t)xPortSysTickHandler);

	hal_irq_set_priority(SVCall_IRQn, 0);
	hal_irq_set_priority(PendSV_IRQn, 15);
	hal_irq_set_priority(SysTick_IRQn, 15);

#if 0
	// Re-init irq api using ram code
	hal_irq_api_init(&sys_irq_api);
#endif
	__set_MSPLIM((uint32_t)&__StackLimit);

	__DSB();
	__ISB();

#if defined(HARD_FAULT_BACK_TRACE) && (HARD_FAULT_BACK_TRACE != 0)
	*hal_int_vector_stubs.ppbk_trace_hdl = (pfault_handler_back_trace_t)&hard_fault_back_trace;
	hal_setup_ns_fault_back_trace_nsc((pfault_handler_back_trace_t)&hard_fault_back_trace);
#endif

	// Jump to application
	app_start();
}

