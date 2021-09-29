/**************************************************************************//**
 * @file     startup.c
 * @brief    The System startup and HAL initialization for rtl8710c.
 * @version  V1.00
 * @date     2016-07-20
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
#include "stdio_port.h"

#define SECTION_SYSSTART_TEXT                 SECTION(".sysstart.text")
#define SECTION_SYSSTART_BSS                  SECTION(".sysstart.bss")

extern uint32_t __Vectors_NS;
extern uint32_t ConfigDebugErr;
extern uint32_t ConfigDebugWarn;
extern uint32_t ConfigDebugInfo;

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
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
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
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
#elif defined ( __GNUC__ )
extern uint32_t __StackTop;
extern uint8_t __rom_bss_start__[];
extern uint8_t __rom_bss_end__[];
extern uint32_t __rom_etext;
extern uint32_t __rom_data_start__;
extern uint32_t __rom_data_end__;
extern uint8_t __rom_temp_bss_start__[];
extern uint8_t __rom_temp_bss_end__[];
#endif

extern void *_memset(void *dst0, int Val, SIZE_T length);

SECTION_ROM_TEMP_BSS hal_uart_adapter_t stdio_uart;
//extern const uint8_t uart_tr_pin[];

SECTION_SYSSTART_TEXT
void start_ns_rtl8735b(void)
{
	uint32_t *pSrc, *pDest;
	uint32_t bss_len;

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1)
	SCB_NS->VTOR = (uint32_t) &__Vectors_NS;
#endif

	// Clear BSS for ROM code
	bss_len = (__rom_bss_end__ - __rom_bss_start__);
	_memset((void *)__rom_bss_start__, 0, bss_len);

	bss_len = (__rom_temp_bss_end__ - __rom_temp_bss_start__);
	_memset((void *)__rom_temp_bss_start__, 0, bss_len);

	// initial ROM .data
	pSrc  = &__rom_etext;
	pDest = &__rom_data_start__;

	for (; pDest < &__rom_data_end__ ;) {
		*pDest++ = *pSrc++;
	}

	// Initial IRQ function table to use default API
	hal_rtl_irq_api_init(NULL);

	// initial global variable
	ConfigDebugErr = 0xFFFFFFFF;
	ConfigDebugWarn = 0;
	ConfigDebugInfo = 0;
}

