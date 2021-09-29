/**************************************************************************//**
 * @file     cinit_start.c
 * @brief    The entry function for system eraly initialization.
 *           This make the boot loader to run a customize initialization
 *           function during booting.
 * @version  V1.00
 * @date     2018-08-08
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
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
//#include "rtl8195bhp_ramstart.h"
#include "hal.h"
#include "memory.h"
#include "diag.h"
#include "fw_img.h"
//#include "hal_lpddr.h"

extern sys_cp_fw_info_t cp_fw_info;

#if   defined ( __CC_ARM )                                            /* ARM Compiler 4/5 */
extern u8 Image$$CINIT_BSS$$ZI$$Limit[];
#define __cinit_bss_end__ Image$$CINIT_BSS$$ZI$$Limit
extern u8 Image$$CINIT_BSS$$ZI$$Base[];
#define __cinit_bss_start__ Image$$CINIT_BSS$$ZI$$Base
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern u8 Image$$CINIT_BSS$$ZI$$Limit[];
#define __cinit_bss_end__ Image$$CINIT_BSS$$ZI$$Limit
extern u8 Image$$CINIT_BSS$$ZI$$Base[];
#define __cinit_bss_start__ Image$$CINIT_BSS$$ZI$$Base
#elif defined ( __GNUC__ )
extern u8 __cinit_bss_start__[];
extern u8 __cinit_bss_end__[];
#elif defined( __ICCARM__ )
extern u8 CINIT_BSS$$Limit[];
#define __cinit_bss_end__ CINIT_BSS$$Limit
extern u8 CINIT_BSS$$Base[];
#define __cinit_bss_start__ CINIT_BSS$$Base
#endif


#if defined (CONFIG_BUILD_RAM) && !defined(CONFIG_BUILD_NONSECURE)
void cinit_start(void);

SECTION_CINIT_ENTRY_FUNC
__ALIGNED(32) CINIT_FUNCTION_START_TABLE cinit_entry_func = {
	.entry_func = cinit_start,
	.sys_cp_fw_info = (void *) &cp_fw_info,
	.init_flags.w = 0
};

/**
 *  @brief The default function for user customize initialization function which
 *         will be executed during booting. User can implement their own function to replace
 *         this function.
 *
 *  @returns void
 */
_WEAK void cinit(void)
{
//    dbg_printf("%s==>\r\n", __func__);
	return;
}

/**
 *  @brief The entry function to run the user customize initialization flow during booting.
 *
 *  @returns void
 */
void cinit_start(void)
{
	uint32_t bss_len;

	bss_len = ((u32)__cinit_bss_end__ - (u32)__cinit_bss_start__);
	if (bss_len > 0) {
		memset((void *)__cinit_bss_start__, 0, bss_len);
		dcache_clean_by_addr((uint32_t *)__cinit_bss_start__, bss_len);
		__DSB();
		__ISB();
	}

	*(hal_gtimer_stubs.ppsys_timer) = cinit_entry_func.psystimer;
#if !defined(CONFIG_BUILD_SECURE)
	*(__rom_stubs_hal_timer_s.ppsys_timer) = cinit_entry_func.psystimer;
#endif

#if defined (CONFIG_EXRAM_LPDDR_EN) && (CONFIG_EXRAM_LPDDR_EN == 1)
	if (cinit_entry_func.init_flags.b.lpddr_inited == 0) {
		hal_lpddr_init();
		cinit_entry_func.init_flags.b.lpddr_inited = 1;
	}
#endif

	cinit();
}

#endif  // end of "#if defined (CONFIG_BUILD_RAM)"

