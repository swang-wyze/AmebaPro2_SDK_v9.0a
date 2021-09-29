/**************************************************************************//**
 * @file     rtl8735b_cache.c
 * @brief    This file implements the Cache HW control function.
 *
 * @version  V1.00
 * @date     2017-02-02
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
#if CONFIG_CACHE_EN
#include "rtl8735b_symbns4s.h"

#define SECTION_CACHE_TEXT           SECTION(".rom.hal_cache.text")
#define SECTION_CACHE_DATA           SECTION(".rom.hal_cache.data")
#define SECTION_CACHE_RODATA         SECTION(".rom.hal_cache.rodata")
#define SECTION_CACHE_BSS            SECTION(".rom.hal_cache.bss")
#define SECTION_CACHE_STUBS          SECTION(".rom.hal_cache.stubs")

/**
* @addtogroup rtl8735b_cache Cache
* @{
*/

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_cache_rom_func Cache control ROM APIs.
 * @ingroup hs_hal_cache
 * @{
 * @brief The APIs for the cache configure and cache data sync.
 * @details The user application(in RAM space) should not call these APIs directly.
 *          There is another RAM base of Cache APIs is provided for the user application.
 */

/**
  * @brief The stubs functions table to exports Cache configure and control functions in ROM.
  */
SECTION_CACHE_STUBS const hal_cache_func_stubs_t hal_cache_stubs = {
	.icache_enable = rtl_icache_enable,
	.icache_disable = rtl_icache_disable,
	.icache_invalidate = rtl_icache_invalidate,
	.dcache_enable = rtl_dcache_enable,
	.dcache_disable = rtl_dcache_disable,
	.dcache_invalidate = rtl_dcache_invalidate,
	.dcache_clean = rtl_dcache_clean,
	.dcache_clean_invalidate = rtl_dcache_clean_invalidate,
	.dcache_invalidate_by_addr = rtl_dcache_invalidate_by_addr,
	.dcache_clean_by_addr = rtl_dcache_clean_by_addr,
	.dcache_clean_invalidate_by_addr = rtl_dcache_clean_invalidate_by_addr
};


/**
  \brief   Enable I-Cache
  \details Turns on I-Cache
  */
SECTION_CACHE_TEXT
void rtl_icache_enable(void)
{
	SCB_EnableICache();
}


/**
  \brief   Disable I-Cache
  \details Turns off I-Cache
  */
SECTION_CACHE_TEXT
void rtl_icache_disable(void)
{
	SCB_DisableICache();
}


/**
  \brief   Invalidate I-Cache
  \details Invalidates I-Cache
  */
SECTION_CACHE_TEXT
void rtl_icache_invalidate(void)
{
	SCB_InvalidateICache();
}


/**
  \brief   Enable D-Cache
  \details Turns on D-Cache
  */
SECTION_CACHE_TEXT
void rtl_dcache_enable(void)
{
	SCB_EnableDCache();
}


/**
  \brief   Disable D-Cache
  \details Turns off D-Cache
  */
SECTION_CACHE_TEXT
void rtl_dcache_disable(void)
{
	SCB_DisableDCache();
}


/**
  \brief   Invalidate D-Cache
  \details Invalidates D-Cache
  */
SECTION_CACHE_TEXT
void rtl_dcache_invalidate(void)
{
	if (is_dcache_enabled()) {
		SCB_InvalidateDCache();
	}
}


/**
  \brief   Clean D-Cache
  \details Cleans D-Cache
  */
SECTION_CACHE_TEXT
void rtl_dcache_clean(void)
{
	if (is_dcache_enabled()) {
		SCB_CleanDCache();
	}
}


/**
  \brief   Clean & Invalidate D-Cache
  \details Cleans and Invalidates D-Cache
  */
SECTION_CACHE_TEXT
void rtl_dcache_clean_invalidate(void)
{
	if (is_dcache_enabled()) {
		SCB_CleanInvalidateDCache();
	}
}


/**
  \brief   D-Cache Invalidate by address
  \details Invalidates D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
SECTION_CACHE_TEXT
void rtl_dcache_invalidate_by_addr(uint32_t *addr, int32_t dsize)
{
	if (is_dcache_enabled()) {
		hal_rtl_interrupt_disable();
		SCB_InvalidateDCache_by_Addr(addr, dsize);
		hal_rtl_interrupt_enable();
	}
}


/**
  \brief   D-Cache Clean by address
  \details Cleans D-Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
SECTION_CACHE_TEXT
void rtl_dcache_clean_by_addr(uint32_t *addr, int32_t dsize)
{
	if (is_dcache_enabled()) {
		hal_rtl_interrupt_disable();
		SCB_CleanDCache_by_Addr(addr, dsize);
		hal_rtl_interrupt_enable();
	}
}


/**
  \brief   D-Cache Clean and Invalidate by address
  \details Cleans and invalidates D_Cache for the given address
  \param[in]   addr    address (aligned to 32-byte boundary)
  \param[in]   dsize   size of memory block (in number of bytes)
*/
SECTION_CACHE_TEXT
void rtl_dcache_clean_invalidate_by_addr(uint32_t *addr, int32_t dsize)
{
	if (is_dcache_enabled()) {
		hal_rtl_interrupt_disable();
		SCB_CleanInvalidateDCache_by_Addr(addr, dsize);
		hal_rtl_interrupt_enable();
	}
}

/** @} */ /* End of group hal_rtl_irq_cache */
/// @endcond

/** @} */ /* End of group rtl8735b_cache */
#endif // CONFIG_CACHE_EN
