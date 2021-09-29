/**************************************************************************//**
 * @file     rtl8195bhp_misc.c
 * @brief    The rtl8195bhp platform HAL misc.
 * @version  V1.00
 * @date     2016-06-07
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
#include "fw_img.h"
#include "memory.h"

#define SECTION_SYSMISC_TEXT                 SECTION(".sys_misc.text")
#define SECTION_SYSMISC_BSS                  SECTION(".sys_misc.bss")
#define SECTION_SYSMISC_STUBS                SECTION(".rom.hal_misc.stubs")

extern stdio_port_t _stdio_port;
extern void hard_fault_handler_c(uint32_t mstack[], uint32_t pstack[], uint32_t lr_value, uint32_t fault_id);

/**
 * @addtogroup hs_hal_misc Misc.
 * @{
 */

SECTION_SYSMISC_BSS phal_timer_adapter_t psystem_timer;
SECTION_SYSMISC_BSS hal_misc_adapter_t *pmisc_adapter;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_misc_rom_func HAL Misc. ROM APIs.
 * @ingroup hs_hal_misc
 * @{
 * @brief HAL miscellaneous ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of HAL Misc. APIs in the RAM space is provided for the user application.
 */

/**
  * @brief The stubs functions table to exports HAL Misc. ROM APIs.
  */
SECTION_SYSMISC_STUBS const hal_misc_func_stubs_t hal_misc_stubs = {
	.pstdio_port = &_stdio_port,

	.hal_misc_init = hal_rtl_misc_init,
	.hal_misc_wdt_set_timeout = hal_rtl_misc_wdt_set_timeout,
	.hal_misc_wdt_init = hal_rtl_misc_wdt_init,
	.hal_misc_wdt_reg_irq = hal_rtl_misc_wdt_reg_irq,
	.hal_misc_nmi_reg_irq = hal_rtl_misc_nmi_reg_irq,
	.hal_misc_cpu_rst = hal_rtl_misc_cpu_rst,
	.hard_fault_handler_c = hard_fault_handler_c
};

/**
 *  @brief Initial the HAL Misc. management adapter.
 *
 *  @param[in]  pmisc_adp  The Misc. entity.
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_init(hal_misc_adapter_t *pmisc_adp)
{
	_memset((void *)pmisc_adp, 0, sizeof(hal_misc_adapter_t));
	pmisc_adapter = pmisc_adp;
}


/**
 *  @brief Configures a hardware G-Timer as the free run system timer.
 *         The system timer is utilised to implement the time delay function.
 *
 *  @param[in]  ptimer_adp  The G-Timer adapter.
 *  @param[in]  tmr_id  The index of the G-Timer. The valid range of this value is 0 ~ 7.
 *  @param[in]  cnt_md  Set the counting mode of the system timer, up counting or down counting.
 *                        \arg \c GTimerCountUp  Up counting.
 *                        \arg \c GTimerCountDown  Down counting.
 *  @param[in]  tick_us  The tick time resolution, in micro-second, of the system timer.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_start_systimer(phal_timer_adapter_t ptimer_adp, uint32_t tmr_id,
								 timer_cnt_mode_t cnt_md, uint32_t tick_us, uint8_t tg)
{
	hal_rtl_timer_init_free_run(ptimer_adp, tmr_id, cnt_md, tick_us, tg);
	psystem_timer = ptimer_adp;
}

/**
 *  @brief Reads the tick count of the system timer. It start the counting
 *         when the system timer is started.
 *
 *  @returns    Current tick count of system time. The uint in us(micro-second).
 */
SECTION_SYSMISC_TEXT
u64 hal_rtl_misc_read_systime(void)
{
	u64 count;
	u64 ns2us;

	count = ((u64)psystem_timer->overflow_fired << 32) + (u64)hal_rtl_timer_indir_read(psystem_timer);
	ns2us = (count * (u64)psystem_timer->tick_r_ns) / (u64)1000;

	return ((u64)count * (u64)psystem_timer->tick_us + ns2us);
}

/**
 *  @brief Reads the count of the system timer. It return the counter register value directly.
 *
 *  @returns    Current tick count of the system time, in us. This count value will be reset to 0
 *              when ever the counter register is overflowed.
 */
SECTION_SYSMISC_TEXT
u32 hal_rtl_misc_read_curtime(void)
{
	u32 count;
	u64 ns2us;

	count = hal_rtl_timer_indir_read(psystem_timer);
	ns2us = (u64)((u64)count * (u64)psystem_timer->tick_r_ns) / (u64)1000;
	return (u32)(((u64)count * (u64)psystem_timer->tick_us) + ns2us);
}

/**
 *  @brief To check if current time is latter than the given expire time.
 *         The expired time is calculated by a start time plus a timeout period.
 *
 *  @param[in]  start_us  The start time of the timeout period, in us.
 *  @param[in]  timeout_us The timeout period, in us. The expire time is start_us + timeout_us.
 *
 *  @returns    Current time is expired or not.
 *                  - true: It is expired.
 *                  - false: It's not expire yet.
 */
SECTION_SYSMISC_TEXT
BOOLEAN hal_rtl_misc_is_timeout(u32 start_us, u32 timeout_us)
{
	u32 past_time;
	u32 cur_time;

	cur_time = hal_rtl_misc_read_curtime();
	if (cur_time >= start_us) {
		past_time = cur_time - start_us;
	} else {
		// timer wrapped
		past_time = 0xFFFFFFFF - start_us + cur_time;
	}

	if (timeout_us > past_time) {
		return FALSE;
	} else {
		return TRUE;
	}
}

/**
 *  @brief Makes a delay.
 *
 *  @param[in]  time_us  The delay period in micro-second.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_delay_us(u32 time_us)
{
	u32 start_time;

	start_time = hal_rtl_misc_read_curtime();
	while (hal_rtl_misc_is_timeout(start_time, time_us) == FALSE);
}

/**
 *  @brief Changes the watch dog timer timeout period.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_wdt_set_timeout(uint32_t time_us)
{
	VDR_Type *vendor = VDR;
	uint32_t div = 1;
	uint32_t ticks;
	uint32_t cnt_limit;
	uint32_t value_found;
	//uint32_t div_max;

	value_found = 0;
	for (cnt_limit = 1; cnt_limit < 12; cnt_limit++) {
		ticks = (2 << cnt_limit) - 1;
		// let max divider=3200 -> min resolution = 100 ms
		// CLK=32000HZ & divider=3200 -> 1 tick of count = 100000us
		if ((ticks * 100000) < time_us) {
			continue;
		}

#if 0
		// << 5 = x32, 1 ticks ~ 32us (1/32K)
		for (div_max = 1; div_max <= 3200; div_max = (div_max << 1)) {
			if (((div_max * ticks) << 5) > time_us) {
				break;
			}
		}

		for (div = (div_max >> 1); div <= 3200; div++) {
			if (((div * ticks) << 5) > time_us) {
				value_found = 1;
				break;
			}
		}

		if (value_found) {
			div = div + (div / 42); // 1/42 ~= (32us - 31.25us)/32us, the tick time compensation for 31.25us vs 32us
			break;
		}
#else
//        div = ((time_us - 1) >> (cnt_limit + 1 + 5)) + 1;   // >> 5 = 1/32, 1 ticks ~ 32us (1/32K)
		div = (((time_us - 1) / ticks) >>  5) + 1; // >> 5 = 1/32, 1 ticks ~ 32us (1/32K)
		if (div <= 3200) {
			div = (div << 10) / 1000;   // div = div*1.024, compensation of tick time 32us vs 31.25us, 32 / 31.25 = 1.024
			value_found = 1;
			break;
		}
#endif
	}

	if (value_found) {
		vendor->watch_dog_timer_b.vndr_divfactor = div - 1;
		vendor->watch_dog_timer_b.cnt_limit = cnt_limit;
	} else {
		vendor->watch_dog_timer_b.vndr_divfactor = 32000;
		vendor->watch_dog_timer_b.cnt_limit = 11;
	}
}

/**
 *  @brief Initials the watch dog timer and setup the timeout period.
 *         The system will be reset by the watch dog timeout event by default.
 *
 *  @param[in]  time_us  The timeout period in micro-second.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_wdt_init(uint32_t time_us)
{
	VDR_Type *vendor = VDR;

	hal_rtl_misc_wdt_set_timeout(time_us);
	// default reset the system when WDT timeout
	vendor->watch_dog_timer_b.wdt_mode = WDT_Mode_Rst;
}

/**
 *  @brief Registers a handler for the watch dog timeout interrupt.
 *         The WDT timeout interrupt will trigger the NMI interrupt.
 *         However the NMI interrupt default is in secure region.
 *         To handle the NMI interrupt, we should make the AIRCR.BFHFNMINS = 1.
 *         We can do this by set SCB_AIRCR_BFHFNMINS_VAL = 1 at secure code build time.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_wdt_reg_irq(irq_handler_t handler, void *arg)
{
	if (pmisc_adapter == NULL) {
		DBG_MISC_ERR("Misc. Adp is NULL\r\n");
		return;
	}

	pmisc_adapter->wdt_handler = handler;
	pmisc_adapter->wdt_arg = arg;
	VDR->watch_dog_timer_b.wdt_mode = WDT_Mode_Int;
}

/**
 *  @brief Invokes a CPU reset. Compares to the system reset, it only reset the
 *         CPU part and the program will be restarted. All other peripheral keeps
 *         their state.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT __NO_RETURN
void hal_rtl_misc_cpu_rst(void)
{
	SCB_Type *scb = SCB;
	uint32_t aircr;

	aircr = scb->AIRCR;
	aircr &= 0x0000FFFF;
	aircr |= ((0x05FA << 16) | SCB_AIRCR_SYSRESETREQ_Msk);
	scb->AIRCR = aircr;
	while (1);
}

/**
 *  @brief Registers a handler function for the NMI interrupt which is
 *         not triggered by the watch dog timer timeout event.
 *         The NMI interrupt default is in secure region.
 *         To handle the NMI interrupt, we should make the AIRCR.BFHFNMINS = 1.
 *         We can do this by set SCB_AIRCR_BFHFNMINS_VAL = 1 at secure code build time.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
SECTION_SYSMISC_TEXT
void hal_rtl_misc_nmi_reg_irq(irq_handler_t handler, void *arg)
{
	if (pmisc_adapter == NULL) {
		DBG_MISC_ERR("Misc. Adp is NULL\r\n");
		return;
	}

	pmisc_adapter->nmi_handler = handler;
	pmisc_adapter->nmi_arg = arg;
}

/** @} */ /* End of group hs_hal_misc_rtl_func */
/// @endcond    /* End of cond DOXYGEN_rtl_HAL_API */

/** @} */ /* End of group hs_hal_misc */

