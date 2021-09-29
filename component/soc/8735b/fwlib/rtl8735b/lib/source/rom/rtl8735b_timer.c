/**************************************************************************//**
 * @file     rtl8195bhp_timer.c
 * @brief    This file implements the HW Timer HAL functions.
 *
 * @version  V1.00
 * @date     2020-11-12
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
#include "rtl8735b_tg_type.h"
#include "rtl8735b_tm_type.h"
//#include "hal_platform_rtl8735b.h"


#if CONFIG_GTIMER_EN

#define SECTION_GTIMER_TEXT           SECTION(".rom.hal_timer.text")
#define SECTION_GTIMET_DATA           SECTION(".rom.hal_timer.data")
#define SECTION_GTIMER_RODATA         SECTION(".rom.hal_timer.rodata")
#define SECTION_GTIMERBA_RODATA       SECTION(".rom.hal_timer_ba.rodata")
#define SECTION_GTIMER_BSS            SECTION(".rom.hal_timer.bss")
#define SECTION_GTIMER_STUBS          SECTION(".rom.hal_timer.stubs")

#define GTIMER_REG_SIZE               0x40


extern void *_memset(void *dst0, int Val, size_t length);

/**
 * @addtogroup hs_hal_timer TIMER
 * @{
 */

extern phal_timer_adapter_t psystem_timer;


#if !defined(CONFIG_BUILD_NONSECURE)
#undef TIMER_GROUP0_REG_BASE
#define TIMER_GROUP0_REG_BASE (TG0_S_BASE)
#undef TIMER_GROUP1_REG_BASE
#define TIMER_GROUP1_REG_BASE (TG1_S_BASE)
#undef TIMER_GROUP2_REG_BASE
#define TIMER_GROUP2_REG_BASE (TG2_S_BASE)
#undef TIMER_GROUP3_REG_BASE
#define TIMER_GROUP3_REG_BASE (TG3_S_BASE)
#else

#undef TIMER_GROUP0_REG_BASE
#define TIMER_GROUP0_REG_BASE (TG0_BASE)
#undef TIMER_GROUP1_REG_BASE
#define TIMER_GROUP1_REG_BASE (TG1_BASE)
#undef TIMER_GROUP2_REG_BASE
#define TIMER_GROUP2_REG_BASE (TG2_BASE)
#undef TIMER_GROUP3_REG_BASE
#define TIMER_GROUP3_REG_BASE (TG3_BASE)

#endif

#if !defined(CONFIG_BUILD_NONSECURE)
#undef TM0_REG_BASE
#define TM0_REG_BASE (TM0_S)
#undef TM1_REG_BASE
#define TM1_REG_BASE (TM1_S)
#undef TM2_REG_BASE
#define TM2_REG_BASE (TM2_S)
#undef TM3_REG_BASE
#define TM3_REG_BASE (TM3_S)
#undef TM4_REG_BASE
#define TM4_REG_BASE (TM4_S)
#undef TM5_REG_BASE
#define TM5_REG_BASE (TM5_S)
#undef TM6_REG_BASE
#define TM6_REG_BASE (TM6_S)
#undef TM7_REG_BASE
#define TM7_REG_BASE (TM7_S)
#undef TM8_REG_BASE
#define TM8_REG_BASE (TM8_S)
#undef TM9_REG_BASE
#define TM9_REG_BASE (TM9_S)
#undef TM10_REG_BASE
#define TM10_REG_BASE (TM10_S)
#undef TM11_REG_BASE
#define TM11_REG_BASE (TM11_S)
#undef TM12_REG_BASE
#define TM12_REG_BASE (TM12_S)
#undef TM13_REG_BASE
#define TM13_REG_BASE (TM13_S)
#undef TM14_REG_BASE
#define TM14_REG_BASE (TM14_S)
#undef TM15_REG_BASE
#define TM15_REG_BASE (TM15_S)
#undef TM16_REG_BASE
#define TM16_REG_BASE (TM16_S)
#undef TM17_REG_BASE
#define TM17_REG_BASE (TM17_S)


#else
#undef TM0_REG_BASE
#define TM0_REG_BASE (TM0)
#undef TM1_REG_BASE
#define TM1_REG_BASE (TM1)
#undef TM2_REG_BASE
#define TM2_REG_BASE (TM2)
#undef TM3_REG_BASE
#define TM3_REG_BASE (TM3)
#undef TM4_REG_BASE
#define TM4_REG_BASE (TM4)
#undef TM5_REG_BASE
#define TM5_REG_BASE (TM5)
#undef TM6_REG_BASE
#define TM6_REG_BASE (TM6)
#undef TM7_REG_BASE
#define TM7_REG_BASE (TM7)
#undef TM8_REG_BASE
#define TM8_REG_BASE (TM8)
#undef TM9_REG_BASE
#define TM9_REG_BASE (TM9)
#undef TM10_REG_BASE
#define TM10_REG_BASE (TM10)
#undef TM11_REG_BASE
#define TM11_REG_BASE (TM11)
#undef TM12_REG_BASE
#define TM12_REG_BASE (TM12)
#undef TM13_REG_BASE
#define TM13_REG_BASE (TM13)
#undef TM14_REG_BASE
#define TM14_REG_BASE (TM14)
#undef TM15_REG_BASE
#define TM15_REG_BASE (TM15)
#undef TM16_REG_BASE
#define TM16_REG_BASE (TM16)
#undef TM17_REG_BASE
#define TM17_REG_BASE (TM17)


#endif



/**
    - timer group 0
    00:32KHz(S1)
    01:IRC_4MHz(ANA_CLK)
    10:PERI_PLL_40MHz
    - Other timer group
    00: 32kHz
    01: 4MHz
    10: 31.25MHz
 */
SECTION_GTIMER_RODATA
const u32 gtimer_sclk_freq[4] = {
	(u32)32000,
	(u32)4000000,
	(u32)40000000,
	(u32)31250000
};

/**
  * @brief minimum tick time(after pre-scaler) in ns for each SCLK.
  */
SECTION_GTIMER_RODATA
const u32 gtimer_min_tick[4] = {
	(u32)31250,
	(u32)250,
	(u32)25,
	(u32)32
};

/**
  * @brief maximum tick time(after pre-scaler) in ns for each SCLK.
  */
SECTION_GTIMER_RODATA
const u32 gtimer_max_tick[4] = {
	(u32)31250000,
	(u32)250000,
	(u32)25000,
	(u32)32000
};


/**
  * @brief The table of all Gtimer tick count.
  */
SECTION_GTIMER_RODATA
const u32 gtimer_sclk_tick_cnt_125ms[4] = {
	(2000000 / 8),
	(20000000 / 8),
	(32000 / 8),
	(32768 / 8)
};

/**
  * @brief The table of all TIMER ID.
  */
SECTION_GTIMER_RODATA
const timer_id_t pwm_avail_timer_list[] = {
	GTimer1,  GTimer2,	GTimer3,  GTimer4,
	GTimer5,   GTimer6,   GTimer7,	GTimer0,
	GTimer9,   GTimer10,  GTimer11, GTimer12,
	GTimer13,  GTimer14,  GTimer15,
	0xFF   /* end of list */
};



/**
  * @brief The table of all TIMER registers base address.
  */
//SECTION_GTIMERBA_RODATA
//const TM_TypeDef *timer_base_address[MaxGTimerNum] = {
//     TM0,  TM1,  TM2,  TM3,
//     TM4,  TM5,  TM6,  TM7,
//     TM9,  TM10, TM11, TM12,
//     TM13, TM14, TM15, TM16};


SECTION_GTIMERBA_RODATA
const TM_TypeDef *timer_base_address[MaxGTimerNum] = {
	TM0_REG_BASE,  TM1_REG_BASE,  TM2_REG_BASE,  TM3_REG_BASE,
	TM4_REG_BASE,  TM5_REG_BASE,  TM6_REG_BASE,  TM7_REG_BASE,
	TM9_REG_BASE,  TM10_REG_BASE, TM11_REG_BASE, TM12_REG_BASE,
	TM13_REG_BASE, TM14_REG_BASE, TM15_REG_BASE, TM16_REG_BASE
};





/**
  * @brief The global data structure to handle common resource
  *        for all TIMER group0 adapters.
  */
SECTION_GTIMER_BSS hal_timer_group_adapter_t *ptimer_group0;

/**
  * @brief The global data structure to handle common resource
  *        for all TIMER group1 adapters.
  */
SECTION_GTIMER_BSS hal_timer_group_adapter_t *ptimer_group1;

/**
  * @brief The global data structure to handle common resource
  *        for all TIMER group2 adapters.
  */
SECTION_GTIMER_BSS hal_timer_group_adapter_t *ptimer_group2;

/**
  * @brief The global data structure to handle common resource
  *        for all TIMER group3 adapters.
  */
SECTION_GTIMER_BSS hal_timer_group_adapter_t *ptimer_group3;



/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_timer_rom_func TIMER HAL ROM APIs.
 * @ingroup hs_hal_timer
 * @{
 * @brief The TIMER HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of TIMER HAL APIs in the RAM space is provided for the user application.
 */

/**
  \brief  The stubs functions table of the TIMER HAL functions in ROM.
*/
SECTION_GTIMER_STUBS const hal_timer_func_stubs_t hal_gtimer_stubs = {
	.pptimer_group0 = &ptimer_group0,
	.pptimer_group1 = &ptimer_group1,
	.pptimer_group2 = &ptimer_group2,
	.pptimer_group3 = &ptimer_group3,
	.hal_timer_convert_ticks_to_us = hal_rtl_timer_convert_ticks_to_us,
	.hal_timer_convert_us_to_ticks = hal_rtl_timer_convert_us_to_ticks,
	.hal_timer_convert_ticks_to_us64 = hal_rtl_timer_convert_ticks_to_us64,
	.hal_timer_convert_us_to_ticks64 = hal_rtl_timer_convert_us_to_ticks64,
	.hal_timer_irq_handler = hal_rtl_timer_irq_handler,
	.hal_timer_me_ctrl = hal_rtl_timer_me_ctrl,
	.hal_timer_set_me_counter = hal_rtl_timer_set_me_counter,
	.hal_timer_group_en_ctrl = hal_rtl_timer_group_en_ctrl,
	.hal_timer_group_pclk_ctrl = hal_rtl_timer_group_pclk_ctrl,
	.hal_timer_group_sclk_ctrl = hal_rtl_timer_group_sclk_ctrl,
	.hal_timer_group_intclk_sel = hal_rtl_timer_group_intclk_sel,
	.hal_timer_group_init = hal_rtl_timer_group_init,
	.hal_timer_group_deinit = hal_rtl_timer_group_deinit,
	.hal_timer_bare_init = hal_rtl_timer_bare_init,
	.hal_timer_deinit = hal_rtl_timer_deinit,
	.hal_timer_group_reg_irq = hal_rtl_timer_group_reg_irq,
	.hal_timer_reg_toirq = hal_rtl_timer_reg_toirq,
	.hal_timer_unreg_toirq = hal_rtl_timer_unreg_toirq,
	.hal_timer_reg_meirq = hal_rtl_timer_reg_meirq,
	.hal_timer_unreg_meirq = hal_rtl_timer_unreg_meirq,
	.hal_timer_set_tick_time = hal_rtl_timer_set_tick_time,
	.hal_timer_init_free_run = hal_rtl_timer_init_free_run,
	.hal_timer_indir_read = hal_rtl_timer_indir_read,
	.hal_timer_read_us = hal_rtl_timer_read_us,
	.hal_timer_read_us64 = hal_rtl_timer_read_us64,
	.hal_timer_init = hal_rtl_timer_init,
	.hal_timer_set_timeout = hal_rtl_timer_set_timeout,
	.hal_timer_start = hal_rtl_timer_start,
	.hal_timer_enable_match_event = hal_rtl_timer_enable_match_event,
	.hal_timer_start_one_shot = hal_rtl_timer_start_one_shot,
	.hal_timer_start_periodical = hal_rtl_timer_start_periodical,
	.hal_timer_allocate = hal_rtl_timer_allocate,
	.hal_timer_event_init = hal_rtl_timer_event_init,
	.hal_timer_event_deinit = hal_rtl_timer_event_deinit,
#if !defined(CONFIG_BUILD_NONSECURE)
	.hal_timer_clock_init = hal_rtl_timer_clock_init,
	.hal_timer_group_sclk_sel = hal_rtl_timer_group_sclk_sel,
#endif


	// hal_misc timer related API
	.ppsys_timer = &psystem_timer,
	.hal_read_systime = hal_rtl_misc_read_systime,
	.hal_read_curtime = hal_rtl_misc_read_curtime,
	.hal_start_systimer = hal_rtl_misc_start_systimer,
	.hal_delay_us = hal_rtl_misc_delay_us,
	.hal_is_timeout = hal_rtl_misc_is_timeout
};

/**
 *  @brief To set timer number according to timer group ID
 *
 *  @param[in] tg_id The ID of timer group.
 *
 *  @returns The timer number in the timer group.
 */
SECTION_GTIMER_TEXT
u8 hal_rtl_timer_num_select(u8 tg_id)
{
	switch (tg_id) {
	case 0:
		return 8;
		break;
	case 1:
		return 1;
		break;
	case 2:
		return 8;
		break;
	case 3:
		return 1;
		break;
	}
	return HAL_OK;
}


/**
 *  @brief To convert a tick count to the time period in us
 *
 *  @param[in] ticks The number of ticks.
 *  @param[in] sclk_idx The timer SCLK selection.
 *
 *  @returns The converted time period in us.
 */
SECTION_GTIMER_TEXT
u32 hal_rtl_timer_convert_ticks_to_us(u32 ticks, u8 sclk_idx)
{
	u32 remain_count;
	u32 ms125;
	u32 tick_cnt_125ms;

	tick_cnt_125ms = gtimer_sclk_tick_cnt_125ms[sclk_idx];
	if (ticks >= tick_cnt_125ms) {
		ms125 = ticks / tick_cnt_125ms;
		remain_count = ticks - (ms125 * tick_cnt_125ms);
		return (125000 * ms125) + (remain_count * 1000000 / gtimer_sclk_freq[sclk_idx]);
	} else {
		return (ticks * 1000000 / gtimer_sclk_freq[sclk_idx]);
	}
}

/**
 *  @brief To convert a time period in us to number of 32K ticks
 *
 *  @param[in] time_us The timer period in us.
 *  @param[in] sclk_idx The timer SCLK selection.
 *
 *  @returns The converted 32K ticks.
 */
SECTION_GTIMER_TEXT
u32 hal_rtl_timer_convert_us_to_ticks(u32 time_us, u8 sclk_idx)
{
	u32 ms125;
	u32 remain_time;
	u32 ticks;
	u32 sclk_freq;

	sclk_freq = gtimer_sclk_freq[sclk_idx];
	if (time_us >= 125000) {
		ms125 = time_us / 125000;
		remain_time = time_us - (ms125 * 125000);
		ticks = (ms125 * gtimer_sclk_tick_cnt_125ms[sclk_idx]) + (remain_time * sclk_freq / 1000000);
	} else {
		remain_time = time_us;
		ticks = time_us * sclk_freq / 1000000;
	}

	if (((remain_time * sclk_freq) % 1000000) >= 500000) {
		ticks++;
	}

	if ((ticks == 0) && (time_us > 0)) {
		// at least 1 tick
		ticks = 1;
	}
	return ticks;
}

/**
 *  @brief To convert a tick count to the time period in us
 *
 *  @param[in] ticks The number of ticks.
 *  @param[in] sclk_idx The timer SCLK selection.
 *
 *  @returns The converted time period in us.
 */
SECTION_GTIMER_TEXT
u64 hal_rtl_timer_convert_ticks_to_us64(u64 ticks, u8 sclk_idx)
{
	u64 remain_count;
	u64 ms125;
	u64 tick_cnt_125ms;

	tick_cnt_125ms = (u64)gtimer_sclk_tick_cnt_125ms[sclk_idx];

	if (ticks >= tick_cnt_125ms) {
		ms125 = ticks / tick_cnt_125ms;
		remain_count = ticks - (ms125 * tick_cnt_125ms);
		return ((u64)125000 * ms125) + (remain_count * (u64)1000000 / gtimer_sclk_freq[sclk_idx]);
	} else {
		return ((ticks * (u64)1000000) / (u64)gtimer_sclk_freq[sclk_idx]);
	}
}

/**
 *  @brief To convert a time period in us to number of 32K ticks
 *
 *  @param[in] time_us The timer period in us.
 *  @param[in] sclk_idx The timer SCLK selection.
 *
 *  @returns The converted 32K ticks.
 */
SECTION_GTIMER_TEXT
u64 hal_rtl_timer_convert_us_to_ticks64(u64 time_us, u8 sclk_idx)
{
	u64 ms125;
	u64 remain_time;
	u64 ticks;
	u64 sclk_freq;

	sclk_freq = (u64)gtimer_sclk_freq[sclk_idx];
	if (time_us >= (u64)125000) {
		ms125 = time_us / (u64)125000;
		remain_time = time_us - (ms125 * (u64)125000);
		ticks = (ms125 * (u64)gtimer_sclk_tick_cnt_125ms[sclk_idx]) + (remain_time * sclk_freq / (u64)1000000);
	} else {
		remain_time = time_us;
		ticks = time_us * sclk_freq / (u64)1000000;
	}

	if (((remain_time * sclk_freq) % (u64)1000000) >= 500000) {
		ticks++;
	}

	if ((ticks == 0) && (time_us > 0)) {
		// at least 1 tick
		ticks = 1;
	}
	return ticks;
}

/**
 *  @brief To handle the timer counter overfloe/underflow interrupt.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_timer_free_run_overflow_handler_rtl8195bhp(phal_timer_adapter_t ptimer_adp)
{
	ptimer_adp->overflow_fired++;
}

/**
 *  @brief The Common IRQ handler of G-Timer group.
 *
 *  @param[in] hid.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_irq_handler(void *hid)
{
	hal_timer_group_adapter_t *ptg = (hal_timer_group_adapter_t *)hid;
	phal_timer_adapter_t ptimer_adp;
	u8 tg_int_sts;
	u8 tmr_num;
	//tm_isr_t tmr_int_sts;
	u32 i;
	u32 j;

	if (NULL == ptg) {
		return;
	}

	tmr_num = hal_rtl_timer_num_select(ptg->tgid);
	//dbg_printf("tg %x\r\n", ptg->tgid);

	tg_int_sts = hal_rtl_timer_group_get_ists(ptg);
	//dbg_printf("isr1 %x\r\n", tg_int_sts);
	for (i = 0; i < tmr_num; i++) {
		if (tg_int_sts & (1 << i)) {
			ptimer_adp = ptg->timer_adapter[i];
			if (ptimer_adp == NULL) {
				// no adapter for this G-Timer

				TM_TypeDef *PTMR;
				u32 ISR;
				PTMR = (TM_TypeDef *)((uint32_t)(ptg->tg_ba) + ((i + 1) * GTIMER_REG_SIZE));
				ISR = PTMR->TM_ISR;
				ISR &= 0xFF;
				PTMR->TM_ISR = ISR;

				continue;
			}

			u32 ISR;
			TM_TypeDef *TM_OBJ = (TM_TypeDef *)(ptimer_adp->tmr_ba);
			ISR = TM_OBJ->TM_ISR;
			TM_OBJ->TM_ISR |= ISR;

			tg_int_sts = hal_rtl_timer_group_get_ists(ptg);

			// Timeout interrupt
			if ((ISR & TM_BIT_TIMEOUT) == 0x1) {
				if (ptimer_adp->reload_mode != GTimerPeriodical) {
					hal_rtl_timer_disable(ptimer_adp);
				}

//                ptimer_adp->tmr_ba->isr_b.timeout = 1;  // write 1 to clear ISR
				if (ptimer_adp->timeout_callback != NULL) {
					ptimer_adp->timeout_callback(ptimer_adp->to_cb_para);
				}
				ISR = ISR & (~TM_BIT_TIMEOUT);
				if (ISR == 0) {
					continue;
				}
			}

			// Check Timer counter match event 0 ~ 3 interrupt
			for (j = 0; j < 4; j++) {
				if (ISR & (1 << (j + 1))) {
					if (ptimer_adp->me_callback[j] != NULL) {
						ptimer_adp->me_callback[j](ptimer_adp->me_cb_para[j]);
					}
					ISR &= ~(1 << (j + 1));
					if (ISR == 0) {
						break; // break for loop
					}
				}
			}
		}
	}

	__DSB();
	__ISB();
}

/**
 *  @brief The IRQ handler of G-Timer group 0.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void TimerGroup0_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(TimerGroup0_IRQn);
	hal_rtl_timer_irq_handler(ptimer_group0);
}

/**
 *  @brief The IRQ handler of G-Timer group 1.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void TimerGroup1_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(TimerGroup1_IRQn);
	hal_rtl_timer_irq_handler(ptimer_group1);
}

/**
 *  @brief The IRQ handler of G-Timer group 1.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void TimerGroup2_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(TimerGroup2_IRQn);
	hal_rtl_timer_irq_handler(ptimer_group2);
}

/**
 *  @brief The IRQ handler of G-Timer group 1.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void TimerGroup3_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(TimerGroup3_IRQn);
	hal_rtl_timer_irq_handler(ptimer_group3);
}



/**
 *  @brief To enable/disbale the timer counter match event.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The match event number(0 ~ 3).
 *  @param[in] enable Enable control. (1: enable, 0: disable)
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_me_ctrl(phal_timer_adapter_t ptimer_adp, timer_match_event_t match_ev, u8 enable)
{
	if (match_ev >= MaxGTimerMatchEvent) {
		DBG_TIMER_ERR("hal_rtl_timer_me_ctrl: Invalid ME number (%u)\r\n", match_ev);
	}

	TM_TypeDef *TM_OBJ = (TM_TypeDef *)(ptimer_adp->tmr_ba);
	u32 MecTrl;

	MecTrl = TM_OBJ->TM_MECTRL;

	if (enable) {
		MecTrl |= 1 << match_ev;
	} else {
		MecTrl &= ~(1 << match_ev);
	}

	TM_OBJ->TM_MECTRL = MecTrl;

}

/**
 *  @brief To set the counter number of a counter match event.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The match event number(0 ~ 3).
 *  @param[in] counter The counter value for the timer match filer to match to.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_set_me_counter(phal_timer_adapter_t ptimer_adp, timer_match_event_t match_ev, u32 counter)
{
	TM_TypeDef *TM_OBJ = (TM_TypeDef *)(ptimer_adp->tmr_ba);
	//u32 MecTrl;

	switch (match_ev) {
	case GTimerMatchEvent0:
		TM_OBJ->TM_ME0 = TM_MASK_ME0 & counter;
		break;

	case GTimerMatchEvent1:
		TM_OBJ->TM_ME1 = TM_MASK_ME1 & counter;
		break;

	case GTimerMatchEvent2:
		TM_OBJ->TM_ME2 = TM_MASK_ME2 & counter;
		break;

	case GTimerMatchEvent3:
		TM_OBJ->TM_ME3 = TM_MASK_ME3 & counter;
		break;

	default:
		DBG_TIMER_ERR("hal_rtl_timer_set_me_counter: Invalid ME number (%u)\r\n", match_ev);
	}
}

#if 1

/**
 *  @brief To enable or disable a timer group block.
 *
 *  @param[in] tgid The timer group ID(index).
 *  @param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_group_en_ctrl(uint32_t tgid, BOOL en)
{

//    switch (tgid)
//        {
//        case 0:
//            HAL_WRITE32(0x40009800, 0x24, 0x2); // Enable timer0 block
//            break;
//        case 1:
//            HAL_WRITE32(0x50008000, 0x104, 0x2); // Enable timer2 block
//            break;
//        case 2:
//            HAL_WRITE32(0x40009800, 0x24, 0x2); // Enable timer0 block
//            break;
//        case 3:
//            HAL_WRITE32(0x40009800, 0x24, 0x2); // Enable timer0 block
//            break;
//        }
}

/**
 *  @brief To enable/disable the APB clock for timer group block.
 *
 *  @param[in] tgid The timer group ID(index).
 *  @param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_group_pclk_ctrl(uint32_t tgid, BOOL en)
{
//    if (!tgid) {
//        SYSON->hs_timer_ctrl_b.hs_tg0_pclk_en = en;
//    } else {
//        SYSON->hs_timer_ctrl_b.hs_tg1_pclk_en = en;
//    }
}

/**
 *  @brief To enable/disable the system clock for timer group block.
 *
 *  @param[in] tgid The timer group ID(index).
 *  @param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_group_sclk_ctrl(uint32_t tgid, BOOL en)
{
//    if (!tgid) {
//        SYSON->hs_timer_ctrl_b.hs_tg0_sclk_en = en;
//    } else {
//        SYSON->hs_timer_ctrl_b.hs_tg1_sclk_en = en;
//    }
}

/**
 *  @brief To select the clock source for timer group block interrupt.
 *         When in sleep mode, just can use the SCLK for the interrupr circute.
 *
 *  @param[in] tgid The timer group ID(index).
 *  @param[in] clk_sel  The clock source selection: 0: APB clock, 1: SCLK.
 *
 *  @returns void
 */

SECTION_GTIMER_TEXT
void hal_rtl_timer_group_intclk_sel(uint32_t tgid, timer_interrupt_clk_t clk_sel)
{

#if !defined(CONFIG_BUILD_NONSECURE)

//    #define PON     0x40009800
//    #define S_SYSON 0x50000800

	u32 val;
	if (tgid == 0) {
		val = HAL_READ32(0x40009800, 0x2C);
		if (clk_sel == 0) {
			val &= ~(PON_BIT_INTR_CLK_TIMER0_SEL);
		} else {
			val |= (PON_BIT_INTR_CLK_TIMER0_SEL);
		}
		HAL_WRITE32(0x40009800, 0x2C, val);
	} else if (tgid == 1) {
		val = HAL_READ32(0x50000800, 0x104);
		if (clk_sel == 0) {
			val &= ~(SYSON_S_BIT_SYS_TG1_INTR_CLK_SEL);
		} else {
			val |= (SYSON_S_BIT_SYS_TG1_INTR_CLK_SEL);
		}
		HAL_WRITE32(0x50000800, 0x104, val);
	} else if (tgid == 2) {
		val = HAL_READ32(0x50000800, 0x104);
		if (clk_sel == 0) {
			val &= ~(SYSON_S_BIT_SYS_TG2_INTR_CLK_SEL);
		} else {
			val |= (SYSON_S_BIT_SYS_TG2_INTR_CLK_SEL);
		}
		HAL_WRITE32(0x50000800, 0x104, val);
	} else if (tgid == 3) {
		val = HAL_READ32(0x50000800, 0x104);
		if (clk_sel == 0) {
			val &= ~(SYSON_S_BIT_SYS_TG3_INTR_CLK_SEL);
		} else {
			val |= (SYSON_S_BIT_SYS_TG3_INTR_CLK_SEL);
		}
		HAL_WRITE32(0x50000800, 0x104, val);
	}

#endif
}



/**
 *  @brief To select the source clock of a timer group.
 *
 *  @param[in] ptg_adp The timer group adapter.
 *  @param[in] sclk_freq The source clock freq.
 *
 *  @returns void
 */

#endif

#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_GTIMER_TEXT
hal_status_t hal_rtl_timer_group_sclk_sel(hal_timer_group_adapter_t *ptg_adp, uint32_t sclk_freq)
{
	//uint8_t i;
	//uint32_t sclk_idx;
	//u32 val;

	if (sclk_freq > 4) {
		return HAL_ERR_PARA;
	} else {
		ptg_adp->sclk_idx = sclk_freq;
		if (sclk_freq == GTimerSClk_MAXID) {
			ptg_adp->sclk_idx = GTimerSClk_4M; // use default 4M
		}
	}

	if (ptg_adp->tgid == 0) {
		hal_rtl_sys_set_clk(TIMER0_SYS, ptg_adp->sclk_idx);
	} else if (ptg_adp->tgid == 1) {
		hal_rtl_sys_set_clk(TIMER1_SYS, ptg_adp->sclk_idx);
	} else if (ptg_adp->tgid == 2) {
		hal_rtl_sys_set_clk(TIMER2_SYS, ptg_adp->sclk_idx);
	} else if (ptg_adp->tgid == 3) {
		hal_rtl_sys_set_clk(TIMER3_SYS, ptg_adp->sclk_idx);
	}


	return HAL_OK;

}
#endif


#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_GTIMER_TEXT
void hal_rtl_timer_clock_init(uint32_t tgid, int en)
{
	if (tgid == 0) {
		hal_rtl_sys_peripheral_en(TIMER0_SYS, en);   // Enable TIMER clock and power on PON
	} else if (tgid == 2) {
		hal_rtl_sys_peripheral_en(TIMER2_SYS, en);   // Enable TIMER clock and power on SYSON_S
	} else if (tgid == 1) {
		hal_rtl_sys_peripheral_en(TIMER1_SYS, en);   // Enable TIMER clock and power on SYSON_S
	} else if (tgid == 3) {
		hal_rtl_sys_peripheral_en(TIMER3_SYS, en);   // Enable TIMER clock and power on SYSON_S
	}
}
#endif

/**
 *  @brief To initial a HW timer group adapter.
 *
 *  @param[in] ptg_adp The timer group adapter.
 *  @param[in] tgid The timer group ID(index).
 *
 *  @returns void
 */
#define sys_ok 0

SECTION_GTIMER_TEXT
void hal_rtl_timer_group_init(hal_timer_group_adapter_t *ptg_adp, uint32_t tgid)
{
	//uint8_t GTIMER_GROUP_TIMER_NUM;
	u8 tmr_num;
	uint32_t i;
	//volatile uint32_t ists;
	//TM_TypeDef *ptmr;
	//u32 val;

//#if !defined(CONFIG_BUILD_NONSECURE)
//
//    #define APB_clk 0
//    #define PON     0x40009800
//    #define S_SYSON 0x50000800
//
//    hal_rtl_timer_clock_init(tgid);
//#endif

	_memset((void *)ptg_adp, 0, sizeof(hal_timer_group_adapter_t));
	switch (tgid) {
	case 0:
		ptimer_group0 = ptg_adp;
		ptg_adp->tg_ba = (TG_TypeDef *)TIMER_GROUP0_REG_BASE; // TG0
		ptg_adp->tgid = 0;
		break;

	case 1:
		ptimer_group1 = ptg_adp;
		ptg_adp->tg_ba = (TG_TypeDef *)TIMER_GROUP1_REG_BASE; // TG1
		ptg_adp->tgid = 1;
		break;

	case 2:
		ptimer_group2 = ptg_adp;
		ptg_adp->tg_ba = (TG_TypeDef *)TIMER_GROUP2_REG_BASE; // TG2
		ptg_adp->tgid = 2;
		break;

	case 3:
		ptimer_group3 = ptg_adp;
		ptg_adp->tg_ba = (TG_TypeDef *)TIMER_GROUP3_REG_BASE; // TG3
		ptg_adp->tgid = 3;
		break;
	}

	TG_TypeDef *TG_OBJ = (TG_TypeDef *)(ptg_adp->tg_ba);
	TM_TypeDef *PTMR = (TM_TypeDef *)(ptg_adp->tg_ba);
	volatile u32 ISTS;
	volatile u32 ISR;

	ISTS = TG_OBJ->TG_ISTS;
	tmr_num = hal_rtl_timer_num_select(tgid);

	if (ISTS != 0) {
		for (i = 0; i < tmr_num; i++) {
			if ((ISTS & (1 << i)) != 0) {
				PTMR = (TM_TypeDef *)timer_base_address[i + (tmr_num * (tgid & 0x01))];
				ISR = PTMR->TM_ISR;
				ISR &= 0xFF;
				PTMR->TM_ISR = ISR;
			}
		}
	}

	if (tgid == 0) {
		hal_rtl_timer_group_reg_irq(ptg_adp, (irq_handler_t)TimerGroup0_IRQHandler);
		hal_rtl_irq_set_priority(TimerGroup0_IRQn, TimerGroup0_IRQPri);
	} else if (tgid == 1) {
		hal_rtl_timer_group_reg_irq(ptg_adp, (irq_handler_t)TimerGroup1_IRQHandler);
		hal_rtl_irq_set_priority(TimerGroup1_IRQn, TimerGroup1_IRQPri);
	} else if (tgid == 2) {
		hal_rtl_timer_group_reg_irq(ptg_adp, (irq_handler_t)TimerGroup2_IRQHandler);
		hal_rtl_irq_set_priority(TimerGroup2_IRQn, TimerGroup2_IRQPri);
	} else if (tgid == 3) {
		hal_rtl_timer_group_reg_irq(ptg_adp, (irq_handler_t)TimerGroup3_IRQHandler);
		hal_rtl_irq_set_priority(TimerGroup3_IRQn, TimerGroup3_IRQPri);
	}
}

/**
 *  @brief To de-initial(disable) a HW timer group adapter.
 *
 *  @param[in] ptg_adp The timer group adapter.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_group_deinit(hal_timer_group_adapter_t *ptg_adp)
{
	uint32_t tgid = ptg_adp->tgid;

//    hal_rtl_timer_group_en_ctrl (tgid, GTimer_Disable);
//    hal_rtl_timer_group_pclk_ctrl (tgid, GTimer_Disable);
//    hal_rtl_timer_group_sclk_ctrl (tgid, GTimer_Disable);

	if (tgid == 0) {
		hal_rtl_irq_disable(TimerGroup0_IRQn);
		__ISB();
		ptimer_group0 = NULL;
	} else if (tgid == 1) {
		hal_rtl_irq_disable(TimerGroup1_IRQn);
		__ISB();
		ptimer_group1 = NULL;
	} else if (tgid == 2) {
		hal_rtl_irq_disable(TimerGroup2_IRQn);
		__ISB();
		ptimer_group2 = NULL;
	} else if (tgid == 3) {
		hal_rtl_irq_disable(TimerGroup3_IRQn);
		__ISB();
		ptimer_group3 = NULL;
	}
}

/**
 *  @brief To initial a HW timer adapter.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] tid The timer ID(index).
 *
 *  @returns     HAL_OK:  Setting succeed.
 *  @returns     HAL_ERR_PARA:  Input arguments are invalid.
 *  @returns     HAL_BUSY:  The Timer setting was not ready.
 */
SECTION_GTIMER_TEXT
hal_status_t hal_rtl_timer_bare_init(phal_timer_adapter_t ptimer_adp, timer_id_t tid, u8 tg)
{
	TM_TypeDef *tmr_ba;
	hal_timer_group_adapter_t *ptg_adp;
	u32 tg_tid = 0;
	//u8 tmr_num;

	if (tg == 0 || tg == 2) {

		if (tid < GTIMER_GROUP_TIMER_NUM) {
			ptg_adp = ptimer_group0;
			//dbg_printf("pg0\r\n");
		} else {
			ptg_adp = ptimer_group2;
			//dbg_printf("pg2\r\n");
		}

		if ((ptimer_adp == NULL) || (tid >= MaxGTimerNum) || (ptg_adp == NULL)) {
			DBG_TIMER_ERR("hal_timer_bare: tg_adp=0x%x, ptimer_adp=0x%x, tid=%u\r\n", ptg_adp, ptimer_adp, tid);
			return HAL_ERR_PARA;
		}

		tg_tid = tid % GTIMER_GROUP_TIMER_NUM;
		//dbg_printf("tid tg_tid %x %x\r\n", tid, tg_tid);
		tmr_ba = (TM_TypeDef *)timer_base_address[tid];

		if (ptg_adp->tmr_in_use & (1 << tg_tid)) {
			return HAL_BUSY;
		}
	} else if (tg == 1) {
		ptg_adp = ptimer_group1;
		tg_tid = 0;
		tmr_ba = (TM_TypeDef *)TM8_REG_BASE;  // TM8
	} else if (tg == 3) {
		ptg_adp = ptimer_group3;
		tg_tid = 0;
		tmr_ba = (TM_TypeDef *)TM17_REG_BASE; // TM17
	} else {
		return HAL_ERR_PARA;
	}

	ptimer_adp->tid = tid;
	ptg_adp->timer_adapter[tg_tid] = ptimer_adp;
	ptg_adp->tmr_in_use |= (1 << tg_tid);
	//tmr_ba = (TM_Type *)timer_base_address[tid];
	ptimer_adp->tmr_ba = tmr_ba;
	tmr_ba->TM_CTRL = 0;
	//tmr_ba->ctrl = 0; // default setting: disable, timer mode, unmask interrupt, up counter
	ptimer_adp->tg_ba = ptg_adp->tg_ba;
	ptimer_adp->ptg_adp = ptg_adp;

	return HAL_OK;
}

/**
 *  @brief To de-initial a HW timer adapter.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_deinit(phal_timer_adapter_t ptimer_adp)
{
	hal_timer_group_adapter_t *ptg_adp;
	TM_TypeDef *TM_OBJ = (TM_TypeDef *)(ptimer_adp->tmr_ba);
	u8 tid;
	u8 tg_tid;

	tid = ptimer_adp->tid;
	tg_tid = tid % GTIMER_GROUP_TIMER_NUM;

	TM_OBJ->TM_CTRL = TM_OBJ->TM_CTRL & (~TM_BIT_EN);
	ptg_adp = ptimer_adp->ptg_adp;
	ptg_adp->timer_adapter[tg_tid] = NULL;
	ptg_adp->tmr_in_use &= ~(1 << tg_tid);



#if 0
	// if no more timer is using this IRQ then un-register the IRQ
	for (i = 0; i < GTIMER_GROUP_TIMER_NUM; i++) {
		if (ptg_adp->timer_adapter[i] != NULL) {
			if (ptg_adp->timer_adapter[i]->timeout_callback != NULL) {
				break;
			}
		}
	}

	if (i >= GTIMER_GROUP_TIMER_NUM) {
		// No active timer, disable the G-Timer interrupt
		if (tid == 0) {
			hal_irq_disable_rtl8195bhp(TimerGroup0_IRQn);
		} else {
			hal_irq_disable_rtl8195bhp(TimerGroup1_IRQn);
		}
	}
#endif
}

/**
 *  @brief To register a IRQ handler for a timer group.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] irq_handler The call back function of the IRQ.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_group_reg_irq(hal_timer_group_adapter_t *ptg_adp, irq_handler_t irq_handler)
{
	int32_t irqn = 1;

#if 0
	if (ptg_adp->tgid == 0) {
		irqn = TimerGroup0_IRQn;
	} else {
		irqn = TimerGroup1_IRQn;
	}
#endif

	switch (ptg_adp->tgid) {
	case 0:
		irqn = TimerGroup0_IRQn;
		break;
	case 1:
		irqn = TimerGroup1_IRQn;
		break;
	case 2:
		irqn = TimerGroup2_IRQn;
		break;
	case 3:
		irqn = TimerGroup3_IRQn;
		break;
	}

	// IRQ vector may has been registered, disable and re-register it
	hal_rtl_irq_disable(irqn);
	//hal_irq_disable_rtl8195bhp (irqn);
	__ISB();
	hal_rtl_irq_set_vector(irqn, (uint32_t)irq_handler);
	hal_rtl_irq_enable(irqn);
}

/**
 *  @brief To register a timer timeout IRQ call back function
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] callback The call back function.
 *  @param[in] phid The argument for call back function calling.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_reg_toirq(phal_timer_adapter_t ptimer_adp, timer_callback_t callback, void *phid)
{
	ptimer_adp->timeout_callback = callback;
	ptimer_adp->to_cb_para = phid;
}

/**
 *  @brief To un-register a timer timeout IRQ call back function
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_unreg_toirq(phal_timer_adapter_t ptimer_adp)
{
	ptimer_adp->timeout_callback = (timer_callback_t)NULL;
	ptimer_adp->to_cb_para = (void *)NULL;
}

/**
 *  @brief To register a timer counter matched IRQ call back function
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The timer counter match event number (0 ~ 3).
 *  @param[in] callback The call back function.
 *  @param[in] phid The argument for call back function calling.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_reg_meirq(phal_timer_adapter_t ptimer_adp, u8 match_ev,
							 timer_callback_t callback, void *phid)
{
	if (match_ev >= 4) {
		DBG_TIMER_ERR("hal_rtl_timer_reg_meirq: Invalid ME number (%u)\r\n", match_ev);
		return;
	}
	ptimer_adp->me_callback[match_ev] = callback;
	ptimer_adp->me_cb_para[match_ev] = phid;
}

/**
 *  @brief To un-register a timer counter matched IRQ call back function
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The timer counter match event number (0 ~ 3).
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_unreg_meirq(phal_timer_adapter_t ptimer_adp, u8 match_ev)
{
	if (match_ev >= 4) {
		DBG_TIMER_ERR("hal_timer_unreg_meirq: Invalid ME number (%u)\r\n", match_ev);
		return;
	}

	ptimer_adp->me_callback[match_ev] = (timer_callback_t)NULL;
	ptimer_adp->me_cb_para[match_ev] = (void *)NULL;
}

/**
 *  @brief To configure the tick period of a timer by set the pre-scaler value.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] tick_ns The period (in ns) of a tick
 *
 *  @returns The real tick time, in ns.
 */
SECTION_GTIMER_TEXT
u32 hal_rtl_timer_set_tick_time(phal_timer_adapter_t ptimer_adp, u32 tick_ns)
{
	u32 scaler;
	u8 sclk_idx;
	u64 tick_ns_tmp;

	sclk_idx = ptimer_adp->ptg_adp->sclk_idx;
	tick_ns_tmp = (u64)tick_ns * (u64)gtimer_sclk_freq[sclk_idx];
//    tick_ns_tmp = div_u64 (((u64)tick_ns * (u64)gtimer_sclk_freq[sclk_idx] ), 1000000000);
	// x / N = (x*A)/2^(32+n)  -->  A = 2^(32+n)/N --> N = 1000000000, n=16 --> A = 281475
	tick_ns_tmp = (tick_ns_tmp * (u64)281475) >> 48;  // * 10^-9
	scaler = (u32)tick_ns_tmp;
	if (scaler > 1024) {
		DBG_TIMER_INFO("hal_rtl_timer_set_tick_time: invalid pre-scaler %lu\r\n", scaler);
		scaler = 1024;
	}

	if (scaler > 0) {
		scaler -= 1;
	}
	hal_rtl_timer_set_prescal(ptimer_adp, (u16)scaler);
	scaler++;
	ptimer_adp->pre_scaler = (u16)(scaler);
	ptimer_adp->tick_us = 1000000 * (scaler) / gtimer_sclk_freq[sclk_idx];
//    tick_ns_tmp = 1000000000 * (scaler) / gtimer_sclk_freq[sclk_idx];
//    tick_ns_tmp =  tick_ns_tmp - (ptimer_adp->tick_us * 1000);
//    ptimer_adp->tick_r_ns = (u32)tick_ns_tmp;
	ptimer_adp->tick_r_ns = 1000000000 / gtimer_sclk_freq[sclk_idx] * scaler;
	ptimer_adp->tick_r_ns -= (ptimer_adp->tick_us * 1000);
	return ((ptimer_adp->tick_us * 1000) + ptimer_adp->tick_r_ns);
}

/**
 *  @brief To initial a HW timer as a free run counter mode.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] tid The timer ID(index).
 *  @param[in] cnt_mode The count mode, up count or down count.
 *  @param[in] tick_us The period (in us) of a tick
 *
 *  @returns void.
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_init_free_run(phal_timer_adapter_t ptimer_adp, timer_id_t tid,
								 timer_cnt_mode_t cnt_mode, u32 tick_us, u8 tg)
{
	u32 tick_ns;
	u32 max_tick_ns;
	u8 sclk_idx;

	//dbg_printf("tg tid %x %x\r\n", tg, tid);

	hal_rtl_timer_bare_init(ptimer_adp, tid, tg);
	hal_rtl_timer_set_opmode(ptimer_adp, GTimerMode_Timer);
	hal_rtl_timer_set_cntmode(ptimer_adp, cnt_mode);
	hal_rtl_timer_set_imr(ptimer_adp, 0);    // disable interrupt

	//dbg_printf("f1\r\n");

	// Configure the pre-scale
	sclk_idx = ptimer_adp->ptg_adp->sclk_idx;
	max_tick_ns = gtimer_max_tick[sclk_idx];
	tick_ns = tick_us * 1000;
	if (tick_ns > max_tick_ns) {
		tick_ns = max_tick_ns;
	}
	hal_rtl_timer_set_tick_time(ptimer_adp, tick_ns);
	hal_rtl_timer_set_reload(ptimer_adp, 0xffffffff);
	//dbg_printf("f2\r\n");
	hal_rtl_timer_reg_toirq(ptimer_adp, (timer_callback_t)hal_timer_free_run_overflow_handler_rtl8195bhp, \
							ptimer_adp);
	hal_rtl_timer_set_imr(ptimer_adp, 1);    // enable interrupt

	hal_rtl_timer_enable(ptimer_adp);
}

/**
 *  @brief To indirect read the timer counter value
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns The timer counter value
 */
SECTION_GTIMER_TEXT
u32 hal_rtl_timer_indir_read(phal_timer_adapter_t ptimer_adp)
{
	TG_TypeDef *TG_OBJ = (TG_TypeDef *)(ptimer_adp->tg_ba);
	//tg_tsel_t tsel;
	uint32_t i;
	u32 TSEL;
	TSEL = 0;


	TSEL = TSEL | (TG_MASK_TSEL & ptimer_adp->tid);
	TSEL = TSEL | TG_BIT_POLL;

	if (ptimer_adp->ptg_adp->sclk_idx == GTimerSClk_40M) {
		TSEL = TSEL | TG_BIT_SYNC_MODE;
	}

	TG_OBJ->TG_TSEL = TSEL;

	for (i = 0; i < 1000000; i++) {
		if ((TG_OBJ->TG_TSEL & TG_BIT_POLL) == 0) {
			break;
		} else {
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
	}

	return TG_OBJ->TG_TC;
}


/**
 *  @brief To read the counter and convert it as us unit.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns The counter value in us.
 */
SECTION_GTIMER_TEXT
u64 hal_rtl_timer_read_us(phal_timer_adapter_t ptimer_adp)
{
	u32 count;
	u64 ns2us;

//    count = hal_timer_read_rtl8195bhp (ptimer_adp);
	count = hal_rtl_timer_indir_read(ptimer_adp);
	if (ptimer_adp->tick_r_ns == 0) {
		return ((u64)count * (u64)ptimer_adp->tick_us);
	} else {
		ns2us = ((u64)count * (u64)ptimer_adp->tick_r_ns) / (u64)1000;
		return ((u64)count * (u64)ptimer_adp->tick_us + ns2us);
	}
}

/**
 *  @brief To read the counter and convert it as us unit.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns The counter value in us.
 */
SECTION_GTIMER_TEXT
u64 hal_rtl_timer_read_us64(phal_timer_adapter_t ptimer_adp)
{
	uint64_t count;
	uint64_t ns2us = 0;

	count = ((uint64_t)(ptimer_adp->overflow_fired) << 32) + (uint64_t)hal_rtl_timer_indir_read(ptimer_adp);
	if (ptimer_adp->tick_r_ns == 0) {
		return ((uint64_t)count * (uint64_t)ptimer_adp->tick_us);
	} else {
		ns2us = (count * (uint64_t)ptimer_adp->tick_r_ns) / (uint64_t)1000;
		return (((uint64_t)count * (uint64_t)ptimer_adp->tick_us) + ns2us);
	}
}


/**
 *  @brief To read the counter and convert it as ns unit.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *
 *  @returns The counter value in ns.
 */
SECTION_GTIMER_TEXT
u64 hal_rtl_timer_read_ns(phal_timer_adapter_t ptimer_adp)
{
	u32 count;

//    count = hal_timer_read_rtl8195bhp (ptimer_adp);
	count = hal_rtl_timer_indir_read(ptimer_adp);
	if (ptimer_adp->tick_r_ns == 0) {
		return ((u64)count * (u64)ptimer_adp->tick_us * (u64)1000);
	} else {
		return (((u64)count * (u64)ptimer_adp->tick_us * (u64)1000) + ((u64)count * (u64)ptimer_adp->tick_r_ns));
	}
}

/**
 *  @brief To initial a HW timer as timer mode for application.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] tid The timer ID(index).
 *
 *  @returns     HAL_OK:  Setting succeed.
 *  @returns     HAL_ERR_PARA:  Input arguments are invalid.
 */
SECTION_GTIMER_TEXT
hal_status_t hal_rtl_timer_init(phal_timer_adapter_t ptimer_adp, timer_id_t tid)
{
	hal_status_t ret;

	ret = hal_rtl_timer_bare_init(ptimer_adp, tid, 0);
	if (ret != HAL_OK) {
		return ret;
	}
	hal_rtl_timer_set_opmode(ptimer_adp, GTimerMode_Timer);
	hal_rtl_timer_set_cntmode(ptimer_adp, GTimerCountDown);
	//dbg_printf("3.25\r\n");

	return ret;
}

/**
 *  @brief To initial a HW timer as timer mode for application.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] time_us The time period of timeout.
 *  @param[in] res_us The resolution of a tick time, in us.
 *                Value of 0 means didn't assign the tick resolution.
 *
 *  @returns The actual timeout value (us).
 */
SECTION_GTIMER_TEXT
u32 hal_rtl_timer_set_timeout(phal_timer_adapter_t ptimer_adp, u32 time_us, u32 res_us)
{
	u8 sclk_idx;
	u32 tick_cnt;
	u32 tick_ns;
	u32 min_tick_ns;
	u32 tick_res_ns;    // needed tick resolution
	u32 max_tick_ns;

	// Configure the pre-scaler
	sclk_idx = ptimer_adp->ptg_adp->sclk_idx;
//    min_tick_ns = (1000000000 / gtimer_sclk_freq[sclk_idx]);
	min_tick_ns = gtimer_min_tick[sclk_idx];
	max_tick_ns = gtimer_max_tick[sclk_idx];

//    tick_ns = min_tick_ns * 1000;   // maximum tick time (@ pre-scaler = 1000)
	if ((time_us <= 10) || ((time_us % 10) != 0)) {
		tick_res_ns = 1000;
	} else if ((time_us <= 100) || ((time_us % 100) != 0)) {
		tick_res_ns = 10000;
	} else if ((time_us <= 1000) || ((time_us % 1000) != 0)) {
		tick_res_ns = 100000;
	} else if ((time_us <= 10000) || ((time_us % 10000) != 0)) {
		tick_res_ns = 1000000;
	} else {
		tick_res_ns = 10000000;
	}
	// try to set the tick time which will count at least 2 ticks to timeout
	if (res_us == 0) {
		// didn't assign the resolution, give a bigger value
		res_us = time_us >> 1;
	}

	tick_ns = tick_res_ns;
	if (tick_ns > min_tick_ns) {
		uint32_t time_ns = time_us * 1000;
		uint32_t res_ns = res_us * 1000;

		while (((tick_ns << 1) > (time_ns)) ||
			   (tick_ns > (res_ns))) {
			// try smaller tick time
			tick_ns =  tick_ns >> 1;
			if (tick_ns <= min_tick_ns) {
				tick_ns = min_tick_ns;
				break;
			}
		}

		if (tick_ns > max_tick_ns) {
			tick_ns = max_tick_ns;
		}
	} else {
		tick_ns = min_tick_ns;
	}

//    dbg_printf("tick_ns %x\r\n", tick_ns);
	tick_ns = hal_rtl_timer_set_tick_time(ptimer_adp, (u32)tick_ns);  // Set prescale
	// set the counter reload
	tick_cnt = ((u64)time_us * 1000) / (ptimer_adp->tick_us * 1000 + ptimer_adp->tick_r_ns);

	if (ptimer_adp->tick_r_ns == 0) {
		while ((tick_cnt * ptimer_adp->tick_us) < time_us) {
			tick_cnt++;
		}
	} else {
		while (((tick_cnt * ptimer_adp->tick_us) + ((tick_cnt * ptimer_adp->tick_r_ns) / 1000)) < time_us) {
			tick_cnt++;
		}
	}
	if (tick_cnt > 0) {
		// timeout = tick_time * (reload_count + 1)
		tick_cnt--;
	}
	hal_rtl_timer_set_reload(ptimer_adp, tick_cnt);                    // Set Load Count
	tick_cnt++;
	if (ptimer_adp->tick_r_ns == 0) {
		return (tick_cnt * ptimer_adp->tick_us);
	} else {
		return ((tick_cnt * ptimer_adp->tick_us) + ((tick_cnt * ptimer_adp->tick_r_ns) / 1000));
	}
}

/**
 *  @brief To start an initialed timer.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] reload_mode The timer counter reload mode, one shot or periodical.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_start(phal_timer_adapter_t ptimer_adp, timer_app_mode_t reload_mode)
{
	ptimer_adp->reload_mode = reload_mode;
	hal_rtl_timer_set_opmode(ptimer_adp, reload_mode);
	hal_rtl_timer_enable(ptimer_adp);
}

/**
 *  @brief To enable and setup a counter match event for a running timer.
 *         The function hal_rtl_timer_set_timeout() must be called
 *         befor calling of this function.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The match event number(0 ~ 3).
 *  @param[in] time_us The time period for the timer match event. This time period will be
 *                 converted to a tick count to set to the match event counter.
 *
 *  @returns     HAL_OK:  Setting succeed.
 *  @returns     HAL_ERR_PARA:  Input arguments are invalid.
 *  @returns     HAL_NOT_READY:  Error with data not ready.
 */
SECTION_GTIMER_TEXT
hal_status_t hal_rtl_timer_enable_match_event(phal_timer_adapter_t ptimer_adp, timer_match_event_t match_ev, u32 time_us)
{
	u32 tick_cnt;
	u32 reload_cnt;

	if (match_ev > GTimerMatchEvent3) {
		DBG_TIMER_ERR("hal_timer_enable_match_event: invalid event index(%u)!!\r\n", match_ev);
		return HAL_ERR_PARA;
	}

	if ((ptimer_adp->tick_us == 0) && (ptimer_adp->tick_r_ns == 0)) {
		DBG_TIMER_ERR("hal_timer_enable_match_event: hal_timer_set_timeout must be called first!!\r\n");
		return HAL_NOT_READY;
	}

	//reload_cnt = ptimer_adp->tmr_ba->lc;
	reload_cnt = ptimer_adp->tmr_ba->TM_LC;

	// ptimer_adp->tick_us & ptimer_adp->tick_r_ns are calculated in hal_timer_set_timeout()
	tick_cnt = (time_us * 1000) / (ptimer_adp->tick_us * 1000 + ptimer_adp->tick_r_ns);
	if (reload_cnt <= tick_cnt) {
		DBG_TIMER_ERR("hal_timer_enable_match_event: match counter(%lu) > timeout counter(%lu)!!\r\n",
					  tick_cnt, reload_cnt);
		return HAL_ERR_PARA;
	}

	// since timer is using count-down mode, so tick count = reload ticks - time period ticks.
	//tick_cnt = ptimer_adp->tmr_ba->lc - tick_cnt;
	tick_cnt = ptimer_adp->tmr_ba->TM_LC - tick_cnt;
	hal_rtl_timer_set_me_counter(ptimer_adp, match_ev, tick_cnt);
	hal_rtl_timer_me_ctrl(ptimer_adp, match_ev, 1);
	return HAL_OK;
}

/**
 *  @brief To disable a counter match event.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] match_ev The match event number(0 ~ 3).
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_disable_match_event(phal_timer_adapter_t ptimer_adp, timer_match_event_t match_ev)
{
	if (match_ev > GTimerMatchEvent3) {
		return;
	}

	hal_rtl_timer_me_ctrl(ptimer_adp, match_ev, 0);
}

/**
 *  @brief To start an initialed timer with one shot mode.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] time_us The time period of timeout.
 *  @param[in] callback The call back function for timeout event.
 *  @param[in] phid The argument for call back function calling.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_start_one_shot(phal_timer_adapter_t ptimer_adp, u32 time_us,
								  timer_callback_t callback, void *phid)
{
	// hook interrupt call back
	if (callback != (timer_callback_t)NULL) {
		hal_rtl_timer_reg_toirq(ptimer_adp, callback, phid);
		hal_rtl_timer_set_imr(ptimer_adp, 1);    // enable interrupt
	}
	hal_rtl_timer_set_timeout(ptimer_adp, time_us, 0);
	hal_rtl_timer_start(ptimer_adp, GTimerOneShot);
}

/**
 *  @brief To start an initialed timer with periodical mode.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] time_us The time period of timeout.
 *  @param[in] callback The call back function for timeout event.
 *  @param[in] phid The argument for call back function calling.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_start_periodical(phal_timer_adapter_t ptimer_adp, u32 time_us,
									timer_callback_t callback, void *phid)
{
	// hook interrupt call back
	if (callback != (timer_callback_t)NULL) {
		hal_rtl_timer_reg_toirq(ptimer_adp, callback, phid);
		hal_rtl_timer_set_imr(ptimer_adp, 1);    // enable interrupt
	}
	hal_rtl_timer_set_timeout(ptimer_adp, time_us, 0);
	hal_rtl_timer_start(ptimer_adp, GTimerPeriodical);
}

/**
 *  @brief To allocate a free timer device.
 *
 *  @param[in] timer_list A list of timer ID, the caller limit the allocated timer
 *                    should be one of this list. The list should end with MaxGTimerNum.
 *                    If the list is NULL, then no limit to the timer allocation.
 *
 *  @returns The allocated timer ID. If allocation failed, it will return 0xFF.
 */
SECTION_GTIMER_TEXT
timer_id_t hal_rtl_timer_allocate(u8 *timer_list)
{
	timer_id_t i;
	timer_id_t tid = 0xFF;
	timer_id_t tg_tid;
	hal_timer_group_adapter_t *ptimer_group;

	if (timer_list == NULL) {
		ptimer_group = ptimer_group0;
		for (i = 0; i < MaxGTimerNum; i++) {
			if (i == GTIMER_GROUP_TIMER_NUM) {
				ptimer_group = ptimer_group2;
			}
			if ((ptimer_group->tmr_in_use & (1 << i)) == 0) {
				break;
			}
		}

		tid = i;
	} else {
		for (i = 0; i < MaxGTimerNum; i++) {
			tid = timer_list[i];
			if (tid > 8) {                      // For id > 9 TG2 allocate
				tid = tid - 1;
			}
			if (tid < MaxGTimerNum) {
				if (tid < GTIMER_GROUP_TIMER_NUM) {
					ptimer_group = ptimer_group0;
				} else {
					ptimer_group = ptimer_group2;
				}
				tg_tid = tid %  GTIMER_GROUP_TIMER_NUM;

				if ((ptimer_group->tmr_in_use & (1 << tg_tid)) == 0) {
					// found a free timer
					break;      // break the for loop
				}
			} else {
				// end of list
				break;
			}
		}
	}

	if (tid < MaxGTimerNum) {
		return tid;
	} else {
		return 0xFF;
	}
}

/**
 *  @brief To allocate and configure a timer to use as a tick source for PWM or ADC.
 *
 *  @param[in] ptimer_adp The timer adapter.
 *  @param[in] ptick_us Assign the tick time for the allocated timer should generate out.
 *         This pointer also be used to return the actual tick time (us).
 *  @param[in] timer_list The timer ID list. It should allocate a timer from this list.
 *
 *  @returns The allocated timer ID.
 */

SECTION_GTIMER_TEXT
timer_id_t hal_rtl_timer_event_init(phal_timer_adapter_t ptimer_adp, u32 *ptick_us, timer_id_t *timer_list)
{
	timer_id_t tid;
	hal_status_t ret;
	u32 act_tick_us;

	if (timer_list == NULL) {
		// no assign feverate timer, use default list
		timer_list = (timer_id_t *)pwm_avail_timer_list;
	}

	// Only the Timer Group1's timer can be used as the tick source of PWM
	tid = hal_rtl_timer_allocate(timer_list);

	if (tid >= GTIMER_GROUP_TIMER_NUM) {
		DBG_TIMER_ERR("hal_rtl_timer_event_init: no free timer\r\n");
		return 0xFF;    // PwmClkSrc_None
	}

	// Initial the G-timer as free run with the given tick time
	ret = hal_rtl_timer_init(ptimer_adp, tid);
	act_tick_us = hal_rtl_timer_set_timeout(ptimer_adp, *ptick_us, 0);
	hal_rtl_timer_set_imr(ptimer_adp, 0);    // disable interrupt

	ptimer_group0->timer_adapter[tid] = NULL;   // this timer adapter will not be save;

	if (ret == HAL_OK) {
		*ptick_us = act_tick_us;
		hal_rtl_timer_start(ptimer_adp, GTimerPeriodical);
	} else {
		return 0xFF;    // PwmClkSrc_None
	}

	return tid;
}

/**
 *  @brief To disable a HW timer which is used as the tick source for ADC or PWM.
 *
 *  @param[in] tid The timer index.
 *
 *  @returns void
 */
SECTION_GTIMER_TEXT
void hal_rtl_timer_event_deinit(timer_id_t tid)
{
	TM_TypeDef *TM_OBJ = (TM_TypeDef *)timer_base_address[tid];
	TM_OBJ->TM_CTRL = TM_OBJ->TM_CTRL & (~TM_BIT_EN);

	ptimer_group0->tmr_in_use &= ~(1 << (tid));

}

/** @} */ /* End of group hs_hal_timer_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_timer */

#endif  // end of "#if CONFIG_GTIMER_EN"

