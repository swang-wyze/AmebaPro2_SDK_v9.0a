
/**************************************************************************//**
 * @file     rtl8195bhp_pwm.c
 * @brief    This file implements the PWM HAL functions.
 *
 * @version  V1.00
 * @date     2016-07-22
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
#include "rtl8735b_pwm.h"
#include "rtl8735b_pin_name.h"
//#include "rtl8735b_pwm_type.h"


#if CONFIG_PWM_EN && CONFIG_GTIMER_EN

#define SECTION_PWM_TEXT           SECTION(".rom.hal_pwm.text")
#define SECTION_PWM_DATA           SECTION(".rom.hal_pwm.data")
#define SECTION_PWM_RODATA         SECTION(".rom.hal_pwm.rodata")
#define SECTION_PWM_BSS            SECTION(".rom.hal_pwm.bss")
#define SECTION_PWM_STUBS          SECTION(".rom.hal_pwm.stubs")

extern void *_memset(void *dst0, int Val, SIZE_T length);
extern phal_timer_group_adapter_t ptimer_group2;
/**
 * @addtogroup hs_hal_pwm PWM
 * @{
 */
void PWM_IRQHandler(void);

#if !defined(CONFIG_BUILD_NONSECURE)
#undef PWM_COMM_REG_BASE
#define PWM_COMM_REG_BASE (PWM_COMM_S)

#else

#undef PWM_COMM_REG_BASE
#define PWM_COMM_REG_BASE (PWM_COMM)

#endif

#if !defined(CONFIG_BUILD_NONSECURE)
#undef PWM0_REG_BASE
#define PWM0_REG_BASE (PWM0_S)
#undef PWM1_REG_BASE
#define PWM1_REG_BASE (PWM1_S)
#undef PWM2_REG_BASE
#define PWM2_REG_BASE (PWM2_S)
#undef PWM3_REG_BASE
#define PWM3_REG_BASE (PWM3_S)
#undef PWM4_REG_BASE
#define PWM4_REG_BASE (PWM4_S)
#undef PWM5_REG_BASE
#define PWM5_REG_BASE (PWM5_S)
#undef PWM6_REG_BASE
#define PWM6_REG_BASE (PWM6_S)
#undef PWM7_REG_BASE
#define PWM7_REG_BASE (PWM7_S)
#undef PWM8_REG_BASE
#define PWM8_REG_BASE (PWM8_S)
#undef PWM9_REG_BASE
#define PWM9_REG_BASE (PWM9_S)
#undef PWM10_REG_BASE
#define PWM10_REG_BASE (PWM10_S)
#undef PWM11_REG_BASE
#define PWM11_REG_BASE (PWM11_S)

#else

#undef PWM0_REG_BASE
#define PWM0_REG_BASE (PWM0)
#undef PWM1_REG_BASE
#define PWM1_REG_BASE (PWM1)
#undef PWM2_REG_BASE
#define PWM2_REG_BASE (PWM2)
#undef PWM3_REG_BASE
#define PWM3_REG_BASE (PWM3)
#undef PWM4_REG_BASE
#define PWM4_REG_BASE (PWM4)
#undef PWM5_REG_BASE
#define PWM5_REG_BASE (PWM5)
#undef PWM6_REG_BASE
#define PWM6_REG_BASE (PWM6)
#undef PWM7_REG_BASE
#define PWM7_REG_BASE (PWM7)
#undef PWM8_REG_BASE
#define PWM8_REG_BASE (PWM8)
#undef PWM9_REG_BASE
#define PWM9_REG_BASE (PWM9)
#undef PWM10_REG_BASE
#define PWM10_REG_BASE (PWM10)
#undef PWM11_REG_BASE
#define PWM11_REG_BASE (PWM11)


#endif


/**
  * @brief The table of all PWM registers base address.
  */

SECTION_PWM_RODATA
const PWM_TypeDef *pwm_base_address[MaxPwmNum] = {
	PWM0_REG_BASE, PWM1_REG_BASE, PWM2_REG_BASE, PWM3_REG_BASE,
	PWM4_REG_BASE, PWM5_REG_BASE, PWM6_REG_BASE, PWM7_REG_BASE,
	PWM8_REG_BASE, PWM9_REG_BASE, PWM10_REG_BASE, PWM11_REG_BASE
};

//SECTION_PWM_RODATA
//const PWM_Type *pwm_base_address[MaxPwmNum] = {
//    PWM0, PWM1, PWM2, PWM3,
//    PWM4, PWM5, PWM6, PWM7,
//    PWM8, PWM9, PWM10, PWM11
//};





/**
  * @brief The table of all PWM pin selection registers address.
  */
SECTION_PWM_RODATA
u32 pwm_pin_table[2 * MaxPwmNum] = {
	PIN_F6, PIN_F7, PIN_F8, PIN_F9, PIN_F10, PIN_F11, PIN_F12, PIN_F13, PIN_F14, PIN_F15, PIN_F16, PIN_F17,  // S0
	PIN_NC, PIN_NC, PIN_NC, PIN_NC, PIN_NC,  PIN_NC,  PIN_NC,  PIN_NC,  PIN_S1,  PIN_S4,  PIN_S5,  PIN_S6   // S1

};


/**
  * @brief The global data structure to handle common resource
  *        for all PWM adapters.
  */
SECTION_PWM_BSS hal_pwm_comm_adapter_t *ppwm_comm_adapter;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_pwm_rom_func PWM HAL ROM APIs.
 * @ingroup hs_hal_pwm
 * @{
 * @brief The PWM HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of PWM HAL APIs in the RAM space is provided for the user application.
 */

/**
  \brief  The stubs functions table of the PWM HAL functions in ROM.
*/
SECTION_PWM_STUBS const hal_pwm_func_stubs_t hal_pwm_stubs = {
	.pppwm_comm_adp = &ppwm_comm_adapter,
	.pwm_pin_table = &pwm_pin_table[0],
	.hal_pwm_irq_handler = PWM_IRQHandler,
	.hal_pwm_comm_irq_reg = hal_rtl_pwm_comm_irq_reg,
	.hal_pwm_comm_irq_unreg = hal_rtl_pwm_comm_irq_unreg,
	.hal_pwm_comm_init = hal_rtl_pwm_comm_init,
	.hal_pwm_comm_tick_source_list = hal_rtl_pwm_comm_tick_source_list,
	.hal_pwm_init = hal_rtl_pwm_init,
	.hal_pwm_comm_deinit = hal_rtl_pwm_comm_deinit,
	.hal_pwm_enable_sts = hal_rtl_pwm_enable_sts,
	.hal_pwm_comm_enable = hal_rtl_pwm_comm_enable,
	.hal_pwm_comm_disable = hal_rtl_pwm_comm_disable,
	.hal_pwm_enable = hal_rtl_pwm_enable,
	.hal_pwm_disable = hal_rtl_pwm_disable,
	.hal_pwm_deinit = hal_rtl_pwm_deinit,
	.hal_pwm_set_clk_sel = hal_rtl_pwm_set_clk_sel,
	.hal_pwm_wait_ctrl_ready = hal_rtl_pwm_wait_ctrl_ready,
	.hal_pwm_set_tick_time = hal_rtl_pwm_set_tick_time,
	.hal_pwm_set_duty = hal_rtl_pwm_set_duty,
	.hal_pwm_read_duty = hal_rtl_pwm_read_duty,
	.hal_pwm_change_duty = hal_rtl_pwm_change_duty,
	.hal_pwm_set_duty_limit = hal_rtl_pwm_set_duty_limit,
	.hal_pwm_set_auto_duty_adj = hal_rtl_pwm_set_auto_duty_adj,
	.hal_pwm_auto_duty_en = hal_rtl_pwm_auto_duty_en,
	.hal_pwm_set_auto_duty_inc = hal_rtl_pwm_set_auto_duty_inc,
	.hal_pwm_set_auto_duty_dec = hal_rtl_pwm_set_auto_duty_dec,
	.hal_pwm_set_auto_duty_loop = hal_rtl_pwm_set_auto_duty_loop,
	.hal_pwm_set_period_int = hal_rtl_pwm_set_period_int,
	.hal_pwm_set_autoadj_int = hal_rtl_pwm_set_autoadj_int,
	.hal_pwm_set_autoadj_loop_int = hal_rtl_pwm_set_autoadj_loop_int,
	.hal_pwm_auto_duty_inc = hal_rtl_pwm_auto_duty_inc,
	.hal_pwm_auto_duty_dec = hal_rtl_pwm_auto_duty_dec,
	.hal_pwm_auto_duty_loop = hal_rtl_pwm_auto_duty_loop,
	.hal_pwm_stop_duty_loop = hal_rtl_pwm_stop_duty_loop,
	.hal_pwm_set_duty_ns = hal_rtl_pwm_set_duty_ns,
	.hal_pwm_auto_duty_ns_inc = hal_rtl_pwm_auto_duty_ns_inc,
	.hal_pwm_auto_duty_ns_dec = hal_rtl_pwm_auto_duty_ns_dec,
	.hal_pwm_read_duty_ns = hal_rtl_pwm_read_duty_ns,
	.hal_pwm_dma_init     = hal_rtl_pwm_dma_init,
	.hal_pwm_dma_send     = hal_rtl_pwm_dma_send,
	.hal_pwm_cache        = hal_rtl_pwm_cache,
	.hal_pwm_complementary = hal_rtl_pwm_complementary,
#if !defined(CONFIG_BUILD_NONSECURE)
	.hal_pwm_clk_sel = hal_rtl_pwm_clk_sel,
	.hal_pwm_clock_init = hal_rtl_pwm_clock_init
#endif
};

/**
 *  \brief To enable or disable a PWM function block.
 *
 *  \param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_rtl_pwm_en_ctrl(BOOL en)
{
	//SYSON->hs_tef_ctrl_b.tef_en = en;
}

/**
 *  \brief To enable/disable the APB clock for PWM.
 *
 *  \param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_rtl_pwm_pclk_ctrl(BOOL en)
{
	//SYSON->hs_tef_ctrl_b.tef_pclk_en = en;
}

/**
 *  \brief To enable/disable the system clock for PWM.
 *
 *  \param[in] en  Enable control: 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_rtl_pwm_sclk_ctrl(BOOL en)
{
	//SYSON->hs_tef_ctrl_b.tef_sclk_en = en;
}

/**
 *  \brief To select the clock source for PWM interrupt detection.
 *
 *  \param[in] int_clk  The clock source selection: 0: APB clock, 1: S-CLK.
 *                  When entering sleep mode, the interrupt clock source
 *                  should be switched to 32K for wake-up by GPIO interrupt.
 *
 *  \returns void
 */
#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_PWM_TEXT
void hal_rtl_pwm_int_clk_sel(pwm_int_clk_t int_clk)
{
	u32 val;
	val = HAL_READ32(0x40009800, 0x2C);
	if (int_clk == PWM_IntClk_APB) {
		val = val & ~(PON_BIT_INTR_CLK_PWM_SEL);
	} else if (int_clk == PWM_IntClk_32K) {
		val = val & ~(PON_BIT_INTR_CLK_PWM_SEL);
		val = val | PON_BIT_INTR_CLK_PWM_SEL;
	}
	HAL_WRITE32(0x40009800, 0x2C, val);

}
#endif


#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_PWM_TEXT
void hal_rtl_pwm_sclk_sel(int sclk_sel)
{

	hal_rtl_sys_set_clk(PWM_SYS, sclk_sel);

//    u32 val;
//
//    val = HAL_READ32(0x40009800, 0x2C);
//    if(sclk_sel == PWM_Sclk_32k){
//        val = val & ~(PON_MASK_SCLK_PWM_SEL);
//    }
//    else if(sclk_sel == PWM_Sclk_4M){
//        val = val & ~(PON_MASK_SCLK_PWM_SEL);
//        val = val | ( (sclk_sel) << PON_SHIFT_SCLK_PWM_SEL);
//    }
//    else if(sclk_sel == PWM_Sclk_40M){
//        val = val & ~(PON_MASK_SCLK_PWM_SEL);
//        val = val | ( (sclk_sel) << PON_SHIFT_SCLK_PWM_SEL);
//    }
//
//    HAL_WRITE32(0x40009800, 0x2C, val);

}
#endif


/**
 *  \brief To handle the PWM interrupt.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void PWM_IRQHandler(void)
{
	hal_pwm_comm_adapter_t *ppwm_com_adp = ppwm_comm_adapter;
	hal_pwm_adapter_t *ppwm_adp;
	//pwm_comm_int_status_t int_sts;
	//pwm_comm_int_status_t int_sts1;
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;
	u32 i;
	u32 j;
	u32 duty;
	u8 dn_lim_int;
	u8 up_lim_int;
	u8 period_int;

	hal_rtl_irq_clear_pending(PWM_IRQn);

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_com_adp->base_addr);
	//PWM_TypeDef* PWM_OBJ = (PWM_TypeDef*)(ppwm_adp->base_addr);

	u32 int_sts_u, int_sts1_u;
	int_sts_u = PWM_COMM_OBJ->PWM_COMM_INT_STATUS;
	int_sts1_u = PWM_COMM_OBJ->PWM_COMM_INT_STATUS1;

	PWM_COMM_OBJ->PWM_COMM_INT_STATUS |= int_sts_u;
	PWM_COMM_OBJ->PWM_COMM_INT_STATUS1 |= int_sts1_u;


	for (i = 0; i < 12; i++) { // MaxPwmNum
		//dbg_printf("i %x\r\n", i);
		if (ppwm_com_adp->pwm_adapter[i] != NULL) {
			ppwm_adp = ppwm_com_adp->pwm_adapter[i];
			PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

			dn_lim_int = (((int_sts_u & PWM_COMM_MASK_DUTY_ADJ_DN_LIM) >> PWM_COMM_SHIFT_DUTY_ADJ_DN_LIM)  >> i) &  0x01;
			up_lim_int = (((int_sts_u & PWM_COMM_MASK_DUTY_ADJ_UP_LIM) >> PWM_COMM_SHIFT_DUTY_ADJ_UP_LIM)   >> i) &  0x01;
			period_int = (((int_sts_u & PWM_COMM_MASK_PERIOD_END)       >> PWM_COMM_SHIFT_PERIOD_END)       >> i) &  0x01;


#if 1
			if (i > 7) {
				//dbg_printf("index %x %x\r\n", i, int_sts1.b.period_end);
//                j = i - 8;
//                dn_lim_int = (int_sts1.b.duty_adj_dn_lim >> j) & 0x01;
//                up_lim_int = (int_sts1.b.duty_adj_up_lim >> j) & 0x01;
//                period_int = (int_sts1.b.period_end >> j) & 0x01;

				j = i - 8;
				dn_lim_int = (((int_sts1_u & PWM_COMM_MASK_DUTY_ADJ_DN_LIM) >> PWM_COMM_SHIFT_DUTY_ADJ_DN_LIM)  >> j) &  0x01;
				up_lim_int = (((int_sts1_u & PWM_COMM_MASK_DUTY_ADJ_UP_LIM) >> PWM_COMM_SHIFT_DUTY_ADJ_UP_LIM)   >> j) &  0x01;
				period_int = (((int_sts1_u & PWM_COMM_MASK_PERIOD_END)       >> PWM_COMM_SHIFT_PERIOD_END)       >> j) &  0x01;

			}
#endif


			//pwm_adj_ctrl.w = ppwm_adp->base_addr->auto_adj_ctrl;
			u32 ADJ_CTRL;
			ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;


			if (dn_lim_int || up_lim_int) {
				// if not in auto adjustment loop mode, disable the interrupt
				if (ppwm_adp->duty_adj.loop_mode == PwmDutyLoopCnt) {
					if (ppwm_adp->adj_loop_count == 0) {
						ppwm_adp->duty_adj.loop_mode = 0;
						// Disable loop mode
//                            pwm_adj_ctrl.b.adj_dir = dn_lim_int; // ovride HW setting for next loop
//                            pwm_adj_ctrl.b.adj_loop_en = 0;
//                            pwm_adj_ctrl.b.adj_en = 0;

						if (dn_lim_int == 1) {
							ADJ_CTRL |= PWM_BIT_ADJ_DIR;
						} else {
							ADJ_CTRL &= (~PWM_BIT_ADJ_DIR);
						}
						ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);
						ADJ_CTRL &= (~PWM_BIT_ADJ_EN);

						// Disable Interrupt
//                            pwm_adj_ctrl.b.duty_dn_lim_ie = 0;
//                            pwm_adj_ctrl.b.duty_up_lim_ie = 0;
//                            ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;

						ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
						ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
						PWM_OBJ->PWM_AUTO_ADJ_CTRL = ADJ_CTRL;

						// woraround for loop mode switch to non-loopmoce
						// write the duty limit to the duty size control
						if (up_lim_int) {
							//duty = ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_up_lim;
							duty = (PWM_OBJ->PWM_AUTO_ADJ_LIMIT & PWM_MASK_DUTY_ADJ_UP_LIM) >> PWM_SHIFT_DUTY_ADJ_UP_LIM;
						} else {
							//duty = ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_dn_lim;
							duty = (PWM_OBJ->PWM_AUTO_ADJ_LIMIT & PWM_MASK_DUTY_ADJ_DN_LIM) >> PWM_SHIFT_DUTY_ADJ_DN_LIM;
						}
						//ppwm_adp->base_addr->ctrl_b.duty = duty;
						PWM_OBJ->PWM_CTRL &= (~PWM_MASK_DUTY);
						PWM_OBJ->PWM_CTRL |= (duty & PWM_MASK_DUTY);
						//ppwm_adp->base_addr->ctrl_b.ctrl_set = 1;
						PWM_OBJ->PWM_CTRL |= PWM_BIT_CTRL_SET;

						if (ppwm_adp->loopout_callback != NULL) {
							ppwm_adp->loopout_callback(ppwm_adp->lo_cb_para);
						}
					} else {
						ppwm_adp->adj_loop_count--;
						if (ppwm_adp->adj_loop_count == 0) {
							//pwm_adj_ctrl.b.adj_loop_en = 0;
							ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);
						}

						if (up_lim_int) {
//                            pwm_adj_ctrl.b.duty_up_lim_ie = 0;
//                            pwm_adj_ctrl.b.duty_dn_lim_ie = 1;

							ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
							ADJ_CTRL |= (PWM_BIT_DUTY_DN_LIM_IE);

						} else {
//                            pwm_adj_ctrl.b.duty_up_lim_ie = 1;
//                            pwm_adj_ctrl.b.duty_dn_lim_ie = 0;

							ADJ_CTRL |= (PWM_BIT_DUTY_UP_LIM_IE);
							ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
						}
						//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;
						PWM_OBJ->PWM_AUTO_ADJ_CTRL = ADJ_CTRL;
					}
				} else if (ppwm_adp->duty_adj.loop_mode == PwmDutyLoopOff) {
//                        pwm_adj_ctrl.b.duty_dn_lim_ie = 0;
//                        pwm_adj_ctrl.b.duty_up_lim_ie = 0;
//                        pwm_adj_ctrl.b.adj_en = 0;
//                        ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;

					ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
					ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
					ADJ_CTRL &= (~PWM_BIT_ADJ_EN);
					PWM_OBJ->PWM_AUTO_ADJ_CTRL = ADJ_CTRL;
				}

				if (ppwm_adp->adj_int_en & PwmAdjIntUpLim) {
					//pwm_adj_ctrl.b.duty_up_lim_ie = 1;
					ADJ_CTRL |= PWM_BIT_DUTY_UP_LIM_IE;
				}

				if (ppwm_adp->bound_callback != NULL) {
					// Use the Up limit interrupt status as the direction indication
					if ((up_lim_int && (ppwm_adp->adj_int_en & PwmAdjIntUpLim)) ||
						(dn_lim_int && (ppwm_adp->adj_int_en & PwmAdjIntDnLim))) {
						ppwm_adp->bound_callback(ppwm_adp->bound_cb_para, up_lim_int);
					}
				}
			}

			if (period_int) {
				// interrupt on period time end, this is a proper time to modify the duty ratio or duty auto-adjustment
				//dbg_printf("period_int\r\n");
				if (ppwm_adp->period_callback != NULL) {
					// call the user PWM period end callback function
					// User may need to disable the period end interrupt in the calback function
					//dbg_printf("period_callback\r\n");
					ppwm_adp->period_callback(ppwm_adp->pe_cb_para);
				}
			}
		}
	}

	__DSB();
}

/**
 *  \brief To register a IRQ handler for the PWM common interrupt.
 *
 *  \param[in] irq_handler The IRQ handler.
 *  \param[in] arg The argument of the IRQ handler.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_irq_reg(irq_handler_t irq_handler)
{
	// IRQ vector may has been registered, disable and re-register it
	hal_rtl_irq_disable(PWM_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(PWM_IRQn, (uint32_t)irq_handler);
	hal_rtl_irq_set_priority(PWM_IRQn, PWM_IRQPri);
	hal_rtl_irq_enable(PWM_IRQn);
	//dbg_printf("pwm irq\r\n");
}

/**
 *  \brief To un-register the PWM common IRQ handler.
 *
 *  \param void.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_irq_unreg(void)
{
	hal_rtl_irq_disable(PWM_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(PWM_IRQn, (uint32_t)_default_handler);
}

#if 0
/**
 *  @brief To register a PWM duty ratio reachs the limit IRQ call back function
 *
 *  @param ppwm_adp The PWM device adapter.
 *  @param callback The call back function.
 *  @param phid The argument for call back function calling.
 *
 *  @returns void
 */
SECTION_PWM_TEXT
void hal_pwm_limirq_reg_rtl8195bhp(hal_pwm_adapter_t *ppwm_adp, pwm_lim_callback_t callback, void *phid)
{
	ppwm_adp->bound_callback = callback;
	ppwm_adp->bound_cb_para = phid;
}

/**
 *  @brief To un-register a PWM duty ratio reachs the limit IRQ.
 *
 *  @param ppwm_adp The PWM device adapter.
 *
 *  @returns void
 */
SECTION_PWM_TEXT
void hal_pwm_limirq_unreg_rtl8195bhp(hal_pwm_adapter_t *ppwm_adp)
{
	ppwm_adp->bound_callback = NULL;
	ppwm_adp->bound_cb_para = NULL;
}
#endif


#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_PWM_TEXT
void hal_rtl_pwm_clock_init(int en)
{
	hal_rtl_sys_peripheral_en(PWM_SYS, en);

}
#endif

#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_PWM_TEXT
void hal_rtl_pwm_clk_sel(int Sclk)
{
	hal_rtl_pwm_sclk_sel(Sclk);
}
#endif

/**
 *  \brief To initial the PWM devices common adapter. This function must be called first, before call
 *         any other PWM functions.
 *
 *  \param[in] ppwm_com_adp The PWM devices common adapter.
 *
 *  \returns void
 */
#define syson_ready 0

SECTION_PWM_TEXT
void hal_rtl_pwm_comm_init(hal_pwm_comm_adapter_t *ppwm_com_adp)
{
	u32 i;
	//u32 val;
	//int sclk_sel;

	ppwm_comm_adapter = ppwm_com_adp;
	ppwm_comm_adapter->base_addr = PWM_COMM; // PWM_COMM_REG_BASE
	for (i = 0; i < MaxPwmNum; i++) {
		ppwm_comm_adapter->pwm_adapter[i] = NULL;
	}
	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	// Clear all interrupt status: write clear
	PWM_COMM_OBJ->PWM_COMM_INT_STATUS = 0xFFFFFFFF;
	PWM_COMM_OBJ->PWM_COMM_INT_STATUS1 = 0xFFFFFFFF;

	// Register IRQ
	hal_rtl_pwm_comm_irq_reg((irq_handler_t)PWM_IRQHandler);


}

/**
 *  \brief To de-initial the PWM devices common adapter.
 *         The whole will be disabled and the clock will be gated.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_deinit(void)
{
	if (ppwm_comm_adapter == NULL) {
		return;
	}

	hal_rtl_pwm_comm_disable(0xFF);      // disable all PWM
	// Clear all interrupt status: write clear
	//ppwm_comm_adapter->base_addr->int_status= 0xFFFFFFFF;
	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);
	PWM_COMM_OBJ->PWM_COMM_INT_STATUS = 0xFFFFFFFF;

	// PWM function and clock disable
	hal_rtl_pwm_pclk_ctrl(OFF);
	hal_rtl_pwm_sclk_ctrl(OFF);
	hal_rtl_pwm_en_ctrl(OFF);

	ppwm_comm_adapter = NULL;
}

/**
 *  \brief To give a list of g-timer ID as the tick source of PWM devices.
 *         The list should end with 0xFF
 *
 *  \param[in] timer_list The timer ID list those can be used as the PWM tick source.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_tick_source_list(uint8_t *timer_list)
{
	ppwm_comm_adapter->timer_list = timer_list;
}

/**
 *  \brief To initial a PWM devices adapter. This is the first function must be called
 *         before to do any operation to the PWM devices.
 *
 *  \param[in] ppwm_adp The PWM devices adapter.
 *  \param[in] pwm_id The PWM devices index, valid value is 0 ~ (MaxPwmNum - 1).
 *  \param[in] duty_res_us The resolution for the duty duration setting, in us.
 *                     The value 0 means the duty resolution will be 1/4000 of PWM period.
 *
 *  \return     HAL_OK:  Setting succeed.
 *  \return     HAL_ERR_PARA:  Input arguments are invalid.
 *  \return     HAL_NOT_READY: Error with data not ready.
 */

#define pinmux_ready 0

SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_init(hal_pwm_adapter_t *ppwm_adp, u8 pwm_id, u8 pin_sel, u16 duty_res_us)
{
#if 0
	SYSON_Type *syson = SYSON;
#endif

	//dbg_printf("hal_pwm_init\r\n");

	if (ppwm_comm_adapter == NULL) {
		DBG_PWM_ERR("hal_pwm_init: PWM Com Struct No Initial!!\r\n");
		return HAL_NOT_READY;
	}

	if (pwm_id >= MaxPwmNum) {
		DBG_PWM_ERR("hal_pwm_init: Invalid PWM index\r\n");
		return HAL_ERR_PARA;
	}

	_memset((void *)ppwm_adp, 0, sizeof(hal_pwm_adapter_t));
	ppwm_adp->pwm_id = pwm_id;
	ppwm_adp->pin_sel = pin_sel;
	ppwm_adp->base_addr = (PWM_TypeDef *)pwm_base_address[pwm_id];
	ppwm_adp->pwm_clk_sel = PwmClkSrc_None;
	ppwm_adp->duty_res_us = duty_res_us;
	ppwm_comm_adapter->pwm_adapter[pwm_id] = ppwm_adp;

	//dbg_printf("pwm_id %x\r\n", pwm_id);

#if 0

	/* PWM function on and pin selection */
	switch (pwm_id) {
	case 0:
		syson->hs_tef_ctrl_b.pwm0_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm0_pin_en = 1;
		break;

	case 1:
		syson->hs_tef_ctrl_b.pwm1_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm1_pin_en = 1;
		break;

	case 2:
		syson->hs_tef_ctrl_b.pwm2_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm2_pin_en = 1;
		break;

	case 3:
		syson->hs_tef_ctrl_b.pwm3_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm3_pin_en = 1;
		break;

	case 4:
		syson->hs_tef_ctrl_b.pwm4_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm4_pin_en = 1;
		break;

	case 5:
		syson->hs_tef_ctrl_b.pwm5_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm5_pin_en = 1;
		break;

	case 6:
		syson->hs_tef_ctrl_b.pwm6_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm6_pin_en = 1;
		break;

	case 7:
		syson->hs_tef_ctrl_b.pwm7_pin_sel = pin_sel;
		syson->hs_tef_ctrl_b.pwm7_pin_en = 1;
		break;

	default:
		break;
	}

#endif

	return HAL_OK;
}

/**
 *  \brief To read a PWM devive enable status
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns 1: Enable.
 *  \returns 0: Disable.
 */
SECTION_PWM_TEXT
BOOLEAN hal_rtl_pwm_enable_sts(hal_pwm_adapter_t *ppwm_adp)
{
	if (ppwm_comm_adapter == NULL) {
		//dbg_printf("e1\r\n");
		return 0;
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	//PWM_COMM_OBJ->PWM_COMM_ENABLE_STATUS & (1 << ppwm_adp->pwm_id);

	if (PWM_COMM_OBJ->PWM_COMM_ENABLE_STATUS & (1 << ppwm_adp->pwm_id)) {
		return 1;
	} else {
		return 0;
	}

}

/**
 *  \brief To enable multiple PWM devive simultaneously. If multiple PWM need to
 *         keep the phase offset, then they should be enabled simultaneously.
 *
 *  \param[in] en_ctrl The bit map for the PWM enable control. Bit [0 .. 7] map to PWM 0 .. 7.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_enable(u32 en_ctrl)
{
	hal_pwm_adapter_t *ppwm_adp;
	u32 i;
	u32 j = 0;

	if (ppwm_comm_adapter == NULL) {
		return;
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	PWM_COMM_OBJ->PWM_COMM_ENABLE_CTRL &= (~PWM_COMM_MASK_PWM_EN);
	PWM_COMM_OBJ->PWM_COMM_ENABLE_CTRL |= (en_ctrl & 0xfff);

	for (i = 0; i < MaxPwmNum; i++) {
		if ((en_ctrl & (1 << i)) && (ppwm_comm_adapter->pwm_adapter[i] != NULL)) {
			ppwm_adp = ppwm_comm_adapter->pwm_adapter[i];

			PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

			if ((((PWM_OBJ->PWM_CTRL) & PWM_BIT_PAUSE)) == 0) {
				while ((((PWM_OBJ->PWM_CTRL) & PWM_BIT_RUN_STS)) == 0) {
					hal_rtl_misc_delay_us(1);
					j++;
					if (j > 1000000) {
						DBG_PWM_ERR("hal_pwm_comm_enable: Wait PWM%u run timeout\r\n", i);
						break;
					}
				}
			}
		}
	}


}

/**
 *  \brief To disable multiple PWM devive simultaneously.
 *
 *  \param[in] dis_ctrl The bit map for the PWM disable control. Bit [0 .. 7] map to PWM 0 .. 7.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_comm_disable(u32 dis_ctrl)
{
	u32 i = 0;

	if (ppwm_comm_adapter == NULL) {
		return;
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);
	PWM_COMM_OBJ->PWM_COMM_DISABLE_CTRL &= (~PWM_COMM_MASK_PWM_DIS);
	PWM_COMM_OBJ->PWM_COMM_DISABLE_CTRL |= (dis_ctrl & 0xff);

	// wait PWM stopped
	while ((PWM_COMM_OBJ->PWM_COMM_ENABLE_STATUS & dis_ctrl) != 0) {
		hal_rtl_misc_delay_us(1);
		i++;
		if (i > 1000000) {
			DBG_PWM_ERR("hal_pwm_comm_disable: Wait PWM stop timeout, en_status=0x%x\r\n",
						PWM_COMM_OBJ->PWM_COMM_ENABLE_STATUS);
			break;
		}
	}
}


/**
 *  \brief To enable a PWM devive.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_enable(hal_pwm_adapter_t *ppwm_adp)
{
	u32 i = 0;

	if (ppwm_comm_adapter == NULL) {
		return;
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);
	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

	PWM_OBJ->PWM_CTRL |= PWM_BIT_CTRL_SET;


	PWM_COMM_OBJ->PWM_COMM_ENABLE_CTRL |= (1 << ppwm_adp->pwm_id);

	// wait PWM start
	if (((PWM_OBJ->PWM_CTRL & PWM_BIT_PAUSE)) == 0) {
		while (((PWM_OBJ->PWM_CTRL & PWM_BIT_RUN_STS)) == 0) {

			hal_rtl_misc_delay_us(1);
			i++;
			if (i > 1000000) {
				DBG_PWM_ERR("hal_pwm_enable: Wait PWM%u start timeout\r\n",
							ppwm_adp->pwm_id);
				break;
			}
		}
	}

}

/**
 *  \brief To disable a PWM devive.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_disable(hal_pwm_adapter_t *ppwm_adp)
{
	u32 i = 0;

	if (ppwm_comm_adapter == NULL) {
		return;
	}

	// pause the PWM first
	hal_rtl_pwm_pause(ppwm_adp, 1);
	// wait the PWM is paused
	while (hal_rtl_pwm_get_run_sts(ppwm_adp) == 1) {
		hal_rtl_misc_delay_us(1);
		i++;
		if (i > ppwm_adp->period_us) {
			break;
		}
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	PWM_COMM_OBJ->PWM_COMM_DISABLE_CTRL |= 1 << ppwm_adp->pwm_id;


	// un-pause the PWM first
	hal_rtl_pwm_pause(ppwm_adp, 0);

}

/**
 *  @brief To disable and de-initial a PWM devices adapter.
 *
 *  @param[in] ppwm_adp The PWM devices adapter.
 *
 *  @returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_deinit(hal_pwm_adapter_t *ppwm_adp)
{
	u8 pwm_id;
	hal_pwm_adapter_t *ptmp_pwm_adp;
#if 0
	SYSON_Type *syson = SYSON;
#endif

	hal_rtl_pwm_disable(ppwm_adp);
	if ((ppwm_adp->pwm_clk_sel != PwmClkSrc_None) && (ppwm_adp->pwm_clk_sel != PwmClkSrc_SClk)) {
		// a G-timer has been assign to this PWM already, we should free this timer and reallocate a new timer
		for (pwm_id = 0; pwm_id < MaxPwmNum; pwm_id++) {
			ptmp_pwm_adp = ppwm_comm_adapter->pwm_adapter[pwm_id];
			if ((ptmp_pwm_adp != NULL) &&
				(ptmp_pwm_adp != ppwm_adp)) {
				if (ptmp_pwm_adp->pwm_clk_sel == ppwm_adp->pwm_clk_sel) {
					// other PWM still using the same G-Timer, cannot disable this timer
					break;  // break for loop
				}
			}
		}

		if (pwm_id == MaxPwmNum) {
			// no other PWM use the same G-timer, so we can disable it
			if ((ppwm_adp->pwm_clk_sel) < PwmClkSrc_SClk) {
				hal_rtl_timer_event_deinit((ppwm_adp->pwm_clk_sel));
			}
			//DBG_PWM_ERR ("gtimer_to_disable = (%d)\r\n", ppwm_adp->pwm_clk_sel);
			ppwm_adp->pwm_clk_sel = PwmClkSrc_None;
		}
	}

	//hal_pwm_disable_rtl8195bhp(ppwm_adp);

	pwm_id = ppwm_adp->pwm_id;
	ppwm_comm_adapter->pwm_adapter[pwm_id] = NULL;


#if 0
	/* PWM disable pin mux */
	syson->hs_tef_ctrl &= ~(1 << (8 + pwm_id));
#endif
}


/**
 *  \brief To set the PWM tick source selection.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] clk_sel The PWM tick source selection
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_set_clk_sel(hal_pwm_adapter_t *ppwm_adp, pwm_clk_sel_t clk_sel)
{
	BOOLEAN enabled;

	enabled = hal_rtl_pwm_enable_sts(ppwm_adp);

	// The PWM should be disabled before change the Clk source selection
	if (enabled) {
		hal_rtl_pwm_disable(ppwm_adp);
	}

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	u32 CLK_SEL;
	CLK_SEL = 0;
	CLK_SEL = (clk_sel << PWM_SHIFT_CLK_SEL);
	PWM_OBJ->PWM_CTRL &= (~PWM_MASK_CLK_SEL);
	PWM_OBJ->PWM_CTRL |= CLK_SEL;
	ppwm_adp->pwm_clk_sel = clk_sel;

	if (enabled) {
		hal_rtl_pwm_enable(ppwm_adp);
	}

}

/**
 *  \brief To wait the PWM HW ready to set new PWM period/duty/offset.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_wait_ctrl_ready(hal_pwm_adapter_t *ppwm_adp)
{
	u32 i;

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

	for (i = 0; i < 1000000; i++) {
		if ((PWM_OBJ->PWM_CTRL & PWM_BIT_CTRL_SET) >> PWM_SHIFT_CTRL_SET == 0) {
			break;  // break the for loop
		} else {
			hal_rtl_misc_delay_us(1);
		}
	}
}

/**
 *  \brief To set the tick time (resolution) of a PWM device.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] tick_p5us The PWM tick time, unit is 500ns. It should be a even number.
 *
 *  \returns HAL_OK: Setting succeed.
 *  \returns HAL_BUSY: Busy.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_tick_time(hal_pwm_adapter_t *ppwm_adp, u32 tick_p5us)
{
	pwm_id_t pwm_id;
	hal_pwm_adapter_t *ptmp_pwm_adp;
	pwm_clk_sel_t pwm_clk_sel = PwmClkSrc_None;
	pwm_clk_sel_t gtimer_to_disable = PwmClkSrc_None;


	if ((ppwm_adp->pwm_clk_sel != PwmClkSrc_None) && (ppwm_adp->tick_p5us == tick_p5us)) {
		// same as current setting, just skip it
		return HAL_OK;
	}

	if ((ppwm_adp->pwm_clk_sel != PwmClkSrc_None) && (ppwm_adp->pwm_clk_sel != PwmClkSrc_SClk)) {
		// a G-timer has been assign to this PWM already, we should free this timer and reallocate a new timer
		for (pwm_id = 0; pwm_id < MaxPwmNum; pwm_id++) {
			ptmp_pwm_adp = ppwm_comm_adapter->pwm_adapter[pwm_id];
			if ((ptmp_pwm_adp != NULL) && (ptmp_pwm_adp != ppwm_adp)) {
				if (ptmp_pwm_adp->pwm_clk_sel == ppwm_adp->pwm_clk_sel) {
					//dbg_printf("s2.5\r\n");
					// other PWM still using the same G-Timer, cannot disable this timer
					break;  // break for loop
				}
			}
		}
		if (pwm_id == MaxPwmNum) {
			// no other PWM use the same G-timer, so we can disable it
//            hal_timer_event_deinit_rtl8195bhp (ppwm_adp->pwm_clk_sel);
#if Pro2
			gtimer_to_disable = ppwm_adp->pwm_clk_sel;
#else
			gtimer_to_disable = ppwm_adp->pwm_clk_sel + 8;
#endif
			ppwm_adp->pwm_clk_sel = PwmClkSrc_None;
		}
	}

	if (tick_p5us >= 1) {
		// tick duration > 0.5us, so use a GTimer as the tick source
		// search other PWM's tick source, to check is it possable to share the same tick source
		tick_p5us = (tick_p5us + 1) & 0xFFFFFFFE;   // make it as an even number
		for (pwm_id = 0; pwm_id < MaxPwmNum; pwm_id++) {
			ptmp_pwm_adp = ppwm_comm_adapter->pwm_adapter[pwm_id];
			if ((ptmp_pwm_adp != NULL) && (ptmp_pwm_adp != ppwm_adp) && (ptmp_pwm_adp->pwm_clk_sel != PwmClkSrc_None) && (ptmp_pwm_adp->tick_p5us != 0)) {
				if ((ptmp_pwm_adp->tick_p5us <= tick_p5us) && ((tick_p5us % ptmp_pwm_adp->tick_p5us) == 0)) {
					if ((((ppwm_adp->period_us << 1) / ptmp_pwm_adp->tick_p5us) - 1) <= 4095) { // prevent period counter over 12-bits
						// we can share this tick source
						ppwm_adp->tick_p5us = ptmp_pwm_adp->tick_p5us;
						pwm_clk_sel = ptmp_pwm_adp->pwm_clk_sel;
						break;  // break for loop
					}
				}
			}
		}
		if (pwm_id == MaxPwmNum) {
			// cannot share tick source with other PWM, allocate a new G-Timer
			hal_timer_adapter_t timer_adp;
			u32 tick_us;

			tick_us = (tick_p5us >> 1);
			pwm_clk_sel = hal_rtl_timer_event_init(&timer_adp, &tick_us, ppwm_comm_adapter->timer_list);

			if (pwm_clk_sel <= PwmClkSrc_Tm7) {
				tick_p5us = tick_us << 1;
				ppwm_adp->tick_p5us = tick_p5us;
				//dbg_printf("PwmClkSrc_Tm7\r\n");
			} else {
				// GTimer allocation failed, use SCLK
				ppwm_adp->tick_p5us = 1;
				pwm_clk_sel = PwmClkSrc_SClk;
			}
		}
	} else {
		// it should use the SClk as the tick source
		ppwm_adp->tick_p5us = 1;
		pwm_clk_sel = PwmClkSrc_SClk;
		pwm_clk_sel = pwm_clk_sel - 8;
	}

	if (pwm_clk_sel != PwmClkSrc_None) {
		hal_rtl_pwm_set_clk_sel(ppwm_adp, pwm_clk_sel);
	} else {
		return HAL_BUSY;
	}

	if ((gtimer_to_disable != PwmClkSrc_None) && (gtimer_to_disable < PwmClkSrc_SClk)) {
		hal_rtl_timer_event_deinit(gtimer_to_disable);
	}
	//dbg_printf("4\r\n");
	return HAL_OK;
}

/**
 *  \brief To set the duty ratio of the PWM
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] period_us The PWM cycle period, unit is us
 *  \param[in] duty_us The PWM on duty duration, unit is us
 *  \param[in] start_offset_us The on duty start timing offset from the start of the PWM periodof, unit is us
 *
 *  \returns HAL_OK: Setting succeed.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_duty(hal_pwm_adapter_t *ppwm_adp, u32 period_us,
								  u32 duty_us, u32 start_offset_us)
{
	u32 ticks_p5us; // how many 0.5us of 1 tick
	u32 period_test; // Pre-Calculated Period value
	u32 period_pre; //  save default period_us
	hal_status_t ret;
	u8 pwm_enabled;
	//pwm_ctrl_t pwm_ctrl;

	if ((period_us != ppwm_adp->period_us) || (ppwm_adp->pwm_clk_sel == PwmClkSrc_None)) {
		// Period changed or Tick source not set yet
		// we devide a PWM cycle as 4000 ticks, since period counter only has 12-bits
		ticks_p5us = period_us * 2 / 4000;
		if (ticks_p5us == 0) {
			ticks_p5us = 1;
		}

		if (ppwm_adp->duty_res_us != 0) {
			ticks_p5us = ppwm_adp->duty_res_us << 1; // unit is 0.5 us
		}

		while (1) {
			if (period_us == 0) {
				break;
			} else {
				period_test = (((period_us << 1) / ticks_p5us) - 1);
			}
			if (period_test > 4095) { // prevent period counter over 12-bits
				ticks_p5us = ticks_p5us + 1;
			} else {
				break;
			}
		}
		period_pre = ppwm_adp->period_us; // save default period_us
		ppwm_adp->period_us = period_us;

		if ((ticks_p5us != ppwm_adp->tick_p5us) || (ppwm_adp->pwm_clk_sel == PwmClkSrc_None)) {
			ret = hal_rtl_pwm_set_tick_time(ppwm_adp, ticks_p5us);
			if (ret != HAL_OK) {
				ppwm_adp->period_us = period_pre; // return default value
				DBG_PWM_ERR("hal_rtl_pwm_set_duty: set tick time = %u err(%d)\r\n", ticks_p5us, ret);
				return ret;
			}
		}
	}

	ticks_p5us = ppwm_adp->tick_p5us;
	//pwm_ctrl.w = ppwm_adp->base_addr->ctrl;
	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	u32 PWM_CTRL;
	PWM_CTRL = PWM_OBJ->PWM_CTRL;
	pwm_enabled = hal_rtl_pwm_enable_sts(ppwm_adp);
	//dbg_printf("enable_sts %x\r\n", pwm_enabled);
	if (pwm_enabled) {
		hal_rtl_pwm_wait_ctrl_ready(ppwm_adp);
	}

	hal_rtl_pwm_set_period(ppwm_adp, (((period_us << 1) / ticks_p5us) - 1));
	hal_rtl_pwm_set_onduty_start(ppwm_adp, ((start_offset_us << 1) / ticks_p5us));
//    hal_pwm_set_duty_size_rtl8195bhp (ppwm_adp, ((duty_us << 1) / ticks_p5us));
	//pwm_ctrl.b.duty = (duty_us << 1) / ticks_p5us;
	PWM_CTRL &= (~PWM_MASK_DUTY);
	PWM_CTRL |= (duty_us << 1) / ticks_p5us;
	if (pwm_enabled) {
		//pwm_ctrl.b.ctrl_set = 1;
		PWM_CTRL |= PWM_BIT_CTRL_SET;
	}
	//ppwm_adp->base_addr->ctrl = pwm_ctrl.w;
	PWM_OBJ->PWM_CTRL = PWM_CTRL;
	ppwm_adp->period_us = period_us;
	ppwm_adp->duty_us = duty_us;



	return HAL_OK;
}

/**
 *  \brief To read the time period of current duty of the PWM
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns u32: The PWM on duty duration, unit is us
 */
SECTION_PWM_TEXT
u32 hal_rtl_pwm_read_duty(hal_pwm_adapter_t *ppwm_adp)
{
	//pwm_comm_indread_idx_t pwm_indrd_idx;
	u32 duty_ticks;
	u32 duty_us;
	u32 i = 0;

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	u32 PWM_INDRD_IDX;
	PWM_INDRD_IDX = 0;

	//PWM_INDRD_IDX &= (~PWM_COMM_MASK_PWM_SEL);

	PWM_INDRD_IDX |= (PWM_COMM_MASK_PWM_SEL & (ppwm_adp->pwm_id));

	PWM_INDRD_IDX |= PWM_COMM_BIT_POOL;
	PWM_INDRD_IDX |= PWM_COMM_BIT_SYNC_MODE;

	PWM_COMM_OBJ->PWM_COMM_INDREAD_IDX = PWM_INDRD_IDX;

//    pwm_indrd_idx.w = 0;
//    pwm_indrd_idx.b.pwm_sel = ppwm_adp->pwm_id;
//    pwm_indrd_idx.b.pool = 1; // enable in-direct read
//    pwm_indrd_idx.b.sync_mode = 1; // in-direct read sync mode

	//ppwm_comm_adapter->base_addr->indread_idx = pwm_indrd_idx.w;
	while (((PWM_COMM_OBJ->PWM_COMM_INDREAD_IDX & PWM_COMM_BIT_POOL) >> PWM_COMM_SHIFT_POOL) == 1) {      // wait HW ready
		hal_rtl_misc_delay_us(1);
		i++;
		if (i > 1000000) {
			DBG_PWM_ERR("hal_pwm_read_duty: Wait indirect-read ready timeout\r\n");
			break;
		}
	}
	duty_ticks = PWM_COMM_OBJ->PWM_COMM_INDREAD_DUTY & PWM_COMM_MASK_PWM_DUTY;
	//duty_ticks = ppwm_comm_adapter->base_addr->indread_duty_b.pwm_duty;
	duty_us = (duty_ticks * ppwm_adp->tick_p5us) >> 1;

	// read the PWM duty tick count by in-direct reading

	return (duty_us);
}

/**
 *  \brief To change the duty ratio of the PWM only and keep other setting.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] duty_us The PWM on duty duration, unit is us
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_change_duty(hal_pwm_adapter_t *ppwm_adp, u32 duty_us)
{
	u32 ticks_p5us; // how many 0.5us of 1 tick
	u8 pwm_enabled;
	//pwm_ctrl_t pwm_ctrl;

	ticks_p5us = ppwm_adp->tick_p5us;
	pwm_enabled = hal_rtl_pwm_enable_sts(ppwm_adp);
	if (pwm_enabled) {
		hal_rtl_pwm_wait_ctrl_ready(ppwm_adp);
	}
	u32 PWM_CTRL;
	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	PWM_CTRL = PWM_OBJ->PWM_CTRL;
	PWM_CTRL &= (~PWM_MASK_DUTY);
	PWM_CTRL |= (duty_us << 1) / ticks_p5us;

	if (pwm_enabled) {
		PWM_CTRL |= PWM_BIT_CTRL_SET;
	}
	PWM_OBJ->PWM_CTRL = PWM_CTRL;
	ppwm_adp->duty_us = duty_us;

}

/**
 *  \brief To set the PWM on duty boundary (up limit / down limit) of the duty
 *         auto-adjustment.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_us The up limit of the duty time, in us.
 *  \param[in] min_duty_us The down limit of the duty time, in us.
 *
 *  \returns HAL_OK: Setting succeed.
 *  \returns HAL_ERR_PARA: Error with invaild parameters.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_duty_limit(hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 min_duty_us)
{
	u32 tick_p5us;  // how many 0.5us of 1 tick

	if (min_duty_us > max_duty_us) {
		DBG_PWM_ERR("hal_pwm_set_duty_limit: dn-limit(%u) > up-limit(%u)\r\n", min_duty_us, max_duty_us);
		return HAL_ERR_PARA;
	}

	if ((ppwm_adp->duty_us < min_duty_us) || (ppwm_adp->duty_us > max_duty_us)) {
		DBG_PWM_ERR("hal_pwm_set_duty_limit: current duty(%u) is not in this range\r\n", ppwm_adp->duty_us);
		return HAL_ERR_PARA;
	}

	tick_p5us = ppwm_adp->tick_p5us;

	// Set the up limit, in ticks
	hal_rtl_pwm_set_max_duty(ppwm_adp, ((max_duty_us << 1) / tick_p5us));
	// Set the down limit, in ticks
	hal_rtl_pwm_set_min_duty(ppwm_adp, (min_duty_us << 1) / tick_p5us);
	return HAL_OK;
}

/**
 *  \brief To setup the PWM duty auto-adjustment registers.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] pauto_duty The duty auto-adjustment configuration. Includes maximum / minum duty size, duty
 *                    increasing / decreasing step size, adjustment period and the callback function for
 *                    the adjustment done indication.
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_set_auto_duty_adj(hal_pwm_adapter_t *ppwm_adp, hal_pwm_auto_duty_adj_t *pauto_duty)
{
	u32 tick_p5us; // how many 0.5us of 1 tick
	u32 cycle_cnt;
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;
	//pwm_auto_adj_limit_t pwm_adj_lim;

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	u32 PWM_ADJ_LIM, PWM_ADJ_CTRL;

	tick_p5us = ppwm_adp->tick_p5us;

	PWM_ADJ_LIM = 0;
	PWM_ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;
	//pwm_adj_lim.w = 0;
	//pwm_adj_ctrl.w = ppwm_adp->base_addr->auto_adj_ctrl;
	PWM_ADJ_LIM = (PWM_ADJ_LIM | PWM_MASK_DUTY_ADJ_UP_LIM) & (((pauto_duty->max_duty_us << 1) / tick_p5us) << PWM_SHIFT_DUTY_ADJ_UP_LIM);
	PWM_ADJ_LIM = (PWM_ADJ_LIM | PWM_MASK_DUTY_ADJ_DN_LIM) & (((pauto_duty->min_duty_us << 1) / tick_p5us) << PWM_SHIFT_DUTY_ADJ_DN_LIM);

//    pwm_adj_lim.b.duty_adj_up_lim = (pauto_duty->max_duty_us << 1) / tick_p5us;
//    pwm_adj_lim.b.duty_adj_dn_lim = (pauto_duty->min_duty_us << 1) / tick_p5us;

	//ppwm_adp->base_addr->auto_adj_limit = pwm_adj_lim.w;
	PWM_OBJ->PWM_AUTO_ADJ_LIMIT = PWM_ADJ_LIM;


	if (pauto_duty->step_period_cnt > 1) {
		cycle_cnt = pauto_duty->step_period_cnt - 1;    // HW cycle count = cycle count - 1;
	} else {
		cycle_cnt = 0;
	}

	hal_rtl_pwm_set_duty_adj_cycle(ppwm_adp, cycle_cnt);

	PWM_ADJ_CTRL |= (PWM_MASK_DUTY_DEC_STEP & (((pauto_duty->duty_dec_step_us << 1) / tick_p5us) << PWM_SHIFT_DUTY_DEC_STEP));
	PWM_ADJ_CTRL |= (PWM_MASK_DUTY_INC_STEP & (((pauto_duty->duty_inc_step_us << 1) / tick_p5us) << PWM_SHIFT_DUTY_INC_STEP));
	if (pauto_duty->init_dir == 1) {
		PWM_ADJ_CTRL = PWM_ADJ_CTRL | PWM_BIT_ADJ_DIR;
	} else {
		PWM_ADJ_CTRL = PWM_ADJ_CTRL & (~PWM_BIT_ADJ_DIR);
	}

//    pwm_adj_ctrl.b.duty_dec_step = (pauto_duty->duty_dec_step_us << 1) / tick_p5us;
//    pwm_adj_ctrl.b.duty_inc_step = (pauto_duty->duty_inc_step_us << 1) / tick_p5us;
//    pwm_adj_ctrl.b.adj_dir = pauto_duty->init_dir;

	if (pauto_duty->loop_mode) {
		PWM_ADJ_CTRL = PWM_ADJ_CTRL | PWM_BIT_ADJ_LOOP_EN;
	} else {
		PWM_ADJ_CTRL = PWM_ADJ_CTRL & (~PWM_BIT_ADJ_LOOP_EN);
	}

//    if (pauto_duty->loop_mode) {
//        pwm_adj_ctrl.b.adj_loop_en = 1;
//    } else {
//        pwm_adj_ctrl.b.adj_loop_en = 0;
//    }

	PWM_OBJ->PWM_AUTO_ADJ_CTRL = PWM_ADJ_CTRL;

	// write the duty auto-adjustment control register
	//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;


}

/**
 *  \brief To enable or disable the PWM duty auto-adjustment HW.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] enable The duty auto-adjustment enable control (0: disable, 1: enable)
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_auto_duty_en(hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable)
{
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;
	hal_pwm_auto_duty_adj_t *duty_adj;

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	duty_adj = &(ppwm_adp->duty_adj);
	u32 PWM_ADJ_CTRL;
	PWM_ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;

	if (enable) {
		PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
		PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
//        pwm_adj_ctrl.b.duty_up_lim_ie = 0;
//        pwm_adj_ctrl.b.duty_dn_lim_ie = 0;


		if (duty_adj->loop_mode == PwmDutyLoopOff) {
			if (duty_adj->init_dir) {
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_UP_LIM_IE);
				//pwm_adj_ctrl.b.duty_up_lim_ie = 1;
			} else {
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_DN_LIM_IE);
				//pwm_adj_ctrl.b.duty_dn_lim_ie = 1;
			}
		} else if (duty_adj->loop_mode == PwmDutyLoopForever) {
			if (ppwm_adp->adj_int_en & PwmAdjIntUpLim) {
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_UP_LIM_IE);
				//pwm_adj_ctrl.b.duty_up_lim_ie = 1;
			}

			if (ppwm_adp->adj_int_en & PwmAdjIntDnLim) {
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_DN_LIM_IE);
				//pwm_adj_ctrl.b.duty_dn_lim_ie = 1;
			}
		} else if (duty_adj->loop_mode == PwmDutyLoopCnt) {
			if (duty_adj->init_dir) {
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_UP_LIM_IE);
				PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
//                pwm_adj_ctrl.b.duty_up_lim_ie = 1;
//                pwm_adj_ctrl.b.duty_dn_lim_ie = 0;
			} else {
				PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
				PWM_ADJ_CTRL |= (PWM_BIT_DUTY_DN_LIM_IE);
//                pwm_adj_ctrl.b.duty_up_lim_ie = 0;
//                pwm_adj_ctrl.b.duty_dn_lim_ie = 1;
			}
		}
		PWM_ADJ_CTRL |= PWM_BIT_ADJ_EN;
		//pwm_adj_ctrl.b.adj_en = 1;
	} else {
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_EN);
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);
//        pwm_adj_ctrl.b.adj_en = 0;
//        pwm_adj_ctrl.b.adj_loop_en = 0;
	}
	PWM_OBJ->PWM_AUTO_ADJ_CTRL = PWM_ADJ_CTRL;
	//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;


}

/**
 *  \brief To configure the PWM duty auto-adjustment for duty duration increasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_us The up limit of the duty duration, in us.
 *  \param[in] step_sz_us The step size of each duty duration increasing, in us.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \returns HAL_OK: Setting succeed.
 *  \returns HAL_ERR_PARA: Error with invaild parameters.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_auto_duty_inc(hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us,
		u32 step_sz_us, u32 step_period_cnt)
{
	hal_pwm_auto_duty_adj_t *pauto_duty;

	if ((max_duty_us > ppwm_adp->period_us)) {
		DBG_PWM_ERR("hal_pwm_auto_duty_inc: max_duty_us(%u) over period(%u)\r\n",
					max_duty_us, ppwm_adp->period_us);
		return HAL_ERR_PARA;
	}

	pauto_duty = &(ppwm_adp->duty_adj);
	pauto_duty->max_duty_us = max_duty_us;
	pauto_duty->duty_inc_step_us = step_sz_us;
	pauto_duty->step_period_cnt = step_period_cnt;
	pauto_duty->loop_mode = PwmDutyLoopOff;
	pauto_duty->init_dir = PwmDutyAdj_Increase;
	hal_rtl_pwm_set_auto_duty_adj(ppwm_adp, pauto_duty);

	return HAL_OK;
}

/**
 *  \brief To configure the PWM duty auto-adjustment for duty duration increasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_us The up limit of the duty duration, in us.
 *  \param[in] step_sz_us The step size of each duty duration increasing, in us.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \returns HAL_OK: Setting succeed.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_auto_duty_dec(hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us,
		u32 step_sz_us, u32 step_period_cnt)
{
	hal_pwm_auto_duty_adj_t *pauto_duty;

	pauto_duty = &(ppwm_adp->duty_adj);
//    auto_duty.max_duty_us = ppwm_adp->period_us;
	pauto_duty->min_duty_us = min_duty_us;
	pauto_duty->duty_dec_step_us = step_sz_us;
	pauto_duty->step_period_cnt = step_period_cnt;
	pauto_duty->loop_mode = PwmDutyLoopOff;
	pauto_duty->init_dir = PwmDutyAdj_Decrease;
	hal_rtl_pwm_set_auto_duty_adj(ppwm_adp, pauto_duty);

	return HAL_OK;
}

/**
 *  \brief To configure the PWM duty auto-adjustment loop mode.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] ini_dir The initial direction for the duty-adjustment loop. 1 -> duty increasing, 0 -> duty decreasing.
 *  \param[in] loop_cnt The number of duty-adjustment loop to run. 1 loop means from min duty to max duty
 *                  or from max duty to min duty.
 *
 *  \returns HAL_OK: Setting succeed.
 *  \returns HAL_ERR_PARA: Error with invaild parameters.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_auto_duty_loop(hal_pwm_adapter_t *ppwm_adp, u8 ini_dir, u32 loop_cnt)
{
	hal_pwm_auto_duty_adj_t *pauto_duty;

	pauto_duty = &(ppwm_adp->duty_adj);

	if ((pauto_duty->min_duty_us >= pauto_duty->max_duty_us) ||
		(pauto_duty->max_duty_us > ppwm_adp->period_us)) {
		DBG_PWM_ERR("hal_pwm_auto_duty_loop: invalid cfg:  max_duty_us(%u) min_duty_us(%u) period(%u)\r\n",
					pauto_duty->max_duty_us, pauto_duty->min_duty_us, ppwm_adp->period_us);
		return HAL_ERR_PARA;
	}

	if (ini_dir) {
		pauto_duty->init_dir = PwmDutyAdj_Increase;
	} else {
		pauto_duty->init_dir = PwmDutyAdj_Decrease;
	}
	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

	if (ini_dir == 1) {
		PWM_OBJ->PWM_AUTO_ADJ_CTRL |= PWM_BIT_ADJ_DIR;
	} else {
		PWM_OBJ->PWM_AUTO_ADJ_CTRL &= (~PWM_BIT_ADJ_DIR);
	}

	//ppwm_adp->base_addr->auto_adj_ctrl_b.adj_dir = ini_dir;

	if (loop_cnt > 0) {
		ppwm_adp->adj_loop_count = loop_cnt - 1;
		ppwm_adp->duty_adj.loop_mode = PwmDutyLoopCnt;
		if (ppwm_adp->adj_loop_count > 0) {
			PWM_OBJ->PWM_AUTO_ADJ_CTRL |= PWM_BIT_ADJ_LOOP_EN;
			//ppwm_adp->base_addr->auto_adj_ctrl_b.adj_loop_en = 1;
		}
	} else {
		ppwm_adp->duty_adj.loop_mode = PwmDutyLoopForever;
		PWM_OBJ->PWM_AUTO_ADJ_CTRL |= PWM_BIT_ADJ_LOOP_EN;
		//ppwm_adp->base_addr->auto_adj_ctrl_b.adj_loop_en = 1;
	}

	return HAL_OK;


}

/**
 *  \brief To enable the PWM period end interrupt.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] callback The callback function. It will be called when the interrupt is accurred.
 *  \param[in] arg The argument of the callback function.
 *  \param[in] int_en To enable(1) or disable(0) the interrupt. For interrupt disable, the arguments
 *                'callback' & 'arg' are ignored.
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_set_period_int(hal_pwm_adapter_t *ppwm_adp, pwm_period_callback_t callback, void *arg, u8 int_en)
{

	if (int_en) {
		if (callback != NULL) {
			ppwm_adp->period_callback = callback;
			ppwm_adp->pe_cb_para = arg;

			hal_rtl_pwm_set_period_ie(ppwm_adp, 1);
		}
	} else {
		hal_rtl_pwm_set_period_ie(ppwm_adp, 0);
	}
}

/**
 *  \brief To setup the PWM duty auto adjustment interrupt.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] callback The callback function. It will be called when the interrupt is accurred.
 *  \param[in] arg The argument of the callback function.
 *  \param[in] int_en The bit map to enable/disable the interrupt. Bit 0 control the interrupt of
 *                duty duration reachs the down limit. Bit 1 control the interrupt of duty duration
 *                reachs the up limit.
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_set_autoadj_int(hal_pwm_adapter_t *ppwm_adp,
								 pwm_lim_callback_t callback, void *arg, u8 int_en)
{
	if (callback != NULL) {
		ppwm_adp->bound_callback = callback;
		ppwm_adp->bound_cb_para = arg;
	}

	ppwm_adp->adj_int_en = int_en;
}

/**
 *  \brief To setup the PWM duty auto adjustment interrupt callback for loop mode.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] callback The callback function. It will be called when the duty adjustment loop count down to 0.
 *  \param[in] arg The argument of the callback function.
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_set_autoadj_loop_int(hal_pwm_adapter_t *ppwm_adp,
									  pwm_lo_callback_t callback, void *arg)
{
	ppwm_adp->loopout_callback = callback;
	ppwm_adp->lo_cb_para = arg;
}

/**
 *  \brief To start the PWM duty auto-adjustment for duty duration increasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_us The up limit of the duty duration, in us.
 *  \param[in] step_sz_us The step size of each duty duration increasing, in us.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \return     HAL_OK:  Setting succeed.
 *  \return     HAL_BUSY:  BUSY.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_auto_duty_inc(hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us,
									   u32 step_sz_us, u32 step_period_cnt)
{
	hal_status_t result;

	if ((hal_rtl_pwm_enable_sts(ppwm_adp)) && ((ppwm_adp->base_addr->PWM_AUTO_ADJ_CTRL & PWM_BIT_ADJ_EN) >> 31)) {
		DBG_PWM_WARN("PWM%u is busy\r\n", ppwm_adp->pwm_id);
		return HAL_BUSY;
	}

	result = hal_rtl_pwm_set_auto_duty_inc(ppwm_adp, max_duty_us, step_sz_us, step_period_cnt);
	if (result == HAL_OK) {
		hal_rtl_pwm_auto_duty_en(ppwm_adp, 1);
	}

	return result;
}

/**
 *  \brief To start the PWM duty auto-adjustment for duty duration increasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_us The up limit of the duty duration, in us.
 *  \param[in] step_sz_us The step size of each duty duration increasing, in us.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \return     HAL_OK:  Setting succeed.
 *  \return     HAL_BUSY:  BUSY.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_auto_duty_dec(hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us,
									   u32 step_sz_us, u32 step_period_cnt)
{
	hal_status_t result;

	if ((hal_rtl_pwm_enable_sts(ppwm_adp)) && ((ppwm_adp->base_addr->PWM_AUTO_ADJ_CTRL & PWM_BIT_ADJ_EN) >> 31)) {
		DBG_PWM_WARN("PWM%u is busy\r\n", ppwm_adp->pwm_id);
		return HAL_BUSY;
	}

	result = hal_rtl_pwm_set_auto_duty_dec(ppwm_adp, min_duty_us, step_sz_us, step_period_cnt);
	if (result == HAL_OK) {
		hal_rtl_pwm_auto_duty_en(ppwm_adp, 1);
	}

	return result;
}

/**
 *  \brief To start the PWM duty auto-adjustment loop mode.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] ini_duty_us The initial value for the loop-mode auto duty adjustment. If this value is 0xFFFFFFFF
 *                       it means use current duty as the initial duty.
 *  \param[in] ini_dir The initial direction for the duty-adjustment loop. 1 -> duty increasing, 0 -> duty decreasing.
 *  \param[in] loop_cnt The number of duty-adjustment loop to run. 1 loop means from min duty to max duty
 *                  or from max duty to min duty.
 *
 *  \return     HAL_OK:  Setting succeed.
 *  \return     HAL_BUSY:  BUSY.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_auto_duty_loop(hal_pwm_adapter_t *ppwm_adp, u32 ini_duty_us, u8 ini_dir, u32 loop_cnt)
{
	hal_status_t result;

	if ((hal_rtl_pwm_enable_sts(ppwm_adp)) && ((ppwm_adp->base_addr->PWM_AUTO_ADJ_CTRL & PWM_BIT_ADJ_EN) >> 31)) {
		DBG_PWM_WARN("PWM%u is busy\r\n", ppwm_adp->pwm_id);
		return HAL_BUSY;
	}

	result = hal_rtl_pwm_set_auto_duty_loop(ppwm_adp, ini_dir, loop_cnt);
	if (result == HAL_OK) {
		if ((ini_duty_us <= ppwm_adp->period_us) && (ini_duty_us != PWM_CURRENT_DUTY)) {
			hal_rtl_pwm_change_duty(ppwm_adp, ini_duty_us);
		}
		hal_rtl_pwm_auto_duty_en(ppwm_adp, 1);
	}

	return result;
}

/**
 *  \brief To stop the PWM duty auto-adjustment loop mode.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] stop_now Is stop the PWM auto duty loop immediately(1) or stop at the next duty limit(0).
 *
 *  \returns void
 */
SECTION_PWM_TEXT
void hal_rtl_pwm_stop_duty_loop(hal_pwm_adapter_t *ppwm_adp, u8 stop_now)
{
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;

	if ((ppwm_adp->duty_adj.loop_mode == PwmDutyLoopOff) ||
		((ppwm_adp->duty_adj.loop_mode == PwmDutyLoopCnt) && (ppwm_adp->adj_loop_count <= 1))) {
		return;
	}

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
	u32 PWM_ADJ_CTRL;
	PWM_ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;
	//pwm_adj_ctrl.w = ppwm_adp->base_addr->auto_adj_ctrl;

	if (stop_now) {
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_EN);
		//pwm_adj_ctrl.b.adj_loop_en = 0;
		//pwm_adj_ctrl.b.adj_en = 0;
		// Disable Interrupt
		PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_DN_LIM_IE);
		PWM_ADJ_CTRL &= (~PWM_BIT_DUTY_UP_LIM_IE);
		//pwm_adj_ctrl.b.duty_dn_lim_ie = 0;
		//pwm_adj_ctrl.b.duty_up_lim_ie = 0;
	} else {
		ppwm_adp->adj_loop_count = 1;
		ppwm_adp->duty_adj.loop_mode = PwmDutyLoopCnt;

//        pwm_adj_ctrl.b.adj_loop_en = 0;
		// enable interrupt and then stop auto duty adjustment in ISR
		PWM_ADJ_CTRL |= PWM_BIT_DUTY_DN_LIM_IE;
		PWM_ADJ_CTRL |= PWM_BIT_DUTY_UP_LIM_IE;

//        pwm_adj_ctrl.b.duty_dn_lim_ie = 1;
//        pwm_adj_ctrl.b.duty_up_lim_ie = 1;
	}
	PWM_OBJ->PWM_AUTO_ADJ_CTRL = PWM_ADJ_CTRL;
	//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;


}

/**
 *  \brief To set the duty(ns) ratio of the PWM.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] period_ns The PWM cycle period, unit is ns
 *  \param[in] duty_ns The PWM on duty duration, unit is ns
 *  \param[in] start_offset_ns The on duty start timing offset from the start of the PWM periodof, unit is ns
 *
 *  \returns HAL_OK: Setting succeed.
 *  \returns HAL_NOT_READY: Error with data not ready.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_set_duty_ns(hal_pwm_adapter_t *ppwm_adp, u32 period_ns,
									 u32 duty_ns, u32 start_offset_ns)
{

	u32 ticks_50ns; // how many 0.5us of 1 tick
	u8 pwm_enabled;
	//pwm_ctrl_t pwm_ctrl;
	ticks_50ns = 50;
	if (ppwm_comm_adapter == NULL) {
		return HAL_NOT_READY;
	}

	if (ptimer_group2->sclk_idx == 1) {
		if (period_ns > 204750) {
			DBG_PWM_ERR("hal_rtl_pwm_set_duty_ns: The period_ns can't be larger than 204750ns\r\n");
			return HAL_NOT_READY;
		} else if (period_ns < 100) {
			DBG_PWM_ERR("hal_rtl_pwm_set_duty_ns: The period_ns can't be less than 100ns\r\n");
			return HAL_NOT_READY;
		} else {
			hal_rtl_pwm_set_clk_sel(ppwm_adp, 8);  // Assign SCLK
			PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
			u32 PWM_CTRL;
			PWM_CTRL = PWM_OBJ->PWM_CTRL;
			//pwm_ctrl.w = ppwm_adp->base_addr->ctrl;
			pwm_enabled = hal_rtl_pwm_enable_sts(ppwm_adp);
			if (pwm_enabled) {
				hal_rtl_pwm_wait_ctrl_ready(ppwm_adp);
			}
			hal_rtl_pwm_set_period(ppwm_adp, (((period_ns) / ticks_50ns) - 1));
			hal_rtl_pwm_set_onduty_start(ppwm_adp, ((start_offset_ns) / ticks_50ns));
			PWM_CTRL &= (~PWM_MASK_DUTY);
			PWM_CTRL |= (duty_ns) / ticks_50ns;
			//pwm_ctrl.b.duty = (duty_ns) / ticks_50ns;
			//DBG_PWM_ERR ("Duty: %d\r\n",pwm_ctrl.b.duty);

			if (pwm_enabled) {
				PWM_CTRL &= (~PWM_BIT_CTRL_SET);
				//pwm_ctrl.b.ctrl_set = 1;
			}
			ppwm_adp->tick_p5us = ticks_50ns / 5000;
			PWM_OBJ->PWM_CTRL = PWM_CTRL;
			//ppwm_adp->base_addr->ctrl = pwm_ctrl.w;
			ppwm_adp->period_us = period_ns / 1000;
			ppwm_adp->duty_us = duty_ns / 1000;
		}
	} else {
		DBG_PWM_ERR("hal_rtl_pwm_set_duty_ns: The timer group1 should be set 20MHz\r\n");
		return HAL_NOT_READY;
	}

	return HAL_OK;


}


/**
 *  \brief To start the PWM duty auto-adjustment for duty duration increasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_ns The up limit of the duty duration, in ns.
 *  \param[in] step_sz_ns The step size of each duty duration increasing, in ns.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_BUSY:  BUSY.
 *  \returns     HAL_NOT_READY: Error with data not ready.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_auto_duty_ns_inc(hal_pwm_adapter_t *ppwm_adp, u32 max_duty_ns,
		u32 step_sz_ns, u32 step_period_cnt)
{
	hal_status_t result;
	u32 tick_ns; // how many 0.5us of 1 tick
	u32 cycle_cnt;
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;
	//pwm_auto_adj_limit_t pwm_adj_lim;
	hal_pwm_auto_duty_adj_t *pauto_duty;

	if (ptimer_group2->sclk_idx == 1) {
		if ((hal_rtl_pwm_enable_sts(ppwm_adp)) && ((ppwm_adp->base_addr->PWM_AUTO_ADJ_CTRL & PWM_BIT_ADJ_EN) >> 31)) {
			DBG_PWM_WARN("PWM%u is busy\r\n", ppwm_adp->pwm_id);
			return HAL_BUSY;
		}

		//result = hal_pwm_set_auto_duty_inc (ppwm_adp, max_duty_ns, step_sz_ns, step_period_cnt);
		pauto_duty = &(ppwm_adp->duty_adj);
		pauto_duty->max_duty_us = max_duty_ns / 1000;
		pauto_duty->duty_inc_step_us = step_sz_ns / 1000;
		pauto_duty->step_period_cnt = step_period_cnt;
		pauto_duty->loop_mode = PwmDutyLoopOff;
		pauto_duty->init_dir = PwmDutyAdj_Increase;
		tick_ns = 50;

		PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
		u32 PWM_ADJ_LIM, PWM_ADJ_CTRL;
		PWM_ADJ_LIM = 0;
		//pwm_adj_lim.w = 0;
		PWM_ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;
		//pwm_adj_ctrl.w = ppwm_adp->base_addr->auto_adj_ctrl;
		PWM_ADJ_LIM |= PWM_MASK_DUTY_ADJ_UP_LIM & ((max_duty_ns / tick_ns) << PWM_SHIFT_DUTY_ADJ_DN_LIM);
		PWM_ADJ_LIM &= (~PWM_MASK_DUTY_ADJ_DN_LIM);


//        pwm_adj_lim.b.duty_adj_up_lim = (max_duty_ns) / tick_ns;
//        pwm_adj_lim.b.duty_adj_dn_lim = 0;
		PWM_OBJ->PWM_AUTO_ADJ_LIMIT = PWM_ADJ_LIM;
		//ppwm_adp->base_addr->auto_adj_limit = pwm_adj_lim.w;


		if (step_period_cnt > 1) {
			cycle_cnt = step_period_cnt - 1;    // HW cycle count = cycle count - 1;
		} else {
			cycle_cnt = 0;
		}

		hal_rtl_pwm_set_duty_adj_cycle(ppwm_adp, cycle_cnt);

		PWM_ADJ_CTRL |= (PWM_MASK_DUTY_INC_STEP & ((step_sz_ns / tick_ns) << PWM_SHIFT_DUTY_INC_STEP));
		PWM_ADJ_CTRL &= (~PWM_MASK_DUTY_DEC_STEP);

//        pwm_adj_ctrl.b.duty_inc_step = (step_sz_ns) / tick_ns;
//        pwm_adj_ctrl.b.duty_dec_step = 0;
		PWM_ADJ_CTRL |= PWM_BIT_ADJ_DIR;
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);

//        pwm_adj_ctrl.b.adj_dir = PwmDutyAdj_Increase;
//        pwm_adj_ctrl.b.adj_loop_en = PwmDutyLoopOff;

		// write the duty auto-adjustment control register
		PWM_OBJ->PWM_AUTO_ADJ_CTRL = PWM_ADJ_CTRL;
		//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;
		result = HAL_OK;
		if (result == HAL_OK) {
			hal_rtl_pwm_auto_duty_en(ppwm_adp, 1);
		}
		return result;
	} else {
		DBG_PWM_ERR("hal_rtl_pwm_auto_duty_ns_inc: The timer group1 should be set 20MHz\r\n");
		return HAL_NOT_READY;
	}



}

/**
 *  \brief To start the PWM duty auto-adjustment for duty duration decreasing.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_ns The up limit of the duty duration, in ns.
 *  \param[in] step_sz_ns The step size of each duty duration increasing, in ns.
 *  \param[in] step_period_cnt The stay time of each duty duration increasing step, uint is PWM period.
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_BUSY:  BUSY.
 *  \returns     HAL_NOT_READY: Error with data not ready.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_auto_duty_ns_dec(hal_pwm_adapter_t *ppwm_adp, u32 min_duty_ns,
		u32 step_sz_ns, u32 step_period_cnt)
{
	hal_status_t result;
	u32 tick_ns; // how many 0.5us of 1 tick
	u32 cycle_cnt;
	//pwm_auto_adj_ctrl_t pwm_adj_ctrl;
	//pwm_auto_adj_limit_t pwm_adj_lim;
	hal_pwm_auto_duty_adj_t *pauto_duty;

	if (ptimer_group2->sclk_idx == 1) {
		if ((hal_rtl_pwm_enable_sts(ppwm_adp)) && ((ppwm_adp->base_addr->PWM_AUTO_ADJ_CTRL & PWM_BIT_ADJ_EN) >> 31)) {
			DBG_PWM_WARN("PWM%u is busy\r\n", ppwm_adp->pwm_id);
			return HAL_BUSY;
		}

		//result = hal_pwm_set_auto_duty_dec (ppwm_adp, min_duty_ns, step_sz_ns, step_period_cnt);

		pauto_duty = &(ppwm_adp->duty_adj);
		pauto_duty->min_duty_us = min_duty_ns / 1000;
		pauto_duty->duty_dec_step_us = step_sz_ns / 1000;
		pauto_duty->step_period_cnt = step_period_cnt;
		pauto_duty->loop_mode = PwmDutyLoopOff;
		pauto_duty->init_dir = PwmDutyAdj_Decrease;

		tick_ns = 50;

		PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);
		u32 PWM_ADJ_LIM, PWM_ADJ_CTRL;
		PWM_ADJ_LIM = 0;
		PWM_ADJ_CTRL = PWM_OBJ->PWM_AUTO_ADJ_CTRL;
		PWM_ADJ_LIM |= PWM_MASK_DUTY_ADJ_DN_LIM & ((min_duty_ns / tick_ns) << PWM_SHIFT_DUTY_ADJ_DN_LIM);
		PWM_ADJ_LIM &= (~PWM_MASK_DUTY_ADJ_UP_LIM);

//        pwm_adj_lim.w = 0;
//        pwm_adj_ctrl.w = ppwm_adp->base_addr->auto_adj_ctrl;
//
//        pwm_adj_lim.b.duty_adj_up_lim = 0;
//        pwm_adj_lim.b.duty_adj_dn_lim = (min_duty_ns) / tick_ns;
//        ppwm_adp->base_addr->auto_adj_limit = pwm_adj_lim.w;

		PWM_OBJ->PWM_AUTO_ADJ_LIMIT = PWM_ADJ_LIM;
		if (step_period_cnt > 1) {
			cycle_cnt = step_period_cnt - 1;    // HW cycle count = cycle count - 1;
		} else {
			cycle_cnt = 0;
		}

		hal_rtl_pwm_set_duty_adj_cycle(ppwm_adp, cycle_cnt);


		PWM_ADJ_CTRL |= (PWM_MASK_DUTY_DEC_STEP & ((step_sz_ns / tick_ns) << PWM_SHIFT_DUTY_DEC_STEP));
		PWM_ADJ_CTRL &= (~PWM_MASK_DUTY_INC_STEP);
//        pwm_adj_ctrl.b.duty_inc_step = 0;
//        pwm_adj_ctrl.b.duty_dec_step = (step_sz_ns) / tick_ns;
//        pwm_adj_ctrl.b.adj_dir = PwmDutyAdj_Decrease;
//        pwm_adj_ctrl.b.adj_loop_en = PwmDutyLoopOff;

		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_DIR);
		PWM_ADJ_CTRL &= (~PWM_BIT_ADJ_LOOP_EN);


		// write the duty auto-adjustment control register
		PWM_OBJ->PWM_AUTO_ADJ_CTRL = PWM_ADJ_CTRL;

		// write the duty auto-adjustment control register
		//ppwm_adp->base_addr->auto_adj_ctrl = pwm_adj_ctrl.w;
		result = HAL_OK;
		if (result == HAL_OK) {
			hal_rtl_pwm_auto_duty_en(ppwm_adp, 1);
		}

		return result;
	} else {
		DBG_PWM_ERR("hal_rtl_pwm_auto_duty_ns_dec: The timer group1 should be set 20MHz\r\n");
		return HAL_NOT_READY;
	}


}

/**
 *  \brief To read the time period of current duty of the PWM
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns u32: The PWM on duty duration, unit is ns
 */
SECTION_PWM_TEXT
u32 hal_rtl_pwm_read_duty_ns(hal_pwm_adapter_t *ppwm_adp)
{
	//pwm_comm_indread_idx_t pwm_indrd_idx;
	u32 duty_ticks;
	u32 duty_ns;
	u32 i = 0;

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	u32 PWM_INDRD_IDX;
	PWM_INDRD_IDX = 0;
	PWM_INDRD_IDX |= ppwm_adp->pwm_id;
	PWM_INDRD_IDX |= PWM_COMM_BIT_POOL;
	PWM_INDRD_IDX |= PWM_COMM_BIT_SYNC_MODE;

	// read the PWM duty tick count by in-direct reading
//    pwm_indrd_idx.w = 0;
//    pwm_indrd_idx.b.pwm_sel = ppwm_adp->pwm_id;
//    pwm_indrd_idx.b.pool = 1; // enable in-direct read
//    pwm_indrd_idx.b.sync_mode = 1; // in-direct read sync mode

	PWM_COMM_OBJ->PWM_COMM_INDREAD_IDX = PWM_INDRD_IDX;

	//ppwm_comm_adapter->base_addr->indread_idx = pwm_indrd_idx.w;
	//while (ppwm_comm_adapter->base_addr->indread_idx_b.pool == 1) {    // wait HW ready
	while (((PWM_COMM_OBJ->PWM_COMM_INDREAD_IDX & PWM_COMM_BIT_POOL) >> PWM_COMM_SHIFT_POOL) == 1) {
		hal_rtl_misc_delay_us(1);
		i++;
		if (i > 1000000) {
			DBG_PWM_ERR("hal_rtl_pwm_read_duty_ns: Wait indirect-read ready timeout\r\n");
			break;
		}
	}
	duty_ticks = PWM_COMM_OBJ->PWM_COMM_INDREAD_DUTY & PWM_COMM_MASK_PWM_DUTY;
	//duty_ticks = ppwm_comm_adapter->base_addr->indread_duty_b.pwm_duty;
	duty_ns = (duty_ticks * 50);
	return (duty_ns);


}

/** @} */ /* End of group hs_hal_pwm_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_pwm */

SECTION_PWM_TEXT
void _pwm_dma_irq_handler(phal_pwm_adapter_t ppwm_adp)
{
#if 0
	UART0_Type *puart = puart_adapter->base_addr;

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
		uart_clear_state_rtl8710c(puart_adapter, HAL_UART_STATE_DMATX_BUSY);
	} else {
		puart->ier_b.etbei = 1;
	}
#endif
}

SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_cache(IN phal_pwm_adapter_t ppwm_adp, IN void *dcache_clean_function, IN void *dcache_invalidate_function)
{
	if (dcache_clean_function == NULL) {
		ppwm_adp->dcache_clean_by_addr = (void (*)(uint32_t *, int32_t))rtl_dcache_clean_by_addr;
	} else {
		ppwm_adp->dcache_clean_by_addr = (void (*)(uint32_t *, int32_t))dcache_clean_function;
	}

	if (dcache_invalidate_function == NULL) {
		ppwm_adp->dcache_invalidate_by_addr = (void (*)(uint32_t *, int32_t))rtl_dcache_invalidate_by_addr;
	} else {
		ppwm_adp->dcache_invalidate_by_addr = (void (*)(uint32_t *, int32_t))dcache_invalidate_function;
	}
	return HAL_OK;
}




/**
 *  @brief To initial a GDMA channel for the PWM TX DMA mode transfer.
 *
 *  @param[in]  pwm_adapter  The PWM adapter.
 *  @param[in]  pgdma_chnl   The GDMA channel adapter. It is use to control the GDMA channel transfer.
 *  @param[in]  mode         Different dma mode, 5: only change duty, 6: only change period,
 *                           7: change duty and period simultaneously.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_dma_init(hal_pwm_adapter_t *ppwm_adp, phal_gdma_adaptor_t pgdma_chnl, int mode)
{
	uint32_t dst_per = 0;

	if ((NULL == ppwm_adp) || (NULL == pgdma_chnl)) {
		//dbg_printf("hal_pwm_tx_gdma_init: pwm_adapter=0x%x pgdma_chnl=0x%x\n", *ppwm_adp, pgdma_chnl);
		return HAL_ERR_PARA;
	}

	PWM_TypeDef *PWM_OBJ = (PWM_TypeDef *)(ppwm_adp->base_addr);

	switch (ppwm_adp->pwm_id) {
	case 0:
		dst_per = 20;
		HAL_WRITE32(0x40045020, 0x14, mode);
		break;
	case 1:
		dst_per = 21;
		HAL_WRITE32(0x40045040, 0x14, mode);
		break;
	case 2:
		dst_per = 22;
		HAL_WRITE32(0x40045060, 0x14, mode);
		break;
	case 3:
		dst_per = 23;
		HAL_WRITE32(0x40045080, 0x14, mode);
		break;
	}

	//dbg_printf("pwm_id %x dst_per %x\r\n", ppwm_adp->pwm_id, dst_per);

	pgdma_chnl->gdma_ctl.tt_fc      = TTFCMemToPeri;
	//pgdma_chnl->gdma_cfg.reload_dst = 1;
	pgdma_chnl->gdma_cfg.dest_per   = (u8)dst_per;                  //
	pgdma_chnl->ch_dar = (u32) & (PWM_OBJ->PWM_DMA_BUFF);
	//dbg_printf("pwm address %x\r\n", (u32)&(PWM_OBJ->PWM_DMA_BUFF) );
	pgdma_chnl->gdma_isr_type = (TransferType | ErrType);
	pgdma_chnl->gdma_ctl.int_en      = 1;
	pgdma_chnl->gdma_ctl.dinc = NoChange;
	pgdma_chnl->gdma_ctl.sinc = IncType;
	pgdma_chnl->have_chnl = 0;

	//hal_gdma_irq_reg(pgdma_chnl, (irq_handler_t)_pwm_dma_irq_handler, ppwm_adp);
	ppwm_adp->ptx_gdma = pgdma_chnl;
	return HAL_OK;



}

/**
 *  @brief To send a block of data by the DMA transmission mode.
 *
 *  @param[in]  pwm_adapter The PWM adapter.
 *  @param[in]  ptx_buf     The buffer of data to be transmitted.
 *  @param[in]  len         The length of data in bytes to be transmitted.
 *
 *  @return     HAL_OK: function execution OK.
 *  @return     HAL_BUSY: PWM TX is in busy state, previous transmission is not finished yet.
 *  @return     HAL_ERR_PARA: Input arguments are invalid.
 *  @return     HAL_NO_RESOURCE: Multiple-block DMA channel allocation failed.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_dma_send(hal_pwm_adapter_t *ppwm_adp, uint8_t *ptx_buf, uint32_t len)
{
	//uint32_t block_size;
	hal_gdma_adaptor_t *pgdma_chnl = ppwm_adp->ptx_gdma;

	if ((ptx_buf == NULL) || (len == 0) || (pgdma_chnl == NULL)) {
		//puart_adapter->tx_status = HAL_UART_STATUS_ERR_PARA;
		DBG_PWM_ERR("hal_pwm_dma_send: Err: pTxData=0x%x,  Length=%d GDMA_Chnl=0x%x\n", ptx_buf, len, pgdma_chnl);
		return HAL_ERR_PARA;
	}

	pgdma_chnl->ch_sar = (uint32_t) ptx_buf;
	if (((len & 0x03) == 0) && (((uint32_t)(ptx_buf) & 0x03) == 0)) {
		// 4-bytes aligned, move 4 bytes each transfer
		pgdma_chnl->gdma_ctl.src_msize   = MsizeOne; // MsizeOne
		pgdma_chnl->gdma_ctl.src_tr_width = TrWidthFourBytes;
		pgdma_chnl->gdma_ctl.block_size = len >> 2;      // move 1 byte each transfer

	} else {
		pgdma_chnl->gdma_ctl.src_msize   = MsizeFour;
		pgdma_chnl->gdma_ctl.src_tr_width = TrWidthOneByte;
		pgdma_chnl->gdma_ctl.block_size = len;

	}


#if 0
	pgdma_chnl->gdma_ctl.dest_msize  = MsizeFour;
	pgdma_chnl->gdma_ctl.dst_tr_width = TrWidthOneByte;
#else
	pgdma_chnl->gdma_ctl.dest_msize  = MsizeOne; // MsizeOne
	pgdma_chnl->gdma_ctl.dst_tr_width = TrWidthFourBytes;
#endif

	if (ppwm_adp->dcache_clean_by_addr != NULL) {
		ppwm_adp->dcache_clean_by_addr((uint32_t *)ptx_buf, (int32_t)len);
	}
	if (pgdma_chnl->gdma_ctl.block_size <= MAX_DMA_BLOCK_SIZE) {
		//puart_adapter->base_addr->miscr_b.txdma_en = 1; // Enable UART TX DMA
		hal_rtl_gdma_transfer_start(pgdma_chnl, 0);
	}

	return HAL_OK;
}

/**
 *  @brief To initial a GDMA channel for the PWM TX DMA mode transfer.
 *
 *  @param[in]  pwm_adapter The PWM adapter.
 *  @param[in]  group       The PWM complementary pair, set this value to 0 for PWM0&1, 1 for PWM2&3...5 for PWMA&B.
 *  @param[in]  DB_value    The DandBand value for chosen PWM complementary pair.
 *  @param[in]  en          Enable or Disable PWM complementary mode.
 *
 *  @return     HAL_ERR_PARA:  Input arguments are invalid.
 *  @return     HAL_OK:  GDMA channel initialization OK.
 */
SECTION_PWM_TEXT
hal_status_t hal_rtl_pwm_complementary(hal_pwm_adapter_t *ppwm_adp, uint8_t group, uint8_t DB_value, bool en)
{
	if ((NULL == ppwm_adp) || (group > 5)) {
		return HAL_ERR_PARA;
	}

	PWM_COMM_TypeDef *PWM_COMM_OBJ =
		(PWM_COMM_TypeDef *)(ppwm_comm_adapter->base_addr);

	if (en == _TRUE) {
		// Enable corresponding pwm pair complementary mode
		PWM_COMM_OBJ->PWM_COMM_COMPLEMENTARY |= ((1 << group) & PWM_COMM_MASK_PWM_COMP_EN);
		// Clear DandBand value in corresponding pwm pair
		PWM_COMM_OBJ->PWM_COMM_COMPLEMENTARY &= ~(0xF << (PWM_COMM_SHIFT_PWM_DB + group * 4));
		// Set DandBand value in corresponding pwm pair
		PWM_COMM_OBJ->PWM_COMM_COMPLEMENTARY |= (DB_value << (PWM_COMM_SHIFT_PWM_DB + group * 4));
	} else if (en == _FALSE) {
		// Disable corresponding pwm pair complementary mode
		PWM_COMM_OBJ->PWM_COMM_COMPLEMENTARY &= ~((1 << group) & PWM_COMM_MASK_PWM_COMP_EN);
		// Clear DandBand value in corresponding pwm pair
		PWM_COMM_OBJ->PWM_COMM_COMPLEMENTARY &= ~(0xF << (PWM_COMM_SHIFT_PWM_DB + group * 4));
	}


	return HAL_OK;
}





#endif  // end of "#if CONFIG_PWM_EN && CONFIG_GTIMER_EN"




