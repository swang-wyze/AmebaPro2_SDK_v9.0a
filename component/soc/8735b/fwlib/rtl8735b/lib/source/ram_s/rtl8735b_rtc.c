/**************************************************************************//**
 * @file     rtl8735b_rtc.c
 * @brief    This file implements the RTC HAL functions.
 *
 * @version  V1.00
 * @date     2021-05-13
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
#include <arm_cmse.h>
#include "rtl8735b_rtc_type.h"
#include "hal_sys_ctrl.h"
#include "hal_irq.h"
#if defined ( __GNUC__ )
#include <limits.h>
#else
#include "__limits.h"
#endif

#if CONFIG_RTC_EN

#define SECTION_RTC_TEXT           //SECTION(".ram.hal_rtc.text")
#define SECTION_RTC_DATA           //SECTION(".ram.hal_rtc.data")
#define SECTION_RTC_RODATA         //SECTION(".ram.hal_rtc.rodata")
#define SECTION_RTC_BSS            //SECTION(".ram.hal_rtc.bss")
#define SECTION_RTC_STUBS          //SECTION(".ram.hal_rtc.stubs")

extern void *memset(void *dst0, int Val, SIZE_T length);

/**
 * @addtogroup ls_hal_rtc RTC
 * @{
 */

//void RTC_IRQHandler(void);

/**
  * @brief The table of RTC registers base address.
  */
SECTION_RTC_RODATA
const RTC_Type *rtc_base_address = { RTC_BASE };//
// rtc //};


/**
  * @brief The global data structure to handle RTC adapters.
  */
SECTION_RTC_BSS hal_rtc_adapter_t *prtc_adapter;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup ls_hal_rtc_rom_func RTC HAL ROM APIs.
 * @ingroup ls_hal_rtc
 * @{
 * @brief The RTC HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of RTC HAL APIs in the RAM space is provided for the user application.
 */

/**
  \brief  The stubs functions table of the RTC HAL functions in ROM.
*/
SECTION_RTC_STUBS const hal_rtc_func_stubs_t hal_rtc_stubs = {
	.prtc_adp = &prtc_adapter,
	//.hal_rtc_irq_handler = RTC_IRQHandler,
	.hal_rtc_comm_irq_reg = hal_rtl_rtc_comm_irq_reg,
	.hal_rtc_comm_irq_unreg = hal_rtl_rtc_comm_irq_unreg,
	.hal_rtc_init = hal_rtl_rtc_init,
	.hal_rtc_deinit = hal_rtl_rtc_deinit,
	.hal_rtc_enable = hal_rtl_rtc_enable,
	.hal_rtc_disable = hal_rtl_rtc_disable,
	//.hal_rtc_isenable = hal_rtc_isenable_rtl8735b,
	.hal_rtc_wait_ctrl_ready = hal_rtl_rtc_wait_ctrl_ready,
	.hal_rtc_write = hal_rtl_rtc_write,
	.hal_rtc_read = hal_rtl_rtc_read,
	.hal_rtc_set_time = hal_rtl_rtc_set_time,
	.hal_rtc_read_time = hal_rtl_rtc_read_time,
	.hal_rtc_set_alarm = hal_rtl_rtc_set_alarm,
	.hal_rtc_read_alarm = hal_rtl_rtc_read_alarm,
	.hal_rtc_reg_alarm_irq = hal_rtl_rtc_reg_alarm_irq,
	.hal_rtc_unreg_alarm_irq = hal_rtl_rtc_unreg_alarm_irq,
	.hal_rtc_set_cnt_alarm = hal_rtl_rtc_set_cnt_alarm,
	.hal_rtc_set_comp = hal_rtl_rtc_set_comp,
};

/**
 *  \brief To handle the RTC interrupt.
 *
 *  \returns void
 */
#if defined (CONFIG_BUILD_SECURE)
typedef void __attribute__((cmse_nonsecure_call)) nsfunc(void);
#endif

SECTION_RTC_TEXT
//void rtc_irq(void)
void RTC_IRQHandler(void)
{

	hal_rtc_adapter_t *prtc_adp;
	prtc_adp = prtc_adapter;
	//u8 buff = 0;
	hal_irq_clear_pending(RTC_IRQn);
	dbg_printf("post clear irq\r\n");
	if (prtc_adp->alarm_callback != NULL) {
#if defined (CONFIG_BUILD_SECURE)
		nsfunc *fp = cmse_nsfptr_create((nsfunc *)(prtc_adp->alarm_callback));
		fp();
#else
		// call the user RTC Alarm callback function
		//(prtc_adp->alarm_callback)(prtc_adp->rtc_cb_para);
		prtc_adp->alarm_callback(prtc_adp);
#endif
	}

	dbg_printf("before clear isr\r\n");
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ISR = 0x1FFFF;     // REG_RTC_ISR, Write 1 Clear
	hal_rtl_rtc_write(prtc_adp, Rtc_Isr, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ISR);
	__DSB();


}



/**
 *  \brief To register a IRQ handler for the RTC common interrupt.
 *
 *  \param[in] irq_handler The IRQ handler.
 *
 *  \returns void
 */
SECTION_RTC_TEXT
void hal_rtl_rtc_comm_irq_reg(irq_handler_t irq_handler)
{


	// IRQ vector may has been registered, disable and re-register it
	hal_irq_disable(RTC_IRQn);
	__ISB();
	hal_irq_set_vector(RTC_IRQn, (uint32_t)irq_handler);
	hal_irq_set_priority(RTC_IRQn, RTC_IRQPri);
	hal_irq_enable(RTC_IRQn);

}

/**
 *  \brief To un-register the RTC common IRQ handler.
 *
 *  \returns void
 */
SECTION_RTC_TEXT
void hal_rtl_rtc_comm_irq_unreg(void)
{

	hal_irq_disable(RTC_IRQn);
	__ISB();
	hal_irq_set_vector(RTC_IRQn, (uint32_t)NULL);
}


/**
 *  \brief To initial a RTC devices adapter. This is the first function must be called
 *         before to do any operation to the RTC devices.
 *
 *  \param[in] prtc_adp The RTC devices adapter.
 *
 *  \returns HAL_OK: Initial succeed.
 */
SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_init(hal_rtc_adapter_t *prtc_adp)  // Weide: Check the entire init function
{
	memset((void *)prtc_adp, 0, sizeof(hal_rtc_adapter_t));
	prtc_adp->base_addr = (RTC_Type *)rtc_base_address;
	prtc_adapter = prtc_adp; // Initialize prtc_adpter
	hal_rtl_rtc_comm_irq_reg((irq_handler_t)RTC_IRQHandler);
	hal_rtl_rtc_enable(prtc_adp);

	return HAL_OK;
}

/**
 *  \brief To de-initial a RTC devices adapter. This is the first function must be called
 *         before to do any operation to the RTC devices.
 *
 *  \param[in] prtc_adp The RTC devices adapter.
 *
 *  \returns HAL_OK: Setting succeed.
 */
SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_deinit(hal_rtc_adapter_t *prtc_adp)
{

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ISR = 0xFFFFFFFF;     // REG_RTC_ISR, Write 1 Clear

	hal_rtl_rtc_write(prtc_adp, Rtc_Isr, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ISR);
	hal_rtl_rtc_comm_irq_unreg();
	hal_rtl_rtc_disable(prtc_adp);
	prtc_adapter = NULL;

	return HAL_OK;

}


/**
 *  \brief To enable a RTC devive.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns void
 */
SECTION_RTC_TEXT
void hal_rtl_rtc_enable(hal_rtc_adapter_t *prtc_adp)
{
	//dbg_printf("hal_rtl_rtc_enable"); //

	AON_TypeDef *aon = AON;
	u32 buff;

	hal_sys_peripheral_en(RTC_SYS, ENABLE);
	hal_delay_ms(4);

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN = 0x03; // dont know how to change /
	// First, put data in LSFIF RWD Reg, after which, trigger the LSFIF CMD to write data to RTC contained in LSFIF RWD Reg

	aon->AON_REG_AON_LSFIF_RWD = prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN ; // LSF(low speed function) write*/

	aon->AON_REG_AON_LSFIF_CMD = Rtc_Fen;   // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD |= 0xF <<
								  AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_WR; // 1: LSF register write transfer, 0: LSF register read transfer
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW

	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);   // Waiting for Write Succeed

}



/**
 *  \brief To disable a RTC devive.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns void
 */

SECTION_RTC_TEXT
void hal_rtl_rtc_disable(hal_rtc_adapter_t *prtc_adp)
{
	AON_TypeDef *aon = AON;

	hal_sys_peripheral_en(RTC_SYS, DISABLE);
	aon->AON_REG_AON_LSFIF_CMD =
		Rtc_Fen;   // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD |= 0xF <<
								  AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_WR; // 1: LSF register write transfer, 0: LSF register read transfer
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW

	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);   // Waiting for Write Succeed

}




/**
 *  \brief To wait the RTC HW ready to set new RTC register.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns void
 */
SECTION_RTC_TEXT
void hal_rtl_rtc_wait_ctrl_ready(hal_rtc_adapter_t *prtc_adp)
{
	AON_TypeDef *aon = AON;

	u32 i;
	for (i = 0; i < 1000000; i++) {
		if ((((aon->AON_REG_AON_LSFIF_CMD) & AON_BIT_AON_LSFIF_POLL) >> AON_SHIFT_AON_LSFIF_POLL) == 0) {
			break;  // break the for loop
		} else {
			hal_delay_ms(1);
		}
	}
}

/**
 *  \brief To Write a RTC register.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] addr The RTC Reg base address.
 *  \param[in] data The RTC Write Data.
 *
 *  \returns void
 */
SECTION_RTC_TEXT
void hal_rtl_rtc_write(hal_rtc_adapter_t *prtc_adp, u16 addr, u32 data)
{

	AON_TypeDef *aon = AON;

	aon->AON_REG_AON_LSFIF_RWD = data;  // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD = addr;   // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address//
	aon->AON_REG_AON_LSFIF_CMD |= 0xF << AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_WR; // 1: LSF register write transfer, 0: LSF register read transferi
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW
	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);

}

/**
 *  \brief To Read a RTC register.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] addr The RTC Reg base address.
 *
 *  \returns HAL_ERR_PARA: Input arguments are invalid.
 *  \returns u32: RTC Data.
 */
SECTION_RTC_TEXT
u32 hal_rtl_rtc_read(hal_rtc_adapter_t *prtc_adp, u16 addr)
{
	AON_TypeDef *aon = AON;
	u32 Data = 0;
	if (prtc_adapter == NULL) {
		DBG_MISC_ERR("hal_rtl_rtc_read RTC Struct isn't Initialize!!\r\n");
		return HAL_ERR_PARA;
	}

	aon->AON_REG_AON_LSFIF_CMD = addr;  // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD |= 0xF << AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD &= ~ AON_BIT_AON_LSFIF_WR; // 0: LSF register read transfer
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW

	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);
	Data = aon->AON_REG_AON_LSFIF_RWD;          // LSF(low speed function) write*/

	return Data;
}


/**
 *  \brief To set the time of the RTC
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] ptm_reg The tm structure information.hal_rtc_reg_alarm_irq
 *  \param[in] leap_year, Is Leap Year. 1: leap year
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
 */

SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_set_time(hal_rtc_adapter_t *prtc_adp, hal_tm_reg_t *ptm_reg, u16 leap_year)
{
	u32 buff_w; // Write Buffer
	hal_status_t ret;
	u8 i = 0;
	AON_TypeDef *aon = AON;

	if (prtc_adapter == NULL) {
		// ////dbg_printf("RTC Struct isn't Initialize!!\r\n");
		return HAL_ERR_PARA;
	}
	if ((ptm_reg->mon > 12) || (ptm_reg->mday > 31) || (ptm_reg->wday > 6) || (ptm_reg->hour > 23) ||
		(ptm_reg->min > 59) || (ptm_reg->sec > 59)) {
		// ////dbg_printf("hal_rtl_rtc_set_time: parameter over-range\r\n");
		return HAL_ERR_PARA;
	}

	// Disable RTC Divider, Reset divider counter
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN = BIT_RTC_EN;   // Disable RTC CLK divider
	aon->AON_REG_AON_LSFIF_RWD = prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN; // LSF(low speed function) write*/
	aon->AON_REG_AON_LSFIF_CMD = Rtc_Fen;  // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD |= 0xF << AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_WR; // 1: LSF register write transfer, 0: LSF register read transfer
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW

	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);   // Waiting for Write Succeed

	// Set RTC Information
	//TIM0

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0  = (ptm_reg->sec); // Seconds value in the range of 0 to 59
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0  |= ((ptm_reg->min) << BIT_SHIFT_RTC_MIN); // Minutes value in the range of 0 to 593
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0  |= ((ptm_reg->hour) << BIT_SHIFT_RTC_HRS); // Hours value in the range of 0 to 23
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0  |= ((ptm_reg->wday) << BIT_SHIFT_RTC_DOW); // Minutes value in the range of 0 to 593
	hal_rtl_rtc_write(prtc_adp, Rtc_Tim0, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0);
	hal_delay_ms(10);//

	//TIM2


	if (leap_year) {// Is leap year?
		prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2 &= ~(BIT_MASK_RTC_M4ISCYR);   // Year is leap year
	} else {
		prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2  |= BIT_MASK_RTC_M4ISCYR; // Year is not leap year
	}
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2 |= (ptm_reg->yday + 1);    // Day of year, value in the range of 1 to 366
	hal_rtl_rtc_write(prtc_adp, Rtc_Tim2, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2);
	hal_delay_ms(10);//

	//TIM0

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM1 = ptm_reg->mday;// Day of month value in the range of 1 to 28, 29, 30, or 31
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM1 |= ((ptm_reg->mon + 1) << BIT_SHIFT_RTC_MON) ; // Month value in the range of 1 to 12 s
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM1 |= ((ptm_reg->year + 1900) << BIT_SHIFT_RTC_YEAR) ; // Year value in the range of 0 to 4095
	hal_rtl_rtc_write(prtc_adp, Rtc_Tim1, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM1);
	hal_delay_ms(10);

	// Enable RTC Divider
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN = (BIT_CKDIV_EN | BIT_RTC_EN); // Enable RTC CLK divider
	aon->AON_REG_AON_LSFIF_RWD = prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_FEN; // LSF(low speed function) write*/

	aon->AON_REG_AON_LSFIF_CMD = Rtc_Fen;  // BIT_AON_LSFIF_AD[15:8]=8'h00: RTC register base address
	aon->AON_REG_AON_LSFIF_CMD |= 0xF << AON_SHIFT_AON_LSFIF_WE; // LSF register byte write enable
	aon->AON_REG_AON_LSFIF_CMD |= AON_BIT_AON_LSFIF_WR; // 1: LSF register write transfer, 0: LSF register read transfer
	aon->AON_REG_AON_LSFIF_CMD |=
		AON_BIT_AON_LSFIF_POLL; // Set this bit to do LSF register read or write transfer depend on BIT_AON_LSFIF_WR. When transfer done, this bit will be clear by HW

	// wait rtc ready
	hal_rtl_rtc_wait_ctrl_ready(prtc_adp);   // Waiting for Write Succeed

	return HAL_OK;

}


/**
 *  \brief To read the current time of RTC
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns     HAL_OK:  Setting succeed.
 */
SECTION_RTC_TEXT
u32 hal_rtl_rtc_read_time(hal_rtc_adapter_t *prtc_adp)
{

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0 = hal_rtl_rtc_read(prtc_adp, Rtc_Tim0);
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM1 = hal_rtl_rtc_read(prtc_adp, Rtc_Tim1);
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2 = hal_rtl_rtc_read(prtc_adp, Rtc_Tim2);
	return (HAL_OK);

}

/**
 *  \brief To set RTC Alarm
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] prtc_alarm The RTC alarm adapter.
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
 */
SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_set_alarm(hal_rtc_adapter_t *prtc_adp, hal_rtc_alarm_t *prtc_alarm)
{
	volatile uint32_t val;


	if (prtc_adapter == NULL) {
		DBG_MISC_ERR("RTC Struct isn't Initialize!!\r\n");
		return HAL_ERR_PARA;
	}

	if ((prtc_alarm->hour > 23) || (prtc_alarm->min > 59) || (prtc_alarm->sec > 59)) {
		DBG_MISC_ERR("hal_rtl_rtc_set_alarm: parameter over-range\r\n");
		return HAL_ERR_PARA;
	}
	// Read RTC Time

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0 = hal_rtl_rtc_read(prtc_adp, Rtc_Tim0);
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2 = hal_rtl_rtc_read(prtc_adp, Rtc_Tim2);
	if (prtc_alarm->yday == ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM2 & BIT_MASK_RTC_DOY) >> BIT_SHIFT_RTC_DOY)) { // prevent Isr_doy wouldn't be set
		if (prtc_alarm->hour == ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0 & BIT_MASK_RTC_HRS) >> BIT_SHIFT_RTC_HRS)) { // prevent Isr_hrs wouldn't be set
			if (prtc_alarm->min < ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0 & BIT_MASK_RTC_MIN) >> BIT_SHIFT_RTC_MIN)) {
				DBG_MISC_ERR("hal_rtl_rtc_set_alarm: alarm_min should not less than RTC min\r\n");
				return HAL_ERR_PARA;
			}
		} else if (prtc_alarm->hour < ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_TIM0 & BIT_MASK_RTC_HRS) >> BIT_SHIFT_RTC_HRS)) {
			DBG_MISC_ERR("hal_rtl_rtc_set_alarm: alarm_hour should not less than RTC hour\r\n");
			return HAL_ERR_PARA;
		}
	}


	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 &= ~(prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 & BIT_MASK_RTC_ALRM_SEC); // clear Second value
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 |= ((prtc_alarm->sec) << BIT_SHIFT_RTC_ALRM_SEC) ; // Seconds value in the range of 0 to 59
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 &= ~(prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 & BIT_MASK_RTC_ALRM_MIN); // clear Minutes value
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 |= ((prtc_alarm->min) << BIT_SHIFT_RTC_ALRM_MIN) ; // Minutes value in the range of 0 to 59
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 &= ~(prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 & BIT_MASK_RTC_ALRM_HRS); // clear Hour value
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 |= ((prtc_alarm->hour) << BIT_SHIFT_RTC_ALRM_HRS); // Min Hours value in the range of 0 to 23
	hal_rtl_rtc_write(prtc_adp, Rtc_Ctrl0, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0);


	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 = ((prtc_alarm->yday) << BIT_SHIFT_RTC_ALRM_DOY); // Day of year, value in the range of 1 to 366
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 |= BIT_MASK_RTC_ALRM_SEC_EN;                // Enable sec,min, hrs, doy Alarm
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 |= BIT_MASK_RTC_ALRM_MIN_EN;                // Enable sec,min, hrs, doy Alarm
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 |= BIT_MASK_RTC_ALRM_HRS_EN;                // Enable sec,min, hrs, doy Alarm
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 |= BIT_MASK_RTC_ALRM_DOY_EN;                // Enable sec,min, hrs, doy Alarm
	hal_rtl_rtc_write(prtc_adp, Rtc_Ctrl2, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2);
	hal_delay_ms(5);//

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_IMR |= BIT_RTC_ALRM_IMR;           // Indicate ALL timer value match ALRM value
	hal_rtl_rtc_write(prtc_adp, Rtc_Imr, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_IMR);

	hal_delay_ms(5);//


	val = AON->AON_REG_AON_SLP_WAKE_EVENT_MSK0;
	//1: enable AON Wakeup event; 0: disable the event to wakeup system //
	val |= (AON_BIT_AON_WEVT_AON_RTC_MSK);
	AON->AON_REG_AON_SLP_WAKE_EVENT_MSK0 = val;

	hal_rtl_rtc_write(prtc_adp, Rtc_Ctrl2, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2);
	return HAL_OK;

}

/**
 *  \brief To read the current alarm setting of RTC
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns     HAL_OK:  Setting succeed.
 */
SECTION_RTC_TEXT
u32 hal_rtl_rtc_read_alarm(hal_rtc_adapter_t *prtc_adp)
{
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_IMR = hal_rtl_rtc_read(prtc_adp, Rtc_Imr);

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL1 = hal_rtl_rtc_read(prtc_adp, Rtc_Ctrl1);

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL0 = hal_rtl_rtc_read(prtc_adp, Rtc_Ctrl0);

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 = hal_rtl_rtc_read(prtc_adp, Rtc_Ctrl2);

	return HAL_OK;
}


/**
 *  \brief To register the RTC interrupt.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] callback The callback function. It will be called when the interrupt is occurred.
 *  \param[in] arg The argument of the callback function.
 *
 *  \returns void
 */

SECTION_RTC_TEXT
void hal_rtl_rtc_reg_alarm_irq(hal_rtc_adapter_t *prtc_adp, rtc_alarm_callback_t callback, void *arg)
{

	if (callback != NULL) {
		prtc_adp->alarm_callback = callback;
		prtc_adp->rtc_cb_para = arg;
	} else {
		DBG_MISC_ERR("hal_rtl_rtc_reg_alarm_irq: callback Func is NULL\r\n");
	}
}

/**
 *  \brief To un-register the RTC interrupt.
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *
 *  \returns void
 */

SECTION_RTC_TEXT
void hal_rtl_rtc_unreg_alarm_irq(hal_rtc_adapter_t *prtc_adp)
{

	prtc_adp->alarm_callback = (rtc_alarm_callback_t)NULL;
	prtc_adp->rtc_cb_para = (void *)NULL;
	//hal_rtl_rtc_comm_irq_unreg();

}

/**
 *  \brief To set RTC Cnt Alarm
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] cnt_alarm The RTC alarm paramter. bit 0 -> sec, bit 1 -> min, bit 2 -> hrs, bit 3 -> dow..bit 7 -> doy
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
 */
SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_set_cnt_alarm(hal_rtc_adapter_t *prtc_adp, u8 cnt_alarm)
{

	if (prtc_adapter == NULL) {
		DBG_MISC_ERR("RTC Struct isn't Initialize!!\r\n");
		return HAL_ERR_PARA;
	}

	// Set RTC Alarm
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2 = 0; // clear alarm ctrl2
	hal_rtl_rtc_write(prtc_adp, Rtc_Ctrl2, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_ALRM_CTRL2);
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_IMR = cnt_alarm;  // REG_RTC_IMR, Initial rtc_imr REG value
	hal_rtl_rtc_write(prtc_adp, Rtc_Imr, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_IMR);
	return HAL_OK;

}

/**
 *  @brief To read calibration value of RTC 32K.
 *
 *  @returns The 32k calculation result.
 */

SECTION_RTC_TEXT
u32 hal_rtc_cal_32k_rtl8735b(void)
{
	u32 buff; // Read Buffer

	VNDR_TypeDef *vendor = VNDR;
	if ((((PON->PON_REG_PON_SYSON_CTRL & PON_BIT_VENDOR_REG_EN) >> PON_SHIFT_VENDOR_REG_EN) == 0) ||
		(((PON->PON_REG_PON_SYSON_CTRL & PON_BIT_VENDOR_CLK_EN) >> PON_SHIFT_VENDOR_CLK_EN) == 0)) {
		//if VNDR and SYSON not on, turn on them
		PON->PON_REG_PON_SYSON_CTRL |= 1 << PON_SHIFT_VENDOR_REG_EN ;
		PON->PON_REG_PON_SYSON_CTRL |= 1 << PON_SHIFT_VENDOR_CLK_EN ;
	}
	hal_delay_us(5);
	vendor->VNDR_REG_ANACK_CAL_CTRL = 0x82000000; //REG_ANACK_CAL_CTRLs
	hal_delay_us(1);
	while ((vendor->VNDR_REG_ANACK_CAL_CTRL &  0x80000000) == 0x1) {
		hal_delay_us(1);
	}
	buff = vendor->VNDR_REG_ANACK_CAL_CTRL & 0xFFFF;


	return buff;
}

/**
 *  @brief To read calibration value of RTC 128K.
 *
 *  @returns The 128k calculation result.
 */

SECTION_RTC_TEXT
u32 hal_rtc_cal_128k_rtl8735b(void)
{
	u32 buff; // Read Buffer

	VNDR_TypeDef *vendor = VNDR;

	if ((((PON->PON_REG_PON_SYSON_CTRL & PON_BIT_VENDOR_REG_EN) >> PON_SHIFT_VENDOR_REG_EN) == 0) ||
		(((PON->PON_REG_PON_SYSON_CTRL & PON_BIT_VENDOR_CLK_EN) >> PON_SHIFT_VENDOR_CLK_EN) == 0)) {

		// ////dbg_printf("line 759\n\r "); //
		PON->PON_REG_PON_SYSON_CTRL |= 1 << PON_SHIFT_VENDOR_REG_EN ;
		PON->PON_REG_PON_SYSON_CTRL |= 1 << PON_SHIFT_VENDOR_CLK_EN ;
	}

	hal_delay_us(5);
	vendor->VNDR_REG_ANACK_CAL_CTRL = 0x81000000; //REG_ANACK_CAL_CTRLs
	hal_delay_us(1);

	while ((vendor->VNDR_REG_ANACK_CAL_CTRL &  0x80000000) == 0x1) {
		hal_delay_us(1);

	}
	buff = (vendor->VNDR_REG_ANACK_CAL_CTRL) & 0xFFFF;


	return buff;
}

/**
 *  \brief To set the compensation value of the RTC CLK
 *
 *  \param[in] prtc_adp The RTC device adapter.
 *  \param[in] func_en The RTC compensation function enable. 1: Enable; 0:Disable
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
 */

SECTION_RTC_TEXT
hal_status_t hal_rtl_rtc_set_comp(hal_rtc_adapter_t *prtc_adp, u8 func_en)
{
	u32 Data = 0;

	/*
	   0000: 40MHz,     0001: 25MHz,
	   0010: 13MHz,     0011: 19.2MHz
	   0100: 20MHz,     0101: 26MHz,
	   0110: 38.4MHz,  0111: 17.664MHz
	   1000: 16MHz,     1001: 14.138MHz
	   1010: 12MHz,     1011:  52MHz
	   1100: 48MHz,     1101: 27MHz,
	   1110: 24MHz,     1111: none
	   */
	u32 buff = 0;
	u32 xtal_sel = 0;
	u8  dir = 0; // 0: plus; 1 mimus
	u8  xtal_ns[16] = {25, 40, 77, 52, 50, 38, 26, 57, 63, 71, 83, 19, 21, 37, 42, 0xff};
	AON_TypeDef *aon = AON;
	//aon_osc_32k_ctrl_t osc_32k_ctrl;

	if (prtc_adapter == NULL) {
		DBG_MISC_ERR("RTC Struct isn't Initialize!!\r\n");
		return HAL_ERR_PARA;
	}

	xtal_sel = (AON->AON_REG_AON_OTP_SYSCFG5) & AON_MASK_XTAL_SEL; // xtal selection, Default=40MHz
	xtal_sel = xtal_sel  & 0xF;
	xtal_sel = 32; // pro2 only have 32

	Data = ((AON->AON_REG_AON_SRC_CLK_CTRL) & AON_MASK_LP_CLK_SEL) >> AON_SHIFT_LP_CLK_SEL;

	if (Data == 0x100) {  // 32768 Xtal
		buff = hal_rtc_cal_32k_rtl8735b();

		if (buff == 0) { // in case buff == 0 will make error
			buff = 1;
		}
		buff = (buff * xtal_sel) >> 3;
		buff = (1000000000 / buff);
		if (buff > 32767) {
			dir = 1;
			if ((buff - 32768) == 0) {
				buff = (0x3FFFF * 10);
				func_en = 0; // The Crystal frequency was accurate, disable comp func
			} else {
				buff = (buff * 10) / (buff - 32768); // 1-(2^15/buff)
			}
		} else {
			dir = 0;
			buff = (buff * 10) / (32768 - buff);
		}


	} else if ((Data == 0x001) || (Data == 0x000)) {  // 128k
		buff = hal_rtc_cal_128k_rtl8735b();
		if (buff == 0) { // in case buff == 0 will make error
			buff = 1;
		}
		buff = (buff * xtal_sel) >> 3;
		buff = (1000000000 / buff);
		if (buff > 131071) {
			dir = 1;
			if ((buff - 131072) == 0) {
				buff = (0x3FFFF * 10);
				func_en = 0; // The Crystal frequency was accurate, disable comp func
			} else {
				buff = (buff * 10) / (buff - 131072); // 1-(2^17/buff)
			}
		} else {
			dir = 0;
			buff = (buff * 10) / (131072 - buff);
		}
	}

	if ((buff % 10) > 4) { // rounding compensation
		buff = buff + 10;
		buff = buff / 10;
	} else {
		buff = buff / 10;
	}

	if (buff > 0x3FFFF) {
		buff = 0x3FFFF;
	} else if (buff > 1) {
		buff = buff - 1;
	}
	// Set RTC Compensation value
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP & RTC_MASK_TYPE_RTC_ADJUST_PERIOD_SEL) <<
			RTC_SHIFT_TYPE_RTC_ADJUST_PERIOD_SEL); //clear
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= buff << RTC_SHIFT_TYPE_RTC_ADJUST_PERIOD_SEL ;  // unit: s

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP & RTC_BIT_TYPE_ADJUST_DIRECTION) <<
			RTC_SHIFT_TYPE_ADJUST_DIRECTION); //clear
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= dir << RTC_SHIFT_TYPE_ADJUST_DIRECTION ; // 0: plus; 1 mimus

	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= ((prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP & RTC_BIT_TYPE_FUNCTION_EN) << RTC_SHIFT_TYPE_FUNCTION_EN); //clear
	prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP |= func_en << RTC_SHIFT_TYPE_FUNCTION_EN ;  // 1: Enable; 0:Disable

	hal_rtl_rtc_write(prtc_adp, Rtc_Cmp, prtc_adp->rtc_reg.RTC_TYPE_REG_RTC_COMP);
	return HAL_OK;

}

/** @} */ /* End of group ls_hal_rtc_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group ls_hal_rtc */

#endif
