/**************************************************************************//**
 * @file     rtl8735b_trng.c
 * @brief    This file implements the trng HAL functions.
 *
 * @version  V1.00
 * @date     2021-01-25
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
#include "rtl8735b_trng_type.h"
#include "rtl8735b_aon_type.h"
#include "rtl8735b_pon_type.h"
#include "rtl8735b_syson_type.h"
#include "hal_irq.h"
#include "hal_sys_ctrl.h"

#if CONFIG_TRNG_EN
#if IS_CUT_TEST(CONFIG_CHIP_VER)

#define SECTION_TRNG_TEXT         //  SECTION(".rom.hal_trng.text")
#define SECTION_TRNG_DATA         //  SECTION(".rom.hal_trng.data")
#define SECTION_TRNG_RODATA      //   SECTION(".rom.hal_trng.rodata")
#define SECTION_TRNG_BSS         //   SECTION(".rom.hal_trng.bss")
#define SECTION_TRNG_STUBS       //   SECTION(".rom.hal_trng.stubs")

extern void *memset(void *dst0, int Val, SIZE_T length);

/**
  * @brief The global data structure to handle trng adapters.
  */
SECTION_TRNG_BSS hal_trng_adapter_t *ptrng_adapter;

/// @cond DOXYGEN_ROM_HAL_API




/**
 *  @brief The interrupt handler for trng interrupt
 *
 *  @returns    void
 */
SECTION_TRNG_TEXT
void trng_IRQHandler_patch(void)
{
////dbg_printf("trng_IRQHandler \n\r ");
	//uint32_t int_sts;

	//   hal_trng_adapter_t *ptrng_adp;
	//uint32_t trng_error_type;

	hal_irq_clear_pending(TRNG_IRQn);

	if (ptrng_adapter->trng_callback == NULL) {
		// Not initialed yet, ignore interrupt
		TRNG->TRNG_RNG_INT &= ~TRNG_BIT_ST_ERR; // clear interrupt
		TRNG->TRNG_RNG_INT &= ~TRNG_BIT_PARITY; // clear interrupt
		__DSB();
		__ISB();
		return;
	}

	if (ptrng_adapter->trng_callback != NULL) {
		// call the user registed call-back
		(ptrng_adapter->trng_callback)(ptrng_adapter->trng_arg);
	}


};



/**
 *  \brief To enable a trng devive.
 *
 *  \param[in] ptrng_adp The trng device adapter.
 *
 *  \returns void
 */
SECTION_TRNG_TEXT
void hal_rtl_trng_enable_128K_patch(hal_trng_adapter_t *ptrng_adp)
{
	//dbg_printf("hal_rtl_trng_enable_128K_patch /n/r ");
#if !defined(CONFIG_BUILD_NONSECURE)
	//dbg_printf("hal_rtl_trng_enable_128K_patch 129 \n\r ");
	hal_sys_peripheral_en(TRNG_128K, ENABLE);
#endif
	hal_delay_us(1);
	//dbg_printf("hal_rtl_trng_enable_128K_patch 133 \n\r ");

}
/**
 *  \brief To enable a trng device with wave input.
 *
 *  \param[in] ptrng_adp The trng device adapter.
 *
 *  \returns void
 */
SECTION_TRNG_TEXT
void hal_rtl_trng_enable_32K_patch(hal_trng_adapter_t *ptrng_adp)
{
	//dbg_printf("hal_rtl_trng_enable_32K_patch /n/r ");
#if !defined(CONFIG_BUILD_NONSECURE)
	//dbg_printf("hal_rtl_trng_enable_32K_patch 129 \n\r ");
	hal_sys_peripheral_en(TRNG_32K, ENABLE);
#endif
	hal_delay_us(1);
	//dbg_printf("hal_rtl_trng_enable_32K_patch 133 \n\r ");

}

/**
 *  \brief To disable a TRNG devive.
 *
 *  \param[in] ptrng_adp The TRNG device adapter.
 *
 *  \returns void
 */

SECTION_TRNG_TEXT
void hal_rtl_trng_disable_patch(hal_trng_adapter_t *ptrng_adp)
{
	////dbg_printf("hal_rtl_trng_disable \n\r ");
#if !defined(CONFIG_BUILD_NONSECURE)
	hal_sys_peripheral_en(TRNG_SYS, DISABLE);
#endif
}
/**
 *  @brief Registers a handler for the  interrupt.
 *
 *  @param[in]  handler  The interrupt handler.
 *  @param[in]  arg  The application data will be passed back to the application
 *                   with the callback function.
 *
 *  @returns    void
 */
SECTION_TRNG_TEXT
void hal_rtl_trng_reg_irq_patch(irq_handler_t handler, uint32_t *arg)
{
	////dbg_printf("hal_rtl_trng_reg_irq \n\r ");
	ptrng_adapter->trng_callback = handler;
	ptrng_adapter->trng_arg = arg;
}


/**
 *  \brief To register a IRQ handler for the trng common interrupt.
 *
 *  \param[in] irq_handler The IRQ handler.
 *
 *  \returns void
 */
SECTION_TRNG_TEXT
void hal_rtl_trng_comm_irq_reg_patch(irq_handler_t irq_handler)
{
	////dbg_printf("hal_rtl_trng_comm_irq_reg \n\r ");
	// IRQ vector may has been registered, disable and re-register it
	hal_irq_disable(TRNG_IRQn);
	__ISB();
	hal_irq_set_vector(TRNG_IRQn, (uint32_t)irq_handler);
	hal_irq_set_priority(SystemOn_IRQn, TRNG_IRQPri);
	hal_irq_enable(TRNG_IRQn);
}

/**
 *  \brief To un-register the trng common IRQ handler.
 *
 *  \returns void
 */
SECTION_TRNG_TEXT
void hal_rtl_trng_comm_irq_unreg_patch(void)
{
	////dbg_printf("hal_rtl_trng_comm_irq_unreg \n\r ");
	hal_irq_disable(TRNG_IRQn);
	__ISB();
	hal_irq_set_vector(TRNG_IRQn, (uint32_t)NULL);
}

/**
 *  \brief To set the control of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]  ptrng_reg
 *
 *  \returns     HAL_OK:  Setting succeed.
 */

SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_control_setting_patch(hal_trng_adapter_t *ptrng_adp, hal_rng_reg_t *ptrng_reg)
{

	ptrng_adp->trng_reg.TRNG_RNG_CTRL    = ((ptrng_reg->LFSR_mode2) << TRNG_SHIFT_LFSR_MODE2); // LFSR_MODE2
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->TRNG_mode) << TRNG_SHIFT_RNG_MODE); // TRNG_mode
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->selftest) << TRNG_SHIFT_ST_EN); // selftest
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->LFSR_mode) << TRNG_SHIFT_LFSR_MODE); // selftest
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->LFSR_bypass1) << TRNG_SHIFT_LFSR_BYPASS_1); // LFSR_bypass1
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->TRNG_dbg1_sel) << TRNG_SHIFT_DBG1_SEL); // TRNG_dbg1_sel
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->TRNG_dbg0_sel) << TRNG_SHIFT_DBG0_SEL); // TRNG_dbg0_sel
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->TRNG_corrector_lmode) << TRNG_SHIFT_CORRECTOR_IMODE); // TRNG_corrector_lmode
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->corrector_bypass) << TRNG_SHIFT_CORRECTOR_BYPASS); // TRNG_corrector_lmode
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->high_speed_clock) << TRNG_SHIFT_CLK_HF_SEL); // TRNG_corrector_lmode
	ptrng_adp->trng_reg.TRNG_RNG_CTRL   |= ((ptrng_reg->RNG_SRST) << TRNG_SHIFT_RNG_SRST); // TRNG_corrector_lmode

	//dbg_printf("ptrng_adp->trng_reg.TRNG_RNG_CTRL = %x \n\r ", ptrng_adp->trng_reg.TRNG_RNG_CTRL);
	TRNG->TRNG_RNG_CTRL = ptrng_adp->trng_reg.TRNG_RNG_CTRL;
	//dbg_printf("hal_rtl_trng_control_setting   \n\r ");

	return HAL_OK;
}


/**
 *  \brief To initial a trng devices adapter. This is the first function must be called
 *         before to do any operation to the trng devices.
 *
 *  \param[in] ptrng_adp The trng devices adapter.
 *
 *  \returns HAL_OK: Initial succeed.
 */
SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_init_patch(hal_trng_adapter_t *ptrng_adp)
{
	//dbg_printf("hal_rtl_trng_init!!!!!!!!!!!!! \n\r ");
	memset((void *)ptrng_adp, 0, sizeof(hal_trng_adapter_t));
	ptrng_adapter = ptrng_adp; // Initialize ptrng_adpter
	hal_rtl_trng_comm_irq_reg_patch((irq_handler_t)trng_IRQHandler_patch);
	return HAL_OK;
}

/**
 *  \brief To de-initial a TRNG devices adapter. This is the first function must be called
 *         before to do any operation to the trng devices.
 *
 *  \param[in] ptrng_adp The TRNG devices adapter.
 *
 *  \returns HAL_OK: Setting succeed.
 */
SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_deinit_patch(hal_trng_adapter_t *ptrng_adp)
{
////dbg_printf("hal_rtl_trng_deinit \n\r ");
	hal_rtl_trng_comm_irq_unreg_patch();
	hal_rtl_trng_disable_patch(ptrng_adp);
	ptrng_adapter = NULL;
	return HAL_OK;

}
/**
 *  \brief load deflaut
 *
 *  \param[in] ptrng_adp The TRNG devices adapter.
 *
 *  \returns HAL_OK: Setting succeed.
 */
SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_load_default_patch(hal_trng_adapter_t *ptrng_adp, hal_rng_st_reg_t *ptrng_st_reg,  hal_rng_reg_t *ptrng_reg)
{
	//dbg_printf("hal_rtl_trng_load_default \n\r ");
	hal_rng_st_reg_t *ptrng_init_st_dat = (hal_rng_st_reg_t *) & (ptrng_st_reg);
	hal_rng_reg_t *ptrng_init_dat = (hal_rng_reg_t *) & (ptrng_reg);
	// trng control

	ptrng_reg->LFSR_mode2 = 0x0;
	ptrng_reg->TRNG_mode = 0x1;
	ptrng_reg->selftest = 0x1;
	ptrng_reg->LFSR_mode = 0x0;
	ptrng_reg->LFSR_bypass1 = 0x0;
	ptrng_reg->TRNG_dbg1_sel = 0x1;
	ptrng_reg->TRNG_dbg0_sel = 0x0;
	ptrng_reg->TRNG_corrector_lmode = 0x0;
	ptrng_reg->corrector_bypass = 0x0;
	ptrng_reg->high_speed_clock = 0x1;
	ptrng_reg->RNG_SRST = 0x0;
	//trng selftest
	ptrng_st_reg->thr_done_adap2 = 0x4;
	ptrng_st_reg->thr_done_adap1 = 0x4;
	ptrng_st_reg->compare_rep = 0x1;
	ptrng_st_reg->compare_unit_adap1 = 0x0;
	ptrng_st_reg->compare_unit_adap2 = 0x0;
	ptrng_st_reg->window_size_adap1 = 0x1;
	ptrng_st_reg->window_size_adap2 = 0x1;
	ptrng_st_reg->thr_err_rep = 0x5;
	ptrng_st_reg->thr_thr_done_rep = 0x0;

	return HAL_OK;

}


/**
 *  \brief To set the default of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]  data
 *
 *  \returns     HAL_OK:  Setting succeed.
 */


SECTION_TRNG_TEXT
void  hal_rtl_trng_reset_patch(hal_trng_adapter_t *ptrng_adp)
{
	//dbg_printf("hal_rtl_trng_reset \n\r ");
	TRNG->TRNG_RNG_CTRL &= (~(TRNG_BIT_RNG_SRST));
}


/**
 *  \brief To set the default of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]  data
 *
 *  \returns     HAL_OK:  Setting succeed.
 */


SECTION_TRNG_TEXT
void  hal_rtl_trng_self_test_setting_patch(hal_trng_adapter_t *ptrng_adp, hal_rng_st_reg_t *ptrng_st_reg)
{
	//dbg_printf("hal_rtl_trng_self_test_setting \r\n");
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING  = ((ptrng_st_reg->thr_done_adap1) << TRNG_SHIFT_THR_DONE_ADAP1); // LFSR_MODE2
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->thr_done_adap2) << TRNG_SHIFT_THR_DONE_ADAP2); // TRNG_mode
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->compare_rep) << TRNG_SHIFT_COMPARE_REP); // selftest
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->compare_unit_adap1) << TRNG_SHIFT_COMPARE_UNIT_ADAP1); // LFSR_bypass1
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->compare_unit_adap2) << TRNG_SHIFT_COMPARE_UNIT_ADAP2); // TRNG_dbg1_sel
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->window_size_adap1) << TRNG_SHIFT_WINDOW_SIZE_ADAP1); // TRNG_dbg0_sel
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->window_size_adap2) << TRNG_SHIFT_WINDOW_SIZE_ADAP2); // TRNG_corrector_lmode
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->thr_err_rep) << TRNG_SHIFT_THR_ERR_REP); // TRNG_corrector_lmode
	ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING |= ((ptrng_st_reg->thr_thr_done_rep) << TRNG_SHIFT_THR_DONE_REP); // TRNG_corrector_lmode
	//dbg_printf("ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING = %x ", ptrng_adp->trng_reg.TRNG_RNG_CTRL);
	TRNG->TRNG_RNG_ST_SETTING = ptrng_adp->trng_reg.TRNG_RNG_ST_SETTING;
	//dbg_printf("hal_rtl_trng_self_test_setting 353 \r\n");


}
/**
 *  \brief To set the interrupt enable of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]   information data
 *
 *  \returns     HAL_OK:  Setting succeed.
 *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
 */


SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_interrupt_reg_patch(hal_trng_adapter_t *ptrng_adp, uint32_t *data)
{
	TRNG->TRNG_RNG_INT_EN = 0XF;
	return HAL_OK;
}


/**
 *  \brief read ready bit of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]   information data
 *
 *  \returns     ready bit
 */


SECTION_TRNG_TEXT
u32 hal_rtl_trng_read_readybit_patch(hal_trng_adapter_t *ptrng_adp)
{
	u32  readybit = 0;
	readybit = ((TRNG->TRNG_RNG_RETURN0) & TRNG_BIT_OUT_READY) >> TRNG_SHIFT_OUT_READY;
	return readybit;
}

/**
 *  \brief error_interrupt bit of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]   information data
 *
 *  \returns     error_interrupt
 */


SECTION_TRNG_TEXT
u32 hal_rtl_trng_read_parity_error_interrupt_patch(hal_trng_adapter_t *ptrng_adp)
{
	u32  error_interrupt = 0;//, st_err = 0, parity = 0;
	error_interrupt = (((TRNG->TRNG_RNG_INT)& TRNG_BIT_PARITY) >> TRNG_SHIFT_PARITY);
	return error_interrupt;
}


/**
 *  \brief clear error_interrupt bit of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]   information data
 *
 *  \returns     HAL_OK:  Setting succeed.
  *  \returns     HAL_ERR_PARA:  Input arguments are invalid.
  */


SECTION_TRNG_TEXT
hal_status_t hal_rtl_trng_clear_interrupt_patch(hal_trng_adapter_t *ptrng_adp)
{
	TRNG->TRNG_RNG_INT &=  ~TRNG_BIT_ST_ERR;
	TRNG->TRNG_RNG_INT &=  ~TRNG_BIT_PARITY;
	return HAL_OK;
}


/**
 *  \brief read data of the TRNG
 *
 *  \param[in] ptrng_adp The TRNG device adapter.

 *   *  @param[in]   information data
 *
 *  \returns     trng data
 */


SECTION_TRNG_TEXT
u32 hal_rtl_trng_read_data_patch(hal_trng_adapter_t *ptrng_adp)
{
	u32  trng_data = 0;
	trng_data = (TRNG->TRNG_RNG_RESULTR);// read data
	return trng_data;
}


#endif

#endif //#if IS_CUT_TEST(CONFIG_CHIP_VER)
