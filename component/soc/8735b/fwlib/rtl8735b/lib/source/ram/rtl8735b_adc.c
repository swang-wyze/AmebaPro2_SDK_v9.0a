/**************************************************************************//**
 * @file     rtl8735b_adc.c
 * @brief    This file implements the ADC HAL functions.
 *
 * @version  V1.00
 * @date     2017-01-17
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
#include "hal_timer.h" // resolve compiler warnings - Using RAM include because rtl8735b_adc.c is now in RAM.
#include "hal_irq.h"
#include "hal_otp.h"

// Removed - Coverity reports that limits.h does not provide needed symbols.
/* #if defined ( __GNUC__ )
#include <limits.h>
#else
#include "__limits.h"
#endif */

#if CONFIG_ADC_EN

#define SECTION_ADC_DATA            //SECTION(".ram.hal_adc.data")
#define SECTION_ADC_RODATA          //SECTION(".ram.hal_adc.rodata")
#define SECTION_ADC_BSS             //SECTION(".ram.hal_adc.bss")
#define SECTION_ADC_TEXT            //SECTION(".ram.hal_adc.text")
#define SECTION_ADC_STUBS           //SECTION(".ram.hal_adc.stubs")

SECTION_ADC_BSS
volatile uint16_t *adc_cv_dat_buf;
SECTION_ADC_BSS
volatile uint32_t adc_cv_dat_len;

/**
 * @addtogroup hs_hal_adc ADC
 * @{
 */

SECTION_ADC_RODATA const hal_pin_map _hp_adc_pins[HP_ADC_CH_NO] = {
	{PIN_F0, PID_ADC0},         //adc0
	{PIN_F1, PID_ADC1},         //adc1
	{PIN_F2, PID_ADC2},         //adc2
	{PIN_F3, PID_ADC3},         //adc3

	{PIN_A0, PID_COMP_ADC},     //adc4 - shared with comparator
	{PIN_A1, PID_COMP_ADC},     //adc5 - shared with comparator
	{PIN_A2, PID_COMP_ADC},     //adc6 - shared with comparator
	{PIN_A3, PID_COMP_ADC},     //adc7 - shared with comparator
};

SECTION_ADC_BSS
volatile hal_adc_adapter_t *adc_irq_adpt;

SECTION_ADC_BSS
volatile hal_adc_cali_para_t adc_cali_paras;

SECTION_ADC_BSS
volatile uint8_t analog_func_en_cnt;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_adc_rom_func ADC HAL ROM APIs.
 * @ingroup hs_hal_adc
 * @{
 * @brief The ADC HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of ADC HAL APIs in the RAM space is provided for the user application.
 */

/**
  * \brief The stubs functions table to exports ADC HAL functions in ROM.
*/
SECTION_ADC_STUBS const hal_adc_func_stubs_t hal_adc_stubs = {
	.hal_adc_pin_list           = _hp_adc_pins,
	.hal_adc_timeout_chk        = hal_rtl_adc_timeout_chk,
	.hal_adc_sw_cv_trig         = hal_rtl_adc_sw_cv_trig,
	.hal_adc_pure_init          = hal_rtl_adc_pure_init,
	.hal_adc_pure_deinit        = hal_rtl_adc_pure_deinit,
	.hal_adc_set_in_type        = hal_rtl_adc_set_in_type,
	.hal_adc_get_in_type        = hal_rtl_adc_get_in_type,
	.hal_adc_set_comp_thld      = hal_rtl_adc_set_comp_thld,
	.hal_adc_set_comp_rule      = hal_rtl_adc_set_comp_rule,
	.hal_adc_clear_comp_intr_sts = hal_rtl_adc_clear_comp_intr_sts,
	.hal_adc_intr_ctrl          = hal_rtl_adc_intr_ctrl,
	.hal_adc_clear_intr_sts     = hal_rtl_adc_clear_intr_sts,
	.hal_adc_set_cvlist         = hal_rtl_adc_set_cvlist,
	.hal_adc_item_to_ch         = hal_rtl_adc_item_to_ch,
	.hal_adc_load_default       = hal_rtl_adc_load_default,
	.hal_adc_reg_irq            = hal_rtl_adc_reg_irq,
	.hal_adc_unreg_irq          = hal_rtl_adc_unreg_irq,
	.hal_adc_init               = hal_rtl_adc_init,
	.hal_adc_deinit             = hal_rtl_adc_deinit,
	.hal_adc_single_read        = hal_rtl_adc_single_read,
	.hal_adc_clear_fifo         = hal_rtl_adc_clear_fifo,
	.hal_adc_read_ch_dat        = hal_rtl_adc_read_ch_dat,
	.hal_adc_read_continuous    = hal_rtl_adc_read_continuous,
	.hal_adc_irq_handler        = hal_rtl_adc_irq_handler,
	.hal_adc_dma_irq_handler    = hal_rtl_adc_dma_irq_handler,
	.phal_adc_irq_adpt          = &adc_irq_adpt,
	.hal_adc_cali_para          = &adc_cali_paras,
	.hal_adc_calc_gain_deno     = hal_rtl_adc_calc_gain_deno,
	.hal_adc_calc_gain_mole     = hal_rtl_adc_calc_gain_mole,
	.hal_adc_calc_offset_deno   = hal_rtl_adc_calc_offset_deno,
	.hal_adc_calc_offset_mole   = hal_rtl_adc_calc_offset_mole,
	.hal_adc_calc_cali_val      = hal_rtl_adc_calc_cali_val,
	.hal_adc_read_cali_param    = hal_rtl_adc_read_cali_param,
	.hal_adc_write_cali_param   = hal_rtl_adc_write_cali_param,
	.hal_adc_read_fifo          = hal_rtl_adc_read_fifo,
};

/** \brief Description of hal_rtl_adc_timeout_chk
 *
 *    hal_rtl_adc_timeout_chk is used to check if adc operation is timeout.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:    Pointer to ADC control adapter.
 *   \param[in] uint32_t start_cnt:                     Timer start count.
 *   \return uint8_t:                                   Timeout or not.\n
 *                                                      0: not time-out. 1: time-out.
 */

uint8_t hal_rtl_adc_timeout_chk(hal_adc_adapter_t *phal_adc_adapter, uint32_t start_cnt)
{
	uint32_t time_out_cnt = 0;
	uint32_t curr_cnt = 0;
	uint32_t expire_cnt = 0;

	if ((phal_adc_adapter->plft_dat.tr_time_out != HP_ADC_TIMEOUT_DISABLE) &&
		(phal_adc_adapter->plft_dat.tr_time_out != HP_ADC_TIMEOUT_ENDLESS)) {
		time_out_cnt = phal_adc_adapter->plft_dat.tr_time_out;
		curr_cnt = hal_read_cur_time();

		if (start_cnt < curr_cnt) {
			expire_cnt =  curr_cnt - start_cnt;//(0xFFFFFFFF - curr_cnt) + start_cnt;
		} else {
			expire_cnt = (0xFFFFFFFF - curr_cnt) + start_cnt;//start_cnt - curr_cnt;
		}

		if (time_out_cnt < expire_cnt) {
			return 1; //time-out
		} else {
			return 0;
		}
	} else if (phal_adc_adapter->plft_dat.tr_time_out == HP_ADC_TIMEOUT_DISABLE) {
		return 1;   //time-out
	}

	return 0;
}


/** \brief Description of hal_rtl_adc_sw_cv_trig
 *
 *    hal_rtl_adc_sw_cv_trig is used to execute ADC software conversion trigger.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:          Pointer to ADC control adapter.
 *   \return void
 */
void hal_rtl_adc_sw_cv_trig(hal_adc_adapter_t *phal_adc_adapter)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	/*if (phal_adc_adapter == NULL) {
	    DBG_ADC_ERR ("hal_rtl_adc_sw_cv_trig: ADC is not initialized\r\n");
	    dbg_printf ("hal_rtl_adc_sw_cv_trig: ADC is not initialized\r\n");
	    return HAL_NOT_READY;
	}*/ // Trying to catch ADC not init error, but not working yet

#if 0
	/* wait for busy end */
	if (phal_adc_adapter->init_dat.use_dly != 0) {
		hal_delay_us(2);
	} else {
		while ((padc_reg->ADC_STS & ADC_BIT_STS_BUSY) == 1) {
		}
	}

	/* execute software trigger */
	padc_reg->ADC_SW_TRIG |= ADC_BIT_SW_TRIG;

	if (phal_adc_adapter->init_dat.use_dly != 0) {
		hal_delay_us(phal_adc_adapter->init_dat.trig_dly);
	} else {
		/* wait busy first */
		while ((padc_reg->ADC_STS & ADC_BIT_STS_BUSY) == 0) {
		}
		/* wait ready */
		while ((padc_reg->ADC_STS & ADC_BIT_STS_BUSY) == 1) {
		}
	}

	padc_reg->ADC_SW_TRIG &= ~(ADC_BIT_SW_TRIG);
#else // Weide try

	padc_reg->ADC_SW_TRIG |= ADC_BIT_SW_TRIG; // Set sw_trig == 1
	hal_delay_us(phal_adc_adapter->init_dat.trig_dly); // 12.5 ADC clocks (1000 us)
	padc_reg->ADC_SW_TRIG &= ~(ADC_BIT_SW_TRIG); // Set sw_trig == 0
	hal_delay_us(phal_adc_adapter->init_dat.trig_dly); // 12.5 ADC clocks (1000 us)

#endif

}

/** \brief Description of hal_rtl_adc_read_fifo
 *
 *    hal_rtl_adc_read_fifo is used to read the fifo register(global) without any trigger operations.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:          Pointer to ADC control adapter.
 *   \param[in] uint8_t out_raw:                              output raw data.
 *   \return uint16_t:                              channel conversion data.
 */
uint16_t hal_rtl_adc_read_fifo(hal_adc_adapter_t *phal_adc_adapter, uint8_t out_raw)
{
	if ((phal_adc_adapter->use_cali) && (out_raw == 0)) {
		DBG_ADC_INFO("Using calibration?: %d, out_raw: %d\r\n", phal_adc_adapter->use_cali, out_raw); // weide added

		return (uint16_t)hal_rtl_adc_calc_cali_val((uint16_t)((phal_adc_adapter->init_dat.reg_base)->ADC_DAT_GLOBAL & ADC_MASK_DAT_GLOBAL_DAT),
				(hal_adc_cali_para_t *)&adc_cali_paras);

	} else {
		return (uint16_t)((phal_adc_adapter->init_dat.reg_base)->ADC_DAT_GLOBAL & ADC_MASK_DAT_GLOBAL_DAT);
	}
}

/** \brief Description of hal_rtl_adc_irq_handler
 *
 *    hal_rtl_adc_irq_handler is used to deal with ADC interrupt events.
 *
 *   \param[in] void
 *   \return void
 */
void hal_rtl_adc_irq_handler(void)
{
	hal_adc_adapter_t *phal_adc_adapter = (hal_adc_adapter_t *)adc_irq_adpt;

	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint16_t adc_dat_tmp16;
	uint16_t *adc_dat_addr;
	uint32_t adc_ch_cnt;
	uint32_t intr_chX_register_data; // Weide added

	uint32_t adc_irq_backup;

	adc_irq_backup = padc_reg->ADC_INTR_STS; // backup ADC IRQ status

	/* The added delay here is due to the a bug that was detected after the test chip was taped out.
	   This bug has been fixed in the MP chip, thus the MP chip should not have any delays here. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
#if CONFIG_FPGA
	hal_delay_us(80);
#elif CONFIG_ASIC
	hal_delay_us(16);
#endif
#endif // end #if IS_CUT_TEST(CONFIG_CHIP_VER) // Test chip only

	padc_reg->ADC_INTR_STS = 0xFFFFFFFF; // clear interrupt status

	/* Clear IRQ first */
	hal_irq_clear_pending(ADC_IRQn);

	if ((adc_irq_backup & (ADC_BIT_IT_FIFO_OVER_STS)) >> ADC_SHIFT_IT_FIFO_OVER_STS) {
		DBG_ADC_ERR("adc ff over intr\r\n");

		padc_reg->ADC_INTR_CTRL &= ~(ADC_BIT_IT_FIFO_OVER_EN); // Disable FF Over Interrupt
		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_FIFO_OVER_STS; // Clear IRQ status

		phal_adc_adapter->err_type = ADCErrorFFOver;
		phal_adc_adapter->status = ADCStatusError;
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.err.cb != NULL) {
			phal_adc_adapter->usr_cb.err.cb((void *)phal_adc_adapter->usr_cb.err.dat);
		}

		phal_adc_adapter->err_type = ADCErrorNone;
		phal_adc_adapter->status = ADCStatusIdle;
	}

	if ((adc_irq_backup & (ADC_BIT_IT_FIFO_FULL_STS)) >> ADC_SHIFT_IT_FIFO_FULL_STS) { // mod here, CHECK ALL!
		DBG_ADC_INFO("adc ff full intr\r\n");

		/* check continuous read flag */
		if (phal_adc_adapter->continuous_rd) {
			if (phal_adc_adapter->use_dma == ADCDisable) {
				//DBG_ADC_INFO("\nlast :%x\n", padc_reg->ADC_LAST_ITEM); // weide temp
				//DBG_ADC_INFO("\nflr :%x\n", padc_reg->ADC_FLR); // weide temp
				while ((phal_adc_adapter->init_dat.reg_base)->ADC_FLR) {
					if (phal_adc_adapter->cv_dat_len) {
						adc_dat_tmp16 = (uint16_t)hal_rtl_adc_read_fifo(phal_adc_adapter, 1);
						//DBG_ADC_INFO("cv_dat_len: %x\r\n", phal_adc_adapter->cv_dat_len); // weide temp
						//DBG_ADC_INFO("\nadc_dat_tmp16 :%x\r\n", adc_dat_tmp16); // weide temp
						*(phal_adc_adapter->cv_dat_buf) = adc_dat_tmp16;
						phal_adc_adapter->cv_dat_buf++;
						phal_adc_adapter->cv_dat_len--;
					}
				}
				if (phal_adc_adapter->cv_dat_len == 0) {
					padc_reg->ADC_CONF &= ~(ADC_BIT_EN); // Disable ADC @ IP level

					/* If it's sw trigger, disable related intr */
					padc_reg->ADC_INTR_CTRL &= ~((ADC_BIT_IT_FIFO_FULL_EN) | (ADC_BIT_IT_FIFO_OVER_EN) | (ADC_BIT_IT_ERR_EN));

					/* If it's auto mode, disable it first */
					if (phal_adc_adapter->init_dat.hw_mode == ADCAutoMod) {
						hal_rtl_adc_auto_chsw_ctrl(phal_adc_adapter, ADCDisable);
					}

					/* If it's timer mode, disable timer first */
					if (phal_adc_adapter->init_dat.hw_mode == ADCTmTrigMod) {
						hal_rtl_timer_disable(phal_adc_adapter->timer_adpt);
					}

					if (phal_adc_adapter->use_cali != 0) {
						adc_dat_addr = (uint16_t *)adc_cv_dat_buf;
						for (adc_ch_cnt = 0; adc_ch_cnt < adc_cv_dat_len; adc_ch_cnt++) {
							adc_dat_tmp16 = *(adc_dat_addr + adc_ch_cnt);
							(*(adc_dat_addr + adc_ch_cnt)) = hal_rtl_adc_calc_cali_val(adc_dat_tmp16,
															 (hal_adc_cali_para_t *)&adc_cali_paras);
						}
					}

					/* invoke user callback */
					if (phal_adc_adapter->usr_cb.rxc.cb != NULL) {
						phal_adc_adapter->usr_cb.rxc.cb((void *)phal_adc_adapter->usr_cb.rxc.dat);
					}

					/* reset list and fifo to the default state */
					phal_adc_adapter->continuous_rd = ADCDisable;
					phal_adc_adapter->status = ADCStatusIdle;
					padc_reg->ADC_CONF &= ~(ADC_MASK_OP_MOD); // cos padc_reg->ADC_CONF = 0 means SW Trigger Mode
					phal_adc_adapter->init_dat.hw_mode = ADCSWTrigMod;
					adc_cv_dat_buf = 0;
					adc_cv_dat_len = 0;
					hal_rtl_adc_reset_list(phal_adc_adapter);
					hal_rtl_adc_clear_fifo(phal_adc_adapter);
					padc_reg->ADC_CONF |= ADC_BIT_EN;

				} else if (phal_adc_adapter->cv_dat_len > 0) {
					/* change full level to 1 when the rest part is less than the threshold */
					if (phal_adc_adapter->cv_dat_len <= (phal_adc_adapter->init_dat.ff_tl + 1)) {
						padc_reg->ADC_FULL_LVL &= ~(ADC_MASK_FULL_LVL); // Reset FIFO Full threshold to 0 (default)
					}

					if (phal_adc_adapter->init_dat.hw_mode == ADCSWTrigMod) {
						hal_rtl_adc_sw_cv_trig(phal_adc_adapter);
					}
				}
			}
		}

		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.full.cb != NULL) {
			phal_adc_adapter->usr_cb.full.cb((void *)phal_adc_adapter->usr_cb.full.dat);
		}

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_FIFO_FULL_STS;
		__DSB();
		__ISB();
	}

	if ((adc_irq_backup & (ADC_BIT_IT_CVLIST_END_STS)) >> ADC_SHIFT_IT_CVLIST_END_STS) {

		DBG_ADC_WARN("adc cvlist end intr\r\n");

		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CVLIST_END_STS;
		__DSB();
		__ISB();

		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.cvlist_end.cb != NULL) {
			phal_adc_adapter->usr_cb.cvlist_end.cb((void *)phal_adc_adapter->usr_cb.cvlist_end.dat);
		}
	}


	if ((adc_irq_backup & (ADC_BIT_IT_CV_END_STS)) >> ADC_SHIFT_IT_CV_END_STS) {

		DBG_ADC_WARN("adc cv end intr\r\n");

		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CV_END_STS;

		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.cv_end.cb != NULL) {
			phal_adc_adapter->usr_cb.cv_end.cb((void *)phal_adc_adapter->usr_cb.cv_end.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_CHCV_END_STS)) >> ADC_SHIFT_IT_CHCV_END_STS) {

		DBG_ADC_WARN("adc chcv end intr\r\n");

		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CHCV_END_STS;

		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch_cv_end.cb != NULL) {
			phal_adc_adapter->usr_cb.ch_cv_end.cb((void *)phal_adc_adapter->usr_cb.ch_cv_end.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_FIFO_EMPTY_STS)) >> ADC_SHIFT_IT_FIFO_EMPTY_STS) {

		DBG_ADC_WARN("adc ff empty intr\r\n");
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_FIFO_EMPTY_STS;

		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.empty.cb != NULL) {
			phal_adc_adapter->usr_cb.empty.cb((void *)phal_adc_adapter->usr_cb.empty.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_DAT_OVW_STS)) >> ADC_SHIFT_IT_DAT_OVW_STS) {

		DBG_ADC_ERR("adc ovw intr\r\n");
		uint32_t intr_chX_base;
		/* ovw check */
		for (adc_ch_cnt = 0; adc_ch_cnt < HP_ADC_CH_NO; adc_ch_cnt++) {

			intr_chX_base = (uint32_t)(&(padc_reg->ADC_DAT_CH0));
			intr_chX_base += (0x04 * adc_ch_cnt); // get the address of respective channel comp threshold register
			intr_chX_register_data = *((uint32_t *)intr_chX_base); // typecast to uint32_t* to make param as an address
			//DBG_ADC_INFO("addr:%x\r\n", (uint32_t *)(&(padc_reg->ADC_DAT_CH0)) + 0x04*adc_ch_cnt); // weide tmp

			if ((intr_chX_register_data & (ADC_BIT_DAT_CH0_OVW)) >> ADC_SHIFT_DAT_CH0_OVW) {

				if (phal_adc_adapter->use_cali != 0) {
					DBG_ADC_ERR("ch(%d) data overwritten, current data: %x\r\n", adc_ch_cnt, hal_rtl_adc_calc_cali_val((uint16_t)(intr_chX_register_data & ADC_MASK_DAT_CH0_DAT),
								(hal_adc_cali_para_t *)&adc_cali_paras));
				} else {
					DBG_ADC_ERR("ch(%d) data overwritten, current data: %x\r\n", adc_ch_cnt, (intr_chX_register_data & (ADC_MASK_DAT_CH0_DAT)));
				}

			}
		}

		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_DAT_OVW_STS;

		phal_adc_adapter->err_type = ADCErrorDatOverWrite;
		phal_adc_adapter->status = ADCStatusError;
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.err.cb != NULL) {
			phal_adc_adapter->usr_cb.err.cb((void *)phal_adc_adapter->usr_cb.err.dat);
		}

		phal_adc_adapter->err_type = ADCErrorNone;
		phal_adc_adapter->status = ADCStatusIdle;
	}

	if ((adc_irq_backup & (ADC_BIT_IT_ERR_STS)) >> ADC_SHIFT_IT_ERR_STS) {

		/* clear status */
		DBG_ADC_ERR("ADC error intr\r\n");
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_ERR_STS;

		phal_adc_adapter->err_type = ADCErrorUnknown;
		phal_adc_adapter->status = ADCStatusError;
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.err.cb != NULL) {
			phal_adc_adapter->usr_cb.err.cb((void *)phal_adc_adapter->usr_cb.err.dat);
		}

		phal_adc_adapter->err_type = ADCErrorNone;
		phal_adc_adapter->status = ADCStatusIdle;
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH0_STS)) >> ADC_SHIFT_IT_COMP_CH0_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH0_STS;
		DBG_ADC_INFO("comp 0, sts: %x\r\n", padc_reg->ADC_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch0_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch0_comp.cb((void *)phal_adc_adapter->usr_cb.ch0_comp.dat);
		}
	}


	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH1_STS)) >> ADC_SHIFT_IT_COMP_CH1_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH1_STS;
		DBG_ADC_INFO("comp 1, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH1_COMP_STS) >> ADC_SHIFT_CH1_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch1_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch1_comp.cb((void *)phal_adc_adapter->usr_cb.ch1_comp.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH2_STS)) >> ADC_SHIFT_IT_COMP_CH2_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH2_STS;
		DBG_ADC_INFO("comp 2, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH2_COMP_STS) >> ADC_SHIFT_CH2_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch2_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch2_comp.cb((void *)phal_adc_adapter->usr_cb.ch2_comp.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH3_STS)) >> ADC_SHIFT_IT_COMP_CH3_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH3_STS;
		DBG_ADC_INFO("comp 3, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH3_COMP_STS) >> ADC_SHIFT_CH3_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch3_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch3_comp.cb((void *)phal_adc_adapter->usr_cb.ch3_comp.dat);
		}
	}


	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH4_STS)) >> ADC_SHIFT_IT_COMP_CH4_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH4_STS;
		DBG_ADC_INFO("comp 4, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH4_COMP_STS) >> ADC_SHIFT_CH4_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch4_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch4_comp.cb((void *)phal_adc_adapter->usr_cb.ch4_comp.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH5_STS)) >> ADC_SHIFT_IT_COMP_CH5_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH5_STS;
		DBG_ADC_INFO("comp 5, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH5_COMP_STS) >> ADC_SHIFT_CH5_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch5_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch5_comp.cb((void *)phal_adc_adapter->usr_cb.ch5_comp.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH6_STS)) >> ADC_SHIFT_IT_COMP_CH6_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH6_STS;
		DBG_ADC_INFO("comp 6, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH6_COMP_STS) >> ADC_SHIFT_CH6_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch6_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch6_comp.cb((void *)phal_adc_adapter->usr_cb.ch6_comp.dat);
		}
	}

	if ((adc_irq_backup & (ADC_BIT_IT_COMP_CH7_STS)) >> ADC_SHIFT_IT_COMP_CH7_STS) {

		/* clear status */
		padc_reg->ADC_INTR_STS |= ADC_BIT_IT_COMP_CH7_STS;
		DBG_ADC_INFO("comp 7, sts: %x\r\n", ((padc_reg->ADC_COMP_STS) & ADC_MASK_CH7_COMP_STS) >> ADC_SHIFT_CH7_COMP_STS);
		/* invoke user callback */
		if (phal_adc_adapter->usr_cb.ch7_comp.cb != NULL) {
			phal_adc_adapter->usr_cb.ch7_comp.cb((void *)phal_adc_adapter->usr_cb.ch7_comp.dat);
		}
	}

}

/** \brief Description of hal_rtl_adc_dma_irq_handler
 *
 *    hal_rtl_adc_dma_irq_handler is used to deal with ADC DMA interrupt events.
 *
 *   \param[in] void
 *   \return void
 */
void hal_rtl_adc_dma_irq_handler(void)
{
	hal_adc_adapter_t *phal_adc_adapter = (hal_adc_adapter_t *)adc_irq_adpt;
	uint16_t *adc_dat_addr;
	uint16_t adc_dat_tmp;
	uint32_t adc_dat_cnt;
	uint32_t adc_dat_len;

	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	DBG_ADC_INFO("adc dma intr >>\r\n");

	/* stop auto operation first if auto control is enabled. */
	if (padc_reg->ADC_AUTO_CHSW_CTRL & (ADC_BIT_AUTO_CHSW_EN)) {
		hal_rtl_adc_auto_chsw_ctrl(phal_adc_adapter, ADCDisable);
	}

	/* disable ADC DMA control and set operation mode to SW mode. */
	padc_reg->ADC_DMA_CON &= ~(ADC_BIT_DMA_CON_EN); // Disable DMA function of ADC
	padc_reg->ADC_CONF &= ~(ADC_MASK_OP_MOD | ADC_BIT_EN); // Disable ADC after setting op_mod to software trigger mode by clearing op_mod

	/* This part is done by GDMA HAL */
	//hal_gdma_clean_chnl_isr_rtl8735b((void*)(phal_adc_adapter->dma_dat.padaptor));
	//hal_gdma_chnl_dis_rtl8735b((void*)(phal_adc_adapter->dma_dat.padaptor));

	/* disable interrupt */
	DBG_ADC_INFO("intr ctrl:%x\r\n", padc_reg->ADC_INTR_CTRL);
	DBG_ADC_INFO("intr state:%x\r\n", padc_reg->ADC_INTR_STS);

	padc_reg->ADC_INTR_CTRL &= ~(ADC_BIT_IT_ERR_EN | ADC_BIT_IT_FIFO_OVER_EN); // Disable FIFO overflow and error interrupts
	DBG_ADC_INFO("intr ctrl:%x\r\n", padc_reg->ADC_INTR_CTRL);
	/* reset list and fifo state to the default state */
	hal_rtl_adc_reset_list(phal_adc_adapter);
	hal_rtl_adc_clear_fifo(phal_adc_adapter);

	padc_reg->ADC_CONF |= ADC_BIT_EN;

	/* clear interrupt state all */
	padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CVLIST_END_STS | ADC_BIT_IT_CV_END_STS | ADC_BIT_IT_CHCV_END_STS |
							  ADC_BIT_IT_FIFO_FULL_STS | ADC_BIT_IT_FIFO_OVER_STS | ADC_BIT_IT_FIFO_EMPTY_STS |
							  ADC_BIT_IT_DAT_OVW_STS | ADC_BIT_IT_ERR_STS | ADC_BIT_IT_COMP_CH0_STS |
							  ADC_BIT_IT_COMP_CH1_STS | ADC_BIT_IT_COMP_CH2_STS | ADC_BIT_IT_COMP_CH3_STS |
							  ADC_BIT_IT_COMP_CH4_STS | ADC_BIT_IT_COMP_CH5_STS | ADC_BIT_IT_COMP_CH6_STS |
							  ADC_BIT_IT_COMP_CH7_STS | ADC_BIT_IT_COMP_CH8_STS;

	/* deal with d-cache sync. (invalidate) */
	if (is_dcache_enabled()) {
		if (phal_adc_adapter->dcache_invalidate_by_addr != NULL) {
			phal_adc_adapter->dcache_invalidate_by_addr((uint32_t *)phal_adc_adapter->cv_dat_buf, (int32_t)phal_adc_adapter->cv_dat_len);
		} else {
			DBG_ADC_ERR("D-Cache is enabled but invalidate function is NOT available.\r\n");
		}
	}

	if (phal_adc_adapter->dma_dat.padaptor->gdma_ctl.src_tr_width == TrWidthFourBytes) {
		adc_dat_len = phal_adc_adapter->cv_dat_len / 4;
	} else {
		adc_dat_len = phal_adc_adapter->cv_dat_len / 2;
	}

	if (phal_adc_adapter->use_cali != 0) {
		adc_dat_addr = phal_adc_adapter->cv_dat_buf;
		for (adc_dat_cnt = 0; adc_dat_cnt < adc_dat_len; adc_dat_cnt++) {
			adc_dat_tmp = *(adc_dat_addr + adc_dat_cnt);
			*(adc_dat_addr + adc_dat_cnt) = hal_rtl_adc_calc_cali_val(adc_dat_tmp, (hal_adc_cali_para_t *)&adc_cali_paras);
		}
	}

	/* invoke user callback if available */
	if (phal_adc_adapter->usr_cb.rxc.cb != NULL) {
		phal_adc_adapter->usr_cb.rxc.cb((void *)phal_adc_adapter->usr_cb.rxc.dat);
	}

	/* update software status */
	phal_adc_adapter->status = ADCStatusIdle;
	//phal_adc_adapter->dma_dat.ch_sts = ADCDmaChInitialed;
	phal_adc_adapter->use_dma = ADCDisable;
	DBG_ADC_INFO("adc dma intr <<\r\n");
}

/** \brief Description of hal_rtl_adc_pure_init
 *
 *    hal_rtl_adc_pure_init is used to initialize ADC module.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:       Pointer to ADC control adapter.
 *   \return hal_status_t
 */
hal_status_t hal_rtl_adc_pure_init(hal_adc_adapter_t *phal_adc_adapter)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base; // Let C struct have base address of ADC

	DBG_ADC_INFO("adc register base: %x\r\n", padc_reg);
	DBG_ADC_INFO("flr :%x\r\n", padc_reg->ADC_FLR);

	/* basic configuration */
	padc_reg->ADC_CONF |= (phal_adc_adapter->init_dat.cvlist_len << ADC_SHIFT_CVLIST_LEN) |
						  (phal_adc_adapter->init_dat.hw_mode << ADC_SHIFT_OP_MOD) |
						  (phal_adc_adapter->init_dat.ref_lvl << ADC_SHIFT_REF_IN_SEL) |
						  (ADC_BIT_EN);

	DBG_ADC_INFO("conf: %x\r\n", padc_reg->ADC_CONF);

	/* set input type to single end type in default state (Unify) */
	padc_reg->ADC_IN_TYPE |= HP_ADC_INPUT_ALL_SINGLE;
	padc_reg->ADC_IT_CHNO_CON |= phal_adc_adapter->init_dat.it_chno_cv << ADC_SHIFT_IT_CHNO_CON;
	padc_reg->ADC_FULL_LVL |= phal_adc_adapter->init_dat.ff_tl << ADC_SHIFT_FULL_LVL;
	padc_reg->ADC_TRIG_TIMER_SEL |= phal_adc_adapter->init_dat.trig_timer << ADC_SHIFT_TRIG_TIMER_SEL;
	padc_reg->ADC_CLK_DIV |= phal_adc_adapter->init_dat.clock_div << ADC_SHIFT_CLK_DIV;
	padc_reg->ADC_INTR_CTRL = 0; // assign value of 0 to register
	padc_reg->ADC_VREF_SETTLE_TIME = phal_adc_adapter->init_dat.vref_settle_time; // assign value straight to register

	DBG_ADC_INFO("full_lvl: %x\r\n", padc_reg->ADC_FULL_LVL);
	DBG_ADC_INFO("clk_div: %x\r\n", padc_reg->ADC_CLK_DIV);

	/* clear intr status */
	padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CVLIST_END_STS | ADC_BIT_IT_CV_END_STS | ADC_BIT_IT_CHCV_END_STS |
							  ADC_BIT_IT_FIFO_FULL_STS | ADC_BIT_IT_FIFO_OVER_STS | ADC_BIT_IT_FIFO_EMPTY_STS |
							  ADC_BIT_IT_DAT_OVW_STS | ADC_BIT_IT_ERR_STS | ADC_BIT_IT_COMP_CH0_STS |
							  ADC_BIT_IT_COMP_CH1_STS | ADC_BIT_IT_COMP_CH2_STS | ADC_BIT_IT_COMP_CH3_STS |
							  ADC_BIT_IT_COMP_CH4_STS | ADC_BIT_IT_COMP_CH5_STS | ADC_BIT_IT_COMP_CH6_STS |
							  ADC_BIT_IT_COMP_CH7_STS | ADC_BIT_IT_COMP_CH8_STS;

	/* update enable state */
	phal_adc_adapter->init_dat.enable = ADCEnable;

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_pure_deinit
 *
 *    hal_rtl_adc_pure_deinit is used to de-initialize ADC module.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:       Pointer to ADC control adapter.
 *   \return hal_status_t
 */
hal_status_t hal_rtl_adc_pure_deinit(hal_adc_adapter_t *phal_adc_adapter)
{

	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	DBG_ADC_INFO("adc register base: %x\r\n", padc_reg);

	padc_reg->ADC_CONF &= ~(ADC_BIT_EN | ADC_MASK_OP_MOD);
	padc_reg->ADC_IN_TYPE = HP_ADC_INPUT_ALL_SINGLE;

	/* update enable state */
	phal_adc_adapter->init_dat.enable = ADCDisable;

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_set_in_type
 *
 *    hal_rtl_adc_set_in_type is used to set ADC channel input type.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                      ADC channel number.
 *   \param[in] uint8_t input_type:                 Input type.
 *   \return hal_status_t:          return HAL_OK when the given parameters is correct.
 *                                  Only ch0~ch7 are capable of two input type (single-end and differential)
 */
hal_status_t hal_rtl_adc_set_in_type(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no, uint8_t input_type)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	/* check channel number */
	if (ch_no > HP_ADC_DUAL_IN_CH) {
		return HAL_ERR_PARA;
	} else {
		if (input_type) { // set to differential mode
			if (ch_no % 2 == 1) { // channel is odd
				padc_reg->ADC_IN_TYPE |= (1 << ch_no) | (1 << (ch_no - 1)); // because ch0~1 are pairs, ch2~3 are pairs, etc
			} else { // channel is even
				padc_reg->ADC_IN_TYPE |= (1 << ch_no) | (1 << (ch_no + 1)); // because ch0~1 are pairs, ch2~3 are pairs, etc
			}
		} else { // set to single-ended mode
			if (ch_no % 2 == 1) { // channel is odd
				padc_reg->ADC_IN_TYPE &= ~((1 << ch_no) | (1 << (ch_no - 1))); // because ch0~1 are pairs, ch2~3 are pairs, etc
			} else { // channel is even
				padc_reg->ADC_IN_TYPE &= ~((1 << ch_no) | (1 << (ch_no + 1))); // because ch0~1 are pairs, ch2~3 are pairs, etc
			}
		}
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_get_in_type
 *
 *    hal_rtl_adc_get_in_type is used to set ADC channel input type.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:        Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                              ADC channel number.
 *   \return uint8_t:                               channel input type.\n
 *                                                  0: single end, 1: differential.
 */
uint8_t hal_rtl_adc_get_in_type(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no)
{
	return (uint8_t)((((phal_adc_adapter->init_dat.reg_base)->ADC_IN_TYPE) & (BIT0 << ch_no)) >> ch_no); // BIT0 is retained - Weide
}

/** \brief Description of hal_rtl_adc_set_comp_thld
 *
 *    hal_rtl_adc_set_comp_thld is used to set ADC channel comparison threshold.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                      ADC channel number.
 *   \param[in] uint16_t thld_high:                 Comparison high threshold.
 *   \param[in] uint16_t thld_low:                  Comparison low threshold.
 *   \return hal_status_t
 */
hal_status_t hal_rtl_adc_set_comp_thld(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no, uint16_t thld_high, uint16_t thld_low)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t comp_th_chX_base;
	uint32_t *comp_th_chX_real_addr;

	if (hal_rtl_adc_chk_chidx(ch_no) != HAL_OK) {
		return HAL_ERR_PARA;
	}

	comp_th_chX_base = (uint32_t)(&(padc_reg->ADC_COMP_TH_CH0));
	comp_th_chX_base += (0x04 * ch_no); // get the address of respective channel comp threshold register
	comp_th_chX_real_addr = (uint32_t *)comp_th_chX_base; // typecast to uint32_t* to make param as an address
	*comp_th_chX_real_addr |= ((thld_high << ADC_SHIFT_CH0_TH_H) | (thld_low << ADC_SHIFT_CH0_TH_L));

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_set_comp_rule
 *
 *    hal_rtl_adc_set_comp_rule is used to set ADC channel comparison control. When this is set to a particular
 *    criterion, the related comparison status and interrupt would be triggered.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                      ADC channel number.
 *   \param[in] uint8_t comp_rule:                  Comparison rule. // maybe state what the comp_rules are - Weide
 *   \return hal_status_t
 */

hal_status_t hal_rtl_adc_set_comp_rule(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no, uint8_t comp_rule)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	if (hal_rtl_adc_chk_chidx(ch_no) != HAL_OK) {
		return HAL_ERR_PARA;
	}

	padc_reg->ADC_COMP_CTRL &= ~(0x3 << (2 * ch_no));
	padc_reg->ADC_COMP_CTRL |= ((comp_rule & 0x03) << (2 * ch_no));

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_clear_comp_intr_sts
 *
 *    hal_rtl_adc_clear_comp_intr_sts is used to clear adc comparison interrupt status.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                      ADC channel number.
 *   \return void
 */
void hal_rtl_adc_clear_comp_intr_sts(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	padc_reg->ADC_INTR_STS |= (1 << (0x08 + ch_no));
}

/** \brief Description of hal_rtl_adc_intr_ctrl
 *
 *    hal_rtl_adc_intr_ctrl is used to set ADC channel comparison control. When this is set to a particular
 *    criterion, the related comparison status and interrupt would be triggered.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t intr_option:                Interrupt option (source).\n
 *                                              0: cvlist end.          1: cv end.\n
 *                                              2: channel cv end.      3: fifo full.\n
 *                                              4: fifo over.           5: fifo empty.\n
 *                                              6: dat ovw.             7: err.
 *
 *   \param[in] uint8_t intr_enable:                Interrupt enable control.
 *   \return void
 */
void hal_rtl_adc_intr_ctrl(hal_adc_adapter_t *phal_adc_adapter, uint8_t intr_option, uint8_t intr_enable)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	DBG_ADC_INFO("intr ctrl: %08x\r\n", padc_reg->ADC_INTR_CTRL);
	padc_reg->ADC_INTR_CTRL &= ~(1 << intr_option);
	padc_reg->ADC_INTR_CTRL |= ((intr_enable & 0x1) << intr_option);
}

/** \brief Description of hal_rtl_adc_clear_intr_sts
 *
 *    hal_rtl_adc_clear_intr_sts is used to clear adc interrupt interrupt status.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t intr_option:                Interrupt option (source).\n
 *                                              0: cvlist end.          1: cv end.\n
 *                                              2: channel cv end.      3: fifo full.\n
 *                                              4: fifo over.           5: fifo empty.\n
 *                                              6: dat ovw.             7: err.
 *   \return void
 */
void hal_rtl_adc_clear_intr_sts(hal_adc_adapter_t *phal_adc_adapter, uint8_t intr_option)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;

	padc_reg->ADC_INTR_STS |= (0x01 << intr_option);
}

/** \brief Description of hal_rtl_adc_set_cvlist
 *
 *    hal_rtl_adc_set_cvlist is used to set ADC channel conversion list.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t *cvlist:                    Pointer to a channel conversion list. It should be an array start address.
 *   \param[in] uint8_t cvlist_len:                 Conversion list length.
 *   \return hal_status_t
 */
hal_status_t hal_rtl_adc_set_cvlist(hal_adc_adapter_t *phal_adc_adapter, uint8_t *cvlist, uint8_t cvlist_len)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t chsw0_offset = 0;
	uint32_t chsw1_offset = 0;
	uint8_t adc_cnt_tmp;
	uint8_t adc_cv_len;

	phal_adc_adapter->init_dat.cvlist_len = cvlist_len - 1;
	adc_cv_len = cvlist_len;

	if (adc_cv_len > HP_ADC_LIST_HALF * 2) {
		DBG_ADC_ERR("cv_len is larger than hardware len.\r\n");
		return HAL_ERR_PARA;
	}

	/* reset list to the default state */
	hal_rtl_adc_reset_list(phal_adc_adapter);
	hal_rtl_adc_clear_fifo(phal_adc_adapter);

	DBG_ADC_INFO("adc_cv_len: %x\r\n", phal_adc_adapter->init_dat.cvlist_len);
	padc_reg->ADC_CONF &= ~(ADC_MASK_CVLIST_LEN);
	padc_reg->ADC_CONF |= (phal_adc_adapter->init_dat.cvlist_len << ADC_SHIFT_CVLIST_LEN);
	DBG_ADC_INFO("set cvlist: %x\r\n", padc_reg->ADC_CONF);

	/* Assign list_0 */
	for (adc_cnt_tmp = HP_ADC_LIST_HALF;
		 (adc_cv_len > 0) && (adc_cnt_tmp > 0); adc_cv_len--, adc_cnt_tmp--) {

		padc_reg->ADC_CHSW_LIST_0 &= ~((0xF) << (ADC_SHIFT_CHSW_0 + chsw0_offset));
		padc_reg->ADC_CHSW_LIST_0 |= (*(cvlist) << (ADC_SHIFT_CHSW_0 + chsw0_offset));
		chsw0_offset += 4;
		cvlist++;
	}

	DBG_ADC_INFO("chsw_list_0: %08x\r\n", padc_reg->ADC_CHSW_LIST_0);

	/* Assign list_1 if it's necessary */
	if (adc_cv_len > 0) {

		for (adc_cnt_tmp = HP_ADC_LIST_HALF;
			 (adc_cv_len > 0) && (adc_cnt_tmp > 0); adc_cv_len--, adc_cnt_tmp--) {

			padc_reg->ADC_CHSW_LIST_1 &= ~((0xF) << (ADC_SHIFT_CHSW_8 + chsw1_offset));
			padc_reg->ADC_CHSW_LIST_1 |= (*(cvlist) << (ADC_SHIFT_CHSW_8 + chsw1_offset));
			chsw1_offset += 4;
			cvlist++;

		}

		DBG_ADC_INFO("chsw_list_1: %08x\r\n", padc_reg->ADC_CHSW_LIST_1);

	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_item_to_ch
 *
 *    hal_rtl_adc_item_to_ch is used to transfer a list item to channel number.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adapter.
 *   \param[in] uint8_t item_no:                    Item number in the hardware list register.
 *   \return uint8_t:                           Channel number of the list item.
 */
uint8_t hal_rtl_adc_item_to_ch(hal_adc_adapter_t *phal_adc_adapter, uint8_t item_no)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t channel_number;

	if (item_no > (HP_ADC_LIST_LEN - 1)) {
		DBG_ADC_ERR("itme number is larger than max value.\r\n");
		return 0xFF;
	}

	// Technique: Mask out the number in the cvlist according to offset, then shift that number to Position 0
	if (item_no < 8) {
		// 0th to 7th channel
		channel_number = (padc_reg->ADC_CHSW_LIST_0 & (0xF << (item_no * 4))) >> (item_no * 4);
	} else {
		// 8th to 15th channel
		channel_number = (padc_reg->ADC_CHSW_LIST_1 & (0xF << ((item_no - 8) * 4))) >> ((item_no - 8) * 4); // weide modified item_no-8 instead of item_no
	}
	return (uint8_t)channel_number;

}

/** \brief Description of hal_rtl_adc_load_default
 *
 *    hal_rtl_adc_load_default is used to load default value for data structure.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:        Pointer to ADC control adaptor.
 *   \return void
 */
void hal_rtl_adc_load_default(hal_adc_adapter_t *phal_adc_adapter)
{
	adc_init_dat_t *padc_init_dat = (adc_init_dat_t *) & (phal_adc_adapter->init_dat);

	/* ADC Initial Default Configuration */
	padc_init_dat->enable   = ADCDisable;
	padc_init_dat->hw_mode  = ADCSWTrigMod;
	padc_init_dat->cvlist_len = 0;              /* 0 is that total length of cvlist is one  */
	padc_init_dat->it_chno_cv = 0;
	padc_init_dat->ff_tl     = 0;               /* 0 is one entry to trigger FIFO full interrupt */
	padc_init_dat->trig_timer = 0;              /* Select Timer Channel 0 as trigger source  */
	padc_init_dat->clock_div = ADCClockDiv640;  /* ADC SCLK (input clock) Divider */
	padc_init_dat->dma_rq_lv = 4;               /* ADC DMA Level to Trigger GDMA */
	padc_init_dat->ref_lvl  = 0;                /* ADC Voltage Reference Level */
#if !defined (CONFIG_BUILD_NONSECURE)
	padc_init_dat->reg_base = (ADC_TypeDef *)ADC_S_BASE; /* ADC Secure Base Address */
#else
	padc_init_dat->reg_base = (ADC_TypeDef *)ADC_BASE; /* ADC Non-Secure Base Address */
#endif
	padc_init_dat->use_dly  = 1;                /* ADC trigger delay enable */ // may be defunct
	padc_init_dat->trig_dly = 1000;             /* ADC trigger delay: DD: at least 5 ADC clk cycles */ // must split into FPGA vs ASIC!!
	padc_init_dat->vref_settle_time = ADCVRefSettleTime20;  /* ADC Reference Voltage Settle Time*/
	phal_adc_adapter->use_cali = 1;             /* ADC Use-Calibration Flag */


	/* ADC Platform Level Configuration */

	phal_adc_adapter->dma_dat.ch_sts = ADCDmaChNone;

	/* clear USER Callback Function */
	memset(&phal_adc_adapter->usr_cb, 0x00, sizeof(adc_user_callback_t));

	/* ADC Platform Configuration */
	phal_adc_adapter->plft_dat.tr_time_out = 60000000;
	phal_adc_adapter->dma_dat.padaptor = NULL;

	/* Update status */
	phal_adc_adapter->err_type = ADCErrorNone;
	phal_adc_adapter->status   = ADCStatusUninitial;
	phal_adc_adapter->cv_dat_len = 0;

}

/** \brief Description of hal_rtl_adc_reg_irq
 *
 *    hal_rtl_adc_reg_irq is used to register a irq handler
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:  Pointer to ADC control adaptor.
 *   \param[in] irq_handler_t handler:                IRQ handler function.
 *   \return void
 */
void hal_rtl_adc_reg_irq(hal_adc_adapter_t *phal_adc_adapter, irq_handler_t handler)
{
	// IRQ vector may have been registered, disable and re-register it
	hal_irq_disable(ADC_IRQn);
	__ISB();
	hal_irq_set_vector(ADC_IRQn, (uint32_t)handler);
	hal_irq_enable(ADC_IRQn);
}

/** \brief Description of hal_rtl_adc_unreg_irq
 *
 *    hal_rtl_adc_unreg_irq is used to unregister a irq handler
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:        Pointer to ADC control adaptor.
 *   \return void
 */
void hal_rtl_adc_unreg_irq(hal_adc_adapter_t *phal_adc_adapter)
{
	hal_irq_disable(ADC_IRQn);
	__ISB();
	hal_irq_set_vector(ADC_IRQn, (uint32_t)NULL);
}

/** \brief Description of hal_rtl_adc_init
 *
 *    hal_rtl_adc_init is used for adc initialization with power state.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:        Pointer to ADC control adaptor.
 *   \return hal_status_t.
 */
hal_status_t hal_rtl_adc_init(hal_adc_adapter_t *phal_adc_adapter)
{
	hal_status_t retv;

	adc_irq_adpt = phal_adc_adapter; // don't understand why???
	hal_rtl_adc_reg_irq(phal_adc_adapter, (irq_handler_t) hal_rtl_adc_irq_handler);
	hal_irq_set_priority(ADC_IRQn, ADC_IRQPri);

	retv = hal_rtl_adc_pure_init(phal_adc_adapter);

	/* read calibration params */
	if (phal_adc_adapter->use_cali != 0) {
		DBG_ADC_INFO("read cali param\r\n");
		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_GAIN_DENO_ADDR, (uint8_t *)(&(adc_cali_paras.gain_deno)), sizeof(adc_cali_paras.gain_deno));
		DBG_ADC_INFO("gain deno: %x\r\n", adc_cali_paras.gain_deno);
		if ((uint16_t)adc_cali_paras.gain_deno == 0xFFFF) {
			DBG_ADC_ERR("Read gain deno failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}
		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_GAIN_MOLE_ADDR, (uint8_t *)(&(adc_cali_paras.gain_mole)), sizeof(adc_cali_paras.gain_mole));
		DBG_ADC_INFO("gain mole: %x\r\n", adc_cali_paras.gain_mole);
		if ((uint16_t)adc_cali_paras.gain_mole == 0xFFFF) {
			DBG_ADC_ERR("Read gain mole failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}

		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_OFFSET_DENO_ADDR, (uint8_t *)(&(adc_cali_paras.offset_deno)), sizeof(adc_cali_paras.offset_deno));
		DBG_ADC_INFO("offset deno: %x\r\n", adc_cali_paras.offset_deno);
		if ((uint32_t)adc_cali_paras.offset_deno == 0xFFFFFFFF) {
			DBG_ADC_ERR("Read offset deno failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}
		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_OFFSET_MOLE_ADDR, (uint8_t *)(&(adc_cali_paras.offset_mole)), sizeof(adc_cali_paras.offset_mole));
		DBG_ADC_INFO("offset mole: %x\r\n", adc_cali_paras.offset_mole);
		if ((uint16_t)adc_cali_paras.offset_mole == 0xFFFF) {
			DBG_ADC_ERR("Read offset mole failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}

		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_BATT_GAIN_DENO_ADDR, (uint8_t *)(&(adc_cali_paras.batt_gain_deno)), sizeof(adc_cali_paras.batt_gain_deno));
		DBG_ADC_INFO("batt gain deno: %x\n", adc_cali_paras.batt_gain_deno);
		if ((uint16_t)adc_cali_paras.batt_gain_deno == 0xFFFF) {
			DBG_ADC_ERR("Read battery gain deno failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}
		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_BATT_GAIN_MOLE_ADDR, (uint8_t *)(&(adc_cali_paras.batt_gain_mole)), sizeof(adc_cali_paras.batt_gain_mole));
		DBG_ADC_INFO("batt gain mole: %x\n", adc_cali_paras.batt_gain_mole);
		if ((uint16_t)adc_cali_paras.batt_gain_mole == 0xFFFF) {
			DBG_ADC_ERR("Read battery gain mole failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}

		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_BATT_OFFSET_DENO_ADDR, (uint8_t *)(&(adc_cali_paras.batt_offset_deno)), sizeof(adc_cali_paras.batt_offset_deno));
		DBG_ADC_INFO("batt offset deno: %x\n", adc_cali_paras.batt_offset_deno);
		if ((uint32_t)adc_cali_paras.batt_offset_deno == 0xFFFF) {
			DBG_ADC_ERR("Read battery offset deno failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}
		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_BATT_OFFSET_MOLE_ADDR, (uint8_t *)(&(adc_cali_paras.batt_offset_mole)), sizeof(adc_cali_paras.batt_offset_mole));
		DBG_ADC_INFO("batt offset mole: %x\n", adc_cali_paras.batt_offset_mole);
		if ((uint16_t)adc_cali_paras.batt_offset_mole == 0xFFFF) {
			DBG_ADC_ERR("Read battery offset mole failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}

		hal_rtl_adc_read_cali_param((uint16_t)HP_ADC_BATT_INTERNAL_R_ADDR, (uint8_t *)(&(adc_cali_paras.batt_internal_r)), sizeof(adc_cali_paras.batt_internal_r));
		DBG_ADC_INFO("batt internal R mole: %x\n", adc_cali_paras.batt_internal_r);
		if ((uint16_t)adc_cali_paras.batt_internal_r == 0xFFFF) {
			DBG_ADC_ERR("Read battery internal R failed\r\n");
			DBG_ADC_ERR("Not to use calibration.\r\n");
			phal_adc_adapter->use_cali = 0;
		}
	}

	hal_rtl_adc_clear_fifo(phal_adc_adapter);
	phal_adc_adapter->status = ADCStatusIdle;

	return retv;
}

/** \brief Description of hal_rtl_adc_deinit
 *
 *    hal_rtl_adc_deinit is used for adc de-initialization with power state.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:        Pointer to ADC control adaptor.
 *   \return hal_status_t.
 */
hal_status_t hal_rtl_adc_deinit(hal_adc_adapter_t *phal_adc_adapter)
{
	hal_rtl_adc_pure_deinit(phal_adc_adapter);

	hal_rtl_adc_unreg_irq(phal_adc_adapter);

	adc_irq_adpt = NULL;
	memset((void *)&adc_cali_paras, 0x00, sizeof(adc_cali_paras));
	phal_adc_adapter->status     = ADCStatusUninitial;
	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_single_read
 *
 *    hal_rtl_adc_single_read is used for single for particular channel.
 *    This function would automatically execute software trigger flow and return the channel data.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:      Pointer to ADC control adaptor.
 *   \param[in] uint8_t ch_no:                      ADC channel number.
 *   \return uint16_t:            ADC sample data.
 */
uint16_t hal_rtl_adc_single_read(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t adc_config_backup;
	uint32_t chsw_list_0_backup;
	uint32_t chX_register_addr;
	uint32_t chX_register_data;

	DBG_ADC_INFO("single read >>\r\n");

	/* check ADC is in progress or not */
	if (phal_adc_adapter->status != ADCStatusIdle) {
		DBG_ADC_WARN("Hardware is NOT in idle state\r\n");
		if (phal_adc_adapter->status == ADCStatusCVing) {
			DBG_ADC_WARN("A conversion is in progress\r\n");
		}
		return 0xFFFF;
	}

	/* backup adc control since it would be switched to sw-trig in this func. */
	adc_config_backup = padc_reg->ADC_CONF;

	/* save list 0 */
	chsw_list_0_backup = padc_reg->ADC_CHSW_LIST_0;

	/* to sw-trig */
	padc_reg->ADC_CONF &= ~(ADC_MASK_OP_MOD); // since SW Trig mode is 0

	/* reset list and clear fifo to the default state */
	hal_rtl_adc_reset_list(phal_adc_adapter);
	hal_rtl_adc_clear_fifo(phal_adc_adapter);

	//dbg_printf("ch_no:%x\r\n", ch_no); // weide temp

	padc_reg->ADC_CHSW_LIST_0 &= ~(0xF << ADC_SHIFT_CHSW_0);
	padc_reg->ADC_CHSW_LIST_0 |= (ch_no << ADC_SHIFT_CHSW_0);
	DBG_ADC_INFO("padc_reg->chsw_list_0 is %x\r\n", padc_reg->ADC_CHSW_LIST_0); // weide temp

	hal_rtl_adc_sw_cv_trig(phal_adc_adapter);

	phal_adc_adapter->status = ADCStatusCVing;

	chX_register_addr = (uint32_t) & (padc_reg->ADC_DAT_CH0) + (0x4 * ch_no);
	DBG_ADC_INFO("adc dat addr: %08x\r\n", chX_register_addr);
	chX_register_data = *((uint32_t *)chX_register_addr);

	DBG_ADC_INFO("padc_reg->conf is %x\r\n", padc_reg->ADC_CONF);

	phal_adc_adapter->status = ADCStatusIdle;

	DBG_ADC_INFO("adc dat reg: %08x\r\n", chX_register_data);
	DBG_ADC_INFO("single read <<\r\n");

	/* restore conf */
	padc_reg->ADC_CONF = adc_config_backup;

	/* restore list 0 */
	padc_reg->ADC_CHSW_LIST_0 = chsw_list_0_backup;

	if (((chX_register_data & (ADC_BIT_DAT_CH0_RDY)) >> ADC_SHIFT_DAT_CH0_RDY) && (((chX_register_data & (ADC_BIT_DAT_CH0_OVW)) >> ADC_SHIFT_DAT_CH0_OVW) == 0)) {
		return (uint16_t)(chX_register_data & ADC_MASK_DAT_CH0_DAT);
	} else {
		return (uint16_t)0xFFFF;
	}

}

/** \brief Description of hal_rtl_adc_clear_fifo
 *
 *    hal_rtl_adc_clear_fifo is used to clear FIFO to the default state.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:          Pointer to ADC control adapter.
 *   \return void
 */
void hal_rtl_adc_clear_fifo(hal_adc_adapter_t *phal_adc_adapter)  // can put to header file and do static inline?
{
	(phal_adc_adapter->init_dat.reg_base)->ADC_CLR_FIFO |= ADC_BIT_CLR_FIFO; // Set bit
#if CONFIG_FPGA
	hal_delay_us(40);
#else
	hal_delay_us(8);
#endif
	(phal_adc_adapter->init_dat.reg_base)->ADC_CLR_FIFO &= ~(ADC_BIT_CLR_FIFO); // Clear bit

}

/** \brief Description of hal_rtl_adc_read_ch_dat
 *
 *    hal_rtl_adc_read_ch_dat is used to read the channel data directly without any trigger operation.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:          Pointer to ADC control adapter.
 *   \param[in] uint8_t ch_no:                          Channel number.
 *   \return uint16_t:                              channel conversion data.\n
 *                                                  If the data is NOT ready or overwritten, a value of 0xFFFF would\n
 *                                                  be returned.
 */
uint16_t hal_rtl_adc_read_ch_dat(hal_adc_adapter_t *phal_adc_adapter, uint8_t ch_no, uint8_t out_raw)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t chX_register_data;

	/* check ADC is in progress or not */
	if (phal_adc_adapter->status != ADCStatusIdle) {

		DBG_ADC_WARN("Hardware is NOT in idle state\r\n");

		if (phal_adc_adapter->status == ADCStatusCVing) {
			DBG_ADC_WARN("A conversion is in progress\r\n");
		}

		return 0xFFFF;
	}

	chX_register_data = *((uint32_t *)(&(padc_reg->ADC_DAT_CH0)) + 0x04 * ch_no);
	DBG_ADC_INFO("adc dat reg: %08x\r\n", chX_register_data);

	if (((chX_register_data & (ADC_BIT_DAT_CH0_RDY)) >> ADC_SHIFT_DAT_CH0_RDY) && ((chX_register_data & (ADC_BIT_DAT_CH0_OVW) >> ADC_SHIFT_DAT_CH0_OVW) == 0)) {
		if (phal_adc_adapter->use_cali != 0) {
			chX_register_data = hal_rtl_adc_calc_cali_val((chX_register_data & ADC_MASK_DAT_CH0_DAT), (hal_adc_cali_para_t *)&adc_cali_paras);
			return (uint16_t)(chX_register_data & ADC_MASK_DAT_CH0_DAT); // weide try
		} else {
			return (uint16_t)(chX_register_data & ADC_MASK_DAT_CH0_DAT);
		}
	} else {
		if (out_raw) {
			return (uint16_t)(chX_register_data & ADC_MASK_DAT_CH0_DAT);
		} else {
			return (uint16_t)0xFFFF;
		}
	}

}

/** \brief Description of hal_rtl_adc_read_continuous
 *
 *    hal_rtl_adc_read_continuous is used to read the channel data continuously according to cvlist and different
 *                                          trigger source.
 *
 *   \param[in] hal_adc_adapter_t *phal_adc_adapter:          Pointer to ADC control adapter
 *   \param[in] uint32_t dat_len:                       ADC conversion data length.
 *   \param[in] uint16_t *dat_buf:                      ADC conversion data pointer.
 *   \param[in] uint8_t trigger_source:                 Inform ADC the trigger source.
 *                                                  0: software, 1: automatic, 2: timer
 *   \return hal_status_t.
 */
hal_status_t hal_rtl_adc_read_continuous(hal_adc_adapter_t *phal_adc_adapter, uint32_t dat_len, uint16_t *dat_buf,
		uint8_t trigger_source)
{
	ADC_TypeDef *padc_reg = (ADC_TypeDef *)phal_adc_adapter->init_dat.reg_base;
	uint32_t adc_cnt_tmp;

	/* check ADC is in progress or not */
	if (phal_adc_adapter->status != ADCStatusIdle) {
		DBG_ADC_WARN("Hardware in NOT in idle state\r\n");
		if (phal_adc_adapter->status == ADCStatusCVing) {
			DBG_ADC_WARN("A conversion is in progress\n");
		}
		return HAL_NOT_READY;
	}

	/* Set local ADC status C struct */
	phal_adc_adapter->cv_dat_len = dat_len;
	phal_adc_adapter->cv_dat_buf = dat_buf;
	adc_cv_dat_buf = dat_buf;
	adc_cv_dat_len = dat_len;
	phal_adc_adapter->continuous_rd = ADCEnable;
	phal_adc_adapter->use_dma = ADCDisable;

	adc_cnt_tmp = padc_reg->ADC_FLR;

	for (; adc_cnt_tmp > 0; adc_cnt_tmp--) {
		padc_reg->ADC_DAT_GLOBAL; // keep reading global data register to flush FIFO
	}

	/* disable adc first */
	padc_reg->ADC_CONF &= ~(ADC_BIT_EN);

	/* reset list to the default state */
	/* clear fifo to the default state */
	hal_rtl_adc_reset_list(phal_adc_adapter);
	hal_rtl_adc_clear_fifo(phal_adc_adapter);

	DBG_ADC_INFO("flr after ff clear, : %x\r\n", padc_reg->ADC_FLR);
	DBG_ADC_INFO("intr sts: %x\r\n", padc_reg->ADC_INTR_RAW_STS);

	/* clear intr status */
	padc_reg->ADC_INTR_STS |= ADC_BIT_IT_CVLIST_END_STS | ADC_BIT_IT_CV_END_STS | ADC_BIT_IT_CHCV_END_STS |
							  ADC_BIT_IT_FIFO_FULL_STS | ADC_BIT_IT_FIFO_OVER_STS | ADC_BIT_IT_FIFO_EMPTY_STS |
							  ADC_BIT_IT_DAT_OVW_STS | ADC_BIT_IT_ERR_STS | ADC_BIT_IT_COMP_CH0_STS |
							  ADC_BIT_IT_COMP_CH1_STS | ADC_BIT_IT_COMP_CH2_STS | ADC_BIT_IT_COMP_CH3_STS |
							  ADC_BIT_IT_COMP_CH4_STS | ADC_BIT_IT_COMP_CH5_STS | ADC_BIT_IT_COMP_CH6_STS |
							  ADC_BIT_IT_COMP_CH7_STS | ADC_BIT_IT_COMP_CH8_STS;

	padc_reg->ADC_INTR_CTRL |= ADC_BIT_IT_FIFO_FULL_EN | ADC_BIT_IT_FIFO_OVER_EN | ADC_BIT_IT_ERR_EN;

	phal_adc_adapter->status = ADCStatusCVing;

	padc_reg->ADC_FULL_LVL = phal_adc_adapter->init_dat.ff_tl << ADC_SHIFT_FULL_LVL;

	padc_reg->ADC_CONF |= ADC_BIT_EN; // why enable before setting the operation mode? - Weide
	padc_reg->ADC_CONF &= ~(ADC_MASK_OP_MOD); // clear any op modes set previously

	if (trigger_source == ADCSWTrigMod) {
		phal_adc_adapter->init_dat.hw_mode = ADCSWTrigMod;
		padc_reg->ADC_CONF |= ADCSWTrigMod << ADC_SHIFT_OP_MOD;
	} else if (trigger_source == ADCAutoMod) {
		phal_adc_adapter->init_dat.hw_mode = ADCAutoMod;
		padc_reg->ADC_CONF |= ADCAutoMod << ADC_SHIFT_OP_MOD;
		hal_rtl_adc_auto_chsw_ctrl(phal_adc_adapter, ADCEnable);
	} else if (trigger_source == ADCTmTrigMod) {
		phal_adc_adapter->init_dat.hw_mode = ADCTmTrigMod;
		padc_reg->ADC_CONF |= ADCTmTrigMod << ADC_SHIFT_OP_MOD;
	}

	// Weide: ADCCompTrigMod is defunct in ProII
	return HAL_OK;
}

/** \brief Description of hal_rtl_adc_calc_gain_deno
 *
 *    hal_rtl_adc_calc_gain_deno is used to calculate the denominator part of adc gain.
 *
 *   \param[in] int16_t real code 0
 *   \param[in] int16_t real code 1
 *   \return int16_t
 */
int16_t hal_rtl_adc_calc_gain_deno(int16_t real_co_0, int16_t real_co_1)

{
	if ((real_co_0 - real_co_1) < 0) {
		return (int16_t)((real_co_0 - real_co_1) * (-1));
	} else {
		return (int16_t)(real_co_0 - real_co_1);
	}

}

/** \brief Description of hal_rtl_adc_calc_gain_mole
 *
 *    hal_rtl_adc_calc_gain_mole is used to calculate the molecular part (numerator) of adc gain.
 *
 *   \param[in] int16_t ideal code 0
 *   \param[in] int16_t ideal code 1
 *   \return int16_t
 */
int16_t hal_rtl_adc_calc_gain_mole(int16_t ideal_co_0, int16_t ideal_co_1)

{
	if ((ideal_co_0 - ideal_co_1) < 0) {
		return (int16_t)((ideal_co_0 - ideal_co_1) * (-1));
	} else {
		return (int16_t)(ideal_co_0 - ideal_co_1);
	}

}

/** \brief Description of hal_rtl_adc_calc_offset_deno
 *
 *    hal_rtl_adc_calc_offset_deno is used to calculate denominator part of adc offset.
 *
 *   \param[in] int16_t real code 0
 *   \param[in] int16_t real code 1
 *   \return int16_t
 */
int16_t hal_rtl_adc_calc_offset_deno(int16_t real_co_0, int16_t real_co_1)
{
	if ((real_co_0 - real_co_1) < 0) {
		return (int16_t)((real_co_0 - real_co_1) * (-1));
	} else {
		return (int16_t)(real_co_0 - real_co_1);
	}

}

/** \brief Description of hal_rtl_adc_calc_offset_mole
 *
 *    hal_rtl_adc_calc_offset_mole is used to calculate molecular part (numerator) of adc offset.
 *
 *   \param[in] int16_t ideal code 0
 *   \param[in] int16_t ideal code 1
 *   \param[in] int16_t real code 0
 *   \param[in] int16_t real code 1

 *   \return int32_t
 */
int32_t hal_rtl_adc_calc_offset_mole(int16_t ideal_co_0, int16_t ideal_co_1, int16_t real_co_0, int16_t real_co_1)
{
	int32_t offset_deno;

	offset_deno = (ideal_co_1 * real_co_0) - (ideal_co_0 * real_co_1);

	if (offset_deno < 0) {
		return (int32_t)(offset_deno * (-1));
	} else {
		return (int32_t)offset_deno;
	}

}

typedef union bytes_tr_s {
	int64_t w64;
	struct {
		int64_t b0 : 8;
		int64_t b1 : 8;
		int64_t b2 : 8;
		int64_t b3 : 8;

		int64_t b4 : 8;
		int64_t b5 : 8;
		int64_t b6 : 8;
		int64_t b7 : 8;
	} b;
} bytes_tr_t, *pbytes_tr_t;

/** \brief Description of hal_rtl_adc_calc_cali_val
 *
 *    hal_rtl_adc_calc_cali_val is used to calculate calibration value
 *
 *   \param[in] uint16_t adc real value
 *   \param[in] hal_adc_cali_para_t *phal_adc_cali_para
 *   \return int32_t
 */
uint32_t hal_rtl_adc_calc_cali_val(uint16_t adc_read_val, hal_adc_cali_para_t *phal_adc_cali_para)
{
	bytes_tr_t bytes_tr;
	uint32_t ret_cali_val;
	__int64_t numerator;
	__int64_t denominator;

	__int64_t read_val_scle = (__int64_t)adc_read_val * (__int64_t)HP_ADC_CALC_SCLE; // scale up, prevent accidental truncation
	DBG_ADC_INFO("read_val_scle rom: %d\r\n", read_val_scle);
	__int64_t gain_deno_scle = (__int64_t)phal_adc_cali_para->gain_deno;
	DBG_ADC_INFO("gain_deno rom: %d\r\n", gain_deno_scle);
	__int64_t gain_mole_scle = (__int64_t)phal_adc_cali_para->gain_mole;
	DBG_ADC_INFO("gain_mole_scle rom: %d\r\n", gain_mole_scle);
	__int64_t offset_deno_scle = (__int64_t)(phal_adc_cali_para->offset_deno) * (__int64_t)HP_ADC_CALC_SCLE;
	DBG_ADC_INFO("offset_deno_scle rom: %d\r\n", offset_deno_scle);
	__int64_t offset_mole_scle = (__int64_t)phal_adc_cali_para->offset_mole * (__int64_t)HP_ADC_CALC_SCLE;
	DBG_ADC_INFO("offset_mole_scle rom: %d\r\n", offset_mole_scle);

	if ((phal_adc_cali_para->gain_deno == 0xFFFF) || (phal_adc_cali_para->gain_mole == 0xFFFF) ||
		(phal_adc_cali_para->offset_deno == 0xFFFFFFFF) || (phal_adc_cali_para->offset_mole == 0xFFFF)) {
		DBG_ADC_ERR("Incorrect cali. value,\n gd: %08x, gm: %08x,\n od: %08x, om: %08x\r\n", phal_adc_cali_para->gain_deno, phal_adc_cali_para->gain_mole,
					phal_adc_cali_para->offset_deno, phal_adc_cali_para->offset_mole);
		return (uint16_t)adc_read_val;
	}

	DBG_ADC_INFO("read val scaled:\n");
	bytes_tr.w64 = read_val_scle;
	DBG_ADC_INFO("b0: %02x\n", bytes_tr.b.b0);
	DBG_ADC_INFO("b1: %02x\n", bytes_tr.b.b1);
	DBG_ADC_INFO("b2: %02x\n", bytes_tr.b.b2);
	DBG_ADC_INFO("b3: %02x\n", bytes_tr.b.b3);
	DBG_ADC_INFO("b4: %02x\n", bytes_tr.b.b4);
	DBG_ADC_INFO("b5: %02x\n", bytes_tr.b.b5);
	DBG_ADC_INFO("b6: %02x\n", bytes_tr.b.b6);
	DBG_ADC_INFO("b7: %02x\n", bytes_tr.b.b7);
	DBG_ADC_INFO("offset deno scaled:\n");
	bytes_tr.w64 = offset_deno_scle;
	DBG_ADC_INFO("b0: %02x\n", bytes_tr.b.b0);
	DBG_ADC_INFO("b1: %02x\n", bytes_tr.b.b1);
	DBG_ADC_INFO("b2: %02x\n", bytes_tr.b.b2);
	DBG_ADC_INFO("b3: %02x\n", bytes_tr.b.b3);
	DBG_ADC_INFO("b4: %02x\n", bytes_tr.b.b4);
	DBG_ADC_INFO("b5: %02x\n", bytes_tr.b.b5);
	DBG_ADC_INFO("b6: %02x\n", bytes_tr.b.b6);
	DBG_ADC_INFO("b7: %02x\n", bytes_tr.b.b7);

	read_val_scle = (read_val_scle * gain_mole_scle); // numerator (penultimate)
	DBG_ADC_INFO("read_val_scle1: %lld\r\n\n", read_val_scle);
	read_val_scle = (read_val_scle + offset_mole_scle); // numerator (final)
	DBG_ADC_INFO("read_val_scle2: %lld\r\n\n", read_val_scle);

	numerator = read_val_scle; // for judging of mathematical rounding of final result (numerator)

	read_val_scle = read_val_scle / offset_deno_scle; // calculation division - Penultimate result
	DBG_ADC_INFO("read_val_scle aft div: %lld\r\n\n", read_val_scle);

	denominator = offset_deno_scle; // for judging rounding of final result (denominator)

	if ((numerator % denominator) > ((denominator / 2) + (denominator % 2))) {
		read_val_scle++;
	}
	ret_cali_val = (uint32_t)read_val_scle; // weide copied out from Jason's code

	if (read_val_scle < 0) {
		ret_cali_val = 0;
		DBG_ADC_WARN("cali value is less than 0.\r\n");
	} else if (read_val_scle > HP_ADC_CODE_MAX) {
		ret_cali_val = HP_ADC_CODE_MAX;
		DBG_ADC_WARN("cali value is greater than 4095.\r\n");
	}

	return (uint32_t)ret_cali_val;

}

/** \brief Description of hal_rtl_adc_read_cali_param
 *
 *    hal_rtl_adc_read_cali_param is used to read calibration parameters.
 *
 *   \param[in] uint16_t parameters address
 *   \param[in] uint8_t *return data address
 *   \param[in] uint8_t parameter length
 *   \return void
 */
void hal_rtl_adc_read_cali_param(uint16_t addr, uint8_t *param_addr, uint8_t param_len)
{

#if !defined(CONFIG_BUILD_NONSECURE)
	uint8_t rd_cnt;
	uint8_t cali_param_readback[8] = {0};
	hal_otp_init();
	for (rd_cnt = 0; rd_cnt < param_len; rd_cnt++) {
		if (!hal_otp_rd_syss(1, addr, (uint8_t *)cali_param_readback, param_len)) { // if function is normal, it returns 0
			DBG_ADC_INFO("val: %x\r\n", cali_param_readback[rd_cnt]);
		} else {
			DBG_ADC_ERR("Error!\r\n");
		}
	}

	for (rd_cnt = 0; rd_cnt < param_len; rd_cnt++) {
		DBG_ADC_INFO("read param (%x): %x\r\n", (addr + rd_cnt), *(param_addr + rd_cnt));
	}

	hal_otp_deinit();
#endif

}

/** \brief Description of hal_rtl_adc_write_cali_param
 *
 *    hal_rtl_adc_write_cali_param is used to write calibration parameters.
 *
 *   \param[in] uint16_t parameters address
 *   \param[in] uint8_t *data address
 *   \param[in] uint8_t parameter length
 *   \return void
 */
void hal_rtl_adc_write_cali_param(uint16_t addr, uint8_t *param_addr, uint8_t param_len)
{

#if !defined(CONFIG_BUILD_NONSECURE)
	uint8_t wr_cnt;
	hal_otp_init();

	hal_otp_wr_syss(1, addr, param_addr, param_len);

	for (wr_cnt = 0; wr_cnt < param_len; wr_cnt++) {
		DBG_ADC_INFO("write param (%x): %x\r\n", (addr + wr_cnt), *(param_addr + wr_cnt));
	}

	hal_otp_deinit();
#endif

}
#endif
/** @} */ /* End of group hs_hal_adc_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_adc */


