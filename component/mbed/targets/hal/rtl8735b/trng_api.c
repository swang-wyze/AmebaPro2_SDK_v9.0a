/**************************************************************************//**
 * @file    trng_api.c
 * @brief    main function example.
 * @version  V1.00
 * @date     2021 8 2
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/


#include "hal_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include "hal_trng.h"
#include "device.h"


volatile hal_trng_adapter_t  trng_veri_adpt;
volatile hal_rng_reg_t hal_rng_reg;
volatile hal_rng_st_reg_t hal_rng_st_reg;


#if DEVICE_TRNG





void trng_setting(hal_trng_adapter_t *ptrng_adp, hal_rng_st_reg_t *ptrng_st_reg,  hal_rng_reg_t *ptrng_reg)
{
	// trng control
	hal_rng_reg.LFSR_mode2 = 0x0;
	hal_rng_reg.TRNG_mode = 0x1;
	hal_rng_reg.selftest = 0x1;
	hal_rng_reg.LFSR_mode = 0x0;
	hal_rng_reg.LFSR_bypass1 = 0x0;
	hal_rng_reg.TRNG_dbg1_sel = 0x1;
	hal_rng_reg.TRNG_dbg0_sel = 0x0;
	hal_rng_reg.TRNG_corrector_lmode = 0x0;
	hal_rng_reg.corrector_bypass = 0x0;
	hal_rng_reg.high_speed_clock = 0x1;
	hal_rng_reg.RNG_SRST = 0x0;
	//trng selftest
	hal_rng_st_reg.thr_done_adap1 = 0x4;
	hal_rng_st_reg.thr_done_adap2 = 0x4;
	hal_rng_st_reg.compare_rep = 0x1;
	hal_rng_st_reg.compare_unit_adap1 = 0x0;
	hal_rng_st_reg.compare_unit_adap2 = 0x0;
	hal_rng_st_reg.window_size_adap1 = 0x1;
	hal_rng_st_reg.window_size_adap2 = 0x1;
	hal_rng_st_reg.thr_err_rep = 0x5;
	hal_rng_st_reg.thr_thr_done_rep = 0x0;

}


void trng_init_32k(void)
{
	hal_trng_init((hal_trng_adapter_t *)&trng_veri_adpt);
	hal_trng_enable_32K((hal_trng_adapter_t *)&trng_veri_adpt);
	trng_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_st_reg_t *)&hal_rng_st_reg, (hal_rng_reg_t *)&hal_rng_reg);
}

void trng_init_128k(void)
{
	hal_trng_init((hal_trng_adapter_t *)&trng_veri_adpt);
	hal_trng_enable_32K((hal_trng_adapter_t *)&trng_veri_adpt);
	trng_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_st_reg_t *)&hal_rng_st_reg, (hal_rng_reg_t *)&hal_rng_reg);
}

void trng_deinit(void)
{
	hal_trng_deinit((hal_trng_adapter_t *)&trng_veri_adpt);
}

void trng_run_32k(uint32_t length, uint32_t *arr)
{
	volatile uint32_t data, readybit, error_interrupt, case3_check = 0 ;
	volatile uint32_t  sample_count = 0;
	//hal_trng_init((hal_trng_adapter_t *)&trng_veri_adpt);
	//hal_trng_enable_32K((hal_trng_adapter_t *)&trng_veri_adpt);
	while (sample_count < length) {
		hal_trng_enable_32K((hal_trng_adapter_t *)&trng_veri_adpt);
		do {
			hal_trng_control_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_reg_t *)&hal_rng_reg);
			hal_delay_us(20);
			hal_trng_self_test_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_st_reg_t *)&hal_rng_st_reg);
			hal_delay_us(20);
			hal_trng_reset((hal_trng_adapter_t *)&trng_veri_adpt);
			hal_delay_us(20);
			hal_trng_self_test_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_st_reg_t *)&hal_rng_st_reg);
			hal_trng_interrupt_reg((hal_trng_adapter_t *)&trng_veri_adpt, (u32)0xF);
			do {
				hal_delay_us(10);
				readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
				error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				hal_delay_us(100);

				if (readybit == 0x0 && error_interrupt == 0x1) {
					readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
					error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
					hal_trng_clear_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				}

				if (readybit == 0x1 && error_interrupt == 0x1) {
					hal_delay_us(10);
					readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
					error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
					hal_trng_clear_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				}

				if (readybit == 0x0 && error_interrupt == 0x0) {
					if (case3_check == 0) {
						hal_delay_us(1000);
						case3_check = 1;
					}
					if (case3_check != 0) {
						hal_delay_us(30);
					}
				}

				if (readybit == 0x1 && error_interrupt == 0x0) {
					data = hal_trng_read_data((hal_trng_adapter_t *)&trng_veri_adpt);
					arr[sample_count]  = data;
					sample_count ++;
				}
				hal_delay_ms(10);
			} while (((readybit = 0x0 && error_interrupt == 0x1) || (readybit == 0x1 && error_interrupt == 0x1) || (readybit == 0x0 && error_interrupt == 0x0)) &&
					 (sample_count < length));
		} while (((readybit == 0x0 && error_interrupt == 0x0) && (sample_count < length)));
	}
}

void trng_run_128k(uint32_t length, uint32_t *arr)
{
	volatile uint32_t data, readybit, error_interrupt, case3_check = 0 ;
	volatile uint32_t  sample_count = 0;

	//hal_trng_init((hal_trng_adapter_t *)&trng_veri_adpt);
	//hal_trng_enable_128K((hal_trng_adapter_t *)&trng_veri_adpt);

	while (sample_count < length) {
		hal_trng_enable_128K((hal_trng_adapter_t *)&trng_veri_adpt);
		do {
			hal_trng_control_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_reg_t *)&hal_rng_reg);
			hal_delay_us(20);
			hal_trng_self_test_setting((hal_trng_adapter_t *)&trng_veri_adpt, (hal_rng_st_reg_t *)&hal_rng_st_reg);
			hal_delay_us(20);
			hal_trng_reset((hal_trng_adapter_t *)&trng_veri_adpt);
			hal_delay_us(20);
			hal_trng_interrupt_reg((hal_trng_adapter_t *)&trng_veri_adpt, (u32)0xF);

			do {
				hal_delay_us(10);
				readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
				error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				hal_delay_us(100);

				if (readybit == 0x0 && error_interrupt == 0x1) {
					readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
					error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
					hal_trng_clear_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				}

				if (readybit == 0x1 && error_interrupt == 0x1) {
					hal_delay_us(10);
					readybit = hal_trng_read_readybit((hal_trng_adapter_t *)&trng_veri_adpt);
					error_interrupt = hal_trng_read_parity_error_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
					hal_trng_clear_interrupt((hal_trng_adapter_t *)&trng_veri_adpt);
				}

				if (readybit == 0x0 && error_interrupt == 0x0) {
					if (case3_check == 0) {
						hal_delay_us(1000);
						case3_check = 1;
					}
					if (case3_check != 0) {
						hal_delay_us(30);
					}
				}

				if (readybit == 0x1 && error_interrupt == 0x0) {
					data = hal_trng_read_data((hal_trng_adapter_t *)&trng_veri_adpt);
					arr[sample_count]  = data;
					sample_count ++;
				}
				hal_delay_ms(10);
			} while (((readybit = 0x0 && error_interrupt == 0x1) || (readybit == 0x1 && error_interrupt == 0x1) || (readybit == 0x0 && error_interrupt == 0x0)) &&
					 (sample_count < length));
		} while (((readybit == 0x0 && error_interrupt == 0x0) && (sample_count < length)));
	} //while(sample_count < sample_size)

}




#endif //#if DEVICE_TRNG
