/**************************************************************************//**
 * @file     fw_voe_rom_boot.c
 * @brief    This file implements the VOE ROM boot related functions.
 *
 * @version  V1.00
 * @date     2021-08-03
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
#include "rtl8735b_voe_type.h"
#include "base_type.h"
#include "fw_voe_rom_boot.h"


#ifdef  _USE_RAM_SIM_FCS_
#include "hal_voe.h"
#endif

SECTION_ROM_TEMP_BSS uint8_t hp_adc_ch_pin_en_sts;
SECTION_ROM_TEMP_BSS uint8_t fcs_adc_use_cali;
SECTION_ROM_TEMP_BSS hal_adc_cali_para_t fcs_adc_cali_paras;
const hal_pin_map _hp_voe_adc_pins[HP_ADC_CH_NO] = {
	{PIN_F0, PID_ADC0},         //adc0
	{PIN_F1, PID_ADC1},         //adc1
	{PIN_F2, PID_ADC2},         //adc2
	{PIN_F3, PID_ADC3},         //adc3

	{PIN_A0, PID_COMP_ADC},     //adc4 - shared with comparator
	{PIN_A1, PID_COMP_ADC},     //adc5 - shared with comparator
	{PIN_A2, PID_COMP_ADC},     //adc6 - shared with comparator
	{PIN_A3, PID_COMP_ADC},     //adc7 - shared with comparator
};


const u32 pwm_pin_table0[MaxPwmNum] = {
	PIN_F6, PIN_F7, PIN_F8, PIN_F9, PIN_F10, PIN_F11, PIN_F12, PIN_F13, PIN_F14, PIN_F15, PIN_F16, PIN_F17  // S0
};




static int hal_rtl_voe_fcs_check_mheader_tm(uint32_t mfcs_header_addr, uint8_t fcs_id)
{
	isp_multi_fcs_hdr_t *mfcs_header = (isp_multi_fcs_hdr_t *)mfcs_header_addr;

	if (mfcs_header->magic != ISP_MULTI_FCS_MAGIC_NUM || mfcs_header->version != ISP_MFCS_DATA_VERSION) {
		HAL_WRITE32(0x40009000, 0x0154, MFCS_DATA_HEADER_ERR);
		DBG_BOOT_ERR("MFCS header error :magic 0x%08x version 0x%08x \n", mfcs_header->magic, mfcs_header->version);

		return MFCS_DATA_HEADER_ERR;
	}
	dbg_printf("mfcs_data version 0x%08x\r\n", mfcs_header->version);

	if (mfcs_header->multi_fcs_cnt > MULTI_FCS_MAX || fcs_id >= mfcs_header->multi_fcs_cnt) {
		HAL_WRITE32(0x40009000, 0x0154, MFCS_FCS_CNT_ID_ERR);
		DBG_BOOT_ERR("MFCS cnt %d id %d error \n", mfcs_header->multi_fcs_cnt, fcs_id);
		return MFCS_FCS_CNT_ID_ERR;
	}

	return 0;

}


static int hal_rtl_voe_fcs_check_header_tm(uint32_t fcs_data_addr)
{
	isp_fcs_header_t *fcs_data_header = (isp_fcs_header_t *)fcs_data_addr;

	if (fcs_data_header->magic != ISP_FCS_DATA_MAGIC_NUM || (fcs_data_header->version & ISP_FCS_VERION_MJR_MASK) != (ISP_FCS_DATA_VERSION)) {
		HAL_WRITE32(0x40009000, 0x0154, FCS_DATA_HEADER_ERR);
		DBG_BOOT_ERR("FCS data header error :magic 0x%08x version 0x%08x \n", fcs_data_header->magic, fcs_data_header->version);

		return FCS_DATA_HEADER_ERR;
	}
	dbg_printf("fcs_data version 0x%08x\r\n", fcs_data_header->version);

	if (fcs_data_header->gpio_cnt > GPIO_MAX) {
		HAL_WRITE32(0x40009000, 0x0154, FCS_DATA_GPIO_CNT_ERR);
		DBG_BOOT_ERR("FCS data gpio cnt error : %d \n", fcs_data_header->gpio_cnt);
		return FCS_DATA_GPIO_CNT_ERR;
	}

	if (fcs_data_header->i2c_device_cnt > I2C_MAX) {
		HAL_WRITE32(0x40009000, 0x0154, FCS_DATA_I2C_CNT_ERR);
		DBG_BOOT_ERR("FCS data i2C cnt error : %d \n", fcs_data_header->i2c_device_cnt);
		return FCS_DATA_I2C_CNT_ERR;
	}
	return 0;

}

hal_status_t hal_rtl_i2c_pin_register_simple(volatile hal_i2c_adapter_t *phal_i2c_adapter)
{
	hal_status_t ret;
	hal_rtl_sys_peripheral_en(I2C0_SYS + phal_i2c_adapter->init_dat.index, 1);

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.scl_pin) != (uint8_t)0xFF) {
		if (phal_i2c_adapter->init_dat.index == 3) {
			ret = hal_rtl_pinmux_register(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C3);
		} else {
			ret = hal_rtl_pinmux_register(phal_i2c_adapter->pltf_dat.scl_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	if ((uint8_t)(phal_i2c_adapter->pltf_dat.sda_pin) != (uint8_t)0xFF) {

		if (phal_i2c_adapter->init_dat.index == 3) {
			ret |= hal_rtl_pinmux_register(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C3);
		} else {
			ret |= hal_rtl_pinmux_register(phal_i2c_adapter->pltf_dat.sda_pin, PID_I2C0 + phal_i2c_adapter->init_dat.index);
		}
	}

	return ret;
}

hal_status_t hal_rtl_adc_pin_init_simple(hal_adc_adapter_t *phal_adc_adapter)
{
	uint8_t adc_ch_cnt;
	hal_status_t retv;

	for (adc_ch_cnt = 0; adc_ch_cnt < HP_ADC_CH_NO; adc_ch_cnt++) { // weide modified
		if (phal_adc_adapter->plft_dat.pin_en.w & ((uint32_t)0x01 << adc_ch_cnt)) {
			if ((hp_adc_ch_pin_en_sts & ((uint32_t)0x01 << adc_ch_cnt)) == 0) {

				if (adc_ch_cnt < 4) { // first 4 pins - pure ADC
					retv = hal_rtl_pinmux_register((_hp_voe_adc_pins[adc_ch_cnt]).pin_name, (PID_ADC0 + adc_ch_cnt));
				} else { // last 4 pins - share with comparator
					retv = hal_rtl_pinmux_register((_hp_voe_adc_pins[adc_ch_cnt]).pin_name, FUNC_COMP_ADC);
				}

				if (retv != HAL_OK) {
					dbg_printf("retv post pwr down: %d\r\n", retv);
					return retv;
				}

				if (retv != HAL_OK) {
					dbg_printf("retv before pin en sts: %d\r\n", retv);
					return retv;
				}

				hp_adc_ch_pin_en_sts |= ((uint32_t)0x01 << adc_ch_cnt);
			} else {
				dbg_printf("ch-%d pin has been enabled.\r\n", adc_ch_cnt);
				dbg_printf("adc pin sts: %x\r\n", hp_adc_ch_pin_en_sts);
			}
		}
	}

	return HAL_OK;
}

void hal_rtl_voe_use_adc_read_cali_param(uint16_t addr, uint8_t *param_addr, uint8_t param_len)
{
	uint32_t rd_cnt = 0x0;
	uint16_t otp_addr = addr;
	uint8_t *cur_param_addr = param_addr;
	for (rd_cnt = 0; rd_cnt < param_len; rd_cnt++) {
		*(cur_param_addr + rd_cnt) = hal_rtl_otp_byte_rd_syss((otp_addr + rd_cnt));
	}
}

void hal_rtl_adc_init_simple(uint8_t *pfcs_adc_use_cali, hal_adc_cali_para_t *p_fcs_adc_cali_paras)
{
	hal_status_t retv;
	uint8_t i;
	fcs_adc_paras_list_t fcs_adc_para_list[FCS_ADC_PARAM_IDX_MAX];

	*pfcs_adc_use_cali = ENABLE;
	_memset(p_fcs_adc_cali_paras, 0xFF, sizeof(hal_adc_cali_para_t));
	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_DENO_IDX].otp_addr      = HP_ADC_GAIN_DENO_ADDR;
	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_DENO_IDX].p_adc_para_v  = &(p_fcs_adc_cali_paras->gain_deno);
	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_DENO_IDX].adc_para_size = FCS_ADC_PARAM_GAIN_DENO_SIZE;

	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_MOLE_IDX].otp_addr      = HP_ADC_GAIN_MOLE_ADDR;
	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_MOLE_IDX].p_adc_para_v  = &(p_fcs_adc_cali_paras->gain_mole);
	fcs_adc_para_list[FCS_ADC_PARAM_GAIN_MOLE_IDX].adc_para_size = FCS_ADC_PARAM_GAIN_MOLE_SIZE;

	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_DENO_IDX].otp_addr      = HP_ADC_OFFSET_DENO_ADDR;
	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_DENO_IDX].p_adc_para_v  = &(p_fcs_adc_cali_paras->offset_deno);
	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_DENO_IDX].adc_para_size = FCS_ADC_PARAM_OFFSET_DENO_SIZE;

	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_MOLE_IDX].otp_addr      = HP_ADC_OFFSET_MOLE_ADDR;
	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_MOLE_IDX].p_adc_para_v  = &(p_fcs_adc_cali_paras->offset_mole);
	fcs_adc_para_list[FCS_ADC_PARAM_OFFSET_MOLE_IDX].adc_para_size = FCS_ADC_PARAM_OFFSET_MOLE_SIZE;
	//adc_irq_adpt = phal_adc_adapter; // don't understand why???
	//hal_rtl_adc_reg_irq(phal_adc_adapter, (irq_handler_t) hal_rtl_adc_irq_handler);
	//hal_irq_set_priority(ADC_IRQn, ADC_IRQPri);

	//retv = hal_rtl_adc_pure_init(phal_adc_adapter);

	/* read calibration params */
	//DBG_ADC_INFO("read cali param\r\n");
	hal_rtl_otp_init();
	for (i = 0; i < FCS_ADC_PARAM_IDX_MAX; i++) {
		hal_rtl_voe_use_adc_read_cali_param((uint16_t)fcs_adc_para_list[i].otp_addr, (uint8_t *)(fcs_adc_para_list[i].p_adc_para_v),
											fcs_adc_para_list[i].adc_para_size);
		if (FCS_ADC_PARAM_OFFSET_MOLE_IDX == i) {
			if ((uint32_t) * (fcs_adc_para_list[i].p_adc_para_v) == 0xFFFFFFFF) {
				DBG_BOOT_ERR("Read adc_para(%d) failed\r\n", i);
				*pfcs_adc_use_cali = DISABLE;
			}
		} else {
			if ((uint16_t) * (fcs_adc_para_list[i].p_adc_para_v) == 0xFFFF) {
				DBG_BOOT_ERR("Read adc_para(%d) failed\r\n", i);
				*pfcs_adc_use_cali = DISABLE;
			}
		}
	}
	hal_rtl_otp_deinit();
#if 0
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
#endif

	//hal_rtl_adc_clear_fifo(phal_adc_adapter);
	//phal_adc_adapter->status = ADCStatusIdle;

	//return retv;
}

int hal_rtl_voe_fcs_init_resource_tm(uint32_t fcs_data_addr, uint8_t *pfcs_adc_use_cali, hal_adc_cali_para_t *p_fcs_adc_cali_paras)
{
	int ret = 0;
	isp_fcs_header_t *fcs_data_header = (isp_fcs_header_t *)fcs_data_addr;

	hal_rtl_sys_peripheral_en(GPIO_AON, 1);
	hal_rtl_sys_peripheral_en(GPIO_SYS, 1);

	for (int i = 0; i < fcs_data_header->gpio_cnt  ; i++) {
		dbg_printf("gpio %04x \r\n", fcs_data_header->gpio_list[i]);
		ret = hal_rtl_pinmux_register(fcs_data_header->gpio_list[i], PID_GPIO);
		if (ret) {
			DBG_BOOT_ERR("gpio hal_rtl_pinmux_register err ret 0x%08x \r\n", ret);
			return ret;
		}
	}

	volatile hal_i2c_adapter_t  i2c_master_sample;
	if (fcs_data_header->i2c_id != 0xFF) {
		i2c_master_sample.pltf_dat.scl_pin  = fcs_data_header->i2c_scl;
		i2c_master_sample.pltf_dat.sda_pin  = fcs_data_header->i2c_sda;
		i2c_master_sample.init_dat.index = fcs_data_header->i2c_id;
		ret  = hal_rtl_i2c_pin_register_simple(&i2c_master_sample);
		if (ret) {
			DBG_BOOT_ERR("hal_rtl_i2c_pin_register_simple err ret 0x%08x \r\n", ret);
			return ret;
		} else {
			dbg_printf("i2c %d scl %04x sda %04x \r\n", fcs_data_header->i2c_id, fcs_data_header->i2c_scl, fcs_data_header->i2c_sda);
		}
	}

#ifdef _FPGA_MIPI_APHY_  // only for FGPA MIPI APHY daughter board
	volatile hal_i2c_adapter_t  i2c_master_aphy_sample;
	i2c_master_aphy_sample.pltf_dat.scl_pin = PIN_E3;
	i2c_master_aphy_sample.pltf_dat.sda_pin = PIN_E4;
	i2c_master_aphy_sample.init_dat.index = 2;
	ret  = hal_rtl_i2c_pin_register_simple(&i2c_master_aphy_sample);

	if (ret) {
		dbg_printf("hal_rtl_i2c_pin_register_simple mipiaphy err ret 0x%08x \r\n", ret);
		return ret;
	} else {
		dbg_printf("_FPGA_MIPI_APHY_ i2c init \r\n");
	}
#endif



	// Enable sensor clock(hclk)
	if (fcs_data_header->snr_clk_pin != 0xFF) {
		ret = hal_rtl_pinmux_register(fcs_data_header->snr_clk_pin, PID_SENSOR);
		dbg_printf("sensor hclk pin %04x \r\n", fcs_data_header->snr_clk_pin);
		if (ret) {
			DBG_BOOT_ERR("snr_clk_pin hal_rtl_pinmux_register err ret 0x%08x \r\n", ret);
			return ret;
		}
	}

	if (fcs_data_header->adc_id != 0xFF) {

		hal_rtl_sys_peripheral_en(ADC_SYS, ENABLE);

		volatile hal_adc_adapter_t hal_adc_adpt;
		hal_adc_adpt.plft_dat.pin_en.w |= (0x01 << fcs_data_header->adc_id);

		/* Pinmux Initialization */
		ret = hal_rtl_adc_pin_init_simple(&hal_adc_adpt);
		if (ret) {
			DBG_BOOT_ERR("hal_rtl_adc_pin_init_simple err ret 0x%08x \r\n", ret);
			return ret;
		}
		hal_rtl_adc_init_simple(pfcs_adc_use_cali, p_fcs_adc_cali_paras);
		dbg_printf("adc_id %02x \r\n", fcs_data_header->adc_id);
	}


	if (fcs_data_header->pwm_id != 0xFF) {

		hal_rtl_sys_peripheral_en(PWM_SYS, ENABLE);

		volatile uint32_t val;
		val = HAL_READ32(0x40009800, 0x2C);
		val |= (0x1 << PON_SHIFT_SCLK_PWM_SEL);
		HAL_WRITE32(0x40009800, 0x2C, val);	//Select OSC4MHz

		u32 pin_name = pwm_pin_table0[fcs_data_header->pwm_id]; // only use pwm sel = 0

		ret = hal_rtl_pinmux_register(pin_name, PID_PWM0 + fcs_data_header->pwm_id);
		if (ret) {
			DBG_BOOT_ERR("pwm hal_rtl_pinmux_register err ret 0x%08x \r\n", ret);
			return ret;
		}
		dbg_printf("pwm id %02x pin  %02x\r\n", fcs_data_header->pwm_id, pin_name);
		//dbg_printf("0x4000982C 0x%08x 0x40045004 0x%08x\r\n", HAL_READ32(0x40009800, 0x2C), HAL_READ32(0x40045000, 0x4));
	}
	return ret;
}


static int hal_rtl_voe_load_FCS_data(voe_cpy_t voe_cpy, uint32_t fcs_data_addr, hal_adc_cali_para_t *p_fcs_adc_cali_paras)
{

	uint32_t reg_value32;
	uint32_t start_time, current_time;

	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;


	isp_fcs_header_t *fcs_data_header = (isp_fcs_header_t *)fcs_data_addr;

	isp_fcs_header_t *KM_fcs_data_header;  // for store adc cali data to KM SRAM


	//printf(" fcs_data itcm(%d) addr 0x%08x dtcm(%d) addr 0x%08x\n", fcs_data_header->itcm_size, fcs_data_header->itcm_addr, fcs_data_header->dtcm_size, fcs_data_header->dtcm_addr);

	start_time = hal_rtl_misc_read_curtime();
	current_time = start_time;
	while (1) {

		reg_value32 = HAL_READ32(KM_STATUS, 0x0);
		if (reg_value32 == FCS_INIT_ROM_RDY_KM) {
			dbg_printf("Wait KM init %d us\r\n", current_time - start_time);
			break;
		}
		current_time = hal_rtl_misc_read_curtime();
		if (current_time - start_time > WAIT_KM_INIT_TIMEOUT_US) {
			DBG_BOOT_ERR("Wait KM init timeout \n");
			HAL_WRITE32(0x40009000, 0x0154, FCS_WAIT_KM_TMOUT_ERR);
			return FCS_WAIT_KM_TMOUT_ERR;

		}
	}

	if (voe_cpy == NULL) {
		DBG_BOOT_ERR("voe FCS data copy API is NULL \n");
		HAL_WRITE32(0x40009000, 0x0154, FCS_CPY_FUNC_ERR);
		return FCS_CPY_FUNC_ERR;
	}

	if (fcs_data_header->dtcm_size != 0) {

		if (fcs_data_header->dtcm_addr < VOE_DRAM_S || (fcs_data_header->dtcm_addr + fcs_data_header->dtcm_size) > VOE_DRAM_E) {
			DBG_BOOT_ERR("dtcm_addr error addr 0x%08x size 0x%08x\n", fcs_data_header->dtcm_addr, fcs_data_header->dtcm_size);
			HAL_WRITE32(0x40009000, 0x0154, FCS_DTCM_INFO_ERR);
			return FCS_DTCM_INFO_ERR;
		}
		voe_cpy((void *)(fcs_data_header->dtcm_addr + REMAP_S7_DTCM), fcs_data_addr, (size_t)fcs_data_header->dtcm_size);
		VOE->VOE_REG_KM_FW_BASE_ADDRESS = fcs_data_header->dtcm_addr; // sync KM fcs_data with this register


		if (ENABLE == fcs_adc_use_cali) { // store adc otp cali data to KM SRAM
			KM_fcs_data_header = (isp_fcs_header_t *)(fcs_data_header->dtcm_addr + REMAP_S7_DTCM);
			KM_fcs_data_header->adc_use_cali = fcs_adc_use_cali;
			KM_fcs_data_header->adc_gain_deno   = p_fcs_adc_cali_paras->gain_deno;
			KM_fcs_data_header->adc_gain_mole   = p_fcs_adc_cali_paras->gain_mole;
			KM_fcs_data_header->adc_offset_deno = p_fcs_adc_cali_paras->offset_deno;
			KM_fcs_data_header->adc_offset_mole = p_fcs_adc_cali_paras->offset_mole;
			dbg_printf("adc_gain_deno = 0x%x\r\n", KM_fcs_data_header->adc_gain_deno);
			dbg_printf("adc_gain_mole = 0x%x\r\n", KM_fcs_data_header->adc_gain_mole);
			dbg_printf("adc_offset_deno = 0x%x\r\n", KM_fcs_data_header->adc_offset_deno);
			dbg_printf("adc_offset_mole = 0x%x\r\n", KM_fcs_data_header->adc_offset_mole);
			dbg_printf("save adc cali data to KM DTCM FCS header\r\n");
		}

		rtl_dcache_clean_invalidate_by_addr((uint32_t *)(fcs_data_header->dtcm_addr + REMAP_S7_DTCM), (int32_t)fcs_data_header->dtcm_size);


	} else if (fcs_data_header->itcm_size != 0) {

		if (fcs_data_header->itcm_addr < VOE_IRAM_S || (fcs_data_header->itcm_addr + fcs_data_header->itcm_size) > VOE_IRAM_E) {
			DBG_BOOT_ERR("itcm_addr error addr 0x%08x size 0x%08x\n", fcs_data_header->itcm_addr, fcs_data_header->itcm_size);
			HAL_WRITE32(0x40009000, 0x0154, FCS_ITCM_INFO_ERR);
			return FCS_ITCM_INFO_ERR;
		}

		voe_cpy((void *)(fcs_data_header->itcm_addr + REMAP_S7_ITCM), fcs_data_addr, (size_t)fcs_data_header->itcm_size);

		VOE->VOE_REG_KM_FW_BASE_ADDRESS = fcs_data_header->itcm_addr;

		if (fcs_adc_use_cali) {
			KM_fcs_data_header = (isp_fcs_header_t *)(fcs_data_header->itcm_addr + REMAP_S7_ITCM);
			KM_fcs_data_header->adc_use_cali = fcs_adc_use_cali;
			KM_fcs_data_header->adc_gain_deno = fcs_adc_cali_paras.gain_deno;
			KM_fcs_data_header->adc_gain_mole = fcs_adc_cali_paras.gain_mole;
			KM_fcs_data_header->adc_offset_deno = fcs_adc_cali_paras.offset_deno;
			KM_fcs_data_header->adc_offset_mole = fcs_adc_cali_paras.offset_mole;
			dbg_printf("save adc cali data to KM ITCM FCS header\r\n");
		}

		rtl_dcache_clean_invalidate_by_addr((uint32_t *)(fcs_data_header->itcm_addr + REMAP_S7_ITCM), (int32_t)fcs_data_header->itcm_size);


	}

	HAL_WRITE32(0x40009000, 0x0154, FCS_DATA_LOAD_OK_TM);
	return 0;
}
#if 0
int hal_rtl_voe_load_FCS_FW(voe_cpy_t voe_cpy, int *fw_addr)
{


	/*
	 *  voe.bin format layout
	 *
	 *  /           voe header                          /
	 *  /   voe year    /   voe month   /   voe day     /
	 *  /       2B      /       1B      /       1B      /
	 *  /   Version     /   itcm size   /   dtcm size   / ddr size /
	 *  /       24B     /       4B      /       4B      / 4B

	 *  shift 20B
	 *  /           voe data            /
	 *  /   itcm data   /   dtcm data   /
	 *  /       XXB     /       XXB     /
	 */


	uint32_t iram_fw_addr = VOE_IRAM_S;
	uint32_t dram_fw_addr = VOE_DRAM_S;


	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	u32 *addr = (u32 *)fw_addr;
	voe_header_t *voe_header = (voe_header_t *)fw_addr;
	u32 header_size = sizeof(voe_header_t);



	if (voe_cpy == NULL) {
		DBG_BOOT_ERR("voe FCS FW copy API is NULL \n");
		return -1;
	}

	//hal_sys_peripheral_en(VOE_SYS,DISABLE);

	dbg_printf("voe_fcs:date %d/%d/%d version:%s\n", voe_header->year, voe_header->month, voe_header->day, voe_header->version);
	dbg_printf("voe_fcs:FW size itcm(%d) dtcm(%d) ddr(%d)\n", voe_header->itcm_size, voe_header->dtcm_size, voe_header->ddr_size);

	//hal_sys_peripheral_en(VOE_SYS,ENABLE);



	//voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS| VOE_BIT_VOE_RESET_DIS| VOE_BIT_KM_RESET_DIS;


	voe_cpy((void *)(iram_fw_addr + REMAP_S7_ITCM)
			, (addr + (header_size >> 2))
			, (size_t)voe_header->itcm_size);

	rtl_dcache_clean_invalidate_by_addr((uint32_t *)(iram_fw_addr + REMAP_S7_ITCM)
										, (int32_t)voe_header->itcm_size);

	voe_cpy((void *)(dram_fw_addr + REMAP_S7_DTCM)
			, (addr + ((header_size + voe_header->itcm_size) >> 2))
			, (size_t)voe_header->dtcm_size);

	rtl_dcache_clean_invalidate_by_addr((uint32_t *)(dram_fw_addr + REMAP_S7_DTCM)
										, (int32_t)voe_header->dtcm_size);

	//printf("copy done \n", __func__);

	//VOE_VECTOR_SET
	voe->VOE_REG_KM_INITVTOR_S = iram_fw_addr;
	voe->VOE_REG_KM_INITVTOR_NS = iram_fw_addr;

	//KM COLD_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS;

	//KM_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS
								   | VOE_BIT_VOE_RESET_DIS;

	//KM RUN
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS
								   | VOE_BIT_VOE_RESET_DIS
								   | VOE_BIT_KM_RESET_DIS;

	return 0;


}
#endif

int hal_rtl_voe_fcs_process(voe_cpy_t voe_cpy, uint32_t mfcs_data_addr, uint8_t fcs_id, uint32_t total_size)
{
	int ret = 0;
	isp_multi_fcs_hdr_t temp_mfcs_header;
	uint32_t target_fcs_addr, target_fcs_size;

	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	voe_cpy((void *)(&temp_mfcs_header), (void *)mfcs_data_addr, sizeof(isp_multi_fcs_hdr_t));

	ret = hal_rtl_voe_fcs_check_mheader_tm(&temp_mfcs_header, fcs_id);
	if (ret) {
		DBG_BOOT_ERR("hal_rtl_voe_fcs_check_mheader_tm err 0x%08x \r\n", ret);
		return ret;
	}

	target_fcs_addr = temp_mfcs_header.fcs_data_offset[fcs_id] + mfcs_data_addr;
	target_fcs_size = temp_mfcs_header.fcs_data_size[fcs_id];

	if (temp_mfcs_header.fcs_data_offset[fcs_id] + target_fcs_size > total_size) {
		HAL_WRITE32(0x40009000, 0x0154, MFCS_FCS_ADDR_SZ_ERR);
		DBG_BOOT_ERR("MFCS target error :offset 0x%08x size 0x%08x \n", temp_mfcs_header.fcs_data_offset[fcs_id], target_fcs_size);
		return MFCS_FCS_ADDR_SZ_ERR;
	}

	isp_fcs_header_t temp_fcs_data_header;

	voe_cpy((void *)(&temp_fcs_data_header), (void *)target_fcs_addr, sizeof(isp_fcs_header_t));

	ret = hal_rtl_voe_fcs_check_header_tm(&temp_fcs_data_header);
	if (ret) {
		DBG_BOOT_ERR("hal_rtl_voe_fcs_check_header_tm err 0x%08x \r\n", ret);
		return ret;
	}

	// turn on VOE
	hal_rtl_sys_peripheral_en(VOE_SYS, ENABLE);

	//KM COLD_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS;

	//KM_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS | VOE_BIT_VOE_RESET_DIS;

	//KM RUN
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS | VOE_BIT_VOE_RESET_DIS | VOE_BIT_KM_RESET_DIS;

	ret = hal_rtl_voe_fcs_init_resource_tm(&temp_fcs_data_header, &fcs_adc_use_cali, &fcs_adc_cali_paras);
	if (ret) {
		DBG_BOOT_ERR("hal_voe_fcs_init_resource_tm err 0x%08x \r\n", ret);
		return ret;
	}



#ifdef  _USE_RAM_SIM_FCS_
	extern int _binary_voe_fcs_bin_start[];         // VOE FW address

	ret = hal_rtl_voe_load_FCS_FW(voe_cpy, (int *)_binary_voe_fcs_bin_start);
	if (ret) {
		DBG_BOOT_ERR("hal_voe_load_FCS_FW err 0x%08x \r\n", ret);
		return ret;
	}
#endif

	ret = hal_rtl_voe_load_FCS_data(voe_cpy, target_fcs_addr, &fcs_adc_cali_paras);
	if (ret) {
		DBG_BOOT_ERR("hal_rtl_voe_load_FCS_data err 0x%08x \r\n", ret);
		return ret;
	}
	//hal_delay_us(10000);
	return ret;
}
/** @} */ /* End of group hs_hal_voe */

