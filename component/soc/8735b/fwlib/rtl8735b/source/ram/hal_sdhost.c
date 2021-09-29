/**************************************************************************//**
 * @file    rtl8735b_sdhost.c
 * @brief    This file implements the SD Host HAL functions.
 * @version V1.00
 * @date    2020-10-12
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
#include <assert.h>
#include "rtl8735b.h"
#include "cmsis.h"
#include "hal.h"
#include "hal_timer.h"
#include "hal_pinmux.h"
#include "hal_cache.h"
#include "hal_api.h"

#if CONFIG_SDHOST_EN

// FIXME NSC
// FIXME check ram call rom
// FIXME SetBlkLen for SDSC

#define SDHOST_VERBOSE              0

#define SDHOST_USE_INIT_VAR         1

#if 0
#define SECTION_SDHOST_TEXT           SECTION(".rom.hal_sdhost.text")
#define SECTION_SDHOST_DATA           SECTION(".rom.hal_sdhost.data")
#define SECTION_SDHOST_RODATA         SECTION(".rom.hal_sdhost.rodata")
#define SECTION_SDHOST_BSS            SECTION(".rom.hal_sdhost.bss")
#define SECTION_SDHOST_STUBS          SECTION(".rom.hal_sdhost.stubs")
#else
#define SECTION_SDHOST_TEXT
#define SECTION_SDHOST_DATA
#define SECTION_SDHOST_RODATA
#define SECTION_SDHOST_BSS
#define SECTION_SDHOST_STUBS
#endif

#undef HAL_MODIFY_REG
#define HAL_MODIFY_REG(REG, VAL, MASK) (REG = ((REG & (~(MASK))) | (VAL & MASK)))

#define PINTAB_SIZE             7

#define LDO_VOSEL_3V            0xf
#define SD_HOST_CLK_TIMEOUT     10000       // 10ms
#define GPIO_CTRL_IMPED         0
#define GPIO_CTRL_LOW           1
#define GPIO_CTRL_HIGH          2
#define GPIO_DRIVING_4mA        0
#define GPIO_DRIVING_8mA        1
#define SDIO_LDO_VOADJ_1793V    8           // voltage selection 1.793V
#define SDIO_LDO_VOADJ_1839V    9           // voltage selection 1.839V
#define SDIO_LDO_VOADJ_1887V    10          // voltage selection 1.887V
#define FAILED_PHASE_WEIGHT     0xF000000
#define PHASE_TUNING_2D         0

SECTION_SDHOST_BSS u8 host_inited;

enum SDHOST_LDO_VOL {
	SDHOST_LDO_3V3,
	SDHOST_LDO_1V8
};

void SDHOST_IRQHandler(void);

/**
 * @addtogroup hs_hal_sdhost_ SDIO_HOST
 * @{
 */

/**
  * @brief Pin mux table for SDIO Host
  */
SECTION_SDHOST_RODATA
u32 sdhost_pin_table[PINTAB_SIZE] = {
	PIN_S0, PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S5, PIN_S6
};

/**
  * @brief The global common data structure for SD Host HAL operations.
  */
SECTION_SDHOST_BSS hal_sdhost_adapter_t *psdhost_adapt_saved;


/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_sdhost__rom_func SDIO Host HAL ROM APIs.
 * @ingroup hs_hal_sdhost_
 * @{
 * @brief The SDIO Host HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of SDIO Host HAL APIs in the RAM space is provided for the user application.
 */

/**
  * @brief The stubs functions table to exports SDIO Host HAL functions in ROM.
  */
SECTION_SDHOST_STUBS const hal_sdhost_func_stubs_t hal_sdhost_stubs = {
	.sdhost_pin_table = (io_pin_t *) &sdhost_pin_table[0],
	.hal_sdhost_irq_handler = SDHOST_IRQHandler,
	.hal_sdhost_irq_reg = hal_rtl_sdhost_irq_reg,
	.hal_sdhost_irq_unreg = hal_rtl_sdhost_irq_unreg,
	.hal_sdhost_init_host = hal_rtl_sdhost_init_host,
	.hal_sdhost_init_card = hal_rtl_sdhost_init_card,
	.hal_sdhost_deinit = hal_rtl_sdhost_deinit,
	.hal_sdhost_read_data = hal_rtl_sdhost_read_data,
	.hal_sdhost_write_data = hal_rtl_sdhost_write_data,
	.hal_sdhost_erase = hal_rtl_sdhost_erase,
	.hal_sdhost_stop_transmission = hal_rtl_sdhost_stop_transmission,
	.hal_sdhost_get_card_status = hal_rtl_sdhost_get_card_status,
	.hal_sdhost_get_sd_status = hal_rtl_sdhost_get_sd_status,
	.hal_sdhost_get_scr = hal_rtl_sdhost_get_scr,
	.hal_sdhost_switch_bus_speed = hal_rtl_sdhost_switch_bus_speed,
	.hal_sdhost_get_curr_signal_level = hal_rtl_sdhost_get_curr_signal_level,
	.hal_sdhost_get_supported_speed = hal_rtl_sdhost_get_supported_speed,
	.hal_sdhost_card_insert_hook = hal_rtl_sdhost_card_insert_hook,
	.hal_sdhost_card_remove_hook = hal_rtl_sdhost_card_remove_hook,
	.hal_sdhost_task_yield_hook = hal_rtl_sdhost_task_yield_hook,
	.hal_sdhost_transfer_done_int_hook = hal_rtl_sdhost_transfer_done_int_hook,
};

SECTION_SDHOST_TEXT
static char *sdhost_speed2str(u8 speed)
{
	switch (speed) {
	case SdHostSpeedDS:
		return "DS";
	case SdHostSpeedHS:
		return "HS";
	case SdHostSpeedSDR12:
		return "SDR12";
	case SdHostSpeedSDR25:
		return "SDR25";
	case SdHostSpeedSDR50:
		return "SDR50";
	case SdHostSpeedSDR104:
		return "SDR104";
	case SdHostSpeedDDR50:
		return "DDR50";
	case SdHostKeepCurSpd:
		return "KeepCurSpd";
	}
	return "NA";
};


SECTION_SDHOST_TEXT
void hal_rtl_sdhost_ldo_ctrl(enum SDHOST_LDO_VOL vol_sel)
{
	// Using PMOS buffer
	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_REG_BUFFERTYPE_SEL_L,
				   SYSON_S_BIT_REG_BUFFERTYPE_SEL_L);
	// LDO standby mode: normal mode
	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_REG_STANDBY_L,
				   SYSON_S_MASK_REG_STANDBY_L);

	if (vol_sel == SDHOST_LDO_3V3) {
		// LDO PWRCUT mode (3.3V)
		HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 1 << SYSON_S_SHIFT_EN_PC_BT_L,
					   SYSON_S_BIT_EN_PC_BT_L);
	} else if (vol_sel == SDHOST_LDO_1V8) {
		// LDO mode
		HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_EN_PC_BT_L,
					   SYSON_S_BIT_EN_PC_BT_L);
		// Enable dummy load to accelerate volage drop
		//HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 11 << SYSON_S_SHIFT_REG_DMYLOAD_L,
		//     SYSON_S_MASK_REG_DMYLOAD_L);
		// Voltage adjust to 1.793V
		HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL,
					   SDIO_LDO_VOADJ_1839V << SYSON_S_SHIFT_VOADJ_L,
					   SYSON_S_MASK_VOADJ_L);
	}
	hal_delay_us(20);

	// Enable SDIO LDO
	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_PD_REGU_L,
				   SYSON_S_BIT_PD_REGU_L);
	hal_delay_us(200);

	// Disable diode
	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_EN_DIODE_L,
				   SYSON_S_BIT_EN_DIODE_L);

	if (vol_sel == SDHOST_LDO_1V8) {
		// Need time to drop the LDO voltage
		hal_delay_ms(100);
		// Disable dummy loading after stable
		//HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_LDO_SDIO_CTRL, 0 << SYSON_S_SHIFT_REG_DMYLOAD_L,
		//         SYSON_S_MASK_REG_DMYLOAD_L);
	}
}

SECTION_SDHOST_TEXT
void hal_rtl_sdhost_en_ctrl(BOOL enable)
{
	BOOL en = (enable) ? 1 : 0;
	HAL_SET_BIT(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL, en << SYSON_S_SHIFT_SYS_SD_SYST_100M_EN);
	u32 wait_us = 0;

	if (en) {
		while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_RDY) == 0) {
			hal_delay_us(1);
			wait_us++;
			if (wait_us > SD_HOST_CLK_TIMEOUT) {
				DBG_SDHOST_ERR("Wait 100M clock ready timeout !!\r\n");
				break;
			}
		}
	}
	HAL_SET_BIT(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL, en << SYSON_S_SHIFT_SYS_SDH_EN);
	HAL_SET_BIT(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL, en << SYSON_S_SHIFT_SYS_SD_SYST_PCLK_EN);
	HAL_SET_BIT(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL, en << SYSON_S_SHIFT_SYS_SD_SYST_SCLK_EN);
}

SECTION_SDHOST_TEXT
static hal_status_t sdhost_chk_cmd_data_state(SDHOST_Type *psdhost_reg)
{
	SDHOST_Type *psdhost = psdhost_reg;

	// check the SD bus status
	if (((psdhost->sd_bus_status) & SDHOST_BUS_STS_MASK) != 0x1F) {
		DBG_SDHOST_ERR("SD bus isn't in the idle state: 0x%x!!\r\n", psdhost->sd_bus_status);
		return HAL_BUSY;
	}

	// check the CMD & DATA state machine
	if (psdhost->sd_cmd_ste_b.cmd_ste_is_idle == 0) {
		DBG_SDHOST_ERR("CMD state machine 0x%x isn't in the idle state!!\r\n", psdhost->sd_cmd_ste);
		return HAL_BUSY;
	}

	if (psdhost->sd_data_ste_b.data_ste_is_idle == 0) {
		DBG_SDHOST_ERR("DATA state machine 0x%x isn't in the idle state!!\r\n", psdhost->sd_data_ste);
		return HAL_BUSY;
	}

	// check the SD card module state machine
	if ((psdhost->sd_xfer_b.idle_ste) == 0) {
		DBG_SDHOST_ERR("SD card module state machine 0x%x isn't in the idle state !!\r\n", psdhost->sd_xfer_b.idle_ste);
		return HAL_BUSY;
	}
	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_chk_xfer_error(SDHOST_Type *psdhost_reg)
{
	SDHOST_Type *psdhost = psdhost_reg;


	// read the error bit
	if (psdhost->sd_xfer_b.err) {
		DBG_SDHOST_ERR("Transfer error(s) occur !! (0x583 = %02X, 0x584 = %02X)\r\n", psdhost->sd_status1, psdhost->sd_status2);
		if (psdhost->sd_status1_b.tune_patrn_err) {
			return HAL_ERR_HW;
		} else {
			return HAL_ERR_UNKNOWN;
		}
	} else {
		return HAL_OK;
	}
}

// Pull GPIOS except SD_CLK, SD_CD
SECTION_SDHOST_TEXT
static void gpio_pull_ctrl(hal_sdhost_adapter_t *psdhost_adapter, u8 level)
{
	// ignore GPIOS_0 (SD_CLK)  GPIOS_4 (SD_CD)
	level &= 0x3;
	// GPIOS 1
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_0_1_CTRL,
				   level << SYSON_SHIFT_SYSON_GPIO1_PULL_CTRL, SYSON_MASK_SYSON_GPIO1_PULL_CTRL);
	// GPIOS 2, 3
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL,
				   level << SYSON_SHIFT_SYSON_GPIO2_PULL_CTRL, SYSON_MASK_SYSON_GPIO2_PULL_CTRL);
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL,
				   level << SYSON_SHIFT_SYSON_GPIO3_PULL_CTRL, SYSON_MASK_SYSON_GPIO3_PULL_CTRL);
	// GPIOS 5
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_4_5_CTRL,
				   level << SYSON_SHIFT_SYSON_GPIO5_PULL_CTRL, SYSON_MASK_SYSON_GPIO5_PULL_CTRL);
	// GPIOS 6
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_6_CTRL,
				   level << SYSON_SHIFT_SYSON_GPIO6_PULL_CTRL, SYSON_MASK_SYSON_GPIO6_PULL_CTRL);
}

SECTION_SDHOST_TEXT
static void gpio_driving_ctrl(hal_sdhost_adapter_t *psdhost_adapter, u8 driving)
{
	// ignore GPIOS_4 (SD_CD)
	driving &= 1;
	// GPIOS 0,1
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_0_1_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO0_DRIVING, SYSON_MASK_SYSON_GPIO0_DRIVING);
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_0_1_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO1_DRIVING, SYSON_MASK_SYSON_GPIO1_DRIVING);
	// GPIOS 2,3
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO2_DRIVING, SYSON_MASK_SYSON_GPIO2_DRIVING);
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_2_3_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO3_DRIVING, SYSON_MASK_SYSON_GPIO3_DRIVING);
	// GPIOS 5
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_4_5_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO5_DRIVING, SYSON_MASK_SYSON_GPIO5_DRIVING);
	// GPIOS 6
	HAL_MODIFY_REG(SYSON->SYSON_REG_SYSON_GPIOS_6_CTRL,
				   driving << SYSON_SHIFT_SYSON_GPIO6_DRIVING, SYSON_MASK_SYSON_GPIO6_DRIVING);
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_chk_bus_status(hal_sdhost_adapter_t *psdhost_adapter, u8 status, u32 timeout_us)
{
	SDHOST_Type *psdhost;
	u32 start_us;
	u16 expected_status;

#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1) // Larger timeout for FPGA which is slower
	timeout_us = timeout_us * 100;
#endif

	if (psdhost_adapter == NULL) {
		DBG_SDHOST_WARN("psdhost_adapter is NULL !!\r\n");
		return HAL_ERR_PARA;
	}
	psdhost = psdhost_adapter->base_addr;

	if (timeout_us == 0) {
		return HAL_OK;
	}

	if ((timeout_us != 0) && (timeout_us != HAL_WAIT_FOREVER)) {
		start_us = hal_read_curtime_us();
	}

	if (status == SdHostBusHigh) {
		expected_status = 0x1F;
	} else if (status == SdHostBusLow) {
		expected_status = 0x00;
	} else {
		return HAL_ERR_PARA;
	}

	do {
		if ((psdhost->sd_bus_status & SDHOST_BUS_STS_MASK) == expected_status) {
			break;
		} else {
			if ((timeout_us != HAL_WAIT_FOREVER) && hal_is_timeout(start_us, timeout_us)) {
				const char *kind = status == SdHostBusHigh ? "Wait bus high" : "Wait bus low";
				DBG_SDHOST_ERR("sdhost_chk_bus_status(): %s timeout !!\r\n", kind);
				DBG_SDHOST_INFO("Bus status: 0x%x expected: 0x%x\r\n", psdhost->sd_bus_status & SDHOST_BUS_STS_MASK, expected_status);
				return HAL_TIMEOUT;
			}
		}
	} while (1);

	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_initial_mode_ctrl(hal_sdhost_adapter_t *psdhost_adapter, BOOL en)
{
	hal_sdhost_adapter_t *psdhost_adp = psdhost_adapter;
	SDHOST_Type *psdhost = psdhost_adp->base_addr;
	hal_status_t ret = HAL_OK;


	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	if (en) {
		psdhost->ckgen_ctl = 0x2100;
		psdhost->sd_config1_b.mode_sel = SdHostModeSd20;
		psdhost->sd_config1_b.clk_div = SdHostSdclkDiv256;  // 390.625 KHz
		psdhost->sd_config1_b.initial_mode = 1;
		psdhost->sd_config1_b.sd30_async_fifo_rst = 1;
		psdhost->sd_config1_b.bus_width = SdHostBus1bit;

		psdhost_adp->curr_sig_level = SdHostSigVol33;
		psdhost_adp->curr_bus_spd = SdHostSpeedDS;
	} else {
		/* Switch to DS mode (3.3V) or SDR12 (1.8V) */
		if ((psdhost_adp->curr_sig_level) == SdHostSigVol18) {
			psdhost->ckgen_ctl = 0x0002;  // 25  MHz
			psdhost->sd_config1_b.mode_sel = SdHostModeSd30;
			psdhost->sd_config1_b.clk_div = SdHostSdclkDiv128;
			psdhost->sd_config1_b.initial_mode = 0;
			psdhost->sd_config1_b.sd30_async_fifo_rst = 1;
			psdhost->sd_config1_b.bus_width = SdHostBus1bit;

			psdhost_adp->curr_bus_spd = SdHostSpeedSDR12;
		} else {
			psdhost->ckgen_ctl = 0x0001;  // 25 MHz
			psdhost->sd_config1_b.mode_sel = SdHostModeSd20;
			psdhost->sd_config1_b.clk_div = SdHostSdclkDiv128;
			psdhost->sd_config1_b.initial_mode = 0;
			psdhost->sd_config1_b.bus_width = SdHostBus1bit;

			psdhost_adp->curr_bus_spd = SdHostSpeedDS;
		}
	}


	return ret;
}


SECTION_SDHOST_TEXT
static void sdhost_config_dma(hal_sdhost_adapter_t *psdhost_adapter, hal_sdhost_dma_ctrl_t *dma_ctl)
{
	hal_sdhost_adapter_t *psdhost_adp = psdhost_adapter;
	SDHOST_Type *psdhost = psdhost_adp->base_addr;


	if ((dma_ctl->type) == SdHostDmaNormal) {
		psdhost->sd_byte_cnt_l = 0;
		psdhost->sd_byte_cnt_h_b.byte_cnt_h = 2;
	} else {
		psdhost->sd_byte_cnt_l = SDHOST_C6R2_BUF_LEN;
		psdhost->sd_byte_cnt_h_b.byte_cnt_h = 0;
	}
	psdhost->sd_blk_cnt_l = (dma_ctl->blk_cnt) & 0xFF;
	psdhost->sd_blk_cnt_h_b.blk_cnt_h = ((dma_ctl->blk_cnt) >> 8) & 0x7F;
	// DMA start address (unit: 8 Bytes)
	psdhost->dma_ctl1_b.dram_sa = (dma_ctl->start_addr) & 0xFFFFFFF;
	// DMA transfer length (unit: 512 Bytes)
	psdhost->dma_ctl2_b.dma_len = dma_ctl->blk_cnt;
	if ((dma_ctl->type) == SdHostDma64b) {
		psdhost->dma_ctl3_b.dat64_sel = 1;
		psdhost->dma_ctl3_b.rsp17_sel = 0;
	} else if ((dma_ctl->type) == SdHostDmaR2) {
		psdhost->dma_ctl3_b.dat64_sel = 0;
		psdhost->dma_ctl3_b.rsp17_sel = 1;
	} else {
		psdhost->dma_ctl3_b.dat64_sel = 0;
		psdhost->dma_ctl3_b.rsp17_sel = 0;
	}

	// clear pending interrupt
	psdhost->sd_isr = 0x16;
	// set the DMA operation
	psdhost->dma_ctl3_b.ddr_wr = dma_ctl->op;
	// set the DMA transfer bit
	psdhost->dma_ctl3_b.dma_xfer = 1;

}


SECTION_SDHOST_TEXT
static void sdhost_reset_dma_settings(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp = psdhost_adapter;
	SDHOST_Type *psdhost = psdhost_adp->base_addr;


	psdhost->sd_byte_cnt_l = 0;
	psdhost->sd_byte_cnt_h_b.byte_cnt_h = 0;
	psdhost->sd_blk_cnt_l = 0;
	psdhost->sd_blk_cnt_h_b.blk_cnt_h = 0;
	psdhost->dma_ctl1_b.dram_sa = 0;
	psdhost->dma_ctl2_b.dma_len = 0;
	psdhost->dma_ctl3 = 0;
	psdhost_adp->xfer_int_sts = 0;
}


SECTION_SDHOST_TEXT
static void sdhost_send_cmd(hal_sdhost_adapter_t *psdhost_adapter, hal_sdhost_cmd_attr_t *cmd_attrib)
{
	hal_sdhost_adapter_t *psdhost_adp = psdhost_adapter;
	SDHOST_Type *psdhost = psdhost_adp->base_addr;

	// set SD_CONFIGURE2 (0x581)
	psdhost->sd_config2_b.crc7_cal = 0;
	if (((cmd_attrib->idx) == SdHostCmdRdSingleBlk) || ((cmd_attrib->idx) == SdHostCmdRdMulBlk) ||
		((cmd_attrib->idx) == SdHostCmdSendTuningBlk)) {
		psdhost->sd_config2_b.crc16_chk = 0;
	} else {
		psdhost->sd_config2_b.crc16_chk = 1;
	}
	if (((cmd_attrib->idx) == SdHostCmdWrBlk) || ((cmd_attrib->idx) == SdHostCmdWrMulBlk)) {
		psdhost->sd_config2_b.wait_crc_sts_timeout = 0;
		psdhost->sd_config2_b.ignore_crc_sts_err = 0;
	} else {
		psdhost->sd_config2_b.wait_crc_sts_timeout = 1;
		psdhost->sd_config2_b.ignore_crc_sts_err = 1;
	}
	if ((cmd_attrib->idx) == SdHostCmdVolSwitch) {
		psdhost->sd_config2_b.wait_busy_end = 0;
	} else {
		psdhost->sd_config2_b.wait_busy_end = 1;

	}
	if (cmd_attrib->rsp_crc_chk) {
		psdhost->sd_config2_b.crc7_chk = 0;
	} else {
		psdhost->sd_config2_b.crc7_chk = 1;
	}
	psdhost->sd_config2_b.rsp_type = cmd_attrib->rsp_type;

	// set SD_CONFIGURE3 (0x582)
	psdhost->sd_config3_b.stop_cmd_start_wait_card_idle = 0;
	if (((cmd_attrib->idx) == SdHostCmdRdMulBlk) || ((cmd_attrib->idx) == SdHostCmdWrMulBlk) ||
		((cmd_attrib->idx) == SdHostCmdStopXsmission)) {
		psdhost->sd_config3_b.cmd_start_wait_card_idle = 1;
	} else {
		psdhost->sd_config3_b.cmd_start_wait_card_idle = 0;
	}
	if (((cmd_attrib->idx) == SdHostCmdVolSwitch) || ((cmd_attrib->idx) == SdHostCmdStopXsmission)) {
		psdhost->sd_config3_b.wait_card_idle = 0;
	} else {
		psdhost->sd_config3_b.wait_card_idle = 1;
	}
	psdhost->sd_config3_b.sd30_clk_stop = 0;
	psdhost->sd_config3_b.sd20_clk_stop = 0;
	psdhost->sd_config3_b.rsp_chk = 0;
	psdhost->sd_config3_b.addr_mode = 0;
	if ((cmd_attrib->rsp_type) == SdHostNoRsp) {
		psdhost->sd_config3_b.rsp_timeout_en = 0;
	} else {
		psdhost->sd_config3_b.rsp_timeout_en = 1;
	}

	// fill the command register
	psdhost->sd_cmd0 = BIT6 | ((cmd_attrib->idx) & 0x3F);
	psdhost->sd_cmd1 = ((cmd_attrib->arg) >> 24) & 0xFF;
	psdhost->sd_cmd2 = ((cmd_attrib->arg) >> 16) & 0xFF;
	psdhost->sd_cmd3 = ((cmd_attrib->arg) >> 8) & 0xFF;
	psdhost->sd_cmd4 = (cmd_attrib->arg) & 0xFF;
	psdhost->sd_cmd5 = 0x0;

	// set the command code
	switch (cmd_attrib->idx) {
	case SdHostCmdSwitchFunc:
		if ((cmd_attrib->data_present) == SdHostNoDataPresent) {
			psdhost->sd_xfer_b.cmd_code = SdHostCmdSendCmdGetRsp;
		} else {
			psdhost->sd_xfer_b.cmd_code = SdHostCmdNormalRd;
		}
		break;
	case SdHostCmdSendSts:
		if ((cmd_attrib->data_present) == SdHostNoDataPresent) {
			psdhost->sd_xfer_b.cmd_code = SdHostCmdSendCmdGetRsp;
		} else {
			psdhost->sd_xfer_b.cmd_code = SdHostCmdNormalRd;
		}
		break;
	case SdHostCmdSendTuningBlk:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdTuning;
		break;
	case SdHostCmdRdSingleBlk:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdAutoRd2;
		break;
	case SdHostCmdRdMulBlk:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdAutoRd1;
		break;
	case SdHostCmdWrBlk:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdAutoWr2;
		break;
	case SdHostCmdWrMulBlk:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdAutoWr1;
		break;
	case SdHostCmdSendScr:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdNormalRd;
		break;
	default:
		psdhost->sd_xfer_b.cmd_code = SdHostCmdSendCmdGetRsp;
	}

	// start to transfer
	psdhost->sd_xfer_b.start = 1;
#if defined(SDHOST_VERBOSE) && SDHOST_VERBOSE == 1
	DBG_SDHOST_INFO("Send CMD %u\r\n", cmd_attrib->idx);
#endif
}

SECTION_SDHOST_TEXT
static hal_status_t sdhost_host_presetting(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp = psdhost_adapter;
	SDHOST_Type *psdhost = psdhost_adp->base_addr;
	hal_status_t ret = HAL_OK;

	// Top register setting
	// init status for newly pluged-in card
	hal_rtl_sdhost_ldo_ctrl(SDHOST_LDO_3V3);
	gpio_pull_ctrl(psdhost_adapter, GPIO_CTRL_HIGH);
	gpio_driving_ctrl(psdhost_adapter, GPIO_DRIVING_4mA);

	// map_sel
	psdhost->sram_ctl_b.map_sel = 1;
	// Lexra burst size = 64 Bytes
	psdhost->sram_ctl_b.lx_burst_size = 0;
	// SD module
	psdhost->card_select_b.card_sel = 2;
	// stop the transfer & the transfer state machine returns to idle state
	psdhost->card_stop_b.sd_module = 1;
	// Initial mode
	ret = sdhost_initial_mode_ctrl(psdhost_adp, ON);
	if (ret != HAL_OK) {
		return ret;
	}

	// enable SD card module clock
	psdhost->card_clk_en_ctl = 0x4;


	return ret;
}

extern const hal_timer_func_stubs_t hal_gtimer_stubs;

SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_chk_cmd_complete(hal_sdhost_adapter_t *psdhost_adapter, u32 timeout_us)
{
	SDHOST_Type *psdhost;
	u32 start_us;

#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1) // Larger timeout for FPGA which is slower
	timeout_us = timeout_us * 100;
#endif
	if (psdhost_adapter == NULL) {
		DBG_SDHOST_WARN("psdhost_adapter is NULL !!\r\n");
		return HAL_ERR_PARA;
	}
	psdhost = psdhost_adapter->base_addr;

	if ((timeout_us != 0) && (timeout_us != HAL_WAIT_FOREVER)) {
		start_us = hal_read_curtime_us();
	}

	if (timeout_us == 0) {
		// FIXME why direct ok
		return HAL_OK;
	}

	do {
		if (((psdhost->sd_xfer_b.end) && (psdhost->sd_xfer_b.idle_ste))) {
			break;
		} else {
			if ((timeout_us != HAL_WAIT_FOREVER) && hal_is_timeout(start_us, timeout_us)) {
				DBG_SDHOST_ERR("hal_rtl_sdhost_chk_cmd_complete(): wait timeout !!\r\n");
				DBG_SDHOST_ERR("t_out=%u\r\n", timeout_us);
				DBG_SDHOST_ERR("now  =%u\r\n", hal_read_curtime_us());
				return HAL_TIMEOUT;
#if 0
			} else {
				if (psdhost_adapter->task_yield_cb != NULL) {
					(psdhost_adapter->task_yield_cb)();
				}
#endif
			}
		}
	} while (1);

	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t hal_rtl_sdhost_chk_xfer_complete(hal_sdhost_adapter_t *psdhost_adapter, u32 timeout_us)
{
	SDHOST_Type *psdhost;
	u32 start_us;

#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1) // Larger timeout for FPGA which is slower
	timeout_us = timeout_us * 100;
#endif

	if (psdhost_adapter == NULL) {
		DBG_SDHOST_WARN("psdhost_adapter is NULL !!\r\n");
		return HAL_ERR_PARA;
	}
	psdhost = psdhost_adapter->base_addr;

	if ((timeout_us != 0) && (timeout_us != HAL_WAIT_FOREVER)) {
		start_us = hal_read_curtime_us();
	}

	if (timeout_us == 0) {
		sdhost_reset_dma_settings(psdhost_adapter);
		return HAL_OK;
	}

	do {
		if ((psdhost->sd_xfer_b.end) && (!(psdhost->dma_ctl3_b.dma_xfer))) {
			sdhost_reset_dma_settings(psdhost_adapter);
			break;
		} else {
			if ((timeout_us != HAL_WAIT_FOREVER) && hal_is_timeout(start_us, timeout_us)) {
				DBG_SDHOST_ERR("hal_rtl_sdhost_chk_xfer_complete(): wait timeout !!\r\n");
				DBG_SDHOST_INFO("start time=%u\r\n", start_us);
				DBG_SDHOST_ERR("timeout_us=%u, now=%u\r\n", timeout_us, hal_read_curtime_us());
				return HAL_TIMEOUT;
#if 0
			} else {
				if (psdhost_adapter->task_yield_cb != NULL) {
					(psdhost_adapter->task_yield_cb)();
				}
#endif
			}
		}
	} while (1);

	return HAL_OK;
}

SECTION_SDHOST_TEXT
static hal_status_t hal_rtl_sdhost_chk_xfer_int(hal_sdhost_adapter_t *psdhost_adapter, u32 timeout_us)
{
	u32 start_us;

#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1) // Larger timeout for FPGA which is slower
	timeout_us = timeout_us * 100;
#endif

	if (psdhost_adapter == NULL) {
		DBG_SDHOST_WARN("psdhost_adapter is NULL !!\r\n");
		return HAL_ERR_PARA;
	}

	if ((timeout_us != 0) && (timeout_us != HAL_WAIT_FOREVER)) {
		start_us = hal_read_curtime_us();
	}

	if (timeout_us == 0) {
		sdhost_reset_dma_settings(psdhost_adapter);
		return HAL_OK;
	}

	do {
		//if ((psdhost->sd_xfer_b.end) && (!(psdhost->dma_ctl3_b.dma_xfer))) {
		if (psdhost_adapter->wait_interrupts == 0) {
			sdhost_reset_dma_settings(psdhost_adapter);
			break;
		} else {
			if ((timeout_us != HAL_WAIT_FOREVER) && hal_is_timeout(start_us, timeout_us)) {
				DBG_SDHOST_ERR("hal_rtl_sdhost_chk_xfer_int(): wait timeout !!\r\n");
				DBG_SDHOST_ERR("start time=%u\r\n", start_us);
				DBG_SDHOST_ERR("timeout_us=%u, now=%u\r\n", timeout_us, hal_read_curtime_us());

#if 1 // FIXME
				SDHOST_Type *psdhost = psdhost_adapter->base_addr;
				DBG_SDHOST_INFO("sd register: sd_xfer: %x, dma_ctl3: %x\r\n",
								psdhost->sd_xfer, psdhost->dma_ctl3);
				DBG_SDHOST_INFO("interrupt indicator: %x\r\n", psdhost_adapter->wait_interrupts);
#endif
				return HAL_TIMEOUT;
			} else {
				if (psdhost_adapter->task_yield_cb != NULL) {
					(psdhost_adapter->task_yield_cb)(psdhost_adapter->task_yield_cb_para);
				}
			}
		}
	} while (1);

	return HAL_OK;
}

SECTION_SDHOST_TEXT
static hal_status_t sdhost_reset_card(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD0 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdGoIdleSte;
	cmd_attr.rsp_type = SdHostNoRsp;
	cmd_attr.rsp_crc_chk = DISABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_voltage_check(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}

	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD8 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = (SDHOST_CMD8_VHS << 8) | SDHOST_CMD8_CHK_PATN;
	cmd_attr.idx = SdHostCmdSendIfCond;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		DBG_SDHOST_ERR("sd_status1=%x. sd_status2=%x.\r\n", psdhost->sd_status1, psdhost->sd_status2);
		if (psdhost->sd_status2_b.rsp_timeout_err) {
			/* for Ver1.x SD card*/
			psdhost_adp->voltage_mismatch = 1;
			return HAL_OK;
		} else {
			return ret;
		}
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendIfCond) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// check the echo-back of check pattern
	if ((psdhost->sd_cmd4) != SDHOST_CMD8_CHK_PATN) {
		DBG_SDHOST_ERR("Check pattern error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}
	// check the VHS
	if (((psdhost->sd_cmd3) & 0xF) != SDHOST_CMD8_VHS) {
		DBG_SDHOST_ERR("Voltage accepted error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}
	psdhost_adp->voltage_mismatch = 0;


	return HAL_OK;
}


// TODO check with SPEC
SECTION_SDHOST_TEXT
static hal_status_t sdhost_voltage_switch(hal_sdhost_adapter_t *psdhost_adapter)
{
	DBG_SDHOST_INFO("sdhost_voltage_switch\r\n");
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}

	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	// send continuous clock before CMD11
	psdhost->sd_bus_status_b.sdclk_toggle = 1;
	psdhost->sd_bus_status_b.stop_sdclk_when_no_xfer = 0;

	// Voltage Switch Sequence Steps

	// 1. send the command
	/***** CMD11 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdVolSwitch;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		if (ret == HAL_TIMEOUT) {
			DBG_SDHOST_ERR("Voltage switch command timeout");
		}
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// 2. card returns R1 response, check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdVolSwitch) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// 3. wait for CMD & DAT[3:0] are drived low by card
	ret = sdhost_chk_bus_status(psdhost_adp, SdHostBusLow, SDHOST_BUS_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// 4. disable SDCLK (host prepare 1.8V GPIO)
	// GPIO pull low
	gpio_pull_ctrl(psdhost_adapter, GPIO_CTRL_LOW);
	// disable clock, stop SDCLK gen when no transaction on CMD/DAT bus
	psdhost->sd_bus_status_b.sdclk_toggle = 0;
	psdhost->sd_bus_status_b.stop_sdclk_when_no_xfer = 1;

	// 5. wait 5ms for host and card voltage regulator stable
	// switch GPIOS voltage to 1.8V
	hal_rtl_sdhost_ldo_ctrl(SDHOST_LDO_1V8);
	hal_delay_ms(5);

	// 6. enable SDCLK @ 1.8V
	// SDCLK pull high
	gpio_pull_ctrl(psdhost_adapter, GPIO_CTRL_HIGH);

	// enable SD card clock, generate SDCLK even there is no transaction on CMD/DAT bus
	psdhost->sd_bus_status_b.sdclk_toggle = 1;
	psdhost->sd_bus_status_b.stop_sdclk_when_no_xfer = 0;
	hal_delay_ms(1);

	// 7.8.9. wait for CMD & DAT[3:0] are high
	ret = sdhost_chk_bus_status(psdhost_adp, SdHostBusHigh, SDHOST_BUS_TIMEOUT);
	if (ret != HAL_OK) {
		psdhost->sd_bus_status_b.sdclk_toggle = 0;
		return ret;
	}

	psdhost->sd_bus_status_b.sdclk_toggle = 0;

	// GPIO S driving select maximum
	gpio_driving_ctrl(psdhost_adapter, GPIO_DRIVING_8mA);
	DBG_SDHOST_INFO("Switch to 1.8V successfully\r\n");

	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_get_ocr(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;
	u32 retry = (SDHOST_ACMD41_TIMEOUT / SDHOST_ACMD41_POLL_INTERVAL);


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** ACMD41 *****/
	// Inquiry ACMD41 (CMD55)
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdAppCmd;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// check the APP_CMD
	if (!((psdhost->sd_cmd4) & BIT5)) {
		DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// Inquiry ACMD41 (CMD41)
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// TODO check SD card supports 3.3


	if (psdhost_adp->voltage_mismatch) {
		cmd_attr.arg = (SdHostSupportSdscOnly << 30) | (SdHostMaxPerformance << 28) | (SdHostUseCurrSigVol << 24) |
					   SDHOST_OCR_VDD_WIN;  // 3.3V
	} else {
		if (psdhost_adp->force_33v) {
			cmd_attr.arg = (SdHostSupportSdhcSdxc << 30) | (SdHostPwrSaving << 28) | (SdHostUseCurrSigVol << 24) |
						   SDHOST_OCR_VDD_WIN;  // 3.3V
		} else {
			cmd_attr.arg = (SdHostSupportSdhcSdxc << 30) | (SdHostMaxPerformance << 28) | (SdHostSwitch18v << 24) |
						   SDHOST_OCR_VDD_WIN;  // 1.8V
		}
	}
	cmd_attr.idx = SdHostCmdSdSendOpCond;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = DISABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	while (retry--) {
		// Normal ACMD41 (CMD55)
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			break;
		}

		cmd_attr.arg = 0;
		cmd_attr.idx = SdHostCmdAppCmd;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			break;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			break;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			ret = HAL_ERR_UNKNOWN;
			break;
		}

		// check the APP_CMD
		if (!((psdhost->sd_cmd4) & BIT5)) {
			DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
			ret = HAL_ERR_UNKNOWN;
			break;
		}

		// Normal ACMD41 (CMD41)
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			break;
		}

		if (psdhost_adp->voltage_mismatch) {
			cmd_attr.arg = (SdHostSupportSdscOnly << 30) | (SdHostMaxPerformance << 28) | (SdHostUseCurrSigVol << 24) |
						   SDHOST_OCR_VDD_WIN;  // 3.3V
		} else {
			if (psdhost_adp->force_33v) {
				cmd_attr.arg = (SdHostSupportSdhcSdxc << 30) | (SdHostMaxPerformance << 28) | (SdHostUseCurrSigVol << 24) |
							   SDHOST_OCR_VDD_WIN;  // 1.8V
			} else {
				cmd_attr.arg = (SdHostSupportSdhcSdxc << 30) | (SdHostMaxPerformance << 28) | (SdHostSwitch18v << 24) |
							   SDHOST_OCR_VDD_WIN;  // 1.8V
			}
		}
		cmd_attr.idx = SdHostCmdSdSendOpCond;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = DISABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			break;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			break;
		}

		// check busy bit of OCR, check if card initialization is complete
		if ((psdhost->sd_cmd1) & BIT7) {
			break;
		}

		hal_delay_us(SDHOST_ACMD41_POLL_INTERVAL);
	}

	if (ret != HAL_OK) {
		return ret;
	}
	if (!retry) {
		DBG_SDHOST_WARN("card busy initialization timeout\r\n");
		return HAL_TIMEOUT;
	}

	// check CCS bit
	if ((psdhost->sd_cmd1) & BIT6) {
		psdhost_adp->is_sdhc_sdxc = 1;
		DBG_SDHOST_INFO("This is a SDHC/SDXC card\r\n");
		// check S18A bit
		if ((psdhost->sd_cmd1) & BIT0) {
			DBG_SDHOST_INFO("Card accepted 1.8v switch\r\n");
			psdhost_adp->is_s18a = 1;
			DBG_SDHOST_INFO("Start voltage switch\r\n");
			ret = sdhost_voltage_switch(psdhost_adp);
			if (ret != HAL_OK) {
				psdhost_adp->curr_sig_level = SdHostSigVol33;
				psdhost_adp->curr_bus_spd = SdHostSpeedDS;
				DBG_SDHOST_INFO("Voltage switch failed, need to power cycled card\r\n");
				return ret;
			}
			psdhost_adp->curr_sig_level = SdHostSigVol18;
			psdhost_adp->curr_bus_spd = SdHostSpeedSDR12;
		} else {
			psdhost_adp->is_s18a = 0;
			psdhost_adp->curr_sig_level = SdHostSigVol33;
			psdhost_adp->curr_bus_spd = SdHostSpeedDS;
			if (psdhost_adp->force_33v) {
				DBG_SDHOST_INFO("Host forced keep 3.3V, card S18A=0\r\n");
			} else {
				DBG_SDHOST_INFO("Card rejected 1.8v switch, card S18A=0\r\n");
			}
		}
	} else {
		psdhost_adp->is_sdhc_sdxc = 0;
		psdhost_adp->is_s18a = 0;
		psdhost_adp->curr_sig_level = SdHostSigVol33;
		psdhost_adp->curr_bus_spd = SdHostSpeedDS;
		DBG_SDHOST_INFO("This is a SDSC card\r\n");
	}


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_get_cid(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD2 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)(psdhost_adp->c6r2_buf), 0, SDHOST_C6R2_BUF_LEN);
	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)(psdhost_adp->c6r2_buf)) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDmaR2;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdAllSendCid;
	cmd_attr.rsp_type = SdHostRsp17Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_SMALL_XFER_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

#if 1
	DBG_SDHOST_INFO("Manufacturer ID: %d\r\n", psdhost_adp->c6r2_buf[1]);
	DBG_SDHOST_INFO("OEM/Application ID: %c%c\r\n", psdhost_adp->c6r2_buf[2], psdhost_adp->c6r2_buf[3]);
	DBG_SDHOST_INFO("Product name: %c%c%c%c%c\r\n", psdhost_adp->c6r2_buf[4], psdhost_adp->c6r2_buf[5], psdhost_adp->c6r2_buf[6],
					psdhost_adp->c6r2_buf[7], psdhost_adp->c6r2_buf[8]);
	DBG_SDHOST_INFO("Product serial number: %02X%02X%02X%02X\r\n", psdhost_adp->c6r2_buf[10], psdhost_adp->c6r2_buf[11],
					psdhost_adp->c6r2_buf[12], psdhost_adp->c6r2_buf[13]);
	DBG_SDHOST_INFO("Manufacturing date: %d/%d (Year/Month)\r\n",
					2000 + (((psdhost_adp->c6r2_buf[14] & 0xF) << 4) | (psdhost_adp->c6r2_buf[15] >> 4)), psdhost_adp->c6r2_buf[15] & 0xF);
#endif


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_get_rca(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD3 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdSendRelAddr;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendRelAddr) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// get RCA
	psdhost_adp->rca = ((psdhost->sd_cmd1) << 8) | (psdhost->sd_cmd2);
	DBG_SDHOST_INFO("RCA = %04X\r\n", psdhost_adp->rca);


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_get_csd(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD9 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)(psdhost_adp->c6r2_buf), 0, SDHOST_C6R2_BUF_LEN);
	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)(psdhost_adp->c6r2_buf)) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDmaR2;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = (psdhost_adp->rca) << 16;
	cmd_attr.idx = SdHostCmdSendCsd;
	cmd_attr.rsp_type = SdHostRsp17Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_SMALL_XFER_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memcpy((u8 *)psdhost_adp->csd, psdhost_adp->c6r2_buf + SD_RESP_HDR_LEN, SD_CSD_LEN);

	// Byte reverse csd data
	for (int i = 0 ; i < (SD_CSD_LEN / 2); i++) {
		u8 *low = (u8 *)(psdhost_adp->csd) + i;
		u8 *high = (u8 *)(psdhost_adp->csd) + (SD_CSD_LEN - i - 1);
		u8 tmp = *low;
		*low = *high;
		*high = tmp;
	}

	if (((sd_csd_v2_t *)psdhost_adp->csd)->csd_structure == 1) {
		sd_csd_v2_t *csd = (sd_csd_v2_t *)psdhost_adp->csd;
		DBG_SDHOST_INFO("CSD Version: 2.0\r\n");
		DBG_SDHOST_INFO("Max. read data block length: %d Bytes\r\n", 1 << csd->read_bl_len);
		DBG_SDHOST_INFO("Max. write data block length: %d Bytes\r\n", 1 << csd->write_bl_len);
		u32 block_capacity = (csd->c_size_1 | (csd->c_size_2 << 16)) * 1024;
		DBG_SDHOST_INFO("User data area capacity: %d MB\r\n", block_capacity / (1024 * 1024 / 512));
	} else {
		DBG_SDHOST_INFO("CSD Version: 1.0\r\n");
		u32 block_nr, mult, block_len;
		sd_csd_v1_t *csd = (sd_csd_v1_t *)psdhost_adp->csd;
		DBG_SDHOST_INFO("Max. read data block length: %d Bytes\r\n", 1 << csd->read_bl_len);
		DBG_SDHOST_INFO("Max. write data block length: %d Bytes\r\n", 1 << csd->write_bl_len);
		mult = 1 << (csd->c_size_mult + 2);
		block_nr = mult * ((csd->c_size_1 | (csd->c_size_2 << 2)) + 1);
		block_len = 1 << csd->read_bl_len;
		u32 block_capacity = block_nr * block_len / 512;
		DBG_SDHOST_INFO("User data area capacity: %d MB\r\n", block_capacity / (1024 * 1024 / 512));
	}
	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_card_select(hal_sdhost_adapter_t *psdhost_adapter, u8 select)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD7 *****/
	if (select == SdHostSelCard) {
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		cmd_attr.arg = (psdhost_adp->rca) << 16;
		cmd_attr.idx = SdHostCmdSelDeselCard;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSelDeselCard) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}
	} else {
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		cmd_attr.arg = 0;
		cmd_attr.idx = SdHostCmdSelDeselCard;
		cmd_attr.rsp_type = SdHostNoRsp;
		cmd_attr.rsp_crc_chk = DISABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}
	}


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_set_bus_width(hal_sdhost_adapter_t *psdhost_adapter, u8 bus_width)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;


	if ((psdhost_adapter == NULL) || (bus_width > SdHostBus4bit)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	if ((psdhost->sd_config1_b.bus_width) == bus_width) {
		DBG_SDHOST_INFO("Current SD bus width is already the specified setting\r\n");
		return HAL_OK;
	}

	/***** ACMD6 (CMD55) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = (psdhost_adp->rca) << 16;
	cmd_attr.idx = SdHostCmdAppCmd;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// check the APP_CMD
	if (!((psdhost->sd_cmd4) & BIT5)) {
		DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	/***** ACMD6 (CMD6) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	if (bus_width == SdHostBus4bit) {
		cmd_attr.arg = 0x2;  // 4-bit bus
	} else {
		cmd_attr.arg = 0x0;  // 1-bit bus
	}
	cmd_attr.idx = SdHostCmdSetBusWidth;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSetBusWidth) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// Host also selects the specified mode
	psdhost->sd_config1_b.bus_width = bus_width;


	return HAL_OK;
}


SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_get_scr(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** ACMD51 (CMD55) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = (psdhost_adp->rca) << 16;
	cmd_attr.idx = SdHostCmdAppCmd;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// check the APP_CMD
	if (!((psdhost->sd_cmd4) & BIT5)) {
		DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	/***** ACMD51 (CMD51) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)(psdhost_adp->c6r2_buf), 0, SDHOST_C6R2_BUF_LEN);
	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)(psdhost_adp->c6r2_buf)) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDma64b;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdSendScr;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_SMALL_XFER_TIMEOUT);
	if (ret != HAL_OK) {
		ret = hal_rtl_sdhost_stop_transmission(psdhost_adp);
		if (ret != HAL_OK) {
			DBG_SDHOST_ERR("Stop transmission error !!\r\n");
		}

		return HAL_ERR_UNKNOWN;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendScr) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	switch (psdhost_adp->c6r2_buf[0] & 0xF) {
	case 2:
		if (psdhost_adp->c6r2_buf[2] >> 7) {
			DBG_SDHOST_INFO("SD specification version: 3.0X\r\n");
			psdhost_adp->sd_spec_ver = SdHostSdSpecV300;
		} else {
			DBG_SDHOST_INFO("SD specification version: 2.00\r\n");
			psdhost_adp->sd_spec_ver = SdHostSdSpecV200;
		}
		break;
	case 1:
		DBG_SDHOST_INFO("SD specification version: 1.10\r\n");
		psdhost_adp->sd_spec_ver = SdHostSdSpecV110;
		break;
	case 0:
		DBG_SDHOST_INFO("SD specification version: 1.01\r\n");
		psdhost_adp->sd_spec_ver = SdHostSdSpecV101;
		break;
	default:
		DBG_SDHOST_WARN("SD specification version: Unknown\r\n");
		psdhost_adp->sd_spec_ver = 0xFF;
	}
	DBG_SDHOST_INFO("Data status after erase: %d\r\n", psdhost_adp->c6r2_buf[1] >> 7);


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_switch_function(hal_sdhost_adapter_t *psdhost_adapter, u8 mode, u8 speed, u8 *buf_32align)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;


	if ((psdhost_adapter == NULL) || (buf_32align == NULL) || (((u32)buf_32align) & 0x1F)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD6 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)buf_32align, 0, SDHOST_C6R2_BUF_LEN);
	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)buf_32align, (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)buf_32align) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDma64b;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = (mode << 31) | (0xF << 20) | (0xF << 16) | (0xF << 12) | (0xF << 8) | (0xF << 4) | speed;
	cmd_attr.idx = SdHostCmdSwitchFunc;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_SMALL_XFER_TIMEOUT);
	if (ret != HAL_OK) {
		ret = hal_rtl_sdhost_stop_transmission(psdhost_adp);
		if (ret != HAL_OK) {
			DBG_SDHOST_ERR("Stop transmission error !!\r\n");
		}

		return HAL_ERR_UNKNOWN;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)buf_32align, (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSwitchFunc) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}


	return HAL_OK;
}


SECTION_SDHOST_TEXT
static hal_status_t sdhost_send_tuning_block(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;


	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD19 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)(psdhost_adp->c6r2_buf), 0, SDHOST_C6R2_BUF_LEN);
	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)(psdhost_adp->c6r2_buf)) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDma64b;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdSendTuningBlk;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

#if 0
	// FIXME maybe use chk_transfer
	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
#else
	// CMD19 needs setting DMA, but no DMA happening, user should reset DMA
	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	sdhost_reset_dma_settings(psdhost_adp);
#endif

	if (ret == HAL_OK) {
	} else {
		return ret;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)(psdhost_adp->c6r2_buf), (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		if (ret == HAL_ERR_HW) {
			DBG_SDHOST_ERR("Current sampling point is NOT ok !!\r\n");
		}
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendTuningBlk) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	return HAL_OK;
}


SECTION_SDHOST_TEXT
static void sdhost_find_longest_continue_bits(u32 p, s32 len, s32 *__max_pos, s32 *__max_len)
{
	s32 max_pos = -1;
	s32 max_len = 0;
	s32 cursor = 0;


	for (cursor = 0; cursor < len;) {
		s32 index = cursor;
		s32 cur_len = 0;

		if (!(p & (1ULL << index))) {
			cursor++;
			continue;
		}

		index++;
		index %= len;
		while (p & (1ULL << index)) {
			index++;
			index %= len;

			if (index == cursor) {
				break;
			}
		}

		if (index <= cursor) {
			cur_len = (index + len) - cursor;
		} else {
			cur_len = index - cursor;
		}

		if (max_len < cur_len) {
			max_len = cur_len;
			max_pos = cursor;
		}

		if (index <= cursor) {
			break;
		} else {
			cursor = index;
		}
	}

	*__max_len = max_len;
	*__max_pos = max_pos;
}

// Tuning phase shift on vp1 clock
SECTION_SDHOST_TEXT
static hal_status_t sdhost_exec_tuning(hal_sdhost_adapter_t *psdhost_adapter)
{
	u32 i, wait_us;
	SDHOST_Type *psdhost = psdhost_adapter->base_addr;
	u32 raw_map = 0;
	s32 start, max_len, best_phase;

	DBG_SDHOST_WARN("Exec tuning\r\n");

	for (i = 0; i < SDHOST_CLK_PHASE_CNT; i++) {
		HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
					   i << SYSON_S_SHIFT_SYS_SD_CLK2_PSYSEL, SYSON_S_MASK_SYS_SD_CLK2_PSYSEL);

		wait_us = 0;
		while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP1_RDY) == 0) {
			hal_delay_us(1);
			wait_us++;
			if (wait_us > SD_HOST_CLK_TIMEOUT) {
				DBG_SDHOST_ERR("Wait VP1 clock ready timeout !!\r\n");
				break;
			}
		}
		psdhost->sd_config1_b.sd30_async_fifo_rst = 1;

		if (!sdhost_send_tuning_block(psdhost_adapter)) {
			raw_map |= (1 << i);
		}
	}

	sdhost_find_longest_continue_bits(raw_map, SDHOST_CLK_PHASE_CNT, &start, &max_len);
	best_phase = start + (max_len / 2);
	best_phase %= SDHOST_CLK_PHASE_CNT;
	DBG_SDHOST_INFO("%02d: start = %d, max_len = %d, best_phase = %d\r\n", raw_map, start, max_len, best_phase);

	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
				   best_phase << SYSON_S_SHIFT_SYS_SD_CLK2_PSYSEL, SYSON_S_MASK_SYS_SD_CLK2_PSYSEL);

	wait_us = 0;
	while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP1_RDY) == 0) {
		hal_delay_us(1);
		wait_us++;
		if (wait_us > SD_HOST_CLK_TIMEOUT) {
			DBG_SDHOST_ERR("Wait VP1 clock ready timeout !!\r\n");
			break;
		}
	}
	psdhost->sd_config1_b.sd30_async_fifo_rst = 1;

	return HAL_OK;
}

// Add weighting to map
SECTION_SDHOST_TEXT
static void sdhost_map_add_weighting(
	s32 map[SDHOST_CLK_PHASE_CNT][SDHOST_CLK_PHASE_CNT], u32 p1, u32 p2)
{
	// Factor is the weighting of cell
	s32 diff, factor;
	u32 half_len = SDHOST_CLK_PHASE_CNT / 2;

	for (u8 i = 0; i < SDHOST_CLK_PHASE_CNT; i++) {
		for (u8 j = 0; j < SDHOST_CLK_PHASE_CNT; j++) {
			diff = p1 - i;
			if (diff < 0) {
				diff = -diff;
			}
			if (diff > half_len) {
				diff = SDHOST_CLK_PHASE_CNT - diff;
			}
			// The closer to pointer, the bigger the factor
			factor = (1 << (4 * (half_len + 1 - diff)));
			map[i][j] += factor;

			diff = p2 - j;
			if (diff < 0) {
				diff = -diff;
			}
			if (diff > half_len) {
				diff = SDHOST_CLK_PHASE_CNT - diff;
			}
			factor = (1 << (4 * (half_len + 1 - diff)));
			map[i][j] += factor;
		}
	}
}

#if 0
SECTION_SDHOST_TEXT
static void dump_map(s32 map[SDHOST_CLK_PHASE_CNT][SDHOST_CLK_PHASE_CNT])
{
	DBG_SDHOST_INFO("map\r\n");
	for (u8 i = 0; i < SDHOST_CLK_PHASE_CNT; i++) {
		for (u8 j = 0; j < SDHOST_CLK_PHASE_CNT; j++) {
			DBG_SDHOST_INFO("%d %d:%10d \r\n", i, j, map[i][j]);
		}
	}
}
#endif

static void sdhost_find_best_phase(u32 raw_map[SDHOST_CLK_PHASE_CNT], u32 *p1, u32 *p2)
{
	s32 weight_map[SDHOST_CLK_PHASE_CNT][SDHOST_CLK_PHASE_CNT] = {0};
	s32 min = FAILED_PHASE_WEIGHT;
	*p1 = 0;
	*p2 = 0;

	// Calculate weighting map
	for (u8 i = 0; i < SDHOST_CLK_PHASE_CNT; i++) {
		for (u8 j = 0; j < SDHOST_CLK_PHASE_CNT; j++) {
			if ((raw_map[i] & (1 << j)) == 0) {
				weight_map[i][j] = FAILED_PHASE_WEIGHT;
				sdhost_map_add_weighting(weight_map, i, j);
			}
		}
	}
	//dump_map(weight_map);

	// Find phase with min wieght
	for (u8 i = 0; i < SDHOST_CLK_PHASE_CNT; i++) {
		for (u8 j = 0; j < SDHOST_CLK_PHASE_CNT; j++) {
			if (weight_map[i][j] < min) {
				min = weight_map[i][j];
				*p1 = i;
				*p2 = j;
			}
		}
	}
	if (min == FAILED_PHASE_WEIGHT) {
		DBG_SDHOST_WARN("Unable to find best phase\r\n");
	}
}

// Tuning phase shift on both-clock
SECTION_SDHOST_TEXT
static hal_status_t sdhost_exec_tuning_2d(hal_sdhost_adapter_t *psdhost_adapter)
{
	SDHOST_Type *psdhost = psdhost_adapter->base_addr;
	u32 raw_map[SDHOST_CLK_PHASE_CNT] = {0};
	u32 best_phase[2];
	u32 wait_us, ret;

	DBG_SDHOST_INFO("2D clock phase tuning\r\n");

	// VP0 = clk1, VP1 = clk2
	for (u8 clk1_shf = 0; clk1_shf < SDHOST_CLK_PHASE_CNT; clk1_shf++) {
		HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
					   clk1_shf << SYSON_S_SHIFT_SYS_SD_CLK1_PSYSEL, SYSON_S_MASK_SYS_SD_CLK1_PSYSEL);
		wait_us = 0;
		while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP0_RDY) == 0) {
			hal_delay_us(1);
			wait_us++;
			if (wait_us > SD_HOST_CLK_TIMEOUT) {
				DBG_SDHOST_ERR("Wait VP0 clock ready timeout !!\r\n");
				break;
			}
		}

		for (u8 clk2_shf = 0; clk2_shf < SDHOST_CLK_PHASE_CNT; clk2_shf++) {
			HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
						   clk2_shf << SYSON_S_SHIFT_SYS_SD_CLK2_PSYSEL, SYSON_S_MASK_SYS_SD_CLK2_PSYSEL);
			wait_us = 0;
			while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP1_RDY) == 0) {
				hal_delay_us(1);
				wait_us++;
				if (wait_us > SD_HOST_CLK_TIMEOUT) {
					DBG_SDHOST_ERR("Wait VP1 clock ready timeout !!\r\n");
					break;
				}
			}

			psdhost->sd_config1_b.sd30_async_fifo_rst = 1;
			ret = sdhost_send_tuning_block(psdhost_adapter);
			if (ret == HAL_OK) {
				raw_map[clk1_shf] |= (1 << clk2_shf);
			}
		}
	}

	for (u8 clk1_shf = 0; clk1_shf < SDHOST_CLK_PHASE_CNT; clk1_shf++) {
		DBG_SDHOST_INFO("raw_map: 0x%x\r\n", raw_map[clk1_shf]);
	}

	sdhost_find_best_phase(raw_map, &best_phase[0], &best_phase[1]);
	DBG_SDHOST_INFO("best_phase: %d %d\r\n", best_phase[0], best_phase[1]);

	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
				   best_phase[0] << SYSON_S_SHIFT_SYS_SD_CLK1_PSYSEL, SYSON_S_MASK_SYS_SD_CLK1_PSYSEL);
	wait_us = 0;
	while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP0_RDY) == 0) {
		hal_delay_us(1);
		wait_us++;
		if (wait_us > SD_HOST_CLK_TIMEOUT) {
			DBG_SDHOST_ERR("Wait VP0 clock ready timeout !!\r\n");
			break;
		}
	}

	HAL_MODIFY_REG(SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL,
				   best_phase[1] << SYSON_S_SHIFT_SYS_SD_CLK2_PSYSEL, SYSON_S_MASK_SYS_SD_CLK2_PSYSEL);
	wait_us = 0;
	while ((SYSON_S->SYSON_S_REG_SYS_SDIO_SD_CTRL & SYSON_S_BIT_SYS_SD_CK100_VP1_RDY) == 0) {
		hal_delay_us(1);
		wait_us++;
		if (wait_us > SD_HOST_CLK_TIMEOUT) {
			DBG_SDHOST_ERR("Wait VP1 clock ready timeout !!\r\n");
			break;
		}
	}
	psdhost->sd_config1_b.sd30_async_fifo_rst = 1;

	return HAL_OK;
}


SECTION_SDHOST_TEXT
void SDHOST_IRQHandler(void)
{
	SDHOST_Type *psdhost = psdhost_adapt_saved->base_addr;
	volatile u32 isr_status;
	hal_irq_clear_pending(SDH_IRQn);

	isr_status = psdhost->sd_isr;
	//DBG_SDHOST_INFO("IRQ: isr_pending: %u\r\n", isr_status);
	if (isr_status) {
		// clear pending interrupt
		psdhost->sd_isr = isr_status;
		psdhost_adapt_saved->xfer_int_sts |= isr_status;
	}

	// Non-blocking read write
	if (psdhost_adapt_saved->wait_interrupts != 0) {
		u8 is_transfer_err = 0;
		u8 is_transfer_done = 0;
		HAL_CLEAR_BIT(psdhost_adapt_saved->wait_interrupts, isr_status);

		if ((isr_status & BIT2)) {
			is_transfer_err = 1;
		}
		if (psdhost_adapt_saved->wait_interrupts == 0) {
			is_transfer_done = 1;
		}
		if (is_transfer_done || is_transfer_err) {
			if (psdhost_adapt_saved->transfer_done_cb != NULL) {
				psdhost_adapt_saved->transfer_done_cb(psdhost_adapt_saved->transfer_done_cb_para);
			}
			psdhost_adapt_saved->wait_interrupts = 0;
		}
	}

	if (psdhost->card_int_pend_b.sd_int_pend) {
		// Delay before read card_exist register (by DD)
		hal_delay_us(30);
		if (psdhost->card_exist_b.sd_exist) {
			psdhost_adapt_saved->is_card_inserted = 1;
#if 0 // wp is not used
			if (psdhost->card_exist_b.sd_wp) {
				psdhost_adapt_saved->is_wp = 1;
			} else {
				psdhost_adapt_saved->is_wp = 0;
			}
#endif
			if (psdhost_adapt_saved->card_insert_cb != NULL) {
				(psdhost_adapt_saved->card_insert_cb)(psdhost_adapt_saved->card_insert_cb_para);
			}
		} else {
			psdhost_adapt_saved->is_card_inserted = 0;
			psdhost_adapt_saved->card_inited = 0;
#if 0
			psdhost_adapt_saved->is_wp = 0;
#endif
			if (psdhost_adapt_saved->card_remove_cb != NULL) {
				(psdhost_adapt_saved->card_remove_cb)(psdhost_adapt_saved->card_remove_cb_para);
			}
		}
		// clear pending interrupt
		psdhost->card_int_pend_b.sd_int_pend = 1;
	}
	__DSB();
}


SECTION_SDHOST_TEXT
void hal_rtl_sdhost_irq_reg(irq_handler_t irq_handler)
{
	// IRQ vector may has been registered, disable and re-register it
	hal_irq_disable(SDH_IRQn);
	__ISB();
	hal_irq_set_vector(SDH_IRQn, (uint32_t)irq_handler);
	hal_irq_enable(SDH_IRQn);
}


SECTION_SDHOST_TEXT
void hal_rtl_sdhost_irq_unreg(void)
{
	hal_irq_disable(SDH_IRQn);
	__ISB();
	hal_irq_set_vector(SDH_IRQn, (uint32_t)NULL);
}

/**
 *  @brief To initialize the SDIO host controller.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  pin_sel The pinmux selection.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_init_host(hal_sdhost_adapter_t *psdhost_adapter)
{
	SDHOST_Type *psdhost;
	hal_status_t ret = HAL_OK;

#if defined(SDHOST_USE_INIT_VAR) && (SDHOST_USE_INIT_VAR == 1)
	if (host_inited) {
		DBG_SDHOST_INFO("Host had been inited\r\n");
		return HAL_OK;
	}
#endif

	DBG_SDHOST_INFO("Init host\r\n");
	psdhost_adapt_saved = psdhost_adapter;
	memset((void *)psdhost_adapter, 0, sizeof(hal_sdhost_adapter_t));

#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1)
	psdhost_adapter->force_33v = 1;
#endif

#if defined(CONFIG_BUILD_SECURE)
	psdhost_adapter->base_addr = (SDHOST_Type *) SDHOST_S_BASE;
#elif defined(CONFIG_BUILD_NONSECURE)
	psdhost_adapter->base_addr = (SDHOST_Type *) SDHOST_BASE;
	//ignore-secure
#elif !defined(CONFIG_BUILD_NONSECURE)
	psdhost_adapter->base_addr = (SDHOST_Type *) SDHOST_S_BASE;
#endif
	psdhost = psdhost_adapter->base_addr;

	for (int i = 0; i < PINTAB_SIZE; i++) {
		ret = hal_pinmux_register(sdhost_pin_table[i], PID_SD_HOST);
		if (HAL_OK != ret) {
			return ret;
		}
	}

	hal_rtl_sdhost_irq_reg((irq_handler_t)SDHOST_IRQHandler);
	hal_irq_set_priority(SDH_IRQn, SDH_IRQPri);

	// enable LX bus & LX bus clock
	SYSON_S->SYSON_S_REG_SYS_PLATFORM_CTRL0 |= SYSON_S_BIT_LXBUS_CLK_EN | SYSON_S_BIT_LXBUS_EN;

	gpio_pull_ctrl(psdhost_adapter, GPIO_CTRL_HIGH);

	hal_rtl_sdhost_ldo_ctrl(SDHOST_LDO_3V3);

	hal_rtl_sdhost_en_ctrl(ON);

	// enable interrupt
	psdhost->card_int_en_b.sd_int_en = 1;
	psdhost->sd_isren = 0x17;
	hal_delay_us(1);
	host_inited = 1;

	return ret;
}


/**
 *  @brief To initialize the SD memory card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_init_card(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_status_t ret;
	u8 supported_speed;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}

#if defined(SDHOST_USE_INIT_VAR) && (SDHOST_USE_INIT_VAR == 1)
	if (psdhost_adapter->card_inited) {
		DBG_SDHOST_INFO("Card had been inited\r\n");
		return HAL_OK;
	}
#endif

	psdhost_adapter->dcache_invalidate_by_addr = hal_cache_stubs.dcache_invalidate_by_addr;
	psdhost_adapter->dcache_clean_by_addr = hal_cache_stubs.dcache_clean_by_addr;

	DBG_SDHOST_INFO("Init card\r\n");

	// card detection & write protection
	psdhost_adapter->is_card_inserted = psdhost_adapter->base_addr->card_exist_b.sd_exist;
#if defined(CONFIG_FPGA) && (CONFIG_FPGA == 1)
	// FPGA card detection interrupt is not working
	psdhost_adapter->is_card_inserted = 1;
#endif

	if (psdhost_adapter->is_card_inserted == 0) {
		DBG_SDHOST_WARN("No card inserted\r\n");
		return HAL_ERR_HW;
	}

	if (0/*!(psdhost_adapter->is_wp)*/) {
		DBG_SDHOST_WARN("Card is write protected !!\r\n");
		return HAL_ERR_HW;
	}

	sdhost_host_presetting(psdhost_adapter);

	ret = sdhost_reset_card(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_voltage_check(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_get_ocr(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_get_cid(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_get_rca(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	// switch to non-initial mode
	ret = sdhost_initial_mode_ctrl(psdhost_adapter, OFF);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_get_csd(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_card_select(psdhost_adapter, SdHostSelCard);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = sdhost_set_bus_width(psdhost_adapter, SdHostBus4bit);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = hal_rtl_sdhost_get_supported_speed(psdhost_adapter, &supported_speed);
	if (ret == HAL_OK) {
		DBG_SDHOST_INFO("Supported speed mode: 0x%x\r\n", supported_speed);
	}

	DBG_SDHOST_INFO("SD card is initialized\r\n");
	psdhost_adapter->card_inited = 1;
	return HAL_OK;
}


/**
 *  @brief To de-initialize the SDIO host controller.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *
 *  @returns    void.
 */
SECTION_SDHOST_TEXT
void hal_rtl_sdhost_deinit(hal_sdhost_adapter_t *psdhost_adapter)
{
	psdhost_adapter->dcache_invalidate_by_addr = NULL;
	psdhost_adapter->dcache_clean_by_addr = NULL;

	/* disable interrupt & clear all pending interrupts */
	psdhost_adapter->base_addr->card_int_pend_b.sd_int_pend = 1;
	psdhost_adapter->base_addr->card_int_en_b.sd_int_en = 0;
	psdhost_adapter->base_addr->sd_isr = 0x16;
	psdhost_adapter->base_addr->sd_isren = 0x16;
	hal_rtl_sdhost_irq_unreg();
	hal_rtl_sdhost_en_ctrl(OFF);

	for (int i = 0; i < PINTAB_SIZE; i++) {
		hal_pinmux_unregister(sdhost_pin_table[i], PID_SD_HOST);
	}
	host_inited = 0;
	psdhost_adapter->card_inited = 0;
}


/**
 *  @brief To read data from the SD card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  start_addr The start address to begin reading from the card.
 *  @param[in]  blk_cnt The block count.
 *  @param[in]  rbuf_32align The buffer to read data blocks (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_read_data(hal_sdhost_adapter_t *psdhost_adapter,
									  u64 start_addr, u16 blk_cnt, u8 *rbuf_32align)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;
	u32 start;

	if ((psdhost_adapter == NULL) || (!blk_cnt) || (rbuf_32align == NULL) || (((u32)rbuf_32align) & 0x1F)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	if (psdhost_adp->is_sdhc_sdxc) {
		// unit: block
		start = (u32)(start_addr / SDHOST_ONE_BLK_LEN);
	} else {
		start = (u32)start_addr;
	}

	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)rbuf_32align, (int32_t)(blk_cnt * SDHOST_ONE_BLK_LEN));
	}

	if (blk_cnt > 1) {
		/***** CMD18 *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}
		// Set waiting interrupts
		psdhost_adapter->wait_interrupts = SDHOST_INT_CARD_END | SDHOST_INT_DMA_END;

		dma_cfg.op = SdHostDmaRead;
		dma_cfg.start_addr = ((u32)rbuf_32align) / 8;
		dma_cfg.blk_cnt = blk_cnt;
		dma_cfg.type = SdHostDmaNormal;
		sdhost_config_dma(psdhost_adp, &dma_cfg);

		cmd_attr.arg = start;
		cmd_attr.idx = SdHostCmdRdMulBlk;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_xfer_int(psdhost_adp, SDHOST_XFER_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// D-Cache sync (Invalidate)
		if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
			psdhost_adp->dcache_invalidate_by_addr((uint32_t *)rbuf_32align, (int32_t)(blk_cnt * SDHOST_ONE_BLK_LEN));
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdStopXsmission) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		// check the address error/block len error
		if (((psdhost->sd_cmd1) & BIT6) || ((psdhost->sd_cmd1) & BIT5)) {
			return HAL_ERR_UNKNOWN;
		}
	} else {
		/***** CMD17 *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}
		// Set waiting interrupts
		psdhost_adapter->wait_interrupts = SDHOST_INT_CARD_END | SDHOST_INT_DMA_END;

		dma_cfg.op = SdHostDmaRead;
		dma_cfg.start_addr = ((u32)rbuf_32align) / 8;
		dma_cfg.blk_cnt = 1;
		dma_cfg.type = SdHostDmaNormal;
		sdhost_config_dma(psdhost_adp, &dma_cfg);

		cmd_attr.arg = start;
		cmd_attr.idx = SdHostCmdRdSingleBlk;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_xfer_int(psdhost_adp, SDHOST_XFER_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			ret = hal_rtl_sdhost_stop_transmission(psdhost_adp);
			if (ret != HAL_OK) {
				DBG_SDHOST_ERR("Stop transmission error !!\r\n");
			}

			return HAL_ERR_UNKNOWN;
		}

		// D-Cache sync (Invalidate)
		if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
			psdhost_adp->dcache_invalidate_by_addr((uint32_t *)rbuf_32align, (int32_t)(blk_cnt * SDHOST_ONE_BLK_LEN));
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdRdSingleBlk) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		// check the address error/block len error
		if (((psdhost->sd_cmd1) & BIT6) || ((psdhost->sd_cmd1) & BIT5)) {
			return HAL_ERR_UNKNOWN;
		}
	}

	return HAL_OK;
}


/**
 *  @brief To write data to the SD card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  start_addr The start address to begin writing to the card.
 *  @param[in]  blk_cnt The block count.
 *  @param[in]  wbuf_32align The buffer to write data blocks (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_write_data(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, const u8 *wbuf_32align)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;
	u32 start;

	if ((psdhost_adapter == NULL) || (!blk_cnt) || (wbuf_32align == NULL) || (((u32)wbuf_32align) & 0x1F)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	if (psdhost_adp->is_sdhc_sdxc) {
		// unit: block
		start = (u32)(start_addr / SDHOST_ONE_BLK_LEN);
	} else {
		start = (u32)start_addr;
	}

	if (blk_cnt > 1) {
		/***** ACMD23 (CMD55) *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		cmd_attr.arg = (psdhost_adp->rca) << 16;
		cmd_attr.idx = SdHostCmdAppCmd;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}
		// check the APP_CMD
		if (!((psdhost->sd_cmd4) & BIT5)) {
			DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		/***** ACMD23 (CMD23) *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		cmd_attr.arg = blk_cnt;
		cmd_attr.idx = SdHostCmdSetWrBlkEraseCnt;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostNoDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSetWrBlkEraseCnt) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		/***** CMD25 *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		if (psdhost_adp->dcache_clean_by_addr != NULL) {
			psdhost_adp->dcache_clean_by_addr((uint32_t *)wbuf_32align, (int32_t)(blk_cnt * SDHOST_ONE_BLK_LEN));
		}

		// Set waiting interrupts
		psdhost_adapter->wait_interrupts = SDHOST_INT_CARD_END | SDHOST_INT_DMA_END;

		dma_cfg.op = SdHostDmaWrite;
		dma_cfg.start_addr = ((u32)wbuf_32align) / 8;
		dma_cfg.blk_cnt = blk_cnt;
		dma_cfg.type = SdHostDmaNormal;
		sdhost_config_dma(psdhost_adp, &dma_cfg);

		cmd_attr.arg = start;
		cmd_attr.idx = SdHostCmdWrMulBlk;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_xfer_int(psdhost_adp, SDHOST_XFER_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			return ret;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdStopXsmission) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		// check the address error/block len error/wp violation
		if (((psdhost->sd_cmd1) & BIT6) || ((psdhost->sd_cmd1) & BIT5) || ((psdhost->sd_cmd1) & BIT2)) {
			return HAL_ERR_UNKNOWN;
		}
	} else {
		/***** CMD24 *****/
		ret = sdhost_chk_cmd_data_state(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		if (psdhost_adp->dcache_clean_by_addr != NULL) {
			psdhost_adp->dcache_clean_by_addr((uint32_t *)wbuf_32align, (int32_t)SDHOST_ONE_BLK_LEN);
		}

		// Set waiting interrupts
		psdhost_adapter->wait_interrupts = SDHOST_INT_CARD_END | SDHOST_INT_DMA_END;

		dma_cfg.op = SdHostDmaWrite;
		dma_cfg.start_addr = ((u32)wbuf_32align) / 8;
		dma_cfg.blk_cnt = 1;
		dma_cfg.type = SdHostDmaNormal;
		sdhost_config_dma(psdhost_adp, &dma_cfg);

		cmd_attr.arg = start;
		cmd_attr.idx = SdHostCmdWrBlk;
		cmd_attr.rsp_type = SdHostRsp6Bytes;
		cmd_attr.rsp_crc_chk = ENABLE;
		cmd_attr.data_present = SdHostDataPresent;
		sdhost_send_cmd(psdhost_adp, &cmd_attr);

		ret = hal_rtl_sdhost_chk_xfer_int(psdhost_adp, SDHOST_XFER_CPLT_TIMEOUT);
		if (ret != HAL_OK) {
			ret = hal_rtl_sdhost_stop_transmission(psdhost_adp);
			if (ret != HAL_OK) {
				DBG_SDHOST_ERR("Stop transmission error !!\r\n");
			}

			return HAL_ERR_UNKNOWN;
		}

		// check if any errors
		ret = sdhost_chk_xfer_error(psdhost);
		if (ret != HAL_OK) {
			return ret;
		}

		// check the command index
		if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdWrBlk) {
			DBG_SDHOST_ERR("Command index error !!\r\n");
			return HAL_ERR_UNKNOWN;
		}

		// check the address error/block len error/wp violation
		if (((psdhost->sd_cmd1) & BIT6) || ((psdhost->sd_cmd1) & BIT5) || ((psdhost->sd_cmd1) & BIT2)) {
			return HAL_ERR_UNKNOWN;
		}
	}


	return HAL_OK;
}


/**
 *  @brief To erase data in the SD card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  start_addr The start address to begin erasing.
 *  @param[in]  end_addr The end address to begin erasing.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_erase(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u64 end_addr)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;
	u32 start, end;


	if ((psdhost_adapter == NULL) || (start_addr > end_addr)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	if (psdhost_adp->is_sdhc_sdxc) {
		// unit: block
		start = (u32)(start_addr / SDHOST_ONE_BLK_LEN);
		end = (u32)(end_addr / SDHOST_ONE_BLK_LEN);
	} else {
		start = (u32)start_addr;
		end = (u32)end_addr;
	}

	/***** CMD32 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = start;
	cmd_attr.idx = SdHostCmdEraseBlkSt;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdEraseBlkSt) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	/***** CMD33 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = end;
	cmd_attr.idx = SdHostCmdEraseBlkEd;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdEraseBlkEd) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	/***** CMD38 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdErase;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, 30000);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdErase) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}


	return HAL_OK;
}


/**
 *  @brief To stop the SD bus transmission.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_stop_transmission(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD12 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdStopXsmission;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdStopXsmission) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// stop the transfer & the transfer state machine returns to idle state
	psdhost->card_stop_b.sd_module = 1;


	return HAL_OK;
}


/**
 *  @brief To get the current state of the SD card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_get_card_status(hal_sdhost_adapter_t *psdhost_adapter)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_cmd_attr_t cmd_attr;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** CMD13 *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = (psdhost_adp->rca) << 16;
	cmd_attr.idx = SdHostCmdSendSts;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendSts) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// get card's current state
	psdhost_adp->card_curr_ste = ((psdhost->sd_cmd3) >> 1) & 0xF;
	//DBG_SDHOST_INFO("card_curr_ste = %d\r\n", psdhost_adp->card_curr_ste);
	return HAL_OK;
}


/**
 *  @brief To get the SD status from the SD card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  buf_32align The buffer to store the SD status (must be 32-Byte alignment).
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_get_sd_status(hal_sdhost_adapter_t *psdhost_adapter, u8 *buf_32align)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	hal_sdhost_dma_ctrl_t dma_cfg;
	hal_sdhost_cmd_attr_t cmd_attr;

	if ((psdhost_adapter == NULL) || (buf_32align == NULL) || (((u32)buf_32align) & 0x1F)) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	/***** ACMD13 (CMD55) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	cmd_attr.arg = (psdhost_adp->rca) << 16;
	cmd_attr.idx = SdHostCmdAppCmd;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostNoDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_cmd_complete(psdhost_adp, SDHOST_CMD_CPLT_TIMEOUT);
	if (ret != HAL_OK) {
		return ret;
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdAppCmd) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	// check the APP_CMD
	if (!((psdhost->sd_cmd4) & BIT5)) {
		DBG_SDHOST_ERR("ACMD isn't expected !!\r\n");
		return HAL_ERR_UNKNOWN;
	}

	/***** ACMD13 (CMD13) *****/
	ret = sdhost_chk_cmd_data_state(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	memset((void *)buf_32align, 0, SDHOST_C6R2_BUF_LEN);

	// D-Cache write-back
	if (psdhost_adp->dcache_clean_by_addr != NULL) {
		psdhost_adp->dcache_clean_by_addr((uint32_t *)buf_32align, (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	dma_cfg.op = SdHostDmaRead;
	dma_cfg.start_addr = ((u32)buf_32align) / 8;
	dma_cfg.blk_cnt = 1;
	dma_cfg.type = SdHostDma64b;
	sdhost_config_dma(psdhost_adp, &dma_cfg);

	cmd_attr.arg = 0;
	cmd_attr.idx = SdHostCmdSendSts;
	cmd_attr.rsp_type = SdHostRsp6Bytes;
	cmd_attr.rsp_crc_chk = ENABLE;
	cmd_attr.data_present = SdHostDataPresent;
	sdhost_send_cmd(psdhost_adp, &cmd_attr);

	ret = hal_rtl_sdhost_chk_xfer_complete(psdhost_adp, SDHOST_SMALL_XFER_TIMEOUT);
	if (ret != HAL_OK) {
		ret = hal_rtl_sdhost_stop_transmission(psdhost_adp);
		if (ret != HAL_OK) {
			DBG_SDHOST_ERR("Stop transmission error !!\r\n");
		}

		return HAL_ERR_UNKNOWN;
	}
	// D-Cache sync (Invalidate)
	if (psdhost_adp->dcache_invalidate_by_addr != NULL) {
		psdhost_adp->dcache_invalidate_by_addr((uint32_t *)buf_32align, (int32_t)SDHOST_C6R2_BUF_LEN);
	}

	// check if any errors
	ret = sdhost_chk_xfer_error(psdhost);
	if (ret != HAL_OK) {
		return ret;
	}

	// check the command index
	if (((psdhost->sd_cmd0) & SDHOST_CMD_IDX_MASK) != SdHostCmdSendSts) {
		DBG_SDHOST_ERR("Command index error !!\r\n");
		return HAL_ERR_UNKNOWN;
	}
	// Byte reverse sd_status
	for (int i = 0 ; i < (SD_STATUS_LEN / 2); i++) {
		u8 *low = (u8 *)(buf_32align) + i;
		u8 *high = (u8 *)(buf_32align) + (SD_STATUS_LEN - i - 1);
		u8 tmp = *low;
		*low = *high;
		*high = tmp;
	}

	return HAL_OK;
}


/**
 *  @brief To switch the SD bus speed.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  speed The specified bus speed.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_switch_bus_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 speed)
{
	hal_sdhost_adapter_t *psdhost_adp;
	SDHOST_Type *psdhost;
	hal_status_t ret;
	u8 sw_spd, supported_speed;

	if (psdhost_adapter == NULL) {
		return HAL_ERR_PARA;
	}
	psdhost_adp = psdhost_adapter;
	psdhost = psdhost_adp->base_addr;

	ret = hal_rtl_sdhost_get_supported_speed(psdhost_adp, &supported_speed);
	if (ret != HAL_OK) {
		return ret;
	}

	// convert speed to access mode code
	if ((psdhost_adp->curr_sig_level) == SdHostSigVol18) {
		sw_spd = speed - 2;
	} else {
		sw_spd = speed;
	}

	if ((psdhost_adp->curr_sig_level) == SdHostSigVol18) {
		if (speed < SdHostSpeedSDR12 || speed == SdHostSpeedSDR104 || speed > SdHostSpeedDDR50) {
			DBG_SDHOST_WARN("Current signal level is 1.8V, speed '%s' is unsupported !!\r\n",
							sdhost_speed2str(speed));
			return HAL_ERR_PARA;
		}
	} else {
		if (speed > SdHostSpeedHS) {
			DBG_SDHOST_WARN("Current signal level is 3.3V, speed '%s' is unsupported !!\r\n",
							sdhost_speed2str(speed));
			return HAL_ERR_PARA;
		}
	}

	if ((psdhost_adp->curr_bus_spd) == speed) {
		DBG_SDHOST_INFO("Current SD bus speed is already the speed '%s'\r\n",
						sdhost_speed2str(speed));
		return HAL_OK;
	}

	if ((supported_speed & (1 << sw_spd)) == 0) {
		DBG_SDHOST_ERR("This card doesn't support the '%s' speed mode !!\r\n",
					   sdhost_speed2str(speed));
		return HAL_ERR_HW;
	}

	if ((psdhost_adp->c6r2_buf[16] & 0xF) == sw_spd) {
		DBG_SDHOST_INFO("Current SD bus speed is already the speed '%s'\r\n",
						sdhost_speed2str(speed));
		return HAL_OK;
	}

	// check if the specified speed can be switched
	ret = sdhost_switch_function(psdhost_adp, SdHostCmd6CheckMode, sw_spd, psdhost_adp->c6r2_buf);
	if (ret != HAL_OK) {
		return ret;
	}

	if ((psdhost_adp->c6r2_buf[16] & 0xF) != sw_spd) {
		DBG_SDHOST_ERR("The '%s' speed mode can't be switched !!\r\n",
					   sdhost_speed2str(speed));
		return HAL_ERR_UNKNOWN;
	}

	// Switch to the specified speed
	ret = sdhost_switch_function(psdhost_adp, SdHostCmd6SwitchMode, sw_spd, psdhost_adp->c6r2_buf);
	if (ret != HAL_OK) {
		return ret;
	}

	if ((psdhost_adp->c6r2_buf[16] & 0xF) != sw_spd) {
		DBG_SDHOST_ERR("Switch speed '%s' request is canceled !!\r\n",
					   sdhost_speed2str(speed));
		return HAL_ERR_UNKNOWN;
	}

	if ((psdhost_adp->curr_sig_level) == SdHostSigVol18) {
		if (speed == SdHostSpeedSDR12) {
			psdhost->ckgen_ctl = 0x0002;  // 25 MHz (SD30 mode)
			psdhost->sd_config1_b.mode_sel = SdHostModeSd30;
			psdhost_adp->curr_bus_spd = SdHostSpeedSDR12;
		} else if (speed == SdHostSpeedSDR25) {
			psdhost->ckgen_ctl = 0x0001;  // 50 MHz (SD30 mode)
			psdhost->sd_config1_b.mode_sel = SdHostModeSd30;
			psdhost_adp->curr_bus_spd = SdHostSpeedSDR25;
		} else if (speed == SdHostSpeedSDR50) {
			// TODO check phase shift failed
			DBG_SDHOST_INFO("is_wp: %x\r\n", psdhost_adp->is_wp);
			// div clock = 0, before using phase shift
			if (PHASE_TUNING_2D || psdhost_adp->is_wp > 10) {
				// select phase shift, crc_clk = vp0, sample_clk = vp1
				psdhost->ckgen_ctl = 0x2010;  // 100 MHz (SD30 mode)
				sdhost_exec_tuning_2d(psdhost_adp);
			} else {
				// select phase shift, push_clk = vp0, sample_clk = vp1
				psdhost->ckgen_ctl = 0x2100;  // 100 MHz (SD30 mode)
				switch (psdhost_adp->is_wp) {
				case 0:
					psdhost->ckgen_ctl = 0x2000;  // 100 MHz (SD30 mode)
					break;
				case 1:
					psdhost->ckgen_ctl = 0x0200;  // 100 MHz (SD30 mode)
					break;
				case 2:
					psdhost->ckgen_ctl = 0x0020;
					break;
				}
				DBG_SDHOST_INFO("ckgen_ctl: %x\r\n", psdhost->ckgen_ctl);
				sdhost_exec_tuning(psdhost_adp);
			}
			psdhost->sd_config1_b.mode_sel = SdHostModeSd30;
			psdhost_adp->curr_bus_spd = SdHostSpeedSDR50;
		} else if (speed == SdHostSpeedDDR50) {
			// FIXME use unify expression
			// FIXME Reset value for other mode?
			HAL_WRITE8(SDHOST_BASE, 0x587, 0x50);
			psdhost->ckgen_ctl = 0x0000;  // 50 MHz (DDR mode)
			psdhost->sd_config1_b.mode_sel = SdHostModeDdr;
			psdhost_adp->curr_bus_spd = SdHostSpeedDDR50;
		} else {
			DBG_SDHOST_ERR("Unhandled speed mode %d at 1.8V\r\n", speed);
			return HAL_ERR_UNKNOWN;
		}
	} else {
		if (speed == SdHostSpeedDS) {
			psdhost->ckgen_ctl = 0x0001;  // 25 MHz (SD20 mode)
			psdhost->sd_config1_b.mode_sel = SdHostModeSd20;
			psdhost_adp->curr_bus_spd = SdHostSpeedDS;
		} else if (speed == SdHostSpeedHS) {
			psdhost->ckgen_ctl = 0x0000;  // 50 MHz (SD20 mode)
			psdhost->sd_config1_b.mode_sel = SdHostModeSd20;
			psdhost_adp->curr_bus_spd = SdHostSpeedHS;
		} else {
			DBG_SDHOST_ERR("Unhandled speed mode %d at 3.3V\r\n", speed);
			return HAL_ERR_UNKNOWN;
		}
	}
	DBG_SDHOST_INFO("SD card changes to speed mode '%s' successfully\r\n",
					sdhost_speed2str(speed));

	return HAL_OK;
}


/**
 *  @brief To get the current signaling level.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *
 *  @returns    The signaling level (1: 1.8V, 0: 3.3V).
 */
SECTION_SDHOST_TEXT
u8 hal_rtl_sdhost_get_curr_signal_level(hal_sdhost_adapter_t *psdhost_adapter)
{
	return (psdhost_adapter->curr_sig_level);
}


/**
 *  @brief To get the speed mode supported by the card.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  value The supported speed mode.
 *
 *  @returns    The result.
 */
SECTION_SDHOST_TEXT
hal_status_t hal_rtl_sdhost_get_supported_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 *value)
{
	hal_status_t ret;


	if ((psdhost_adapter == NULL) || (value == NULL)) {
		return HAL_ERR_PARA;
	}

	// check if this card supports CMD6
	ret = hal_rtl_sdhost_get_scr(psdhost_adapter);
	if (ret != HAL_OK) {
		return ret;
	}

	if ((psdhost_adapter->sd_spec_ver) >= SdHostSdSpecV110) {
		// get the supported speed modes
		ret = sdhost_switch_function(psdhost_adapter, SdHostCmd6CheckMode, SdHostKeepCurSpd, psdhost_adapter->c6r2_buf);
		if (ret != HAL_OK) {
			return ret;
		}

		*value = psdhost_adapter->c6r2_buf[13];
		// save the info. of speed mode that supported by this card
		psdhost_adapter->card_support_spd_mode = psdhost_adapter->c6r2_buf[13];
	} else {
		DBG_SDHOST_WARN("This card doesn't support CMD6 and can't get the supported speed !!\r\n");
		*value = 0;
		return HAL_ERR_HW;
	}
	return HAL_OK;
}


/**
 *  @brief To hook a callback function for SD card insertion interrupt.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  pcallback The callback function.
 *  @param[in]  pdata The argument of the callback function.
 *
 *  @returns    void.
 */
SECTION_SDHOST_TEXT
void hal_rtl_sdhost_card_insert_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata)
{
	psdhost_adapter->card_insert_cb = pcallback;
	psdhost_adapter->card_insert_cb_para = pdata;

	// for 1st card insertion
	if (psdhost_adapter->is_card_inserted) {
		psdhost_adapter->card_insert_cb(psdhost_adapter->card_insert_cb_para);
	}
}


/**
 *  @brief To hook a callback function for SD card removal interrupt.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  pcallback The callback function.
 *  @param[in]  pdata The argument of the callback function.
 *
 *  @returns    void.
 */
SECTION_SDHOST_TEXT
void hal_rtl_sdhost_card_remove_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata)
{
	psdhost_adapter->card_remove_cb = pcallback;
	psdhost_adapter->card_remove_cb_para = pdata;
}


/**
 *  @brief To hook a callback function to make OS do a context-switch while waiting.
 *
 *  @param[in]  psdhost_adapter The SDIO host adapter.
 *  @param[in]  task_yield The callback function.
 *
 *  @returns    void.
 */
SECTION_SDHOST_TEXT
void hal_rtl_sdhost_task_yield_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t task_yield, void *pdata)
{
	psdhost_adapter->task_yield_cb = task_yield;
	psdhost_adapter->task_yield_cb_para = pdata;
}

/**
 *  @brief To hook a callback function for completion of non-blocking read/write.
 *         NOTE: At least one of the two transfer hook will be triggered when transfer is done.
 *
 *  @param[in]  psdhost_adapter The SD host adapter.
 *  @param[in]  transfer_done_cb The callback function.
 *  @param[in]  pdata The argument of the callback function.
 *
 *  @returns    void.
 */
SECTION_SDHOST_TEXT
void hal_rtl_sdhost_transfer_done_int_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t transfer_done_cb, void *pdata)
{
	psdhost_adapter->transfer_done_cb = transfer_done_cb;
	psdhost_adapter->transfer_done_cb_para = pdata;
}

/** @} */ /* End of group hs_hal_sdhost__rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_sdhost_ */


#endif  // end of "#if CONFIG_SDHOST_EN


