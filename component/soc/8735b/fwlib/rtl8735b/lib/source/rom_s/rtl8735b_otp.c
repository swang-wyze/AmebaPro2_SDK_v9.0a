/**************************************************************************//**
 * @file     rtl8735b_otp.c
 * @brief    This file implements the UART IP power, clock, baud rate
 *           configuration functions.
 * @version  V1.00
 * @date     2021-07-16
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2020 Realtek Corporation. All rights reserved.
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
#include "rtl8735b.h"
#include "rtl8735b_otp.h"

#if CONFIG_OTP_EN

#define SECTION_OTP_TEXT           SECTION(".rom.hal_otp.text")
#define SECTION_OTP_DATA           SECTION(".rom.hal_otp.data")
#define SECTION_OTP_RODATA         SECTION(".rom.hal_otp.rodata")
#define SECTION_OTP_BSS            SECTION(".rom.hal_otp.bss")
#define SECTION_OTP_STUBS          SECTION(".rom.hal_otp.stubs")
/**
* @addtogroup hal_otp OTP
* @{
*/

/**
* @brief The global common data structure to store common resource
*        for all OTP adapters.
*/
SECTION_OTP_BSS hal_otp_adapter_t otp_adpt;

/**
* @brief The global common data structure to store common resource
*        for all OTP adapters.
*/
SECTION_OTP_BSS hal_otp_adapter_t *_potp_adpt;
//SECTION_OTP_BSS uint32_t    otp_wr_dly;
SECTION_OTP_BSS uint8_t     otp_adj_vol_lvl = 0xFF;
/**/
SECTION_OTP_BSS uint8_t     otp_op_to       = 100;

/// @cond DOXYGEN_ROM_HAL_API

/**
* @addtogroup hs_hal_uart_rom_func UART HAL ROM APIs.
* @ingroup hs_hal_uart
* @{
* @brief UART HAL ROM API. The user application(in RAM space) should not call these APIs directly.
*        There is another set of UART HAL APIs in the RAM space is provided for the user application.
*/

/**
* @brief The stubs functions table to exports OTP HAL functions in ROM.
*/
SECTION_OTP_STUBS const hal_otp_func_stubs_t hal_otp_stubs = {
	.ppotp_adapter                     = &_potp_adpt,
	.hal_otp_init                  = hal_rtl_otp_init,
	.hal_otp_deinit                = hal_rtl_otp_deinit,
	.hal_otp_ctrl_sel              = hal_rtl_otp_ctrl_sel,
	.hal_otp_ctrl_sel_sts          = hal_rtl_otp_ctrl_sel_sts,
	.hal_otp_pmc_gnt_ctrl          = hal_rtl_otp_pmc_gnt_ctrl,
	.hal_otp_wr_gnt_ctrl           = hal_rtl_otp_wr_gnt_ctrl,
	.hal_otpc_ctrl                 = hal_rtl_otpc_ctrl,
	.hal_otp_pwr_ctrl              = hal_rtl_otp_pwr_ctrl,
	.hal_otp_test_mod_cfg          = hal_rtl_otp_test_mod_cfg,
	.hal_otp_test_row_ctrl         = hal_rtl_otp_test_row_ctrl,
	.hal_otp_chk_err_sts           = hal_rtl_otp_chk_err_sts,
	.hal_otp_dsb_ctrl              = hal_rtl_otp_dsb_ctrl,
	.hal_otp_rb_ctrl               = hal_rtl_otp_rb_ctrl,
	.hal_otp_cz_ctrl               = hal_rtl_otp_cz_ctrl,
	.hal_otp_chk_cz_sts            = hal_rtl_otp_chk_cz_sts,
	.hal_otp_clr_cz_sts            = hal_rtl_otp_clr_cz_sts,
	.hal_otp_prct_ctrl             = hal_rtl_otp_prct_ctrl,
	.hal_otp_chk_prct_sts          = hal_rtl_otp_chk_prct_sts,
	.hal_otp_rd_lz_er_addr         = hal_rtl_otp_rd_lz_er_addr,
	.hal_otp_clr_prct_sts          = hal_rtl_otp_clr_prct_sts,
	.hal_otp_rd_sz_prct_sts        = hal_rtl_otp_rd_sz_prct_sts,
	.hal_otp_clr_sz_prct_sts       = hal_rtl_otp_clr_sz_prct_sts,
	.hal_otp_rd_ssz_prct_sts       = hal_rtl_otp_rd_ssz_prct_sts,
	.hal_otp_clr_ssz_prct_sts      = hal_rtl_otp_clr_ssz_prct_sts,
	.hal_otp_szwl_ctrl             = hal_rtl_otp_szwl_ctrl,
	.hal_otp_rd_aldn_sts           = hal_rtl_otp_rd_aldn_sts,
	.hal_otp_clr_aldn_sts          = hal_rtl_otp_clr_aldn_sts,
	.hal_otp_rd_al_sts             = hal_rtl_otp_rd_al_sts,
	.hal_otp_clr_al_sts            = hal_rtl_otp_clr_al_sts,
	.hal_otp_al_cfg                = hal_rtl_otp_al_cfg,
	.hal_otp_al_ctrl_aon           = hal_rtl_otp_al_ctrl_aon,
	.hal_otp_bust_ctrl             = hal_rtl_otp_bust_ctrl,
	.hal_otp_rd_sub_aon            = hal_rtl_otp_rd_sub_aon,
	.hal_otp_rd_aon                = hal_rtl_otp_rd_aon,
	.hal_otp_byte_rd_aon           = hal_rtl_otp_byte_rd_aon,
	.hal_otp_wr_sub_aon            = hal_rtl_otp_wr_sub_aon,
	.hal_otp_wr_aon                = hal_rtl_otp_wr_aon,
	.hal_otp_byte_wr_aon           = hal_rtl_otp_byte_wr_aon,
	.hal_otp_cmp_aon               = hal_rtl_otp_cmp_aon,
	.hal_otp_rd_sub_sys            = hal_rtl_otp_rd_sub_sys,
	.hal_otp_rd_sys                = hal_rtl_otp_rd_sys,
	.hal_otp_byte_rd_sys           = hal_rtl_otp_byte_rd_sys,
	.hal_otp_wr_sub_sys            = hal_rtl_otp_wr_sub_sys,
	.hal_otp_wr_sys                = hal_rtl_otp_wr_sys,
	.hal_otp_byte_wr_sys           = hal_rtl_otp_byte_wr_sys,
	.hal_otp_cmp_sys               = hal_rtl_otp_cmp_sys,
	.hal_otp_al_ctrl_sys           = hal_rtl_otp_al_ctrl_sys,
	.hal_otp_rd_sub_syss           = hal_rtl_otp_rd_sub_syss,
	.hal_otp_rd_syss               = hal_rtl_otp_rd_syss,
	.hal_otp_byte_rd_syss          = hal_rtl_otp_byte_rd_syss,
	.hal_otp_wr_sub_syss           = hal_rtl_otp_wr_sub_syss,
	.hal_otp_wr_syss               = hal_rtl_otp_wr_syss,
	.hal_otp_byte_wr_syss          = hal_rtl_otp_byte_wr_syss,
	.hal_otp_cmp_syss              = hal_rtl_otp_cmp_syss,
	.hal_otp_al_ctrl_syss          = hal_rtl_otp_al_ctrl_syss,
	.hal_otp_byte_wr_marr_aon      = hal_rtl_otp_byte_wr_marr_aon,
	.hal_otp_byte_wr_marr_sys      = hal_rtl_otp_byte_wr_marr_sys,
	.hal_otp_byte_wr_marr_syss     = hal_rtl_otp_byte_wr_marr_syss,
	.hal_otp_adj_vol_lvl           = &otp_adj_vol_lvl,
	.hal_otp_rp_mar_rd_syss        = hal_rtl_otp_rp_mar_rd_syss,
	.hal_otp_rp_chk_syss           = hal_rtl_otp_rp_chk_syss,
	.hal_otp_rp_pg_syss            = hal_rtl_otp_rp_pg_syss,
	.hal_otp_set_aon_vol           = hal_rtl_otp_set_aon_vol,
};

/**
*  @brief Description of hal_rtl_otp_init
*         hal_rtl_otp_init is used to do OTP parameter initialization
*
*  @param[in]  void
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_init(void)
{
	DBG_OTP_INFO("otp init-> adpt addr: %08x, padpt addr: %08x\n", &otp_adpt, &_potp_adpt);
	_potp_adpt = &otp_adpt;
	_potp_adpt->slpc_en_dly = OTP_DEFAULT_SLPCEN_DLY;
	_potp_adpt->pc_en_dly   = OTP_DEFAULT_PCEN_DLY;
	_potp_adpt->wr_dly      = OTP_DEFAULT_WR_DLY;
	_potp_adpt->rd_dly      = OTP_DEFAULT_RD_DLY;
	otp_adj_vol_lvl = 0x04;
}

/**
*  @brief Description of hal_rtl_otp_deinit
*         hal_rtl_otp_deinit is used to do OTP parameter de-initialization
*
*  @param[in]  void
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_deinit(void)
{
	DBG_OTP_INFO("otp deinit-> adpt addr: %08x, padpt addr: %08x\n", &otp_adpt, &_potp_adpt);
	_potp_adpt->slpc_en_dly = (uint32_t)NULL;
	_potp_adpt->pc_en_dly   = (uint32_t)NULL;
	_potp_adpt->wr_dly      = (uint32_t)NULL;
	_potp_adpt->rd_dly      = (uint32_t)NULL;
	_potp_adpt = (hal_otp_adapter_t *)NULL;
	otp_adj_vol_lvl = 0xFF;
}

/**
*  @brief Description of hal_rtl_otp_ctrl_sel
*         hal_rtl_otp_ctrl_sel is used to select OTP access control source
*
*  @param[in]  uint8_t otp_ctrl_sel: select AON/SYSON control source
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_ctrl_sel(uint8_t otp_ctrl_sel)
{
	uint32_t reg_tmp = 0;

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_SYS_CTRL)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RW_REG_SEL);
	HAL_SET_BIT(reg_tmp, otp_ctrl_sel << AON_SHIFT_OTP_RW_REG_SEL);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_SYS_CTRL)), reg_tmp);
}

/**
*  @brief Description of hal_rtl_otp_ctrl_sel_sts
*         hal_rtl_otp_ctrl_sel_sts is used to show current selection of control source
*
*  @param[in]  void
*
*  @returns uint8_t curent selection of control sourcee
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_ctrl_sel_sts(void)
{
	uint32_t reg_tmp = 0;

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_SYS_CTRL)));
	reg_tmp = ((reg_tmp & AON_BIT_OTP_RW_REG_SEL) >> AON_SHIFT_OTP_RW_REG_SEL);
	return reg_tmp;
}

/**
*  @brief Description of hal_rtl_otp_pmc_gnt_ctrl
*         hal_rtl_otp_pmc_gnt_ctrl is used to control power control grant enable/disable
*
*  @param[in]  uint8_t gnt_en: power control grant control
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_pmc_gnt_ctrl(uint8_t gnt_en)
{
	uint32_t reg_tmp = 0;

	if (!gnt_en) {
		/* Clear grant bit after access power control */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)));
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OPT_PWR_CTRL_GNT);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)), reg_tmp);
	} else {
		/* Set grant bit before access power control */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)));
		HAL_SET_BIT(reg_tmp, AON_BIT_OPT_PWR_CTRL_GNT);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)), reg_tmp);;
	}

	DBG_OTP_INFO("c gnt reg(%x): %08x\n", (AON_BASE + (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL))),
				 HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL))));
}

/**
*  @brief Description of hal_rtl_otp_wr_gnt_ctrl
*         hal_rtl_otp_wr_gnt_ctrl is used to control write grant enable/disable
*
*  @param[in]  uint8_t gnt_en: write grant control
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_wr_gnt_ctrl(uint8_t gnt_en)
{
	uint32_t reg_tmp = 0;

	if (!gnt_en) {
		/* Clear grant bit after access power control */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)));
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_BURN_GNT);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)), reg_tmp);
	} else {
		/* Set grant bit before access power control */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)));
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_BURN_GNT);
		HAL_SET_BIT(reg_tmp, OTP_WR_GRNT_CODE);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)), reg_tmp);
	}

	DBG_OTP_INFO("wr gnt reg(%x): %08x\n", (AON_BASE + (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL))),
				 HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL))));
}

/**
*  @brief Description of hal_rtl_otpc_ctrl
*         hal_rtl_otpc_ctrl is used to configure OTP controller and its read/write
*         clock.
*
*  @param[in]  uint8_t otpc_en: OTP controller enable
*  @param[in]  uint8_t wclk_en: write clock enable
*  @param[in]  uint8_t rclk_en: read clock enable
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otpc_ctrl(uint8_t otpc_en, uint8_t wclk_en, uint8_t rclk_en)
{
	uint32_t reg_tmp = 0;
	uint32_t pwr_ctrl_bak;
	/* Check and backup PMC grant status */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PMC_DBG_CTRL)));
	pwr_ctrl_bak = HAL_READ_BIT(reg_tmp, AON_BIT_OPT_PWR_CTRL_GNT);
	if (pwr_ctrl_bak == 0) {
		DBG_OTP_WARN("enable pwr gnt\n");
		hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);
	}

	/* Configure clock first */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)));
	if (wclk_en) {
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_PGCLK_GT_EN);
	} else {
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_PGCLK_GT_EN);
	}

	if (rclk_en) {
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RDCLK_GT_EN);
	} else {
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDCLK_GT_EN);
	}

	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
	DBG_OTP_INFO("pwr ctrl(1)(%x): %08x\n", (AON_BASE + (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))),
				 HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))));
	/* Configure controller */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	if (!otpc_en) {
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_CTRLER_FEN);
	} else {
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_CTRLER_FEN);
	}

	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
	DBG_OTP_INFO("pwr ctrl(2)(%x): %08x\n", (AON_BASE + (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))),
				 HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))));
#else
	uint8_t rma_sts;
	rma_sts = hal_rtl_sys_get_rma_state();
	if (RMA_NORMAL_STATE == rma_sts) {
		if (!otpc_en) {
			HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_CTRLER_FEN);
		} else {
			HAL_SET_BIT(reg_tmp, AON_BIT_OTP_CTRLER_FEN);
		}

		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
		DBG_OTP_INFO("pwr ctrl(2)(%x): %08x\n", (AON_BASE + (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))),
					 HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL))));
	} else {
		if (!otpc_en) {
			DBG_OTP_WARN("Try to disable controller in RMA state\n");
		}
	}

#endif
	if (pwr_ctrl_bak == 0) {
		DBG_OTP_WARN("disable pwr gnt\n");
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
}

/**
*  @brief Description of hal_rtl_otp_pwr_ctrl
*         hal_rtl_otp_pwr_ctrl is used to enable/disable OTP power control
*
*  @param[in]  uint8_t pwr_en: OTP power control
*
*  @returns uint32_t: power control result
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_pwr_ctrl(uint8_t pwr_en)
{
	uint32_t reg_tmp = 0;
	uint32_t ret_sts = 0;

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)));
	if (!pwr_en) {
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_ISO2AON_EN);
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_ISO2PON_EN);
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_ISO2SYSON_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_PC_SL_EN);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_PC_BG_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
	} else {
		/* Enable small PC first */
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_PC_SL_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
		hal_delay_us(_potp_adpt->slpc_en_dly);
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_PC_BG_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_ISO2AON_EN);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_ISO2PON_EN);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_ISO2SYSON_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)), reg_tmp);
	}

	ret_sts = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PWR_CTRL)));

	return ret_sts;
}

/**
*  @brief Description of hal_rtl_otp_test_mod_cfg
*         hal_rtl_otp_test_mod_cfg is used to configure OTP test mode selection register
*
*  @param[in]  uint8_t test_mod: OTP test mode
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_test_mod_cfg(uint8_t test_mod)
{
	uint32_t reg_tmp = 0;

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)));
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_TEST_SEL);
	HAL_SET_BIT(reg_tmp, ((test_mod & OTP_TEST_MOD_MSK) << AON_SHIFT_OTP_TEST_SEL));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)), reg_tmp);

	DBG_OTP_INFO("Test Mode Cfg: %x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0))));
}

/**
*  @brief Description of hal_rtl_otp_test_row_ctrl
*         hal_rtl_otp_test_row_ctrl is used to enable/disable test row access
*
*  @param[in]  uint8_t test_row_en: test row enable control
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_test_row_ctrl(uint8_t test_row_en)
{
	uint32_t reg_tmp = 0;

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_TROW_EN);
	HAL_SET_BIT(reg_tmp, ((test_row_en & OTP_TEST_ROW_MSK) << AON_SHIFT_OTP_TROW_EN));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)), reg_tmp);
}

/**
*  @brief Description of hal_rtl_otp_chk_err_sts
*         hal_rtl_otp_chk_err_sts is used to check error status
*
*  @param[in]  void
*
*  @returns uint32_t: OTP error flag
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_chk_err_sts(void)
{
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)));
	ret_val = (reg_tmp & AON_BIT_OTP_ERR_FLAG) >> AON_SHIFT_OTP_ERR_FLAG;

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_dsb_ctrl
*         hal_rtl_otp_dsb_ctrl is used to control DSB enable
*
*  @param[in]  uint8_t dsb_en: OTP DSB control
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_dsb_ctrl(uint8_t dsb_en)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_DSB_EN);
	HAL_SET_BIT(reg_tmp, ((dsb_en & OTP_DSB_EN_MSK) << AON_SHIFT_OTP_DSB_EN));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)), reg_tmp);
}

/**
*  @brief Description of hal_rtl_otp_rb_ctrl
*         hal_rtl_otp_rb_ctrl is used to control reboot(PTRIM) enable/disable
*
*  @param[in]  uint8_t rb_en: reboot operaton enable/disable after repair program
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_rb_ctrl(uint8_t rb_en)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_1)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RB_CTRL);
	HAL_SET_BIT(reg_tmp, ((rb_en & OTP_RB_EN_MSK) << AON_SHIFT_OTP_RB_CTRL));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_1)), reg_tmp);
}

/**
*  @brief Description of hal_rtl_otp_cz_ctrl
*         hal_rtl_otp_cz_ctrl is used to configure counting zero control
*
*  @param[in]  uint8_t cz_en: OTP counting zero enable/disable
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_cz_ctrl(uint8_t cz_en)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_B_OTP_CNTZ_CTRL);
	HAL_SET_BIT(reg_tmp, ((cz_en & OTP_CZ_EN_MSK) << AON_SHIFT_B_OTP_CNTZ_CTRL));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)), reg_tmp);
}

/**
*  @brief Description of hal_rtl_otp_chk_cz_sts
*         hal_rtl_otp_chk_cz_sts is used to grep CZ status according to a given key selection
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t: CZ status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_chk_cz_sts(uint32_t key_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)));

	if (key_sel == OTP_CZ_STS_ALL) {
		return reg_tmp;
	} else {
		reg_tmp &= (0x01 << key_sel);
		reg_tmp = reg_tmp >> key_sel;
		return reg_tmp;
	}
}

/**
*  @brief Description of hal_rtl_otp_clr_cz_sts
*         hal_rtl_otp_clr_cz_sts is used to clear CZ status register and return value after clearing
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t:¡@CZ status after clearing
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_cz_sts(uint32_t key_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)));

	if (key_sel == OTP_CZ_STS_ALL) {
		reg_tmp = OTP_CZ_STS_ALL;
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)), reg_tmp);
		return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)));
	} else {
		reg_tmp |= (OTP_CZ_EN_MSK << key_sel);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)), reg_tmp);
		return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_OTP_CNT_Z_CTRL)));
	}
}

/**
*  @brief Description of hal_rtl_otp_prct_ctrl
*         hal_rtl_otp_prct_ctrl is used to configure OTP protect control and return the status
*
*  @param[in]  uint8_t prct_en: protectoin control enable/disable
*
*  @returns uint32_t: protection control status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_prct_ctrl(uint8_t prct_en)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_B_OTP_PRCT_CTRL);
	HAL_SET_BIT(reg_tmp, ((prct_en & OTP_PRCT_EN_MSK) << AON_SHIFT_B_OTP_PRCT_CTRL));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);

	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
}

/**
*  @brief Description of hal_rtl_otp_chk_prct_sts
*         hal_rtl_otp_chk_prct_sts is used to check protect status according to a given selection.
*
*  @param[in]  uint32_t pwr_en: protection selection
*
*  @returns uint32_t: protection status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_chk_prct_sts(uint32_t prct_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));

	if (prct_sel == OTP_PRCT_STS_ALL) {
		return reg_tmp;
	} else {
		reg_tmp &= (0x01 << prct_sel);
		reg_tmp = reg_tmp >> prct_sel;
		return reg_tmp;
	}
}

/**
*  @brief Description of hal_rtl_otp_rd_lz_er_addr
*         hal_rtl_otp_rd_lz_er_addr is used to read logical zone CRC error address
*         which is start address of a entry
*
*  @param[in]  void
*
*  @returns uint32_t: logical zone CRC error address
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_lz_er_addr(void)
{
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));

	ret_val = (reg_tmp & AON_MASK_B_OTP_LZ_CRC_ER_ADDR) >> AON_SHIFT_B_OTP_LZ_CRC_ER_ADDR;
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_clr_prct_sts
*         hal_rtl_otp_clr_prct_sts is used to clear protect status according to a given selection
*         and return result after clear
*
*  @param[in]  uint32_t pwr_en: protection selection, 0xFFFFFFFF is to clear all
*
*  @returns uint32_t protection status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_prct_sts(uint32_t prct_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));

	if (prct_sel == OTP_PRCT_STS_ALL) {
		reg_tmp = OTP_CZ_STS_ALL;
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);
		return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	} else {
		reg_tmp |= (OTP_PRCT_EN_MSK << prct_sel);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);
		return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	}
}

/**
*  @brief Description of hal_rtl_otp_rd_sz_prct_sts
*         hal_rtl_otp_rd_sz_prct_sts is used to read secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t: protection status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_sz_prct_sts(void)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	reg_tmp &= AON_MASK_B_OTP_SZ_CRC_STS;
	reg_tmp = reg_tmp >> AON_SHIFT_B_OTP_SZ_CRC_STS;
	return reg_tmp;
}

/**
*  @brief Description of hal_rtl_otp_clr_sz_prct_sts
*         hal_rtl_otp_clr_sz_prct_sts is used to clear secure zone protection status
*
*  @param[in]  uint8_t sz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t:   secure zone protecion status after clearing
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_sz_prct_sts(uint32_t sz_prct_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	reg_tmp |= OTP_PRCT_EN_MSK << (sz_prct_sts_sel);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);
	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
}

/**
*  @brief Description of hal_rtl_otp_rd_ssz_prct_sts
*         hal_rtl_otp_rd_ssz_prct_sts is used to read super secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t:   super secure zone status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_ssz_prct_sts(void)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	reg_tmp &= AON_MASK_B_OTP_SSZ_CRC_STS;
	reg_tmp = reg_tmp >> AON_SHIFT_B_OTP_SSZ_CRC_STS;
	return reg_tmp;
}

/**
*  @brief Description of hal_rtl_otp_clr_ssz_prct_sts
*         hal_rtl_otp_clr_ssz_prct_sts is used to clear super secure zone status
*
*  @param[in]  uint32_t ssz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t: super secure zone protection status after clearing
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_ssz_prct_sts(uint32_t ssz_prct_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	reg_tmp |= OTP_PRCT_EN_MSK << (ssz_prct_sts_sel);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);
	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
}

/**
*  @brief Description of hal_rtl_otp_szwl_ctrl
*         hal_rtl_otp_szwl_ctrl is used to configure secure zone write lock function
*
*  @param[in]  uint32_t sz_wl_en: enable/disable secure zone write lock function
*                                 1: enable, 0: disable
*
*  @returns uint32_t: protection control regsiter
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_szwl_ctrl(uint32_t sz_wl_en)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_B_OTP_SZ_WL_CTRL);
	HAL_SET_BIT(reg_tmp, ((sz_wl_en & OTP_PRCT_SZ_WL_MSK) << AON_SHIFT_B_OTP_SZ_WL_CTRL));
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)), reg_tmp);

	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_PRCT_CTRL)));
}


/**
*  @brief Description of hal_rtl_otp_rd_aldn_sts
*         hal_rtl_otp_rd_aldn_sts is used to read autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status bit
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_aldn_sts(uint32_t aldn_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	reg_tmp = ((reg_tmp & (OTP_ALDN_STS_MSK << aldn_sts_sel)) >> aldn_sts_sel);
	return reg_tmp;
}

/**
*  @brief Description of hal_rtl_otp_clr_aldn_sts
*         hal_rtl_otp_clr_aldn_sts is used to clear autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status register
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_aldn_sts(uint32_t aldn_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	if (aldn_sts_sel == OTPAlSysonDnSts) {
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_ALDN_SYSON_STS);
	} else {
		reg_tmp |= (OTP_ALDN_STS_MSK << aldn_sts_sel);
	}

	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)), reg_tmp);
	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
}

/**
*  @brief Description of hal_rtl_otp_rd_al_sts
*         hal_rtl_otp_rd_al_sts is used to read autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status bit
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_al_sts(uint32_t al_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	reg_tmp = ((reg_tmp & (OTP_AL_STS_MSK << al_sts_sel)) >> al_sts_sel);
	return reg_tmp;
}

/**
*  @brief Description of hal_rtl_otp_clr_al_sts
*         hal_rtl_otp_clr_al_sts is used to clear autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status register
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_clr_al_sts(uint32_t al_sts_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	if (al_sts_sel == OTPAlSysonSts) {
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_AL_SYSON_STS);
	} else {
		reg_tmp |= OTP_AL_STS_MSK << al_sts_sel;
	}

	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)), reg_tmp);
	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
}

/**
*  @brief Description of hal_rtl_otp_al_cfg
*         hal_rtl_otp_al_cfg is used to configure autoload enable according to a given selection
*
*  @param[in]  uint32_t al_trg_sel: autoload target selection
*
*  @returns uint32_t: autoload status regsiter
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_al_cfg(uint32_t al_trg_sel)
{
	uint32_t reg_tmp = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	HAL_CLEAR_BIT(reg_tmp, (AON_BIT_OTP_AL_AON_EN | AON_BIT_OTP_AL_PON_EN | AON_BIT_OTP_AL_SYSON_EN));
	HAL_SET_BIT(reg_tmp, al_trg_sel << AON_SHIFT_OTP_AL_AON_EN);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)), reg_tmp);
	return HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
}

/**
*  @brief Description of hal_rtl_otp_al_ctrl_aon
*         hal_rtl_otp_al_ctrl_aon is used to control autoload enable/disable flow by AON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_al_ctrl_aon(uint32_t al_en)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t reg_tmp = 0;
	uint32_t to_cnt = 0;
	uint32_t al_cfg;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = 0;

	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	/* change access control to AON */
	hal_rtl_otp_ctrl_sel(OTPCtrlAon);

	DBG_OTP_INFO("al_sts before enable: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS))));
	/* set mode first */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_DATA);
	HAL_SET_BIT(reg_tmp, (OTPALEnMod << AON_SHIFT_OTP_MODE_SEL));
	/* clear ready bit first */
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
	DBG_OTP_INFO("chk AS_A: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A))));
	DBG_OTP_INFO("wr to AS_A: %08x\n", reg_tmp);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

	/* enable autoload operation */
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
	DBG_OTP_INFO("wr to AS_A: %08x\n", reg_tmp);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

	to_cnt = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	DBG_OTP_INFO("AL_STS: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS))));
	al_cfg = (reg_tmp & (AON_BIT_OTP_AL_AON_EN | AON_BIT_OTP_AL_PON_EN | AON_BIT_OTP_AL_SYSON_EN));
	/* wait for autoload done */
	while (((reg_tmp & (al_cfg << AON_SHIFT_OTP_ALDN_AON_STS)) == 0) && (to_cnt < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
		DBG_OTP_INFO("AL_STS(%d): %08x\r\n", to_cnt, reg_tmp);
		to_cnt++;
	}

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

	if (to_cnt == OTP_RW_TO_CNT) {
		DBG_OTP_ERR("timeout!!!\n");
		to_cnt = OTP_RET_FAIL_FLG;
	} else {
		to_cnt = OTPStsSuccess;
	}

	/* restore access control selection */
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable pwoer cut first then disable conroller,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(OTP_DEFAULT_PCEN_DLY);

	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return to_cnt;
}

/**
*  @brief Description of hal_rtl_otp_bust_ctrl
*         hal_rtl_otp_bust_ctrl is used to control burst operatoin enable/disable
*
*  @param[in]  uint8_t bust_en: burst enable
*
*  @returns uint32_t: operation succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_bust_ctrl(uint8_t bust_en)
{
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)));
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_BURST);
	HAL_SET_BIT(reg_tmp, (bust_en & OTP_BUST_EN_MSK) << AON_SHIFT_OTP_BURST);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_CTRL_0)), reg_tmp);

	if (bust_en == OTP_DISABLE) {
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_RD_CHK_INTV);
			reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
			ret_val++;
		}

		if (ret_val == OTP_RW_TO_CNT) {
			ret_val = OTP_RET_FAIL_FLG;
		} else {
			ret_val = OTPStsSuccess;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rd_sub_aon
*         hal_rtl_otp_rd_sub_aon is used for read sub looping function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_sub_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint32_t rd_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (rd_cnt = 0; rd_cnt < len; rd_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (rd_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif
		/* set mode/addr first */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPRdMod << AON_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + rd_cnt)& OTP_ADDR_MSK) << AON_SHIFT_OTP_ADDR));
		/* clear ready bit first */
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
		/* enable read operation */
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

		ret_val = 0;
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_RD_CHK_INTV);
			reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		*(rd_data + rd_cnt) = reg_tmp & AON_MASK_OTP_DATA;
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_ERR("TO.(%d)\n", rd_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= rd_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/* Disable power cut */
		if ((pwr_cl_en != 0) || (rd_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != OTPStsSuccess)) {
			/* Disable OTP memory power */
			DBG_OTP_INFO("disable power\n");
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rd_aon
*         hal_rtl_otp_rd_aon is used for read function by AON
*
*  @param[in]  uint8_t pwr_cl_en: in test chip, power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*                                 in MP chip, this param is used to control disable power and controller after read process finished.
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlAon);
	/* to do read operation*/
	ret_val = hal_rtl_otp_rd_sub_aon(pwr_cl_en, addr, rd_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable power cut first then disable conroller,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif


	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_rd_aon
*         hal_rtl_otp_byte_rd_aon is used to read a single byte according to given address
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_rd_aon(uint32_t addr)
{
	uint8_t rd_dat_tmp;
	hal_rtl_otp_rd_aon(OTP_ENABLE, addr, &rd_dat_tmp, 1);
	return rd_dat_tmp;
}

/**
*  @brief Description of hal_rtl_otp_wr_sub_aon
*         hal_rtl_otp_wr_sub_aon is used for write sub looping function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_sub_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint32_t wr_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (wr_cnt = 0; wr_cnt < len; wr_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (wr_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		/* enable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_ENABLE);
		/* set mode/addr/data first */
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPWrMod << AON_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + wr_cnt)& OTP_ADDR_MSK) << AON_SHIFT_OTP_ADDR));
		HAL_SET_BIT(reg_tmp, (*(wr_data + wr_cnt) << AON_SHIFT_OTP_DATA));
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
		/* enable write operation */
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
		hal_delay_us(_potp_adpt->wr_dly);

		ret_val = 0;
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_WR_CHK_INTV);
			reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_INFO("TO.(%d)\n", wr_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= wr_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

		/* disable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_DISABLE);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/**/
		if ((pwr_cl_en != 0) || (wr_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != 0)) {
			DBG_OTP_INFO("diable power\n");
			/* Disable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_wr_aon
*         hal_rtl_otp_wr_aon is used for write function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	DBG_OTP_INFO("pc: %x, addr: %08x, data: %02x, len: %d\n", pwr_cl_en, addr, *wr_data, len);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlAon);

	ret_val = hal_rtl_otp_wr_sub_aon(pwr_cl_en, addr, wr_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif

	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_rd_aon
*         hal_rtl_otp_byte_rd_aon is used to do a one-byte write operation
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_byte_wr_aon(uint32_t addr, uint8_t wr_data)
{
	hal_rtl_otp_wr_aon(OTP_ENABLE, addr, &wr_data, 1);
}

/**
*  @brief Description of hal_rtl_otp_cmp_aon
*         hal_rtl_otp_cmp_aon is used to do a comparison between given OTP memory address and data
*
*  @param[in]  uint32_t addr: OTP memory address which contains data to be compare with given data
*  @param[in]  uint32_t addr: given data to be compared with data of target OTP address
*
*  @returns uint32_t: comparison result, succeeded(0) failed(1)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_cmp_aon(uint32_t addr, uint8_t cmp_data)
{
	uint32_t reg_tmp = 0;
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);
	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif


	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlAon);

	/* set mode/addr/data first */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));

	/* clear ready, compare result */
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
	DBG_OTP_INFO("to write AS_CTRL_A -1: %08x\n", reg_tmp);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);

	/* set address, data, mode */
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, AON_MASK_OTP_DATA);

	HAL_SET_BIT(reg_tmp, (OTPCmpMod << AON_SHIFT_OTP_MODE_SEL));
	HAL_SET_BIT(reg_tmp, ((addr & OTP_ADDR_MSK) << AON_SHIFT_OTP_ADDR));
	HAL_SET_BIT(reg_tmp, (cmp_data << AON_SHIFT_OTP_DATA));

	DBG_OTP_INFO("to write AS_CTRL_A -2: %08x\n", reg_tmp);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
	DBG_OTP_INFO("after write AS_CTRL_A -2: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A))));
	/* enable compare operation */
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
	DBG_OTP_INFO("to write AS_CTRL_A -3: %08x\n", reg_tmp);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
	hal_delay_us(_potp_adpt->rd_dly);
	DBG_OTP_INFO("after write AS_CTRL_A -3: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A))));

	ret_val = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	/* wait for operation done */
	while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		ret_val++;
	}

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	DBG_OTP_INFO("as_ctrl after ready: %08x\n", reg_tmp);
	reg_tmp = HAL_READ_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT) >> AON_SHIFT_OTP_COMP_RESULT;

	if (ret_val == OTP_RW_TO_CNT) {
		DBG_OTP_INFO("timeout\n");
		ret_val = OTP_RET_FAIL_FLG;
	} else {
		ret_val = reg_tmp;
	}

	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, AON_BIT_OTP_OP_EN);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
#if 0
	/* clear compare result anyway */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
#endif

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	DBG_OTP_INFO("disable power\n");
	/* Disable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}


/**
*  @brief Description of hal_rtl_otp_rd_sub_sys
*         hal_rtl_otp_rd_sub_sys is used for read sub looping function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_sub_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint32_t rd_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (rd_cnt = 0; rd_cnt < len; rd_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (rd_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif
		/* set mode/addr first */
		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPRdMod << SYSON_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + rd_cnt)& OTP_ADDR_MSK) << SYSON_SHIFT_OTP_ADDR));
		/* clear ready bit first */
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
		/* enable read operation */
		HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

		ret_val = 0;
		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, SYSON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_RD_CHK_INTV);
			reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		*(rd_data + rd_cnt) = reg_tmp & SYSON_MASK_OTP_DATA;
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_ERR("TO.(%d)\n", rd_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= rd_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/**/
		if ((pwr_cl_en != 0) || (rd_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != 0)) {
			/* Disable OTP memory power */
			DBG_OTP_INFO("disable power\n");
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rd_sys
*         hal_rtl_otp_rd_sys is used for read function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);
	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlSys);
	/* to do read operation*/
	ret_val = hal_rtl_otp_rd_sub_sys(pwr_cl_en, addr, rd_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable power cut first then disable conroller,
	   In MP chip, this flow should be revered. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif
	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_rd_sys
*         hal_rtl_otp_byte_rd_sys is used to read a single byte according to given address
*         by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_rd_sys(uint32_t addr)
{
	uint8_t rd_dat_tmp;
	hal_rtl_otp_rd_sys(OTP_ENABLE, addr, &rd_dat_tmp, 1);
	return rd_dat_tmp;
}


/**
*  @brief Description of hal_rtl_otp_wr_sub_aon
*         hal_rtl_otp_wr_sub_aon is used for write sub looping function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_sub_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint32_t wr_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (wr_cnt = 0; wr_cnt < len; wr_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (wr_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		/* enable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_ENABLE);

		/* set mode/addr/data first */
		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPWrMod << SYSON_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + wr_cnt) & OTP_ADDR_MSK) << SYSON_SHIFT_OTP_ADDR));
		HAL_SET_BIT(reg_tmp, (*(wr_data + wr_cnt) << SYSON_SHIFT_OTP_DATA));
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
		/* enable write operation */
		HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
		hal_delay_us(_potp_adpt->wr_dly);

		ret_val = 0;
		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, SYSON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_WR_CHK_INTV);
			reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
		HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_INFO("TO.(%d)\n", wr_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= wr_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

		/* disable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_DISABLE);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/**/
		if ((pwr_cl_en != 0) || (wr_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != 0)) {
			DBG_OTP_INFO("diable power\n");
			/* Disable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_wr_sys
*         hal_rtl_otp_wr_sys is used for write function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	DBG_OTP_INFO("pc: %x, addr: %08x, data: %02x, len: %d\n", pwr_cl_en, addr, *wr_data, len);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	ret_val = hal_rtl_otp_wr_sub_sys(pwr_cl_en, addr, wr_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif
	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_wr_sys
*         hal_rtl_otp_byte_wr_sys is used to do a one-byte write operation by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_byte_wr_sys(uint32_t addr, uint8_t wr_data)
{
	hal_rtl_otp_wr_sys(OTP_ENABLE, addr, &wr_data, 1);
}

/**
*  @brief Description of hal_rtl_otp_cmp_sys
*         hal_rtl_otp_cmp_sys is used to do a comparison between given OTP memory address and data
*         by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address which contains data to be compare with given data
*  @param[in]  uint32_t addr: given data to be compared with data of target OTP address
*
*  @returns uint32_t: comparison result, succeeded(0) failed(1)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_cmp_sys(uint32_t addr, uint8_t cmp_data)
{
	uint32_t reg_tmp = 0;
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);
	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	/* set mode/addr/data first */
	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));

	/* clear ready, compare result */
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_COMP_RESULT);
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	DBG_OTP_INFO("to write AS_CTRL_SS -1: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

	/* set address, data, mode */
	HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_DATA);

	HAL_SET_BIT(reg_tmp, (OTPCmpMod << SYSON_S_SHIFT_OTP_MODE_SEL));
	HAL_SET_BIT(reg_tmp, ((addr & OTP_ADDR_MSK) << SYSON_SHIFT_OTP_ADDR));
	HAL_SET_BIT(reg_tmp, (cmp_data << SYSON_SHIFT_OTP_DATA));

	DBG_OTP_INFO("to write AS_CTRL_S -2: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
	DBG_OTP_INFO("after write AS_CTRL_S -2: %08x\n", HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS))));
	/* enable compare operation */
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
	DBG_OTP_INFO("to write AS_CTRL_SS -3: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
	hal_delay_us(_potp_adpt->rd_dly);
	DBG_OTP_INFO("after write AS_CTRL_SS -3: %08x\n", HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS))));

	ret_val = 0;
	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
	/* wait for operation done */
	while ((HAL_READ_BIT(reg_tmp, SYSON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
		ret_val++;
	}

	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
	DBG_OTP_INFO("as_ctrl after ready: %08x\n", reg_tmp);
	reg_tmp = HAL_READ_BIT(reg_tmp, SYSON_BIT_OTP_COMP_RESULT) >> SYSON_SHIFT_OTP_COMP_RESULT;

	if (ret_val == OTP_RW_TO_CNT) {
		DBG_OTP_INFO("timeout\n");
		ret_val = OTP_RET_FAIL_FLG;
	} else {
		ret_val = reg_tmp;
	}

	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
#if 0
	/* clear compare result anyway */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
#endif

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	DBG_OTP_INFO("disable power\n");
	/* Disable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_al_ctrl_sys
*         hal_rtl_otp_al_ctrl_sys is used to control autoload enable/disable flow by SYSON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_al_ctrl_sys(uint32_t al_en)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t reg_tmp = 0;
	uint32_t to_cnt = 0;
	uint32_t al_cfg;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	DBG_OTP_INFO("al_sts before enable: %08x\n", HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS))));
	/* set mode first */
	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_DATA);
	HAL_SET_BIT(reg_tmp, (OTPALEnMod << SYSON_SHIFT_OTP_MODE_SEL));
	/* clear ready bit first */
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	DBG_OTP_INFO("chk AS_S: %08x\n", HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS))));
	DBG_OTP_INFO("wr to AS_S: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);
	/* enable autoload operation */
	HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
	DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

	//hal_rtl_otp_ctrl_sel(OTPCtrlAon);
	to_cnt = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	DBG_OTP_INFO("AL_STS: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS))));
	al_cfg = (reg_tmp & (AON_BIT_OTP_AL_AON_EN | AON_BIT_OTP_AL_PON_EN | AON_BIT_OTP_AL_SYSON_EN));
	/* wait for autoload done */
	while (((reg_tmp & (al_cfg << AON_SHIFT_OTP_ALDN_AON_STS)) == 0) && (to_cnt < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
		DBG_OTP_INFO("AL_STS(%d): %08x\r\n", to_cnt, reg_tmp);
		to_cnt++;
	}

	//hal_rtl_otp_ctrl_sel(OTPCtrlSys);
	reg_tmp = HAL_READ32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)));
	HAL_SET_BIT(reg_tmp, SYSON_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, SYSON_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_BIT_OTP_OP_EN);
	HAL_WRITE32(SYSON_BASE, (uint32_t)(&(SYSON_BASE_REG->SYSON_REG_OTP_AS_CTRL_NS)), reg_tmp);

	if (to_cnt == OTP_RW_TO_CNT) {
		DBG_OTP_ERR("timeout!!!\n");
		to_cnt = OTP_RET_FAIL_FLG;
	} else {
		to_cnt = OTPStsSuccess;
	}

	/* restore access control selection */
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable pwoer cut first then disable conroller,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(OTP_DEFAULT_PCEN_DLY);

	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return to_cnt;
}




/**
*  @brief Description of hal_rtl_otp_rd_sub_syss
*         hal_rtl_otp_rd_sub_syss is used for read sub looping function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_sub_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint32_t rd_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (rd_cnt = 0; rd_cnt < len; rd_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (rd_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif
		/* set mode/addr first */
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPRdMod << SYSON_S_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + rd_cnt)& OTP_ADDR_MSK) << SYSON_S_SHIFT_OTP_ADDR));
		/* clear ready bit first */
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
		/* enable read operation */
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		ret_val = 0;
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_RD_CHK_INTV);
			reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		*(rd_data + rd_cnt) = reg_tmp & SYSON_MASK_OTP_DATA;
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_ERR("timeout(%d)\n", rd_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= rd_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/**/
		if ((pwr_cl_en != 0) || (rd_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != 0)) {
			/* Disable OTP memory power */
			DBG_OTP_INFO("disable power\n");
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif
		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rd_syss
*         hal_rtl_otp_rd_syss is used for read function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_rd_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);
	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlSys);
	/* to do read operation*/
	ret_val = hal_rtl_otp_rd_sub_syss(pwr_cl_en, addr, rd_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable power cut first then disable conroller,
	   In MP chip, this flow should be revered. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif
	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_rd_syss
*         hal_rtl_otp_byte_rd_syss is used to read a single byte according to given address
*         by SYSON-S
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_rd_syss(uint32_t addr)
{
	uint8_t rd_dat_tmp;
	hal_rtl_otp_rd_syss(OTP_ENABLE, addr, &rd_dat_tmp, 1);
	return rd_dat_tmp;
}

/**
*  @brief Description of hal_rtl_otp_wr_sub_syss
*         hal_rtl_otp_wr_sub_syss is used for write sub looping function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_sub_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint32_t wr_cnt = 0;
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	for (wr_cnt = 0; wr_cnt < len; wr_cnt++) {
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		if ((pwr_cl_en != 0) || (wr_cnt == 0)) {
			DBG_OTP_INFO("enable power\n");
			/* Enable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif
		/* enable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_ENABLE);

		/* set mode/addr/data first */
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPWrMod << SYSON_S_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, (((addr + wr_cnt) & OTP_ADDR_MSK) << SYSON_S_SHIFT_OTP_ADDR));
		HAL_SET_BIT(reg_tmp, (*(wr_data + wr_cnt) << SYSON_S_SHIFT_OTP_DATA));
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
		/* enable write operation */
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_A: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
		hal_delay_us(_potp_adpt->wr_dly);

		ret_val = 0;
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_WR_CHK_INTV);
			reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_INFO("TO.(%d)\n", wr_cnt);
			ret_val = OTP_RET_FAIL_FLG;
			ret_val |= wr_cnt;
		} else {
			ret_val = OTPStsSuccess;
		}

		/* disable grant for write operation */
		hal_rtl_otp_wr_gnt_ctrl(OTP_DISABLE);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
		/**/
		if ((pwr_cl_en != 0) || (wr_cnt == len - 1) || ((ret_val & OTP_RET_FAIL_FLG) != 0)) {
			DBG_OTP_INFO("diable power\n");
			/* Disable OTP memory power */
			hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
			hal_delay_us(_potp_adpt->pc_en_dly);
		}
#endif

		if ((ret_val & OTP_RET_FAIL_FLG) != 0) {
			break;
		}
	}

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_wr_syss
*         hal_rtl_otp_wr_syss is used for write function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_wr_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	DBG_OTP_INFO("pc: %x, addr: %08x, data: %02x, len: %d\n", pwr_cl_en, addr, *wr_data, len);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	ret_val = hal_rtl_otp_wr_sub_syss(pwr_cl_en, addr, wr_data, len);

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
#else
	if (OTP_ENABLE == pwr_cl_en) {
		/* Disable controller */
		hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

		/* Disable OTP memory power */
		DBG_OTP_INFO("disable power\n");
		hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
		hal_delay_us(_potp_adpt->pc_en_dly);

		/* Disable PMC control */
		hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);
	}
#endif
	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_wr_syss
*         hal_rtl_otp_byte_wr_syss is used to do a one-byte write operation by SYSON-S
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*
*  @returns void
*/
SECTION_OTP_TEXT
void hal_rtl_otp_byte_wr_syss(uint32_t addr, uint8_t wr_data)
{
	hal_rtl_otp_wr_syss(OTP_ENABLE, addr, &wr_data, 1);
}

/**
*  @brief Description of hal_rtl_otp_cmp_syss
*         hal_rtl_otp_cmp_syss is used to do a comparison between given OTP memory address and data
*         by SYSON-S
*
*  @param[in]  uint32_t addr: OTP memory address which contains data to be compare with given data
*  @param[in]  uint32_t addr: given data to be compared with data of target OTP address
*
*  @returns uint32_t: comparison result, succeeded(0) failed(1)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_cmp_syss(uint32_t addr, uint8_t cmp_data)
{
	uint32_t reg_tmp = 0;
	uint8_t as_ctrl_sel_bak;
	uint32_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = 0;

	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	/* set mode/addr/data first */
	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));

	/* clear ready, compare result */
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_COMP_RESULT);
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	DBG_OTP_INFO("to write AS_CTRL_SS -1: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

	/* set address, data, mode */
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_DATA);

	HAL_SET_BIT(reg_tmp, (OTPCmpMod << SYSON_S_SHIFT_OTP_MODE_SEL));
	HAL_SET_BIT(reg_tmp, ((addr & OTP_ADDR_MSK) << SYSON_S_SHIFT_OTP_ADDR));
	HAL_SET_BIT(reg_tmp, (cmp_data << SYSON_S_SHIFT_OTP_DATA));

	DBG_OTP_INFO("to write AS_CTRL_SS -2: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
	DBG_OTP_INFO("after write AS_CTRL_SS -2: %08x\n", HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S))));
	/* enable compare operation */
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
	DBG_OTP_INFO("to write AS_CTRL_SS -3: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
	hal_delay_us(_potp_adpt->rd_dly);
	DBG_OTP_INFO("after write AS_CTRL_SS -3: %08x\n", HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S))));

	ret_val = 0;
	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
	/* wait for operation done */
	while ((HAL_READ_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		ret_val++;
	}

	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
	DBG_OTP_INFO("as_ctrl after ready: %08x\n", reg_tmp);
	reg_tmp = HAL_READ_BIT(reg_tmp, SYSON_S_BIT_OTP_COMP_RESULT) >> SYSON_S_SHIFT_OTP_COMP_RESULT;

	if (ret_val == OTP_RW_TO_CNT) {
		DBG_OTP_INFO("timeout\n");
		ret_val = OTP_RET_FAIL_FLG;
	} else {
		ret_val = reg_tmp;
	}

	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_COMP_RESULT);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
#if 0
	/* clear compare result anyway */
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	HAL_SET_BIT(reg_tmp, AON_BIT_OTP_COMP_RESULT);
	HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)), reg_tmp);
#endif

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

#if IS_CUT_TEST(CONFIG_CHIP_VER)
	DBG_OTP_INFO("disable power\n");
	/* Disable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return ret_val;
}


/**
*  @brief Description of hal_rtl_otp_al_ctrl_aon
*         hal_rtl_otp_al_ctrl_aon is used to control autoload enable/disable flow by SYSON-S
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_al_ctrl_syss(uint32_t al_en)
{
	uint8_t as_ctrl_sel_bak;
	uint32_t reg_tmp = 0;
	uint32_t to_cnt = 0;
	uint32_t al_cfg;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = 0;

	//hal_rtl_otp_ctrl_sel(OTPCtrlAon);
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);

	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#else
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif

	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	DBG_OTP_INFO("al_sts before enable: %08x\n", HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S))));
	/* set mode first */
	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_ADDR);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_DATA);
	HAL_SET_BIT(reg_tmp, (OTPALEnMod << SYSON_S_SHIFT_OTP_MODE_SEL));
	/* clear ready bit first */
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	DBG_OTP_INFO("chk AS_SS: %08x\n", HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S))));
	DBG_OTP_INFO("wr to AS_SS: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);
	/* enable autoload operation */
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
	DBG_OTP_INFO("wr to AS_SS: %08x\n", reg_tmp);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

	//hal_rtl_otp_ctrl_sel(OTPCtrlAon);
	to_cnt = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
	DBG_OTP_INFO("AL_STS: %08x\n", HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS))));
	al_cfg = (reg_tmp & (AON_BIT_OTP_AL_AON_EN | AON_BIT_OTP_AL_PON_EN | AON_BIT_OTP_AL_SYSON_EN));
	/* wait for autoload done */
	while (((reg_tmp & (al_cfg << AON_SHIFT_OTP_ALDN_AON_STS)) == 0) && (to_cnt < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AL_CTRL_STS)));
		DBG_OTP_INFO("AL_STS(%d): %08x\r\n", to_cnt, reg_tmp);
		to_cnt++;
	}

	//hal_rtl_otp_ctrl_sel(OTPCtrlSys);
	reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
	HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
	HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
	HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

	if (to_cnt == OTP_RW_TO_CNT) {
		DBG_OTP_ERR("timeout!!!\n");
		to_cnt = OTP_RET_FAIL_FLG;
	} else {
		to_cnt = OTPStsSuccess;
	}

	/* restore access control selection */
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* In test chip, disable pwoer cut first then disable conroller,
	   In MP chip, this flow should be reversed. */
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(OTP_DEFAULT_PCEN_DLY);

	/* Disable PMC control */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);
#else
	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);
#endif
	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);
	return to_cnt;
}

/**
*  @brief Description of hal_rtl_otp_byte_wr_marr_aon
*         hal_rtl_otp_byte_wr_marr_aon is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_wr_marr_aon(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	uint8_t ret_val = 0xFF;
	uint32_t otp_tmp_val;
	uint32_t otp_aon_ldo_bak;
	uint32_t otp_aon_ldo_adj;
	uint8_t as_ctrl_sel_bak;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_wr_aon(OTP_DISABLE, addr, &wr_data, 1);

	/* set control sel to AON*/
	hal_rtl_otp_ctrl_sel(OTPCtrlAon);

	/* Adjust AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		otp_aon_ldo_bak = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)));
		otp_aon_ldo_adj = otp_aon_ldo_bak & 0xFFFFFF0F;
		if (0xFF != otp_adj_vol_lvl) {
			otp_aon_ldo_adj = otp_aon_ldo_adj | ((otp_adj_vol_lvl & 0x0F) << AON_SHIFT_LDOH09_V09ADJ_L_3_0);
			HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_adj);
		}

		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	DBG_OTP_INFO("set pg margin read\n");
	hal_rtl_otp_test_mod_cfg(OTPPgMarRdMod);


	otp_tmp_val = hal_rtl_otp_rd_sub_aon(OTP_DISABLE, addr, &ret_val, 1);
	if (OTP_RET_FAIL_FLG == otp_tmp_val) {
		DBG_OTP_ERR("PG margin read error\n");
		ret_val = 0xFF;
	}

	DBG_OTP_INFO("pg margin read data: %x\n", ret_val);
	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);

	/* restore AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_bak);
		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_wr_marr_sys
*         hal_rtl_otp_byte_wr_marr_sys is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_wr_marr_sys(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	uint8_t ret_val = 0xFF;
	uint32_t otp_tmp_val;
	uint32_t otp_aon_ldo_bak;
	uint32_t otp_aon_ldo_adj;
	uint8_t as_ctrl_sel_bak;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_wr_sys(OTP_DISABLE, addr, &wr_data, 1);

	/* set control sel to AON*/
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	/* Adjust AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		otp_aon_ldo_bak = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)));
		otp_aon_ldo_adj = otp_aon_ldo_bak & 0xFFFFFF0F;
		if (0xFF != otp_adj_vol_lvl) {
			otp_aon_ldo_adj = otp_aon_ldo_adj | ((otp_adj_vol_lvl & 0x0F) << AON_SHIFT_LDOH09_V09ADJ_L_3_0);
			HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_adj);
		}

		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	DBG_OTP_INFO("set pg margin read\n");
	hal_rtl_otp_test_mod_cfg(OTPPgMarRdMod);


	otp_tmp_val = hal_rtl_otp_rd_sub_sys(OTP_DISABLE, addr, &ret_val, 1);
	if (OTP_RET_FAIL_FLG == otp_tmp_val) {
		DBG_OTP_ERR("PG margin read error\n");
		ret_val = 0xFF;
	}

	DBG_OTP_INFO("pg margin read data: %x\n", ret_val);
	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);

	/* restore AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_bak);
		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_byte_wr_marr_syss
*         hal_rtl_otp_byte_wr_marr_syss is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_byte_wr_marr_syss(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	uint8_t ret_val = 0xFF;
	uint32_t otp_tmp_val;
	uint32_t otp_aon_ldo_bak;
	uint32_t otp_aon_ldo_adj;
	uint8_t as_ctrl_sel_bak;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_wr_syss(OTP_DISABLE, addr, &wr_data, 1);

	/* set control sel to AON*/
	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	/* Adjust AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		otp_aon_ldo_bak = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)));
		otp_aon_ldo_adj = otp_aon_ldo_bak & 0xFFFFFF0F;
		if (0xFF != otp_adj_vol_lvl) {
			otp_aon_ldo_adj = otp_aon_ldo_adj | ((otp_adj_vol_lvl & 0x0F) << AON_SHIFT_LDOH09_V09ADJ_L_3_0);
			HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_adj);
		}

		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	DBG_OTP_INFO("set pg margin read\n");
	hal_rtl_otp_test_mod_cfg(OTPPgMarRdMod);


	otp_tmp_val = hal_rtl_otp_rd_sub_syss(OTP_DISABLE, addr, &ret_val, 1);
	if (OTP_RET_FAIL_FLG == otp_tmp_val) {
		DBG_OTP_ERR("PG margin read error\n");
		ret_val = 0xFF;
	}

	DBG_OTP_INFO("pg margin read data: %x\n", ret_val);
	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);

	/* restore AON LDO voltage */
	if (OTP_DISABLE != adj_vol) {
		HAL_WRITE32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_AON_LDO_CTRL0)), otp_aon_ldo_bak);
		hal_delay_us(_potp_adpt->pc_en_dly);
	}

	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	/* Disable controller */
	hal_rtl_otpc_ctrl(OTP_DISABLE, OTP_DISABLE, OTP_DISABLE);

	/* Disable OTP memory power */
	DBG_OTP_INFO("disable power\n");
	hal_rtl_otp_pwr_ctrl(OTP_DISABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Disable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_DISABLE);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rp_mar_rd_aon
*         hal_rtl_otp_rp_mar_rd_aon is used to perform a repair margin read operation
*
*  @returns uint8_t: status of repair margin read
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_rp_mar_rd_syss(uint8_t auto_en)
{
	uint8_t as_ctrl_sel_bak;
	uint8_t ret_val = 0;
	uint32_t tmp_addr;
	uint32_t reg_tmp;

	DBG_OTP_INFO("%s>\n", __func__);
#if 0
	/* Enable PMC control */
	hal_rtl_otp_pmc_gnt_ctrl(OTP_ENABLE);

	/* In test chip, enable controller first then enable pwoer cut,
	   In MP chip, this flow should be reversed. */
	DBG_OTP_INFO("enable power\n");
	/* Enable OTP memory power */
	hal_rtl_otp_pwr_ctrl(OTP_ENABLE);
	hal_delay_us(_potp_adpt->pc_en_dly);

	/* Enable controller */
	hal_rtl_otpc_ctrl(OTP_ENABLE, OTP_ENABLE, OTP_ENABLE);
#endif
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	DBG_OTP_INFO("set auto repair margin read, auto_en: %x\n", auto_en);
	if (OTP_ENABLE == auto_en) {
		hal_rtl_otp_test_mod_cfg(OTPAutoRpMarRdMOd);
	} else {
		hal_rtl_otp_test_mod_cfg(OTPRpMarRdMOd);
	}

#if 0
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	/* wait for operation done */
	while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
		ret_val++;
	}
	DBG_OTP_INFO("ready\r\n");
#endif

	DBG_OTP_INFO("to do normal read\r\n");
	for (tmp_addr = 0; tmp_addr < OTP_MEM_SIZE; tmp_addr++) {

		/* set mode/addr first */
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_MODE_SEL);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_ADDR);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_MASK_OTP_DATA);
		HAL_SET_BIT(reg_tmp, (OTPRdMod << SYSON_S_SHIFT_OTP_MODE_SEL));
		HAL_SET_BIT(reg_tmp, ((tmp_addr & OTP_ADDR_MSK) << SYSON_S_SHIFT_OTP_ADDR));
		/* clear ready bit first */
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		/* enable read operation */
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		DBG_OTP_INFO("to write AS_CTRL_NS: %08x\n", reg_tmp);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		ret_val = 0;
		reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
		/* wait for operation done */
		while ((HAL_READ_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
			hal_delay_us(OTP_RD_CHK_INTV);
			reg_tmp = HAL_READ32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)));
			DBG_OTP_INFO("reg_tmp: %08x\r\n", reg_tmp);
			ret_val++;
		}

		HAL_SET_BIT(reg_tmp, SYSON_S_BIT_OTP_RDY);
		HAL_CLEAR_BIT(reg_tmp, SYSON_S_BIT_OTP_OP_EN);
		HAL_WRITE32(SYSON_S_BASE, (uint32_t)(&(SYSON_S_BASE_REG->SYSON_S_REG_OTP_AS_CTRL_S)), reg_tmp);

		if (ret_val == OTP_RW_TO_CNT) {
			DBG_OTP_ERR("TO.(%d)\n", tmp_addr);
			ret_val = OTPStsFailed;
			break;
		} else {
			DBG_OTP_INFO("ready\r\n");
			ret_val = OTPStsSuccess;
		}

		if (OTP_ENABLE == auto_en) {
			break;
		}
	}

	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rp_chk_aon
*         hal_rtl_otp_rp_chk_aon is used to perform repair check by AON register
*
*  @returns uint8_t: read result of repair check
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_rp_chk_syss(void)
{
	uint8_t as_ctrl_sel_bak;
	uint8_t ret_val = 0;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	DBG_OTP_INFO("set repair check\n");
	hal_rtl_otp_test_mod_cfg(OTPRpChkMod);

	ret_val = 0;
	hal_rtl_otp_rd_sub_syss(OTP_DISABLE, 0, &ret_val, 1);

	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_rp_pg_aon
*         hal_rtl_otp_rp_pg_aon is used to perform repair program
*
*  @returns uint8_t: read result of repair program
*/
SECTION_OTP_TEXT
uint8_t hal_rtl_otp_rp_pg_syss(void)
{
	uint8_t as_ctrl_sel_bak;
	uint8_t ret_val = 0;
	uint32_t tmp_addr;
	uint32_t reg_tmp;

	DBG_OTP_INFO("%s>\n", __func__);
	as_ctrl_sel_bak = hal_rtl_otp_ctrl_sel_sts();

	hal_rtl_otp_ctrl_sel(OTPCtrlSys);

	DBG_OTP_INFO("set repair program\n");
	hal_rtl_otp_test_mod_cfg(OTPRpPgMod);

	ret_val = 0;
	hal_rtl_otp_wr_sub_syss(OTP_DISABLE, 0, &ret_val, 1);

	DBG_OTP_INFO("set user read\n");
	hal_rtl_otp_test_mod_cfg(OTPTestUserMOd);
	hal_rtl_otp_ctrl_sel(as_ctrl_sel_bak);

	DBG_OTP_INFO("%s<\n", __func__);

	return ret_val;
}

/**
*  @brief Description of hal_rtl_otp_set_aon_vol
*         hal_rtl_otp_set_aon_vol is used to set AON LDO voltage
*
*  @param[in]  uint8_t aon_vol: target AON LDO voltage
*/
SECTION_OTP_TEXT
void hal_rtl_otp_set_aon_vol(uint8_t aon_vol)
{
	otp_adj_vol_lvl = aon_vol;
	DBG_OTP_INFO("AON vol: %x\n", otp_adj_vol_lvl);
}


#if 0
SECTION_OTP_TEXT
uint32_t hal_rtl_otp_chk_rdy_sts(void)
{
	uint32_t reg_tmp = 0;
	uint32_t ret_val = 0;

	ret_val = 0;
	reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
	/* wait for OTP ready status */
	while ((HAL_READ_BIT(reg_tmp, AON_BIT_OTP_RDY) == 0) && (ret_val < OTP_RW_TO_CNT)) {
		hal_delay_us(OTP_RD_CHK_INTV);
		reg_tmp = HAL_READ32(AON_BASE, (uint32_t)(&(AON_BASE_REG->AON_REG_OTP_AS_CTRL_A)));
		ret_val++;
	}

	if (OTP_RW_TO_CNT == ret_val) {
		ret_val = OTPStsFailed;
	} else {
		ret_val = OTPStsSuccess;
	}

	return ret_val;
}
#endif
#endif // end of "#if CONFIG_UART_EN"

