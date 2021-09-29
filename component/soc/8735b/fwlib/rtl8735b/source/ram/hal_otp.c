/**************************************************************************//**
 * @file     hal_otp.c
 * @brief    This file implements the OTP IP power, clock, access related functions.
 * @version  V1.00
 * @date     2020-08-19
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
#include "hal_otp.h"

#if CONFIG_OTP_EN

extern const hal_otp_func_stubs_t hal_otp_stubs;
/**
* @addtogroup hal_otp OTP
* @{
*/



/// @cond DOXYGEN_ROM_HAL_API

/**
* @addtogroup hs_hal_uart_rom_func UART HAL ROM APIs.
* @ingroup hs_hal_uart
* @{
* @brief UART HAL ROM API. The user application(in RAM space) should not call these APIs directly.
*        There is another set of UART HAL APIs in the RAM space is provided for the user application.
*/



/**
*  @brief Description of hal_otp_init
*         hal_otp_init is used to do OTP parameter initialization
*
*  @param[in]  void
*
*  @returns void
*/
void hal_otp_init(void)
{
	hal_otp_stubs.hal_otp_init();
}

/**
*  @brief Description of hal_otp_deinit
*         hal_otp_deinit is used to do OTP parameter de-initialization
*
*  @param[in]  void
*
*  @returns void
*/
void hal_otp_deinit(void)
{
	hal_otp_stubs.hal_otp_deinit();
}

/**
*  @brief Description of hal_otp_ctrl_sel
*         hal_otp_ctrl_sel is used to select OTP access control source
*
*  @param[in]  uint8_t otp_ctrl_sel: select AON/SYSON control source
*
*  @returns void
*/
void hal_otp_ctrl_sel(uint8_t otp_ctrl_sel)
{
	hal_otp_stubs.hal_otp_ctrl_sel(otp_ctrl_sel);
}

/**
*  @brief Description of hal_otp_ctrl_sel_sts
*         hal_otp_ctrl_sel_sts is used to show current selection of control source
*
*  @param[in]  void
*
*  @returns uint8_t curent selection of control sourcee
*/
uint8_t hal_otp_ctrl_sel_sts(void)
{
	return hal_otp_stubs.hal_otp_ctrl_sel_sts();
}

/**
*  @brief Description of hal_otp_pmc_gnt_ctrl
*         hal_otp_pmc_gnt_ctrl is used to control power control grant enable/disable
*
*  @param[in]  uint8_t gnt_en: power control grant control

*
*  @returns void
*/
void hal_otp_pmc_gnt_ctrl(uint8_t gnt_en)
{
	hal_otp_stubs.hal_otp_pmc_gnt_ctrl(gnt_en);
}

/**
*  @brief Description of hal_otp_wr_gnt_ctrl
*         hal_otp_wr_gnt_ctrl is used to control write grant enable/disable
*
*  @param[in]  uint8_t gnt_en: write grant control

*
*  @returns void
*/
void hal_otp_wr_gnt_ctrl(uint8_t gnt_en)
{
	hal_otp_stubs.hal_otp_wr_gnt_ctrl(gnt_en);
}

/**
*  @brief Description of hal_otpc_ctrl
*         hal_otpc_ctrl is used to configure OTP controller and its read/write
*         clock.
*
*  @param[in]  uint8_t otpc_en: OTP controller enable
*  @param[in]  uint8_t wclk_en: write clock enable
*  @param[in]  uint8_t rclk_en: read clock enable
*
*  @returns void
*/
void hal_otpc_ctrl(uint8_t otpc_en, uint8_t wclk_en, uint8_t rclk_en)
{
	hal_otp_stubs.hal_otpc_ctrl(otpc_en, wclk_en, rclk_en);
}

/**
*  @brief Description of hal_otp_pwr_ctrl
*         hal_otp_pwr_ctrl is used to enable/disable OTP power control
*
*  @param[in]  uint8_t pwr_en: OTP power control
*
*  @returns uint32_t: power control result
*/
uint32_t hal_otp_pwr_ctrl(uint8_t pwr_en)
{
	return hal_otp_stubs.hal_otp_pwr_ctrl(pwr_en);
}

/**
*  @brief Description of hal_otp_test_mod_cfg
*         hal_otp_test_mod_cfg is used to configure OTP test mode selection register
*
*  @param[in]  uint8_t test_mod: OTP test mode
*
*  @returns void
*/
void hal_otp_test_mod_cfg(uint8_t test_mod)
{
	hal_otp_stubs.hal_otp_test_mod_cfg(test_mod);
}

/**
*  @brief Description of hal_otp_test_row_ctrl
*         hal_otp_test_row_ctrl is used to enable/disable test row access
*
*  @param[in]  uint8_t test_row_en: test row enable control
*
*  @returns void
*/
void hal_otp_test_row_ctrl(uint8_t test_row_en)
{
	hal_otp_stubs.hal_otp_test_row_ctrl(test_row_en);
}

/**
*  @brief Description of hal_otp_chk_err_sts
*         hal_otp_chk_err_sts is used to check error status
*
*  @param[in]  void
*
*  @returns uint32_t: OTP error flag
*/
uint32_t hal_otp_chk_err_sts(void)
{
	return hal_otp_stubs.hal_otp_chk_err_sts();
}

/**
*  @brief Description of hal_otp_dsb_ctrl
*         hal_otp_dsb_ctrl is used to control DSB enable
*
*  @param[in]  uint8_t dsb_en: OTP DSB control
*
*  @returns void
*/
void hal_otp_dsb_ctrl(uint8_t dsb_en)
{
	hal_otp_stubs.hal_otp_dsb_ctrl(dsb_en);
}

/**
*  @brief Description of hal_otp_rb_ctrl
*         hal_otp_rb_ctrl is used to control reboot(PTRIM) enable/disable
*
*  @param[in]  uint8_t rb_en: reboot operaton enable/disable after repair program
*
*  @returns void
*/
void hal_otp_rb_ctrl(uint8_t rb_en)
{
	hal_otp_stubs.hal_otp_rb_ctrl(rb_en);
}

/**
*  @brief Description of hal_otp_cz_ctrl
*         hal_otp_cz_ctrl is used to configure counting zero control
*
*  @param[in]  uint8_t cz_en: OTP counting zero enable/disable
*
*  @returns void
*/
void hal_otp_cz_ctrl(uint8_t cz_en)
{
	hal_otp_stubs.hal_otp_cz_ctrl(cz_en);
}

/**
*  @brief Description of hal_otp_chk_cz_sts
*         hal_otp_chk_cz_sts is used to grep CZ status according to a given key selection
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t: CZ status
*/
uint32_t hal_otp_chk_cz_sts(uint32_t key_sel)
{
	return hal_otp_stubs.hal_otp_chk_cz_sts(key_sel);
}

/**
*  @brief Description of hal_otp_clr_cz_sts
*         hal_otp_clr_cz_sts is used to clear CZ status register and return value after clearing
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t:!@CZ status after clearing
*/
uint32_t hal_otp_clr_cz_sts(uint32_t key_sel)
{
	return hal_otp_stubs.hal_otp_clr_cz_sts(key_sel);
}

/**
*  @brief Description of hal_otp_prct_ctrl
*         hal_otp_prct_ctrl is used to configure OTP protect control and return the status
*
*  @param[in]  uint8_t prct_en: protectoin control enable/disable
*
*  @returns uint32_t: protection control status
*/
uint32_t hal_otp_prct_ctrl(uint8_t prct_en)
{
	return hal_otp_stubs.hal_otp_prct_ctrl(prct_en);
}

/**
*  @brief Description of hal_otp_chk_prct_sts
*         hal_otp_chk_prct_sts is used to check protect status according to a given selection.
*
*  @param[in]  uint32_t pwr_en: protection selection
*
*  @returns uint32_t: protection status
*/
uint32_t hal_otp_chk_prct_sts(uint32_t prct_sel)
{
	return hal_otp_stubs.hal_otp_chk_prct_sts(prct_sel);
}

/**
*  @brief Description of hal_otp_rd_lz_er_addr
*         hal_otp_rd_lz_er_addr is used to read logical zone CRC error address
*         which is start address of a entry
*
*  @param[in]  void
*
*  @returns uint32_t: logical zone CRC error address
*/
uint32_t hal_otp_rd_lz_er_addr(void)
{
	return hal_otp_stubs.hal_otp_rd_lz_er_addr();
}

/**
*  @brief Description of hal_otp_clr_prct_sts
*         hal_otp_clr_prct_sts is used to clear protect status according to a given selection
*         and return result after clear
*
*  @param[in]  uint32_t pwr_en: protection selection, 0xFFFFFFFF is to clear all
*
*  @returns uint32_t protection status
*/
uint32_t hal_otp_clr_prct_sts(uint32_t prct_sel)
{
	return hal_otp_stubs.hal_otp_clr_prct_sts(prct_sel);
}

/**
*  @brief Description of hal_otp_rd_sz_prct_sts
*         hal_otp_rd_sz_prct_sts is used to read secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t: protection status
*/
uint32_t hal_otp_rd_sz_prct_sts(void)
{
	return hal_otp_stubs.hal_otp_rd_sz_prct_sts();
}

/**
*  @brief Description of hal_otp_clr_sz_prct_sts
*         hal_otp_clr_sz_prct_sts is used to clear secure zone protection status
*
*  @param[in]  uint8_t sz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t:   secure zone protecion status after clearing
*/
uint32_t hal_otp_clr_sz_prct_sts(uint32_t sz_prct_sts_sel)
{
	return hal_otp_stubs.hal_otp_clr_sz_prct_sts(sz_prct_sts_sel);
}

/**
*  @brief Description of hal_otp_rd_ssz_prct_sts
*         hal_otp_rd_ssz_prct_sts is used to read super secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t:   super secure zone status
*/
uint32_t hal_otp_rd_ssz_prct_sts(void)
{
	return hal_otp_stubs.hal_otp_rd_ssz_prct_sts();
}

/**
*  @brief Description of hal_otp_clr_ssz_prct_sts
*         hal_otp_clr_ssz_prct_sts is used to clear super secure zone status
*
*  @param[in]  uint32_t ssz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t: super secure zone protection status after clearing
*/
uint32_t hal_otp_clr_ssz_prct_sts(uint32_t ssz_prct_sts_sel)
{
	return hal_otp_stubs.hal_otp_clr_ssz_prct_sts(ssz_prct_sts_sel);
}

/**
*  @brief Description of hal_otp_szwl_ctrl
*         hal_otp_szwl_ctrl is used to configure secure zone write lock function
*
*  @param[in]  uint32_t sz_wl_en: enable/disable secure zone write lock function
*                                 1: enable, 0: disable
*
*  @returns uint32_t: protection control regsiter
*/
uint32_t hal_otp_szwl_ctrl(uint32_t sz_wl_en)
{
	return hal_otp_stubs.hal_otp_szwl_ctrl(sz_wl_en);
}


/**
*  @brief Description of hal_otp_rd_aldn_sts
*         hal_otp_rd_aldn_sts is used to read autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status bit
*/
uint32_t hal_otp_rd_aldn_sts(uint32_t aldn_sts_sel)
{
	return hal_otp_stubs.hal_otp_rd_aldn_sts(aldn_sts_sel);
}

/**
*  @brief Description of hal_otp_clr_aldn_sts
*         hal_otp_clr_aldn_sts is used to clear autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status register
*/
uint32_t hal_otp_clr_aldn_sts(uint32_t aldn_sts_sel)
{
	return hal_otp_stubs.hal_otp_clr_aldn_sts(aldn_sts_sel);
}

/**
*  @brief Description of hal_otp_rd_al_sts
*         hal_otp_rd_al_sts is used to read autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status bit
*/
uint32_t hal_otp_rd_al_sts(uint32_t al_sts_sel)
{
	return hal_otp_stubs.hal_otp_rd_al_sts(al_sts_sel);
}

/**
*  @brief Description of hal_otp_clr_al_sts
*         hal_otp_clr_al_sts is used to clear autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status register
*/
uint32_t hal_otp_clr_al_sts(uint32_t al_sts_sel)
{
	return hal_otp_stubs.hal_otp_clr_al_sts(al_sts_sel);
}

/**
*  @brief Description of hal_otp_al_cfg
*         hal_otp_al_cfg is used to configure autoload enable according to a given selection
*
*  @param[in]  uint32_t al_trg_sel: autoload target selection
*
*  @returns uint32_t: autoload status regsiter
*/
uint32_t hal_otp_al_cfg(uint32_t al_trg_sel)
{
	return hal_otp_stubs.hal_otp_al_cfg(al_trg_sel);
}

/**
*  @brief Description of hal_otp_al_ctrl_aon
*         hal_otp_al_ctrl_aon is used to control autoload enable/disable flow by AON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
uint32_t hal_otp_al_ctrl_aon(uint32_t al_en)
{
	return hal_otp_stubs.hal_otp_al_ctrl_aon(al_en);
}

/**
*  @brief Description of hal_otp_bust_ctrl
*         hal_otp_bust_ctrl is used to control burst operatoin enable/disable
*
*  @param[in]  uint8_t bust_en: burst enable
*
*  @returns uint32_t: operation succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
uint32_t hal_otp_bust_ctrl(uint8_t bust_en)
{
	return hal_otp_stubs.hal_otp_bust_ctrl(bust_en);
}

/**
*  @brief Description of hal_otp_rd_sub_aon
*         hal_otp_rd_sub_aon is used for read sub looping function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
uint32_t hal_otp_rd_sub_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_sub_aon(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_rd_aon
*         hal_otp_rd_aon is used for read function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
uint32_t hal_otp_rd_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_aon(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_byte_rd_aon
*         hal_otp_byte_rd_aon is used to read a single byte according to given address
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
uint8_t hal_otp_byte_rd_aon(uint32_t addr)
{
	return hal_otp_stubs.hal_otp_byte_rd_aon(addr);
}

/**
*  @brief Description of hal_otp_wr_sub_aon
*         hal_otp_wr_sub_aon is used for write sub looping function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
uint32_t hal_otp_wr_sub_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_sub_aon(pwr_cl_en, addr, wr_data, len);
}

/**
*  @brief Description of hal_otp_wr_aon
*         hal_otp_wr_aon is used for write function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
uint32_t hal_otp_wr_aon(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_aon(pwr_cl_en, addr, wr_data, len);
}

/**
*  @brief Description of hal_otp_byte_wr_aon
*         hal_otp_byte_wr_aon is used to do a one-byte write operation
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*
*  @returns void
*/
void hal_otp_byte_wr_aon(uint32_t addr, uint8_t wr_data)
{
	hal_otp_stubs.hal_otp_byte_wr_aon(addr, wr_data);
}

/**
*  @brief Description of hal_otp_cmp_aon
*         hal_otp_cmp_aon is used to do a comparison between given OTP memory address and data
*
*  @param[in]  uint32_t addr: OTP memory address which contains data to be compare with given data
*  @param[in]  uint32_t addr: given data to be compared with data of target OTP address
*
*  @returns uint32_t: comparison result, succeeded(0) failed(1)
*/
uint32_t hal_otp_cmp_aon(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_stubs.hal_otp_cmp_aon(addr, cmp_data);
}


/**
*  @brief Description of hal_otp_rd_sub_sys
*         hal_otp_rd_sub_sys is used for read sub looping function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
uint32_t hal_otp_rd_sub_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_sub_sys(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_rd_sys
*         hal_otp_rd_sys is used for read function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
uint32_t hal_otp_rd_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_sys(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_byte_rd_sys
*         hal_otp_byte_rd_sys is used to read a single byte according to given address
*         by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
uint8_t hal_otp_byte_rd_sys(uint32_t addr)
{
	return hal_otp_stubs.hal_otp_byte_rd_sys(addr);
}


/**
*  @brief Description of hal_otp_wr_sub_sys
*         hal_otp_wr_sub_sys is used for write sub looping function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
uint32_t hal_otp_wr_sub_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_sub_sys(pwr_cl_en, addr, wr_data, len);
}

/**
*  @brief Description of hal_otp_wr_sys
*         hal_otp_wr_sys is used for write function by SYSON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
uint32_t hal_otp_wr_sys(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_sys(pwr_cl_en, addr, wr_data, len);
}

/**
*  @brief Description of hal_otp_byte_wr_sys
*         hal_otp_byte_wr_sys is used to do a one-byte write operation by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*
*  @returns void
*/
void hal_otp_byte_wr_sys(uint32_t addr, uint8_t wr_data)
{
	hal_otp_stubs.hal_otp_byte_wr_sys(addr, wr_data);
}

/**
*  @brief Description of hal_otp_cmp_sys
*         hal_otp_cmp_sys is used to do a comparison between given OTP memory address and data
*         by SYSON
*
*  @param[in]  uint32_t addr: OTP memory address which contains data to be compare with given data
*  @param[in]  uint32_t addr: given data to be compared with data of target OTP address
*
*  @returns uint32_t: comparison result, succeeded(0) failed(1)
*/
uint32_t hal_otp_cmp_sys(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_stubs.hal_otp_cmp_sys(addr, cmp_data);
}

/**
*  @brief Description of hal_otp_al_ctrl_sys
*         hal_otp_al_ctrl_sys is used to control autoload enable/disable flow by SYSON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)

*/
uint32_t hal_otp_al_ctrl_sys(uint32_t al_en)
{
	return hal_otp_stubs.hal_otp_al_ctrl_sys(al_en);
}


/**
*  @brief Description of hal_otp_rd_sub_syss
*         hal_otp_rd_sub_syss is used for read sub looping function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)

*/
uint32_t hal_otp_rd_sub_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_sub_syss(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_rd_syss
*         hal_otp_rd_syss is used for read function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)

*/
uint32_t hal_otp_rd_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_rd_syss(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_otp_byte_rd_syss
*         hal_otp_byte_rd_syss is used to read a single byte according to given address
*         by SYSON-S
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
uint8_t hal_otp_byte_rd_syss(uint32_t addr)
{
	return hal_otp_stubs.hal_otp_byte_rd_syss(addr);
}

/**
*  @brief Description of hal_otp_wr_sub_syss
*         hal_otp_wr_sub_syss is used for write sub looping function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status

*/
uint32_t hal_otp_wr_sub_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_sub_syss(pwr_cl_en, addr, wr_data, len);
}

/**
*  @brief Description of hal_otp_wr_syss
*         hal_otp_wr_syss is used for write function by SYSON-S
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *wr_data:  write data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: write operation status
*/
uint32_t hal_otp_wr_syss(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_stubs.hal_otp_wr_syss(pwr_cl_en, addr, wr_data, len);
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
void hal_otp_byte_wr_syss(uint32_t addr, uint8_t wr_data)
{
	hal_otp_stubs.hal_otp_byte_wr_syss(addr, wr_data);
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
uint32_t hal_otp_cmp_syss(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_stubs.hal_otp_cmp_syss(addr, cmp_data);
}

/**
*  @brief Description of hal_otp_al_ctrl_syss
*         hal_otp_al_ctrl_syss is used to control autoload enable/disable flow by SYSON-S
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
uint32_t hal_otp_al_ctrl_syss(uint32_t al_en)
{
	return hal_otp_stubs.hal_otp_al_ctrl_syss(al_en);
}

/**
*  @brief Description of hal_otp_byte_wr_marr_aon
*         hal_otp_byte_wr_marr_aon is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
uint8_t hal_otp_byte_wr_marr_aon(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	return hal_otp_stubs.hal_otp_byte_wr_marr_aon(addr, wr_data, adj_vol);
}

/**
*  @brief Description of hal_otp_byte_wr_marr_sys
*         hal_otp_byte_wr_marr_sys is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
uint8_t hal_otp_byte_wr_marr_sys(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	return hal_otp_stubs.hal_otp_byte_wr_marr_sys(addr, wr_data, adj_vol);
}

/**
*  @brief Description of hal_otp_byte_wr_marr_syss
*         hal_otp_byte_wr_marr_syss is used to perform a byte write and then execute a program margin read for check
*
*  @param[in]  uint32_t addr: OTP memory address to be written
*  @param[in]  uint8_t wr_data: data to be written
*  @param[in]  uint8_t adj_vol: enable control to decide whether to adjust AON LDO voltage or not
*
*  @returns uint8_t: read result of program margin read
*/
uint8_t hal_otp_byte_wr_marr_syss(uint32_t addr, uint8_t wr_data, uint8_t adj_vol)
{
	return hal_otp_stubs.hal_otp_byte_wr_marr_syss(addr, wr_data, adj_vol);
}

uint8_t hal_otp_rp_mar_rd_syss(uint8_t auto_en)
{
	return hal_otp_stubs.hal_otp_rp_mar_rd_syss(auto_en);
}

uint8_t hal_otp_rp_chk_syss(void)
{
	return hal_otp_stubs.hal_otp_rp_chk_syss();
}

uint8_t hal_otp_rp_pg_syss(void)
{
	return hal_otp_stubs.hal_otp_rp_pg_syss();
}

/**
*  @brief Description of hal_otp_set_aon_vol
*         hal_otp_set_aon_vol is used to set AON LDO voltage
*
*  @param[in]  uint8_t aon_vol: AON LDO voltage
*  @returns NA
*/
void hal_otp_set_aon_vol(uint8_t aon_vol)
{

	hal_otp_stubs.hal_otp_set_aon_vol(aon_vol);
}

#endif // end of "#if CONFIG_OTP_EN"


