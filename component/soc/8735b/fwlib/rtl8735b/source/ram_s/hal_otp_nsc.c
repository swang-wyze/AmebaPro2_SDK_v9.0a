/**************************************************************************//**
 * @file     hal_otp_nsc.c
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
#include "hal_otp_nsc.h"

#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#undef SECTION_NS_ENTRY_FUNC
#undef NS_ENTRY
#define SECTION_NS_ENTRY_FUNC
#define NS_ENTRY
#endif

//#if defined(CONFIG_BUILD_SECURE)
#if  CONFIG_OTP_EN
/** \brief Description of Non-secure callable function hal_otp_init_nsc
 *
 *    hal_
 *
 *   \return void.
 */
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_init_nsc(void)
{
	hal_otp_init();
}


SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_deinit_nsc(void)
{
	hal_otp_deinit();
}

/**
*  @brief Description of hal_rtl_otp_ctrl_sel
*         hal_rtl_otp_ctrl_sel is used to select OTP access control source
*
*  @param[in]  uint8_t otp_ctrl_sel: select AON/SYSON control source
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_ctrl_sel_nsc(uint8_t otp_ctrl_sel)
{
	hal_otp_ctrl_sel(otp_ctrl_sel);
}

/**
*  @brief Description of hal_rtl_otp_ctrl_sel_sts
*         hal_rtl_otp_ctrl_sel_sts is used to show current selection of control source
*
*  @param[in]  void
*
*  @returns uint8_t curent selection of control sourcee
*/
SECTION_NS_ENTRY_FUNC
uint8_t NS_ENTRY hal_otp_ctrl_sel_sts_nsc(void)
{
	return hal_otp_ctrl_sel_sts();
}

/**
*  @brief Description of hal_rtl_otp_pmc_gnt_ctrl
*         hal_rtl_otp_pmc_gnt_ctrl is used to control power control grant enable/disable
*
*  @param[in]  uint8_t gnt_en: power control grant control
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_pmc_gnt_ctrl_nsc(uint8_t gnt_en)
{
	hal_otp_pmc_gnt_ctrl(gnt_en);
}

/**
*  @brief Description of hal_rtl_otp_wr_gnt_ctrl
*         hal_rtl_otp_wr_gnt_ctrl is used to control write grant enable/disable
*
*  @param[in]  uint8_t gnt_en: write grant control
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_wr_gnt_ctrl_nsc(uint8_t gnt_en)
{
	hal_otp_wr_gnt_ctrl(gnt_en);
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
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otpc_ctrl_nsc(uint8_t otpc_en, uint8_t wclk_en, uint8_t rclk_en)
{
	hal_otpc_ctrl(otpc_en, wclk_en, rclk_en);
}

/**
*  @brief Description of hal_rtl_otp_pwr_ctrl
*         hal_rtl_otp_pwr_ctrl is used to enable/disable OTP power control
*
*  @param[in]  uint8_t pwr_en: OTP power control
*
*  @returns uint32_t: power control result
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_pwr_ctrl_nsc(uint8_t pwr_en)
{
	return hal_otp_pwr_ctrl(pwr_en);
}

/**
*  @brief Description of hal_rtl_otp_test_mod_cfg
*         hal_rtl_otp_test_mod_cfg is used to configure OTP test mode selection register
*
*  @param[in]  uint8_t test_mod: OTP test mode
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_test_mod_cfg_nsc(uint8_t test_mod)
{
	hal_otp_test_mod_cfg(test_mod);
}

/**
*  @brief Description of hal_rtl_otp_test_row_ctrl
*         hal_rtl_otp_test_row_ctrl is used to enable/disable test row access
*
*  @param[in]  uint8_t test_row_en: test row enable control
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_test_row_ctrl_nsc(uint8_t test_row_en)
{
	hal_otp_test_row_ctrl(test_row_en);
}

/**
*  @brief Description of hal_rtl_otp_chk_err_sts
*         hal_rtl_otp_chk_err_sts is used to check error status
*
*  @param[in]  void
*
*  @returns uint32_t: OTP error flag
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_chk_err_sts_nsc(void)
{
	return hal_otp_chk_err_sts();
}

/**
*  @brief Description of hal_rtl_otp_dsb_ctrl
*         hal_rtl_otp_dsb_ctrl is used to control DSB enable
*
*  @param[in]  uint8_t dsb_en: OTP DSB control
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_dsb_ctrl_nsc(uint8_t dsb_en)
{
	hal_otp_dsb_ctrl(dsb_en);
}

/**
*  @brief Description of hal_rtl_otp_rb_ctrl
*         hal_rtl_otp_rb_ctrl is used to control reboot(PTRIM) enable/disable
*
*  @param[in]  uint8_t rb_en: reboot operaton enable/disable after repair program
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_rb_ctrl_nsc(uint8_t rb_en)
{
	hal_otp_rb_ctrl(rb_en);
}

/**
*  @brief Description of hal_rtl_otp_cz_ctrl
*         hal_rtl_otp_cz_ctrl is used to configure counting zero control
*
*  @param[in]  uint8_t cz_en: OTP counting zero enable/disable
*
*  @returns void
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_cz_ctrl_nsc(uint8_t cz_en)
{
	hal_otp_cz_ctrl(cz_en);
}

/**
*  @brief Description of hal_rtl_otp_chk_cz_sts
*         hal_rtl_otp_chk_cz_sts is used to grep CZ status according to a given key selection
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t: CZ status
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_chk_cz_sts_nsc(uint32_t key_sel)
{
	return hal_otp_chk_cz_sts(key_sel);
}

/**
*  @brief Description of hal_rtl_otp_clr_cz_sts
*         hal_rtl_otp_clr_cz_sts is used to clear CZ status register and return value after clearing
*
*  @param[in]  uint32_t key_sel: if key_sel is 0xFFFFFFFF, return the whole register
*
*  @returns uint32_t:¡@CZ status after clearing
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_cz_sts_nsc(uint32_t key_sel)
{
	return hal_otp_clr_cz_sts(key_sel);
}

/**
*  @brief Description of hal_rtl_otp_prct_ctrl
*         hal_rtl_otp_prct_ctrl is used to configure OTP protect control and return the status
*
*  @param[in]  uint8_t prct_en: protectoin control enable/disable
*
*  @returns uint32_t: protection control status
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_prct_ctrl_nsc(uint8_t prct_en)
{
	return hal_otp_prct_ctrl(prct_en);
}

/**
*  @brief Description of hal_rtl_otp_chk_prct_sts
*         hal_rtl_otp_chk_prct_sts is used to check protect status according to a given selection.
*
*  @param[in]  uint32_t pwr_en: protection selection
*
*  @returns uint32_t: protection status
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_chk_prct_sts_nsc(uint32_t prct_sel)
{
	return hal_otp_chk_prct_sts(prct_sel);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_lz_er_addr_nsc(void)
{
	return hal_otp_rd_lz_er_addr();
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_prct_sts_nsc(uint32_t prct_sel)
{
	return hal_otp_clr_prct_sts(prct_sel);
}

/**
*  @brief Description of hal_rtl_otp_rd_sz_prct_sts
*         hal_rtl_otp_rd_sz_prct_sts is used to read secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t: protection status
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_sz_prct_sts_nsc(void)
{
	return hal_otp_rd_sz_prct_sts();
}

/**
*  @brief Description of hal_rtl_otp_clr_sz_prct_sts
*         hal_rtl_otp_clr_sz_prct_sts is used to clear secure zone protection status
*
*  @param[in]  uint8_t sz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t:   secure zone protecion status after clearing
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_sz_prct_sts_nsc(uint32_t sz_prct_sts_sel)
{
	return hal_otp_clr_sz_prct_sts(sz_prct_sts_sel);
}

/**
*  @brief Description of hal_rtl_otp_rd_ssz_prct_sts
*         hal_rtl_otp_rd_ssz_prct_sts is used to read super secure zone key protection status (CRC)
*
*  @param[in]  void
*
*  @returns uint32_t:   super secure zone status
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_ssz_prct_sts_nsc(void)
{
	return hal_otp_rd_ssz_prct_sts();
}

/**
*  @brief Description of hal_rtl_otp_clr_ssz_prct_sts
*         hal_rtl_otp_clr_ssz_prct_sts is used to clear super secure zone status
*
*  @param[in]  uint32_t ssz_prct_sts_sel: secure zone protection status selection
*
*  @returns uint32_t: super secure zone protection status after clearing
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_ssz_prct_sts_nsc(uint32_t ssz_prct_sts_sel)
{
	return hal_otp_clr_ssz_prct_sts(ssz_prct_sts_sel);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_szwl_ctrl_nsc(uint32_t sz_wl_en)
{
	return hal_otp_szwl_ctrl(sz_wl_en);
}


/**
*  @brief Description of hal_rtl_otp_rd_aldn_sts
*         hal_rtl_otp_rd_aldn_sts is used to read autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status bit
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_aldn_sts_nsc(uint32_t aldn_sts_sel)
{
	return hal_otp_rd_aldn_sts(aldn_sts_sel);
}

/**
*  @brief Description of hal_rtl_otp_clr_aldn_sts
*         hal_rtl_otp_clr_aldn_sts is used to clear autoload done status according to a given selection
*
*  @param[in]  uint32_t aldn_sts_sel: autoload done status selection
*
*  @returns uint32_t: autoload done status register
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_aldn_sts_nsc(uint32_t aldn_sts_sel)
{
	return hal_otp_clr_aldn_sts(aldn_sts_sel);
}

/**
*  @brief Description of hal_rtl_otp_rd_al_sts
*         hal_rtl_otp_rd_al_sts is used to read autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status bit
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_al_sts_nsc(uint32_t al_sts_sel)
{
	return hal_otp_rd_al_sts(al_sts_sel);
}

/**
*  @brief Description of hal_rtl_otp_clr_al_sts
*         hal_rtl_otp_clr_al_sts is used to clear autoload status(pass/failed) according to a given selection
*
*  @param[in]  uint32_t al_sts_sel: autoload status selection
*
*  @returns uint32_t: autoload status register
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_clr_al_sts_nsc(uint32_t al_sts_sel)
{
	return hal_otp_clr_al_sts(al_sts_sel);
}

/**
*  @brief Description of hal_rtl_otp_al_cfg
*         hal_rtl_otp_al_cfg is used to configure autoload enable according to a given selection
*
*  @param[in]  uint32_t al_trg_sel: autoload target selection
*
*  @returns uint32_t: autoload status regsiter
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_al_cfg_nsc(uint32_t al_trg_sel)
{
	return hal_otp_al_cfg(al_trg_sel);
}

/**
*  @brief Description of hal_rtl_otp_al_ctrl_aon
*         hal_rtl_otp_al_ctrl_aon is used to control autoload enable/disable flow by AON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_al_ctrl_aon_nsc(uint32_t al_en)
{
	return hal_otp_al_ctrl_aon(al_en);
}

/**
*  @brief Description of hal_rtl_otp_bust_ctrl
*         hal_rtl_otp_bust_ctrl is used to control burst operatoin enable/disable
*
*  @param[in]  uint8_t bust_en: burst enable
*
*  @returns uint32_t: operation succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_bust_ctrl_nsc(uint8_t bust_en)
{
	return hal_otp_bust_ctrl(bust_en);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_sub_aon(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_rtl_otp_rd_aon
*         hal_rtl_otp_rd_aon is used for read function by AON
*
*  @param[in]  uint8_t pwr_cl_en: power cycle in each loop control, 0: disable, 1:enable, power on/off each loop
*  @param[in]  uint32_t addr:     OTP memory address
*  @param[in]  uint8_t *rd_data:  read data buffer pointer
*  @param[in]  uint16_t len: read data length
*
*  @returns uint32_t: read status, succeeded(0), failed(OTP_RET_FAIL_FLG and failed data number)
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_aon(pwr_cl_en, addr, rd_data, len);
}

/**
*  @brief Description of hal_rtl_otp_byte_rd_aon
*         hal_rtl_otp_byte_rd_aon is used to read a single byte according to given address
*
*  @param[in]  uint32_t addr: OTP memory address
*
*  @returns uint8_t: read byte
*/
SECTION_NS_ENTRY_FUNC
uint8_t NS_ENTRY hal_otp_byte_rd_aon_nsc(uint32_t addr)
{
	return hal_otp_byte_rd_aon(addr);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_sub_aon(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_aon(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_byte_wr_aon_nsc(uint32_t addr, uint8_t wr_data)
{
	hal_otp_byte_wr_aon(addr, wr_data);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_cmp_aon_nsc(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_cmp_aon(addr, cmp_data);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_sub_sys(pwr_cl_en, addr, rd_data, len);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_sys(pwr_cl_en, addr, rd_data, len);
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
SECTION_NS_ENTRY_FUNC
uint8_t NS_ENTRY hal_otp_byte_rd_sys_nsc(uint32_t addr)
{
	return hal_otp_byte_rd_sys(addr);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_sub_sys(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_sys(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_byte_wr_sys_nsc(uint32_t addr, uint8_t wr_data)
{
	hal_otp_byte_wr_sys(addr, wr_data);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_cmp_sys_nsc(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_cmp_sys(addr, cmp_data);
}

/**
*  @brief Description of hal_rtl_otp_al_ctrl_sys
*         hal_rtl_otp_al_ctrl_sys is used to control autoload enable/disable flow by SYSON
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_al_ctrl_sys_nsc(uint32_t al_en)
{
	return hal_otp_al_ctrl_sys(al_en);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_sub_syss(pwr_cl_en, addr, rd_data, len);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_rd_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len)
{
	return hal_otp_rd_syss(pwr_cl_en, addr, rd_data, len);
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
SECTION_NS_ENTRY_FUNC
uint8_t NS_ENTRY hal_otp_byte_rd_syss_nsc(uint32_t addr)
{
	return hal_otp_byte_rd_syss(addr);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_sub_syss(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_wr_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len)
{
	return hal_otp_wr_syss(pwr_cl_en, addr, wr_data, len);
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
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_otp_byte_wr_syss_nsc(uint32_t addr, uint8_t wr_data)
{
	hal_otp_byte_wr_syss(addr, wr_data);
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
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_cmp_syss_nsc(uint32_t addr, uint8_t cmp_data)
{
	return hal_otp_cmp_syss(addr, cmp_data);
}


/**
*  @brief Description of hal_rtl_otp_al_ctrl_aon
*         hal_rtl_otp_al_ctrl_aon is used to control autoload enable/disable flow by SYSON-S
*
*  @param[in]  uint32_t al_en: autoload enable
*
*  @returns uint32_t: autoload flow succeeded(0) or failed(OTP_RET_FAIL_FLG)
*/
SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY hal_otp_al_ctrl_syss_nsc(uint32_t al_en)
{
	return hal_otp_al_ctrl_syss(al_en);
}

#endif // end of "#if CONFIG_OTP_EN"

//#endif

