/**************************************************************************//**
 * @file     hal_otp_nsc.h
 * @brief    The header file of hal_flash_nsc.c.
 * @version  1.00
 * @date     2017-08-22
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

#ifndef _HAL_OTP_NSC_H_
#define _HAL_OTP_NSC_H_
#include "cmsis.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

#ifdef  __cplusplus
extern "C"
{
#endif

#if defined(CONFIG_BUILD_SECURE)
/**

        \addtogroup hs_hal_otp_nsc_api OTP HAL NSC APIs
        \ingroup hal_otp
        \brief The OTP HAL NSC (Non-Secure Callable) APIs. Non-secure domain is allowed to access secure functions through NSC APIs.
        @{
*/
void NS_ENTRY hal_otp_init_nsc(void);
void NS_ENTRY hal_otp_deinit_nsc(void);
void NS_ENTRY hal_otp_ctrl_sel_nsc(uint8_t otp_ctrl_sel);
uint8_t NS_ENTRY hal_otp_ctrl_sel_sts_nsc(void);
void NS_ENTRY hal_otp_pmc_gnt_ctrl_nsc(uint8_t gnt_en);
void NS_ENTRY hal_otp_wr_gnt_ctrl_nsc(uint8_t gnt_en);
void NS_ENTRY hal_otpc_ctrl_nsc(uint8_t otpc_en, uint8_t wclk_en, uint8_t rclk_en);
uint32_t NS_ENTRY hal_otp_pwr_ctrl_nsc(uint8_t pwr_en);
void NS_ENTRY hal_otp_test_mod_cfg_nsc(uint8_t test_mod);
void NS_ENTRY hal_otp_test_row_ctrl_nsc(uint8_t test_row_en);
uint32_t NS_ENTRY hal_otp_chk_err_sts_nsc(void);
void NS_ENTRY hal_otp_dsb_ctrl_nsc(uint8_t dsb_en);
void NS_ENTRY hal_otp_rb_ctrl_nsc(uint8_t rb_en);
void NS_ENTRY hal_otp_cz_ctrl_nsc(uint8_t cz_en);
uint32_t NS_ENTRY hal_otp_chk_cz_sts_nsc(uint32_t key_sel);
uint32_t NS_ENTRY hal_otp_clr_cz_sts_nsc(uint32_t key_sel);
uint32_t NS_ENTRY hal_otp_prct_ctrl_nsc(uint8_t prct_en);
uint32_t NS_ENTRY hal_otp_chk_prct_sts_nsc(uint32_t prct_sel);
uint32_t NS_ENTRY hal_otp_rd_lz_er_addr_nsc(void);
uint32_t NS_ENTRY hal_otp_clr_prct_sts_nsc(uint32_t prct_sel);
uint32_t NS_ENTRY hal_otp_rd_sz_prct_sts_nsc(void);
uint32_t NS_ENTRY hal_otp_clr_sz_prct_sts_nsc(uint32_t sz_prct_sts_sel);
uint32_t NS_ENTRY hal_otp_rd_ssz_prct_sts_nsc(void);
uint32_t NS_ENTRY hal_otp_clr_ssz_prct_sts_nsc(uint32_t ssz_prct_sts_sel);
uint32_t NS_ENTRY hal_otp_szwl_ctrl_nsc(uint32_t sz_wl_en);
uint32_t NS_ENTRY hal_otp_rd_aldn_sts_nsc(uint32_t aldn_sts_sel);
uint32_t NS_ENTRY hal_otp_clr_aldn_sts_nsc(uint32_t aldn_sts_sel);
uint32_t NS_ENTRY hal_otp_rd_al_sts_nsc(uint32_t al_sts_sel);
uint32_t NS_ENTRY hal_otp_clr_al_sts_nsc(uint32_t al_sts_sel);
uint32_t NS_ENTRY hal_otp_al_cfg_nsc(uint32_t al_trg_sel);
uint32_t NS_ENTRY hal_otp_al_ctrl_aon_nsc(uint32_t al_en);
uint32_t NS_ENTRY hal_otp_bust_ctrl_nsc(uint8_t bust_en);
uint32_t NS_ENTRY hal_otp_rd_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_rd_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t NS_ENTRY hal_otp_byte_rd_aon_nsc(uint32_t addr);
uint32_t NS_ENTRY hal_otp_wr_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_wr_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void NS_ENTRY hal_otp_byte_wr_aon_nsc(uint32_t addr, uint8_t wr_data);
uint32_t NS_ENTRY hal_otp_cmp_aon_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t NS_ENTRY hal_otp_rd_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_rd_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t NS_ENTRY hal_otp_byte_rd_sys_nsc(uint32_t addr);
uint32_t NS_ENTRY hal_otp_wr_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_wr_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void NS_ENTRY hal_otp_byte_wr_sys_nsc(uint32_t addr, uint8_t wr_data);
uint32_t NS_ENTRY hal_otp_cmp_sys_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t NS_ENTRY hal_otp_al_ctrl_sys_nsc(uint32_t al_en);
uint32_t NS_ENTRY hal_otp_rd_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_rd_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t NS_ENTRY hal_otp_byte_rd_syss_nsc(uint32_t addr);
uint32_t NS_ENTRY hal_otp_wr_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t NS_ENTRY hal_otp_wr_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void NS_ENTRY hal_otp_byte_wr_syss_nsc(uint32_t addr, uint8_t wr_data);
uint32_t NS_ENTRY hal_otp_cmp_syss_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t NS_ENTRY hal_otp_al_ctrl_syss_nsc(uint32_t al_en);

/** *@} */ /* End of group hal_otp_nsc_api */
#endif // end of "#if defined(CONFIG_BUILD_SECURE)"

#if defined(CONFIG_BUILD_NONSECURE)
void hal_otp_init_nsc(void);
void hal_otp_deinit_nsc(void);
void hal_otp_ctrl_sel_nsc(uint8_t otp_ctrl_sel);
uint8_t hal_otp_ctrl_sel_sts_nsc(void);
void hal_otp_pmc_gnt_ctrl_nsc(uint8_t gnt_en);
void hal_otp_wr_gnt_ctrl_nsc(uint8_t gnt_en);
void hal_otpc_ctrl_nsc(uint8_t otpc_en, uint8_t wclk_en, uint8_t rclk_en);
uint32_t hal_otp_pwr_ctrl_nsc(uint8_t pwr_en);
void hal_otp_test_mod_cfg_nsc(uint8_t test_mod);
void hal_otp_test_row_ctrl_nsc(uint8_t test_row_en);
uint32_t hal_otp_chk_err_sts_nsc(void);
void hal_otp_dsb_ctrl_nsc(uint8_t dsb_en);
void hal_otp_rb_ctrl_nsc(uint8_t rb_en);
void hal_otp_cz_ctrl_nsc(uint8_t cz_en);
uint32_t hal_otp_chk_cz_sts_nsc(uint32_t key_sel);
uint32_t hal_otp_clr_cz_sts_nsc(uint32_t key_sel);
uint32_t hal_otp_prct_ctrl_nsc(uint8_t prct_en);
uint32_t hal_otp_chk_prct_sts_nsc(uint32_t prct_sel);
uint32_t hal_otp_rd_lz_er_addr_nsc(void);
uint32_t hal_otp_clr_prct_sts_nsc(uint32_t prct_sel);
uint32_t hal_otp_rd_sz_prct_sts_nsc(void);
uint32_t hal_otp_clr_sz_prct_sts_nsc(uint32_t sz_prct_sts_sel);
uint32_t hal_otp_rd_ssz_prct_sts_nsc(void);
uint32_t hal_otp_clr_ssz_prct_sts_nsc(uint32_t ssz_prct_sts_sel);
uint32_t hal_otp_szwl_ctrl_nsc(uint32_t sz_wl_en);
uint32_t hal_otp_rd_aldn_sts_nsc(uint32_t aldn_sts_sel);
uint32_t hal_otp_clr_aldn_sts_nsc(uint32_t aldn_sts_sel);
uint32_t hal_otp_rd_al_sts_nsc(uint32_t al_sts_sel);
uint32_t hal_otp_clr_al_sts_nsc(uint32_t al_sts_sel);
uint32_t hal_otp_al_cfg_nsc(uint32_t al_trg_sel);
uint32_t hal_otp_al_ctrl_aon_nsc(uint32_t al_en);
uint32_t hal_otp_bust_ctrl_nsc(uint8_t bust_en);
uint32_t hal_otp_rd_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t hal_otp_rd_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t hal_otp_byte_rd_aon_nsc(uint32_t addr);
uint32_t hal_otp_wr_sub_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t hal_otp_wr_aon_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void hal_otp_byte_wr_aon_nsc(uint32_t addr, uint8_t wr_data);
uint32_t hal_otp_cmp_aon_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t hal_otp_rd_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t hal_otp_rd_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t hal_otp_byte_rd_sys_nsc(uint32_t addr);
uint32_t hal_otp_wr_sub_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t hal_otp_wr_sys_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void hal_otp_byte_wr_sys_nsc(uint32_t addr, uint8_t wr_data);
uint32_t hal_otp_cmp_sys_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t hal_otp_al_ctrl_sys_nsc(uint32_t al_en);
uint32_t hal_otp_rd_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint32_t hal_otp_rd_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *rd_data, uint16_t len);
uint8_t hal_otp_byte_rd_syss_nsc(uint32_t addr);
uint32_t hal_otp_wr_sub_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
uint32_t hal_otp_wr_syss_nsc(uint8_t pwr_cl_en, uint32_t addr, uint8_t *wr_data, uint16_t len);
void hal_otp_byte_wr_syss_nsc(uint32_t addr, uint8_t wr_data);
uint32_t hal_otp_cmp_syss_nsc(uint32_t addr, uint8_t cmp_data);
uint32_t hal_otp_al_ctrl_syss_nsc(uint32_t al_en);

/* Wrapper for NSC functions */
#define hal_otp_init                            hal_otp_init_nsc
#define hal_otp_deinit                          hal_otp_deinit_nsc
#define hal_otp_ctrl_sel                        hal_otp_ctrl_sel_nsc
#define hal_otp_ctrl_sel_sts                    hal_otp_ctrl_sel_sts_nsc
#define hal_otp_pmc_gnt_ctrl                    hal_otp_pmc_gnt_ctrl_nsc
#define hal_otp_wr_gnt_ctrl                     hal_otp_wr_gnt_ctrl_nsc
#define hal_otpc_ctrl                           hal_otpc_ctrl_nsc
#define hal_otp_pwr_ctrl                        hal_otp_pwr_ctrl_nsc
#define hal_otp_test_mod_cfg                    hal_otp_test_mod_cfg_nsc
#define hal_otp_test_row_ctrl                   hal_otp_test_row_ctrl_nsc
#define hal_otp_chk_err_sts                     hal_otp_chk_err_sts_nsc
#define hal_otp_dsb_ctrl                        hal_otp_dsb_ctrl_nsc
#define hal_otp_rb_ctrl                         hal_otp_rb_ctrl_nsc
#define hal_otp_cz_crtl                         hal_otp_cz_ctrl_nsc
#define hal_otp_chk_cz_sts                      hal_otp_chk_cz_sts_nsc
#define hal_otp_clr_cz_sts                      hal_otp_clr_cz_sts_nsc
#define hal_otp_prct_ctrl                       hal_otp_prct_ctrl_nsc
#define hal_otp_chk_prct_sts                    hal_otp_chk_prct_sts_nsc
#define hal_otp_rd_lz_er_addr                   hal_otp_rd_lz_er_addr_nsc
#define hal_otp_clr_prct_sts                    hal_otp_clr_prct_sts_nsc
#define hal_otp_rd_sz_prct_sts                  hal_otp_rd_sz_prct_sts_nsc
#define hal_otp_clr_sz_prct_sts                 hal_otp_clr_sz_prct_sts_nsc
#define hal_otp_rd_ssz_prct_sts                 hal_otp_rd_ssz_prct_sts_nsc
#define hal_otp_clr_ssz_prct_sts                hal_otp_clr_ssz_prct_sts_nsc
#define hal_otp_szwl_ctrl                       hal_otp_szwl_ctrl_nsc
#define hal_otp_rd_aldn_sts                     hal_otp_rd_aldn_sts_nsc
#define hal_otp_clr_aldn_sts                    hal_otp_clr_aldn_sts_nsc
#define hal_otp_rd_al_sts                       hal_otp_rd_al_sts_nsc
#define hal_otp_clr_al_sts                      hal_otp_clr_al_sts_nsc
#define hal_otp_al_cfg                          hal_otp_al_cfg_nsc
#define hal_otp_al_ctrl_aon                     hal_otp_al_ctrl_aon_nsc
#define hal_otp_bust_ctrl                       hal_otp_bust_ctrl_nsc
#define hal_otp_rd_sub_aon                      hal_otp_rd_sub_aon_nsc
#define hal_otp_rd_aon                          hal_otp_rd_aon_nsc
#define hal_otp_byte_rd_aon                     hal_otp_byte_rd_aon_nsc
#define hal_otp_wr_sub_aon                      hal_otp_wr_sub_aon_nsc
#define hal_otp_wr_aon                          hal_otp_wr_aon_nsc
#define hal_otp_byte_wr_aon                     hal_otp_byte_wr_aon_nsc
#define hal_otp_cmp_aon                         hal_otp_cmp_aon_nsc
#define hal_otp_rd_sub_sys                      hal_otp_rd_sub_sys_nsc
#define hal_otp_rd_sys                          hal_otp_rd_sys_nsc
#define hal_otp_byte_rd_sys                     hal_otp_byte_rd_sys_nsc
#define hal_otp_wr_sub_sys                      hal_otp_wr_sub_sys_nsc
#define hal_otp_wr_sys                          hal_otp_wr_sys_nsc
#define hal_otp_byte_wr_sys                     hal_otp_byte_wr_sys_nsc
#define hal_otp_cmp_sys                         hal_otp_cmp_sys_nsc
#define hal_otp_al_ctrl_sys                     hal_otp_al_ctrl_sys_nsc
#define hal_otp_rd_sub_syss                     hal_otp_rd_sub_syss_nsc
#define hal_otp_rd_syss                         hal_otp_rd_syss_nsc
#define hal_otp_byte_rd_syss                    hal_otp_byte_rd_syss_nsc
#define hal_otp_wr_sub_syss                     hal_otp_wr_sub_syss_nsc
#define hal_otp_wr_syss                         hal_otp_wr_syss_nsc
#define hal_otp_byte_wr_syss                    hal_otp_byte_wr_syss_nsc
#define hal_otp_cmp_syss                        hal_otp_cmp_syss_nsc
#define hal_otp_al_ctrl_syss                    hal_otp_al_ctrl_syss_nsc

#endif  // end of "#if defined(CONFIG_BUILD_NONSECURE)"

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_OTP_NSC_H_"


