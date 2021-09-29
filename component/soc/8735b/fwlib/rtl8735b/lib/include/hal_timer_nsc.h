/**************************************************************************//**
* @file        hal_timer_nsc.h
* @brief       The HAL Non-secure callable API implementation for the EFUSE
*
* @version     V1.00
* @date        2020-11-12
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



#ifndef _HAL_TIMER_NSC_H_
#define _HAL_TIMER_NSC_H_
#include "cmsis.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

#ifdef  __cplusplus
extern "C"
{
#endif



#if defined(CONFIG_BUILD_SECURE)
void NS_ENTRY hal_timer_clock_init_nsc(uint32_t tgid, int en);
void NS_ENTRY hal_timer_group_sclk_sel_nsc(hal_timer_group_adapter_t *ptg_adp, uint32_t sclk_freq);

#endif

#if defined(CONFIG_BUILD_NONSECURE)
void hal_timer_clock_init_nsc(uint32_t tgid, int en);
void hal_timer_group_sclk_sel_nsc(hal_timer_group_adapter_t *ptg_adp, uint32_t sclk_freq);

#define hal_timer_clock_init        hal_timer_clock_init_nsc
#define hal_timer_group_sclk_sel    hal_timer_group_sclk_sel_nsc


#endif  // end of "#if defined(CONFIG_BUILD_NONSECURE)"


#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_TIMER_NSC_H_"
