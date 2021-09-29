/**************************************************************************//**
* @file        hal_timer_nsc.c
* @brief       This file implements the entry functions of the Efuse Non-secure callable HAL functions.
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
#include "hal_crypto.h"
#include "hal_crypto_nsc.h"


#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#undef SECTION_NS_ENTRY_FUNC
#undef NS_ENTRY
#define SECTION_NS_ENTRY_FUNC
#define NS_ENTRY
#endif

//SECTION_NS_ENTRY_FUNC
//void NS_ENTRY hal_pwm_clock_init_nsc(int en)
//{
//    hal_pwm_stubs.hal_pwm_clock_init(en);
//}
//
//SECTION_NS_ENTRY_FUNC
//void NS_ENTRY hal_pwm_clk_sel_nsc(int Intclk, int Sclk)
//{
//    hal_pwm_stubs.hal_pwm_clk_sel(Intclk, Sclk);
//}

