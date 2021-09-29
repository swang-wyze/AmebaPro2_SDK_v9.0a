/**************************************************************************//**
 * @file     rtl8735b_symbns4s.c
 * @brief    Define the symbol table to be referenced by Secure Region ROM code
 * @version  V1.00
 * @date     2020-11-11
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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
#include "rtl8735b_symbns4s.h"

#define SECTION_NS4S_STUBS                  SECTION(".rom.ns4s.stubs")

extern void _stdio_port_init(void *adapter, stdio_putc_t putc, stdio_getc_t getc);
extern void shell_cmd_task_init(void);
extern void start_ns_rtl8735b(void);
//extern void _curve25519_donna(unsigned char *, const unsigned char *, const unsigned char *);
extern void rom_cmd_shell_set_prompt(void);

extern const cmd_shell_func_stubs_t cmd_shell_stubs;
extern const hal_gpio_func_stubs_t hal_gpio_stubs;
//extern const hal_spic_func_stubs_t hal_spic_stubs;
//extern const hal_flash_func_stubs_t hal_flash_stubs;
//extern const hal_sdiod_func_stubs_t hal_sdiod_stubs;

extern phal_timer_adapter_t psystem_timer;
#if 0
SECTION_NS4S_STUBS const symb_ns4s_t symb_ns4s_stubs = {
	.pcmd_shell_stubs = &cmd_shell_stubs,
	.phal_gpio_stubs = &hal_gpio_stubs,
	//.phal_spic_stubs = &hal_spic_stubs,
	//.phal_flash_stubs = &hal_flash_stubs,
	//.phal_sdiod_stubs = &hal_sdiod_stubs,
	.ppsystem_timer = &psystem_timer,
	.startup_ns = start_ns_rtl8735b,
	.stdio_port_init_ns = _stdio_port_init,
	.shell_cmd_task_init = shell_cmd_task_init,
	//.curve25519_donna = _curve25519_donna,
	.rom_cmd_shell_set_prompt = rom_cmd_shell_set_prompt
};
#endif

SECTION_NS4S_STUBS const symb_ns4s_t symb_ns4s_stubs = {
	.pcmd_shell_stubs = &cmd_shell_stubs,
	.phal_gpio_stubs = &hal_gpio_stubs,
	//.phal_spic_stubs = &hal_spic_stubs,
	//.phal_flash_stubs = &hal_flash_stubs,
	//.phal_sdiod_stubs = &hal_sdiod_stubs,
	.ppsystem_timer = &psystem_timer,
	.startup_ns = start_ns_rtl8735b,
	.stdio_port_init_ns = _stdio_port_init
};

