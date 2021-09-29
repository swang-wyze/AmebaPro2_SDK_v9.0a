/**************************************************************************//**
 * @file     system_rtl8735b.c
 * @brief    CMSIS Device System Source File for rtl8735b platform.
 *
 * @version  1.00
 * @date     2016-07-20
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

#if defined (__ARM_FEATURE_CMSE) &&  (__ARM_FEATURE_CMSE == 3U)
#include "partition_rtl8735b.h"
#endif

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  SYSTEM_CLOCK    CONFIG_CPU_CLK


/*----------------------------------------------------------------------------
  Externals
 *----------------------------------------------------------------------------*/
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1)
extern uint32_t __Vectors;
#endif

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
//uint32_t SystemCoreClock = SYSTEM_CLOCK;

/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
//    SystemCoreClock = SYSTEM_CLOCK;
}

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit(void)
{
	uint32_t i;

	// clear all pending interrupt
	for (i = 0; i < MAX_PERIPHERAL_IRQ_NUM; i++) {
		NVIC_DisableIRQ(i);
		NVIC_ClearPendingIRQ(i);
	}

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1)
	SCB->VTOR = (uint32_t) &__Vectors;
#endif

#if defined (__FPU_USED) && (__FPU_USED == 1)
	SCB->CPACR |= ((3U << 10 * 2) |         /* set CP10 Full Access */
				   (3U << 11 * 2));         /* set CP11 Full Access */
	// disable FPU auto stacking
	// for debugging HAL_WRITE32 (0, 0xE000EF34, (HAL_READ32 (0, 0xE000EF34) & 0x00FFFFFF));
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
	SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif
	SCB->CCR |= SCB_CCR_BP_Msk;

	SCB->CCR |= 0x10; // Usage Fault: enable div-by-0
	// Enable Security Fault, Usage Fault, Bus Fault and Mem Management Fault Interrupt
	SCB->SHCSR |= SCB_SHCSR_SECUREFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk |
				  SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

//    SystemCoreClock = SYSTEM_CLOCK;
}

