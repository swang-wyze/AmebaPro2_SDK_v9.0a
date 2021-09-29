/**************************************************************************//**
 * @file     startup_rtl8195bh.h
 * @brief    CMSIS Core Device Startup File for for rtl8195b-hp.
 * @version  V1.00
 * @date     2016-07-19
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

#include <stdint.h>
#include "cmsis.h"


/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)       /* ARM Compiler 6 */
extern uint32_t Image$$_STACK$$ZI$$Limit;
#define __StackTop Image$$_STACK$$ZI$$Limit
#else
extern uint32_t __StackTop;
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler Function Prototype
 *----------------------------------------------------------------------------*/
typedef void(*pFunc)(void);


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern void _start_rtl8735b(void) __attribute__((noreturn));    /* main entry point */
#define __START    _start_rtl8735b

#ifndef __NO_SYSTEM_INIT
extern void SystemInit(void);             /* CMSIS System Initialization      */
#endif


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);                          /* Default empty handler */
void Reset_Handler(void);                            /* Reset Handler */
void HardFault_Handler(void);                        /* Hard Fault Handler */
void BusFault_Handler(void);                         /* Bus Fault Handler */
void UsageFault_Handler(void);                       /* Usage Fault Handler */
void MemManage_Handler(void);                        /* Mem Management Fault Handler */
void SecureFault_Handler(void);                      /* Security Fault Handler */

/*----------------------------------------------------------------------------
  User Initial Stack & Heap
 *----------------------------------------------------------------------------*/
#ifndef __STACK_SIZE
#define   __STACK_SIZE  0x00001000
#endif
static uint8_t stack[__STACK_SIZE] __attribute__((aligned(8), used, section(".stack")));

#if 0 // Raymond remove rom use HEAP
#ifndef __HEAP_SIZE
#define __HEAP_SIZE   0x00002000
#endif
#if __HEAP_SIZE > 0
//static uint8_t heap[__HEAP_SIZE]   __attribute__ ((aligned(8), used, section(".heap")));
#endif
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* RTL8735B TM9 Processor Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
//void MemManage_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
//void BusFault_Handler           (void) __attribute__ ((weak, alias("Default_Handler")));
//void UsageFault_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
//void SecureFault_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* RTL8735B TM9 Platform Peripheral Interrupts */
void SystemOn_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TimerGroup0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TimerGroup1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TimerGroup2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TimerGroup3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SGPIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BTUART_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPORT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SDIOH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SDH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void MII_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WLAN_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WLAN_PWR_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RSA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPIC_NAND_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GDMA0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GDMA1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void Crypto_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FlashCtrl_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void AON_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ISP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ENC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void VOE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void NN_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SGDMA0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SGDMA1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SCrypto_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECDSA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ED25519_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SRXI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RXI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WDT_VOE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WDT_WL_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SRC_OCP_OUT_L_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OCP_OUT_L_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PonGPIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TRNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BOD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void Comparator_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void Flash_SEC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FEPHY_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
const pFunc __Vectors[] __attribute__((section(".rom.vectors"))) = {
	/* RTL8735B TM9 Processor Exceptions Handler */
	(pFunc)((uint32_t)&__StackTop),           /*      Initial Stack Pointer     */
	Reset_Handler,                            /*      Reset Handler             */
	NMI_Handler,                              /*      NMI Handler               */
	HardFault_Handler,                        /*      Hard Fault Handler        */
	MemManage_Handler,                        /*      MPU Fault Handler         */
	BusFault_Handler,                         /*      Bus Fault Handler         */
	UsageFault_Handler,                       /*      Usage Fault Handler       */
	SecureFault_Handler,                      /*      Secure Fault Handler      */
	0,                                        /*      Reserved                  */
	0,                                        /*      Reserved                  */
	0,                                        /*      Reserved                  */
	SVC_Handler,                              /*      SVCall Handler            */
	DebugMon_Handler,                         /*      Debug Monitor Handler     */
	0,                                        /*      Reserved                  */
	PendSV_Handler,                           /*      PendSV Handler            */
	SysTick_Handler,                          /*      SysTick Handler           */

	/* RTL8735B TM9 Platform External interrupts */
	SystemOn_IRQHandler,                      /*  0:  System On                 */
	TimerGroup0_IRQHandler,                   /*  1:  G-Timer (Group 0)         */
	TimerGroup1_IRQHandler,                   /*  2:  G-Timer (Group 1)         */
	TimerGroup2_IRQHandler,                   /*  3:  G-Timer (Group 2)         */
	TimerGroup3_IRQHandler,                   /*  4:  G-Timer (Group 3)         */
	GPIO_IRQHandler,                          /*  5:  GPIO                      */
	PWM_IRQHandler,                           /*  6:  PWM                       */
	ADC_IRQHandler,                           /*  7:  ADC                       */
	SGPIO_IRQHandler,                         /*  8:  Serial-GPIO               */
	UART0_IRQHandler,                         /*  9:  UART0                     */
	UART1_IRQHandler,                         /* 10:  UART1                     */
	UART2_IRQHandler,                         /* 11:  UART2                     */
	UART3_IRQHandler,                         /* 12:  UART3                     */
	BTUART_IRQHandler,                        /* 13:  BT UART                   */
	I2C0_IRQHandler,                          /* 14:  I2C0                      */
	I2C1_IRQHandler,                          /* 15:  I2C1                      */
	I2C2_IRQHandler,                          /* 16:  I2C2                      */
	I2C3_IRQHandler,                          /* 17:  I2C3                      */
	SPI0_IRQHandler,                          /* 18:  SPI0                      */
	SPI1_IRQHandler,                          /* 19:  SPI1                      */
	HSPI0_IRQHandler,                         /* 20:  H-SPI0                    */
	HSPI1_IRQHandler,                         /* 21:  H-SPI1                    */
	I2S0_IRQHandler,                          /* 22:  I2S0                      */
	I2S1_IRQHandler,                          /* 23:  I2S1                      */
	SPORT_IRQHandler,                         /* 24:  SPORT                     */
	USB_IRQHandler,                           /* 25:  USB_OTG                   */
	SDIOH_IRQHandler,                         /* 26:  SDIO_H                    */
	SDH_IRQHandler,                           /* 27:  SD_Host                   */
	MII_IRQHandler,                           /* 28:  MII (GMAC)                */
	WLAN_IRQHandler,                          /* 29:  WLan                      */
	WLAN_PWR_IRQHandler,                      /* 30:  WLAN_power                */
	RSA_IRQHandler,                           /* 31:  RSA                       */
	SPIC_NAND_IRQHandler,                     /* 32:  spi_nand_int              */
	GDMA0_IRQHandler,                         /* 33:  GDMA0                     */
	GDMA1_IRQHandler,                         /* 34:  GDMA1                     */
	Crypto_IRQHandler,                        /* 35:  Crypto Engine             */
	FlashCtrl_IRQHandler,                     /* 36:  SPI Flash Controller      */
	AON_IRQHandler,                           /* 37:  AON (AON all IPs)         */
	ISP_IRQHandler,                           /* 38:  ISP                       */
	ENC_IRQHandler,                           /* 39:  h264/h265/JPEG            */
	VOE_IRQHandler,                           /* 40:  VOE                       */
	NN_IRQHandler,                            /* 41:  NN                        */
	SGDMA0_IRQHandler,                        /* 42:  Secure GDMA0              */
	SGDMA1_IRQHandler,                        /* 43:  Secure GDMA1              */
	SCrypto_IRQHandler,                       /* 44:  Secure Crypto engine      */
	ECDSA_IRQHandler,                         /* 45:  ECDSA                     */
	ED25519_IRQHandler,                       /* 46:  ED25519                   */
	SRXI_IRQHandler,                          /* 47:  Secure RXI-300            */
	RXI_IRQHandler,                           /* 48:  Non-Secure RXI-300        */
	WDT_VOE_IRQHandler,                       /* 49:  WDT_VOE                   */
	WDT_WL_IRQHandler,                        /* 50:  WDT_WL                    */
	SRC_OCP_OUT_L_IRQHandler,                 /* 51:  SRC_Over Current Protection*/
	OCP_OUT_L_IRQHandler,                     /* 52:  None                      */
	PonGPIO_IRQHandler,                       /* 53:  PON GPIO                  */
	TRNG_IRQHandler,                          /* 54:  TRNG interrupt            */
	BOD_IRQHandler,                           /* 55:  BOD interrupt             */
	RTC_IRQHandler,                           /* 56:  RTC interrupt             */
	Comparator_IRQHandler,                    /* 57:  Comprartor interrupt      */
	Flash_SEC_IRQHandler,                     /* 58:  Flash SEC                 */
	FEPHY_IRQHandler,                         /* 59:  FEPHY                     */
};


/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
void Reset_Handler(void)
{
	SCB->VTOR = (uint32_t)__Vectors;
	__set_MSP((uint32_t)&__StackTop);
//	__START();
	__ASM volatile(
		"LDR R2,=_start_rtl8735b\n\t"
		"BX R2\n\t"
	);

}

void UsageFault_Handler(void)
{
	__ASM volatile(
		"PUSH {R4-R11}\n\t"
		"MRS R0, MSP\n\t"
		"MRS R1, PSP\n\t"
		"MOV R2, LR\n\t" /* second parameter is LR current value */
		"MOV R3, #1\n\t"
		"SUB.W	R4, R0, #128\n\t"
		"MSR MSP, R4\n\t"
		"LDR R4,=hard_fault_handler_c\n\t"
		"BX R4\n\t"
	);
}

void BusFault_Handler(void)
{
	__ASM volatile(
		"PUSH {R4-R11}\n\t"
		"MRS R0, MSP\n\t"
		"MRS R1, PSP\n\t"
		"MOV R2, LR\n\t" /* second parameter is LR current value */
		"MOV R3, #2\n\t"
		"SUB.W	R4, R0, #128\n\t"
		"MSR MSP, R4\n\t"
		"LDR R4,=hard_fault_handler_c\n\t"
		"BX R4\n\t"
	);
}

void MemManage_Handler(void)
{
	__ASM volatile(
		"PUSH {R4-R11}\n\t"
		"MRS R0, MSP\n\t"
		"MRS R1, PSP\n\t"
		"MOV R2, LR\n\t" /* second parameter is LR current value */
		"MOV R3, #3\n\t"
		"SUB.W	R4, R0, #128\n\t"
		"MSR MSP, R4\n\t"
		"LDR R4,=hard_fault_handler_c\n\t"
		"BX R4\n\t"
	);
}

void SecureFault_Handler(void)
{
	__ASM volatile(
		"PUSH {R4-R11}\n\t"
		"MRS R0, MSP\n\t"
		"MRS R1, PSP\n\t"
		"MOV R2, LR\n\t" /* second parameter is LR current value */
		"MOV R3, #4\n\t"
		"SUB.W	R4, R0, #128\n\t"
		"MSR MSP, R4\n\t"
		"LDR R4,=hard_fault_handler_c\n\t"
		"BX R4\n\t"
	);
}

#if 0
void HardFault_Handler(void)
{
	__ASM volatile(
		"TST lr, #4\n\t"
		"ITE EQ\n\t"
		"MRSEQ r0, MSP\n\t"
		"MRSNE r0, PSP\n\t"
		"MOV R1, LR\n\t" /* second parameter is LR current value */
		"LDR R2,=hard_fault_handler_c\n\t"
		"BX R2\n\t"
	);
}
#else
void HardFault_Handler(void)
{
	__ASM volatile(
		"PUSH {R4-R11}\n\t"
		"MRS R0, MSP\n\t"
		"MRS R1, PSP\n\t"
		"MOV R2, LR\n\t" /* second parameter is LR current value */
		"MOV R3, #0\n\t"
		"SUB.W	R4, R0, #128\n\t"
		"MSR MSP, R4\n\t"   // Move MSP to upper to for we can dump current stack contents without chage contents
		"LDR R4,=hard_fault_handler_c\n\t"
		"BX R4\n\t"
	);
}
#endif

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
extern  void _default_handler(void);

void Default_Handler(void)
{

	_default_handler();
}

