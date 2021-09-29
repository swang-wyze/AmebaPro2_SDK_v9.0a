/**************************************************************************//**
* @file        rtl8735b_sys_ctrl.h
* @brief       The HAL API implementation for SYSTEM CONTROL
*
* @version     V1.00
* @date        2021-08-07
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



#ifndef _RTL8735B_SYS_CTRL_H_
#define _RTL8735B_SYS_CTRL_H_
//#include "cmsis.h"
#include "rtl8735b_syson_s_type.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define BOOT_INFO_SW_RESV_MAX_SIZE              (5)
#define OTP_CHIP_ID_MAX_SIZE                    (4)

typedef enum {
	RMA_NORMAL_STATE   = 0x00,
	RMA_STATE0         = 0xA0,
	RMA_STATE1         = 0xA1,
	RMA_STATE2         = 0xA2
} RMA_STATE_T;

typedef enum {
	VIDEO_INFO_NN   = 9,
	VIDEO_INFO_H265 = 10,
	VIDEO_INFO_ENC  = 11,
	VIDEO_INFO_ISP  = 12,
	VIDEO_INFO_ALL  = 0xFF,
} VIDEO_INFO_IDX_T;

typedef enum {
	BOOT_INFO_IDX0  = 0,
	BOOT_INFO_IDX1  = 1,
	BOOT_INFO_IDX2  = 2,
	BOOT_INFO_IDX3  = 3,
	BOOT_INFO_IDX4  = 4
} BOOT_INFO_IDX_T;

/**
* @addtogroup N/A
* @{
*/

typedef enum {
	EDDSA_SYS = 0,
	GPIO_SYS = 1,
	GPIO_PON = 2,
	GPIO_AON = 3,

	DDR_SYS = 4,

	GDMA0_SYS = 5,
	GDMA1_SYS = 6,
	FLASH_SYS = 7,
	SPI0_SYS = 8,
	SPI1_SYS = 9,
	HS_SPI0_SYS = 10,
	HS_SPI1_SYS = 11,
	UART0_SYS = 12,
	UART1_SYS = 13,
	UART2_SYS = 14,
	UART3_SYS = 15,
	UART4_SYS = 16,
	TIMER0_SYS = 17,
	TIMER1_SYS = 18,
	TIMER2_SYS = 19,
	TIMER3_SYS = 20,
	PWM_SYS = 21,
	RSA_SYS = 22,
	CRYPTO_SYS = 23,
	I2C0_SYS = 24,
	I2C1_SYS = 25,
	I2C2_SYS = 26,
	I2C3_SYS = 27,
	ECDSA_SYS = 28,
	ADC_SYS = 29,
	RTC_SYS = 30,
	SPORT_SYS = 31,
	I2S0_SYS = 32,
	I2S1_SYS = 33,
	LXBUS_SYS = 34,

	ISP_SYS = 35,
	MIPI_SYS = 36,
	ENC_SYS = 37,
	NN_SYS = 38,
	VOE_SYS = 39,

	CPU_SYS = 40,
	SGPIO_SYS = 41,
	TRNG_SYS = 42,

	SI_SYS = 43,
	AUDIO_CODEC_EN = 44,
	AUDIO_CODEC_SCLK_EN = 45,
	AUDIO_CODEC_PORB_EN = 46,
	AUDIO_CODEC_LDO_EN = 47,
	AUDIO_CODEC_EXTE_EN = 48,
	AUDIO_CODEC_LDO_TUNE = 49,
	TRNG_32K = 50,
	TRNG_128K = 51,
	LDO_SDIO_3V3_EN = 52,
} IPs_CLK_FUNC_t;

typedef enum {
	UART_IRC_4M = 0,
	UART_PERI_40M,
	UART_XTAL,
	RSVD
} UART0_CLK_SELECT_t;


typedef enum {
	ENC_166M = 0,   // SYS_PLL/3,  500Mhz or 300Mhz/3
	ENC_125M,       // SYS_PLL/4,  500Mhz or 300Mhz/4
	ENC_100M,       // Peri_PLL/4, 400Mhz/4=100Mhz
} ENC_CLK_SELECT_t;

typedef enum {
	CPU_500M = 0,   // CLK 500Mhz
	CPU_400M,       // CLK 400Mhz
	CPU_300M,       // CLK 300Mhz
} CPU_CLK_SELECT_t;


typedef enum {
	NN_500M = 0,   // CLK 500Mhz
	NN_400M,       // CLK 400Mhz
	NN_250M,       // CLK 2500Mhz
} NN_CLK_SELECT_t;

typedef union {
	__IOM uint8_t byte;

	struct {
		__IOM uint8_t resolution           : 2;       /*!< [1..0]  */
		__IOM uint8_t mem_size_sel1        : 2;       /*!< [3..2]  */
		__IOM uint8_t mem_size_sel2        : 2;       /*!< [5..4]  */
		__IOM uint8_t dual_band            : 2;       /*!< [7..6]  */
	} bit;
} otp_chip_id0_t, *potp_chip_id0_t;

typedef union {
	__IOM uint8_t byte;

	struct {
		__IOM uint8_t mipi_intf_en         : 2;       /*!< [1..0]  */
		__IOM uint8_t isp_en               : 2;       /*!< [3..2] (2'b11:ON, 2'b10:OFF, 2'b01:OFF, 2'b00:ON) */
		__IOM uint8_t video_enc_sel        : 2;       /*!< [5..4] (2'b11:H.265+H.264, 2'b10:H.264, 2'b01:H.265+H.264, 2'b00:OFF)*/
		__IOM uint8_t lps_mode_en          : 2;       /*!< [7..6]  */
	} bit;
} otp_chip_id1_t, *potp_chip_id1_t;

typedef union {
	__IOM uint8_t byte;

	struct {
		__IOM uint8_t ddr2_3_sel           : 2;       /*!< [1..0]  */
		__IOM uint8_t nn_turn_key_sel      : 2;       /*!< [3..2]  */
		__IOM uint8_t nn_engine_en         : 2;       /*!< [5..4]  (2'b11:ON, 2'b10:OFF, 2'b01:OFF, 2'b00:ON) */
		__IOM uint8_t eth_en               : 2;       /*!< [7..6]  */
	} bit;
} otp_chip_id2_t, *potp_chip_id2_t;

typedef union {
	__IOM uint8_t byte;

	struct {
		__IM  uint8_t : 8;                        /*!< [7..0] Reserved */
	} bit;
} otp_chip_id3_t, *potp_chip_id3_t;

typedef union {
	__IOM uint32_t word;

	struct {
		otp_chip_id0_t cfg0;                 /*!< [0..7]   OTP chid0 defined at 0x498 */
		otp_chip_id1_t cfg1;                 /*!< [8..15]  OTP chid1 defined at 0x499 */
		otp_chip_id2_t cfg2;                 /*!< [16..23] OTP chid2 defined at 0x49A*/
		otp_chip_id3_t cfg3;                 /*!< [24..31] OTP chid3 defined at 0x49B */
	} byte;
} otp_chip_id_t, *potp_chip_id_t;

typedef enum {
	ISP_CHIP_ID_ON_1     = 0x0,
	ISP_CHIP_ID_OFF_1    = 0x1,
	ISP_CHIP_ID_OFF_2    = 0x2,
	ISP_CHIP_ID_ON_2     = 0x3,
} ISP_CHIP_ID_EN_CTRL_t;

typedef enum {
	V_EN_CHIP_ID_SEL_H265_H264_1  = 0x0,
	V_EN_CHIP_ID_SEL_OFF          = 0x1,
	V_EN_CHIP_ID_H264             = 0x2,
	V_EN_CHIP_ID_SEL_H265_H264_2  = 0x3,
} VIDEO_ENC_CHIP_ID_SEL_CTRL_t;

typedef enum {
	NN_ENG_CHIP_ID_ON_1     = 0x0,
	NN_ENG_CHIP_ID_OFF_1    = 0x1,
	NN_ENG_CHIP_ID_OFF_2    = 0x2,
	NN_ENG_CHIP_ID_ON_2     = 0x3,
} NN_ENG_CHIP_ID_EN_CTRL_t;


#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
uint8_t hal_rtl_sys_get_rma_state(void);
void hal_rtl_sys_en_rma_mgn_pg(void);
#endif

void hal_rtl_sys_peripheral_en(uint8_t id, uint8_t en);
void hal_rtl_sys_set_clk(uint8_t id, uint8_t sel_val);
uint32_t hal_rtl_sys_get_clk(uint8_t id);

void hal_rtl_sys_boot_info_assign_val(uint8_t info_idx, uint32_t info_v);
uint32_t hal_rtl_sys_boot_info_get_val(uint8_t info_idx);
void hal_rtl_sys_boot_footpath_init(uint8_t info_idx);
void hal_rtl_sys_boot_footpath_store(uint8_t info_idx, uint8_t fp_v);
void hal_rtl_sys_boot_footpath_clear(uint8_t info_idx, uint8_t fp_v);
void hal_rtl_vdr_s_jtag_key_write(uint8_t *pkey);
void hal_rtl_vdr_ns_jtag_key_write(uint8_t *pkey);
uint32_t hal_rtl_sys_get_video_info(uint8_t idx);
void hal_rtl_sys_set_video_info(uint8_t idx, uint8_t en_ctrl);
void hal_rtl_sys_get_chip_id(uint32_t *pchip_id);

#define ROM_FOOTPH_INIT(idx)                hal_rtl_sys_boot_footpath_init(idx)
#define ROM_FOOTPH_STORE(idx,fp_v)          hal_rtl_sys_boot_footpath_store(idx,fp_v)
#define ROM_FOOTPH_CLR(idx,fp_v)            hal_rtl_sys_boot_footpath_clear(idx,fp_v)


/**
  \brief  The data structure of the stubs function for the SYSON HAL functions in ROM
*/
typedef struct hal_sys_ctrl_func_stubs_s {
	void (*hal_sys_peripheral_en)(uint8_t id, uint8_t en);
	void (*hal_sys_set_clk)(uint8_t id, uint8_t sel_val);
	uint32_t (*hal_sys_get_clk)(uint8_t id);
	uint32_t (*hal_sys_boot_info_get_val)(uint8_t info_idx);
	void (*hal_sys_boot_info_assign_val)(uint8_t info_idx, uint32_t info_v);
	void (*hal_sys_boot_footpath_init)(uint8_t info_idx);
	void (*hal_sys_boot_footpath_store)(uint8_t info_idx, uint8_t fp_v);
	void (*hal_vdr_s_jtag_key_write)(uint8_t *pkey);
	void (*hal_vdr_ns_jtag_key_write)(uint8_t *pkey);
	uint32_t (*hal_sys_get_video_info)(uint8_t idx);
	void (*hal_sys_set_video_info)(uint8_t idx, uint8_t en_ctrl);
	void (*hal_sys_get_chip_id)(uint32_t *pchip_id);
	void (*hal_sys_boot_footpath_clear)(uint8_t info_idx, uint8_t fp_v);
	uint8_t (*hal_sys_get_rma_state)(void);
	void (*hal_sys_high_value_assets_otp_lock)(const uint8_t lock_obj);
	uint32_t reserved[9];  // reserved space for next ROM code version function table extending.
} hal_sys_ctrl_func_stubs_t;



/** @} */ /* End of group hs_hal_efuse */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _RTL8735B_SYS_CTRL_H_"


