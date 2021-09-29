/**************************************************************************//**
 * @file    rtl8735b_sdhost.h
 * @brief    The HAL related definition and macros for SD Host controller.
 *           Includes Registers and data type definition.
 * @version V1.00
 * @date    2020-10-12
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
#ifndef _RTL8735B_SDHOST_H_
#define _RTL8735B_SDHOST_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hal_sdhost SDHOST
 * @ingroup 8195bh_hal
 * @{
 * @brief The SD Host HAL module of the HS platform.
 */

#include "rtl8735b_sdhost_type.h"



/// Defines the max. timeout value when checking the flag of command complete
#define SDHOST_CMD_CPLT_TIMEOUT      5000        // 5 ms
/// Defines the max. timeout value when checking the flag of transfer complete
//#define SDHOST_XFER_TIMEOUT_MAX      0xF0000000
/// Defines the timeout value when checking transfer complete
#define SDHOST_XFER_CPLT_TIMEOUT     3000000     // 3 s
/// Defines the timeout value when checking small transfer complete
#define SDHOST_SMALL_XFER_TIMEOUT    5000        // 5 ms
/// Defines the timeout value for waiting bus status
#define SDHOST_BUS_TIMEOUT           1000        // 1 ms
/// Defines the max. timeout value when checking the process of card initialization
#define SDHOST_ACMD41_TIMEOUT        1000000     // 1 sec
/// Defines the time interval when checking the process of card initialization
#define SDHOST_ACMD41_POLL_INTERVAL  10000       // 10 ms

/// Defines the mask of register "SD Bus Status"
#define SDHOST_BUS_STS_MASK          0x1F        // 0x585 bit[4:0]
/// Defines the mask of register "SD_CMD0"
#define SDHOST_CMD_IDX_MASK          0x3F        // 0x589 bit[5:0]

/// Defines the value of "VHS" field in CMD8
#define SDHOST_CMD8_VHS              0x1         // 2.7-3.6V
/// Defines the value of "Check pattern" field in CMD8
#define SDHOST_CMD8_CHK_PATN         0xAA

/// Defines the value of "OCR" field in ACMD41
#define SDHOST_OCR_VDD_WIN           0xFF8000    // OCR bit[23:0]
/// Defines the buffer length for storing the returned data of CMD6, R2 response, etc.
#define SDHOST_C6R2_BUF_LEN          64          // buffer for CMD6, R2, etc.
/// Defines the size of one data block
#define SDHOST_ONE_BLK_LEN           512         // 512 Bytes

/// Defines the size of one data block
#define SD_BLK_LEN                   512         // 512 Bytes
/// Defines the size of SD status data structure
#define SD_STATUS_LEN                64          // 64 Bytes
/// Defines the length of CSD register
#define SD_CSD_LEN                   16          // 128 bits
/// Defines the byte size of SD response header (start + transmission + cmd_id)
#define SD_RESP_HDR_LEN              1

/// Defines the total number of phases when tuning sample clock
#define SDHOST_CLK_PHASE_CNT    4

#define SDHOST_INT_CARD_END         BIT1
#define SDHOST_INT_DMA_END          BIT4


/**
  \brief  Defines SDIO Host DMA transfer type.
*/
enum  sdhost_dma_type_e {
	SdHostDmaNormal  = 0,
	SdHostDma64b     = 1,
	SdHostDmaR2      = 2
};

/**
  \brief  Defines SDIO Host DMA operation.
*/
enum  sdhost_dma_op_e {
	SdHostDmaWrite   = 0,
	SdHostDmaRead    = 1
};

/**
  \brief  Defines SDIO Host clock source.
*/
enum  sdhost_clk_src_e {
	SdHostSscClk     = 0,
	SdHostSscClkVp0  = 1,
	SdHostSscClkVp1  = 2
};

/**
  \brief  Defines SDIO Host clock divider.
*/
enum  sdhost_clk_div_e {
	SdHostClkDiv1    = 0,
	SdHostClkDiv2    = 1,
	SdHostClkDiv4    = 2,
	SdHostClkDiv8    = 3
};

/**
  \brief  Defines SDCLK divider.
*/
enum  sdhost_sdclk_divider_e {
	SdHostSdclkDiv128    = 0,
	SdHostSdclkDiv256    = 1
};

/**
  \brief  Defines SDIO Host mode selection.
*/
enum  sdhost_mode_sel_e {
	SdHostModeSd20   = 0,
	SdHostModeDdr    = 1,
	SdHostModeSd30   = 2
};

/**
  \brief  Defines SDIO Host bus width.
*/
enum  sdhost_bus_width_e {
	SdHostBus1bit    = 0,
	SdHostBus4bit    = 1
};

/**
  \brief  Defines SDIO Host response type.
*/
enum  sdhost_rsp_type_e {
	SdHostNoRsp      = 0,
	SdHostRsp6Bytes  = 1,
	SdHostRsp17Bytes = 2
};

/**
  \brief  Defines SDIO bus status.
*/
enum  sdhost_bus_status_e {
	SdHostBusLow     = 0,
	SdHostBusHigh    = 1
};

/**
  \brief  Defines SDIO Host command code.
*/
enum  sdhost_cmd_code_e {
	SdHostCmdNormalWr        = 0,
	SdHostCmdAutoWr3         = 1,
	SdHostCmdAutoWr4         = 2,
	SdHostCmdAutoRd3         = 5,
	SdHostCmdAutoRd4         = 6,
	SdHostCmdSendCmdGetRsp   = 8,
	SdHostCmdAutoWr1         = 9,
	SdHostCmdAutoWr2         = 10,
	SdHostCmdNormalRd        = 12,
	SdHostCmdAutoRd1         = 13,
	SdHostCmdAutoRd2         = 14,
	SdHostCmdTuning          = 15
};

/**
  \brief  Defines SDIO Bus Signaling Level.
*/
enum  sdhost_sig_level_e {
	SdHostSigVol33           = 0,
	SdHostSigVol18           = 1
};

/**
  \brief  Defines the data present select in current transaction.
*/
enum  sdhost_data_present_e {
	SdHostNoDataPresent      = 0,
	SdHostDataPresent        = 1
};

/**
  \brief  Defines CMD6 operation mode.
*/
enum  sdhost_cmd6_mode_e {
	SdHostCmd6CheckMode      = 0,
	SdHostCmd6SwitchMode     = 1
};

/**
  \brief  Defines CMD6 function group 1 (access mode).
*/
enum  sdhost_access_mode_e {
	SdHostSpeedDS            = 0,  // 3.3V Function 0
	SdHostSpeedHS            = 1,  // 3.3V Function 1
	SdHostSpeedSDR12         = 2,  // 1.8V Function 0
	SdHostSpeedSDR25         = 3,  // 1.8V Function 1
	SdHostSpeedSDR50         = 4,  // 1.8V Function 2
	SdHostSpeedSDR104        = 5,  // 1.8V Function 3
	SdHostSpeedDDR50         = 6,  // 1.8V Function 4
	SdHostKeepCurSpd         = 15
};

/**
  \brief  Defines CMD6 function group 3 (driver strength).
*/
enum  sdhost_driver_strength_e {
	SdHostDriverTypeB        = 0,
	SdHostDriverTypeA        = 1,
	SdHostDriverTypeC        = 2,
	SdHostDriverTypeD        = 3
};

/**
  \brief  Defines CMD6 function group 4 (current limit).
*/
enum  sdhost_current_limit_e {
	SdHostCurLim200mA        = 0,
	SdHostCurLim400mA        = 1,
	SdHostCurLim600mA        = 2,
	SdHostCurLim800mA        = 3,
};

/**
  \brief  Defines CMD7 select/de-select card.
*/
enum  sdhost_card_selection_e {
	SdHostDeselCard          = 0,
	SdHostSelCard            = 1
};

/**
  \brief  Defines host capacity support (ACMD41).
*/
enum  sdhost_host_capacity_e {
	SdHostSupportSdscOnly    = 0,
	SdHostSupportSdhcSdxc    = 1
};

/**
  \brief  Defines SDXC power control (ACMD41).
*/
enum  sdhost_sdxc_pwr_ctl_e {
	SdHostPwrSaving          = 0,
	SdHostMaxPerformance     = 1
};

/**
  \brief  Defines switch to 1.8V request (ACMD41).
*/
enum  sdhost_switch_18v_req_e {
	SdHostUseCurrSigVol      = 0,
	SdHostSwitch18v          = 1
};

/**
  \brief  Defines SD card's current state (Card Status bit[12:9]).
*/
enum  sdhost_card_curr_ste_e {
	SdHostCardSteIdle        = 0,
	SdHostCardSteReady       = 1,
	SdHostCardSteIdent       = 2,
	SdHostCardSteStby        = 3,
	SdHostCardSteTran        = 4,
	SdHostCardSteData        = 5,
	SdHostCardSteRcv         = 6,
	SdHostCardStePrg         = 7,
	SdHostCardSteDis         = 8
};

/**
  \brief  Defines SD physical layer specification version.
*/
enum  sdhost_sd_spec_e {
	SdHostSdSpecV101         = 0,
	SdHostSdSpecV110         = 1,
	SdHostSdSpecV200         = 2,
	SdHostSdSpecV300         = 3
};

/**
  \brief  Defines SDIO command index.
*/
enum  sdhost_cmd_idx_e {
	SdHostCmdGoIdleSte           = 0,
	SdHostCmdAllSendCid          = 2,
	SdHostCmdSendRelAddr         = 3,
	SdHostCmdSetDsr              = 4,
	SdHostCmdSwitchFunc          = 6,
	SdHostCmdSetBusWidth         = 6,  // ACMD6
	SdHostCmdSelDeselCard        = 7,
	SdHostCmdSendIfCond          = 8,
	SdHostCmdSendCsd             = 9,
	SdHostCmdSendCid             = 10,
	SdHostCmdVolSwitch           = 11,
	SdHostCmdStopXsmission       = 12,
	SdHostCmdSendSts             = 13,
	SdHostCmdSetBlklen           = 16,
	SdHostCmdRdSingleBlk         = 17,
	SdHostCmdRdMulBlk            = 18,
	SdHostCmdSendTuningBlk       = 19,
	SdHostCmdSendNumWrBlks       = 22,  // ACMD22
	SdHostCmdSetBlkCnt           = 23,
	SdHostCmdSetWrBlkEraseCnt    = 23,  // ACMD23
	SdHostCmdWrBlk               = 24,
	SdHostCmdWrMulBlk            = 25,
	SdHostCmdProgCsd             = 27,
	SdHostCmdEraseBlkSt          = 32,
	SdHostCmdEraseBlkEd          = 33,
	SdHostCmdErase               = 38,
	SdHostCmdSdSendOpCond        = 41,  // ACMD41
	SdHostCmdSendScr             = 51,  // ACMD51
	SdHostCmdAppCmd              = 55
};


/**
  \brief  Callback function type without parameter.
*/
typedef void (*sdhost_cb_t)(void);

/**
  \brief  Callback function type with parameter.
*/
typedef void (*sdhost_para_cb_t)(void *para);


/**
  \brief  The structure of the settings for DMA control.
*/
typedef struct hal_sdhost_dma_ctrl_s {
	u32 start_addr;
	u16 blk_cnt;
	u8 op;
	u8 type;
} hal_sdhost_dma_ctrl_t, *phal_sdhost_dma_ctrl_t;

/**
  \brief  The structure of command attributes.
*/
typedef struct hal_sdhost_cmd_attr_s {
	u32 arg;
	u8 idx;
	u8 rsp_type;
	u8 rsp_crc_chk;
	u8 data_present;
} hal_sdhost_cmd_attr_t, *phal_sdhost_cmd_attr_t;

// size should be 64 bytes
typedef struct sd_status {
	u8 rsvd1[49];
	u32 uhs_au_size      : 4;
	u32 uhs_speed_grade  : 4;
	u32 erase_offset     : 2;
	u32 erase_timeout    : 6;
	u8 erase_size1;
	u8 erase_size3;
	u32 rsvd2            : 4;
	u32 au_size          : 4;
	u8 perf_move;
	u8 speed_class;
	u16 prot_area_size1;
	u16 prot_area_size2;
	u16 sd_card_type;
	u32 rsvd3           : 13;
	u32 secured_mode    : 1;
	u32 dat_bus_width   : 2;
} __attribute__((aligned(4))) sd_status_t;

// size should be 16 bytes
typedef struct sd_csd_v1 {
	u32 not_used_1      : 1;
	u32 crc             : 7;
	u32 rsvd1           : 2;
	u32 file_format     : 2;
	u32 tmp_write_prot  : 1;
	u32 perm_write_prot : 1;
	u32 copy            : 1;
	u32 file_format_grp : 1;
	u32 rsvd2           : 5;
	u32 write_bl_partial: 1;
	u32 write_bl_len    : 4;
	u32 r2w_factor      : 3;
	u32 rsvd3           : 2;
	u32 wp_grp_enable   : 1;
	u32 wp_grp_size     : 7;
	u32 sector_size     : 7;
	u32 erase_blk_en    : 1;
	u32 c_size_mult     : 3;
	u32 vdd_w_curr_max  : 3;
	u32 vdd_w_curr_min  : 3;
	u32 vdd_r_curr_max  : 3;
	u32 vdd_r_curr_min  : 3;
	u32 c_size_1        : 2;
	u32 c_size_2        : 10;
	u32 rsvd4           : 2;
	u32 dsr_imp         : 1;
	u32 rd_blk_misalign : 1;
	u32 wr_blk_misalign : 1;
	u32 read_bl_partial : 1;
	u32 read_bl_len     : 4;
	u32 ccc             : 12;
	u32 tran_speed      : 8;
	u32 nsac            : 8;
	u32 taac            : 8;
	u32 rsvd5           : 6;
	u32 csd_structure   : 2;
} __attribute__((aligned(4))) sd_csd_v1_t;

// size should be 16 bytes
typedef struct sd_csd_v2 {
	u32 not_used_1      : 1;
	u32 crc             : 7;
	u32 rsvd1           : 2;
	u32 file_format     : 2;
	u32 tmp_write_prot  : 1;
	u32 perm_write_prot : 1;
	u32 copy            : 1;
	u32 file_format_grp : 1;
	u32 rsvd2           : 5;
	u32 write_bl_partial: 1;
	u32 write_bl_len    : 4;
	u32 r2w_factor      : 3;
	u32 rsvd3           : 2;
	u32 wp_grp_enable   : 1;
	u32 wp_grp_size     : 7;
	u32 sector_size     : 7;
	u32 erase_blk_en    : 1;
	u32 rsvd4           : 1;
	u32 c_size_1        : 16;
	u32 c_size_2        : 6;
	u32 rsvd5           : 6;
	u32 dsr_imp         : 1;
	u32 rd_blk_misalign : 1;
	u32 wr_blk_misalign : 1;
	u32 read_bl_partial : 1;
	u32 read_bl_len     : 4;
	u32 ccc             : 12;
	u32 tran_speed      : 8;
	u32 nsac            : 8;
	u32 taac            : 8;
	u32 rsvd6           : 6;
	u32 csd_structure   : 2;
} __attribute__((aligned(4))) sd_csd_v2_t;

/**
  \brief  The data structure for SDIO host HAL operations.
*/
typedef struct hal_sdhost_adapter_s {
	SDHOST_Type *base_addr;
	volatile u8 is_card_inserted;
	volatile u8 is_wp;
	volatile u8 card_inited;
	u8 curr_sig_level;
	u8 is_sdhc_sdxc;
	u8 card_curr_ste;
	u8 sd_spec_ver;
	u8 curr_bus_spd;
	u8 card_support_spd_mode;
	u8 is_s18a;
	u8 voltage_mismatch;
	u8 force_33v;
	u8 wait_interrupts;
	u8 csd[SD_CSD_LEN];
	u8 rsvd[2];
	u16 rca;
	sdhost_para_cb_t card_insert_cb;
	sdhost_para_cb_t card_remove_cb;
	sdhost_para_cb_t transfer_done_cb;
	sdhost_para_cb_t task_yield_cb;
	void *card_insert_cb_para;
	void *card_remove_cb_para;
	void *transfer_done_cb_para;
	void *task_yield_cb_para;
	void (*dcache_invalidate_by_addr)(uint32_t *addr, int32_t dsize);   /*! callback function to do the D-cache invalidate  */
	void (*dcache_clean_by_addr)(uint32_t *addr, int32_t dsize);    /*! callback function to do the D-cache clean  */
	u32 xfer_int_sts;
	u8 c6r2_buf[SDHOST_C6R2_BUF_LEN] __attribute__((aligned(32)));
	u8 rsvd2[27];
} hal_sdhost_adapter_t, *phal_sdhost_adapter_t;

/**
  \brief  The data structure of the stubs function for the SDIO Host HAL functions in ROM
*/
typedef struct hal_sdhost_func_stubs_s {
	io_pin_t *sdhost_pin_table;
	void (*hal_sdhost_irq_handler)(void);
	void (*hal_sdhost_irq_reg)(irq_handler_t irq_handler);
	void (*hal_sdhost_irq_unreg)(void);
	hal_status_t (*hal_sdhost_init_host)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_init_card)(hal_sdhost_adapter_t *psdhost_adapter);
	void (*hal_sdhost_deinit)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_read_data)(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, u8 *rbuf_32align);
	hal_status_t (*hal_sdhost_write_data)(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, const u8 *wbuf_32align);
	hal_status_t (*hal_sdhost_erase)(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u64 end_addr);
	hal_status_t (*hal_sdhost_stop_transmission)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_get_card_status)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_get_sd_status)(hal_sdhost_adapter_t *psdhost_adapter, u8 *buf_32align);
	hal_status_t (*hal_sdhost_get_scr)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_switch_bus_speed)(hal_sdhost_adapter_t *psdhost_adapter, u8 speed);
	u8(*hal_sdhost_get_curr_signal_level)(hal_sdhost_adapter_t *psdhost_adapter);
	hal_status_t (*hal_sdhost_get_supported_speed)(hal_sdhost_adapter_t *psdhost_adapter, u8 *value);
	void (*hal_sdhost_card_insert_hook)(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata);
	void (*hal_sdhost_card_remove_hook)(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata);
	void (*hal_sdhost_task_yield_hook)(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t task_yield, void *pdata);
	void (*hal_sdhost_transfer_done_int_hook)(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t transfer_done_cb, void *pdata);
	uint32_t reserved[11];  // 32 bytes align for next ROM code version function table extending.
} hal_sdhost_func_stubs_t;


/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_sdhost__rom_func SDIO Host HAL ROM APIs.
 * @ingroup hs_hal_sdhost_
 * @{
 */

void hal_rtl_sdhost_irq_reg(irq_handler_t irq_handler);
void hal_rtl_sdhost_irq_unreg(void);
hal_status_t hal_rtl_sdhost_init_host(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_init_card(hal_sdhost_adapter_t *psdhost_adapter);
void hal_rtl_sdhost_deinit(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_read_data(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, u8 *rbuf_32align);
hal_status_t hal_rtl_sdhost_write_data(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u16 blk_cnt, const u8 *wbuf_32align);
hal_status_t hal_rtl_sdhost_erase(hal_sdhost_adapter_t *psdhost_adapter, u64 start_addr, u64 end_addr);
hal_status_t hal_rtl_sdhost_stop_transmission(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_get_card_status(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_get_sd_status(hal_sdhost_adapter_t *psdhost_adapter, u8 *buf_32align);
hal_status_t hal_rtl_sdhost_get_scr(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_switch_bus_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 speed);
u8 hal_rtl_sdhost_get_curr_signal_level(hal_sdhost_adapter_t *psdhost_adapter);
hal_status_t hal_rtl_sdhost_get_supported_speed(hal_sdhost_adapter_t *psdhost_adapter, u8 *value);
void hal_rtl_sdhost_card_insert_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata);
void hal_rtl_sdhost_card_remove_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t pcallback, void *pdata);
void hal_rtl_sdhost_task_yield_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t task_yield, void *pdata);
void hal_rtl_sdhost_transfer_done_int_hook(hal_sdhost_adapter_t *psdhost_adapter, sdhost_para_cb_t transfer_done_cb, void *pdata);

/** @} */ /* End of group hs_hal_sdhost__rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */
/** @} */ /* End of group hs_hal_sdhost_ */

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _RTL8735B_SDHOST_H_"


