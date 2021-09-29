/**************************************************************************//**
 * @file     rtl8735b_snand.c
 * @brief    Implement serial nand flash controller RAM code functions.
 * @version  1.10
 * @date     2021-06-28
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
#include <arm_cmse.h>
#include "rtl8735b_snand.h"

#if CONFIG_SNAND_FLASH_EN

#if !defined(CONFIG_BUILD_BOOT)

#if (defined(CONFIG_BUILD_RAM) && (CONFIG_BUILD_RAM == 1))
#include "utility.h"
#include "hal_cache.h"
#endif

#if (defined(CONFIG_BUILD_ALL) && (CONFIG_BUILD_ALL == 1)) || (defined(CONFIG_BUILD_ROM) && (CONFIG_BUILD_ROM == 1))
#define SECTION_SNAND_TEXT           SECTION(".rom.hal_snand.text")
#define SECTION_SNAND_DATA           SECTION(".rom.hal_snand.data")
#define SECTION_SNAND_RODATA         SECTION(".rom.hal_snand.rodata")
#define SECTION_SNAND_BSS            SECTION(".rom.hal_snand.bss")
#define SECTION_SNAND_STUBS          SECTION(".rom.hal_snand.stubs")
#else
#define SECTION_SNAND_TEXT
#define SECTION_SNAND_DATA
#define SECTION_SNAND_RODATA
#define SECTION_SNAND_BSS
#define SECTION_SNAND_STUBS
#endif


#define DBG_MSG_TESTCHIP (0)

/**

        \addtogroup hal_snand Serial NAND Flash Controller (SNAFC)
)
        @{
*/


/// @cond DOXYGEN_ROM_HAL_API

/**

        \addtogroup hal_rtl_snand_func Serial NAND Flash Controller HAL ROM APIs
        \ingroup hal_snand
        \brief Configure SPIC functions to control flash controller and communicate with a flash device.
               The user application(in RAM space) should not call these APIs directly.
               There is another set of Serial NAND Flash Controller HAL APIs in the RAM space is provided for the user application.
        @{
*/

/**
  \brief The stubs functions table to exports Serial NAND Flash Controller HAL functions in ROM.
*/
SECTION_SNAND_STUBS
const hal_snand_func_stubs_t hal_snand_stubs = {
	.hal_snand_init                 = hal_rtl_snand_init,
	.hal_snand_deinit               = hal_rtl_snand_deinit,
	.hal_snand_reset_to_spi         = hal_rtl_snand_issusResetOpCmd,
	.hal_snand_read_id              = hal_rtl_snand_issueReadIdOpCmd,
	.hal_snand_set_quad_enable      = hal_rtl_snand_quadModeEnable,
	.hal_snand_unset_quad_enable    = hal_rtl_snand_quadModeDisable,
	.hal_snand_set_feature          = hal_rtl_snand_issueSetFeatureRegisterOpCmd,
	.hal_snand_set_feature_no_check = hal_rtl_snand_issueSetFeatureRegisterOpCmd_NoCheck,
	.hal_snand_get_feature          = hal_rtl_snand_issueGetFeatureRegisterOpCmd,
	.hal_snand_wait_ready           = hal_rtl_snand_waitSnandOipComplete,
	.hal_snand_set_write_enable     = hal_rtl_snand_issueWriteEnableOpCmd,
	.hal_snand_set_write_disable    = hal_rtl_snand_issueWriteDisableOpCmd,
	.hal_snand_block_erase          = hal_rtl_snand_issueBlockEraseOpCmd,
	.hal_snand_page_program         = hal_rtl_snand_issueProgramExecuteOpCmd,
	.hal_snand_pageWrite            = hal_rtl_snand_pageWrite,
	.hal_snand_pageRead             = hal_rtl_snand_pageRead,
};

#define SNAFC_XCMR_LEN(val) ((val-1))
#define SNAFC_IO_WIDTH_LEN(io_width, w_len) (unsigned int)((io_width<<28)|w_len)

typedef uint32_t hal_rtl_snafc_bus_op_func(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, void *wr_buf);
hal_rtl_snafc_bus_op_func _hal_rtl_snafc_txCmdRaw, _hal_rtl_snafc_TxCmdAndWriteBuffer, _hal_rtl_snafc_TxCmdAndReadBuffer;
uint32_t _hal_rtl_snafc_tx_cmd(uint32_t opcode, hal_rtl_snafc_bus_op_func pSnandFunc, uint32_t data, uint32_t w_io_len, uint32_t r_io_len,
							   uint32_t byteLens,
							   void *wr_buf);
#define SNAFC_DONT_CARE (0xFFFFFFFF)

SECTION_SNAND_DATA
snand_bus_cfg_t mDefaultSnandCmdInfo = {
	.w_cmd_cycle = DEFAULT_W_CMD_CYCLE,
	.w_cmd = DEFAULT_W_CMD_OP,
	.w_addr_io = DEFAULT_W_CMD_IOWIDTH,
	.w_data_io = DEFAULT_W_DAT_IOWIDTH,
	.r_cmd = DEFAULT_R_CMD_OP,
	.r_cmd_cycle = DEFAULT_R_CMD_CYCLE,
	.r_addr_io = DEFAULT_R_CMD_IOWIDTH,
	.r_data_io = DEFAULT_R_DAT_IOWIDTH,
};

#if defined(CONFIG_ASIC)
#define MAX_SNAFC_CTRLR_RDY_CNT 0x4000
#define MAX_SNAFC_DMA_RDY_CNT 0x10000
#else /* !defined(CONFIG_ASIC) */
#define MAX_SNAFC_CTRLR_RDY_CNT 0x200
#define MAX_SNAFC_DMA_RDY_CNT 0x800
#endif /* !defined(CONFIG_ASIC) */

SECTION_SNAND_DATA static uint32_t mMaxSnafcCtrlRdyCnt;  /* = MAX_SNAFC_CTRLR_RDY_CNT */
SECTION_SNAND_DATA static uint32_t mMaxSnafcDmaRdyCnt; /* = MAX_SNAFC_DMA_RDY_CNT */

/**
 Control SNAFC's chip select signal
 */
#if 0
SECTION_SNAND_TEXT
static void
_hal_rtl_snand_DumpCmdInfo(
	snand_bus_cfg_t *pSnandCmdInfo
)
{
	if (!pSnandCmdInfo) {
		return;
	}
	switch (pSnandCmdInfo->w_data_io) {
	case SNAFC_SIO_WIDTH:
		dbg_printf("cmdW(1-1-1).");
		break;
	case SNAFC_DIO_WIDTH:
		dbg_printf("cmdW(1-1-2).");
		break;
	case SNAFC_QIO_WIDTH:
		dbg_printf("cmdW(1-1-4).");
		break;
	default:
		dbg_printf("ERR.cmdW(%d).", pSnandCmdInfo->w_data_io);
	}

	switch (pSnandCmdInfo->r_data_io) {
	case SNAFC_SIO_WIDTH:
		dbg_printf("cmdR(1-1-1).");
		break;
	case SNAFC_DIO_WIDTH:
		dbg_printf("cmdR(1-1-2).");
		break;
	case SNAFC_QIO_WIDTH:
		dbg_printf("cmdR(1-1-4).");
		break;
	default:
		dbg_printf("ERR.cmdR(%d).", pSnandCmdInfo->r_data_io);
	}
	dbg_printf("\r\n");
#if (defined(CONFIG_BUILD_ALL) && (CONFIG_BUILD_ALL == 1)) || (defined(CONFIG_BUILD_ROM) && (CONFIG_BUILD_ROM == 1))
	dump_for_one_bytes(pSnandCmdInfo, sizeof(snand_bus_cfg_t));   /* PhilipDebug */
#else
	dump_bytes(pSnandCmdInfo, sizeof(snand_bus_cfg_t));
#endif
	dbg_printf("\r\n");

	return;
} /* _hal_rtl_snand_DumpCmdInfo */
#endif

/**
 Check if the SPI NAND flash controller (SNAFC) completes it operation.
 */
SECTION_SNAND_TEXT
static uint32_t
_polling_snafc_ctrlr_ready(uint32_t timeout)
{
	SNAFC_TypeDef *snafcDev = SNAFC_S;
	uint32_t dmyCntr = 0;
	do {
		dmyCntr++;
	} while ((snafcDev->SNAFC_SR & SNAFC_BIT_NFCOS) && (dmyCntr < timeout));
	if (dmyCntr >= timeout) {
		return FAIL;
	} else {
		return SUCCESS;
	}
}

/**
 Check if the SPI NAND flash controller (SNAFC) DMA completes or not.
 */
SECTION_SNAND_TEXT
static uint32_t
_polling_snafc_dma_ready(uint32_t timeout)
{
	SNAFC_TypeDef *snafcDev = SNAFC_S;
	uint32_t dmyCntr = 0;
	do {
		dmyCntr++;
	} while ((0 != (snafcDev->SNAFC_SR & (SNAFC_BIT_NFDRS | SNAFC_BIT_NFDWS)))  && (dmyCntr < timeout));
	if (dmyCntr >= timeout) {
		return FAIL;
	} else {
		return SUCCESS;
	}
}

/**
 Control SNAFC's chip select signal
 */
SECTION_SNAND_TEXT
static void
_hal_rtl_snafc_chipSelect(
	uint32_t en
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	if (en == ENABLE) {
		snafc_s->SNAFC_CCR = 0x0; /* Set #CS Low (active) */
	} else {
		snafc_s->SNAFC_CCR = 0x1; /* Set #CS high (inactive) */
	}
} /* _hal_rtl_snafc_chipSelect */


/**
 Set SNAFC's #CS as LOW (active)
 */
SECTION_SNAND_TEXT
static void
_hal_rtl_snafc_csLow(
	void
)
{
	_hal_rtl_snafc_chipSelect(ENABLE);
} /* _hal_rtl_snafc_csLow */


/**
 Set SNAFC's #CS as HIGH (inactive)
 */
SECTION_SNAND_TEXT
static void
_hal_rtl_snafc_csHigh(
	void
)
{
	_hal_rtl_snafc_chipSelect(DISABLE);
} /* _hal_rtl_snafc_csHigh */


/**
 PIO Read After-Write (RAW) command
 This command can help to read back the SPI NAND flash register value through PIO operation.
 */
SECTION_SNAND_TEXT
uint32_t
_hal_rtl_snafc_txCmdRaw(
	uint32_t opcode,
	uint32_t data,
	uint32_t w_io_len,
	uint32_t r_io_len,
	uint32_t byteLens,
	void *wr_buf
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint32_t rdval = 0;
	uint32_t w_data = opcode;

	if (data != SNAFC_DONT_CARE) {
		uint32_t temp = (w_io_len & 0x3);
		if (0 != temp) {
			w_data = opcode | (data << ((3 - temp) * 8));
		}
	}

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	if (r_io_len != SNAFC_DONT_CARE) {
		snafc_s->SNAFC_RCMR = r_io_len; //Trigger PIO read /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		rdval = snafc_s->SNAFC_RDR; //Getting r_len-BYTE data @REG32(SNFRDR)
	}
	return rdval;
} /* _hal_rtl_snafc_txCmdRaw */


/**
 PIO Write command (cmd with buffer write)
 This function can help to send a command to SPI NAND flash through PIO operation.
 */
SECTION_SNAND_TEXT
uint32_t
_hal_rtl_snafc_TxCmdAndWriteBuffer(
	uint32_t opcode,
	uint32_t col_addr,
	uint32_t addr_io,
	uint32_t data_io,
	uint32_t byteLens,
	void *wr_buf
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint8_t tmp[4];
	uint32_t w_data;
	uint32_t w_io_len;
	uint8_t *pTmp = (uint8_t *) wr_buf;

	/* Command: SIO, 1-Byte */
	w_data = opcode;
	w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write

	/* Address: addr_io_len, 2-Byte */
	w_data = col_addr << 16;
	w_io_len = SNAFC_IO_WIDTH_LEN(addr_io, SNAFC_XCMR_LEN(2));
	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write

	/* Data: data_io_len, wr_len-Byte */
	w_io_len = SNAFC_IO_WIDTH_LEN(data_io, SNAFC_XCMR_LEN(1));
	while (byteLens > 0) {
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		if (byteLens < 4) {
			snafc_s->SNAFC_WCMR = (w_io_len | SNAFC_XCMR_LEN(byteLens)); /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
			byteLens = 0;
		} else {
			snafc_s->SNAFC_WCMR = (w_io_len | SNAFC_XCMR_LEN(4)); /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
			byteLens -= 4;
		}
		/* The last few bytes may be garbage but won't be written to flash, since its length is explicitly given */
		tmp[0] = *((uint8_t *)pTmp++);
		tmp[1] = *((uint8_t *)pTmp++);
		tmp[2] = *((uint8_t *)pTmp++);
		tmp[3] = *((uint8_t *)pTmp++);
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		snafc_s->SNAFC_WDR = *((unsigned int *)tmp); //Trigger PIO Write
	}
	return SUCCESS;
} /* _hal_rtl_snafc_TxCmdAndWriteBuffer */


/**
 PIO Read command (cmd with buffer read)
 This function can help to read back the SPI NAND flash register value through PIO operation.
 */
SECTION_SNAND_TEXT
uint32_t
_hal_rtl_snafc_TxCmdAndReadBuffer(
	uint32_t opcode,
	uint32_t col_addr,
	uint32_t addr_io,
	uint32_t data_io,
	uint32_t byteLens,
	void *wr_buf
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint32_t w_data;
	uint32_t w_io_len;
	uint32_t r_io_len;
	uint32_t wr_bound;
	uint32_t i;
	uint32_t dummy = 0x00;
	uint8_t tmp[4];
	uint8_t *pTmp = NULL;

	/* Command: SIO, 1-Byte */
	w_data = opcode;
	w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write

	/* Address: addr_io_len, 4-Byte */
#ifdef NAND_SPI_USE_QIO
	/*(Step4) Send 4-Byte Address for QIO , Fast read op(0xeb)need 4T's addr + 4T's dummy*/
	w_data = (col_addr << 24) | (dummy << 16);
	w_io_len = SNAFC_IO_WIDTH_LEN(addr_io, SNAFC_XCMR_LEN(4));

#else  /*(Step4) Send 3-Byte Address for SIO/DIO */
	w_data = (col_addr << 16) | (dummy << 8);
	w_io_len = SNAFC_IO_WIDTH_LEN(addr_io, SNAFC_XCMR_LEN(3));
#endif

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	/* Data: data_io_len, wr_len-Byte */
	r_io_len = SNAFC_IO_WIDTH_LEN(data_io, SNAFC_XCMR_LEN(1));
	wr_bound = (uint32_t)wr_buf + byteLens;

	if (wr_buf) {
		pTmp = (uint8_t *) wr_buf;
	}

	if (byteLens >= 4) {
		while ((unsigned int)pTmp < (wr_bound & 0xFFFFFFFC)) {
			snafc_s->SNAFC_RCMR = (r_io_len | SNAFC_XCMR_LEN(4)); /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
			_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
			*((uint32_t *)tmp) = snafc_s->SNAFC_RDR;
			*((uint8_t *)pTmp++) = tmp[0];
			*((uint8_t *)pTmp++) = tmp[1];
			*((uint8_t *)pTmp++) = tmp[2];
			*((uint8_t *)pTmp++) = tmp[3];
		}
	}
	for (i = 0; i < (byteLens & 0x3); i++) {
		snafc_s->SNAFC_RCMR = (r_io_len); /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		*((uint8_t *)pTmp++) = (snafc_s->SNAFC_RDR >> 24);
	}
	return SUCCESS;
} /* _hal_rtl_snafc_TxCmdAndReadBuffer */


/**
 PIO command wrapper for write/read-after-write/read behavior.
*/
SECTION_SNAND_TEXT
uint32_t
_hal_rtl_snafc_tx_cmd(
	uint32_t opcode,
	hal_rtl_snafc_bus_op_func pSnandFunc,
	uint32_t data,
	uint32_t w_io_len,
	uint32_t r_io_len,
	uint32_t byteLens,
	void *pBuf
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint32_t ret = 0x0;

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	_hal_rtl_snafc_csHigh(); /* deactivate CS */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	_hal_rtl_snafc_csLow();  /* activate CS */

	ret = pSnandFunc((opcode << 24), data, w_io_len, r_io_len, byteLens,
					 pBuf); /* _hal_rtl_snafc_txCmdRaw, _hal_rtl_snafc_TxCmdAndWriteBuffer, _hal_rtl_snafc_TxCmdAndReadBuffer */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
	_hal_rtl_snafc_csHigh(); /* deactivate CS */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	return ret;
} /* _hal_rtl_snafc_tx_cmd */


/**
 Enabling SNAFC, and its LX bus, and switch pinmux to Serial NAND FLASH (SNAFC)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snand_init(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	SYSON_S_TypeDef *syson_s = SYSON_S;
	uint32_t val;
	uint32_t tClkSel = 0;
	uint32_t tClkSrcSel = 0, tBakClkSrcSel = 0;

	mMaxSnafcCtrlRdyCnt = MAX_SNAFC_CTRLR_RDY_CNT;
	mMaxSnafcDmaRdyCnt = MAX_SNAFC_DMA_RDY_CNT;

	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		tClkSel = pSnafcAdaptor->clkSel;

		if (tClkSel >= SNAFC_INITVAL_SETS) {
			tClkSel = SNAFC_SPEED_SEL_0;
		}

		/*
		SYSON_S_REG_SYS_FLASH_CTRL
		    [6], R/W, 0, BIT_NAND_FLASH_CLK_EN, 1: Enable NAND FLASH CLK
		    [5], R/W, 0, BIT_NAND_FLASH_EN,     1: Enable NAND FLASH
		    [4], R, 0, BIT_HS_FLASH_INIT_OK,    BOOT_finish
		    [2], R/W, 0, BIT_SYS_FLASH_SRC_SEL, 0: 250MHz, 1:200MHz
		    [1], R/W, 0, BIT_NOR_FLASH_CLK_EN,  1: Enable NOR FLASH CLK
		    [0], R/W, 0, BIT_NOR_FLASH_EN,      1: Enable NOR FLASH
		*/
		if (tClkSel & 0x1) {
			tClkSrcSel = 0;
		} else {
			tClkSrcSel = 1;
		}
		if (pSnafcAdaptor->maxCtrlRdyCnt) {
			mMaxSnafcCtrlRdyCnt = pSnafcAdaptor->maxCtrlRdyCnt;
		}
		if (pSnafcAdaptor->maxDmaRdyCnt) {
			mMaxSnafcDmaRdyCnt = pSnafcAdaptor->maxDmaRdyCnt;
		}
	} else {
		tClkSrcSel = 0;
	}

	/* 0x6 to 0x5000088C [7:4] for Enabling NAND flash */
	val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
	tBakClkSrcSel = (val & SYSON_S_BIT_SYS_FLASH_SRC_SEL); /* keep [2] BIT_SYS_FLASH_SRC_SEL */
	val |= (SYSON_S_BIT_NAND_FLASH_CLK_EN | SYSON_S_BIT_NAND_FLASH_EN | ((tClkSrcSel == 1) ? SYSON_S_BIT_SYS_FLASH_SRC_SEL : 0));
	syson_s->SYSON_S_REG_SYS_FLASH_CTRL = val;

	/* 0x3 to 0x5000089C [17:16] for Enabling LX bus */
	val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
	val |= (SYSON_S_BIT_LXBUS_CLK_EN | SYSON_S_BIT_LXBUS_EN);
	syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;

	/* 0x1 to 0x50000890 [7] for Selecting NAND flash */
	val = syson_s->SYSON_S_REG_SYS_FLASH_PHY_CTRL0;
	val |= SYSON_S_BIT_FLASH_PHY_MUX_SEL;
	syson_s->SYSON_S_REG_SYS_FLASH_PHY_CTRL0 = val;

	if (pAdaptor) {
		snafc_s->SNAFC_CFR = pSnafcAdaptor->initVal[tClkSel].l;
		pSnafcAdaptor->bakClkSrcSel = (uint8_t) tBakClkSrcSel;
	} else {
		val = snafc_s->SNAFC_CFR;
		DBG_SNAND_INFO("[0x%08x] = 0x%08x. (snafc_s->SNAFC_CFR)++\r\n", (void *) & (snafc_s->SNAFC_CFR), val);
		/**
		    [31:31] 0: disable dummy read after DMA write transaction
		    [28:28] 0: Not boot from spi nand flash
		    [26:24] DEBUG_SELECT
		    [22:22] NAND Flash Read Byte Ordering.
		    [21:21] NAND Flash Write Byte Ordering.
		    [20:20] NAND flash controller DMA interrupt enable.
		    [14:14] 0: Slave data is the same order with LX slave bus
		    [13:13] 0: DMA data is the same order with LX master bus
		    [12:12] 1: LX bus DMA precise. 0: LX bus DMA imprecise.
		    [9:8] set the data latch pipe latency = 0; (Power-on default is 2b'00 (data latch pipe latency=0))
		    [6:4] set the SPI operating clock rate = DIV2; (Power-on default is 3b'111 (div16))
		    [1:0] set LBC burst size to 2b'11 (128 bytes); (Power-on default is 2b'11 (128 bytes))
		        2b'00 (16 bytes)
		        2b'01 (32 bytes)
		        2b'10 (64 bytes)
		        2b'11 (128 bytes)(Power-on default)
		*/
		val &= ~(SNAFC_BIT_DMA_ENDIAN);
		//val &= ~(SNAFC_BIT_PRECISE);
		val &= ~(SNAFC_MASK_PIPE_LAT);
		val &= ~(SNAFC_MASK_SPI_CLK_DIV);
		val &= ~(SNAFC_MASK_LBC_BSZ);
		val |= (3 << SNAFC_SHIFT_LBC_BSZ);

#if 1 /* input clk for SNAFC 200MHz, readid shift 1-bit when spi_clk=100MHz*/
#if defined(CONFIG_ASIC)
#define DEFAULT_SPI_CLK_DIV     (0)
#define DEFAULT_PIPE_LAT        (1)
#else /* !defined(CONFIG_ASIC) */
#define DEFAULT_SPI_CLK_DIV     (0)
#define DEFAULT_PIPE_LAT        (0)
#endif /* !defined(CONFIG_ASIC) */

		val |= ((DEFAULT_SPI_CLK_DIV << SNAFC_SHIFT_SPI_CLK_DIV) & SNAFC_MASK_SPI_CLK_DIV);
		val |= ((DEFAULT_PIPE_LAT << SNAFC_SHIFT_PIPE_LAT) & SNAFC_MASK_PIPE_LAT);
#endif /* input clk for SNAFC 200MHz, readid shift 1-bit when spi_clk=100MHz*/

		snafc_s->SNAFC_CFR = val;
		DBG_SNAND_INFO("[0x%08x] = 0x%08x. (snafc_s->SNAFC_CFR=0x%08x)--\r\n", (void *) & (snafc_s->SNAFC_CFR), val, snafc_s->SNAFC_CFR);
	}
	return;
} /* hal_rtl_snand_init */


/**
 Disabling SNAFC, and its LX bus, and switch pinmux to SPI NOR FLASH (SPIC)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snand_deinit(
	void *pAdaptor
)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	uint32_t val;
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	uint32_t tBakClkSrcSel = 0;

	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		tBakClkSrcSel = (uint32_t) pSnafcAdaptor->bakClkSrcSel;
	} else {
		tBakClkSrcSel = 0;
	}

	/* 0x0 to 0x50000890 [7] for Selecting SPI NOR flash */
	val = syson_s->SYSON_S_REG_SYS_FLASH_PHY_CTRL0;
	val &= ~SYSON_S_BIT_FLASH_PHY_MUX_SEL;
	syson_s->SYSON_S_REG_SYS_FLASH_PHY_CTRL0 = val;

	/* 0x0 to 0x5000089C [17:16] for Enabling LX bus */
	val = syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0;
	val &= ~(SYSON_S_BIT_LXBUS_CLK_EN | SYSON_S_BIT_LXBUS_EN);
	syson_s->SYSON_S_REG_SYS_PLATFORM_CTRL0 = val;

	/* 0x6 to 0x5000088C [7:4] for Enabling NAND flash */
	val = syson_s->SYSON_S_REG_SYS_FLASH_CTRL;
	val &= ~(SYSON_S_BIT_NAND_FLASH_CLK_EN | SYSON_S_BIT_NAND_FLASH_EN | SYSON_S_BIT_SYS_FLASH_SRC_SEL);
	val |= tBakClkSrcSel;
	syson_s->SYSON_S_REG_SYS_FLASH_CTRL = val;

	return;
} /* hal_rtl_snand_deinit */


/**
 Send RESET_OP command to bus interface
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snfc_TxResetOpCmd(
	void *pAdaptor
)
{
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	uint32_t ret = _hal_rtl_snafc_tx_cmd(SNAND_RESET_OP, _hal_rtl_snafc_txCmdRaw, SNAFC_DONT_CARE, w_io_len, SNAFC_DONT_CARE, 0, 0);
	return ((ret >> 8) & 0xFFFFFF);
} /* hal_rtl_snfc_TxResetOpCmd */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_issusResetOpCmd(
	void *pAdaptor
)
{
	return hal_rtl_snfc_TxResetOpCmd(pAdaptor);
} /* hal_rtl_snand_issusResetOpCmd */


/**
 Send READ_ID_OP command to bus interface, and return flash vendor information code
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snafc_TxReadIdOpCmd(
	void *pAdaptor
)
{
	// write length = 2byte
	uint32_t w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(2));
	// read length = 3byte
	uint32_t r_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(3));

	uint32_t ret = _hal_rtl_snafc_tx_cmd(SNAND_RDID_OP, _hal_rtl_snafc_txCmdRaw, SNAFC_DONT_CARE, w_io_len, r_io_len, 0, 0);
	DBG_SNAND_INFO("\r\nspi_nand_id =0x%x\r\n", ((ret >> 8) & 0xFFFFFF));
	return ((ret >> 8) & 0xFFFFFF);
} /* hal_rtl_snafc_TxReadIdOpCmd */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_issueReadIdOpCmd(
	void *pAdaptor
)
{
	return hal_rtl_snafc_TxReadIdOpCmd(pAdaptor);
} /* hal_rtl_snand_issueReadIdOpCmd */


/**
 Send DATA IO for QUAD IO (QIO)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snand_quadModeEnable(
	void *pAdaptor
)
{
	/* FIXME ToDo */
} /* hal_rtl_snand_quadModeEnable */


/**
 Send DATA IO for SINGLE IO (SIO)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snand_quadModeDisable(
	void *pAdaptor
)
{
	/* FIXME ToDo */
} /* hal_rtl_snand_quadModeDisable */


/**
 Send SET_FEATURE_OP command and data to bus interface.
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_TxSetFeatureRegisterOpCmd(
	void *pAdaptor,
	uint32_t feature_addr,
	uint32_t value
)
{
	/* Set configuration (block protect, OTP feature) register. */
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(3));
	_hal_rtl_snafc_tx_cmd(SNAND_SET_FEATURE_OP, _hal_rtl_snafc_txCmdRaw, ((feature_addr << 8) | value), w_io_len, SNAFC_DONT_CARE, 0, 0);
	return;
} /* hal_rtl_snafc_TxSetFeatureRegisterOpCmd */

SECTION_SNAND_TEXT
void
hal_rtl_snand_issueSetFeatureRegisterOpCmd(
	void *pAdaptor,
	uint32_t feature_addr,
	uint32_t value
)
{
	hal_rtl_snafc_TxSetFeatureRegisterOpCmd(pAdaptor, feature_addr, value);
	return;
} /* hal_rtl_snand_issueSetFeatureRegisterOpCmd */


/**
 Send SET_FEATURE_OP command and data to bus interface.
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_TxSetFeatureRegisterOpCmd_NoCheck(
	void *pAdaptor,
	uint32_t feature_addr,
	uint32_t value
)
{
	/* Set configuration (block protect, OTP feature) register. */
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(3));
	_hal_rtl_snafc_tx_cmd(SNAND_SET_FEATURE_OP, _hal_rtl_snafc_txCmdRaw, ((feature_addr << 8) | value), w_io_len, SNAFC_DONT_CARE, 0, 0);
	return;
} /* hal_rtl_snafc_TxSetFeatureRegisterOpCmd_NoCheck */

SECTION_SNAND_TEXT
void
hal_rtl_snand_issueSetFeatureRegisterOpCmd_NoCheck(
	void *pAdaptor,
	uint32_t feature_addr,
	uint32_t value
)
{
	hal_rtl_snafc_TxSetFeatureRegisterOpCmd_NoCheck(pAdaptor, feature_addr, value);
	return;
} /* hal_rtl_snand_issueSetFeatureRegisterOpCmd_NoCheck */


/**
 Send GET_FEATURE_OP command to bus interface, and return flash feature.
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snafc_TxGetFeatureRegisterOpCmd(
	void *pAdaptor,
	uint32_t feature_addr
)
{
	/* Get configuration (block protect, OTP feature) register and status (op_in_progress, write_enable_latch, erase_fail, program_fail, ecc_sts) register. */
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(2));
	unsigned int r_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	unsigned int ret = _hal_rtl_snafc_tx_cmd(SNAND_GET_FEATURE_OP, _hal_rtl_snafc_txCmdRaw, feature_addr, w_io_len, r_io_len, 0, 0);
	return ((ret >> 24) & 0xFF);
} /* hal_rtl_snafc_TxGetFeatureRegisterOpCmd */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_issueGetFeatureRegisterOpCmd(
	void *pAdaptor,
	uint32_t feature_addr
)
{
	return hal_rtl_snafc_TxGetFeatureRegisterOpCmd(pAdaptor, feature_addr);;
} /* hal_rtl_snand_issueGetFeatureRegisterOpCmd */


/**
 Wait SPI NAND flash ready (MX35LF1GE4AB-Z4I, MX35LF2GE4AB-MI)
 */
SECTION_SNAND_TEXT
static uint32_t
_snand_MXIC_waitSnandOipComplete(
	void *pAdaptor
)
{
	/*
	    MX35LF2GE4AB_MI Feature Register definition:
	    * reg[0xC0] = 0x20
	        [5:4] ECC_Sts=10 (00b=0 bit error / 01b= 1~4 bits error corrected / 10b=More than 4 bit error not corrected / 11b=Reserved)
	        [3]P_Fail=0 (Program Fail=0)
	        [2]E_Fail=0 (Erase Fail=0)
	        [1]WEL=0 (Write Enable Latch=0)
	        [0]OIP=0 (Op_In_progress=0)

	    <<<Warning>>>: Sometimes, bitmap different between each Flash chip part-number.
	*/
	unsigned int feature_addr = 0xC0;
	unsigned int oip = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr); /* [0] OIP = Op_In_Progress */
	unsigned int swTo = 0;

#if defined(CONFIG_ASIC)
#define MAX_SNAND_OIP_MSK_CNT 0x800
#define MAX_SNAND_OIP_TIMEOUT_CNT 0x4000
#else /* !defined(CONFIG_ASIC) */
#define MAX_SNAND_OIP_MSK_CNT 0x200
#define MAX_SNAND_OIP_TIMEOUT_CNT 0x1000
#endif /* !defined(CONFIG_ASIC) */

	while ((oip & 0x1) != 0) {
		oip = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr);
		swTo++;
		if ((swTo & MAX_SNAND_OIP_MSK_CNT) == MAX_SNAND_OIP_MSK_CNT) {
			DBG_SNAND_ERR("%s Ln %u. oip=0x%0x\r\n", __FILE__, __LINE__, oip);
		}
		if (swTo > MAX_SNAND_OIP_TIMEOUT_CNT) {
			break;
		}
	}
	if (swTo > MAX_SNAND_OIP_TIMEOUT_CNT) {
		DBG_SNAND_ERR("%s Ln %u. swTo. oip=0x%0x\r\n", __FILE__, __LINE__, oip);
		return FAIL;
	}
	return SUCCESS;

} /* _snand_MXIC_waitSnandOipComplete */


/**
 Wait SPI NAND flash ready
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_waitSnandOipComplete(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		if (pSnafcAdaptor->funcWaitSnandReady) {
			return pSnafcAdaptor->funcWaitSnandReady(pSnafcAdaptor->maxWaitOipCnt);
		}
	}
	return _snand_MXIC_waitSnandOipComplete(pAdaptor);
} /* hal_rtl_snand_waitSnandOipComplet */


/**
 Check WEL (Write-Enable-Latched) Status
 */
SECTION_SNAND_TEXT
static uint32_t
_snand_MXIC_chkWelStsAfterWriteEnableOp(
	void *pAdaptor
)
{
	/*
	    MX35LF2GE4AB_MI Feature Register definition:
	    * reg[0xC0]
	        [5:4] ECC_Sts=10 (00b=0 bit error / 01b= 1~4 bits error corrected / 10b=More than 4 bit error not corrected / 11b=Reserved)
	        [3]P_Fail=0 (Program Fail=0)
	        [2]E_Fail=0 (Erase Fail=0)
	        [1]WEL=0 (Write Enable Latch=0)
	        [0]OIP=0 (Op_In_progress=0)

	    <<<Warning>>>: Sometimes, bitmap different between each Flash chip part-number.
	*/
	uint32_t feature_addr = 0xC0;
	uint32_t regValue = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr);
	uint32_t wel = ((regValue >> 1) & 0x1);
	return wel;
} /* _snand_MXIC_chkWelStsAfterWriteEnableOp */


/**
 Check Erase Status (Erase Fail, or SUCCESS)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_chkWelStsAfterWriteEnableOp(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		if (pSnafcAdaptor->funcChkWelSts) {
			return pSnafcAdaptor->funcChkWelSts();
		}
	}
	return _snand_MXIC_chkWelStsAfterWriteEnableOp(pAdaptor);
} /* hal_rtl_snand_chkWelStsAfterWriteEnableOp */


/**
 Check Erase Status (Erase Fail, or SUCCESS) (MX35LF1GE4AB-Z4I, MX35LF2GE4AB-MI)
 */
SECTION_SNAND_TEXT
static uint32_t
_snand_MXIC_chkEraseStsAfterEraseOp(
	void *pAdaptor
)
{
	/*
	    MX35LF2GE4AB_MI Feature Register definition:
	    * reg[0xC0]
	        [5:4] ECC_Sts=10 (00b=0 bit error / 01b= 1~4 bits error corrected / 10b=More than 4 bit error not corrected / 11b=Reserved)
	        [3]P_Fail=0 (Program Fail=0)
	        [2]E_Fail=0 (Erase Fail=0)
	        [1]WEL=0 (Write Enable Latch=0)
	        [0]OIP=0 (Op_In_progress=0)

	    <<<Warning>>>: Sometimes, bitmap different between each Flash chip part-number.
	*/
	uint32_t feature_addr = 0xC0;
	uint32_t regValue = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr);
	uint32_t efail = ((regValue >> 2) & 0x1);

	/*
	 * Check Program Status
	 */
	if (1 == efail) {
		DBG_SNAND_ERR("Program Fail\n");
		return FAIL;
	} else {
		return SUCCESS;
	}
} /* _snand_MXIC_chkEraseStsAfterEraseOp */


/**
 Check Erase Status (Erase Fail, or SUCCESS)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_chkEraseStsAfterEraseOp(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		if (pSnafcAdaptor->funcChkEraseSts) {
			return pSnafcAdaptor->funcChkEraseSts();
		}
	}
	return _snand_MXIC_chkEraseStsAfterEraseOp(pAdaptor);
} /* hal_rtl_snand_chkEraseStsAfterEraseOp */


/**
 Check Program Status (Program Fail, or SUCCESS) (MX35LF1GE4AB-Z4I, MX35LF2GE4AB-MI)
 */
SECTION_SNAND_TEXT
static uint32_t
_snand_MXIC_chkPgmStsAfterWriteOp(
	void *pAdaptor
)
{
	/*
	    MX35LF2GE4AB_MI Feature Register definition:
	    * reg[0xC0]
	        [5:4] ECC_Sts=10 (00b=0 bit error / 01b= 1~4 bits error corrected / 10b=More than 4 bit error not corrected / 11b=Reserved)
	        [3]P_Fail=0 (Program Fail=0)
	        [2]E_Fail=0 (Erase Fail=0)
	        [1]WEL=0 (Write Enable Latch=0)
	        [0]OIP=0 (Op_In_progress=0)

	    <<<Warning>>>: Sometimes, bitmap different between each Flash chip part-number.
	*/
	uint32_t feature_addr = 0xC0;
	uint32_t regValue = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr);
	uint32_t pfail = ((regValue >> 3) & 0x1);

	/*
	 * Check Program Status
	 */
	if (1 == pfail) {
		DBG_SNAND_ERR("Program Fail\n");
		return FAIL;
	} else {
		return SUCCESS;
	}
} /* _snand_MXIC_chkPgmStsAfterWriteOp */


/**
 Check Program Status (Program Fail, or SUCCESS))
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_chkPgmStsAfterWriteOp(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		if (pSnafcAdaptor->funcChkPgmSts) {
			return pSnafcAdaptor->funcChkPgmSts();
		}
	}
	return _snand_MXIC_chkPgmStsAfterWriteOp(pAdaptor);
} /* hal_rtl_snand_chkPgmStsAfterWriteOp */


/**
 Check ECC Status (Success (No err) / bit err but FIXED / bit err NOT FIX) (MX35LF1GE4AB-Z4I, MX35LF2GE4AB-MI)
 */
SECTION_SNAND_TEXT
static uint32_t
_snand_MXIC_chkEccStsAfterReadOp(
	void *pAdaptor
)
{
	/*
	    MX35LF2GE4AB_MI Feature Register definition:
	    * reg[0xC0]
	        [5:4] ECC_Sts=10 (00b=0 bit error / 01b= 1~4 bits error corrected / 10b=More than 4 bit error not corrected / 11b=Reserved)
	        [3]P_Fail=0 (Program Fail=0)
	        [2]E_Fail=0 (Erase Fail=0)
	        [1]WEL=0 (Write Enable Latch=0)
	        [0]OIP=0 (Op_In_progress=0)

	    <<<Warning>>>: Sometimes, bitmap different between each Flash chip part-number.
	*/
	uint32_t feature_addr = 0xC0;
	uint32_t regValue = hal_rtl_snand_issueGetFeatureRegisterOpCmd(pAdaptor, feature_addr);
	uint32_t eccSts = ((regValue >> 4) & 0x3);

	/*
	 * Check ECC Status
	 */
	if (1 == eccSts) {
		DBG_SNAND_INFO("BIT Err, ALREADY FIX\r\n");
	} else if (2 == eccSts) {
		DBG_SNAND_ERR("ECC Err, CANNOT FIX\r\n");
	}
	return eccSts;
} /* _snand_MXIC_chkEccStsAfterReadOp */


/**
 Check ECC Status (Success (No err) / bit err but FIXED / bit err NOT FIX)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_chkEccStsAfterReadOp(
	void *pAdaptor
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
		if (pSnafcAdaptor->funcChkEccSts) {
			return pSnafcAdaptor->funcChkEccSts();
		}
	}
	return _snand_MXIC_chkEccStsAfterReadOp(pAdaptor);

} /* hal_rtl_snand_chkEccStsAfterReadOp */


/**
 Send WRITE_ENABLE_OP command to bus interface
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_TxWriteEnableOpCmd(
	void *pAdaptor
)
{
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	_hal_rtl_snafc_tx_cmd(SNAND_WRITE_ENABLE_OP, _hal_rtl_snafc_txCmdRaw, SNAFC_DONT_CARE, w_io_len, SNAFC_DONT_CARE, 0, 0);
} /* hal_rtl_snafc_TxWriteEnableOpCmd */

SECTION_SNAND_TEXT
void
hal_rtl_snand_issueWriteEnableOpCmd(
	void *pAdaptor
)
{
	hal_rtl_snafc_TxWriteEnableOpCmd(pAdaptor);
} /* hal_rtl_snand_issueWriteEnableOpCmd */


/**
 Send WRITE_DISABLE_OP command to bus interface
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_TxWriteDisableOpCmd(
	void *pAdaptor
)
{
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	_hal_rtl_snafc_tx_cmd(SNAND_WRITE_DISABLE_OP, _hal_rtl_snafc_txCmdRaw, SNAFC_DONT_CARE, w_io_len, SNAFC_DONT_CARE, 0, 0);
} /* hal_rtl_snafc_TxWriteDisableOpCmd */

SECTION_SNAND_TEXT
void
hal_rtl_snand_issueWriteDisableOpCmd(
	void *pAdaptor
)
{
	hal_rtl_snafc_TxWriteDisableOpCmd(pAdaptor);
} /* hal_rtl_snand_issueWriteDisableOpCmd */


/**
 Erase block within SPI NAND flash
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snafc_TxBlockEraseOpCmd(
	void *pAdaptor,
	uint32_t blkPageAddr
)
{
	hal_rtl_snand_issueWriteEnableOpCmd(pAdaptor);
	/*
	 * 1-BYTE CMD + 1-BYTE Dummy + 2-BYTE Address
	 */
	uint32_t w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(4));
	_hal_rtl_snafc_tx_cmd(SNAND_BLOCK_ERASE_OP, _hal_rtl_snafc_txCmdRaw, blkPageAddr, w_io_len, SNAFC_DONT_CARE, 0, 0);
	return SUCCESS;
} /* hal_rtl_snafc_TxBlockEraseOpCmd */

/**
 Erase block within SPI NAND flash
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_issueBlockEraseOpCmd(
	void *pAdaptor,
	uint32_t blkPageAddr
)
{
	uint32_t retVal;
	hal_rtl_snafc_TxBlockEraseOpCmd(pAdaptor, blkPageAddr);
#if 1 /* FIXME ToDo: install callback */
	/*
	 * Wait S-NAND device complete
	 */
	retVal = hal_rtl_snand_waitSnandOipComplete(pAdaptor);
#endif /* FIXME ToDo: install callback */
#if 1 /* FIXME ToDo: install callback */
	/*
	 * Check Erase Status
	 */
	retVal |= hal_rtl_snand_chkEraseStsAfterEraseOp(pAdaptor);
#endif /* FIXME ToDo: install callback */
	return retVal;
} /* hal_rtl_snand_issueBlockEraseOpCmd */


/**
 Send PROGRAM_EXECUTE_OP command to bus interface, and polling flash status for ready.
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snafc_TxProgramExecuteOpCmd(
	void *pAdaptor,
	uint32_t blkPageAddr
)
{
	/* 1-BYTE CMD + 1-BYTE Dummy + 2-BYTE Address */
	unsigned int w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(4));
	_hal_rtl_snafc_tx_cmd(SNAND_PROGRAM_EXECUTE_OP, _hal_rtl_snafc_txCmdRaw, blkPageAddr, w_io_len, SNAFC_DONT_CARE, 0, 0);
	return SUCCESS;
} /* hal_rtl_snafc_TxProgramExecuteOpCmd */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_issueProgramExecuteOpCmd(
	void *pAdaptor,
	uint32_t blkPageAddr
)
{
	uint32_t retVal = hal_rtl_snafc_TxProgramExecuteOpCmd(pAdaptor, blkPageAddr);
#if 1 /* FIXME ToDo: install callback */
	/*
	 * Wait S-NAND device complete
	 */
	retVal |= hal_rtl_snand_waitSnandOipComplete(pAdaptor);
#endif /* FIXME ToDo: install callback */
#if 1 /* FIXME ToDo: install callback */
	retVal |= hal_rtl_snand_chkPgmStsAfterWriteOp(pAdaptor);
#endif /* FIXME ToDo: install callback */
	return retVal;
} /* hal_rtl_snand_issueProgramExecuteOpCmd */


/*
 * PIO Write one Chunk (2112-Byte)
 * Start from the assigned cache register address (CA=col_addr)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snafc_pioPageWrite(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	uint32_t col_addr = 0;
	uint32_t retVal = SUCCESS;

	if (pAdaptor) {
		pSnafcAdaptor = (hal_snafc_adaptor_t *)pAdaptor;
	}
	hal_rtl_snand_issueWriteEnableOpCmd(pAdaptor);

	if (pAdaptor) {
		col_addr = pSnafcAdaptor->col_addr;
		if ((0 != pSnafcAdaptor->snand_cmd_info.w_cmd) && (mDefaultSnandCmdInfo.w_cmd != pSnafcAdaptor->snand_cmd_info.w_cmd)) {
			mDefaultSnandCmdInfo.w_cmd = pSnafcAdaptor->snand_cmd_info.w_cmd;
			mDefaultSnandCmdInfo.w_addr_io = pSnafcAdaptor->snand_cmd_info.w_addr_io;
			mDefaultSnandCmdInfo.w_data_io = pSnafcAdaptor->snand_cmd_info.w_data_io;
		}
	} else {
		col_addr = 0;
	}

	_hal_rtl_snafc_tx_cmd(mDefaultSnandCmdInfo.w_cmd, _hal_rtl_snafc_TxCmdAndWriteBuffer, col_addr, mDefaultSnandCmdInfo.w_addr_io, mDefaultSnandCmdInfo.w_data_io,
						  byteLens, memAddr);
	retVal = hal_rtl_snand_issueProgramExecuteOpCmd(pAdaptor, blkPageAddr);
	hal_rtl_snand_issueWriteDisableOpCmd(pAdaptor);
	return retVal;
} /* hal_rtl_snafc_pioPageWrite */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_pioPageWrite(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	return hal_rtl_snafc_pioPageWrite(pAdaptor, memAddr, byteLens, blkPageAddr);
} /* hal_rtl_snand_pioPageWrite */

/*
 * PIO Write Less than One Chunk (Less than 2112-Byte)
 * Start from the assigned cache register address (CA=col_addr)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_TxPageDataReadToCacheBufOpCmd(
	uint32_t blkPageAddr
)
{
	/* 1-BYTE CMD + 1-BYTE Dummy + 2-BYTE Address */
	uint32_t w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(4));
	_hal_rtl_snafc_tx_cmd(SNAND_PAGE_DATA_READ_OP, _hal_rtl_snafc_txCmdRaw, blkPageAddr, w_io_len, SNAFC_DONT_CARE, 0, 0);

} /* hal_rtl_snafc_TxPageDataReadToCacheBufOpCmd */


SECTION_SNAND_TEXT
void
hal_rtl_snand_issuePageDataReadToCacheBufOpCmd(
	uint32_t blkPageAddr
)
{
	hal_rtl_snafc_TxPageDataReadToCacheBufOpCmd(blkPageAddr);
} /* hal_rtl_snand_issuePageDataReadToCacheBufOpCmd */


/*
 * PIO Read One Chunk (2112-Byte)
 * Start from the assigned cache register address (CA=col_addr)
 */
SECTION_SNAND_TEXT
void
hal_rtl_snafc_pioPageRead(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	uint32_t col_addr = 0;

	if (pAdaptor) {
		pSnafcAdaptor = pAdaptor;
	}
	hal_rtl_snand_issuePageDataReadToCacheBufOpCmd(blkPageAddr);
	hal_rtl_snand_waitSnandOipComplete(pSnafcAdaptor);

	if (pAdaptor) {
		col_addr = pSnafcAdaptor->col_addr;
		if ((0 != pSnafcAdaptor->snand_cmd_info.r_cmd) && (mDefaultSnandCmdInfo.r_cmd != pSnafcAdaptor->snand_cmd_info.r_cmd)) {
			mDefaultSnandCmdInfo.r_cmd = pSnafcAdaptor->snand_cmd_info.r_cmd;
			mDefaultSnandCmdInfo.r_addr_io = pSnafcAdaptor->snand_cmd_info.r_addr_io;
			mDefaultSnandCmdInfo.r_data_io = pSnafcAdaptor->snand_cmd_info.r_data_io;
		}
	} else {
		col_addr = 0;
	}

	_hal_rtl_snafc_tx_cmd(mDefaultSnandCmdInfo.r_cmd, _hal_rtl_snafc_TxCmdAndReadBuffer, col_addr, mDefaultSnandCmdInfo.r_addr_io, mDefaultSnandCmdInfo.r_data_io,
						  byteLens, memAddr);
} /* hal_rtl_snafc_pioPageRead */

SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_pioPageRead(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	uint32_t retVal;
	hal_rtl_snafc_pioPageRead(pAdaptor, memAddr, byteLens, blkPageAddr);
#if 1 /* FIXME ToDo: install callback for checking ECC */
	retVal = SUCCESS;
#endif /* FIXME ToDo: install callback */
	return retVal;
} /* hal_rtl_snand_pioPageRead */

/*
 * Trigger SNAND's DMA engine to move data.
 * dir: 0 read (from NAND flash to local memory); 1 write (from local memory to NAND flash)
 */
SECTION_SNAND_TEXT
static void
_hal_rtl_snafc_dmaTrigger(
	uint32_t dma_phy_addr,
	uint32_t dma_io_len,
	uint32_t wr_dir
)
{
	SNAFC_TypeDef *snafc_s = SNAFC_S;

	if (FAIL == _polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt)) {
		DBG_SNAND_ERR("CTLR TO.SR=(0x%08x).DCDSR=(0x%08x)\r\n", snafc_s->SNAFC_SR, snafc_s->SNAFC_DCDSR);
	}

	snafc_s->SNAFC_DRSAR = dma_phy_addr;
	snafc_s->SNAFC_DLR = dma_io_len; /* [29:28](I/O mode,0~2); [16:0](dataLen, n-1) */
	snafc_s->SNAFC_DTR = wr_dir; /* Trigger DMA write or read */ /* [0](DMARWE) 1 for WRITE; 0 for READ. */

	if (FAIL == _polling_snafc_dma_ready(mMaxSnafcDmaRdyCnt)) {
		DBG_SNAND_ERR("DMA TO.SR=(0x%08x).DCDSR=(0x%08x)\r\n", snafc_s->SNAFC_SR, snafc_s->SNAFC_DCDSR);
	}
	return;
} /* _hal_rtl_snafc_dmaTrigger */

/*
 * DMA Write Chunk (Less than 2112-Byte)
 * Start from the assigned cache register address (CA=col_addr)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_dmaPageWrite(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint32_t retVal;
	uint32_t w_data;
	uint32_t w_io_len;
	uint32_t dma_io_len;
	uint32_t virt_to_phy_addr; /*FIXME*/
	uint32_t col_addr = 0;
	uint8_t tPageAddrCycle = 3; /* 3 for 3 bytes pageAddr; 4 (or !=3) for 4 bytes pageAddrs. */
#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	uint8_t tRemainSz = 0; /* Due to SNAFC's DMA SZ MUST ALIGN4, using PIO to access remainSz bytes. (1,2,3) */
#endif

	DBG_SNAND_INFO("hal_rtl_snand_dmaPageWrite++\r\n");
	if (pAdaptor) {
		pSnafcAdaptor = pAdaptor;
		if ((0 != pSnafcAdaptor->snand_cmd_info.w_addr_cycle) && (tPageAddrCycle != pSnafcAdaptor->snand_cmd_info.w_addr_cycle)) {
			tPageAddrCycle = pSnafcAdaptor->snand_cmd_info.w_addr_cycle;
		}
		col_addr = pSnafcAdaptor->col_addr;
		if ((0 != pSnafcAdaptor->snand_cmd_info.w_cmd) && (mDefaultSnandCmdInfo.w_cmd != pSnafcAdaptor->snand_cmd_info.w_cmd)) {
			mDefaultSnandCmdInfo.w_cmd = pSnafcAdaptor->snand_cmd_info.w_cmd;
			mDefaultSnandCmdInfo.w_addr_io = pSnafcAdaptor->snand_cmd_info.w_addr_io;
			mDefaultSnandCmdInfo.w_data_io = pSnafcAdaptor->snand_cmd_info.w_data_io;
		}
	} else {
		tPageAddrCycle = 3;
		col_addr = 0;
	}

	hal_rtl_snand_issueWriteEnableOpCmd(pAdaptor);
	/*++parallel process, saving wait time++*/
	virt_to_phy_addr = (uint32_t) memAddr; /*FIXME*/
	/*--parallel process, saving wait time--*/

	/* Control csLow()/csHigh() for dmaTrigger() */
	_hal_rtl_snafc_csLow();

	/**
	 According to (cmdWidth-addrWidth-dataWidth) configuration, generate waveform on SPI bus.
	 */
	/* Command: SIO, 1-Byte (OP) */
	w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	w_data = (mDefaultSnandCmdInfo.w_cmd << 24); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	if (tPageAddrCycle == 3) {
		/* Address: addr_io_len, 2-Bytes (colAddr) */
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.w_addr_io, SNAFC_XCMR_LEN(2));
		w_data = (col_addr << 16); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

		snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	} else {
		/* (tPageAddrCycle != 3) */ /* (it should be 4) */
		/* Address: addr_io_len, 3-Bytes (colAddr) */
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.w_addr_io, SNAFC_XCMR_LEN(3));
		w_data = (col_addr << 8); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

		snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	}

#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	tRemainSz = (byteLens & 0x3);
	byteLens = (byteLens & ~0x3);
#endif

	dma_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.w_data_io, byteLens); /* [29:28](I/O mode,0~2); [16:0](dataLen, N. (NOT N-1). round_down_4) */

	/* Data: data_io_len, byteLens (data payload) */
	_hal_rtl_snafc_dmaTrigger(virt_to_phy_addr, dma_io_len, 1); /* 0 for READ; 1 for WRITE */
#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	if (tRemainSz) {
		uint8_t tmp[4];
		uint8_t *pTmp = NULL;
		pTmp = (uint8_t *)(virt_to_phy_addr + byteLens);
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.w_data_io, SNAFC_XCMR_LEN(tRemainSz));
		/* The last few bytes may be garbage but won't be written to flash, since its length is explicitly given */
		tmp[0] = *((uint8_t *)pTmp++);
		tmp[1] = *((uint8_t *)pTmp++);
		tmp[2] = *((uint8_t *)pTmp++);
		tmp[3] = *((uint8_t *)pTmp++);
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		snafc_s->SNAFC_WDR = *((unsigned int *)tmp); //Trigger PIO Write
	}
#endif

	/* Control csLow()/csHigh() for dmaTrigger() */
	_hal_rtl_snafc_csHigh();

	retVal = hal_rtl_snand_issueProgramExecuteOpCmd(pAdaptor, blkPageAddr);
	hal_rtl_snand_issueWriteDisableOpCmd(pAdaptor);

	DBG_SNAND_INFO("hal_rtl_snand_dmaPageWrite--\r\n");
	return retVal;
} /* hal_rtl_snand_dmaPageWrite */


/*
 * DMA Read One Chunk (2112-Byte)
 * Start from the assigned cache register address (CA=col_addr)
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_dmaPageRead(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	SNAFC_TypeDef *snafc_s = SNAFC_S;
	uint32_t w_data;
	uint32_t w_io_len;
	uint32_t dma_io_len;
	uint32_t virt_to_phy_addr; /*FIXME*/
	uint32_t col_addr = 0;
	uint8_t tPageAddrCycle = 3; /* 3 for 3 bytes pageAddr; 4 (or !=3) for 4 bytes pageAddrs. */
#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	uint8_t tRemainSz = 0; /* Due to SNAFC's DMA SZ MUST ALIGN4, using PIO to access remainSz bytes. (1,2,3) */
#endif
	uint32_t retVal;

	DBG_SNAND_INFO("hal_rtl_snand_dmaPageRead(0x%x,0x%x,0x%x,0x%x)++\r\n", memAddr, byteLens, blkPageAddr, col_addr);
	if (pAdaptor) {
		pSnafcAdaptor = pAdaptor;
		if ((0 != pSnafcAdaptor->snand_cmd_info.r_addr_cycle) && (tPageAddrCycle != pSnafcAdaptor->snand_cmd_info.r_addr_cycle)) {
			tPageAddrCycle = pSnafcAdaptor->snand_cmd_info.r_addr_cycle;
		}
		col_addr = pSnafcAdaptor->col_addr;
		if ((0 != pSnafcAdaptor->snand_cmd_info.r_cmd) && (mDefaultSnandCmdInfo.r_cmd != pSnafcAdaptor->snand_cmd_info.r_cmd)) {
			mDefaultSnandCmdInfo.r_cmd = pSnafcAdaptor->snand_cmd_info.r_cmd;
			mDefaultSnandCmdInfo.r_addr_io = pSnafcAdaptor->snand_cmd_info.r_addr_io;
			mDefaultSnandCmdInfo.r_data_io = pSnafcAdaptor->snand_cmd_info.r_data_io;
		}
	} else {
		tPageAddrCycle = 3;
		col_addr = 0;
	}

	hal_rtl_snand_issuePageDataReadToCacheBufOpCmd(blkPageAddr);
	/*++parallel process, saving wait time++*/
	virt_to_phy_addr = (uint32_t) memAddr; /*FIXME*/
	/*--parallel process, saving wait time--*/
	hal_rtl_snand_waitSnandOipComplete(pAdaptor);

	/* Control csLow()/csHigh() for dmaTrigger() */
	_hal_rtl_snafc_csLow();

	/**
	 According to (cmdWidth-addrWidth-dataWidth) configuration, generate waveform on SPI bus.
	 */
	/* Command: SIO, 1-Byte (OP) */
	w_io_len = SNAFC_IO_WIDTH_LEN(SNAFC_SIO_WIDTH, SNAFC_XCMR_LEN(1));
	w_data = (mDefaultSnandCmdInfo.r_cmd << 24); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	if (tPageAddrCycle == 3) {
		/* Address: addr_io_len, 2-Bytes (colAddr) */
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_addr_io, SNAFC_XCMR_LEN(2));
		w_data = (col_addr << 16); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

		snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	} else {
		/* (tPageAddrCycle != 3) */ /* (it should be 4) */
		/* Address: addr_io_len, 3-Bytes (colAddr) */
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_addr_io, SNAFC_XCMR_LEN(3));
		w_data = (col_addr << 8); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

		snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	}

	/* Add 1 byte Dummy cycle, only for READ */
	/* (Dummy) Address: addr_io_len, 1-Bytes */
	w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_addr_io, SNAFC_XCMR_LEN(1));
	w_data = (0 << 24); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */

	_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);

	snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
	snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write

	/* Extra 1 byte Dummy cycle, only for "Fast Read Quad I/O" (EBh). i.e. 2 dummy cycle for (r_cmd==0xEB). */
	if (mDefaultSnandCmdInfo.r_cmd == SNAND_FAST_READ_QIO_OP) {
		/* (Dummy) Address: addr_io_len, 1-Bytes (Dummy) */
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_addr_io, SNAFC_XCMR_LEN(1));
		w_data = (0 << 24); /* SNAFC always fetch from WCMR[3]; then WCMR[2], then WCMR[1], then WCMR[0]. */
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		snafc_s->SNAFC_WCMR = w_io_len; /* [29:28](I/O mode,0~2); [1:0](dataLen, n-1) */
		snafc_s->SNAFC_WDR = w_data; //Trigger PIO Write
	}

#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	tRemainSz = (byteLens & 0x3);
	byteLens = (byteLens & ~0x3);
#endif

	dma_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_data_io, byteLens); /* [29:28](I/O mode,0~2); [16:0](dataLen, N. (NOT N-1). round_down_4) */

	/* Data: data_io_len, byteLens (data payload) */
	_hal_rtl_snafc_dmaTrigger(virt_to_phy_addr, dma_io_len, 0); /* 0 for READ; 1 for WRITE */
#if 1 /* ROUND_DOWN to ALIGN4 for fitting DMA engine spec. */
	if (tRemainSz) {
		uint8_t tmp[4];
		uint8_t *pTmp = NULL;
		pTmp = (uint8_t *)(virt_to_phy_addr + byteLens);
		_polling_snafc_ctrlr_ready(mMaxSnafcCtrlRdyCnt);
		w_io_len = SNAFC_IO_WIDTH_LEN(mDefaultSnandCmdInfo.r_data_io, SNAFC_XCMR_LEN(tRemainSz));
		/* Copy remain data (1~3 bytes) from HW FIFO to memory */
		*(unsigned int *)tmp = snafc_s->SNAFC_RDR;
		*(uint8_t *)pTmp = tmp[3];
		pTmp++;
		if (tRemainSz > 1) {
			*(uint8_t *)pTmp = tmp[2];
			pTmp++;
		}
		if (tRemainSz > 2) {
			*(uint8_t *)pTmp = tmp[1];
			pTmp++;
		}
	}
#endif
	/* Control csLow()/csHigh() for dmaTrigger() */
	_hal_rtl_snafc_csHigh();

#if 1 /* FIXME ToDo: install callback for checking ECC */
	retVal = SUCCESS;
#endif /* FIXME ToDo: install callback */

	DBG_SNAND_INFO("hal_rtl_snand_dmaPageRead--\r\n");
	return retVal;
} /* hal_rtl_snand_dmaPageRead */


/*
 * Wrapper API for data write and data read
 */

/*
 * Wrapper API for writing data to S-NAND flash
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_pageWrite(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	uint32_t dma_en;
	uint32_t retVal;

	if (pAdaptor) {
		pSnafcAdaptor = pAdaptor;
		dma_en = pSnafcAdaptor->dma_en;
	} else {
		dma_en = 1;
	}

	if (!dma_en) {
		retVal = hal_rtl_snand_pioPageWrite(pAdaptor, memAddr, byteLens, blkPageAddr);
	} else {
		retVal = hal_rtl_snand_dmaPageWrite(pAdaptor, memAddr, byteLens, blkPageAddr);
	}
	DBG_SNAND_INFO("hal_rtl_snand_pageWrite--\r\n");
	return retVal;
} /* hal_rtl_snand_pageWrite */


/*
 * Wrapper API for reading data from S-NAND flash
 */
SECTION_SNAND_TEXT
uint32_t
hal_rtl_snand_pageRead(
	void *pAdaptor,
	void *memAddr,
	uint32_t byteLens,
	uint32_t blkPageAddr
)
{
	hal_snafc_adaptor_t *pSnafcAdaptor = NULL;
	uint32_t dma_en;
	uint32_t retVal;

	if (pAdaptor) {
		pSnafcAdaptor = pAdaptor;
		dma_en = pSnafcAdaptor->dma_en;
	} else {
		dma_en = 1;
	}

	if (!dma_en) {
		retVal = hal_rtl_snand_pioPageRead(pAdaptor, memAddr, byteLens, blkPageAddr);
	} else {
		retVal = hal_rtl_snand_dmaPageRead(pAdaptor, memAddr, byteLens, blkPageAddr);
	}

	DBG_SNAND_INFO("hal_rtl_snand_pageRead--\r\n");
	return retVal;
} /* hal_rtl_snand_pageRead */

/** *@} */ /* End of group hal_rtl_snand_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** *@} */

#endif /* !defined(CONFIG_BUILD_BOOT) */

#endif /* CONFIG_SNAND_FLASH_EN */