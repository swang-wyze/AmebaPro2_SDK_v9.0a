/**************************************************************************//**
 * @file     rtl8735b_gdma.c
 * @brief    Implement HAL GDMA ROM code functions.
 * @version  1.00
 * @date     2020-08-13
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

#include "cmsis.h"

#define SECTION_GDMA_TEXT           SECTION(".rom.hal_gdma.text")
#define SECTION_GDMA_DATA           SECTION(".rom.hal_gdma.data")
#define SECTION_GDMA_RODATA         SECTION(".rom.hal_gdma.rodata")
#define SECTION_GDMA_BSS            SECTION(".rom.hal_gdma.bss")
#define SECTION_GDMA_STUBS          SECTION(".rom.hal_gdma.stubs")

extern void *_memset(void *s, int c, SIZE_T n);
void GDMA0_IRQHandler(void);
void GDMA1_IRQHandler(void);

/**

        \addtogroup hal_gdma GDMA
        @{
*/


#if !defined (CONFIG_BUILD_NONSECURE)

/**
  \brief The physical address mapping of GDMA channels on secure domain.
*/
SECTION_GDMA_RODATA const u32 chnl_base[MAX_GDMA_INDX + 1][MAX_GDMA_CHNL] = {
	{
		SGDMA0_CH0_BASE, SGDMA0_CH1_BASE
		, SGDMA0_CH2_BASE, SGDMA0_CH3_BASE
		, SGDMA0_CH4_BASE, SGDMA0_CH5_BASE
	},
	{
		SGDMA1_CH0_BASE, SGDMA1_CH1_BASE
		, SGDMA1_CH2_BASE, SGDMA1_CH3_BASE
		, SGDMA1_CH4_BASE, SGDMA1_CH5_BASE
	}
};

/**
  \brief GDMA channel pool, list all available channels.
*/
SECTION_GDMA_RODATA const hal_gdma_chnl_t gdma_chnl_option_rom[] = {
	{0, 0},
	{1, 0},
	{0, 1},
	{1, 1},
	{0, 2},
	{1, 2},
	{0, 3},
	{1, 3},
	{0, 4},
	{1, 4},
	{0, 5},
	{1, 5},
	{0xff, 0}   // end
};

/**
  \brief GDMA channel pool for multi-block transfer.
*/
SECTION_GDMA_RODATA const hal_gdma_chnl_t gdma_multi_block_chnl_option_rom[] = {
	{0, 4},
	{1, 4},
	{0, 5},
	{1, 5},
	{0xff, 0}   // end
};

#else

/**
  \brief The physical address mapping of GDMA channels on non-secure domain.
*/
SECTION_GDMA_RODATA const u32 chnl_base[MAX_GDMA_INDX + 1][MAX_GDMA_CHNL] = {
	{GDMA0_CH0_BASE, GDMA0_CH1_BASE, GDMA0_CH2_BASE, GDMA0_CH3_BASE, GDMA0_CH4_BASE, GDMA0_CH5_BASE},
	{GDMA1_CH0_BASE, GDMA1_CH1_BASE, GDMA1_CH2_BASE, GDMA1_CH3_BASE, GDMA1_CH4_BASE, GDMA1_CH5_BASE}
};

#endif

/**
  \brief The global common data structure to store and manage common resources for all GDMA adaptors.
*/
SECTION_GDMA_BSS phal_gdma_group_t phal_gdma_group;

/// @cond DOXYGEN_ROM_HAL_API

/**

        \addtogroup hal_gdma_rom_func GDMA HAL ROM APIs
        \ingroup hal_gdma
        \brief GDMA functions to control GDMA and initiate GDMA transfer in high speed platform.
               The user application(in RAM space) should not call these APIs directly.
               There is another set of GDMA HAL APIs in the RAM space is provided for the user application.
        @{
*/


/**
  \brief The stubs functions table to exports GDMA HAL functions in ROM.
*/
SECTION_GDMA_STUBS const hal_gdma_func_stubs_t hal_gdma_stubs = {
	.pphal_gdma_group = &phal_gdma_group,
	.hal_gdma_on = hal_rtl_gdma_on,
	.hal_gdma_off = hal_rtl_gdma_off,
	.hal_gdma_chnl_en = hal_rtl_gdma_chnl_en,
	.hal_gdma_chnl_dis = hal_rtl_gdma_chnl_dis,
	.hal_gdma_isr_en = hal_rtl_gdma_isr_en,
	.hal_gdma_isr_dis = hal_rtl_gdma_isr_dis,
	.hal_gdma_clean_pending_isr = hal_rtl_gdma_clean_pending_isr,
	.hal_gdma_clean_chnl_isr = hal_rtl_gdma_clean_chnl_isr,
	.hal_gdma_chnl_clean_auto_src = hal_rtl_gdma_chnl_clean_auto_src,
	.hal_gdma_chnl_clean_auto_dst = hal_rtl_gdma_chnl_clean_auto_dst,
	.hal_gdma_chnl_setting = hal_rtl_gdma_chnl_setting,
	.hal_gdma_chnl_block_setting = hal_rtl_gdma_chnl_block_setting,
	.hal_gdma_query_dar = hal_rtl_gdma_query_dar,
	.hal_gdma_query_sar = hal_rtl_gdma_query_sar,
	.hal_gdma_query_chnl_en = hal_rtl_gdma_query_chnl_en,
	.hal_gdma_query_send_bytes = hal_rtl_gdma_query_send_bytes,
	.hal_gdma_query_abort_recv_bytes = hal_rtl_gdma_query_abort_recv_bytes,
	.hal_gdma_query_recv_bytes = hal_rtl_gdma_query_recv_bytes,
#if !defined (CONFIG_BUILD_NONSECURE)
	.hal_gdma_chnl_register = hal_rtl_gdma_chnl_register,
	.hal_gdma_chnl_unregister = hal_rtl_gdma_chnl_unregister,
#endif
	.hal_gdma_chnl_init = hal_rtl_gdma_chnl_init,
	.hal_gdma_chnl_irq_free = hal_rtl_gdma_chnl_irq_free,
	.hal_gdma_handshake_init = hal_rtl_gdma_handshake_init,
	.hal_gdma_memcpy_irq_hook = hal_rtl_gdma_memcpy_irq_hook,
	.hal_gdma_memcpy_irq_handler = hal_rtl_gdma_memcpy_irq_handler,
	.hal_gdma0_irq_handler = GDMA0_IRQHandler,
	.hal_gdma1_irq_handler = GDMA1_IRQHandler,
	.hal_gdma_irq_set_priority = hal_rtl_gdma_irq_set_priority,
	.hal_gdma_irq_reg = hal_rtl_gdma_irq_reg,
	.hal_gdma_transfer_start = hal_rtl_gdma_transfer_start,
	.hal_gdma_group_init = hal_rtl_gdma_group_init,
	.hal_gdma_multi_block_init = hal_rtl_gdma_multi_block_init,
	.hal_gdma_memcpy_config = hal_rtl_gdma_memcpy_config,
	.hal_gdma_linked_list_block_init = hal_rtl_gdma_linked_list_block_init,
	.hal_gdma_linked_list_block_config = hal_rtl_gdma_linked_list_block_config,
	.hal_gdma_abort = hal_rtl_gdma_abort,
	.hal_gdma_chnl_reset = hal_rtl_gdma_chnl_reset
};


/** \brief Description of hal_rtl_gdma_on
 *
 *    hal_rtl_gdma_on is used to turn on GDMA IP.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_on(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;

	gdma_dev->GDMA_DMA_CFG_REG = GDMA_BIT_DMA_EN;
}

/** \brief Description of hal_rtl_gdma_off
 *
 *    hal_rtl_gdma_off is used to turn off GDMA IP.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_off(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;

	gdma_dev->GDMA_DMA_CFG_REG &= ~GDMA_BIT_DMA_EN;
}

/** \brief Description of hal_rtl_gdma_chnl_en
 *
 *    hal_rtl_gdma_chnl_en is used to enable the GDMA channel, then GDMA transfer will start.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_en(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;

	gdma_dev->GDMA_CH_EN_REG |= phal_gdma_adaptor->ch_en;
}

/** \brief Description of hal_rtl_gdma_chnl_dis
 *
 *    hal_rtl_gdma_chnl_dis is used to disable the GDMA channel, then GDMA transfer stops.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_dis(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u32 value = gdma_dev->GDMA_CH_EN_REG;

	value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
	value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
	gdma_dev->GDMA_CH_EN_REG = value;
}

/** \brief Description of hal_rtl_gdma_isr_en
 *
 *    hal_rtl_gdma_isr_en is used to enable(unmask) GDMA ISR Type.
 *    Typically we will unmask Transfer type interrupt to notify all transfer are complete
 *    and Error type interrupt to inform us an error occurs during transmission
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_isr_en(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u32 gdma_isr_type = phal_gdma_adaptor->gdma_isr_type;

	if (gdma_isr_type & TransferType) {
		gdma_dev->GDMA_MASK_TFR |= phal_gdma_adaptor->ch_en;
	}

	if (gdma_isr_type & BlockType) {
		gdma_dev->GDMA_MASK_BLOCK |= phal_gdma_adaptor->ch_en;

	}

	if (gdma_isr_type & SrcTransferType) {
		gdma_dev->GDMA_MASK_SRC_TRAN |= phal_gdma_adaptor->ch_en;

	}

	if (gdma_isr_type & DstTransferType) {
		gdma_dev->GDMA_MASK_DST_TRAN |= phal_gdma_adaptor->ch_en;

	}

	if (gdma_isr_type & ErrType) {
		gdma_dev->GDMA_MASK_ERR_LOW |= phal_gdma_adaptor->ch_en;
		gdma_dev->GDMA_MASK_ERR_UP |= phal_gdma_adaptor->ch_en;
	}
}


/** \brief Description of hal_rtl_gdma_isr_dis
 *
 *    hal_rtl_gdma_isr_dis is used to mask GDMA ISR type.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_isr_dis(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u32 gdma_isr_type = phal_gdma_adaptor->gdma_isr_type;
	u32 value;

	if (gdma_isr_type & TransferType) {
		value = gdma_dev->GDMA_MASK_TFR;
		value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
		value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
		gdma_dev->GDMA_MASK_TFR = value;
	}

	if (gdma_isr_type & BlockType) {
		value = gdma_dev->GDMA_MASK_BLOCK;
		value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
		value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
		gdma_dev->GDMA_MASK_BLOCK = value;
	}

	if (gdma_isr_type & SrcTransferType) {
		value = gdma_dev->GDMA_MASK_SRC_TRAN;
		value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
		value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
		gdma_dev->GDMA_MASK_SRC_TRAN = value;
	}

	if (gdma_isr_type & DstTransferType) {
		value = gdma_dev->GDMA_MASK_DST_TRAN;
		value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
		value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
		gdma_dev->GDMA_MASK_DST_TRAN = value;
	}

	if (gdma_isr_type & ErrType) {
		value = gdma_dev->GDMA_MASK_ERR_LOW;
		value = value & ~(phal_gdma_adaptor->ch_en & 0xFF);
		value = value | (phal_gdma_adaptor->ch_en & 0xFF00);
		gdma_dev->GDMA_MASK_ERR_LOW = value;
		gdma_dev->GDMA_MASK_ERR_UP = value;
	}
}

/** \brief Description of hal_rtl_gdma_clean_pending_isr
 *
 *    hal_rtl_gdma_clean_pending_isr is used to clear pending interrupt regardless of ISR types.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_clean_pending_isr(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u8  chnl_bit_map = (phal_gdma_adaptor->ch_en & 0xFF);

	if (gdma_dev->GDMA_RAW_TFR & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_TFR = chnl_bit_map;
	}

	if (gdma_dev->GDMA_RAW_BLOCK & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_BLOCK = chnl_bit_map;
	}

	if (gdma_dev->GDMA_RAW_SRC_TRAN & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_SRC_TRAN = chnl_bit_map;
	}

	if (gdma_dev->GDMA_RAW_DST_TRAN & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_DST_TRAN = chnl_bit_map;
	}

	if (gdma_dev->GDMA_RAW_ERR_LOW & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_ERR_LOW = chnl_bit_map;
	}

	if (gdma_dev->GDMA_RAW_ERR_UP & chnl_bit_map) {
		gdma_dev->GDMA_CLEAR_ERR_UP = chnl_bit_map;
	}
}

/** \brief Description of hal_rtl_gdma_clean_chnl_isr
 *
 *    hal_rtl_gdma_clean_chnl_isr is used to clean pending interrupts which we unmask .
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_clean_chnl_isr(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u8 gdma_isr_type = phal_gdma_adaptor->gdma_isr_type;
	u8 chnl_bit_map = (phal_gdma_adaptor->ch_en & 0xFF);

	if (gdma_isr_type & TransferType) {
		if (gdma_dev->GDMA_RAW_TFR & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_TFR = chnl_bit_map;
		}
	}

	if (gdma_isr_type & BlockType) {
		if (gdma_dev->GDMA_RAW_BLOCK & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_BLOCK = chnl_bit_map;
		}
	}

	if (gdma_isr_type & SrcTransferType) {
		if (gdma_dev->GDMA_RAW_SRC_TRAN & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_SRC_TRAN = chnl_bit_map;
		}
	}

	if (gdma_isr_type & DstTransferType) {
		if (gdma_dev->GDMA_RAW_DST_TRAN & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_DST_TRAN = chnl_bit_map;
		}
	}

	if (gdma_isr_type & ErrType) {
		if (gdma_dev->GDMA_RAW_ERR_LOW & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_ERR_LOW = chnl_bit_map;
		}

		if (gdma_dev->GDMA_RAW_ERR_UP & chnl_bit_map) {
			gdma_dev->GDMA_CLEAR_ERR_UP = chnl_bit_map;
		}
	}
}

/** \brief Description of hal_rtl_gdma_chnl_clean_auto_src
 *
 *    hal_rtl_gdma_chnl_clean_auto_src is used to disable auto reload function of source.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_clean_auto_src(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;

	chnl_dev->GDMA_CH_CFG_LOW &= ~GDMA_BIT_CH_RELOAD_SRC;

	DBG_GDMA_INFO("CFG Low data:0x%x\n", chnl_dev->GDMA_CH_CFG_LOW);
}

/** \brief Description of hal_rtl_gdma_chnl_clean_auto_dst
 *
 *    hal_rtl_gdma_chnl_clean_auto_dst is used to disable auto reload function of destination.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_clean_auto_dst(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;

	chnl_dev->GDMA_CH_CFG_LOW &= ~GDMA_BIT_CH_RELOAD_DST;

	DBG_GDMA_INFO("CFG Low data:0x%x\n", chnl_dev->GDMA_CH_CFG_LOW);
}

/** \brief Description of hal_rtl_gdma_chnl_setting
 *
 *    hal_rtl_gdma_chnl_setting is used to set GDMA registers.
 *    Four registers are configured according to information carried by the adaptor:
 *    SAR(Source address), DAR(Destination address), CTL(Control register), CFG(Configuration register)
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_statu_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_setting(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 ctl_up = 0;
	u32 ctl_low = 0;
	u32 cfg_up = 0;
	u32 cfg_low = 0;

	/*Check channel availability*/
	if ((gdma_dev->GDMA_CH_EN_REG & GDMA_MASK_CH_EN) & (phal_gdma_adaptor->ch_en & 0xFF)) {
		//4 Disable Channel
		DBG_GDMA_WARN("Channel had used; Disable Channel!!!!\n");
		hal_rtl_gdma_chnl_dis(phal_gdma_adaptor);
	}

	/*Clear pending interrupt for the channel*/
	hal_rtl_gdma_clean_pending_isr(phal_gdma_adaptor);

	/*Assign global variables to the local ctl struct*/
	ctl_low = GDMA_BIT_CH_INT_EN | (phal_gdma_adaptor->gdma_ctl.tt_fc << GDMA_SHIFT_CH_TT_FC);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.dst_tr_width << GDMA_SHIFT_CH_DST_TR_WIDTH) | (phal_gdma_adaptor->gdma_ctl.src_tr_width << GDMA_SHIFT_CH_SRC_TR_WIDTH);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.sinc << GDMA_SHIFT_CH_SINC) | (phal_gdma_adaptor->gdma_ctl.dinc << GDMA_SHIFT_CH_DINC);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.src_msize << GDMA_SHIFT_CH_SRC_MSIZE) | (phal_gdma_adaptor->gdma_ctl.dest_msize << GDMA_SHIFT_CH_DEST_MSIZE);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.llp_src_en << GDMA_SHIFT_CH_LLP_SRC_EN) | (phal_gdma_adaptor->gdma_ctl.llp_dst_en << GDMA_SHIFT_CH_LLP_DST_EN);
	ctl_up = phal_gdma_adaptor->gdma_ctl.block_size & GDMA_MASK_CH_BLOCK_TS;

	/*Write the ctl registers from the local ctl struct*/
	chnl_dev->GDMA_CH_CTL_LOW = ctl_low;
	chnl_dev->GDMA_CH_CTL_UP = ctl_up;

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting : CTLx Low data:0x%x\r\n", chnl_dev->GDMA_CH_CTL_LOW);
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting : CTLx Up data:0x%x\r\n", chnl_dev->GDMA_CH_CTL_UP);

	/*Write source address to the source address register*/
	chnl_dev->GDMA_CH_SAR = phal_gdma_adaptor->ch_sar;

	/*Write destination address to the destination address register*/
	chnl_dev->GDMA_CH_DAR = phal_gdma_adaptor->ch_dar;

	/*Assign global variables to the local cfg struct*/
	cfg_low = (phal_gdma_adaptor->gdma_cfg.reload_src << GDMA_SHIFT_CH_RELOAD_SRC) | (phal_gdma_adaptor->gdma_cfg.reload_dst << GDMA_SHIFT_CH_RELOAD_DST);
	cfg_up = GDMA_BIT_CH_FIFO_MODE;
	cfg_up |= ((phal_gdma_adaptor->gdma_cfg.src_per & 0xF) << GDMA_SHIFT_CH_SRC_PER) | ((phal_gdma_adaptor->gdma_cfg.src_per >> 4) <<
			  GDMA_SHIFT_CH_EXTENDED_SRC_PER);
	cfg_up |= ((phal_gdma_adaptor->gdma_cfg.dest_per & 0xF) << GDMA_SHIFT_CH_DEST_PER) | ((phal_gdma_adaptor->gdma_cfg.dest_per >> 4) <<
			  GDMA_SHIFT_CH_EXTENDED_DEST_PER);

#if !defined (CONFIG_BUILD_NONSECURE)
	cfg_up &= ~GDMA_BIT_CH_SECURE_EN;
	cfg_up |= (SecureType << GDMA_SHIFT_CH_SECURE_EN);
#endif

	/*Write the cfg registers from the local ctl struct*/
	chnl_dev->GDMA_CH_CFG_LOW = cfg_low;
	chnl_dev->GDMA_CH_CFG_UP = cfg_up;

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting: CFG Low data:0x%x\r\n", chnl_dev->GDMA_CH_CFG_LOW);
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting: CFG High data:0x%x\r\n", chnl_dev->GDMA_CH_CFG_UP);

	return HAL_OK;
}

/** \brief Description of hal_rtl_gdma_chnl_block_setting
 *
 *    hal_rtl_gdma_chnl_block_setting is used to set GDMA registers in multi-block mode.
 *    Five registers are configured according to information carried by the adaptor:
 *    SAR(Source address), DAR(Destination address), CTL(Control register), CFG(Configuration register), LLP(Linking list register).
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_statu_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_block_setting(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	pgdma_ch_lli_t pgdma_ch_lli;
	u32 ctl_up = 0;
	u32 ctl_low = 0;
	u32 cfg_up = 0;
	u32 cfg_low = 0;
	u32 block_num = phal_gdma_adaptor->max_block_num;
	u32 index = 0;

	pgdma_ch_lli = phal_gdma_adaptor->pgdma_ch_lli;

	/*Check channel availability*/
	if ((gdma_dev->GDMA_CH_EN_REG & GDMA_MASK_CH_EN) & (phal_gdma_adaptor->ch_en & 0xFF)) {
		//4 Disable Channel
		DBG_GDMA_WARN("Channel had used; Disable Channel!!!!\n");
		hal_rtl_gdma_chnl_dis(phal_gdma_adaptor);
	}

	/*Clear pending interrupt for the channel*/
	hal_rtl_gdma_clean_pending_isr(phal_gdma_adaptor);

	/*Assign global variables to the local ctl struct*/
	ctl_low = GDMA_BIT_CH_INT_EN | (phal_gdma_adaptor->gdma_ctl.tt_fc << GDMA_SHIFT_CH_TT_FC);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.dst_tr_width << GDMA_SHIFT_CH_DST_TR_WIDTH) | (phal_gdma_adaptor->gdma_ctl.src_tr_width << GDMA_SHIFT_CH_SRC_TR_WIDTH);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.sinc << GDMA_SHIFT_CH_SINC) | (phal_gdma_adaptor->gdma_ctl.dinc << GDMA_SHIFT_CH_DINC);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.src_msize << GDMA_SHIFT_CH_SRC_MSIZE) | (phal_gdma_adaptor->gdma_ctl.dest_msize << GDMA_SHIFT_CH_DEST_MSIZE);
	ctl_low |= (phal_gdma_adaptor->gdma_ctl.llp_src_en << GDMA_SHIFT_CH_LLP_SRC_EN) | (phal_gdma_adaptor->gdma_ctl.llp_dst_en << GDMA_SHIFT_CH_LLP_DST_EN);
	ctl_up = pgdma_ch_lli->ctlx_up;

	/*Write the ctl registers from the local ctl struct*/
	chnl_dev->GDMA_CH_CTL_LOW = ctl_low;
	chnl_dev->GDMA_CH_CTL_UP = ctl_up;

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting : CTLx Low data:0x%x\r\n", chnl_dev->GDMA_CH_CTL_LOW);
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting : CTLx Up data:0x%x\r\n", chnl_dev->GDMA_CH_CTL_UP);

	/*Write source address to the source address register*/
	chnl_dev->GDMA_CH_SAR = phal_gdma_adaptor->ch_sar;

	/*Write destination address to the destination address register*/
	chnl_dev->GDMA_CH_DAR = phal_gdma_adaptor->ch_dar;

	/*Assign global variables to the local cfg struct*/
	cfg_low = (phal_gdma_adaptor->gdma_cfg.reload_src << GDMA_SHIFT_CH_RELOAD_SRC) | (phal_gdma_adaptor->gdma_cfg.reload_dst << GDMA_SHIFT_CH_RELOAD_DST);
	cfg_up = GDMA_BIT_CH_FIFO_MODE;
	cfg_up |= ((phal_gdma_adaptor->gdma_cfg.src_per & 0xF) << GDMA_SHIFT_CH_SRC_PER) | ((phal_gdma_adaptor->gdma_cfg.src_per >> 4) <<
			  GDMA_SHIFT_CH_EXTENDED_SRC_PER);
	cfg_up |= ((phal_gdma_adaptor->gdma_cfg.dest_per & 0xF) << GDMA_SHIFT_CH_DEST_PER) | ((phal_gdma_adaptor->gdma_cfg.dest_per >> 4) <<
			  GDMA_SHIFT_CH_EXTENDED_DEST_PER);


#if !defined (CONFIG_BUILD_NONSECURE)
	cfg_up &= ~GDMA_BIT_CH_SECURE_EN;
	cfg_up |= (SecureType << GDMA_SHIFT_CH_SECURE_EN);
#endif
	/*Write the cfg registers from the local ctl struct*/
	chnl_dev->GDMA_CH_CFG_LOW = cfg_low;
	chnl_dev->GDMA_CH_CFG_UP = cfg_up;

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting: CFG Low data:0x%x\r\n", chnl_dev->GDMA_CH_CFG_LOW);
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_setting: CFG High data:0x%x\r\n", chnl_dev->GDMA_CH_CFG_UP);

	if (((u32)pgdma_ch_lli) & 0x3) {
		DBG_GDMA_ERR("Linked list item address 0x%x is not 4 bytes alignment \n", pgdma_ch_lli);
		return HAL_ERR_MEM;
	}

	chnl_dev->GDMA_CH_LLP = (u32)pgdma_ch_lli;
	DBG_GDMA_INFO("chnl_dev->llp = %x\r\n", chnl_dev->GDMA_CH_LLP);

	for (index = 0; index < block_num; index++) {
		if (index == (block_num - 1)) {
			ctl_low &= ~(GDMA_BIT_CH_LLP_DST_EN | GDMA_BIT_CH_LLP_SRC_EN);
		}

		pgdma_ch_lli->ctlx_low = ctl_low;
		pgdma_ch_lli++;
	}

	if (phal_gdma_adaptor->dcache_clean_by_addr != NULL) {
		phal_gdma_adaptor->dcache_clean_by_addr((uint32_t *)phal_gdma_adaptor->pgdma_ch_lli, (int32_t)(phal_gdma_adaptor->max_block_num * sizeof(gdma_ch_lli_t)));
	}

	return HAL_OK;
}


/** \brief Description of hal_rtl_gdma_query_dar
 *
 *    hal_rtl_gdma_query_dar is used to get current destination address.
 *    The address may not be accurate. It is recommended to use query receive/send byte functions to get correct information.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return u32: current destination address .
 */
SECTION_GDMA_TEXT
u32 hal_rtl_gdma_query_dar(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 dar;

	dar = chnl_dev->GDMA_CH_DAR;

	return dar;
}

/** \brief Description of hal_rtl_gdma_query_sar
 *
 *    hal_rtl_gdma_query_sar is used to get current source address.
 *    The address may not be accurate. It is recommended to use query receive/send byte functions to get correct information.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return u32: current source address .
 */
SECTION_GDMA_TEXT
u32 hal_rtl_gdma_query_sar(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 sar;

	sar = chnl_dev->GDMA_CH_SAR;

	return sar;
}

/** \brief Description of hal_rtl_gdma_query_chnl_en
 *
 *    hal_rtl_gdma_query_chnl_en is used to check whether the channel is enabled or not.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return BOOL: 1: Channel is enabled, 0: Channel is disabled.
 */
SECTION_GDMA_TEXT
BOOL hal_rtl_gdma_query_chnl_en(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;

	if ((gdma_dev->GDMA_CH_EN_REG & GDMA_MASK_CH_EN) & (phal_gdma_adaptor->ch_en & 0xFF)) {
		return 1;
	} else {
		return 0;
	}
}

/** \brief Description of hal_rtl_gdma_query_abort_recv_bytes
 *
 *    hal_rtl_gdma_query_abort_recv_bytes is used to query how many bytes GDMA received when the transmission is aborted.
 *    Source : Peripheral , Destination : Memory. Query how many bytes being received by the destination
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return u32: Number of bytes already received by the destination.
 */
SECTION_GDMA_TEXT
u32 hal_rtl_gdma_query_abort_recv_bytes(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 recv_bytes;
	u32 abort_dst_addr;
	u32 dst_offset;
	u32 recv_blocks;
	u32 dma_block_bytes;

	dma_block_bytes = (u32)MAX_DMA_BLOCK_SIZE  << phal_gdma_adaptor->gdma_ctl.src_tr_width;

	abort_dst_addr = (u32)chnl_dev->GDMA_CH_DAR;
	dst_offset = (abort_dst_addr - phal_gdma_adaptor->ch_dar);
	recv_blocks = dst_offset / dma_block_bytes;

	DBG_GDMA_INFO("abort_dar=0x%x, org_dra=0x%x, transferred bytes of the last block=0x%x,  recv_blocks=%lu\r\n", \
				  abort_dst_addr, phal_gdma_adaptor->ch_dar, (abort_dst_addr - phal_gdma_adaptor->ch_dar), chnl_dev->GDMA_CH_CTL_UP, recv_blocks);

	recv_bytes = chnl_dev->GDMA_CH_CTL_UP + (recv_blocks * dma_block_bytes);

	return recv_bytes;
}

/** \brief Description of hal_rtl_gdma_query_recv_bytes
 *
 *    hal_rtl_gdma_query_recv_bytes is used to query how many bytes GDMA received when the transmission is complete.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return u32: Number of bytes already received by the destination.
 */
SECTION_GDMA_TEXT
u32 hal_rtl_gdma_query_recv_bytes(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 recv_bytes;
	u32 abort_dst_addr;
	u32 dst_offset;
	u32 recv_blocks;
	u32 dma_block_bytes;

	dma_block_bytes = (u32)MAX_DMA_BLOCK_SIZE  << phal_gdma_adaptor->gdma_ctl.src_tr_width;

	abort_dst_addr = (u32)chnl_dev->GDMA_CH_DAR;
	dst_offset = (abort_dst_addr - phal_gdma_adaptor->ch_dar);
	recv_blocks = dst_offset / dma_block_bytes;

	DBG_GDMA_INFO("abort_dar=0x%x, org_dra=0x%x, transferred bytes of the last block=0x%x,  recv_blocks=%lu\r\n", \
				  abort_dst_addr, phal_gdma_adaptor->ch_dar, (abort_dst_addr - phal_gdma_adaptor->ch_dar), chnl_dev->GDMA_CH_CTL_UP, recv_blocks);

	recv_bytes = chnl_dev->GDMA_CH_CTL_UP + (recv_blocks * dma_block_bytes);

	return recv_bytes;
}


/** \brief Description of hal_rtl_gdma_query_send_bytes
 *
 *    hal_rtl_gdma_query_send_bytes is used to query how many bytes GDMA sent when it stops.
 *    Source : Memory , Destination : Peripheral. Query how many bytes being sent from the source.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return u32: Number of bytes already sent from the source.
 */
SECTION_GDMA_TEXT
u32 hal_rtl_gdma_query_send_bytes(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;
	u32 sent_bytes;
	u32 abort_sar_addr;
	u32 sent_blocks;
	u32 dma_block_bytes;

	dma_block_bytes = (u32)MAX_DMA_BLOCK_SIZE  << phal_gdma_adaptor->gdma_ctl.src_tr_width;
	abort_sar_addr = (u32)chnl_dev->GDMA_CH_SAR;
	sent_blocks = (abort_sar_addr - phal_gdma_adaptor->ch_sar) / dma_block_bytes;
	DBG_GDMA_INFO("abort_sar=0x%x, org_sar=0x%x, transferred bytes of the last block=0x%x, send_blocks=%lu\r\n", \
				  abort_sar_addr, phal_gdma_adaptor->ch_sar, (abort_sar_addr - phal_gdma_adaptor->ch_sar), chnl_dev->GDMA_CH_CTL_UP, sent_blocks);

	sent_bytes = chnl_dev->GDMA_CH_CTL_UP + (sent_blocks * dma_block_bytes);
	return sent_bytes;
}


#if !defined (CONFIG_BUILD_NONSECURE)
/** \brief Description of hal_rtl_gdma_chnl_register
 *
 *    hal_rtl_gdma_chnl_register is used to manage and register GDMA channel.
 *    It will check the viability of the target channel.
 *    If no one occupies this channel, the target channel is registered so that others cannot use this one.
 *    The GDMA clock is enabled and the reset is released if target GDMA has not been used before(GDMA is off).
 *
 *   \param u8 gdma_index:      The target GDMA, could be GDMA0 or GDMA1.
 *   \param u8 chnl_num:      The target GDMA channel, could be 0~5.
 *
 *   \return hal_status.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_register(u8 gdma_index, u8 chnl_num)
{
	GDMA_TypeDef *gdma_dev;
	u32 mask;

	if ((gdma_index > MAX_GDMA_INDX) || (chnl_num > MAX_GDMA_CHNL)) {
		// invalid gdma index or channel number
		return HAL_ERR_PARA;
	}

	mask = 1 << chnl_num;
	if (((phal_gdma_group->hal_gdma_reg[gdma_index]) & mask) != 0) {
		return HAL_BUSY;
	} else {
		if ((phal_gdma_group->hal_gdma_reg[gdma_index]) == 0) {
			if (gdma_index == 0) {
				hal_rtl_sys_peripheral_en(GDMA0_SYS, ENABLE);
				gdma_dev = (GDMA_TypeDef *) SGDMA0_BASE;
				gdma_dev->GDMA_DMA_CFG_REG = GDMA_BIT_DMA_EN;
			} else {
				hal_rtl_sys_peripheral_en(GDMA1_SYS, ENABLE);
				gdma_dev = (GDMA_TypeDef *) SGDMA1_BASE;
				gdma_dev->GDMA_DMA_CFG_REG = GDMA_BIT_DMA_EN;
			}
		}

		(phal_gdma_group->hal_gdma_reg[gdma_index]) |= mask;

		DBG_GDMA_INFO("hal_rtl_gdma_chnl_register: hal_gdma_reg[0] = %x\r\n", (phal_gdma_group->hal_gdma_reg[0]));
		DBG_GDMA_INFO("hal_rtl_gdma_chnl_register: hal_gdma_reg[1] = %x\r\n", (phal_gdma_group->hal_gdma_reg[1]));
		return HAL_OK;
	}
}

/** \brief Description of hal_rtl_gdma_chnl_unregister
 *
 *    hal_rtl_gdma_chnl_unregister is used to manage and unregister GDMA channel.
 *    When the transfer is complete and the channel is no long used, we can release this channel by unregistering it.
 *    The GDMA clock is disabled if no one uses this GDMA.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_unregister(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	u8 gdma_index = phal_gdma_adaptor->gdma_index;
	u8 ch_num = phal_gdma_adaptor->ch_num;
	u32 mask;

	if ((gdma_index > MAX_GDMA_INDX) || (ch_num > MAX_GDMA_CHNL)) {
		// invalid gdma index or channel number
		return HAL_ERR_PARA;
	}

	mask = 1 << ch_num;

	(phal_gdma_group->hal_gdma_reg[gdma_index]) &= ~mask;

	(phal_gdma_adaptor->chnl_dev)->GDMA_CH_SAR = 0;
	(phal_gdma_adaptor->chnl_dev)->GDMA_CH_DAR = 0;
	(phal_gdma_adaptor->chnl_dev)->GDMA_CH_CFG_UP |= (NonSecureType << GDMA_SHIFT_CH_SECURE_EN);

	if ((phal_gdma_group->hal_gdma_reg[gdma_index]) == 0) {
		if (gdma_index == 0) {
			hal_rtl_sys_peripheral_en(GDMA0_SYS, DISABLE);
		} else {
			hal_rtl_sys_peripheral_en(GDMA1_SYS, DISABLE);
		}
	}

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_unregister: hal_gdma_reg[0] = %x\r\n", (phal_gdma_group->hal_gdma_reg[0]));
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_unregister: hal_gdma_reg[1] = %x\r\n", (phal_gdma_group->hal_gdma_reg[1]));
	return HAL_OK;
}

#endif

/** \brief Description of hal_rtl_gdma_chnl_init
 *
 *    hal_rtl_gdma_chnl_init is used to initialize register bases and channel related settings when the target channel is specified.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return hal_status_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_init(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	u8 ch_num;
	u8 gdma_index;

	phal_gdma_adaptor->gdma_dev = 0;
	ch_num = phal_gdma_adaptor->ch_num;
	gdma_index = phal_gdma_adaptor->gdma_index;
	phal_gdma_adaptor->ch_en = 0x0101 << ch_num;
	phal_gdma_group->phal_gdma_adaptor[gdma_index][ch_num] = phal_gdma_adaptor;

	if (ch_num >= MAX_GDMA_CHNL) {
		DBG_GDMA_ERR("No such channel, please re-allocate a valid channel\r\n");
		return HAL_ERR_PARA;
	}

	if (gdma_index == 0) {
#if !defined (CONFIG_BUILD_NONSECURE)
		phal_gdma_adaptor->gdma_dev = (GDMA_TypeDef *)SGDMA0_BASE;
#else
		phal_gdma_adaptor->gdma_dev = (GDMA_TypeDef *)GDMA0_BASE;
#endif
	} else {
#if !defined (CONFIG_BUILD_NONSECURE)
		phal_gdma_adaptor->gdma_dev = (GDMA_TypeDef *)SGDMA1_BASE;
#else
		phal_gdma_adaptor->gdma_dev = (GDMA_TypeDef *)GDMA1_BASE;
#endif
	}

	phal_gdma_adaptor->chnl_dev = (GDMA_CH_TypeDef *)chnl_base[gdma_index][ch_num];

#if !defined (CONFIG_BUILD_NONSECURE)
	/*Clear secure bit to zero to set secure mode*/
	(phal_gdma_adaptor->chnl_dev)->GDMA_CH_CFG_UP &= ~GDMA_BIT_CH_SECURE_EN;
#endif

	return HAL_OK;
}

/** \brief Description of hal_rtl_gdma_chnl_irq_free
 *
 *    hal_rtl_gdma_chnl_irq_free is used to disable GDMA IRQ for the corresponding GDMA.
 *    Before disabling the GDMA IRQ, it will check all channels of the GDMA are not used.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_irq_free(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	u8 gdma_index = phal_gdma_adaptor->gdma_index;
	u8 ch_num = phal_gdma_adaptor->ch_num;

	(phal_gdma_group->chnl_in_use[gdma_index]) &= ~(1 << ch_num);

	if (0 == (phal_gdma_group->chnl_in_use[gdma_index])) {
		hal_rtl_irq_disable(phal_gdma_adaptor->gdma_irq_num);
		__ISB();
		DBG_GDMA_INFO("GDMA IRQ is disabled\r\n");
	} else {
		DBG_GDMA_WARN("Other channel is still operating, GDMA IRQ is not disabled.\r\n");
	}

	DBG_GDMA_INFO("hal_rtl_gdma_chnl_irq_free: chnl_in_use[0] = %x\r\n", (phal_gdma_group->chnl_in_use[0]));
	DBG_GDMA_INFO("hal_rtl_gdma_chnl_irq_free: chnl_in_use[1] = %x\r\n", (phal_gdma_group->chnl_in_use[1]));

}

/** \brief Description of hal_rtl_gdma_handshake_init
 *
 *    hal_rtl_gdma_handshake_init is used to choose which GDMA to handshake for the target peripheral.
 *    Each peripheral has its own handshake number with GDMA. If the peripheral with handshake number x wants to
 *    use GDMA 1, then the x bit is set to 1, otherwise the x bit is 0.
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param u8 handshake_num:      The handshake number of the peripheral.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_handshake_init(phal_gdma_adaptor_t phal_gdma_adaptor, u8 handshake_num)
{
	u32 reg_value;
	VNDR_TypeDef *vdr = (VNDR_TypeDef *) VDR;

	reg_value = vdr->VNDR_REG_GDMA_HSKS_CTRL;

	if (phal_gdma_adaptor->gdma_index == 0) {
		reg_value &= ~(1 << handshake_num);
	} else {
		reg_value |= (1 << handshake_num);
	}

	vdr->VNDR_REG_GDMA_HSKS_CTRL = reg_value;
}


/** \brief Description of hal_rtl_gdma_memcpy_irq_hook
 *
 *    hal_rtl_gdma_memcpy_irq_hook is used to initialize the callback function and its parameter for memcpy transfer.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param gdma_callback_t gdma_cb_func:      The pointer of the callback function.
 *   \param void* gdma_cb_data:      The pointer of the paramter of the callback function.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_memcpy_irq_hook(phal_gdma_adaptor_t phal_gdma_adaptor, gdma_callback_t gdma_cb_func, void *gdma_cb_data)
{
	phal_gdma_adaptor->gdma_cb_func = gdma_cb_func;
	phal_gdma_adaptor->gdma_cb_para = gdma_cb_data;
}


/** \brief Description of hal_rtl_gdma_memcpy_irq_handler
 *
 *    hal_rtl_gdma_memcpy_irq_handler is the memcpy irq handler function.
 *    Once GDMA identifies which channel triggers interrupt, it then enters this function to jump to callback function of this channel.
 *    The IRQ flow is shown below:
 *    GDMA_IRQHandler(This function is reigstered to NVIC IRQ vector)-> hal_rtl_gdma_memcpy_irq_handler
 *      -> channel callback function(callback function carried by hal_rtl_gdma_memcpy_irq_hook).
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_memcpy_irq_handler(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	volatile u32 addr;
	volatile u32 len;

	addr = phal_gdma_adaptor->ch_dar;
	len = phal_gdma_adaptor->gdma_ctl.block_size * (1 << phal_gdma_adaptor->gdma_ctl.src_tr_width);

	// D-Cache sync (Invalidate)
	if (phal_gdma_adaptor->dcache_invalidate_by_addr != NULL) {
		phal_gdma_adaptor->dcache_invalidate_by_addr((uint32_t *)addr, (int32_t)len);
	}

	if (phal_gdma_adaptor->gdma_cb_func != NULL) {
		phal_gdma_adaptor->gdma_cb_func((void *)phal_gdma_adaptor->gdma_cb_para);
	}
}

/** \brief Description of GDMA0_IRQHandler
 *
 *    GDMA0_IRQHandler is used to handle GDMA0 interrupt.
 *    This function is the first function being called when the interrupt is triggered.
 *    It is also the function to be registered in NVIC.
 *    Since all channels share the same interrupt signal, this function will read the vendor register to identify which
 *    channel triggers the interrupt. It then jump to the corresponding channel irq handler function.
 *
 *   \param void.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void GDMA0_IRQHandler(void)
{
	phal_gdma_adaptor_t phal_gdma_adaptor;
	u8 i = 0;
	u8 gdma0_isr = 0;;

#if !defined (CONFIG_BUILD_NONSECURE)
	VNDR_S_TypeDef *vdr = (VNDR_S_TypeDef *) VDR_S;

	hal_rtl_irq_clear_pending(SGDMA0_IRQn);
	gdma0_isr = vdr->VNDR_S_REG_SECURE_GDMA0_ISR;
#else
	VNDR_TypeDef *vdr = (VNDR_TypeDef *) VDR;

	hal_rtl_irq_clear_pending(GDMA0_IRQn);
	gdma0_isr = vdr->VNDR_REG_GDMA0_ISR;
#endif
	DBG_GDMA_INFO("\r\nirq0 handler\r\n");

	for (i = 0; i < MAX_GDMA_CHNL; i++) {
		if (gdma0_isr & (1 << i)) {
			phal_gdma_adaptor = phal_gdma_group->phal_gdma_adaptor[0][i];

			///Clear pending isr
			hal_rtl_gdma_clean_chnl_isr((void *) phal_gdma_adaptor);
			hal_rtl_gdma_chnl_dis((void *) phal_gdma_adaptor);

			phal_gdma_adaptor->busy = 0;
			if (phal_gdma_adaptor->gdma_irq_func != NULL) {
				DBG_GDMA_INFO("chnl %x ISR \r\n", i);
				phal_gdma_adaptor->gdma_irq_func(phal_gdma_adaptor->gdma_irq_para);
			}
		}
	}
}

/** \brief Description of GDMA1_IRQHandler
 *
 *    GDMA1_IRQHandler is used to handle GDMA1 interrupt.
 *    This function is the first function being called when the interrupt is triggered.
 *    It is also the function to be registered in NVIC.
 *    Since all channels share the same interrupt signal, this function will read the vendor register to identify which
 *    channel triggers the interrupt. It then jump to the corresponding channel irq handler function.
 *
 *   \param void.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void GDMA1_IRQHandler(void)
{
	phal_gdma_adaptor_t phal_gdma_adaptor;
	u8 i = 0;
	u8 gdma1_isr = 0;;

#if !defined (CONFIG_BUILD_NONSECURE)
	VNDR_S_TypeDef *vdr = (VNDR_S_TypeDef *) VDR_S;

	hal_rtl_irq_clear_pending(SGDMA1_IRQn);
	gdma1_isr = vdr->VNDR_S_REG_SECURE_GDMA1_ISR;
#else
	VNDR_TypeDef *vdr = (VNDR_TypeDef *) VDR;

	hal_rtl_irq_clear_pending(GDMA1_IRQn);
	gdma1_isr  = vdr->VNDR_REG_GDMA1_ISR;
#endif
	DBG_GDMA_INFO("irq1 handler\r\n");

	for (i = 0; i < MAX_GDMA_CHNL; i++) {
		if (gdma1_isr & (1 << i)) {
			phal_gdma_adaptor = phal_gdma_group->phal_gdma_adaptor[1][i];

			///Clear pending isr
			hal_rtl_gdma_clean_chnl_isr((void *) phal_gdma_adaptor);
			hal_rtl_gdma_chnl_dis((void *) phal_gdma_adaptor);

			phal_gdma_adaptor->busy = 0;
			if (phal_gdma_adaptor->gdma_irq_func != NULL) {
				DBG_GDMA_INFO("chnl %x ISR \r\n", i);
				phal_gdma_adaptor->gdma_irq_func(phal_gdma_adaptor->gdma_irq_para);
			}
		}
	}
}

/** \brief Description of hal_rtl_gdma_irq_set_priority
 *
 *    hal_rtl_gdma_irq_set_priority is used to set irq priority of the target GDMA.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param u32 irq_priority:      The priority.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_irq_set_priority(phal_gdma_adaptor_t phal_gdma_adaptor, u32 irq_priority)
{
	hal_rtl_irq_set_priority(phal_gdma_adaptor->gdma_irq_num, irq_priority);
}

/** \brief Description of hal_rtl_gdma_irq_reg
 *
 *    hal_rtl_gdma_irq_reg is used to initialize irq number, channel irq handler function and parameter.
 *    Each periperhal which interacts with GDMA, including GDMA itself for memcpy, should call this function to initialize its GDMA channel irq handler function.
 *    The IRQ is enabled if it was not enabled before.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param irq_handler_t irq_handler:      The pointer of GDMA irq handler.
 *   \param void* irq_data:      The pointer of the irq parameter.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_irq_reg(phal_gdma_adaptor_t phal_gdma_adaptor, irq_handler_t irq_handler, void *irq_data)
{
	u8 gdma_index;

	phal_gdma_adaptor->gdma_irq_func = irq_handler;
	phal_gdma_adaptor->gdma_irq_para = irq_data;
	gdma_index = phal_gdma_adaptor->gdma_index;

	if (0 == gdma_index) {
#if !defined (CONFIG_BUILD_NONSECURE)
		phal_gdma_adaptor->gdma_irq_num = SGDMA0_IRQn;
#else
		phal_gdma_adaptor->gdma_irq_num = GDMA0_IRQn;
#endif
	} else {
#if !defined (CONFIG_BUILD_NONSECURE)
		phal_gdma_adaptor->gdma_irq_num = SGDMA1_IRQn;
#else
		phal_gdma_adaptor->gdma_irq_num = GDMA1_IRQn;
#endif
	}

	if (phal_gdma_group->chnl_in_use[gdma_index] == 0) {
		hal_rtl_irq_enable(phal_gdma_adaptor->gdma_irq_num);
	}

	(phal_gdma_group->chnl_in_use[gdma_index]) |=  1 << (phal_gdma_adaptor->ch_num);

	DBG_GDMA_INFO("hal_gdma_irq_reg: chnl_in_use[0] = %x\r\n", (phal_gdma_group->chnl_in_use[0]));
	DBG_GDMA_INFO("hal_gdma_irq_reg: chnl_in_use[1] = %x\r\n", (phal_gdma_group->chnl_in_use[1]));
}

/** \brief Description of hal_rtl_gdma_transfer_start
 *
 *    hal_rtl_gdma_transfer_start consists of several steps:
 *    Turn on GDMA IP-> Enable ISR(Unmask)->Set GDMA registers->Enable channel to start a transfer
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param u8 multi_blk_en:      Determine if multi-block mode is used.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_transfer_start(phal_gdma_adaptor_t phal_gdma_adaptor, u8 multi_blk_en)
{
	hal_rtl_gdma_isr_en(phal_gdma_adaptor);

	if (multi_blk_en) {
		hal_rtl_gdma_multi_block_init(phal_gdma_adaptor);
		hal_rtl_gdma_chnl_block_setting(phal_gdma_adaptor);
	} else {
		phal_gdma_adaptor->gdma_ctl.llp_src_en = 0;
		phal_gdma_adaptor->gdma_ctl.llp_dst_en = 0;
		phal_gdma_adaptor->gdma_cfg.reload_dst = 0;
		phal_gdma_adaptor->gdma_cfg.reload_src = 0;
		phal_gdma_adaptor->max_block_num = 1;
		hal_rtl_gdma_chnl_setting(phal_gdma_adaptor);
	}

	hal_rtl_gdma_chnl_en(phal_gdma_adaptor);
}


/** \brief Description of hal_rtl_gdma_group_init
 *
 *    hal_rtl_gdma_group_init is used to assign the address of gdma group adaptor and initialize global IRQ setting.
 *    GDMA group adoptor plays an important role to manage channel usage, the address of the group adaptor is carried via this function.
 *    The IRQ handler, IRQ number are fixed values and should not be modified, so we initialize these parameters before any other gdma functions.
 *
 *   \param phal_gdma_group_t pgdma_group:      The pointer of GDMA group adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_group_init(phal_gdma_group_t pgdma_group)
{
	phal_gdma_group = pgdma_group;
#if !defined (CONFIG_BUILD_NONSECURE)
	hal_rtl_irq_disable(SGDMA0_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(SGDMA0_IRQn, (uint32_t)GDMA0_IRQHandler);
	hal_rtl_irq_set_priority(SGDMA0_IRQn, SGDMA0_IRQPri);

	hal_rtl_irq_disable(SGDMA1_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(SGDMA1_IRQn, (uint32_t)GDMA1_IRQHandler);
	hal_rtl_irq_set_priority(SGDMA1_IRQn, SGDMA1_IRQPri);
#else
	hal_rtl_irq_disable(GDMA0_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(GDMA0_IRQn, (uint32_t)GDMA0_IRQHandler);
	hal_rtl_irq_set_priority(GDMA0_IRQn, GDMA0_IRQPri);

	hal_rtl_irq_disable(GDMA1_IRQn);
	__ISB();
	hal_rtl_irq_set_vector(GDMA1_IRQn, (uint32_t)GDMA1_IRQHandler);
	hal_rtl_irq_set_priority(GDMA1_IRQn, GDMA1_IRQPri);
#endif
}

/** \brief Description of hal_rtl_gdma_multi_block_init
 *
 *    hal_rtl_gdma_multi_block_init is used to configure linked list pointers under multi-block mode.
 *    This function automatically divides block size which is larger than MAX_DMA_BLOCK_SIZE into several blocks.
 *    These blocks are connected with each other sequentially via linked list structs.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_multi_block_init(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	u32 total_length;
	u8 block_num;
	u8 block_index;
	u8 tt_fc;
	u32 single_block_bytes;
	u32 src_blk_shift = 0;
	u32 dst_blk_shift = 0;
	pgdma_ch_lli_t pgdma_ch_lli;

	tt_fc = phal_gdma_adaptor->gdma_ctl.tt_fc;
	total_length = phal_gdma_adaptor->gdma_ctl.block_size;
	single_block_bytes = MAX_DMA_BLOCK_SIZE * (1 << (phal_gdma_adaptor->gdma_ctl.src_tr_width)) ;
	block_num = phal_gdma_adaptor->gdma_ctl.block_size / MAX_DMA_BLOCK_SIZE;

	if (phal_gdma_adaptor->gdma_ctl.block_size % MAX_DMA_BLOCK_SIZE) {
		block_num++;
	}

	DBG_GDMA_INFO("MAX_DMA_BLOCK_SIZE = %x, total_length = %x, single_block_bytes = %x, block_num = %x\n",
				  MAX_DMA_BLOCK_SIZE, total_length, single_block_bytes, block_num);

	if (block_num > MAX_MULTI_BLOCK_NUM) {
		DBG_GDMA_ERR("Data length is too long!Please separate into multiple DMA transfers\n");
	}

	phal_gdma_adaptor->max_block_num = block_num;
	phal_gdma_adaptor->gdma_ctl.llp_src_en = 1;
	phal_gdma_adaptor->gdma_ctl.llp_dst_en = 1;

	switch (tt_fc) {
	case TTFCMemToMem:
		src_blk_shift = single_block_bytes;
		dst_blk_shift = single_block_bytes;
		break;

	case TTFCMemToPeri:
		src_blk_shift = single_block_bytes;
		dst_blk_shift = 0;
		break;

	case TTFCPeriToMem:
		src_blk_shift = 0;
		dst_blk_shift = single_block_bytes;
		break;

	default:
		DBG_GDMA_ERR("Unknow Transfer Type!\n");
		break;
	}

	pgdma_ch_lli = phal_gdma_adaptor->pgdma_ch_lli;

	for (block_index = 0; block_index < block_num; block_index++) {
		DBG_GDMA_INFO("pgdma_ch_lli = %x\r\n", pgdma_ch_lli);

		pgdma_ch_lli->sarx = (u32)(phal_gdma_adaptor->ch_sar + block_index * src_blk_shift);
		pgdma_ch_lli->darx = (u32)(phal_gdma_adaptor->ch_dar + block_index * dst_blk_shift);

		if (block_index == block_num - 1) {
			pgdma_ch_lli->llpx = (u32) NULL;
			pgdma_ch_lli->ctlx_up = total_length - block_index * MAX_DMA_BLOCK_SIZE;
		} else {
			pgdma_ch_lli->llpx = (u32)((pgdma_ch_lli_t)pgdma_ch_lli + 1);
			pgdma_ch_lli->ctlx_up = MAX_DMA_BLOCK_SIZE;
		}

		DBG_GDMA_INFO("pgdma_ch_lli->llpx =%x\n", pgdma_ch_lli->llpx);
		pgdma_ch_lli ++;
	}

}

/** \brief Description of hal_rtl_gdma_memcpy_config
 *
 *    hal_rtl_gdma_memcpy_config is used to initialize relevant settings for memcpy transfer.
 *    source address, destination address and transfer length are stored in the gdma adaptor for later use.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *   \param void *pdest     :      The destination address.
 *   \param void *psrc      :      The source address.
 *   \param u32 len         :      The transfer length, the unit is byte.
 *
 *   \return hal_status_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_memcpy_config(phal_gdma_adaptor_t phal_gdma_adaptor, void *pdest, void *psrc, u32 len)
{
	if (phal_gdma_adaptor->busy) {
		DBG_GDMA_ERR("hal_gdma_memcpy : GDMA is busy\r\n");
		return HAL_ERR_PARA;
	}

	phal_gdma_adaptor->busy = 1;
	DBG_GDMA_INFO("hal_gdma_memcpy : src=0x%x dst=0x%x len=%d\r\n", psrc, pdest, len);

	phal_gdma_adaptor->gdma_ctl.src_msize   = MsizeEight;
	phal_gdma_adaptor->gdma_ctl.src_tr_width = TrWidthFourBytes;
	phal_gdma_adaptor->gdma_ctl.dest_msize = MsizeEight;
	phal_gdma_adaptor->gdma_ctl.dst_tr_width = TrWidthFourBytes;
	phal_gdma_adaptor->gdma_ctl.block_size = len >> 2;

	if (len & 0x03) {
		phal_gdma_adaptor->gdma_ctl.src_tr_width = TrWidthOneByte;
		phal_gdma_adaptor->gdma_ctl.block_size = len;
	}

	phal_gdma_adaptor->ch_sar = (u32)psrc;
	phal_gdma_adaptor->ch_dar = (u32)pdest;

	return HAL_OK;
}

/** \brief Description of hal_rtl_gdma_linked_list_block_init
 *
 *    hal_rtl_gdma_linked_list_block_init is used to configure linked list struct for linked list block transer.
 *    This function allows addresses between different blocks to be non-sequential either for source side or for destination side.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor   :      The pointer of GDMA adaptor.
 *   \param phal_gdma_block_t phal_gdma_block       :      The pointer of gdma block struct.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_linked_list_block_init(phal_gdma_adaptor_t phal_gdma_adaptor, phal_gdma_block_t phal_gdma_block)
{
	u8 block_number;
	u8 block_index;
	pgdma_ch_lli_t pgdma_ch_lli;

	block_number = phal_gdma_adaptor->block_num;
	phal_gdma_adaptor->gdma_ctl.llp_src_en = 1;
	phal_gdma_adaptor->gdma_ctl.llp_dst_en = 1;
	phal_gdma_adaptor->gdma_ctl.src_msize   = MsizeEight;
	phal_gdma_adaptor->gdma_ctl.src_tr_width = TrWidthOneByte;
	phal_gdma_adaptor->gdma_ctl.dest_msize = MsizeEight;
	phal_gdma_adaptor->gdma_ctl.dst_tr_width = TrWidthFourBytes;

	pgdma_ch_lli = phal_gdma_adaptor->pgdma_ch_lli;

	for (block_index = 0; block_index < block_number; block_index++) {
		DBG_GDMA_INFO("pgdma_ch_lli = %x\r\n", pgdma_ch_lli);

		pgdma_ch_lli->sarx = phal_gdma_block[block_index].src_addr;
		pgdma_ch_lli->darx = phal_gdma_block[block_index].dst_addr;
		pgdma_ch_lli->ctlx_up = phal_gdma_block[block_index].block_length;

		if (block_index == block_number - 1) {
			pgdma_ch_lli->llpx = (u32) NULL;
			phal_gdma_adaptor->gdma_ctl.block_size = phal_gdma_block[block_index].dst_addr - phal_gdma_block[0].dst_addr + phal_gdma_block[block_index].block_length;
		} else {
			pgdma_ch_lli->llpx = (u32)((pgdma_ch_lli_t)pgdma_ch_lli + 1);
		}
		DBG_GDMA_INFO("pgdma_ch_lli->llpx =%x\n", pgdma_ch_lli->llpx);

		pgdma_ch_lli++;
	}
}

/** \brief Description of hal_rtl_gdma_linked_list_block_config
 *
 *    hal_rtl_gdma_linked_list_block_config is used to initialize linked list block transfer.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor   :      The pointer of GDMA adaptor.
 *   \param phal_gdma_block_t phal_gdma_block       :      The pointer of gdma block struct.
 *   \param u8 block_num                            :      The number of blocks.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_linked_list_block_config(phal_gdma_adaptor_t phal_gdma_adaptor, phal_gdma_block_t phal_gdma_block, u8 block_num)
{
	phal_gdma_adaptor->block_num = block_num;

	if (phal_gdma_adaptor->busy) {
		DBG_GDMA_ERR("hal_gdma_multi_block_memcpy : GDMA is busy\r\n");
		return HAL_ERR_PARA;
	}

	if (block_num > MAX_MULTI_BLOCK_NUM) {
		DBG_GDMA_ERR("The maximum block number is 32!\r\n");
	}

	phal_gdma_adaptor->busy = 1;
	phal_gdma_adaptor->max_block_num = block_num;
	phal_gdma_adaptor->ch_sar = phal_gdma_block[0].src_addr;
	phal_gdma_adaptor->ch_dar = phal_gdma_block[0].dst_addr;

	hal_rtl_gdma_linked_list_block_init(phal_gdma_adaptor, phal_gdma_block);
	return HAL_OK;
}

/** \brief Description of hal_rtl_gdma_abort
 *
 *    hal_rtl_gdma_abort is used to stop gdma transfer.
 *    Once the transfer is terminated, it cannot resume.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_abort(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	u32 fail_count = 0;
	GDMA_CH_TypeDef *chnl_dev = phal_gdma_adaptor->chnl_dev;

	chnl_dev->GDMA_CH_CFG_LOW |= GDMA_BIT_CH_CH_SUSP;

	while (1) {
		if (chnl_dev->GDMA_CH_CFG_LOW & GDMA_BIT_CH_INACTIVE) {
			break;
		} else {
			fail_count++;
			hal_delay_us(100);
			if (fail_count == 10) {
				DBG_GDMA_ERR("Transfer cannot stop!\n");
				break;
			}
		}
	}

	phal_gdma_adaptor->busy = 0;
}

/** \brief Description of hal_rtl_gdma_chnl_reset
 *
 *    hal_rtl_gdma_chnl_reset is used to reset gdma channel.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_reset(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	GDMA_TypeDef *gdma_dev = phal_gdma_adaptor->gdma_dev;
	u32 ch_en = phal_gdma_adaptor->ch_en;

	/*Set reset bit to reset the channel*/
	gdma_dev->GDMA_CH_RESET_REG = ch_en & 0xFFFF;

	/*Clear reset bit to finish the reset process*/
	gdma_dev->GDMA_CH_RESET_REG = ch_en & 0xFF00;

	DBG_GDMA_INFO("channel %d is reset.", phal_gdma_adaptor->ch_num);
}

#if !defined (CONFIG_BUILD_NONSECURE)

/** \brief Description of hal_rtl_gdma_chnl_free
 *
 *    hal_rtl_gdma_chnl_free is used to release channel completely.
 *    The channel is unregistered and its IRQ would also be disabled.
 *    At the end of this function, the gdma adaptor is marked to have no channel.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_chnl_free(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	/*Mask the corresponding ISR bits of the cahnnel*/
	hal_rtl_gdma_isr_dis(phal_gdma_adaptor);

	/*Return the channel*/
	hal_rtl_gdma_chnl_unregister(phal_gdma_adaptor);

	hal_rtl_gdma_chnl_irq_free(phal_gdma_adaptor);

	phal_gdma_adaptor->have_chnl = 0;
}

/** \brief Description of hal_rtl_gdma_chnl_alloc
 *
 *    hal_rtl_gdma_chnl_alloc is used to determine viable channels and check which channel can be allocated one by one.
 *    If the channel is verified by hal_gdma_chnl_register that can be allocated,
 *    the channel is assigned to the gdma adaptor.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor   :      The pointer of GDMA adaptor.
 *   \param u8 multi_blk_en                         :      The parameter to decide whether to allocate channels for multi-block mode or not.
 *
 *   \return hal_status_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_chnl_alloc(phal_gdma_adaptor_t phal_gdma_adaptor, u8 multi_blk_en)
{
	phal_gdma_chnl_t pgdma_chnl;

	if (phal_gdma_adaptor->have_chnl == 1) {
		if (multi_blk_en && (phal_gdma_adaptor->ch_num < 4)) {
			hal_rtl_gdma_chnl_free(phal_gdma_adaptor);
		}
	}

	if (phal_gdma_adaptor->have_chnl == 0) {
		if (multi_blk_en) {
			pgdma_chnl = (phal_gdma_chnl_t)(&gdma_multi_block_chnl_option_rom[0]);
		} else {
			pgdma_chnl = (phal_gdma_chnl_t)(&gdma_chnl_option_rom[0]);
		}

		while (pgdma_chnl->gdma_indx <= MAX_GDMA_INDX) {
			if (hal_rtl_gdma_chnl_register(pgdma_chnl->gdma_indx, pgdma_chnl->gdma_chnl) == HAL_OK) {
				phal_gdma_adaptor->gdma_index = pgdma_chnl->gdma_indx;
				phal_gdma_adaptor->ch_num = pgdma_chnl->gdma_chnl;
				phal_gdma_adaptor->have_chnl = 1;
				DBG_GDMA_INFO("GDMA Index = %d, Channel = %d\n", phal_gdma_adaptor->gdma_index, phal_gdma_adaptor->ch_num);
				// this gdma channel is available
				break;
			}
			pgdma_chnl += 1;
		}

		if (pgdma_chnl->gdma_indx > MAX_GDMA_INDX) {
			return HAL_ERR_PARA;
		}
	}

	phal_gdma_adaptor->dcache_invalidate_by_addr = rtl_dcache_invalidate_by_addr;
	phal_gdma_adaptor->dcache_clean_by_addr = rtl_dcache_clean_by_addr;

	return HAL_OK;
}

/** \brief Description of hal_rtl_gdma_memcpy_init
 *
 *    hal_rtl_gdma_memcpy_init is used to initialize memory copy transfer by gdma.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor   :      The pointer of GDMA adaptor.
 *   \param pgdma_ch_lli_t pgdma_ch_lli             :      The pointer of gdma linked list struct for sequential blocks.
 *
 *   \return BOOL: 1: Succeed 0: Fail.
 */
SECTION_GDMA_TEXT
BOOL hal_rtl_gdma_memcpy_init(phal_gdma_adaptor_t phal_gdma_adaptor, pgdma_ch_lli_t pgdma_ch_lli)
{
	_memset((void *)phal_gdma_adaptor, 0, sizeof(hal_gdma_adaptor_t));

	phal_gdma_adaptor->gdma_ctl.tt_fc = TTFCMemToMem;
	phal_gdma_adaptor->gdma_isr_type = (TransferType | ErrType);
	phal_gdma_adaptor->gdma_ctl.int_en      = 1;
	phal_gdma_adaptor->gdma_ctl.sinc = IncType;
	phal_gdma_adaptor->gdma_ctl.dinc = IncType;
	phal_gdma_adaptor->busy = 0;
	phal_gdma_adaptor->have_chnl = 0;

	if (pgdma_ch_lli != NULL) {
		phal_gdma_adaptor->pgdma_ch_lli = pgdma_ch_lli;
	}

	if (hal_rtl_gdma_chnl_alloc(phal_gdma_adaptor, MultiBlkDis) != HAL_OK) {
		DBG_GDMA_ERR("Cannot Allocate Channel !\n");
		return _FALSE;
	} else {
		hal_rtl_gdma_chnl_init(phal_gdma_adaptor);
	}

	hal_rtl_gdma_irq_reg(phal_gdma_adaptor, (irq_handler_t) hal_rtl_gdma_memcpy_irq_handler, phal_gdma_adaptor);
	return _TRUE;
}

/** \brief Description of hal_rtl_gdma_memcpy_deinit
 *
 *    hal_rtl_gdma_memcpy_deinit is used to deinit the gdma channel after the memcpy is complete and no longer needed.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor:      The pointer of GDMA adaptor.
 *
 *   \return void.
 */
SECTION_GDMA_TEXT
void hal_rtl_gdma_memcpy_deinit(phal_gdma_adaptor_t phal_gdma_adaptor)
{
	phal_gdma_adaptor->pgdma_ch_lli = NULL;
	hal_rtl_gdma_chnl_free(phal_gdma_adaptor);
}



/** \brief Description of hal_rtl_gdma_memcpy
 *
 *    hal_rtl_gdma_memcpy is used to configure gdma setting for the current transfer and start the transfer at the end of the function.
 *
 *   \param phal_gdma_adaptor_t phal_gdma_adaptor   :      The pointer of GDMA adaptor.
 *   \param void *pdest                             :      The destination address.
 *   \param void *psrc                              :      The source address.
 *   \param u32 len                                 :      The transfer length, the unit is byte.
 *
 *   \return hal_status_t.
 */
SECTION_GDMA_TEXT
hal_status_t hal_rtl_gdma_memcpy(phal_gdma_adaptor_t phal_gdma_adaptor, void *pdest, void *psrc, u32 len)
{
	u8 multi_block_en = MultiBlkDis;

	hal_rtl_gdma_memcpy_config(phal_gdma_adaptor, pdest, psrc, len);

	if (phal_gdma_adaptor->gdma_ctl.block_size > MAX_DMA_BLOCK_SIZE) {
		multi_block_en = MultiBlkEn;
		if (phal_gdma_adaptor->ch_num < 4) {
			if (HAL_OK != hal_rtl_gdma_chnl_alloc(phal_gdma_adaptor, multi_block_en)) {
				DBG_GDMA_ERR("Cannot Allocate Channel !\n");
				return HAL_ERR_HW;
			} else {
				hal_rtl_gdma_chnl_init(phal_gdma_adaptor);
			}

			hal_rtl_gdma_irq_reg(phal_gdma_adaptor, (irq_handler_t) hal_rtl_gdma_memcpy_irq_handler, phal_gdma_adaptor);
		}
	}

	if (phal_gdma_adaptor->dcache_clean_by_addr != NULL) {
		phal_gdma_adaptor->dcache_clean_by_addr((uint32_t *) psrc, (int32_t) len);
	}

	hal_rtl_gdma_transfer_start(phal_gdma_adaptor, multi_block_en);

	return HAL_OK;
}
#endif
/** *@} */ /* End of group hal_rtl_gdma_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** *@} */

