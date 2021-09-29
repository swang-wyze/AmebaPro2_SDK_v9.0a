/**************************************************************************//**
 * @file     rom_i2c.c
 * @brief    This file implements the I2C HAL functions.
 *
 * @version  V1.00
 * @date     2021-06-29
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

#ifdef CONFIG_I2C_EN

extern void *_memset(void *dst0, int Val,   size_t length);

#define SECTION_I2C_TEXT            SECTION(".rom.hal_i2c.text")
#define SECTION_I2C_DATA            SECTION(".rom.hal_i2c.data")
#define SECTION_I2C_RODATA          SECTION(".rom.hal_i2c.rodata")
#define SECTION_I2C_RODATA          SECTION(".rom.hal_i2c.rodata")
#define SECTION_I2C_BSS             SECTION(".rom.hal_i2c.bss")
#define SECTION_I2C_STUBS           SECTION(".rom.hal_i2c.stubs")

/**
 * @addtogroup hs_hal_i2c I2C
 * @{
 */

SECTION_I2C_BSS
hal_pin_map i2c_scl_pin_map[] = {
	{PIN_A0,  PID_I2C0},
	{PIN_B0, PID_I2C0},
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_D15, PID_I2C1},
	{PIN_F2, PID_I2C1},
	{PIN_E7, PID_I2C2},
	{PIN_E9, PID_I2C2},
	{PIN_D14, PID_I2C3},
#else
	{PIN_D19, PID_I2C1},
	{PIN_F1, PID_I2C1},
	{PIN_E3, PID_I2C2},
	{PIN_E5, PID_I2C2},
	{PIN_D12, PID_I2C3},
#endif
	{0xFF,    0xFF} // end of table
};

SECTION_I2C_BSS
hal_pin_map i2c_sda_pin_map[] = {
	{PIN_A1,  PID_I2C0},
	{PIN_B1, PID_I2C0},
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	{PIN_D16, PID_I2C1},
	{PIN_F3, PID_I2C1},
	{PIN_E8, PID_I2C2},
	{PIN_E10, PID_I2C2},
	{PIN_D12, PID_I2C3},
#else
	{PIN_D20, PID_I2C1},
	{PIN_F2, PID_I2C1},
	{PIN_E4, PID_I2C2},
	{PIN_E6, PID_I2C2},
	{PIN_D10, PID_I2C3},
#endif
	{0xFF,    0xFF} // end of table
};

SECTION_I2C_BSS
volatile hal_i2c_group_adapter_t i2c_gadapter;

SECTION_I2C_BSS
volatile uint8_t hal_i2c_comm_irq_reg;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_i2c_rom_func I2C HAL ROM APIs.
 * @ingroup hs_hal_i2c
 * @{
 * @brief The I2C HAL ROM APIs. The user application(in RAM space) should not call these APIs directly.
 *        There is another set of I2C HAL APIs in the RAM space is provided for the user application.
 */

/**
  * \brief The stubs functions table to exports I2C HAL functions in ROM.
*/
SECTION_I2C_STUBS const hal_i2c_func_stubs_t hal_i2c_stubs = {
	.hal_i2c_scl_pin_map        = &i2c_scl_pin_map[0],
	.hal_i2c_sda_pin_map        = &i2c_sda_pin_map[0],
	.hal_i2c_group_adpt         = (hal_i2c_group_adapter_t *) &i2c_gadapter,
	.hal_i2c_timeout_chk        = hal_rtl_i2c_timeout_chk,
	.hal_i2c_chk_mod            = hal_rtl_i2c_chk_mod,
	.hal_i2c_pure_init          = hal_rtl_i2c_pure_init,
	.hal_i2c_pure_deinit        = hal_rtl_i2c_pure_deinit,
	.hal_i2c_en_ctrl            = hal_rtl_i2c_en_ctrl,
	.hal_i2c_set_clk            = hal_rtl_i2c_set_clk,
	.hal_i2c_intr_ctrl          = hal_rtl_i2c_intr_ctrl,
	.hal_i2c_wr                 = hal_rtl_i2c_wr,
	.hal_i2c_mst_send_rdcmd     = hal_rtl_i2c_mst_send_rdcmd,
	.hal_i2c_dma_ctrl           = hal_rtl_i2c_dma_ctrl,
	.hal_i2c_mst_restr_ctrl     = hal_rtl_i2c_mst_restr_ctrl,
	.hal_i2c_mst_gc_sb_ctrl     = hal_rtl_i2c_mst_gc_sb_ctrl,
	.hal_i2c_slv_no_ack_ctrl    = hal_rtl_i2c_slv_no_ack_ctrl,
	.hal_i2c_slv_no_ack_sts     = hal_rtl_i2c_slv_no_ack_sts,
	.hal_i2c_slv_ack_gc_ctrl    = hal_rtl_i2c_slv_ack_gc_ctrl,
	.hal_i2c_slv_ack_gc_sts     = hal_rtl_i2c_slv_ack_gc_sts,
	.hal_i2c_slv_to_slp         = hal_rtl_i2c_slv_to_slp,
	.hal_i2c_slv_chk_mst_wr     = hal_rtl_i2c_slv_chk_mst_wr,
	.hal_i2c_slv_chk_rd_req     = hal_rtl_i2c_slv_chk_rd_req,
	.hal_i2c_pin_init           = hal_rtl_i2c_pin_init,
	.hal_i2c_pin_deinit         = hal_rtl_i2c_pin_deinit,
	.hal_i2c_init               = hal_rtl_i2c_init,
	.hal_i2c_deinit             = hal_rtl_i2c_deinit,
	.hal_i2c_load_default       = hal_rtl_i2c_load_default,
	.hal_i2c_set_tar            = hal_rtl_i2c_set_tar,
	.hal_i2c_send_addr_retry    = hal_rtl_i2c_send_addr_retry,
	.hal_i2c_send_poll          = hal_rtl_i2c_send_poll,
	.hal_i2c_send_intr          = hal_rtl_i2c_send_intr,
	//.hal_i2c_send_dma_init      = hal_rtl_i2c_send_dma_init,
	//.hal_i2c_send_dma_deinit    = hal_rtl_i2c_send_dma_deinit,
	//.hal_i2c_send_dma           = hal_rtl_i2c_send_dma,
	//.hal_i2c_send               = hal_rtl_i2c_send,
	.hal_i2c_recv_addr_retry    = hal_rtl_i2c_recv_addr_retry,
	.hal_i2c_recv_poll          = hal_rtl_i2c_recv_poll,
	.hal_i2c_recv_intr          = hal_rtl_i2c_recv_intr,
	//.hal_i2c_recv_dma_init      = hal_rtl_i2c_recv_dma_init,
	//.hal_i2c_recv_dma_deinit    = hal_rtl_i2c_recv_dma_deinit,
	//.hal_i2c_recv_dma           = hal_rtl_i2c_recv_dma,
	//.hal_i2c_receive            = hal_rtl_i2c_receive,
	.hal_i2c_set_sar            = hal_rtl_i2c_set_sar,
	.hal_i2c_slv_recv_poll      = hal_rtl_i2c_slv_recv_poll,
	.hal_i2c_slv_recv_intr      = hal_rtl_i2c_slv_recv_intr,
	//.hal_i2c_slv_recv_dma       = hal_rtl_i2c_slv_recv_dma,
	//.hal_i2c_slv_recv           = hal_rtl_i2c_slv_recv,
	.hal_i2c_slv_send_poll      = hal_rtl_i2c_slv_send_poll,
	.hal_i2c_slv_send_intr      = hal_rtl_i2c_slv_send_intr,
	//.hal_i2c_slv_send_dma       = hal_rtl_i2c_slv_send_dma,
	//.hal_i2c_slv_send           = hal_rtl_i2c_slv_send,
	.hal_i2c_dma_irq_handler    = hal_rtl_i2c_dma_irq_handler,
	.hal_i2c_mst_irq_handler    = hal_rtl_i2c_mst_irq_handler,
	.hal_i2c_slv_irq_handler    = hal_rtl_i2c_slv_irq_handler,
};

/** \brief Description of hal_rtl_i2c_timeout_chk
 *
 *    hal_rtl_i2c_timeout_chk is used to check if I2C operation is time-out.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter: Pointer to I2C control adapter.
 *   \param[in] uint32_t start_cnt: Start system time count.
 *
 *   \return uint8_t:       Time out result \n\r\t
 *                          0: NOT time-out, 1: Time-out.
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_timeout_chk(hal_i2c_adapter_t *phal_i2c_adapter, uint32_t start_cnt)
{
	uint32_t time_out_cnt = 0;
	uint32_t curr_cnt = 0;
	uint32_t expire_cnt = 0;

	if ((phal_i2c_adapter->pltf_dat.tr_time_out != HP_I2C_TIMEOUT_DISABLE) &&
		(phal_i2c_adapter->pltf_dat.tr_time_out != HP_I2C_TIMEOUT_ENDLESS)) {
		time_out_cnt = phal_i2c_adapter->pltf_dat.tr_time_out;
		curr_cnt = hal_read_cur_time();

		if (start_cnt < curr_cnt) {
			expire_cnt =  curr_cnt - start_cnt;//(0xFFFFFFFF - curr_cnt) + start_cnt;
		} else {
			expire_cnt = (0xFFFFFFFF - curr_cnt) + start_cnt;//start_cnt - curr_cnt;
		}

		if (time_out_cnt < expire_cnt) {
			return 1;//time-out
		} else {
			return 0;
		}
	} else if (phal_i2c_adapter->pltf_dat.tr_time_out == HP_I2C_TIMEOUT_DISABLE) {
		return 1;   //time-out
	}

	return 0;
}

/** \brief Description of hal_rtl_i2c_dma_irq_handler
 *
 *    hal_rtl_i2c_dma_irq_handler is used to deal with I2C master DMA related interrupt.
 *
 *   \param[in] void *data:                 Pointer to I2C control adapter.
 *   \return void
 *
 */
SECTION_I2C_TEXT void hal_rtl_i2c_dma_irq_handler(void *data)
{
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)data;
	i2c_tx_info_t   *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	i2c_rx_info_t   *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)(phal_i2c_adapter->init_dat.reg_base);
	uint32_t tx_abrt_src;
	uint32_t dma_tr_len;

	DBG_I2C_INFO("dma irq\n\r");

	if (phal_i2c_adapter->status == I2CStatusTxing) {
		DBG_I2C_INFO("i2c dma cmd: %x\n\r", pi2c_reg->I2C_DMA_CMD);
		DBG_I2C_INFO("tx dma done\n\r");
		dma_tr_len = hal_rtl_gdma_query_send_bytes((void *)(phal_i2c_adapter->tx_dma_dat.padaptor));
		DBG_I2C_INFO("tx data len: %d\n\r", dma_tr_len);

		/* disable tx dma first */
		HAL_CLEAR_BIT(pi2c_reg->I2C_DMA_CR, I2C_BIT_DMA_TDMAE);
		//if (pi2c_reg->dma_mod == I2CDmaDwc) {
		//    DBG_I2C_INFO("Disable DMA in Dwc mode\n\r");
		//    hal_i2c_dma_ctrl_rom(phal_i2c_adapter, I2CForceDisable);
		//}

		/* This part is done by GDMA HAL */
		//hal_gdma_clean_chnl_isr_rom((void*)(phal_i2c_adapter->tx_dma_dat.padaptor));
		//hal_gdma_chnl_dis_rom((void*)(phal_i2c_adapter->tx_dma_dat.padaptor));

		/*wait for tx fifo empty*/
		while (!(pi2c_reg->I2C_STS & I2C_BIT_STS_TFE)) {
			if (pi2c_reg->I2C_INTR_STAT & I2C_BIT_INTR_TX_ABRT) {
				tx_abrt_src = pi2c_reg->I2C_TX_ABRT_SRC;
				DBG_I2C_ERR("tx failed after TX DMA complete\n\r");
				DBG_I2C_ERR("tx abrt src:%08x\n\r", tx_abrt_src);
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}
			}
		}

		/* update buf and length */
		pi2c_tx_info->len = phal_i2c_adapter->tx_dma_dat.dat_len - dma_tr_len;
		pi2c_tx_info->buf += dma_tr_len;

		/* invoke user callback */
		if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
			phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
		}

		/* clear interrupt mask */
		pi2c_reg->I2C_INTR_MSK &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_OVER));
		phal_i2c_adapter->status = I2CStatusIdle;
		//pi2c_tx_info->buf += pi2c_tx_info->len;
	} else if (phal_i2c_adapter->status == I2CStatusRxing) {
		DBG_I2C_INFO("i2c dma cmd: %x\n\r", pi2c_reg->I2C_DMA_CMD);
		DBG_I2C_INFO("rx dma done\n\r");
		DBG_I2C_INFO("original rx buf: 0x%x, len: 0x%x\n\r", phal_i2c_adapter->rx_dat.buf, phal_i2c_adapter->rx_dat.len);
		dma_tr_len = hal_rtl_gdma_query_recv_bytes((void *)(phal_i2c_adapter->rx_dma_dat.padaptor));
		DBG_I2C_INFO("rx data len: %d\n\r", dma_tr_len);
		//phal_gdma_adaptor = (phal_gdma_adaptor_t)(phal_i2c_adapter->rx_dma_dat.padaptor);
		//if (pi2c_reg->dma_mod == I2CDmaDwc) {
		//    DBG_I2C_INFO("Disable DMA in Dwc mode\n\r");
		//    hal_rtl_i2c_dma_ctrl(phal_i2c_adapter, I2CForceDisable);
		//}

		/* disable rx dma */
		HAL_CLEAR_BIT(pi2c_reg->I2C_DMA_CR, I2C_BIT_DMA_RDMAE);
		/* This part is done by GDMA HAL */
		//hal_rtl_gdma_clean_chnl_isr((void*)(phal_i2c_adapter->rx_dma_dat.padaptor));
		//hal_rtl_gdma_chnl_dis((void*)(phal_i2c_adapter->rx_dma_dat.padaptor));
		// D-Cache sync (Invalidate)
		if (is_dcache_enabled()) {
			if (phal_i2c_adapter->dcache_invalidate_by_addr != NULL) {
				phal_i2c_adapter->dcache_invalidate_by_addr((uint32_t *)phal_i2c_adapter->rx_dat.buf, (int32_t)phal_i2c_adapter->rx_dat.len);
			} else {
				DBG_I2C_ERR("D-Cache is enabled but invalidate function is NOT available when RX DMA done.\n\r");
			}
		}

		/* modify read command length, since dma_len register records the dma len */
		if (phal_i2c_adapter->mst_spe_func & I2CAddressRetry) {
			phal_i2c_adapter->rd_cmd_no -= (dma_tr_len - 1);
		} else {
			phal_i2c_adapter->rd_cmd_no -= dma_tr_len;
		}
		phal_i2c_adapter->rd_cmd_no -= dma_tr_len;
		pi2c_rx_info->len = phal_i2c_adapter->rx_dma_dat.dat_len - dma_tr_len;
		pi2c_rx_info->buf += dma_tr_len;

		/* invoke user callback function */
		if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
			phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
		}

		pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER));
		phal_i2c_adapter->status = I2CStatusIdle;
		/* modify read command length, since dma_len register records the dma len */
		//phal_i2c_adapter->rd_cmd_no -= pi2c_reg->dma_len;
		//pi2c_rx_info->buf += pi2c_rx_info->len;
	}
}

/** \brief Description of hal_rtl_i2c_mst_irq_handler
 *
 *    hal_rtl_i2c_mst_irq_handler is used to deal with I2C master related interrupt.
 *
 *   \param[in] void *data:                 Pointer to I2C control adapter.
 *   \return void
 *
 */
SECTION_I2C_TEXT void hal_rtl_i2c_mst_irq_handler(void *data)
{
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)data;
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)(phal_i2c_adapter->init_dat.reg_base);
	uint32_t ic_intr_sts_tmp;
	i2c_tx_info_t *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	i2c_rx_info_t *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	uint8_t  wr_ctrl = 0;
	uint32_t tx_abrt_src;

	ic_intr_sts_tmp = pi2c_reg->I2C_INTR_STAT;

	DBG_I2C_INFO("mst irq\n\r");
	DBG_I2C_INFO("intr sts: %x\n\r", ic_intr_sts_tmp);
	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_UNDER) {
		DBG_I2C_ERR("rx under\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_UNDER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorRxUnder;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_OVER) {
		DBG_I2C_ERR("rx over\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_OVER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorRxOver;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_FULL) {
		if ((phal_i2c_adapter->status == I2CStatusRxReady) || (phal_i2c_adapter->status == I2CStatusRxing)) {
			while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {
				phal_i2c_adapter->status = I2CStatusRxing;
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}

			if (pi2c_rx_info->len == 0) {
				pi2c_reg->I2C_INTR_MSK &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER
													   | I2C_BIT_M_RX_UNDER));
				if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
					phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
				}

				phal_i2c_adapter->status = I2CStatusIdle;
			}
		} else {
			if (phal_i2c_adapter->usr_cb.rx_full.cb != NULL) {
				phal_i2c_adapter->usr_cb.rx_full.cb((void *)phal_i2c_adapter->usr_cb.rx_full.dat);
			}
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_OVER) {
		DBG_I2C_ERR("tx over\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_OVER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorTxOver;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_EMPTY) {
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_INTR);
		if ((phal_i2c_adapter->status == I2CStatusTxReady) || (phal_i2c_adapter->status == I2CStatusTxing)) {
			//READ_CLEAR_I2C_REG(pi2c_reg, clr_intr);
			if (pi2c_tx_info->len == 0) {
				pi2c_reg->I2C_INTR_MSK &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_TX_OVER));
				if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
					phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
				}
				phal_i2c_adapter->status = I2CStatusIdle;
			} else {
				//while ((pi2c_tx_info->len) && (BIT_GET_IC_TFNF(pi2c_reg->status))) {
				while ((pi2c_tx_info->len) && (pi2c_reg->I2C_STS & I2C_BIT_STS_TFNF)) {
					phal_i2c_adapter->status = I2CStatusTxing;
					/* Wrtie data into I2C TX FIFO */
					wr_ctrl = 0;
					if (pi2c_tx_info->len == 1) {
						if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
							wr_ctrl = 2;    /* Set RESTART for the next byte */
						} else {
							if (pi2c_tx_info->mst_stop == 1) {
								wr_ctrl = 1;    /* Set STOP for the last byte */
							}
						}
					}

					hal_rtl_i2c_wr(phal_i2c_adapter, pi2c_tx_info->buf, 1, wr_ctrl);
					pi2c_tx_info->buf++;
					pi2c_tx_info->len--;
				}
			}
		} else if ((phal_i2c_adapter->status == I2CStatusRxReady) || (phal_i2c_adapter->status == I2CStatusRxing)) {
			//READ_CLEAR_I2C_REG(pi2c_reg, clr_intr);
			if (phal_i2c_adapter->rd_cmd_no > 0) {
				/* Wrtie cmd into I2C TX FIFO */
				wr_ctrl = 0;
				if (phal_i2c_adapter->rd_cmd_no == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_rx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
					DBG_I2C_INFO("rx, wr_ctrl: %x\n\r", wr_ctrl);
				}


				hal_rtl_i2c_mst_send_rdcmd(phal_i2c_adapter, 1, wr_ctrl);
				phal_i2c_adapter->rd_cmd_no--;
			} else if (phal_i2c_adapter->op_mode == I2CModeDMA) {
				HAL_CLEAR_BIT(pi2c_reg->I2C_INTR_MSK, I2C_BIT_M_TX_EMPTY);
			}
		} else {
			if (phal_i2c_adapter->usr_cb.tx_empty.cb != NULL) {
				phal_i2c_adapter->usr_cb.tx_empty.cb((void *)phal_i2c_adapter->usr_cb.tx_empty.dat);
			}
		}


	}

	//if (ic_intr_sts_tmp.b.rd_req) {
	//
	//}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_ABRT) {
		tx_abrt_src = pi2c_reg->I2C_TX_ABRT_SRC;
		DBG_I2C_ERR("tx abrt src:%08x\n\r", tx_abrt_src);


		if ((phal_i2c_adapter->status == I2CStatusTxReady) || (phal_i2c_adapter->status == I2CStatusTxing)) {
			switch (phal_i2c_adapter->op_mode) {
			case I2CModePoll:
				break;
			case I2CModeInterrupt:
				DBG_I2C_ERR("terminate tx op. after error\n\r");
				DBG_I2C_ERR("residual len: %d\n\r", phal_i2c_adapter->tx_dat.len);
				DBG_I2C_ERR("last dat: %x\n\r", *(phal_i2c_adapter->tx_dat.buf - 1));
				pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_TX_OVER));
				break;
			case I2CModeDMA:
				DBG_I2C_ERR("terminate dma op. after error\n\r");
				DBG_I2C_ERR("dma len:%08x\n\r", pi2c_reg->I2C_DMA_LEN);
				HAL_CLEAR_BIT(pi2c_reg->I2C_DMA_CR, I2C_BIT_DMA_TDMAE);
				hal_rtl_gdma_clean_chnl_isr((void *)(phal_i2c_adapter->tx_dma_dat.padaptor));
				hal_rtl_gdma_chnl_dis((void *)(phal_i2c_adapter->tx_dma_dat.padaptor));

				pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_OVER));
				phal_i2c_adapter->tx_dma_dat.ch_sts = I2CDmaChGot;
				pi2c_tx_info->buf += pi2c_tx_info->len;
				break;
			default:
				break;
			}
		}

		if ((phal_i2c_adapter->status == I2CStatusRxReady) || (phal_i2c_adapter->status == I2CStatusRxing)) {
			switch (phal_i2c_adapter->op_mode) {
			case I2CModePoll:
				break;
			case I2CModeInterrupt:
				DBG_I2C_ERR("terminate rx op. after error\n\r");
				DBG_I2C_ERR("not received len: %d\n\r", phal_i2c_adapter->rx_dat.len);
				DBG_I2C_ERR("last dat: %x\n\r", *(phal_i2c_adapter->rx_dat.buf - 1));
				pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER |
														I2C_BIT_M_RX_UNDER));
				break;
			case I2CModeDMA:
				DBG_I2C_ERR("terminate dma op. after error\n\r");

				HAL_CLEAR_BIT(pi2c_reg->I2C_DMA_CR, I2C_BIT_DMA_RDMAE);
				hal_rtl_gdma_clean_chnl_isr((void *)(phal_i2c_adapter->rx_dma_dat.padaptor));
				hal_rtl_gdma_chnl_dis((void *)(phal_i2c_adapter->rx_dma_dat.padaptor));

				pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_TX_ABRT | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER));
				phal_i2c_adapter->rx_dma_dat.ch_sts = I2CDmaChGot;
				/* modify read command length, since dma_len register records the dma len */
				phal_i2c_adapter->rd_cmd_no -= pi2c_reg->I2C_DMA_LEN;
				pi2c_rx_info->buf += pi2c_rx_info->len;
				break;
			default:
				break;
			}
		}

		phal_i2c_adapter->status = I2CStatusError;

		phal_i2c_adapter->err_type = I2CErrorTxAbort;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}

		phal_i2c_adapter->status = I2CStatusIdle;
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_ABRT);
		DBG_I2C_INFO("sts:%x\n\r", pi2c_reg->I2C_STS);
		DBG_I2C_INFO("raw:%x\n\r", pi2c_reg->I2C_RAW_INTR_STAT);
	}

	//if (ic_intr_sts_tmp.b.rx_done) {
	//
	//}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_ACT) {
		DBG_I2C_INFO("\n\ractivity\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ACT);
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_STRT_DET) {
		DBG_I2C_INFO("\n\rstart det\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STRT_DET);
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_STP_DET) {
		DBG_I2C_INFO("\n\rstop det\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STP_DET);
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_DMA_I2C_DONE) {
		DBG_I2C_INFO("\n\rdma done\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_DMA_DONE);
	}
//if (ic_intr_sts_tmp.b.gen_call) {
//
//}
}

/** \brief Description of hal_rtl_i2c_slv_irq_handler
 *
 *    hal_rtl_i2c_slv_irq_handler is used to deal with I2C slave related interrupt.
 *
 *   \param[in] void *data:                 Pointer to I2C control adapter.
 *   \return void
 *
 */
SECTION_I2C_TEXT void hal_rtl_i2c_slv_irq_handler(void *data)
{
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)data;
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)(phal_i2c_adapter->init_dat.reg_base);
	i2c_tx_info_t *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	i2c_rx_info_t *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	uint32_t ic_intr_sts_tmp;
	uint8_t  wr_ctrl = 0;
	uint32_t tx_abrt_src;
	uint32_t ic_fltr_tmp;

	DBG_I2C_INFO("slv irq\n\r");
	DBG_I2C_INFO("intr msk: %x\n\r", pi2c_reg->I2C_INTR_MSK);

	ic_intr_sts_tmp = pi2c_reg->I2C_INTR_STAT;
	DBG_I2C_INFO("ic_intr_sts_tmp : %x\n\r", ic_intr_sts_tmp);
	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_UNDER) {
		DBG_I2C_ERR("rx under\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_UNDER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorRxUnder;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_OVER) {
		DBG_I2C_ERR("rx over\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_OVER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorRxOver;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_FULL) {
		//i2c_real_recv_cnt++;
		//DBG_I2C_INFO("rx full:%d\n\r", pi2c_reg->rxflr);
		if ((phal_i2c_adapter->status == I2CStatusRxReady) || (phal_i2c_adapter->status == I2CStatusRxing)) {
			phal_i2c_adapter->status = I2CStatusRxing;
			while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {

				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}

			if (pi2c_rx_info->len == 0) {
				pi2c_reg->I2C_INTR_MSK &= (uint32_t)(~(I2C_BIT_M_STRT_DET | I2C_BIT_M_STP_DET | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER |
													   I2C_BIT_M_RX_UNDER));
				if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
					phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
				}

				phal_i2c_adapter->status = I2CStatusIdle;
			}
		} else {
			if (phal_i2c_adapter->usr_cb.rx_full.cb != NULL) {
				phal_i2c_adapter->usr_cb.rx_full.cb((void *)phal_i2c_adapter->usr_cb.rx_full.dat);
			}
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_OVER) {
		DBG_I2C_ERR("tx over\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_OVER);
		phal_i2c_adapter->status = I2CStatusError;
		phal_i2c_adapter->err_type = I2CErrorTxOver;
		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_EMPTY) {
		DBG_I2C_ERR("slv tx empty\n\r");
		HAL_CLEAR_BIT(pi2c_reg->I2C_INTR_MSK, I2C_BIT_M_TX_EMPTY);
		if (phal_i2c_adapter->usr_cb.tx_empty.cb != NULL) {
			phal_i2c_adapter->usr_cb.tx_empty.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RD_REQ) {
		DBG_I2C_INFO("rd req\n\r");
		if (phal_i2c_adapter->op_mode == I2CModeDMA) {
			READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RD_REQ);
		}

		if ((phal_i2c_adapter->status == I2CStatusTxReady) || (phal_i2c_adapter->status == I2CStatusTxing)) {
			READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RD_REQ);
			if (!pi2c_tx_info->len) {
				DBG_I2C_INFO("*** A read cmd to slv but its tx buf runs out.***\n\r");
				/*
				                pi2c_reg->intr_mask &= ~(BIT_M_RX_DONE | BIT_M_TX_ABRT | BIT_M_RD_REQ | BIT_M_TX_OVER);
				                if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
				                    phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
				                }

				                phal_i2c_adapter->status = I2CStatusIdle;
				*/
			} else {
				phal_i2c_adapter->status = I2CStatusTxing;
				/* Wrtie data into I2C TX FIFO */
				wr_ctrl = 0;
				if (pi2c_tx_info->len == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_tx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
				}
				hal_rtl_i2c_wr(phal_i2c_adapter, pi2c_tx_info->buf, 1, wr_ctrl);
				pi2c_tx_info->buf++;
				pi2c_tx_info->len--;
			}
		} else if (phal_i2c_adapter->status == I2CStatusIdle) {
			hal_rtl_i2c_slv_clear_for_mst_rd_cmd(phal_i2c_adapter);

			if (phal_i2c_adapter->usr_cb.rd_req.cb != NULL) {
				phal_i2c_adapter->usr_cb.rd_req.cb((void *)phal_i2c_adapter->usr_cb.rd_req.dat);
			}
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_TX_ABRT) {
		tx_abrt_src = pi2c_reg->I2C_TX_ABRT_SRC;
		DBG_I2C_ERR("tx abrt src:%08x\n\r", tx_abrt_src);
		phal_i2c_adapter->status = I2CStatusError;
		if (tx_abrt_src & I2C_BIT_SLVFLUSH_TXFIFO) {
			DBG_I2C_ERR("Slv TX FIFO is flushed when getting a read command.\n\r");
			phal_i2c_adapter->err_type = I2CErrorSlaveFlushFIFO;
		} else if (tx_abrt_src & I2C_BIT_SLV_ARBLOST) {
			DBG_I2C_ERR("Slv lost arbitration in TX\n\r");
			phal_i2c_adapter->err_type = I2CErrorSlaveLostArb;
		} else if (tx_abrt_src & I2C_BIT_SLVRD_INTX) {
			DBG_I2C_ERR("Slv gets data with RD cmd bit is set when TX\n\r");
			DBG_I2C_ERR("Data is still sent in this case\n\r");
			phal_i2c_adapter->err_type = I2CErrorSlaveRDCmdInTX;
		}

		if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
			phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
		}

		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_ABRT);
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_RX_DONE) {
		DBG_I2C_WARN("rx done\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_DONE);
		if ((phal_i2c_adapter->status == I2CStatusTxReady) || (phal_i2c_adapter->status == I2CStatusTxing)) {
			pi2c_reg->I2C_INTR_MSK &= (uint32_t)(~(I2C_BIT_M_RX_DONE | I2C_BIT_M_TX_ABRT | I2C_BIT_M_RD_REQ | I2C_BIT_M_TX_OVER));
			if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
				phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
			}

			phal_i2c_adapter->status = I2CStatusIdle;
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_ACT) {
		DBG_I2C_INFO("\n\ractivity\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ACT);
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_STRT_DET) {
		DBG_I2C_INFO("slv start det\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STRT_DET);
		HAL_CLEAR_BIT(pi2c_reg->I2C_INTR_MSK, I2C_BIT_M_STRT_DET);
		if ((phal_i2c_adapter->status == (volatile uint8_t)I2CStatusRxing)) {
			while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}

			pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_STRT_DET | I2C_BIT_M_STP_DET | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER));
			DBG_I2C_INFO("start, recv, l:%x\n\r", pi2c_rx_info->len);
			if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
				phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
			}

			phal_i2c_adapter->status = I2CStatusIdle;
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_STP_DET) {
		DBG_I2C_INFO("slv stop det\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STP_DET);
		if ((phal_i2c_adapter->status == I2CStatusRxReady) || (phal_i2c_adapter->status == I2CStatusRxing)) {
			while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}

			pi2c_reg->I2C_INTR_MSK  &= (uint32_t)(~(I2C_BIT_M_STRT_DET | I2C_BIT_M_STP_DET | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER));
			DBG_I2C_INFO("stop, recv, l:%x\n\r", pi2c_rx_info->len);
			if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
				phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
			}

			phal_i2c_adapter->status = I2CStatusIdle;
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_GEN_CALL) {
		DBG_I2C_INFO("general call\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_GEN_CALL);

		if (phal_i2c_adapter->usr_cb.gen_call.cb != NULL) {
			phal_i2c_adapter->usr_cb.gen_call.cb((void *)phal_i2c_adapter->usr_cb.gen_call.dat);
		}
	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_ADDR0_MATCH) {
		DBG_I2C_INFO("m 0\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ADDR_MATCH);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STP_DET);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STRT_DET);
		//if (BIT_GET_IC_SLP_CLK_GATED(pi2c_reg->sleep)) {
		if (pi2c_reg->I2C_SLP & I2C_BIT_SLP_CLK_GATED) {
			HAL_CLEAR_BIT(pi2c_reg->I2C_SLP, I2C_BIT_CLK_CTRL);
			DBG_I2C_INFO("clear sleep, %x\n\r", pi2c_reg->I2C_SLP);

		}
		HAL_CLEAR_BIT(pi2c_reg->I2C_INTR_MSK, I2C_BIT_M_ADDR0_MATCH);

		if (phal_i2c_adapter->usr_cb.wake.cb != NULL) {
			phal_i2c_adapter->usr_cb.wake.cb((void *)phal_i2c_adapter->usr_cb.wake.dat);
		}

		if (phal_i2c_adapter->init_dat.dig_fltr_en) {
			ic_fltr_tmp = pi2c_reg->I2C_FLTR;
			HAL_SET_BIT(ic_fltr_tmp, I2C_BIT_DIG_FLTR_EN);
			pi2c_reg->I2C_FLTR = ic_fltr_tmp;
		}

	}

	if (ic_intr_sts_tmp & I2C_BIT_INTR_ADDR1_MATCH) {
		DBG_I2C_INFO("\n\rm 1\n\r");
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ADDR_MATCH);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STP_DET);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STRT_DET);
		//if (BIT_GET_IC_SLP_CLK_GATED(pi2c_reg->sleep)) {
		if (pi2c_reg->I2C_SLP & I2C_BIT_SLP_CLK_GATED) {
			HAL_CLEAR_BIT(pi2c_reg->I2C_SLP, I2C_BIT_CLK_CTRL);
			DBG_I2C_INFO("clear sleep, %x\n\r", pi2c_reg->I2C_SLP);
		}
		HAL_CLEAR_BIT(pi2c_reg->I2C_INTR_MSK, I2C_BIT_M_ADDR1_MATCH);

		if (phal_i2c_adapter->usr_cb.wake.cb != NULL) {
			phal_i2c_adapter->usr_cb.wake.cb((void *)phal_i2c_adapter->usr_cb.wake.dat);
		}

		if (phal_i2c_adapter->init_dat.dig_fltr_en) {
			ic_fltr_tmp = pi2c_reg->I2C_FLTR;
			HAL_SET_BIT(ic_fltr_tmp, I2C_BIT_DIG_FLTR_EN);
			pi2c_reg->I2C_FLTR = ic_fltr_tmp;
		}
	}
}

/** \brief Description of I2C0_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
SECTION_I2C_TEXT void I2C0_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(I2C0_IRQn);
	if (i2c_gadapter.irq_fun[0] != NULL) {
		i2c_gadapter.irq_fun[0]((void *)i2c_gadapter.adapter[0]);
	}

	DBG_I2C_INFO("I2C0_IRQHandler\n\r");
}
/** \brief Description of I2C1_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
SECTION_I2C_TEXT void I2C1_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(I2C1_IRQn);
	if (i2c_gadapter.irq_fun[1] != NULL) {
		i2c_gadapter.irq_fun[1]((void *)i2c_gadapter.adapter[1]);
	}

	DBG_I2C_INFO("I2C1_IRQHandler\n\r");
}
/** \brief Description of I2C2_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
SECTION_I2C_TEXT void I2C2_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(I2C2_IRQn);
	if (i2c_gadapter.irq_fun[2] != NULL) {
		i2c_gadapter.irq_fun[2]((void *)i2c_gadapter.adapter[2]);
	}

	DBG_I2C_INFO("I2C2_IRQHandler\n\r");
}
/** \brief Description of I2C3_IRQHandler
 *
 *    I2C_IRQHandler is IRQ entry handler for all I2C.
 *
 *   \return void
 */
SECTION_I2C_TEXT void I2C3_IRQHandler(void)
{
	hal_rtl_irq_clear_pending(I2C3_IRQn);
	if (i2c_gadapter.irq_fun[3] != NULL) {
		i2c_gadapter.irq_fun[3]((void *)i2c_gadapter.adapter[3]);
	}

	DBG_I2C_INFO("I2C3_IRQHandler\n\r");
}

/** \brief Description of hal_rtl_i2c_chk_mod
 *
 *    hal_rtl_i2c_chk_mod is used to check I2C operation mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t:           Check mode status. \n\r
 *                              0: Slave mode\n\r
 *                              1: Master mode\n\r
 *                              2: Configuration error.\n\r
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_chk_mod(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint32_t ic_con_tmp;

	ic_con_tmp = phal_i2c_adapter->init_dat.reg_base->I2C_CON;

	if (((ic_con_tmp & I2C_BIT_MST_MOD) == 0) &&
		((ic_con_tmp & (I2C_BIT_SLV_DIS | I2C_BIT_SLV_DIS_1)) != (I2C_BIT_SLV_DIS | I2C_BIT_SLV_DIS_1))) {
		/* 0 for slave mode */
		return 0;
	} else if (((ic_con_tmp & I2C_BIT_MST_MOD) == 1) &&
			   ((ic_con_tmp & (I2C_BIT_SLV_DIS | I2C_BIT_SLV_DIS_1)) == (I2C_BIT_SLV_DIS | I2C_BIT_SLV_DIS_1))) {
		/* 1 for master mode */
		return 1;
	} else {
		/* 2 for error config */
		return 2;
	}
}

/** \brief Description of hal_rtl_i2c_pure_init
 *
 *    hal_rtl_i2c_pure_init is used to initialize I2C module.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t :      i2c enable status
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_pure_init(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint8_t  restart;
	uint32_t ic_value_tmp;
	i2c_init_dat_t *pi2c_init_data = (i2c_init_dat_t *) & (phal_i2c_adapter->init_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_con_tmp;
	uint32_t ic_tar_tmp;
	uint32_t ic_filter_tmp;

	DBG_I2C_INFO("hal_rtl_i2c_pure_init\n\r");

	restart = 0;

	/* Disable the IC first */
	hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CDisable);

	/* Master case*/
	if (pi2c_init_data->master) {
		/*RESTART MUST be set in these condition in Master mode.
		       But it might be NOT compatible in old slaves.*/
		if ((pi2c_init_data->addr_mod == I2CAddress10bit) || (pi2c_init_data->spd_mod == I2CHighSpeed)) {
			restart = 1;
		}

		ic_con_tmp = 0;
		HAL_SET_BIT(ic_con_tmp, I2C_BIT_SLV_DIS | I2C_BIT_SLV_DIS_1
					| (restart << I2C_SHIFT_RSTRT_EN)
					| (pi2c_init_data->addr_mod << I2C_SHIFT_MST_10BIT_ADDR)
					| (pi2c_init_data->spd_mod << I2C_SHIFT_SPD)
					| (pi2c_init_data->master << I2C_SHIFT_MST_MOD));
		pi2c_reg->I2C_CON = ic_con_tmp;
		DBG_I2C_INFO("Init master, con[%2x]: %x, %x\n\r", &(pi2c_reg->I2C_CON), pi2c_reg->I2C_CON, ic_con_tmp);
		/* To set target addr.*/
		ic_tar_tmp = 0;
		HAL_SET_BIT(ic_tar_tmp, (pi2c_init_data->addr_mod << I2C_SHIFT_TAR_10BIT_ADDR)
					| (pi2c_init_data->ack_addr0 << I2C_SHIFT_TAR));
		pi2c_reg->I2C_TAR = ic_tar_tmp;
		DBG_I2C_INFO("Init master, tar[%2x]: %x, %x\n\r", &(pi2c_reg->I2C_TAR), pi2c_reg->I2C_TAR, ic_tar_tmp);

		/* To Set I2C clock*/
		hal_rtl_i2c_set_clk(phal_i2c_adapter);

		/* Set master code */
		pi2c_reg->I2C_HS_MADDR = pi2c_init_data->hs_maddr;
		DBG_I2C_INFO("Init master, hs_maddr[%2x]: %x, %x\n\r", &(pi2c_reg->I2C_HS_MADDR), pi2c_reg->I2C_HS_MADDR, pi2c_init_data->hs_maddr);
	} else {
		ic_con_tmp = 0;
		HAL_SET_BIT(ic_con_tmp, (pi2c_init_data->master << I2C_SHIFT_SLV_DIS)
					| (pi2c_init_data->master << I2C_SHIFT_SLV_DIS_1)
					| (pi2c_init_data->addr_mod << I2C_SHIFT_SLV_10BIT_ADDR)
					| (pi2c_init_data->spd_mod << I2C_SHIFT_SPD)
					| (pi2c_init_data->master << I2C_SHIFT_MST_MOD));
		pi2c_reg->I2C_CON = ic_con_tmp;
		DBG_I2C_INFO("Init slave, con%d[%2x]: %x\n\r", pi2c_init_data->index, &pi2c_reg->I2C_CON, pi2c_reg->I2C_CON);

		/* To set slave addr. 0 */
		pi2c_reg->I2C_SAR = pi2c_init_data->ack_addr0;
		DBG_I2C_INFO("Init slave, sar%d[%2x]: %x\n\r", pi2c_init_data->index, &pi2c_reg->I2C_SAR, pi2c_reg->I2C_SAR);

		/* To set slave addr. 1 */
		pi2c_reg->I2C_SAR1 = pi2c_init_data->ack_addr1;
		DBG_I2C_INFO("Init slave, sar1%d[%2x]: %x\n\r", pi2c_init_data->index, &pi2c_reg->I2C_SAR1, pi2c_reg->I2C_SAR1);

		/* To Set I2C clock*/
		hal_rtl_i2c_set_clk(phal_i2c_adapter);


	}

	/* to set SDA hold time */
	ic_con_tmp = pi2c_reg->I2C_CON;
	if (((ic_con_tmp & I2C_MASK_SPD) >> I2C_SHIFT_SPD) == I2CStandardSpeed) {
		ic_value_tmp = pi2c_reg->I2C_SS_SCL_LCNT;
	} else if (((ic_con_tmp & I2C_MASK_SPD) >> I2C_SHIFT_SPD) == I2CFastSpeed) {
		ic_value_tmp = pi2c_reg->I2C_FS_SCL_LCNT;
	} else {
		ic_value_tmp = pi2c_reg->I2C_HS_SCL_LCNT;
	}

	if (pi2c_init_data->master) {
		if (pi2c_init_data->sda_hold > (ic_value_tmp - 2)) {
			ic_value_tmp = (ic_value_tmp - 2);
			if (ic_value_tmp < I2C_MST_SDA_HOLD_MIN) {
				ic_value_tmp = I2C_MST_SDA_HOLD_MIN;
			}
			pi2c_reg->I2C_SDA_HOLD = ic_value_tmp;
		} else {
			pi2c_reg->I2C_SDA_HOLD = pi2c_init_data->sda_hold;
		}
	} else {
		if (pi2c_init_data->sda_hold > (ic_value_tmp - 2)) {
			ic_value_tmp = (ic_value_tmp - 2);
			if (ic_value_tmp < I2C_SLV_SDA_HOLD_MIN) {
				ic_value_tmp = I2C_SLV_SDA_HOLD_MIN;
			}
			pi2c_reg->I2C_SDA_HOLD = ic_value_tmp;
		} else {
			pi2c_reg->I2C_SDA_HOLD = pi2c_init_data->sda_hold;
		}

		/* to set SDA setup time, only for slave-transmitter */
		pi2c_reg->I2C_SDA_SETUP = (ic_value_tmp >> 1);
	}

	DBG_I2C_INFO("sda_setup: %x\n\r", pi2c_reg->I2C_SDA_SETUP);
	DBG_I2C_INFO("sda_hold: %x\n\r", pi2c_reg->I2C_SDA_HOLD);
	//DBG_8195BL("sda_setup: %x\n\r", pi2c_reg->sda_setup);
	//DBG_8195BL("sda_hold: %x\n\r", pi2c_reg->sda_hold);
	/* To set default digital filter */
	ic_filter_tmp = 0;
	HAL_SET_BIT(ic_filter_tmp, (pi2c_init_data->dig_fltr_en << I2C_SHIFT_DIG_FLTR_EN)
				| (pi2c_init_data->dig_fltr_lvl << I2C_SHIFT_DIG_FLTR_DEG));
	pi2c_reg->I2C_FLTR  = ic_filter_tmp;

	/* To set TX_Empty Level */
	pi2c_reg->I2C_TX_TL = pi2c_init_data->ff_txtl;

	/* To set RX_Full Level */
	pi2c_reg->I2C_RX_TL = pi2c_init_data->ff_rxtl;

	DBG_I2C_INFO("pi2c_reg->tx_tl[%x]: %x\n\r", &pi2c_reg->I2C_TX_TL, pi2c_reg->I2C_TX_TL);
	DBG_I2C_INFO("pi2c_reg->rx_tl[%x]: %x\n\r", &pi2c_reg->I2C_RX_TL, pi2c_reg->I2C_RX_TL);
	/* To set DMA default TX/RX FIFO level */
	pi2c_reg->I2C_DMA_TDLR = 0x09;
	pi2c_reg->I2C_DMA_RDLR = 0x03;
	DBG_I2C_INFO("Init i2c dev, dma_tdlr%d[%2x]: %x\n\r", pi2c_init_data->index, &pi2c_reg->I2C_DMA_TDLR, pi2c_reg->I2C_DMA_TDLR);
	DBG_I2C_INFO("Init i2c dev, dma_rdlr%d[%2x]: %x\n\r", pi2c_init_data->index, &pi2c_reg->I2C_DMA_RDLR, pi2c_reg->I2C_DMA_RDLR);

	/*I2C Clear all interrupts first*/
	READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_INTR);

	/*I2C Disable all interrupts first*/
	hal_rtl_i2c_intr_ctrl(phal_i2c_adapter, 0, 0xFFFF);

	/* Enable the IC first and return the enable status */
	return hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CEnable);
}

/** \brief Description of hal_rtl_i2c_pure_deinit
 *
 *    hal_rtl_i2c_pure_deinit is used to deinit I2C module.
 *    It's directly to disable I2C module.
 *
 *   \param hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_pure_deinit(hal_i2c_adapter_t *phal_i2c_adapter)
{
	return hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CDisable);
}


/** \brief Description of hal_rtl_i2c_en_ctrl
 *
 *    hal_rtl_i2c_en_ctrl is used to enable/disable I2C module.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t enable: To enable/disable I2C.\n\r
 *                                  0: Disable.\n\r
 *                                  1: Enable\n\r
 *                                  2: Force Disable
 *   \retrun uint8_t:               I2C enable status.
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_en_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_en_tmp;

	ic_en_tmp = pi2c_reg->I2C_ENABLE;
	if (enable == 2) {
		HAL_SET_BIT(ic_en_tmp, I2C_BIT_ABRT);
		HAL_CLEAR_BIT(ic_en_tmp, I2C_BIT_EN);
	} else if (enable == 0) {
		HAL_CLEAR_BIT(ic_en_tmp, I2C_BIT_ABRT);
		HAL_CLEAR_BIT(ic_en_tmp, I2C_BIT_EN);
	} else {
		HAL_CLEAR_BIT(ic_en_tmp, I2C_BIT_ABRT);
		HAL_SET_BIT(ic_en_tmp, I2C_BIT_EN);
	}

	/* Enable/Disable I2C module */
	pi2c_reg->I2C_ENABLE = ic_en_tmp;
	DBG_I2C_INFO("ic_enable(%x): %x, %x\n\r", &(pi2c_reg->I2C_ENABLE), pi2c_reg->I2C_ENABLE, ic_en_tmp);
	//DBG_8195BL("ic_enable(%x): %x\n\r", &(pi2c_reg->enable), pi2c_reg->enable);

	if (enable != 1) {
		while (pi2c_reg->I2C_EN_STS & I2C_BIT_EN_STS) {
		}
	} else {
		while (!(pi2c_reg->I2C_EN_STS & I2C_BIT_EN_STS)) {
		}
	}

	return (uint8_t)(pi2c_reg->I2C_EN_STS & I2C_BIT_EN_STS);
}

/** \brief Description of hal_rtl_i2c_set_clk
 *
 *    hal_rtl_i2c_set_clk is used to set I2C clock.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return hal_status_t:      HAL status.
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_set_clk(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t clock   = phal_i2c_adapter->init_dat.clock;
	uint8_t  idx     = phal_i2c_adapter->init_dat.index;
	uint32_t ic_hl_cnt;
	uint32_t ic_h_time;
	uint32_t ic_l_time;

	/* Get the IC-Clk setting first for the following process*/
	uint32_t sys_clk  = (100000000) / 1000000;

	if (idx == 2) {
		sys_clk  = (50000000) / 1000000;
	}

	DBG_I2C_INFO("hal_rtl_i2c_set_clk, spd mod: %x\n\r", phal_i2c_adapter->init_dat.spd_mod);

	switch (phal_i2c_adapter->init_dat.spd_mod) {
	case I2CStandardSpeed:
		ic_h_time = ((1000000 / clock) * I2C_SS_MIN_SCL_HTIME) / (I2C_SS_MIN_SCL_HTIME + I2C_SS_MIN_SCL_LTIME);
		ic_l_time = ((1000000 / clock) * I2C_SS_MIN_SCL_LTIME) / (I2C_SS_MIN_SCL_HTIME + I2C_SS_MIN_SCL_LTIME);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_SS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>4) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 4;
		//}

		pi2c_reg->I2C_SS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("ICHtime:%x\n\r", pi2c_reg->I2C_SS_SCL_HCNT);
		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_SS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>3) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}
		pi2c_reg->I2C_SS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("ICLtime:%x\n\r", pi2c_reg->I2C_SS_SCL_LCNT);
		break;

	case I2CFastSpeed:
		ic_h_time = ((1000000 / clock) * I2C_FS_MIN_SCL_HTIME) / (I2C_FS_MIN_SCL_HTIME + I2C_FS_MIN_SCL_LTIME);
		ic_l_time = ((1000000 / clock) * I2C_FS_MIN_SCL_LTIME) / (I2C_FS_MIN_SCL_HTIME + I2C_FS_MIN_SCL_LTIME);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_FS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			DBG_I2C_ERR("h_cnt: %x\n\r", ic_hl_cnt);
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>4) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 4;
		//}

		pi2c_reg->I2C_FS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("fs_scl_hcnt%d: %x\n\r", idx, pi2c_reg->I2C_FS_SCL_HCNT);
		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_FS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			DBG_I2C_ERR("h_cnt: %x\n\r", ic_hl_cnt);
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>3) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}

		pi2c_reg->I2C_FS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("fs_scl_lcnt%d: %x\n\r", idx, pi2c_reg->I2C_FS_SCL_LCNT);
		break;

	case I2CHighSpeed:
		ic_hl_cnt = 400;
		pi2c_reg->I2C_SS_SCL_HCNT = ic_hl_cnt;

		ic_hl_cnt = 470;
		pi2c_reg->I2C_SS_SCL_LCNT = ic_hl_cnt;

		ic_hl_cnt = 60;
		pi2c_reg->I2C_FS_SCL_HCNT = ic_hl_cnt;

		ic_hl_cnt = 130;
		pi2c_reg->I2C_FS_SCL_LCNT = ic_hl_cnt;

		ic_h_time = ((1000000 / clock) * I2C_HS_MIN_SCL_HTIME_100) / (I2C_HS_MIN_SCL_HTIME_100 + I2C_HS_MIN_SCL_LTIME_100);
		ic_l_time = ((1000000 / clock) * I2C_HS_MIN_SCL_LTIME_100) / (I2C_HS_MIN_SCL_HTIME_100 + I2C_HS_MIN_SCL_LTIME_100);
		DBG_I2C_INFO("ICHtime:%x\n\r", ic_h_time);
		DBG_I2C_INFO("ICLtime:%x\n\r", ic_l_time);

		ic_hl_cnt = (ic_h_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_HS_HTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>8) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 3;
		//}

		pi2c_reg->I2C_HS_SCL_HCNT = ic_hl_cnt;
		DBG_I2C_INFO("hs_scl_hcnt%d: %x\n\r", idx, pi2c_reg->I2C_HS_SCL_HCNT);

		ic_hl_cnt = (ic_l_time * sys_clk) / 1000;
		if (ic_hl_cnt < I2C_HS_LTIME_MIN) {
			DBG_I2C_ERR("*** I2C clock is not available.\n\r");
			return HAL_ERR_PARA;
		}
		//if (ic_hl_cnt>6) {/*this part is according to the fine-tune result*/
		//    ic_hl_cnt -= 6;
		//}

		pi2c_reg->I2C_HS_SCL_LCNT = ic_hl_cnt;
		DBG_I2C_INFO("hs_scl_lcnt%d: %x\n\r", idx, pi2c_reg->I2C_HS_SCL_LCNT);
		break;

	default:
		break;
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_intr_ctrl
 *
 *    hal_rtl_i2c_intr_ctrl is used to set/clear interrupt mask bits.\n\r
 *    The given intrmsk bits would be cleared first in this function therefore the related\n\r
 *    interrupt mask is masked.\n\r
 *    If it's to set mask bits, the related interrupt mask would be set therefore the realted\n\r
 *    interrupt mask is unmasked.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter: Pointer to I2C control adapter.
 *   \param[in] uint8_t set_clr: To set or clear interrupt. \n\r
 *                               The interrupt mask register is active low.\n\r
 *                                1: set, which means the related interrupt mask is unmasked.\n\r
 *                                0: clear, which means the related interrupt mask is masked.\n\r
 *   \param[in] uint16_t intr_msk: I2C interrupt mask bit.
 *   \return hal_status_t:      HAL status.
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_intr_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t set_clr, uint16_t intr_msk)
{
	uint32_t ic_intr_msk_tmp;

	ic_intr_msk_tmp  = phal_i2c_adapter->init_dat.reg_base->I2C_INTR_MSK;
	ic_intr_msk_tmp  &= (~intr_msk);
	if (set_clr) {
		ic_intr_msk_tmp |= intr_msk;
	}

	phal_i2c_adapter->init_dat.reg_base->I2C_INTR_MSK  = ic_intr_msk_tmp;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_wr
 *
 *    hal_rtl_i2c_wr is used to send I2C data. In master mode, this function could send write command, too.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] const uint8_t *dat_buf:     Data buffer address for I2C write.
 *   \param[in] uint32_t dat_len:           Data length for I2C write.
 *   \param[in] uint8_t ctrl:           To control what I2C would do when data is all sent. (Only for Master Mode)\n\r
 *                                      In slave mode, this part should be always 0.\n\r
 *                                      0: Do nothing. There will not be a STOP condition when all data is sent.\n\r
 *                                      And there will not be any START/RESTART condition in the next transmission ,either.\n\r
 *                                      1: Send STOP after the last data.\n\r
 *                                      2: There will NOT be a STOP condition after the last data sent. But\n\r
 *                                      the next transmision would have a START/RESTART condition sent first.\n\r
 *                                      So setting 2 to this field could influence the following transmission.\n\r
 *   \return hal_status_t:      When the given Ctrl is NOT fit with hardware setting, a value of HAL_STS_ERR_HW\n\r
 *                              will be returned. Otherwise, a value of HAL_STS_OK will be returned.
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_wr(hal_i2c_adapter_t *phal_i2c_adapter, const uint8_t *dat_buf, uint32_t dat_len, uint8_t ctrl)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t dat_cnt  = 0;
	uint32_t ic_dat_cmd;

	/* Check RESTART configuration is enabled or not */
	//if ((ctrl == 2) && (BIT_GET_IC_RESTART_EN(pi2c_reg->con) == 0)) {
	if ((ctrl == 2) && ((pi2c_reg->I2C_CON & I2C_BIT_RSTRT_EN) == 0)) {
		return HAL_ERR_HW;
	}

	for (dat_cnt = 0; dat_cnt < dat_len; dat_cnt++) {
		ic_dat_cmd = 0;
		if ((dat_cnt == (dat_len - 1)) && (ctrl != 0)) {
			ic_dat_cmd |= (0x01 << (I2C_SHIFT_STP + ctrl - 1));
		}
		HAL_SET_BIT(ic_dat_cmd, (((uint8_t)(*(dat_buf + dat_cnt))) << I2C_SHIFT_DAT));
		HAL_CLEAR_BIT(ic_dat_cmd, I2C_BIT_CMD); /* i2c write */
		DBG_I2C_INFO("dat_cnt = %x, dc: %x, ctrl: %x\n\r", dat_cnt, ic_dat_cmd, ctrl);
		pi2c_reg->I2C_DAT_CMD = ic_dat_cmd;
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_mst_send_rdcmd
 *
 *    hal_rtl_i2c_mst_send_rdcmd is used to send I2C master read command.
 *    It should only be used for master mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint32_t cmd_len:           Read commnad length for I2C read.
 *   \param[in] uint8_t ctrl:               To control what I2C would do when data is all sent. (Only for Master Mode)\n\r
 *                                      0: Do nothing. There will not be a STOP condition when all data is sent.\n\r
 *                                      And there will not be any START/RESTART condition in the next transmission ,either.\n\r
 *                                      1: Send STOP after the last data.\n\r
 *                                      2: There will NOT be a STOP condition after the last data sent. But\n\r
 *                                      the next transmision would have a START/RESTART condition sent first.\n\r
 *                                      So setting 2 to this field could influence the following transmission.
 *
 *  \return void
 */
SECTION_I2C_TEXT void hal_rtl_i2c_mst_send_rdcmd(hal_i2c_adapter_t *phal_i2c_adapter, uint32_t cmd_len, uint8_t ctrl)
{
	uint32_t cmd_cnt = 0;
	uint32_t ic_dat_cmd;

	for (cmd_cnt = 0; cmd_cnt < cmd_len; cmd_cnt++) {
		ic_dat_cmd = 0;

		if ((cmd_cnt == (cmd_len - 1)) && (ctrl != 0)) {
			ic_dat_cmd |= (0x01 << (I2C_SHIFT_STP + ctrl - 1));
		}

		HAL_SET_BIT(ic_dat_cmd, I2C_BIT_CMD);   /* i2c read */
		phal_i2c_adapter->init_dat.reg_base->I2C_DAT_CMD = ic_dat_cmd;
	}
}


/** \brief Description of hal_rtl_i2c_dma_ctrl
 *
 *    hal_rtl_i2c_dma_ctrl is used to enable/disable I2C DMA function.\n\r
 *    Enable is used to control that this function performs enable or disable operaiton.\n\r
 *    And it would set/clear related register according to the given DMAEnMsk.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t enable:             To enable/disable I2C DMA function.\n\r
 *                                      0: Disable.\n\r
 *                                      1: Enable.
 *   \return uint32_t:         I2C DMA enable status.
 */
SECTION_I2C_TEXT uint32_t hal_rtl_i2c_dma_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_dma_cmd_tmp;

	if (enable == I2CForceDisable) {
		/* To disable I2C DMA */
		ic_dma_cmd_tmp = pi2c_reg->I2C_DMA_CMD;
		ic_dma_cmd_tmp &= (uint32_t)(~(I2C_BIT_DMA_EN));

		pi2c_reg->I2C_DMA_CMD = ic_dma_cmd_tmp;
	} else {
		while (pi2c_reg->I2C_DMA_CMD & I2C_BIT_DMA_EN) {
		}

		while (pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_DMA_I2C_DONE) {
		}

		/* To disable I2C DMA */
		ic_dma_cmd_tmp = pi2c_reg->I2C_DMA_CMD;
		ic_dma_cmd_tmp &= (uint32_t)(~(I2C_BIT_DMA_EN));
		if (enable) {
			ic_dma_cmd_tmp |= (I2C_BIT_DMA_EN);
		}

		pi2c_reg->I2C_DMA_CMD = ic_dma_cmd_tmp;
	}

	DBG_I2C_INFO("dam_cmd: %x\n\r", pi2c_reg->I2C_DMA_CMD);
	return (uint32_t)(pi2c_reg->I2C_DMA_CMD);
}

/** \brief Description of hal_rtl_i2c_mst_restr_ctrl
 *
 *    hal_rtl_i2c_mst_restr_ctrl is used to enable/disable RESTART feature.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t restr_en:               To enable/disable I2C RESTART fearture. Software should enable\n\r
 *                                              this feature before it sets RESTART command in REG_IC_DATA_CMD\n\r
 *                                              register.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_mst_restr_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t restr_en)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_en_sts_bak;
	uint32_t    ic_con_tmp;

	/* Check if I2C is in master mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 1) {
		return HAL_ERR_PARA;
	}

	/* Disable i2c first */
	ic_en_sts_bak = pi2c_reg->I2C_EN_STS;
	if (ic_en_sts_bak & I2C_BIT_EN_STS) {
		if (hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CDisable)) {
			return HAL_ERR_HW;
		}
	}

	if (((phal_i2c_adapter->init_dat.addr_mod == I2CAddress10bit) || (phal_i2c_adapter->init_dat.spd_mod == I2CHighSpeed))
		&& (restr_en == I2CDisable)) {
		DBG_I2C_ERR("RESTART enable could not be disabled in 10bit or high speed mode\n\r");
		return HAL_ERR_PARA;
	}

	ic_con_tmp = pi2c_reg->I2C_CON;
	HAL_CLEAR_BIT(ic_con_tmp, I2C_BIT_RSTRT_EN);
	HAL_SET_BIT(ic_con_tmp, restr_en << I2C_SHIFT_RSTRT_EN);
	pi2c_reg->I2C_CON = ic_con_tmp;

	if (ic_en_sts_bak & I2C_BIT_EN_STS) {
		hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CEnable);
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_mst_gc_sb_ctrl
 *
 *    hal_rtl_i2c_mst_gc_sb_ctrl is used to enable/disable General Call or START Byte features.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t enable:             To enable/disable I2C General Call or START Byte feature.
 *   \param[in] uint8_t gc_sb:          Select General Call or START Byte feature.\n\r
 *                                      0: General Call\n\r
 *                                      1: START Byte
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_mst_gc_sb_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t enable, uint8_t gc_sb)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_tar_tmp;

	/* Check if I2C is in master mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 1) {
		return HAL_ERR_PARA;
	}

	/* To set GC register bit, TX FIFO should be empty */
	//if (!(BIT_GET_IC_TFE(pi2c_reg->sts))){
	if ((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) {
		return HAL_ERR_HW;
	}

	if (((pi2c_reg->I2C_CON & I2C_BIT_RSTRT_EN) == 0) && (gc_sb == I2CStartByte)) {
		DBG_I2C_ERR("For HW feature, RESTART_EN should be set first before sending START Byte\n\r");
		return HAL_ERR_HW;
	}

	ic_tar_tmp = pi2c_reg->I2C_TAR;
	HAL_CLEAR_BIT(ic_tar_tmp, I2C_BIT_SPEC | I2C_BIT_GC_START_BYTE);

	if (enable) {
		/* I2CGCSB = 0: to enable GC */
		/* I2CGCSB = 1: to enable SB */
		if (!gc_sb) {
			HAL_SET_BIT(ic_tar_tmp, I2C_BIT_SPEC);
		} else {
			HAL_SET_BIT(ic_tar_tmp, I2C_BIT_SPEC | I2C_BIT_GC_START_BYTE);
		}
	}

	pi2c_reg->I2C_TAR = ic_tar_tmp;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_slv_no_ack_ctrl
 *
 *    hal_rtl_i2c_slv_no_ack_ctrl is used to enable/disable no acknowledge feature in\n\r
 *    slave mode. If SlvNoAck is enabled, I2C would generate NACK after a data byte is \n\r
 *    received in slave mode. If SlvNoAck is disabled, I2C would gernerate ACK/NACK,\n\r
 *    depending on normal criteria.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t no_ack_en:              To enable/disable slave no ack feature.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_slv_no_ack_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t no_ack_en)
{
	uint32_t ic_slv_nack_tmp;

	/* Check if I2C is in slave mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 0) {
		return HAL_ERR_PARA;
	}

	//while ((BIT_GET_BUS_STATUS(pi2c_reg->sts) != I2CBusIdle) && (BIT_GET_BUS_STATUS(pi2c_reg->sts) != I2CBusClockStretch)) {
	while ((((phal_i2c_adapter->init_dat.reg_base->I2C_STS & I2C_MASK_STS_BUS_STS) >> I2C_SHIFT_STS_BUS_STS) != I2CBusIdle) &&
		   (((phal_i2c_adapter->init_dat.reg_base->I2C_STS & I2C_MASK_STS_BUS_STS) >> I2C_SHIFT_STS_BUS_STS) != I2CBusClockStretch)) {
	}

	ic_slv_nack_tmp = phal_i2c_adapter->init_dat.reg_base->I2C_SLV_DAT_NACK;
	HAL_CLEAR_BIT(ic_slv_nack_tmp, I2C_BIT_SLV_DAT_NACK);
	HAL_SET_BIT(ic_slv_nack_tmp, no_ack_en << I2C_SHIFT_SLV_DAT_NACK);
	phal_i2c_adapter->init_dat.reg_base->I2C_SLV_DAT_NACK = ic_slv_nack_tmp;

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_slv_no_ack_sts
 *
 *    hal_rtl_i2c_slv_no_ack_sts is used to get the enable status of slave no ack function.\n\r
 *    If its return value is 0, I2C slave no ack is disabled and could ack to normal transfermation.\n\r
 *    If its return value is 1, I2C slave no ack is enabled and could ONLY send NACK to all
 *    master command or data.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t:               slave mode no-ack status.\n\r
 *                                  0: slave no ack disabled. 1: slave no ack enabled.
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_slv_no_ack_sts(hal_i2c_adapter_t *phal_i2c_adapter)
{
	/* Check if I2C is in slave mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 0) {
		return 0xFF;
	}

	return (uint8_t)(phal_i2c_adapter->init_dat.reg_base->I2C_SLV_DAT_NACK & I2C_BIT_SLV_DAT_NACK);
}

/** \brief Description of hal_rtl_i2c_slv_ack_gc_ctrl
 *
 *    hal_rtl_i2c_slv_ack_gc_ctrl is used to enable/disable I2C responding General Call\n\r
 *    feature. If it's enabled, I2C could responds to a General Call.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \param[in] uint8_t slv_gc_en:      To enable/disable slave ack General Call feature.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_slv_ack_gc_ctrl(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t slv_gc_en)
{
	uint32_t ic_ack_gc_tmp;

	/* Check if I2C is in slave mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 0) {
		return HAL_ERR_PARA;
	}

	ic_ack_gc_tmp = phal_i2c_adapter->init_dat.reg_base->I2C_ACK_GEN_CALL;
	HAL_CLEAR_BIT(ic_ack_gc_tmp, I2C_BIT_ACK_GEN_CALL);
	HAL_SET_BIT(ic_ack_gc_tmp, slv_gc_en << I2C_SHIFT_ACK_GEN_CALL);
	phal_i2c_adapter->init_dat.reg_base->I2C_ACK_GEN_CALL = ic_ack_gc_tmp;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_slv_ack_gc_sts
 *
 *    hal_rtl_i2c_slv_ack_gc_sts is used to get the enable status of slave ack General
 *    Call.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t:               slave mode adc GC status.\n\r
 *                                  0: Slave does NOT ack GC. 1: Slave could ack GC.
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_slv_ack_gc_sts(hal_i2c_adapter_t *phal_i2c_adapter)
{
	/* Check if I2C is in slave mode. */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 0) {
		return 0xFF;
	}

	return (uint8_t)(phal_i2c_adapter->init_dat.reg_base->I2C_ACK_GEN_CALL & I2C_BIT_ACK_GEN_CALL);
}

/** \brief Description of hal_rtl_i2c_slv_to_slp
 *
 *    hal_rtl_i2c_slv_to_slp is used to set I2C slave into sleep state.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_slv_to_slp(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint32_t ic_fltr_tmp;
	uint32_t ic_sleep_tmp;

	/* Confirm that I2C is in slave mode */
	if (hal_rtl_i2c_chk_mod(phal_i2c_adapter) != 0) {
		return HAL_ERR_HW;
	}

	/* Disable a few features immediately. When going back to active mode,
	   software should set some features back according to I2C_INIT_DAT. */
	ic_fltr_tmp = phal_i2c_adapter->init_dat.reg_base->I2C_FLTR;
	HAL_CLEAR_BIT(ic_fltr_tmp, I2C_BIT_DIG_FLTR_EN);
	phal_i2c_adapter->init_dat.reg_base->I2C_FLTR = ic_fltr_tmp;

	ic_sleep_tmp = phal_i2c_adapter->init_dat.reg_base->I2C_SLP;
	HAL_SET_BIT(ic_sleep_tmp, I2C_BIT_CLK_CTRL);
	phal_i2c_adapter->init_dat.reg_base->I2C_SLP = ic_sleep_tmp;

	while (!(phal_i2c_adapter->init_dat.reg_base->I2C_SLP & I2C_BIT_SLP_CLK_GATED)) {
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_slv_chk_mst_wr
 *
 *    hal_rtl_i2c_slv_chk_mst_wr is used to check if there is a master write command for slave addresses of
 *    rtl8195b.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t:               0: An master write command for slave address 0 is matched.\n\r
 *                                  1: An master write command for slave address 1 is matched.\n\r
 *                                  0xEF: Error status, disable/deinit process is recommended.\n\r
 *                                  0xFF: No write commnad rtl8195b I2C slave.
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_slv_chk_mst_wr(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint8_t retv = 0xFF;

	if (phal_i2c_adapter->init_dat.reg_base->I2C_STS & I2C_BIT_STS_RFNE) {
		if (phal_i2c_adapter->init_dat.reg_base->I2C_RAW_INTR_STAT & I2C_BIT_RAW_ADDR0_MATCH) {
			retv = 0;
		} else if (phal_i2c_adapter->init_dat.reg_base->I2C_RAW_INTR_STAT & I2C_BIT_RAW_ADDR1_MATCH) {
			retv = 1;
		} else {
			retv = 0xEF;
		}
	}

	return retv;
}

/** \brief Description of hal_rtl_i2c_slv_chk_rd_req
 *
 *    hal_rtl_i2c_slv_chk_rd_req is used to check if there is a master read command for slave addresses of
 *    rtl8195b.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:     Pointer to I2C control adapter.
 *   \return uint8_t:       0: An master read command for slave address 0 is matched.\n\r
 *                          1: An master read command for slave address 1 is matched.\n\r
 *                          0xEF: Error status, disable/deinit process is recommended.\n\r
 *                          0xFF: No read commnad rtl8195b I2C slave.
 *
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_slv_chk_rd_req(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint8_t retv = 0xFF;

	if (phal_i2c_adapter->init_dat.reg_base->I2C_RAW_INTR_STAT & I2C_BIT_RAW_RD_REQ) {
		if (phal_i2c_adapter->init_dat.reg_base->I2C_RAW_INTR_STAT & I2C_BIT_RAW_ADDR0_MATCH) {
			retv = 0;
		} else if (phal_i2c_adapter->init_dat.reg_base->I2C_RAW_INTR_STAT & I2C_BIT_RAW_ADDR1_MATCH) {
			retv = 1;
		} else {
			retv = 0xEF;
		}
		//READ_CLEAR_I2C_REG(pi2c_reg, clr_addr_match);
	}

	return retv; /* no master read request */
}

/** \brief Description of hal_rtl_i2c_pin_init
 *
 *    hal_rtl_i2c_pin_init is used to initialize i2c clock and pin.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_pin_init(hal_i2c_adapter_t *phal_i2c_adapter)
{

	return HAL_OK;

}

/** \brief Description of hal_rtl_i2c_pin_deinit
 *
 *    hal_rtl_i2c_pin_deinit is used to deinitialize i2c clock and pin.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_pin_deinit(hal_i2c_adapter_t *phal_i2c_adapter)
{
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_reg_comm_irq
 *
 *    hal_rtl_i2c_reg_comm_irq is used to register common IRQ handler.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] irq_handler_t handler:           Common IRQ handler function.
 *   \return void
 */
SECTION_I2C_TEXT void hal_rtl_i2c_reg_comm_irq(hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler)
{
	hal_rtl_irq_disable(I2C0_IRQn + phal_i2c_adapter->init_dat.index);
	__ISB();

	// Register I2C common IRQ handler
	hal_rtl_irq_set_vector(I2C0_IRQn + phal_i2c_adapter->init_dat.index, (uint32_t)handler);
	hal_rtl_irq_clear_pending(I2C0_IRQn + phal_i2c_adapter->init_dat.index);
	hal_rtl_irq_enable(I2C0_IRQn + phal_i2c_adapter->init_dat.index);
}

/** \brief Description of hal_rtl_i2c_reg_irq
 *
 *    hal_rtl_i2c_reg_irq is used to register IRQ handler.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:        Pointer to I2C control adaptor.
 *   \param[in] irq_handler_t handler:           IRQ handler function.
 *   \return void
 */
SECTION_I2C_TEXT void hal_rtl_i2c_reg_irq(hal_i2c_adapter_t *phal_i2c_adapter, irq_handler_t handler)
{
	i2c_gadapter.adapter[phal_i2c_adapter->init_dat.index] = phal_i2c_adapter;
	i2c_gadapter.irq_fun[phal_i2c_adapter->init_dat.index] = handler;
}

/** \brief Description of hal_rtl_i2c_unreg_irq
 *
 *    hal_rtl_i2c_unreg_irq is used to unregister IRQ handler.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint8_t
 */
SECTION_I2C_TEXT uint8_t hal_rtl_i2c_unreg_irq(hal_i2c_adapter_t *phal_i2c_adapter)
{
	uint8_t i;

	i2c_gadapter.adapter[phal_i2c_adapter->init_dat.index] = NULL;
	i2c_gadapter.irq_fun[phal_i2c_adapter->init_dat.index] = (irq_handler_t)NULL;
	for (i = 0; i < HP_I2C_NO; i++) {
		if (i2c_gadapter.irq_fun[phal_i2c_adapter->init_dat.index] != NULL) {
			break;
		}
	}

	if (i == (HP_I2C_NO)) {
		// No any UART port has IRQ handler, so disable the common interrupt
		for (i = 0; i < HP_I2C_NO; i++) {
			hal_rtl_irq_disable(I2C0_IRQn + i);
			__ISB();
			hal_rtl_irq_set_vector(I2C0_IRQn + i, (uint32_t)NULL);
		}

		return 1;
	}
	return 0;
}

/** \brief Description of hal_rtl_i2c_init
 *
 *    hal_rtl_i2c_init is used to initialize i2c including platform related features.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_init(hal_i2c_adapter_t *phal_i2c_adapter)
{
	if (/*hal_i2c_comm_irq_reg == 0*/1) {

		if (phal_i2c_adapter->init_dat.index == 0) {
			hal_rtl_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C0_IRQHandler);
		} else if (phal_i2c_adapter->init_dat.index == 1) {
			hal_rtl_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C1_IRQHandler);
		} else if (phal_i2c_adapter->init_dat.index == 2) {
			hal_rtl_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C2_IRQHandler);
		} else if (phal_i2c_adapter->init_dat.index == 3) {
			hal_rtl_i2c_reg_comm_irq(phal_i2c_adapter, (irq_handler_t)I2C3_IRQHandler);
		}

		hal_rtl_irq_set_priority(I2C0_IRQn + phal_i2c_adapter->init_dat.index, I2C0_IRQPri + phal_i2c_adapter->init_dat.index);
		hal_i2c_comm_irq_reg = 1;
	}

	if (phal_i2c_adapter->init_dat.master) {
		hal_rtl_i2c_reg_irq(phal_i2c_adapter, hal_rtl_i2c_mst_irq_handler);
	} else {
		hal_rtl_i2c_reg_irq(phal_i2c_adapter, hal_rtl_i2c_slv_irq_handler);
	}

	hal_rtl_i2c_pure_init(phal_i2c_adapter);

	if (phal_i2c_adapter->init_dat.reg_base->I2C_EN_STS & I2C_BIT_EN_STS) {
		phal_i2c_adapter->status   = I2CStatusInitialized;
		phal_i2c_adapter->init_dat.enable  = I2CEnable;
		phal_i2c_adapter->status   = I2CStatusIdle;
		return HAL_OK;
	} else {
		phal_i2c_adapter->status   = I2CStatusUninitial;
		phal_i2c_adapter->init_dat.enable  = I2CDisable;
		return HAL_ERR_HW;
	}

}

/** \brief Description of hal_rtl_i2c_deinit
 *
 *    hal_rtl_i2c_deinit is used to deinitialize i2c including platform related features.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_deinit(hal_i2c_adapter_t *phal_i2c_adapter)
{
	phal_i2c_adapter->init_dat.enable  = hal_rtl_i2c_pure_deinit(phal_i2c_adapter);

	if (hal_rtl_i2c_unreg_irq(phal_i2c_adapter) != 0) {
		hal_i2c_comm_irq_reg = 0;
	}

	if (phal_i2c_adapter->init_dat.reg_base->I2C_EN_STS & I2C_BIT_EN_STS) {
		return HAL_ERR_HW;
	} else {
		phal_i2c_adapter->status     = I2CStatusUninitial;
		return HAL_OK;
	}
}

/** \brief Description of hal_rtl_i2c_load_default
 *
 *    hal_rtl_i2c_load_default is used to load default setting for i2c module.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] uint8_t index:   i2c device index.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_load_default(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t index)
{
	i2c_init_dat_t  *pi2c_init_dat      = (i2c_init_dat_t *) & (phal_i2c_adapter->init_dat);
	i2c_platform_data_t *pi2c_plft_dat  = (i2c_platform_data_t *) & (phal_i2c_adapter->pltf_dat);
	i2c_user_callback_t *pi2c_usr_cb    = (i2c_user_callback_t *) & (phal_i2c_adapter->usr_cb);

	/* Check the input I2C index first */
	if (hal_rtl_i2c_idx_chk(index)) {
		return HAL_ERR_UNKNOWN;
	}

	phal_i2c_adapter->init_dat.index = index;

	/* I2C Initial Default Configuration */
	pi2c_init_dat->enable       = I2CDisable;
	pi2c_init_dat->master       = I2CMasterMode;
	pi2c_init_dat->addr_mod     = I2CAddress7bit;
	pi2c_init_dat->spd_mod      = I2CStandardSpeed;
	pi2c_init_dat->sda_setup    = 2;
	pi2c_init_dat->ff_rxtl      = 0x00;
	pi2c_init_dat->ff_txtl      = 0x00;
	pi2c_init_dat->clock        = 100;              // 100 kHz
	pi2c_init_dat->ack_addr0    = 0x11;
	pi2c_init_dat->ack_addr1    = 0x11;
	pi2c_init_dat->sda_hold     = 4;
	pi2c_init_dat->bus_ld       = 100;
	pi2c_init_dat->dma_mod      = I2CDmaDwc;
	pi2c_init_dat->tx_dma_rq_lv = 0x04;
	pi2c_init_dat->rx_dma_rq_lv = 0x07;
	pi2c_init_dat->rx_dma_rq_lv_s1  = 0x03;
	pi2c_init_dat->dig_fltr_en  = I2CEnable;
	pi2c_init_dat->dig_fltr_lvl = 2;

	/* I2C Platform Level Configuration */
	pi2c_plft_dat->scl_pin      = PIN_A0;
	pi2c_plft_dat->sda_pin      = PIN_A1;
	pi2c_plft_dat->tx_dma_bound = 8;
	pi2c_plft_dat->rx_dma_bound = 8;
	pi2c_plft_dat->tr_time_out  = 0xFFFFFFFF;

	/* I2C Interrupt Handle Configuration */
#if defined(CONFIG_BUILD_NONSECURE)
	switch (phal_i2c_adapter->init_dat.index) {
	case 0:
		pi2c_init_dat->reg_base     = (void *)I2C0_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C0_IRQn;
		break;
	case 1:
		pi2c_init_dat->reg_base     = (void *)I2C1_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C1_IRQn;
		break;
	case 2:
		pi2c_init_dat->reg_base     = (void *)I2C2_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C2_IRQn;
		break;
	case 3:
		pi2c_init_dat->reg_base     = (void *)I2C3_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C3_IRQn;
		break;

	default:
		return HAL_ERR_UNKNOWN;
	}
#else
	switch (phal_i2c_adapter->init_dat.index) {
	case 0:
		pi2c_init_dat->reg_base     = (void *)I2C0_S_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C0_IRQn;
		break;
	case 1:
		pi2c_init_dat->reg_base     = (void *)I2C1_S_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C1_IRQn;
		break;
	case 2:
		pi2c_init_dat->reg_base     = (void *)I2C2_S_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C2_IRQn;
		break;
	case 3:
		pi2c_init_dat->reg_base     = (void *)I2C3_S_BASE;
		phal_i2c_adapter->irq_config.irq_num   = I2C3_IRQn;
		break;

	default:
		return HAL_ERR_UNKNOWN;
	}

#endif

	phal_i2c_adapter->irq_config.data      = (void *)phal_i2c_adapter;
	phal_i2c_adapter->irq_config.irq_fun   = hal_rtl_i2c_mst_irq_handler;
	phal_i2c_adapter->irq_config.priority  = HP_I2C_IRQ_PR;

	phal_i2c_adapter->tx_dma_dat.ch_sts    = I2CDmaChNone;
	phal_i2c_adapter->rx_dma_dat.ch_sts    = I2CDmaChNone;
	phal_i2c_adapter->status   = I2CStatusUninitial;
	phal_i2c_adapter->op_mode  = I2CModePoll;
	//phal_i2c_adapter->mst_spe_func |= I2CAddressRetry;
	phal_i2c_adapter->err_type = I2CErrorNone;
	phal_i2c_adapter->rd_cmd_no = 0;

	/* Clear USER Callback Function Pointer */
	_memset(pi2c_usr_cb, 0x00, sizeof(i2c_user_callback_t));

	/* Clear TRX information */
	_memset(&(phal_i2c_adapter->tx_dat), 0x00, sizeof(i2c_tx_info_t));
	_memset(&(phal_i2c_adapter->rx_dat), 0x00, sizeof(i2c_rx_info_t));

	/* Clear TRX DMA information */
	_memset(&(phal_i2c_adapter->tx_dma_dat), 0x00, sizeof(i2c_dma_info_t));
	_memset(&(phal_i2c_adapter->rx_dma_dat), 0x00, sizeof(i2c_dma_info_t));
	phal_i2c_adapter->tx_dma_dat.ch_sts = I2CDmaChNone;
	phal_i2c_adapter->rx_dma_dat.ch_sts = I2CDmaChNone;

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_set_tar
 *
 *    hal_rtl_i2c_set_tar is to set target slave address in master mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] uint8_t mst_rw:   This target address is for read or write usage.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_set_tar(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t mst_rw)
{
	uint32_t start_time = 0;
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)(phal_i2c_adapter->init_dat.reg_base);
	uint32_t ic_tar_tmp;

	/* check if it's necessary to update TAR */
	ic_tar_tmp  = pi2c_reg->I2C_TAR;
	if (mst_rw == 0) {
		if (((ic_tar_tmp & I2C_MASK_TAR) >> I2C_SHIFT_TAR) != phal_i2c_adapter->tx_dat.addr) {
			HAL_CLEAR_BIT(ic_tar_tmp, I2C_MASK_TAR);
			HAL_SET_BIT(ic_tar_tmp, phal_i2c_adapter->tx_dat.addr << I2C_SHIFT_TAR);
		} else {
			return HAL_OK;
		}
	} else {
		if ((ic_tar_tmp & I2C_MASK_TAR) >> I2C_SHIFT_TAR != phal_i2c_adapter->rx_dat.addr) {
			HAL_CLEAR_BIT(ic_tar_tmp, I2C_MASK_TAR);
			HAL_SET_BIT(ic_tar_tmp, phal_i2c_adapter->rx_dat.addr << I2C_SHIFT_TAR);
		} else {
			return HAL_OK;
		}
	}

	/* Check TX FIFO status, wait for all tx is done. */
	start_time = hal_read_cur_time();
	//while (!BIT_GET_IC_TFE(pi2c_reg->status)) {
	while ((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) {
		if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorTarTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_TIMEOUT;
		}
	}

	/* Check Master activity status, wait until it's ready */
	start_time  = hal_read_cur_time();
	//while (BIT_GET_MST_ACTIVITY(pi2c_reg->status)) {
	while (pi2c_reg->I2C_STS & I2C_BIT_STS_MST_ACT) {
		if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorTarTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_TIMEOUT;
		}
	}

	pi2c_reg->I2C_TAR = ic_tar_tmp;
	return HAL_OK;
}

/** \brief Description of hal_i2c_rom_send_addr_retry
 *
 *    hal_rtl_i2c_send_addr_retry is to send address and the first data byte with fail retry.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_send_addr_retry(hal_i2c_adapter_t *phal_i2c_adapter)
{
	i2c_tx_info_t *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t start_time  = 0;
	uint32_t start_time_tx = 0;
	uint32_t ic_tx_abrt_tmp;
	uint32_t ic_rty_cnt  = 0;
	uint8_t  wr_ctrl     = 0;

	phal_i2c_adapter->status = I2CStatusTxReady;
	ic_rty_cnt = 0;
	start_time = hal_read_cur_time();

	for (;;) {
		if (pi2c_tx_info->len == 0) {
			break;
		}

		//if (BIT_GET_IC_TFE(pi2c_reg->status)) {
		if (pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) {
			/* To send the first data since TX FIFO is empty. */
			if (pi2c_tx_info->len > 0) {
				/* Wrtie data into I2C TX FIFO */
				wr_ctrl = 0;
				if (pi2c_tx_info->len == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_tx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
				}

				hal_rtl_i2c_wr(phal_i2c_adapter, pi2c_tx_info->buf, 1, wr_ctrl);
				pi2c_tx_info->buf++;
				pi2c_tx_info->len--;
				/* Wait for 1st TX data done */
				start_time_tx   = hal_read_cur_time();
				//while (!(BIT_GET_IC_TFE(pi2c_reg->status))) {
				while ((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) {
					//if (BIT_GET_RAW_TX_ABRT(pi2c_reg->raw_intr_stat)) {
					if (pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_TX_ABRT) {
						break;
					}

					if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time_tx) == 1) {
						phal_i2c_adapter->status   = I2CStatusTimeOut;
						phal_i2c_adapter->err_type = I2CErrorTxAddrTimeOut;
						if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
							phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
						}

						return HAL_TIMEOUT;
					}
				}
			}

			hal_delay_us((uint32_t)((1000 * 11) / phal_i2c_adapter->init_dat.clock));
			//if ((BIT_GET_IC_TFE(pi2c_reg->status) == 0) || (BIT_GET_RAW_TX_ABRT(pi2c_reg->raw_intr_stat) != 0)) {
			if (((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) || ((pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_TX_ABRT) != 0)) {
				ic_rty_cnt++;
				ic_tx_abrt_tmp = pi2c_reg->I2C_TX_ABRT_SRC;

				DBG_I2C_ERR("tx retry, abrt:%x\n\r", ic_tx_abrt_tmp);
				/* Check retry limit */
				if (ic_rty_cnt > HP_I2C_ADD_RTY_MAX) {
					if ((ic_tx_abrt_tmp & (I2C_BIT_ADDR_7BIT_NACK | I2C_BIT_ADDR1_10BIT_NACK |
										   I2C_BIT_ADDR2_10BIT_NACK)) != 0) {
						phal_i2c_adapter->err_type |= I2CErrorMasterAddrNack;
					} else if ((ic_tx_abrt_tmp & I2C_BIT_TXDAT_NACK) != 0) {
						phal_i2c_adapter->err_type |= I2CErrorMasterDataNack;
					} else {
						phal_i2c_adapter->err_type |= I2CErrorTxAbort;
					}

					phal_i2c_adapter->status = I2CStatusError;
					pi2c_tx_info->buf--;
					pi2c_tx_info->len++;
					if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
						phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
					}

					return HAL_ERR_HW;
				}

				hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CForceDisable);
				hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CEnable);
				phal_i2c_adapter->status  = I2CStatusTxReady;
				phal_i2c_adapter->err_type = 0;
				pi2c_tx_info->buf--;
				pi2c_tx_info->len++;
			} else {
				break;  /* Retry end, ready to return to main process */
			}

			start_time = hal_read_cur_time();
		} else {
			if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
				phal_i2c_adapter->status   = I2CStatusTimeOut;
				phal_i2c_adapter->err_type = I2CErrorTxAddrTimeOut;
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}

				return HAL_TIMEOUT;
			}
		}
	}

	phal_i2c_adapter->status = I2CStatusTxing;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_send_poll
 *
 *    hal_rtl_i2c_send_poll is to send i2c data by polling (blocking) mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_send_poll(hal_i2c_adapter_t *phal_i2c_adapter)
{
	i2c_tx_info_t   *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t start_time  = 0;
	uint32_t ic_raw_state_tmp;
	uint8_t  wr_ctrl = 0;

	start_time = hal_read_cur_time();

	for (;;) {
		if (pi2c_tx_info->len == 0) {
			break;
		}

		phal_i2c_adapter->status = I2CStatusTxing;
		ic_raw_state_tmp = pi2c_reg->I2C_RAW_INTR_STAT;
		//if (ic_raw_state_tmp.d32 & (BIT_RAW_TX_OVER | BIT_RAW_TX_ABRT)) {
		if ((ic_raw_state_tmp & I2C_BIT_RAW_TX_OVER) || (ic_raw_state_tmp & I2C_BIT_RAW_TX_ABRT)) {
			phal_i2c_adapter->status = I2CStatusError;
			if (ic_raw_state_tmp & I2C_BIT_RAW_TX_OVER) {
				phal_i2c_adapter->err_type = I2CErrorTxOver;
			} else {
				phal_i2c_adapter->err_type = I2CErrorTxAbort;
			}

			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_ERR_HW;
			//} else if (BIT_GET_IC_TFNF(pi2c_reg->status)) {
		} else if (pi2c_reg->I2C_STS & I2C_BIT_STS_TFNF) {
			/* Wrtie data into I2C TX FIFO */
			wr_ctrl = 0;
			if (pi2c_tx_info->len == 1) {
				if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
					wr_ctrl = 2;    /* Set RESTART for the next byte */
				} else {
					if (pi2c_tx_info->mst_stop == 1) {
						wr_ctrl = 1;    /* Set STOP for the last byte */
					}
				}
			}
			hal_rtl_i2c_wr(phal_i2c_adapter, pi2c_tx_info->buf, 1, wr_ctrl);
			pi2c_tx_info->buf++;
			pi2c_tx_info->len--;
			start_time = hal_read_cur_time();

		} else {
			if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
				phal_i2c_adapter->status   = I2CStatusTimeOut;
				phal_i2c_adapter->err_type = I2CErrorTxCmdTimeOut;
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}

				return HAL_TIMEOUT;
			}
		}
	}

	start_time = hal_read_cur_time();
	while (!(pi2c_reg->I2C_STS & I2C_BIT_STS_TFE)) {
		if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorTxCmdTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_TIMEOUT;
		}
	}

	phal_i2c_adapter->status = I2CStatusIdle;
	if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
		phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_send_intr
 *
 *    hal_rtl_i2c_send_intr is to send i2c data by interrupt mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_send_intr(hal_i2c_adapter_t *phal_i2c_adapter)
{
	//i2c_tx_info_t   *pi2c_tx_info = (i2c_tx_info_t *)&(phal_i2c_adapter->tx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;

	phal_i2c_adapter->status = I2CStatusTxReady;
	pi2c_reg->I2C_INTR_MSK  = (I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_TX_OVER);
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_recv_addr_retry
 *
 *    hal_rtl_i2c_recv_addr_retry is to send slave address and the first read command with retry features.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_recv_addr_retry(hal_i2c_adapter_t *phal_i2c_adapter)
{
	i2c_rx_info_t   *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t start_time  = 0;
	uint32_t start_time_cmd = 0;
	uint32_t ic_tx_abrt_tmp;
	uint32_t ic_rty_cnt  = 0;
	uint8_t  wr_ctrl     = 0;

	phal_i2c_adapter->rd_cmd_no = phal_i2c_adapter->rx_dat.len;

	if (phal_i2c_adapter->rd_cmd_no < 1) {
		return HAL_OK;
	}

	phal_i2c_adapter->status = I2CStatusRxReady;
	READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_ABRT);
	READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ACT);
	READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_INTR);

	ic_rty_cnt = 0;
	start_time = hal_read_cur_time();
	for (;;) {
		//if (BIT_GET_IC_TFE(pi2c_reg->sts)) {
		if (pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) {
			if (phal_i2c_adapter->rd_cmd_no > 0) {
				/* Wrtie cmd into I2C TX FIFO */
				wr_ctrl = 0;
				if (phal_i2c_adapter->rd_cmd_no == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_rx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
				}

				hal_rtl_i2c_mst_send_rdcmd(phal_i2c_adapter, 1, wr_ctrl);

				/* Wait for 1st TX data done */
				start_time_cmd = hal_read_cur_time();
				//while (!(BIT_GET_IC_TFE(pi2c_reg->status))) {
				while ((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) {
					//if (BIT_GET_RAW_TX_ABRT(pi2c_reg->raw_intr_stat)) {
					if ((pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_TX_ABRT)) {
						break;
					}

					if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time_cmd) == 1) {
						phal_i2c_adapter->status = I2CStatusTimeOut;
						phal_i2c_adapter->err_type = I2CErrorRxAddrTimeOut;
						if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
							phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
						}
						return HAL_TIMEOUT;
					}
				}
			}

			//hal_delay_us((u32)((1000*30)/phal_i2c_adapter->init_dat.clock));
			hal_delay_us((uint32_t)((1000 * 11) / phal_i2c_adapter->init_dat.clock));

			//if ((BIT_GET_IC_TFE(pi2c_reg->status) == 0) || (BIT_GET_RAW_TX_ABRT(pi2c_reg->raw_intr_stat) != 0)) {
			if (((pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) == 0) || ((pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_TX_ABRT) != 0)) {
				ic_rty_cnt++;
				ic_tx_abrt_tmp = pi2c_reg->I2C_TX_ABRT_SRC;

				DBG_I2C_WARN("rx retry, abrt:%x\n\r", ic_tx_abrt_tmp);
				/* Check retry limit */
				if (ic_rty_cnt > HP_I2C_ADD_RTY_MAX) {
					//if ((ic_tx_abrt_tmp.w & (BIT_ABRT_7B_ADDR_NOACK | BIT_ABRT_10ADDR1_NOACK |
					//                            BIT_ABRT_10ADDR2_NOACK)) != 0) {
					if ((ic_tx_abrt_tmp & (I2C_BIT_ADDR_7BIT_NACK | I2C_BIT_ADDR1_10BIT_NACK |
										   I2C_BIT_ADDR2_10BIT_NACK)) != 0) {
						phal_i2c_adapter->err_type |= I2CErrorMasterAddrNack;
					} else {
						phal_i2c_adapter->err_type |= I2CErrorTxAbort;
					}

					phal_i2c_adapter->status = I2CStatusError;
					if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
						phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
					}

					return HAL_ERR_HW;
				}

				hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CForceDisable);
				hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CEnable);
				phal_i2c_adapter->status  = I2CStatusRxReady;
				phal_i2c_adapter->err_type = 0;
			} else {
				//rd_cmd_no--;
				phal_i2c_adapter->rd_cmd_no--;
				break;
			}
			start_time = hal_read_cur_time();

		} else {
			if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
				phal_i2c_adapter->status = I2CStatusTimeOut;
				phal_i2c_adapter->err_type = I2CErrorRxAddrTimeOut;
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}
				return HAL_TIMEOUT;

			}
		}
	}

	phal_i2c_adapter->status = I2CStatusRxing;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_recv_poll
 *
 *    hal_rtl_i2c_recv_poll is to receive I2C data by polling (blocking) mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_recv_poll(hal_i2c_adapter_t *phal_i2c_adapter)
{
	i2c_rx_info_t   *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_sts_tmp;
	uint32_t ic_raw_state_tmp;
	uint32_t start_time  = 0;
	uint8_t  wr_ctrl     = 0;

	DBG_I2C_INFO("hal_rtl_i2c_recv_poll\n\r");
	if (!(phal_i2c_adapter->mst_spe_func & I2CAddressRetry)) {
		phal_i2c_adapter->rd_cmd_no = pi2c_rx_info->len;
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_ABRT);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ACT);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_INTR);
	}

	start_time = hal_read_cur_time();
	for (;;) {
		if (pi2c_rx_info->len == 0) {
			break;
		}

		phal_i2c_adapter->status = I2CStatusRxing;
		ic_raw_state_tmp = pi2c_reg->I2C_RAW_INTR_STAT;
		if (((ic_raw_state_tmp) & (I2C_BIT_RAW_RX_UNDER | I2C_BIT_RAW_RX_OVER | I2C_BIT_RAW_TX_OVER | I2C_BIT_RAW_TX_ABRT)) != 0) {
			phal_i2c_adapter->status = I2CStatusError;
			if ((ic_raw_state_tmp & I2C_BIT_RAW_RX_UNDER) != 0) {
				phal_i2c_adapter->err_type = I2CErrorRxUnder;
			} else if ((ic_raw_state_tmp & I2C_BIT_RAW_RX_OVER) != 0) {
				phal_i2c_adapter->err_type = I2CErrorRxOver;
			} else if ((ic_raw_state_tmp & I2C_BIT_RAW_TX_OVER) != 0) {
				phal_i2c_adapter->err_type = I2CErrorTxOver;
			} else if ((ic_raw_state_tmp & I2C_BIT_RAW_TX_ABRT) != 0) {
				phal_i2c_adapter->err_type = I2CErrorTxAbort;
			} else {
				phal_i2c_adapter->err_type = ic_raw_state_tmp;
			}

			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_ERR_HW;
//        } else if (BIT_GET_IC_TFNF(pi2c_reg->status)) {
		} else if (pi2c_reg->I2C_STS & I2C_BIT_STS_TFE) {
			if (phal_i2c_adapter->rd_cmd_no > 0) {
				/* Wrtie cmd into I2C TX FIFO */
				wr_ctrl = 0;
				if (phal_i2c_adapter->rd_cmd_no == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_rx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
				}
				DBG_I2C_INFO("rx wr_ctrl:%x: %x\n\r", wr_ctrl, (pi2c_reg->I2C_STS & I2C_BIT_STS_TFE));
				hal_rtl_i2c_mst_send_rdcmd(phal_i2c_adapter, 1, wr_ctrl);
				phal_i2c_adapter->rd_cmd_no--;
				start_time = hal_read_cur_time();
			}
		} else {
			if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
				phal_i2c_adapter->status = I2CStatusTimeOut;
				phal_i2c_adapter->err_type = I2CErrorRxCmdTimeOut;
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}

				return HAL_TIMEOUT;
			}
		}

		ic_sts_tmp = pi2c_reg->I2C_STS;
		if (((ic_sts_tmp & I2C_BIT_STS_RFNE) != 0) || ((ic_sts_tmp & I2C_BIT_STS_RFF) != 0)) {
			*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
			pi2c_rx_info->buf++;
			pi2c_rx_info->len--;
		}
	}

	if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
		phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
	}

	phal_i2c_adapter->status = I2CStatusIdle;
	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_recv_intr
 *
 *    hal_rtl_i2c_recv_intr is to receive I2C data by interrupt (non-blocking) mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_recv_intr(hal_i2c_adapter_t *phal_i2c_adapter)
{
	i2c_rx_info_t   *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	uint32_t start_time  = 0;
	uint8_t  wr_ctrl = 0;

	DBG_I2C_INFO("hal_i2c_recv_intr_rom\n\r");
	if (!(phal_i2c_adapter->mst_spe_func & I2CAddressRetry)) {
		phal_i2c_adapter->rd_cmd_no = pi2c_rx_info->len;
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_TX_ABRT);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_ACT);
		READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_INTR);
	}

	start_time = hal_read_cur_time();
	phal_i2c_adapter->status = I2CStatusRxReady;
	for (;;) {
		/* check if there is any received data in fifo */
		if ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) || (pi2c_rx_info->len == 0) ||
			(phal_i2c_adapter->rd_cmd_no == 0)) {
			pi2c_reg->I2C_INTR_MSK  = (I2C_BIT_M_TX_ABRT | I2C_BIT_M_TX_EMPTY | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER);
			break;
			//} else if (pi2c_reg->raw_intr_stat & (BIT_RAW_TX_ABRT)){
		} else if (pi2c_reg->I2C_INTR_STAT & I2C_BIT_INTR_TX_ABRT) {
			phal_i2c_adapter->status = I2CStatusError;
			phal_i2c_adapter->err_type = I2CErrorTxAbort;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}
			break;
		} else {
			//if ((phal_i2c_adapter->rd_cmd_no > 0) && (BIT_GET_IC_TFNF(pi2c_reg->status))) {
			if ((phal_i2c_adapter->rd_cmd_no > 0) && (pi2c_reg->I2C_STS & I2C_BIT_STS_TFNF)) {
				/* Wrtie cmd into I2C TX FIFO */
				wr_ctrl = 0;
				if (phal_i2c_adapter->rd_cmd_no == 1) {
					if (phal_i2c_adapter->mst_spe_func & I2CMasterRestart) {
						wr_ctrl = 2;    /* Set RESTART for the next byte */
					} else {
						if (pi2c_rx_info->mst_stop == 1) {
							wr_ctrl = 1;    /* Set STOP for the last byte */
						}
					}
				}

				hal_rtl_i2c_mst_send_rdcmd(phal_i2c_adapter, 1, wr_ctrl);
				phal_i2c_adapter->rd_cmd_no--;
				start_time = hal_read_cur_time();
			} else if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
				phal_i2c_adapter->status   = I2CStatusTimeOut;
				phal_i2c_adapter->err_type = I2CErrorTxCmdTimeOut;
				if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
					phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
				}

				return HAL_TIMEOUT;
			}
		}

	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_set_sar
 *
 *    hal_rtl_i2c_set_sar is to set slave own address in slave mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \param[in] uint8_t sar_idx:                 SAR index.
 *   \param[in] uint16_t slv_addr:               SAR.
 *   \return hal_status_t
 */
SECTION_I2C_TEXT hal_status_t hal_rtl_i2c_set_sar(hal_i2c_adapter_t *phal_i2c_adapter, uint8_t sar_idx, uint16_t slv_addr)
{
	I2C_TypeDef *pi2c_reg = phal_i2c_adapter->init_dat.reg_base;
	uint32_t ic_en_sts_bak;

	ic_en_sts_bak = pi2c_reg->I2C_EN_STS;
	hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CDisable);

	if (!sar_idx) {
		pi2c_reg->I2C_SAR = slv_addr;
	} else {
		pi2c_reg->I2C_SAR1 = slv_addr;
	}

	if (ic_en_sts_bak & I2C_BIT_EN_STS) {
		hal_rtl_i2c_en_ctrl(phal_i2c_adapter, I2CEnable);
	}

	return HAL_OK;
}

/** \brief Description of hal_rtl_i2c_slv_recv_poll
 *
 *    hal_rtl_i2c_slv_recv_poll is to execute slave receive in polling(blocking) mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t
 */
SECTION_I2C_TEXT uint32_t hal_rtl_i2c_slv_recv_poll(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	i2c_rx_info_t *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	uint32_t i2c_rx_len = pi2c_rx_info->len;
	uint32_t start_time  = 0;

	DBG_I2C_INFO("slv secv poll\n\r");
	DBG_I2C_INFO("intr msk: %x\n\r", pi2c_reg->I2C_INTR_MSK);
	phal_i2c_adapter->status = I2CStatusRxReady;
	start_time = hal_read_cur_time();
	while (pi2c_rx_info->len) {
		if (pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) {
			phal_i2c_adapter->status = I2CStatusRxing;
			READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_STRT_DET);
#if 0
			while (pi2c_reg->status & (BIT_IC_RFNE | BIT_IC_RFF)) {
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}
#else
			*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
			pi2c_rx_info->buf++;
			pi2c_rx_info->len--;
#endif
			start_time = hal_read_cur_time();
		} else if ((pi2c_reg->I2C_RAW_INTR_STAT & (I2C_BIT_RAW_STRT_DET | I2C_BIT_RAW_STP_DET)) &&
				   (phal_i2c_adapter->slv_spe_func & I2CSlaveRXBySTPSTR) && (phal_i2c_adapter->status == I2CStatusRxing)) {
			/* detect a START or STOP bit, current receiving would be regarded as complete. */
			/* read rest data in FIFO */
			while (pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) {
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}
			DBG_I2C_INFO("stop recv, l:%x\n\r", pi2c_rx_info->len);
			break;
		} else if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorRxFIFOTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			break;
		}
	}

	if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
		phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
	}

	phal_i2c_adapter->status = I2CStatusIdle;
	return (uint32_t)(i2c_rx_len - pi2c_rx_info->len);
}

/** \brief Description of hal_rtl_i2c_slv_recv_intr
 *
 *    hal_rtl_i2c_slv_recv_intr is to execute slave receive in interrupt(non-blocking) mode.
 *
 *   \param[in] hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t
 */
SECTION_I2C_TEXT uint32_t hal_rtl_i2c_slv_recv_intr(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	i2c_rx_info_t *pi2c_rx_info = (i2c_rx_info_t *) & (phal_i2c_adapter->rx_dat);
	//uint32_t i2c_rx_len = pi2c_rx_info->len;
	//uint32_t start_time  = 0;

	/* if rx fifo is NOT empty, read it out first before further process */
	/* otherwise, clear STOP and START state for a new round process */
	if ((pi2c_reg->I2C_RXFLR != 0) && (phal_i2c_adapter->slv_spe_func & I2CSlaveRXBySTPSTR)) {
		DBG_I2C_INFO("slv has data in fifo\n\r");
		phal_i2c_adapter->status = I2CStatusRxing;

		if (pi2c_rx_info->len == 0) {
			if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
				phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
			}
			phal_i2c_adapter->status = I2CStatusIdle;
			return 0;
		} else if ((pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_STP_DET) || (pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_STRT_DET)) {
			while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {
				*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
				pi2c_rx_info->buf++;
				pi2c_rx_info->len--;
			}
			if (phal_i2c_adapter->usr_cb.rxc.cb != NULL) {
				phal_i2c_adapter->usr_cb.rxc.cb((void *)phal_i2c_adapter->usr_cb.rxc.dat);
			}
			phal_i2c_adapter->status = I2CStatusIdle;
			return 0;
		}

		while ((pi2c_reg->I2C_STS & (I2C_BIT_STS_RFNE | I2C_BIT_STS_RFF)) && (pi2c_rx_info->len > 0)) {
			*(pi2c_rx_info->buf) = pi2c_reg->I2C_DAT_CMD;
			pi2c_rx_info->buf++;
			pi2c_rx_info->len--;
		}
	}

	//READ_CLEAR_I2C_REG(pi2c_reg, clr_stp_det);
	//READ_CLEAR_I2C_REG(pi2c_reg, clr_strt_det);
#if 0
	if (pi2c_reg->rxflr != 0) {
		DBG_8195BL("slv has data in fifo\n\r");
		phal_i2c_adapter->status = I2CStatusRxReady;
		while ((pi2c_reg->status & (BIT_IC_RFNE | BIT_IC_RFF)) && (pi2c_rx_info->len > 0)) {
			*(pi2c_rx_info->buf) = pi2c_reg->data_cmd;
			pi2c_rx_info->buf++;
			pi2c_rx_info->len--;
		}

		return (uint32_t)(i2c_rx_len - pi2c_rx_info->len);
	} else {
		READ_CLEAR_I2C_REG(pi2c_reg, clr_stop_det);
		READ_CLEAR_I2C_REG(pi2c_reg, clr_start_det);
	}
#endif
	/* umask START and STOP interrupt for check end of a transmission */

	if (phal_i2c_adapter->status != I2CStatusRxing) {
		phal_i2c_adapter->status = I2CStatusRxReady;
	}

	if (phal_i2c_adapter->slv_spe_func & I2CSlaveRXBySTPSTR) {
		pi2c_reg->I2C_INTR_MSK  = (I2C_BIT_M_STRT_DET | I2C_BIT_M_STP_DET | I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER);
	} else {
		pi2c_reg->I2C_INTR_MSK  = (I2C_BIT_M_RX_FULL | I2C_BIT_M_RX_OVER | I2C_BIT_M_RX_UNDER);
	}

	return 0;
}

/** \brief Description of hal_rtl_i2c_slv_send_poll
 *
 *    hal_rtl_i2c_slv_send_poll is to execute slave send transfer by polling mode.
 *
 *   \param hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t
 */
SECTION_I2C_TEXT uint32_t hal_rtl_i2c_slv_send_poll(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;
	i2c_tx_info_t *pi2c_tx_info = (i2c_tx_info_t *) & (phal_i2c_adapter->tx_dat);
	uint32_t i2c_tx_len = pi2c_tx_info->len;
	uint32_t start_time  = 0;


	start_time = hal_read_cur_time();
	while (pi2c_tx_info->len) {
		DBG_I2C_INFO("slv send > %d\n\r", pi2c_tx_info->len);

		//if (BIT_GET_RAW_RD_REQ(pi2c_reg->raw_intr_stat)) {
		if (pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_RD_REQ) {
			phal_i2c_adapter->status = I2CStatusTxing;
			READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RD_REQ);
			//if (BIT_GET_IC_TFNF(pi2c_reg->status)) {
			if (pi2c_reg->I2C_STS & I2C_BIT_STS_TFNF) {
				pi2c_reg->I2C_DAT_CMD = *(pi2c_tx_info->buf);
				pi2c_tx_info->buf++;
				pi2c_tx_info->len--;
			}

			start_time = hal_read_cur_time();
			//} else if (pi2c_reg->raw_intr_stat & BIT_RAW_RX_DONE) {
		} else if (pi2c_reg->I2C_RAW_INTR_STAT & I2C_BIT_RAW_RX_DONE) {
			break;
		} else if (pi2c_reg->I2C_RAW_INTR_STAT & (I2C_BIT_RAW_TX_ABRT | I2C_BIT_RAW_TX_OVER)) {
			DBG_I2C_ERR("err:%x\n\r", pi2c_reg->I2C_RAW_INTR_STAT);
			phal_i2c_adapter->status = I2CStatusError;
			phal_i2c_adapter->err_type = I2CErrorTxAbort;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}
		} else if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorRxFIFOTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			break;
		}
	}

	start_time = hal_read_cur_time();
	while (!(pi2c_reg->I2C_STS & I2C_BIT_STS_TFE)) {
		if (hal_rtl_i2c_timeout_chk(phal_i2c_adapter, start_time) == 1) {
			phal_i2c_adapter->status   = I2CStatusTimeOut;
			phal_i2c_adapter->err_type = I2CErrorTxCmdTimeOut;
			if (phal_i2c_adapter->usr_cb.err.cb != NULL) {
				phal_i2c_adapter->usr_cb.err.cb((void *)phal_i2c_adapter->usr_cb.err.dat);
			}

			return HAL_TIMEOUT;
		}
	}

	if (phal_i2c_adapter->usr_cb.txc.cb != NULL) {
		phal_i2c_adapter->usr_cb.txc.cb((void *)phal_i2c_adapter->usr_cb.txc.dat);
	}

	phal_i2c_adapter->status = I2CStatusIdle;
	READ_CLEAR_I2C_REG(pi2c_reg, I2C_CLR_RX_DONE);

	return (uint32_t)(i2c_tx_len - pi2c_tx_info->len);
}

/** \brief Description of hal_rtl_i2c_slv_send_intr
 *
 *    hal_rtl_i2c_slv_send_intr is to execute slave send transfer by interrupt mode.
 *
 *   \param hal_i2c_adapter_t *phal_i2c_adapter:   pointer to I2C control adapter.
 *   \return uint32_t
 */
SECTION_I2C_TEXT uint32_t hal_rtl_i2c_slv_send_intr(hal_i2c_adapter_t *phal_i2c_adapter)
{
	I2C_TypeDef *pi2c_reg = (I2C_TypeDef *)phal_i2c_adapter->init_dat.reg_base;

	phal_i2c_adapter->status = I2CStatusTxReady;
	pi2c_reg->I2C_INTR_MSK  = (I2C_BIT_M_RX_DONE | I2C_BIT_M_TX_ABRT | I2C_BIT_M_RD_REQ | I2C_BIT_M_TX_OVER);
	return 0;
}

#endif

/** @} */ /* End of group hs_hal_i2c_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

/** @} */ /* End of group hs_hal_i2c */

