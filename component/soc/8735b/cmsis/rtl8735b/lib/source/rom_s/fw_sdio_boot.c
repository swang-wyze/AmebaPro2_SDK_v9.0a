/**************************************************************************//**
 * @file     fw_uart_boot.c
 * @brief    Implement the FW image load over UART X-Modem.
 *
 * @version  V1.00
 * @date     2017-05-05
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 ******************************************************************************/

#include "cmsis.h"
#include "fw_img.h"
#include "rtl8735b_symbns4s.h"
#include "rtl8735b_ramstart.h"
#include "memory.h"
#include "efuse_boot_cfg.h"
#include "strproc.h"
#include "fw_sdio_boot.h"
#include "fw_uart_boot.h"
#include "rtl8735b_sdio_dev.h"
#include "hal_pinmux.h"
#include "rtl8735b_misc.h"
#if 0
#include "hal_cache.h"
#include "hal_timer.h"
#include "hal_gpio.h"
#include "hal_irq.h"
#endif

/* if enable Flash image load debug message */
#define SDIO_BOOT_DBG               0

#if SDIO_BOOT_DBG
#define SDIO_BOOT_DBG_PRINTF(...)     do {\
    dbg_printf(__VA_ARGS__);\
}while(0)
#else
#define SDIO_BOOT_DBG_PRINTF(...)
#endif

extern void uart_boot_init(void) ;
extern int32_t uart_boot_frame_handler(char *frame_ptr,  uint32_t frame_num, uint32_t frame_size);

extern uboot_xm_frame_handler_t uboot_frame_handle;

psdiod_rx_packet_t sdio_boot_alloc_rx_pkt(hal_sdio_dev_adapter_t *psdio_adp);

SECTION_ROM_TEMP_BSS static volatile uint8_t sdio_boot_end;
SECTION_ROM_TEMP_BSS hal_sdio_dev_adapter_t sdio_boot_adapt;
SECTION_ROM_TEMP_BSS uint32_t sdio_boot_frame_num;
SECTION_ROM_TEMP_BSS sdiod_txbd_t sdio_boot_tx_bd[SDIO_BOOT_TX_BD_NUM] __ALIGNED(32);
SECTION_ROM_TEMP_BSS sdiod_txbd_hdl_t sdio_boot_tx_bd_hdl[SDIO_BOOT_TX_BD_NUM];
SECTION_ROM_TEMP_BSS uint8_t sdio_boot_tx_buf[SDIO_BOOT_TX_BD_NUM][SDIO_BOOT_TX_BD_BUF_SIZE] __ALIGNED(32);

SECTION_ROM_TEMP_BSS sdiod_rxbd_t sdio_boot_rx_bd[SDIO_BOOT_RX_BD_NUM] __ALIGNED(32);
SECTION_ROM_TEMP_BSS sdiod_rxbd_hdl_t sdio_boot_rx_bd_hdl[SDIO_BOOT_RX_BD_NUM] __ALIGNED(32);
SECTION_ROM_TEMP_BSS sdiod_rx_desc_t sdio_boot_rx_pkt_desc[SDIO_BOOT_RX_PKT_NUM] __ALIGNED(32);
SECTION_ROM_TEMP_BSS sdiod_rx_packet_t sdio_boot_rx_pkt_hdl[SDIO_BOOT_RX_PKT_NUM] __ALIGNED(32);

static const uint8_t sdio_dev_pins[] = {
	PIN_SDIO_D2,  PIN_SDIO_D3, PIN_SDIO_CMD,
	PIN_SDIO_CLK, PIN_SDIO_D0, PIN_SDIO_D1,
	PIN_SDIO_INT, 0xFF
};

/**
 *  @brief To send the data of the given memory address to the SDIO host.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @param[in]  addr  the memory read start address.
 *  @param[in]  len the length of data to read.
 *
 *  @return     SUCCESS  the packet has been put into the RX packet pending queue.
 *  @return     FAIL  failed to send the packet, resource insufficient.
 */
int8_t sdio_boot_read_mem(hal_sdio_dev_adapter_t *psdio_adp, psdiod_tx_desc_mr_t ptx_desc_mr)
{
	SDIO_DEV_Type *sdio_dev = SDIO_DEV;
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const hal_sdiod_func_stubs_t *psdiod_stubs = symb_ns4s_stubs->phal_sdiod_stubs;
	psdiod_rx_desc_mr_t prx_desc;
	sdiod_rx_packet_t *ppkt;
	uint8_t *addr;
	uint16_t len;

	addr = (uint8_t *)ptx_desc_mr->start_addr;
	len =  ptx_desc_mr->read_len;

	DBG_SDIO_DEV_INFO("sdio_boot_read_mem: Addr=0x%x, Len=%d\n", (u32)addr, len);

	sdio_dev->ccpwm2 &= ~SDIO_BOOT_MEM_RD_DONE;

	ppkt = sdio_boot_alloc_rx_pkt(psdio_adp);
	if (ppkt == NULL) {
		DBG_BOOT_ERR("sdio_boot_read_mem: Err!! No Free RX PKT!\n");
		return FAIL;
	}

	prx_desc = (psdiod_rx_desc_mr_t)(ppkt->prx_desc);
	prx_desc->type = SDIO_BOOT_CMD_MEMRD_RSP;
	prx_desc->pkt_len = len;
	prx_desc->offset = sizeof(sdiod_rx_desc_t);
	prx_desc->start_addr = (uint32_t)addr;
	ppkt->pdata = addr;
	ppkt->offset = 0;
	psdiod_stubs->rx_pkt_enqueue(psdio_adp, ppkt);

	// Update the CPWM2 register to indicate the host command is done
	sdio_dev->ccpwm2 |= SDIO_BOOT_MEM_RD_DONE;
	sdio_dev->ccpwm2 ^= SDIO_BOOT_CPWM2_TOGGLE;

	return SUCCESS;
}

/**
 *  @brief To handle the SDIO boot memory write command.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @param[in]  dst_addr  the memory write destenation address.
 *  @param[in]  src_addr  the memory write source address.
 *  @param[in]  len the length of data to write.
 *  @param[in]  reply indicates a packet to response to this memory write command is needed or not.
 *
 *  @return     SUCCESS  the packet has been put into the RX packet pending queue.
 *  @return     FAIL  failed to send the packet, resource insufficient.
 */
int8_t sdio_boot_write_mem(hal_sdio_dev_adapter_t *psdio_adp, psdiod_tx_desc_mw_t ptx_desc_mw, uint8_t *src_addr)
{
	SDIO_DEV_Type *sdio_dev = SDIO_DEV;
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const hal_sdiod_func_stubs_t *psdiod_stubs = symb_ns4s_stubs->phal_sdiod_stubs;
	uint8_t *dst_addr = (uint8_t *)ptx_desc_mw->start_addr;
	uint32_t len = (uint32_t)ptx_desc_mw->write_len;

	sdio_dev->ccpwm2 &= ~SDIO_BOOT_MEM_WR_DONE;
	sdio_boot_frame_num++;
	dbg_printf("%s=>%lu\r\n", __func__, sdio_boot_frame_num);
	if (ptx_desc_mw->is_raw_dat == 0) {
		uart_boot_frame_handler((char *)src_addr, sdio_boot_frame_num, len);
	} else {
		_memcpy(dst_addr, src_addr, len);
	}

	// Update the CPWM2 register to indicate the host command is done
	sdio_dev->ccpwm2 |= SDIO_BOOT_MEM_WR_DONE;
	sdio_dev->ccpwm2 ^= SDIO_BOOT_CPWM2_TOGGLE;

	if (ptx_desc_mw->reply) {
		sdiod_rx_desc_mw_t *prx_desc;
		sdiod_rx_packet_t *ppkt;

		ppkt = sdio_boot_alloc_rx_pkt(psdio_adp);
		if (ppkt == NULL) {
			DBG_BOOT_ERR("sdio_boot_write_mem: Err!! No Free RX PKT!\n");
			return FAIL;
		}

		prx_desc = (psdiod_rx_desc_mw_t)(ppkt->prx_desc);
		prx_desc->type = SDIO_BOOT_CMD_MEMWR_RSP;
		prx_desc->pkt_len = 0;
		prx_desc->offset = sizeof(sdiod_rx_desc_t);
		prx_desc->start_addr = (uint32_t)dst_addr;
		prx_desc->write_len = len;
		prx_desc->result = 0;
		ppkt->pdata = NULL;
		ppkt->offset = 0;
		psdiod_stubs->rx_pkt_enqueue(psdio_adp, ppkt);
	}
	return SUCCESS;
}

/******************************************************************************
 * Function: SDIO_SetMem_Rom
 * Desc: To handle a memory set command from Host side.
 *
 * Para:
 *   pSDIODev: The SDIO device data structor.
 *   DesAddr: the memory set destenation address
 *   Data: the value to be write to the memory.
 *   Len: The length of data to written.
 *   Reply: Is need to send a packet to response the memory set command.
 ******************************************************************************/
int8_t sdio_boot_set_mem(hal_sdio_dev_adapter_t *psdio_adp, psdiod_tx_desc_ms_t ptx_desc_ms)
{
	SDIO_DEV_Type *sdio_dev = SDIO_DEV;
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const hal_sdiod_func_stubs_t *psdiod_stubs = symb_ns4s_stubs->phal_sdiod_stubs;
	uint8_t *dst_addr;
	uint8_t data;
	uint16_t len;

	dst_addr = (uint8_t *)ptx_desc_ms->start_addr;
	data = ptx_desc_ms->data;
	len = ptx_desc_ms->write_len;

	sdio_dev->ccpwm2  &= ~SDIO_BOOT_MEM_ST_DONE;

	_memset(dst_addr, data, len);

	// Update the CPWM2 register to indicate the host command is done
	sdio_dev->ccpwm2 |= SDIO_BOOT_MEM_ST_DONE;
	sdio_dev->ccpwm2 ^= SDIO_BOOT_CPWM2_TOGGLE;

	if (ptx_desc_ms->reply) {
		psdiod_rx_desc_ms_t prx_desc;
		sdiod_rx_packet_t *ppkt;

		ppkt = sdio_boot_alloc_rx_pkt(psdio_adp);
		if (ppkt == NULL) {
			DBG_BOOT_ERR("sdio_boot_set_mem: Err!! No Free RX PKT!\n");
			return FAIL;
		}

		prx_desc = (psdiod_rx_desc_ms_t)(ppkt->prx_desc);
		prx_desc->pkt_len = 0;
		prx_desc->offset = sizeof(psdiod_rx_desc_ms_t);
		prx_desc->type = SDIO_BOOT_CMD_MEMST_RSP;
		prx_desc->start_addr = (u32)dst_addr;
		prx_desc->write_len = len;
		prx_desc->result = 0;
		ppkt->pdata = NULL;
		ppkt->offset = 0;
		psdiod_stubs->rx_pkt_enqueue(psdio_adp, ppkt);
	}
	return SUCCESS;
}

void sdio_boot_tx_pkt_handle(hal_sdio_dev_adapter_t *psdio_adp, uint8_t *ppkt)
{
	SDIO_DEV_Type *sdio_dev = SDIO_DEV;
	union {
		psdiod_tx_desc_t ptx_desc;
		psdiod_tx_desc_mr_t ptx_desc_mr;
		psdiod_tx_desc_mw_t ptx_desc_mw;
		psdiod_tx_desc_ms_t ptx_desc_ms;
		psdiod_tx_desc_js_t ptx_desc_js;
	} pdesc;
	uint8_t cmd_type;

	pdesc.ptx_desc = (psdiod_tx_desc_t)(ppkt);
	cmd_type = pdesc.ptx_desc->type;

	DBG_SDIO_DEV_INFO("sdio_boot_tx_pkt_handle => cmd_type = 0x%x\n", cmd_type);

	switch (cmd_type) {
	case SDIO_BOOT_CMD_MEMRD:
		DBG_SDIO_DEV_INFO("Mem Read @ 0x%x, len=%d\n", pdesc.ptx_desc_mr->start_addr, pdesc.ptx_desc_mr->read_len);
		sdio_boot_read_mem(psdio_adp, pdesc.ptx_desc_mr);
		break;

	case SDIO_BOOT_CMD_MEMWR:
		DBG_SDIO_DEV_INFO("Mem Write @ 0x%x, len=%d, reply=%d\n",
						  pdesc.ptx_desc_mw->start_addr, pdesc.ptx_desc_mw->write_len, pdesc.ptx_desc_mw->reply);
		sdio_boot_write_mem(psdio_adp, pdesc.ptx_desc_mw, (uint8_t *)(ppkt + pdesc.ptx_desc_mw->offset));
		break;

	case SDIO_BOOT_CMD_MEMST:
		DBG_SDIO_DEV_INFO("Mem Set @ 0x%x, len=%d, reply=%d\n",
						  pdesc.ptx_desc_ms->start_addr, pdesc.ptx_desc_ms->write_len, pdesc.ptx_desc_ms->reply);
		sdio_boot_set_mem(psdio_adp, pdesc.ptx_desc_ms);
		break;

	case SDIO_BOOT_CMD_STARTUP:
		DBG_SDIO_DEV_INFO("Jump to Entry Func @ 0x%x\n", pdesc.ptx_desc_js->start_fun);

		if (pdesc.ptx_desc_js->start_valid) {
			uboot_frame_handle.ram_start_tbl = pdesc.ptx_desc_js->start_fun;
		}

		if (pdesc.ptx_desc_js->disconnect) {
			/* Indicate the Host system that the TX/RX is NOT ready */
			sdio_dev->sys_ind_b.sys_cpu_rdy_ind = 0;
			sdio_dev->int_mask = 0; // disable all interrupt
			hal_irq_disable_rtl8195bhp(SDIOD_IRQn);
			__ISB();

			// Indicate the Host that Ameba is InActived
			sdio_dev->ccpwm2_b.active = 0;
			sdio_dev->ccpwm2_b.toggle ^= 1;

			// Reset SDIO DMA
			sdio_dev->sdio_ctrl_b.sdio_dma_rst = 1;

			sdio_boot_end = 1;
		}
		break;
	}

}

hal_status_t sdio_boot_txbd_hdl_init(sdiod_txbd_hdl_t *ptxbd_hdl, uint16_t txbd_idx)
{
	hal_status_t ret = HAL_OK;

	// Allocate buffer for each TX BD
	ptxbd_hdl->ptxbd->addr = (uint32_t)&sdio_boot_tx_buf[txbd_idx][0];

	return ret;
}

void sdio_boot_txbd_hdl_deinit(sdiod_txbd_hdl_t *ptxbd_hdl, uint16_t txbd_idx)
{
	return;
}

void sdio_boot_txbd_buf_do_refill(sdiod_txbd_hdl_t *ptxbd_hdl)
{
	return;
}

int8_t sdio_boot_txbd_rdy_callback(hal_sdio_dev_adapter_t *psdio_adp, sdiod_txbd_hdl_t *ptxbd_hdl,
								   sdiod_tx_desc_t *ptx_desc)
{
	// packet data includes TX Desc
	sdio_boot_tx_pkt_handle(psdio_adp, (uint8_t *)ptx_desc);
	return SUCCESS;
}

void sdio_boot_rxbd_tr_done_callback(hal_sdio_dev_adapter_t *psdio_adp, sdiod_rxbd_hdl_t *prxbd_hdl)
{
	sdiod_rx_packet_t *ppkt;
	sdiod_rx_desc_t *prx_desc;

	ppkt = prxbd_hdl->ppkt;
	prx_desc = ppkt->prx_desc;

	if (psdio_adp->rx_done_callback != NULL) {
		psdio_adp->rx_done_callback(psdio_adp->prx_done_cb_para, (void *)ppkt->pdata, \
									ppkt->offset, prx_desc->pkt_len, prx_desc->type);
	}
}

/**
 *  @brief Gets a RX packet from the RX packets queue. A RX packet is used to
 *         handle a data packet which will be send to the SDIO host. The SDIO
 *         device HAL initialization function will allocate some RX packets
 *         and put them in the RX free packets queue. This function will takes
 *         a RX packet out from this RX free packet queue.
 *  @param[in]  psdio_adp  the SDIO device HAL adapter.
 *  @return     value != NULL  the gotten RX packet.
 *  @return     value == NULL  failed to get RX packet(queue is empty).
 */
psdiod_rx_packet_t sdio_boot_alloc_rx_pkt(hal_sdio_dev_adapter_t *psdio_adp)
{
	struct list_head *plist;
	sdiod_rx_packet_t *ppkt;

	if (list_empty(&psdio_adp->free_rx_pkt_list)) {
		DBG_BOOT_ERR("sdio_boot_dev_alloc_rx_pkt: no free pkt\n");
		return NULL;
	}

	plist = psdio_adp->free_rx_pkt_list.next;
	ppkt = container_of(plist, sdiod_rx_packet_t, list);

	list_del_init(&ppkt->list);

	return ppkt;
}

void sdio_boot_free_rx_pkt(hal_sdio_dev_adapter_t *psdio_adp, sdiod_rx_packet_t *ppkt)
{
	list_add_tail(&ppkt->list, &psdio_adp->free_rx_pkt_list);
}

hal_status_t sdio_boot_init(void)
{
	SDIO_DEV_Type *sdio_dev = SDIO_DEV;
	symb_ns4s_t *symb_ns4s_stubs = (symb_ns4s_t *)(NS_SYMBOL_STUBS_ADDR);
	const hal_sdiod_func_stubs_t *psdiod_stubs = symb_ns4s_stubs->phal_sdiod_stubs;
	const hal_gpio_func_stubs_t *pgpio_stubs = symb_ns4s_stubs->phal_gpio_stubs;
	hal_sdio_dev_adapter_t *psdio_adp;
	hal_status_t ret;
	uint16_t tx_bd_num;
	uint16_t rx_bd_num;
	uint32_t i;
	sdiod_rx_packet_t *prx_pkt;

	psdio_adp = &sdio_boot_adapt;
	_memset((void *)psdio_adp, 0, sizeof(hal_sdio_dev_adapter_t));

	ret = hal_pinmux_register_rtl8195bhp((io_pin_t *)sdio_dev_pins, PID_SDIOD);
	if (ret != HAL_OK) {
		DBG_SDIO_DEV_ERR("sdio_boot_init: Pin Mux Err!\r\n");
		return ret;
	}

	pgpio_stubs->hal_gpio_schmitt_ctrl(PIN_SDIO_CLK, ON, IO_3p3V);   // Enable Smith trigger on SDIO_CLK pin

	psdio_adp->dcache_clean_by_addr = dcache_clean_by_addr_rtl8195bhp;
	psdio_adp->dcache_invalidate_by_addr = dcache_invalidate_by_addr_rtl8195bhp;

	tx_bd_num = SDIO_BOOT_TX_BD_NUM;
	rx_bd_num = SDIO_BOOT_RX_BD_NUM;
	psdio_adp->tx_bd_buf_size = SDIO_BOOT_TX_BD_BUF_SIZE;

	/// TX path init
	/**** Initial SDIO TX: initial TX BD, TX BD Handle, TX Buffer ****/
	DBG_SDIO_DEV_INFO("Tx BD Init==>\n");

	// allocate a buffer for TX BD. for cache sync consideration we need to make the TX BD
	// start with 32bytes aligned address
	psdio_adp->ptxbd_addr = (uint8_t *)sdio_boot_tx_bd;
	psdio_adp->tx_bd_num = tx_bd_num;

	psdio_adp->ptxbd_hdl = sdio_boot_tx_bd_hdl;

	psdio_adp->txbd_hdl_init = sdio_boot_txbd_hdl_init;
	psdio_adp->txbd_hdl_deinit = sdio_boot_txbd_hdl_deinit;
	psdio_adp->txbd_buf_refill = sdio_boot_txbd_buf_do_refill;
	psdio_adp->rxbd_tr_done_callback = sdio_boot_rxbd_tr_done_callback;
	psdio_adp->txbd_rdy_callback = sdio_boot_txbd_rdy_callback;
	psdio_adp->tx_callback = (sdiod_tx_callback_t)sdio_boot_txbd_rdy_callback;
	psdio_adp->free_rx_pkt = sdio_boot_free_rx_pkt;

	/// RX Path init
	psdio_adp->rx_bd_num = rx_bd_num;
	// hardware design is the RX BD start address must be aligned to 8-bytes,
	// but for cache sync consideration we make it align to 32-bytes
	psdio_adp->prxbd_addr = (uint8_t *)sdio_boot_rx_bd;
	psdio_adp->free_rx_bd_cnt = 1;

	psdio_adp->prxbd_hdl = sdio_boot_rx_bd_hdl;

	INIT_LIST_HEAD(&psdio_adp->free_rx_pkt_list);   // Init the list for free packet handler

	/* Allocate memory for RX Packet descripters */
	// make the RX packet descripters start with 32-bytes aligned address, for the D-cache sync. issue.
	psdio_adp->prx_desc_buf = (uint8_t *)sdio_boot_rx_pkt_desc;
	psdio_adp->prx_desc_addr = sdio_boot_rx_pkt_desc;

	/* Allocate memory for RX Packets handler */
	psdio_adp->prx_pkt_handler = sdio_boot_rx_pkt_hdl;

	/* Add all RX packet handler into the Free Queue(list) */
	for (i = 0; i < SDIO_BOOT_RX_PKT_NUM; i++) {
		prx_pkt = psdio_adp->prx_pkt_handler + i;
		prx_pkt->prx_desc = psdio_adp->prx_desc_addr + i;
		list_add_tail(&prx_pkt->list, &psdio_adp->free_rx_pkt_list);
	}

	INIT_LIST_HEAD(&psdio_adp->pend_rx_pkt_list);   // Init the list for RX packet to be send to the SDIO bus

	ret = psdiod_stubs->init(psdio_adp);
	psdio_adp->ccpwm.w = 0;
	sdio_dev->ccpwm = psdio_adp->ccpwm.w;

	// Update the CPWM register to indicate the host sdio init is done
	psdio_adp->ccpwm2.w = sdio_dev->ccpwm2;
	psdio_adp->ccpwm2.w |= SDIO_BOOT_INIT_DONE;
	psdio_adp->ccpwm2.w ^= SDIO_BOOT_CPWM2_TOGGLE;
	sdio_dev->ccpwm2 = psdio_adp->ccpwm2.w;

	if (ret != HAL_OK) {
		DBG_BOOT_ERR("SDIO Boot init err (0x%x)\r\n", ret);
	}

	return ret;
}

int32_t sdio_boot(PRAM_FUNCTION_START_TABLE *pram_start_func)
{
	hal_status_t hal_ret;
	uint32_t wait_cnt;

#if SDIO_BOOT_DBG
	DBG_INFO_MSG_ON(_DBG_MISC_ | _DBG_SDIO_DEV_);
	DBG_WARN_MSG_ON(_DBG_MISC_ | _DBG_SDIO_DEV_);
#endif

	// use UART boot mechanism to load image, just the packet comes from SDIO
	uart_boot_init();

	hal_ret = sdio_boot_init();
	if (hal_ret != HAL_OK) {
		return FAIL;
	}
	sdio_boot_end = 0;

	wait_cnt = 0;
	while (sdio_boot_end == 0) {
		hal_delay_ms(1);
		wait_cnt++;
		// wait 100sec
//        if (wait_cnt > 100000) {
		if (wait_cnt > 30000) {
			dbg_printf("SDIO boot timeout!!\r\n");
			return FAIL;
		}
	}

	if (sdio_boot_frame_num > 0) {
		dbg_printf("Load Image over SDIO: ram_start_tbl @0x%x\r\n", uboot_frame_handle.ram_start_tbl);
		if (uboot_frame_handle.ram_start_tbl != 0) {
			*pram_start_func = (PRAM_FUNCTION_START_TABLE)(uboot_frame_handle.ram_start_tbl);
			return SUCCESS;
		} else {
			return FAIL;
		}
	} else {
		dbg_printf("Load Image over SDIO Failed\r\n");
		return FAIL;
	}
}


