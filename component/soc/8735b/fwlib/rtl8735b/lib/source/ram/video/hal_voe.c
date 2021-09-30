/**************************************************************************//**
 * @file     hal_voe.c
 * @brief    This file implements the entry functions of the VOE HAL RAM functions.
 *
 * @version  V1.00
 * @date     2017-011-11
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
#include "platform_stdlib.h"

#include "cmsis.h"
#include "cmsis_os.h"               // CMSIS RTOS header file

#include "rtl8735b_voe_cmd.h"
#include "rtl8735b_voe_type.h"
#include "hal.h"
#include "base_type.h"
#include "hal_voe.h"
#include "fw_voe_rom_boot.h"


// converter LF --> CRLF
#define printf(fmt, arg...)				printf(fmt"\r", ##arg)


#define VOE_DDR_HEADER		64				// VOE DDR HEADER
#define SENSOR_FW_HEADER	32				// SENSOR FW HEADER
#define SENSOR_FW_SIZE		8192 			// VOE DDR¡@HEADER + SENSOR

static VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;
static xSemaphoreHandle voe_semaphore;

char hal_voe_version[] = "RTL8735B_VOE_0.4.0.4";

/**
 * @addtogroup hs_hal_voe VOE
 * @{
 * @brief The VOE HAL APIs.
 */

/*
  \brief  The data structure to handle the voe fw header, includes the year, month, day, compiler info, image size.
*/
typedef struct voe_header_s {
	u16 year;				// 2B
	u8 month;				// 1B
	u8 day;					// 1B
	u8 version[24];			// 24B
	u32 itcm_size;			// 4B
	u32 dtcm_size;			// 4B
	u32 ddr_size;			// 4B
} voe_header_t;


static hal_voe_adapter_t g_pvoe_adpt;

int voe_addr;

/**
 *  @brief Send command to VOE
 *
 *  @param[in]  cmd			:	Send command. len: 14 bit
 *  @param[in]  param1		:	Send parameter data 1. len:32 bit
 *  @param[in]  param2		:	Send parameter data 2. len:32 bit
 *  @param[in]  pvoe_adpt	:	The data structure to handle the voe module.
 *
 *  @returns Result.
 *
 */
u32 hal_voe_send2voe(u32 cmd, u32 param1, u32 param2)
{
	xSemaphoreTake(voe_semaphore, portMAX_DELAY);
	u32 tm_send_message;
	g_pvoe_adpt.voe2tm_ack = 0;
	/*Send command to VOE*/
	voe->VOE_REG_KM_FW_BASE_ADDRESS = param1;	// TX param1
	voe->VOE_REG_KM_FW_LEN          = param2;	// TX param2
	uint32_t t1, t2;
	g_pvoe_adpt.voe_cmd = 0;
	tm_send_message = ((cmd & 0x3FFF) << 16 | BIT31);
	while ((voe->VOE_REG_SEND_MESSAGE_EXTERNAL_CTRL & VOE_BIT_INVALID_AP_QUEUE) != 0) {
		printf("voe is busy cmd=%x\r\n", cmd);
	}

	voe->VOE_REG_SEND_MESSAGE_EXTERNAL_CTRL = tm_send_message;

	t1 = hal_read_curtime_us();


	while(g_pvoe_adpt.voe_cmd != cmd) {
//		printf("<<test>><%s><%d>%x %x\n",__func__,__LINE__,g_pvoe_adpt.voe_cmd,cmd);
		vTaskDelay(1);			// busy waiting ACK
	}
	t2 = hal_read_curtime_us();
	if ( (t2 - t1) > 10000 ) {
		printf("hal_voe_send2voe too long %d cmd 0x%08x p1 0x%08x p2 0x%08x\n", t2 - t1, cmd, param1, param2);
	}

	g_pvoe_adpt.voe2tm_ack = 0;

    xSemaphoreGive(voe_semaphore);
	return OK;
}





hal_status_t hal_voe_erac_init(voe_callback_t wait_cmd_sent_cb, void *cb_para)
{

	uint16_t tm_receive_cmd;

	// Write ERAC init cmd to KM
	g_pvoe_adpt.voe_send_message = 0;
	hal_voe_send2voe(FW_ERAC_INIT_CMD, NULL, NULL);

	if (wait_cmd_sent_cb != NULL) {
		wait_cmd_sent_cb(cb_para);

		tm_receive_cmd = PARSE_TM_CMD(g_pvoe_adpt.voe_send_message);

		if (FW_RET_ERAC_OK == tm_receive_cmd) {
			return HAL_OK;
		} else  if (0 == tm_receive_cmd) {
			return HAL_TIMEOUT;
		} else {
			return HAL_ERR_UNKNOWN;
		}

	} else {
		return HAL_ERR_PARA;
	}


}
#if 0
hal_status_t hal_voe_erac_wr(u32 data_addr, u32 offset, u16 wr_num, voe_callback_t callback, void *arg)
{


	u32 enc_reg_addr = H264_REG_BASE + offset;
	u32 wr_reg_offset = enc_reg_addr - VOE_ADDR_BASE;
	u32 data_offset = data_addr - VOE_ADDR_BASE;
	hal_voe_write_cmd_data1_rtl8195bhp(data_offset);
	hal_voe_write_cmd_data2_rtl8195bhp(wr_reg_offset);



	if (hal_voe_check_message_voe_queue_rtl8195bhp() || hal_voe_check_message_ap_queue_rtl8195bhp()) {
		DBG_VOE_ERR("\r\n %s: previous vaild message need to be processed %d %d \r\n", __FUNCTION__, hal_voe_check_message_voe_queue_rtl8195bhp(), hal_voe_check_message_ap_queue_rtl8195bhp());	
        return HAL_BUSY;
    }

	if (callback != NULL) {
		g_pvoe_adpt.erac_finish_cb = callback;			// ERAC finish callback
		g_pvoe_adpt.erac_finish_cb_para = arg;
	}

	g_pvoe_adpt.voe_send_message = 0;
	hal_voe_send2voe(FW_ERAC_WRITE_CMD, wr_num, g_pvoe_adpt);
	// busy waiting receive command from VOE
//	hal_voe_receive_cmd(FW_RET_ERAC_OK, g_pvoe_adpt);


	return HAL_OK;


}

hal_status_t hal_voe_check_erac_cmd_status(void)
{

	u16 tm_receive_cmd;

	tm_receive_cmd = PARSE_TM_CMD(g_pvoe_adpt.voe_send_message);

	if (FW_RET_ERAC_OK == tm_receive_cmd) {
		return HAL_OK;
	} else  if (0 == tm_receive_cmd) {
		return HAL_TIMEOUT;
	} else {
		return HAL_ERR_UNKNOWN;
	}
}

#endif




bool hal_voe_check_erac_write_cmd(void)
{
	if (FW_ERAC_WRITE_CMD == g_pvoe_adpt.voe_cmd) {
		return TRUE;
	} else {
		return FALSE;
	}
	return TRUE;
}




/**
 *  @brief To handle the VOE interrupt.
 *
 *  @param void
 *
 *  @returns void
 */
static void VOE_IRQHandler(void)
{
	uint32_t read_value;
	u32 param1 = voe->VOE_REG_KM_FW_LEN;
	read_value = voe->VOE_REG_INTERRUPT_STATUS_OF_EXTERNAL_CPU;
	u32 cmd;
	if (read_value & VOE_BIT_KM_MESSAGE_ISR_STATUS) {
		g_pvoe_adpt.voe_send_message = voe->VOE_REG_READ_MESSAGE_EXTERNAL_CTRL & VOE_MASK_MESSAGE;


		cmd = PARSE_TM_CMD(voe->VOE_REG_READ_MESSAGE_EXTERNAL_CTRL);
		if(cmd != VOE_OUT_CMD) {
			g_pvoe_adpt.voe_cmd = cmd;
//			printf("<<test>><%s><%d> cmd= %x\n",__func__,__LINE__, cmd);
		}

		voe->VOE_REG_READ_MESSAGE_EXTERNAL_CTRL = ~VOE_BIT_INVALID_VOE_QUEUE;
		voe->VOE_REG_INTERRUPT_STATUS_OF_EXTERNAL_CPU = VOE_BIT_KM_MESSAGE_ISR_STATUS;
		if (g_pvoe_adpt.km_message_cb) {
			g_pvoe_adpt.km_message_cb((void *)param1);
		}
	}
	if (read_value & VOE_BIT_ERAC_FINISH) {		// ERAC finish callback
		voe->VOE_REG_INTERRUPT_STATUS_OF_EXTERNAL_CPU = VOE_BIT_ERAC_FINISH;

		if (g_pvoe_adpt.erac_finish_cb) {
			g_pvoe_adpt.erac_finish_cb(g_pvoe_adpt.erac_finish_cb_para);
		}
	}

	g_pvoe_adpt.voe2tm_ack = 1;
	hal_irq_clear_pending(VOE_IRQn);
	//dbg_printf("int 0x%08x\r\n", read_value);

}

/**
 *  @brief To handle the VOE¡@WDT interrupt.
 *
 *  @param void
 *
 *  @returns void
 */
void VOE_WDT_IRQHandler(void)
{
	printf("<<test>><%s><%d>\n",__func__,__LINE__);

	voe->VOE_REG_VOE_WATCHDOG_CTRL |= VOE_BIT_WDT_TO;
	voe->VOE_REG_VOE_WATCHDOG_CTRL &= ~VOE_BIT_WDT_EN_BYTE;

    NVIC_ClearPendingIRQ(WDT_VOE_IRQn);    // while(1);
//	hal_irq_disable(WDT_VOE_IRQn);
}

/**
 *  @brief To register a IRQ handler for the VOE interrupt.
 *
 *  @param irq_handler The IRQ handler.
 *
 *  @returns void
 */

static __inline void hal_voe_irq_reg(uint32_t irq_handler)
{

	hal_irq_disable(VOE_IRQn);
	__ISB();
	hal_irq_set_vector(VOE_IRQn, irq_handler);
	hal_irq_set_priority(VOE_IRQn, VOE_IRQPri);
	hal_irq_clear_pending(VOE_IRQn);
	hal_irq_enable(VOE_IRQn);

}

static __inline void hal_voe_set_km_message_int(u8 int_en)
{
	volatile uint32_t val;
	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	if (int_en == ENABLE) {

		val = voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU;
		val |= VOE_BIT_KM_MESSAGE_ISR_EN;
		voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU = val;

	} else { //en == DISABLE

		val = voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU;
		val &= ~VOE_BIT_KM_MESSAGE_ISR_EN;
		voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU = val;
	}
}

/**
 *  @brief Set KM ERAC to TM9 interrupt.
 *
 *  @param int_en The bit map to enable/disable the interrupt.
 *
 *  @returns void
 */
void hal_voe_set_erac_int(u8 int_en)
{

	volatile uint32_t val;
	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	if (int_en == ENABLE) {

		val = voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU;
		val |= VOE_BIT_ERAC_AP_FINISH_EN;
		voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU = val;

	} else { //en == DISABLE

		val = voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU;
		val &= ~VOE_BIT_ERAC_AP_FINISH_EN;
		voe->VOE_REG_INTERRUPT_MASK_OF_EXTERNAL_CPU = val;
	}

}

/**
 *  @brief To Set VOE watch dog
 *  This function be set voe watch dog.
 *
 *
 *  @param[in]  enhable			:	enable/disable VOE watch dog
 *
 *  @returns int 				:	Set watch dog result OK/NOK.
 *
 */
int hal_voe_set_wdt(int sec)
{

	if(sec) {
		hal_irq_set_vector(WDT_VOE_IRQn, (uint32_t)VOE_WDT_IRQHandler);
		hal_irq_set_priority(WDT_VOE_IRQn, WDT_VOE_IRQPri);
		hal_irq_clear_pending(WDT_VOE_IRQn);
		hal_irq_enable(WDT_VOE_IRQn);
	}
	else {	// disable
		hal_irq_disable(WDT_VOE_IRQn);
	}

	hal_voe_send2voe(VOE_SET_WDT_CMD, sec, 0);
	return OK;
}

/**
 *  @brief To Load Sensor firmware binary.
 *  This function be load Sensor firmware binary.
 *
 *
 *  @param[in]  voe_cpy			:	call back function replace memcpy (DMA or CPU copy)
 *  @param[in]  fw_addr			:	Sensor firmware binary address
 *  @param[in]  voe_ddr_addr	:	VOE code text use DDR address
 *
 *  @returns int 				:	Sensor load firmware result OK/NOK.
 *
 */
int hal_voe_load_sensor(voe_cpy_t voe_cpy, int *fw_addr, int *voe_ddr_addr)
{
	/*
	 *	sensor.bin format layout
	 *
	 *	/			sensor header	32B						/
	 *	/	voe year	/	voe month	/	voe day		/
	 *	/		2B		/		1B		/		1B		/
	 *	/	Version		/	sensor size	/
	 *	/		24B		/		4B		/
	 *
	 *  /	sensor data	/
	 *  /		XXB		/
	 */

	u32 *addr = (u32 *)fw_addr;
	voe_header_t *voe_header = (voe_header_t *)fw_addr;

	printf("sensor:date %d/%d/%d version:%s\n", voe_header->year, voe_header->month, voe_header->day, voe_header->version);
	printf("sensor:FW size (%d)\n", voe_header->itcm_size);

	voe_addr = (int)voe_ddr_addr;
	memcpy((void *)(voe_ddr_addr+8),(void *)(voe_header->version),24);

	voe_cpy((void *)(voe_ddr_addr+(VOE_DDR_HEADER>>2))
			, addr + (SENSOR_FW_HEADER>>2)
			, (size_t)voe_header->itcm_size);

	dcache_clean_invalidate_by_addr((uint32_t *)(voe_ddr_addr+(VOE_DDR_HEADER>>2))
									, (int32_t)voe_header->itcm_size);
	return OK;
}

/**
 *  @brief To Load VOE firmware binary.
 *  This function be load VOE firmware binary.
 *
 *
 *  @param[in]  voe_cpy			:	call back function replace memcpy (DMA or CPU copy)
 *  @param[in]  fw_addr			:	VOE firmware binary address
 *  @param[in]  voe_ddr_addr	:	VOE code text use DDR address
 *
 *  @returns int 				:	VOE load firmware result OK/NOK.
 *
 */
int hal_voe_load_fw(voe_cpy_t voe_cpy, int *fw_addr, int *voe_ddr_addr)
{


	/*
	 *	voe.bin format layout
	 *
	 *	/			voe header	40B						/
	 *	/	voe year	/	voe month	/	voe day		/
	 *	/		2B		/		1B		/		1B		/
	 *	/	Version		/
	 *	/		24B		/
	 *  /	itcm size	/	dtcm size	/	ddr size	/
	 *  /		4B		/		4B		/ 		4B		/
	 *
	 *	/					voe data					/
	 *  /	itcm data	/	dtcm data	/   ddr date    /
	 *  /		XXB		/		XXB		/   XXB         /
	 */


	/*
	 *	voe ddr partition layout
	 *
	 *	/			voe version	28B						/
	 *	/	voe year	/	voe month	/	voe day		/
	 *	/		2B		/		1B		/		1B		/
	 *	/	Version		/
	 *	/		24B		/
	 *  /	rsvd		/
	 *  /		4B		/
	 *	/			sensor version	28B						/
	 *	/	voe year	/	voe month	/	voe day		/
	 *	/		2B		/		1B		/		1B		/
	 *	/	Version		/
	 *	/		24B		/
	 *  /	rsvd		/
	 */

	uint32_t iram_fw_addr = VOE_IRAM_S;
	uint32_t dram_fw_addr = VOE_DRAM_S;


	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	u32 *addr = (u32 *)fw_addr;
	voe_header_t _voe_header;
	voe_header_t *voe_header = &_voe_header;
	u32 header_size = sizeof(voe_header_t);

	if (voe_cpy == NULL) {
		printf("voe firmware copy API is NULL \n");
		return NOK;
	}
	printf("<<test>><%s><%d> VOE DDR == %x\n",__func__,__LINE__,voe_ddr_addr);

	voe_cpy(voe_header, fw_addr, sizeof(voe_header_t));

#if IS_CUT_TEST(CONFIG_CHIP_VER)    // Test-chip

	hal_sys_peripheral_en(VOE_SYS, DISABLE);
#else
	if ( (HAL_READ32(SYSON_S_BASE, 0x118) & 0x3000) == 0x3000 ) {
		hal_voe_fcs_set_voe_fm_load_flag_final();
		HAL_WRITE32(TM_STATUS, 0x0, 0x0); // lock KM at initial while(1)
	}
	hal_sys_peripheral_en(VOE_SYS, DISABLE);
#endif


	printf("voe:date %d/%d/%d version:%s\r\n", voe_header->year, voe_header->month, voe_header->day, voe_header->version);
	printf("voe:FW size itcm(%d) dtcm(%d) ddr(%d)\r\n", voe_header->itcm_size, voe_header->dtcm_size, voe_header->ddr_size);

	voe_addr = (int)voe_ddr_addr;
	memcpy((void *)voe_ddr_addr,(void *)(voe_header->version),24);

	hal_sys_peripheral_en(VOE_SYS, ENABLE);

	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS
								   | VOE_BIT_VOE_RESET_DIS
								   | VOE_BIT_KM_RESET_DIS;


	//printf ("VOE status 0x%08x \r\n",voe->VOE_REG_VOE_SYSTEM_CTRL);

	//memcpy((void *)(iram_fw_addr+REMAP_S7_ITCM)
	voe_cpy((void *)(iram_fw_addr + REMAP_S7_ITCM)
			, (addr + (header_size >> 2))
			, (size_t)voe_header->itcm_size);

	dcache_clean_invalidate_by_addr((uint32_t *)(iram_fw_addr + REMAP_S7_ITCM)
									, (int32_t)voe_header->itcm_size);

	voe_cpy((void *)(dram_fw_addr + REMAP_S7_DTCM)
			, (addr + ((header_size + voe_header->itcm_size) >> 2))
			, (size_t)voe_header->dtcm_size);

	dcache_clean_invalidate_by_addr((uint32_t *)(dram_fw_addr + REMAP_S7_DTCM)
									, (int32_t)voe_header->dtcm_size);

	voe_cpy((void *)(voe_ddr_addr + ((SENSOR_FW_SIZE)>>2))
			, (addr + (( header_size + voe_header->itcm_size + voe_header->dtcm_size) >> 2))
			, (size_t)voe_header->ddr_size);

	dcache_clean_invalidate_by_addr((uint32_t *)(voe_ddr_addr + ((SENSOR_FW_SIZE+VOE_DDR_HEADER)>>2))
									, (int32_t)voe_header->ddr_size);


	return OK;
}

/**
 *  @brief To initialize the VOE.
 *  This function be enable VOE .
 *
 *
 *  @param[in]  voe_cb			:	VOE KM2TM interrupt callback function
 *  @param[in]  erac_done		:	VOE ERAC interrupt callback function
 *  @param[in]  heap_addr		:	VOE heap address
 *  @param[in]  heap_size		:	VOE heap size
 *
 *  @returns N/A.
 *
 */

#define ENABLE_KM_JTAG_CHAIN 0

int hal_voe_init(void *voe_cb, void *erac_done, u32 heap_addr, u32 heap_size)
{
	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;
	uint32_t iram_fw_addr = VOE_IRAM_S;

	/*
	 *
	 *	/			VOE version header on DDR 64B						/
	 *	/			VOE FW binary version 32B
	 *	/			Sensor binary version 32B
	 *
	 */


	printf("voe   :%s \n", (int *)(voe_addr));
	printf("sensor:%s \n", (int *)(voe_addr+32));
	printf("hal   :%s \n", hal_voe_version);

	voe_semaphore = xSemaphoreCreateMutex();
	if(voe_semaphore == NULL) {
		printf("VOE semaphore open fail \n");
		return NOK;
	}

	if( memcmp((void *)(voe_addr),(void *)(voe_addr+32),20) != 0) {
		printf("VOE/Sensor version check fail \n");
		return NOK;
	}

	if( memcmp((void *)(voe_addr),hal_voe_version,20) != 0) {
		printf("VOE/HAL version check fail \n");
		return NOK;
	}

	hal_voe_irq_reg((uint32_t)VOE_IRQHandler);


	hal_voe_set_km_message_int(ENABLE);

	hal_voe_set_erac_int(ENABLE);

	memset((void *)&g_pvoe_adpt, 0, sizeof(hal_voe_adapter_t));

	g_pvoe_adpt.km_message_cb = (voe_callback_t)voe_cb;
	g_pvoe_adpt.km_message_cb_para = NULL;
	g_pvoe_adpt.erac_finish_cb = (voe_callback_t)erac_done;
	g_pvoe_adpt.erac_finish_cb_para = NULL;


	voe->VOE_REG_KM_FW_BASE_ADDRESS = heap_addr;
	voe->VOE_REG_KM_FW_LEN = heap_size;

	//VOE_VECTOR_SET
	voe->VOE_REG_KM_INITVTOR_S = iram_fw_addr;
	voe->VOE_REG_KM_INITVTOR_NS = iram_fw_addr;

	//KM COLD_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS;

	//KM_RESET
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS
								   | VOE_BIT_VOE_RESET_DIS;

	//KM RUN
	voe->VOE_REG_VOE_SYSTEM_CTRL = VOE_BIT_CLOCK_GATING_DIS
								   | VOE_BIT_VOE_RESET_DIS
								   | VOE_BIT_KM_RESET_DIS;


#if ENABLE_KM_JTAG_CHAIN
	HAL_WRITE32(0x40000000, 0x70, 0x1F);
#endif

	while (g_pvoe_adpt.voe2tm_ack == 0);			// busy waiting ACK
	g_pvoe_adpt.voe2tm_ack = 0;

	return OK;

}

/**
 *  @brief To de-initialize the VOE . Disable VOE related SYS power and setting.
 *
 *
 *  @param void.
 *
 *  @returns void
 */
void hal_voe_deinit(void)
{
	volatile uint32_t val;
	VOE_TypeDef *voe = (VOE_TypeDef *)VOE_BASE;

	hal_voe_set_km_message_int(DISABLE);
	hal_voe_set_erac_int(DISABLE);


	val = voe->VOE_REG_VOE_SYSTEM_CTRL;
	val &= ~(VOE_BIT_KM_RESET_DIS | VOE_BIT_VOE_RESET_DIS);
	voe->VOE_REG_VOE_SYSTEM_CTRL = val;

	val &= ~VOE_BIT_CLOCK_GATING_DIS;
	voe->VOE_REG_VOE_SYSTEM_CTRL = val;
	hal_sys_peripheral_en(VOE_SYS, DISABLE);

	if(voe_semaphore != NULL) {
		vSemaphoreDelete(voe_semaphore);
		voe_semaphore = NULL;
	}

}




uint32_t check_km_cmd(void)
{
	return PARSE_TM_CMD(g_pvoe_adpt.voe_send_message);
}

uint32_t check_km_ch(void)
{
	return PARSE_TM_CMD_CH(g_pvoe_adpt.voe_send_message);
}

uint32_t check_km_status(void)
{
	return PARSE_TM_CMD_STATUS(g_pvoe_adpt.voe_send_message);
}


int hal_voe_fcs_check_km_run_done(void)
{

	uint32_t reg_value;

	reg_value = HAL_READ32(KM_STATUS, 0);

	if (reg_value == FCS_RUN_DATA_OK_KM || reg_value == FCS_RUN_DATA_NG_KM) {
		return TRUE;
	} else {
		return FALSE;
	}
}

int hal_voe_fcs_check_km_run_error(void)
{

	uint32_t status;

	status = HAL_READ32(KM_STATUS, 0);

	if (status == FCS_RUN_DATA_NG_KM) {
		return TRUE;
	} else {
		return FALSE;
	}

}

int hal_voe_fcs_check_km_run_OK(void)
{

	uint32_t reg_value;

	reg_value = HAL_READ32(KM_STATUS, 0);

	if (reg_value == FCS_RUN_DATA_OK_KM) {
		return TRUE;
	} else {
		return FALSE;
	}
}




static uint32_t volatile g_fcs_status = 0;

#if 0
void hal_voe_fcs_set_voe_fm_load_flag(void) {


	if (hal_voe_fcs_check_km_run_error()) {
		HAL_WRITE32(TM_STATUS, 0x0, FULL_LOAD_OK_FCS_NG_TM);
		g_fcs_status = FULL_LOAD_OK_FCS_NG_TM;
	} else {
		HAL_WRITE32(TM_STATUS, 0x0, FULL_LOAD_OK_FCS_OK_TM);
		g_fcs_status = FULL_LOAD_OK_FCS_OK_TM;
	}


}
#endif
void hal_voe_fcs_set_voe_fm_load_flag_final(void)
{


	if (hal_voe_fcs_check_km_run_OK()) {
		HAL_WRITE32(TM_STATUS, 0x0, FULL_LOAD_OK_FCS_OK_TM);
		g_fcs_status = FULL_LOAD_OK_FCS_OK_TM;
	    printf("hal_voe_fcs_set_voe_fm_load_flag_final OK %d\r\n", g_fcs_status);
	}
	else {  // NG and Timeout case
		HAL_WRITE32(TM_STATUS, 0x0, FULL_LOAD_OK_FCS_NG_TM);
		g_fcs_status = FULL_LOAD_OK_FCS_NG_TM;
	    printf("hal_voe_fcs_set_voe_fm_load_flag_final NG %d\r\n", g_fcs_status);
	}
	dcache_clean_invalidate_by_addr((uint32_t *)(&g_fcs_status)
									, sizeof(g_fcs_status));
}


int hal_voe_fcs_check_OK(void)
{
#if IS_CUT_A(CONFIG_CHIP_VER)
	uint32_t status;

	status = HAL_READ32(TM_STATUS, 0);
	printf("hal_voe_fcs_check_OK 0x%x 0x%x \r\n", g_fcs_status, status);

	if (g_fcs_status == FULL_LOAD_OK_FCS_OK_TM || status == FULL_LOAD_OK_FCS_OK_TM ) {
		return 1;
	} else {
		return 0;
	}
#else

	return 0;

#endif
}











/** @} */ /* End of group hs_hal_voe */

