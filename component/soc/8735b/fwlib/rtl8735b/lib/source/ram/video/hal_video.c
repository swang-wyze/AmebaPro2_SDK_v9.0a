/**************************************************************************//**
 * @file     hal_video.c
 * @brief    This file implements the Video HAL functions.
 *
 * @version  V1.00
 * @date     2021-01-14
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
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
#include "hal_video.h"
//#include <math.h>

#if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9
#endif

/* For parameter parsing */
//#include "SEGGER_SEMIHOST.h"

#if CONFIG_VIDEO_EN

#define MAX_CMD_ARGC 64

/**
 * @addtogroup hal_video Video
 * @{
 */

hal_video_adapter_t *video_adapter;

#define fprintf(stdio, ...)       printf(__VA_ARGS__)

/* Test bench definition of comment header */




/**
  \brief  The data structure to handle the common resource and setting for all Enc adapters.
*/

//void ENC_IRQHandler(void); // on ewl_ameba.c

void hal_video_output_task(void const *argument);        			// thread function
osThreadDef(hal_video_output_task, osPriorityNormal, 1, 64 * 1024);		// thread object


void voe_cb(u32 param1)
{
	BaseType_t xHigherPriorityTaskWoken = pdTRUE; // pdTRUE/pdFALSE
	enc2out_t *enc2out;
	int cmd = check_km_cmd();


	if ((VOE_OUT_CMD == cmd) || (VOE_STOP_CMD == cmd)) {
		enc2out = (enc2out_t *)video_adapter->enc2out;
		dcache_invalidate_by_addr((uint32_t *)enc2out, sizeof(enc2out_t));
		enc2out->cmd = cmd;

		if (enc2out->queue != NULL) {
			if (xQueueSendFromISR(enc2out->queue, enc2out,  &xHigherPriorityTaskWoken) != pdPASS) {
				printf("<<test>><%s><%d> enc2out Queue Full \n", __func__, __LINE__);
			}
		}
	} else if (VOE_OPEN_CMD == cmd) {
		enc2out->cmd = cmd;
		printf("<<test>><%s><%d> VOE open done \n", __func__, __LINE__);
	}
#if	CONFIG_VERIFY_VOE
	else if (FW_BOOT_DONE_CMD == cmd) {
		enc2out = (enc2out_t *)video_adapter->enc2out;
		dcache_invalidate_by_addr((uint32_t *)enc2out, sizeof(enc2out_t));
		enc2out->cmd = cmd;
		if (enc2out->queue != NULL) {
			if (xQueueSendFromISR(enc2out->queue, enc2out,  &xHigherPriorityTaskWoken) != pdPASS) {
				printf("<<test>><%s><%d> enc2out Queue Full \n", __func__, __LINE__);
			}
		}
	}
#endif // CONFIG_VERIFY_VOE

}



void hal_video_output_task(void const *argument)
{
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)argument;

	commandLine_s *cml;
	enc2out_t e2o;
	enc2out_t *enc2out = v_adp->enc2out;
	e2o.finish = NON_FRAME;

	while (1) {
		// finish_frame == 5: first frame, 9: last frame, 2 normal frame

		if (xQueueReceive(enc2out->queue, &e2o,	portMAX_DELAY) != pdPASS) {
			printf("<<test>><%s><%d> receive queue fail \n", __func__, __LINE__);
		}

#if	CONFIG_VERIFY_VOE
		if (FW_BOOT_DONE_CMD == e2o.cmd) {
			continue;
		}
#endif

		if (e2o.finish == FINISH) {
			break;
		}

		cml = v_adp->cmd[e2o.ch];

		if (cml->voe == 1) {// VOE enable
			if (e2o.finish <= NORMAL_FRAME) {
				v_adp->out_cb(&e2o, v_adp, v_adp->ctx[e2o.ch]);
			}
			dcache_clean_invalidate_by_addr((uint32_t *)&e2o, sizeof(enc2out_t));
		} else {
			v_adp->out_cb(&e2o, v_adp, v_adp->ctx[e2o.ch]);
		}
	}

	printf("output task closed \n");
	osThreadTerminate(osThreadGetId());
}


hal_video_adapter_t *hal_video_init(void)
{
	hal_video_adapter_t *v_adp;

	int i;
#if (CONFIG_CHIP_VER > CHIP_TEST_CUT)
	printf("ISP:%d ENC:%d H265:%d NN:%d\n"
		   , hal_sys_get_video_info(VIDEO_INFO_ISP)
		   , hal_sys_get_video_info(VIDEO_INFO_ENC)
		   , hal_sys_get_video_info(VIDEO_INFO_H265)
		   , hal_sys_get_video_info(VIDEO_INFO_NN));
	if (hal_sys_get_video_info(VIDEO_INFO_ISP) != ENABLE) {
		printf("Chip not support ISP\n");
		return NULL;
	}
	if (hal_sys_get_video_info(VIDEO_INFO_ENC) != ENABLE) {
		printf("Chip not support ENC\n");
		return NULL;
	}
#endif

	if (video_adapter != NULL) {
		printf("video is running\n");
		return video_adapter;
	}
#if 1
#if CONFIG_FPGA // FPGA
	// Enable I2c
	volatile hal_i2c_adapter_t  i2c_master_sample;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	i2c_master_sample.pltf_dat.scl_pin		= PIN_D14;//PIN_F2;
	i2c_master_sample.pltf_dat.sda_pin	   = PIN_D12;//PIN_F3;
#else
	i2c_master_sample.pltf_dat.scl_pin		= PIN_D12;//PIN_F2;
	i2c_master_sample.pltf_dat.sda_pin	   = PIN_D10;//PIN_F3;
#endif
	i2c_master_sample.init_dat.index = 3;//1;
	hal_i2c_pin_register_simple(&i2c_master_sample);

	// Enable APHY I2c
	volatile hal_i2c_adapter_t	i2c_master_aphy_sample;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	i2c_master_aphy_sample.pltf_dat.scl_pin		= PIN_E7;
	i2c_master_aphy_sample.pltf_dat.sda_pin	   = PIN_E8;
#else
	i2c_master_aphy_sample.pltf_dat.scl_pin		= PIN_E3;
	i2c_master_aphy_sample.pltf_dat.sda_pin	   = PIN_E4;
#endif
	i2c_master_aphy_sample.init_dat.index = 2;
	hal_i2c_pin_register_simple(&i2c_master_aphy_sample);
#else

//#ifdef _NO_FCS_
	if (!hal_voe_fcs_check_OK()) {
		// Enable I2c
		volatile hal_i2c_adapter_t  i2c_master_sample;
		i2c_master_sample.pltf_dat.scl_pin		= PIN_D14;//PIN_F2;
		i2c_master_sample.pltf_dat.sda_pin	   = PIN_D12;//PIN_F3;
		i2c_master_sample.init_dat.index = 3;//1;
		hal_i2c_pin_register_simple(&i2c_master_sample);

		dbg_printf("i2c: %x pin init ready\n\r", i2c_master_sample.init_dat.index);
	}
//#endif

#endif
#endif

//#ifdef _NO_FCS_
	if (!hal_voe_fcs_check_OK()) {
		// Enable GPIO
		hal_sys_peripheral_en(GPIO_SYS, ENABLE);
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		hal_pinmux_register(PIN_D13, PID_GPIO);//reset pin
		hal_pinmux_register(PIN_D11, PID_GPIO);//power down pin
#else
		hal_pinmux_register(PIN_E0, PID_GPIO);//reset pin
		hal_pinmux_register(PIN_D11, PID_GPIO);//power down pin
#endif
		// Enable sensor clock(hclk)
#if IS_CUT_TEST(CONFIG_CHIP_VER)
		hal_pinmux_register(PIN_D10, PID_SENSOR);//reset pin
#else
		hal_pinmux_register(PIN_D13, PID_SENSOR);//reset pin
#endif
	}
	// Enable ISP HW
	hal_sys_peripheral_en(ISP_SYS, ENABLE);
	hal_sys_set_clk(ISP_SYS, 0);
	hal_sys_set_clk(MIPI_SYS, 0);
	//hal_irq_clear_pending (ISP_IRQn);
	//hal_irq_set_priority(ISP_IRQn, ISP_IRQPri);

	// Enable Encoder HW
	hal_sys_peripheral_en(ENC_SYS, ENABLE);
	hal_irq_clear_pending(ENC_IRQn);
	hal_irq_set_priority(ENC_IRQn, ENC_IRQPri);
//	hal_irq_set_vector(ENC_IRQn, (uint32_t)ENC_IRQHandler);
//	hal_irq_enable (ENC_IRQn);

	// malloc hal_video_adapter
	v_adp = (hal_video_adapter_t *)malloc(sizeof(hal_video_adapter_t));
	if ((void *)v_adp == NULL) {
		printf("v_adp malloc fail\n");
		return NULL;
	}
	memset(v_adp, 0, sizeof(hal_video_adapter_t));

	v_adp->enc2out = malloc(sizeof(enc2out_t));
	if (v_adp->enc2out == NULL) {
		return NULL;
	}
	memset(v_adp->enc2out, 0, sizeof(enc2out_t));

#if 0
	enc2out_t *enc2out =  v_adp->enc2out;

	// Need modify enc2out queue depth
	if (enc2out->queue == NULL) {
		enc2out->queue = xQueueCreate(60, sizeof(enc2out_t));
		if (enc2out->queue == NULL) {
			printf("open enc2out queue fail\n");
			return NULL;
		}
	}
#endif


	for (i = 0; i < MAX_CHANNEL; i++) {
		// malloc command line struct
		v_adp->cmd[i] = malloc(sizeof(commandLine_s));
		if (v_adp->cmd[i] == NULL) {
			return NULL;
		}
		// set default command line parameter
		default_parameter(v_adp->cmd[i]);
	}

	// malloc hal_isp_adapter
	v_adp->isp_adapter = malloc(sizeof(hal_isp_adapter_t));
	if (v_adp->isp_adapter == NULL) {
		printf("isp_adapter alloc fail \n");
		return NULL;
	}

	memset(v_adp->isp_adapter, 0, sizeof(hal_isp_adapter_t));
	v_adp->isp_adapter->v_adapter = (void *)v_adp;
	if (hal_voe_fcs_check_OK()) {
		v_adp->isp_adapter->fcs_ready = 1;
		printf("isp_adapter fcs_ready \n");
	}
	video_adapter = (void *)v_adp;



	return v_adp;

}



void hal_video_deinit(hal_video_adapter_t *v_adp)
{
	hal_isp_adapter_t *isp_adp = v_adp->isp_adapter;

	int i;


	// ISP HW disable
	hal_irq_disable(ISP_IRQn);
	hal_irq_clear_pending(ISP_IRQn);
	hal_sys_peripheral_en(ISP_SYS, DISABLE);

#if CONFIG_FPGA // FPGA

	// Disable I2c
	volatile hal_i2c_adapter_t  i2c_master_sample;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	i2c_master_sample.pltf_dat.scl_pin		= PIN_D14;//PIN_F2;
	i2c_master_sample.pltf_dat.sda_pin	   = PIN_D12;//PIN_F3;
#else
	i2c_master_sample.pltf_dat.scl_pin		= PIN_D12;//PIN_F2;
	i2c_master_sample.pltf_dat.sda_pin	   = PIN_D10;//PIN_F3;
#endif
	i2c_master_sample.init_dat.index = 3;//1;
	hal_i2c_pin_unregister_simple(&i2c_master_sample);

	// Disable APHY I2c
	volatile hal_i2c_adapter_t	i2c_master_aphy_sample;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	i2c_master_aphy_sample.pltf_dat.scl_pin		= PIN_E7;
	i2c_master_aphy_sample.pltf_dat.sda_pin	   = PIN_E8;
#else
	i2c_master_aphy_sample.pltf_dat.scl_pin		= PIN_E3;
	i2c_master_aphy_sample.pltf_dat.sda_pin	   = PIN_E4;
#endif
	i2c_master_aphy_sample.init_dat.index = 2;
	hal_i2c_pin_unregister_simple(&i2c_master_aphy_sample);
#else
	// Disable I2c
	volatile hal_i2c_adapter_t	i2c_master_sample;
	i2c_master_sample.pltf_dat.scl_pin		= PIN_D14;//PIN_F2;
	i2c_master_sample.pltf_dat.sda_pin	   = PIN_D12;//PIN_F3;
	i2c_master_sample.init_dat.index = 3;//1;
	hal_i2c_pin_unregister_simple(&i2c_master_sample);
#endif

	//Disable GPIO
	hal_sys_peripheral_en(GPIO_SYS, DISABLE);
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	hal_pinmux_unregister(PIN_D13, PID_GPIO);//reset pin
	hal_pinmux_unregister(PIN_D11, PID_GPIO);//power down pin
#else
	hal_pinmux_unregister(PIN_E0, PID_GPIO);//reset pin
	hal_pinmux_unregister(PIN_D11, PID_GPIO);//power down pin
#endif
	// Enable sensor clock(hclk)
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	hal_pinmux_unregister(PIN_D10, PID_SENSOR);//reset pin
#else
	hal_pinmux_unregister(PIN_D13, PID_SENSOR);//reset pin
#endif


	if (isp_adp != NULL) {
		if (isp_adp->addr3dnr != NULL) {
			free(isp_adp->addr3dnr);
		}
		free(isp_adp);
	}

	// ENC HW disable
	hal_irq_disable(ENC_IRQn);
	hal_irq_clear_pending(ENC_IRQn);
	hal_sys_peripheral_en(ENC_SYS, DISABLE);




	for (i = 0; i < MAX_CHANNEL; i++) {
		if (v_adp->cmd[i] != NULL) {
			free(v_adp->cmd[i]);
		}
	}

	if (v_adp->enc2out != NULL) {
#if 1
		enc2out_t *enc2out = v_adp->enc2out;;
		if (enc2out->queue != NULL) {
			vQueueDelete(enc2out->queue);
			enc2out->queue = NULL;
		}
#endif
		free(v_adp->enc2out);
		v_adp->enc2out = NULL;
	}

#if 0 // next stage implement scale down
	if (v_adp->pictureStabMem.virtualAddress != NULL) {
//	    EWLFreeLinear(ewl_inst, &v_adp->pictureStabMem);
	}
#endif

	if (v_adp != NULL) {
		free((void *)v_adp);
		v_adp = NULL;
	}
	video_adapter = NULL;

}

int hal_video_buf(hal_video_adapter_t *v_adp, int ch)
{
	i32 ret = OK;

	ret = hal_voe_send2voe(VOE_BUF_CMD, (u32)NULL, (u32)ch);
	if (ret < 0) {
		printf("VOE_BUF_CMD command fail %x\n", ret);
		return NOK;
	}
	return OK;
}

int hal_video_mem(hal_video_adapter_t *v_adp, int ch)
{
	i32 ret = OK;
	ret = hal_voe_send2voe(VOE_MEM_CMD, (u32)NULL, (u32)ch);
	if (ret < 0) {
		printf("VOE_MEM_CMD command fail %x\n", ret);
		return NOK;
	}
	return OK;
}

int hal_video_print(hal_video_adapter_t *v_adp, int mode)
{
	i32 ret = OK;
	ret = hal_voe_send2voe(VOE_PRINT_CMD, (u32)mode, (u32)NULL);
	if (ret < 0) {
		printf("VOE_PRINT_CMD command fail %x\n", ret);
		return NOK;
	}
	return OK;

}
int hal_video_release(int ch, int len)
{
	i32 ret = OK;
	commandLine_s *cml = video_adapter->cmd[ch];
	if (cml->CodecEnable & (CODEC_HEVC | CODEC_H264 | CODEC_RGB)) {

		ret = hal_voe_send2voe(VOE_RELEASE_SLOT_CMD, ch, len);
		if (ret < 0) {
			printf("VOE_RELEASE_SLOT_CMD command fail %x\n", ret);
			return NOK;
		}
	}
	return OK;
}
int hal_video_start(hal_video_adapter_t *v_adp, int ch)
{
	i32 ret = OK;
	commandLine_s *cml = v_adp->cmd[ch];
	//DBG_WARN_MSG_ON(_DBG_ISP_);
	//DBG_ERR_MSG_ON(_DBG_ISP_);
	//DBG_INFO_MSG_ON(_DBG_ISP_);
	if (cml->voe == 1) {// VOE enable
		ret = hal_voe_send2voe(VOE_START_CMD, (u32)v_adp->enc2out, (u32)ch);
		if (ret < 0) {
			printf("VOE_START_CMD command fail %x\n", ret);
			hal_video_close(v_adp, ch);
			return NOK;
		}
		return OK;
	}
	return OK;

}

int hal_video_stop(hal_video_adapter_t *v_adp, int ch)
{
	i32 ret = OK;

	commandLine_s  *cml = v_adp->cmd[ch];

	/* End stream */
	if (cml->voe == 1) {// VOE enable
		dcache_clean_invalidate();
		ret = hal_voe_send2voe(VOE_STOP_CMD, (u32)NULL, (u32)ch);
		if (ret < 0) {
			printf("VOE_STOP_CMD command fail %x\n", ret);
			return NOK;
		}
	}

	hal_delay_us(300000); //delay 3ms workaround for (cmd: vs 0 10 bus fault issue)

	return OK;
}

/*------------------------------------------------------------------------------

    hal_video_open
        Create and configure an encoder instance.

    Params:
        v_adp     - video adapter struct
    Return:
        0   - for success
        -1  - error

------------------------------------------------------------------------------*/
int hal_video_open(hal_video_adapter_t *v_adp, int ch)
{
	VCEncRet ret;

	commandLine_s *cml = v_adp->cmd[ch];

	v_adp->crc32 = 0;
#if (CONFIG_CHIP_VER > CHIP_TEST_CUT)
	// Check Chip support HEVC
	if ((hal_sys_get_video_info(VIDEO_INFO_H265) != ENABLE)
		&& ((cml->CodecEnable & CODEC_HEVC) != 0)) {
		printf("Chip not support HEVC\n");
		return NOK;
	}
#endif
	// Check IQ binary
	if (v_adp->isp_adapter->iq_addr == NULL) {
		printf("No IQ binary\n");
		return NOK;
	}


	// Create internal video task
	if (v_adp->tid_output == NULL) {
		v_adp->tid_output = osThreadCreate(osThread(hal_video_output_task), v_adp);
		if (v_adp->tid_output == NULL) {
			dbg_printf("Create output task error\r\n");
		}
	}


	if ((cml->voe == 1)) { // VOE enable
		v_adp->open_done = 0;

		dcache_clean_invalidate();
		ret = hal_voe_send2voe((u32)VOE_OPEN_CMD, (u32)v_adp, (u32)ch);
		if (ret < 0) {
			printf("VOE_OPEN_CMD command fail\n");
			//hal_video_close(v_adp, ch);
			return NOK;
		}
		while (v_adp->open_done == 1) {
			hal_delay_us(1000000); //delay 1ms
			printf("wating open done\n");
		}
		dcache_invalidate_by_addr((uint32_t *)v_adp, sizeof(hal_video_adapter_t));

	}

	return OK;
}

/*------------------------------------------------------------------------------
    hal_video_close
       Release an encoder insatnce.

   Params:
        encoder - the instance to be released

   Return:
        0   - for success
        -1  - error

------------------------------------------------------------------------------*/
int hal_video_close(hal_video_adapter_t *v_adp, int ch)
{
	commandLine_s *cml = v_adp->cmd[ch];
	enc2out_t *enc2out = v_adp->enc2out;
	int i = 0;
	i32 ret = OK;

	if (cml->voe == 1) {// VOE enable
		ret = hal_voe_send2voe(VOE_CLOSE_CMD, (u32)NULL, (u32)ch);
		if (ret < 0) {
			printf("VOE_CLOSE_CMD command fail %x\n", ret);
			return NOK;
		}
		//hal_voe_deinit();
		//if(v_adp->voe_heap_addr != NULL)
		//free(v_adp->voe_heap_addr);
	}

	enc2out->finish = FINISH;
	if (xQueueSend(enc2out->queue, enc2out,  portMAX_DELAY) != pdPASS) {
		printf("<<test>><%s><%d> enc2out Queue Full \n", __func__, __LINE__);
	}

	while (eTaskGetState(v_adp->tid_output) != eDeleted) {
		hal_delay_us(1000); //delay 1ms
		//printf("<<test>><%s><%d> %d\n",__func__, __LINE__, eTaskGetState(v_adp->tid_output));
		i++;
		if (i > 1000) {
			printf("== output_task force close ==\n");
			osThreadTerminate(v_adp->tid_output);
		}
	}
	v_adp->tid_output = NULL;
	return OK;
}

int hal_video_set_voe(hal_video_adapter_t *v_adp, int heap_size, int queue)
{
	int ret = OK;
	if (v_adp->voe_heap_addr == NULL) {
		v_adp->voe_heap_size = heap_size;
		dcache_clean_invalidate();

		v_adp->voe_heap_addr = malloc(v_adp->voe_heap_size);
		if (v_adp->voe_heap_addr == NULL) {
			printf("VOE heap malloc fail \n");
			return NOK;
		}

		enc2out_t *enc2out =  v_adp->enc2out;
		// Need modify enc2out queue depth
		if (enc2out->queue == NULL) {
			enc2out->queue = xQueueCreate(queue, sizeof(enc2out_t));
			if (enc2out->queue == NULL) {
				printf("open enc2out queue fail\n");
				return NOK;
			}
		}

		dcache_clean_invalidate_by_addr((uint32_t *)enc2out, sizeof(enc2out_t) +32);

		ret = hal_voe_init(voe_cb, NULL, (u32)v_adp->voe_heap_addr, (u32)v_adp->voe_heap_size);
		if (ret == NOK) {
			return ret;
		}

	} else {
		printf("VOE already running \n");
	}

	return ret;

}

int hal_video_froce_i(int ch)
{
	hal_voe_send2voe(VOE_FORCE_I_CMD, ch, 0x0);
	return OK;
}

int hal_video_jpg_out(int ch, int mode)
{
	hal_voe_send2voe(VOE_JPG_OUT_CMD, ch, mode);
	return OK;
}

int hal_video_yuv_out(int ch, int mode)
{
	hal_voe_send2voe(VOE_YUV_OUT_CMD, ch, mode);
	return 0;
}

int hal_video_roi_mode(int ch)
{
	return OK;
}

int hal_video_roi_region(int ch, int x, int y, int width, int height, int value)
{
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)video_adapter;
	hal_video_roi_s *roi = v_adp->roi[ch];
	commandLine_s *cml = v_adp->cmd[ch];

	char *roi_table = (char *)roi->roi_table;
	int i;
	if (v_adp == NULL) {
		printf("video_adapter not ready\n");
		return NOK;
	}
	if (cml->roiMapDeltaQpEnable != 1) {
		printf("No enable ROI map\n");
		return NOK;
	}

	for (i = y ; i < y + height ; i++) {
		memset(roi_table + x + (i * roi->table_width), value, width);
	}
	dcache_clean_invalidate_by_addr((uint32_t *)roi_table, roi->table_size);

#if 0 // Show ROI Table
	int line_idx = 0;


	printf("-----\n");
	for (line_idx = 0 ; line_idx < roi->table_height ; line_idx++) {
		for (i = 0 ; i < roi->table_width ; i++) {
			printf(" %2d", *(roi_table + i + (line_idx * roi->table_width)));
		}
		printf("\n");
	}
	printf("-----\n");

#endif

	return OK;
}


int hal_video_osd_region(int ch, int x, int y, int width, int height, int value)
{
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)video_adapter;
	hal_video_roi_s *roi = v_adp->roi[ch];
	commandLine_s *cml = v_adp->cmd[ch];

	char *roi_table = (char *)roi->roi_table;
	int i;
	if (v_adp == NULL) {
		printf("video_adapter not ready\n");
		return NOK;
	}
	if (cml->roiMapDeltaQpEnable != 1) {
		printf("No enable ROI map\n");
		return NOK;
	}

	for (i = y ; i < y + height ; i++) {
		memset(roi_table + x + (i * roi->table_width), value, width);
	}
	dcache_clean_invalidate_by_addr((uint32_t *)roi_table, roi->table_size);

	return OK;
}
int hal_video_set_rc(rate_ctrl_s *rc, int ch)
{
	dcache_clean_invalidate_by_addr((uint32_t *)rc, sizeof(rate_ctrl_s) + 32); //(addr aligned to 32-byte boundary)
	//dcache_clean_invalidate();
	hal_voe_send2voe(VOE_SET_RC_CMD, (u32)rc, ch);
	return 0;
}

int hal_video_obj_region(obj_ctrl_s *obj_r, int ch)
{
	dcache_clean_invalidate_by_addr((uint32_t *)obj_r, sizeof(obj_ctrl_s) + 32); //(addr aligned to 32-byte boundary)
	//dcache_clean_invalidate();
	hal_voe_send2voe(VOE_OBJECT_REGION_CMD, (u32)obj_r, ch);
	return OK;
}


int hal_video_str_parsing(char *str, hal_video_adapter_t *v_adp)
{
	int cmd_arg[MAX_CMD_ARGC * 4];
	int *file_argv = NULL;
	int file_argc = 0;
	int ch = 0;
	commandLine_s *cml;
	file_argv = cmd_arg;
	if (file_argv == NULL) {
		printf("file_argv malloc fail on %s\n", __func__);
		return NOK;

	}
	memset((void *)file_argv, 0x0, MAX_CMD_ARGC * 4);

	strtok((char *)str, " ");
	file_argc = 1;
	//parser command parameter

	do {
		char *value = strtok(NULL, " ");
		*(file_argv + file_argc) = (int)value;
		file_argc++;
		if (file_argc >= MAX_CMD_ARGC) {
			printf("Err: input command over range [%d] > MAX_ARGC[%d]\n", file_argc, MAX_CMD_ARGC);
			return NOK;
		}

		if ((value == 0x0) || (*value == 0x0) || (*(value + strlen(value) + 1) == '\t')) {
			break;
		}
	} while (1);

	if (strncmp(str, "rgb", 3) == 0) {
		ch = atoi((char *) * (file_argv + 1));
		if (ch != 4) {
			printf("RGB output only on ch4 \n");
			return NOK;
		}
	} else {
		ch = atoi((char *) * (file_argv + 1));
		if (ch >= 3) {
			printf("select ch%d over range\n", ch);
			return NOK;
		}
	}
	cml = v_adp->cmd[ch];
	//default_parameter(cml);


	cml->ch = ch;
	if ((strncmp(str, "hevc", 4) == 0) || (strncmp(str, "h264", 4) == 0)) { // hevc/h264
		if (strncmp(str, "hevc", 4) == 0) {
			cml->CodecEnable |= CODEC_HEVC;
			cml->outputFormat = VCENC_VIDEO_CODEC_HEVC;
		} else {
			cml->CodecEnable |= CODEC_H264;
			cml->outputFormat = VCENC_VIDEO_CODEC_H264;
		}
		if (parameter_enc_get(file_argc, (char **)file_argv, cml)) {
			printf("Input parameter error\n");
			return NOK;
		}
	} else if (strncmp(str, "jpeg", 4) == 0) {
		/* Parse command line parameters */
		cml->CodecEnable |= CODEC_JPEG;
		if (parameter_jpg_get(file_argc, (char **)file_argv, cml) != 0) {
			printf("Input parameter error\n");
			return NOK;
		}
	} else if (strncmp(str, "nv12", 4) == 0) {
		cml->CodecEnable |= CODEC_NV12;
		cml->outputFormat = VCENC_VIDEO_CODEC_NV12;
		if (parameter_enc_get(file_argc, (char **)file_argv, cml)) {
			printf("Input parameter error\n");
			return NOK;
		}
	} else if (strncmp(str, "rgb", 3) == 0) {
		cml->CodecEnable |= CODEC_RGB;
		//cml->outputFormat = VCENC_VIDEO_CODEC_RGB;

		if (parameter_enc_get(file_argc, (char **)file_argv, cml)) {
			printf("Input parameter error\n");
			return NOK;
		}
	} else if (strncmp(str, "nv16", 4) == 0) {
		cml->CodecEnable |= CODEC_NV16;
		//cml->outputFormat = VCENC_VIDEO_CODEC_NV16;

		if (parameter_enc_get(file_argc, (char **)file_argv, cml)) {
			printf("Input parameter error\n");
			return NOK;
		}
	} else {
		printf("NON support is format %s \n", str);
		return NOK;
	}
	return ch;
}
int hal_video_cmd_reset(hal_video_adapter_t *v_adp, int ch)
{
	default_parameter(v_adp->cmd[ch]);
	return OK;
}

int hal_video_str2cmd(char *str1, char *str2, char *str3, hal_video_adapter_t *v_adp)
{
	int ch = -1, str1_ch, str2_ch, str3_ch;

	if (strlen(str1) != 0) {
		str1_ch = hal_video_str_parsing(str1, v_adp);
		if (str1_ch == NOK) {
			printf("str1 paring fail\n");
			return NOK;
		}
		ch = str1_ch;
	}

	if (strlen(str2) != 0) {
		str2_ch = hal_video_str_parsing(str2, v_adp);
		if (str2_ch == NOK) {
			printf("str2 paring fail\n");
			return NOK;
		}
		if ((ch != str2_ch) && (ch != -1)) {
			printf("str1 & str2 channel not match\n");
			return NOK;
		}
		ch = str2_ch;
	}

	if (strlen(str3) != 0) {
		str3_ch = hal_video_str_parsing(str3, v_adp);
		if (str3_ch == NOK) {
			printf("str3 paring fail\n");
			return NOK;
		}
		if ((ch != str3_ch) && (ch != -1)) {
			printf("str1 & str3 channel not match\n");
			return NOK;
		}
	}
	return OK;
}

int hal_video_cb_register(hal_video_adapter_t *v_adp
						  , output_callback_t output_cb
						  , u32 arg
						  , int ch)
{
	v_adp->out_cb = output_cb;
	v_adp->ctx[ch] = arg;
	return OK;
}

int hal_video_isp_ctrl(int ctrl_id, int set_flag, int value)
{
	int ret;
	if (set_flag == 0) {
		ret = hal_voe_send2voe(VOE_ISP_CTRL_GET_CMD, ctrl_id, 0x0);
	} else {
		ret = hal_voe_send2voe(VOE_ISP_CTRL_SET_CMD, ctrl_id, value);
	}
	return ret;
}

int hal_video_isp_tuning(int tuning_req, struct isp_tuning_cmd *tuning_cmd)
{
	int ret;

	dcache_clean_invalidate_by_addr((uint32_t *)tuning_cmd, sizeof(struct isp_tuning_cmd) + 32); //(addr aligned to 32-byte boundary)
	//dcache_clean_invalidate();
	ret = hal_voe_send2voe(tuning_req, (u32)tuning_cmd, 0x0);

	return ret;
}

/** @} */ /* End of group hal_enc */

#endif	// CONFIG_VIDEO_EN
