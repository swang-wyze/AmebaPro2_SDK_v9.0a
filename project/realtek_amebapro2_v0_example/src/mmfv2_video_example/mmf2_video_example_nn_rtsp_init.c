/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "module_rtsp2.h"

#include "module_rtknn.h"
#include "hal_video.h"
#include "hal_isp.h"
//#include "avcodec.h"

//#include "input_image_640x360x3.h"
// TODO: move model id to proper header


/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define RTSP_CHANNEL 1
#define RTSP_RESOLUTION VIDEO_HD
#define RTSP_FPS 30
#define RTSP_GOP 30
#define RTSP_BPS 1*1024*1024
#define VIDEO_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#include "sample_h265.h"
#define RTSP_TYPE VIDEO_HEVC
#define RTSP_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define RTSP_TYPE VIDEO_H264
#define RTSP_CODEC AV_CODEC_ID_H264
#endif

#if RTSP_RESOLUTION == VIDEO_VGA
#define RTSP_WIDTH	640
#define RTSP_HEIGHT	480
#elif RTSP_RESOLUTION == VIDEO_HD
#define RTSP_WIDTH	1280
#define RTSP_HEIGHT	720
#elif RTSP_RESOLUTION == VIDEO_FHD
#define RTSP_WIDTH	1920
#define RTSP_HEIGHT	1080
#endif

static video_params_t video_v1_params = {
	.stream_id 		= RTSP_CHANNEL,
	.type 			= RTSP_TYPE,
	.resolution 	= RTSP_RESOLUTION,
	.width 			= RTSP_WIDTH,
	.height 		= RTSP_HEIGHT,
	.bps            = RTSP_BPS,
	.fps 			= RTSP_FPS,
	.gop 			= RTSP_GOP,
	.rc_mode        = VIDEO_RCMODE,
	.use_static_addr = 1
};


static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = RTSP_CODEC,
			.fps      = RTSP_FPS,
			.bps      = RTSP_BPS
		}
	}
};


#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_WVGA
#define NN_FPS 5
#define NN_GOP 5
#define NN_BPS 256*1024
#define NN_EXCUTE_FPS 5

#define NN_TYPE VIDEO_RGB

#include "nn_model_init.h"
#include "mobilenet_ssd_uint8.h"
//#include "face300x300_uint8.h"

#if NN_RESOLUTION == VIDEO_VGA
#define NN_WIDTH	640
#define NN_HEIGHT	480
#elif NN_RESOLUTION == VIDEO_WVGA
#define NN_WIDTH	640
#define NN_HEIGHT	360
#endif

static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.resolution	 	= NN_RESOLUTION,
	.width 			= NN_WIDTH,
	.height 		= NN_HEIGHT,
	.fps 			= NN_FPS,
	.gop 			= NN_GOP,
	.direct_output 	= 0,
	.use_static_addr = 1
};

static rtknn_params_t rtknn_ssd_params = {
	.model_id = MOBILENETSSD_20OBJ,	// for built-in postbuild
	.model = mobilenet_ssd_uint8,
	.m_width = NNMODEL_SM_WIDTH,
	.m_height = NNMODEL_SM_HEIGHT,

	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = 640,
		.ymax = 360,
	}
};
/*
static rtknn_params_t rtknn_face_params = {
	.model_id = MODEL_FACE,			// for built-in postbuild
	.model = face_300x300_uint8,
	.m_width = NNMODEL_FACE_WIDTH,
	.m_height = NNMODEL_FACE_HEIGHT,

	.roi = {
		.xmin = 320,
		.ymin = 0,
		.xmax = 640,
		.ymax = 360,
	}
};
*/
#include "module_array.h"
static array_params_t h264_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_RGB888,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 5,
		}
	}
};

#include "input_image_640x360x3.h"


#define V1_ENA 1
#define V4_ENA 1
#define V4_SIM 0

static mm_context_t *array_ctx            = NULL;
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;

static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *rtknn_ctx            = NULL;
static mm_siso_t *siso_array_rtknn         = NULL;


typedef struct memscan_item_s {
	uint32_t start;
	uint32_t size;
} memscan_item_t;

typedef struct memscan_s {
	int cnt;

	memscan_item_t *items;
} memscan_t;

// video only scan NN memory area
memscan_item_t scan0_items[] = {
	{0x73000000, 16 * 1024 * 1024}
};

// NN only scan reset DRAM area
memscan_item_t scan1_items[] = {
	{0x70000000, 48 * 1024 * 1024}
};

memscan_t scan0 = {
	.cnt = sizeof(scan0_items) / sizeof(scan0_items[0]),
	.items = scan0_items
};

memscan_t scan1 = {
	.cnt = sizeof(scan1_items) / sizeof(scan1_items[0]),
	.items = scan1_items
};


void memory_scanner(void *ctx)
{
	memscan_t *scan = (memscan_t *)ctx;

	while (1) {
		vTaskDelay(5000);
		for (int i = 0; i < scan->cnt; i++) {
			uint32_t *zone = (uint32_t *)scan->items[i].start;
			uint32_t sum = 0;
			for (int x = 0; x < scan->items[i].size / 4; x++) {
				sum += zone[x];
				if (zone[x] != 0) {
					printf("zone %x size %x polluted at offset %x\n\r", scan->items[i].start, scan->items[i].size, x);
				}
			}
		}
	}
}

void run_memory_scanner(memscan_t *scan)
{
	// clear target memory
	for (int i = 0; i < scan->cnt; i++) {
		memset((uint8_t *)scan->items[i].start, 0, scan->items[i].size);
	}
	// create Task to polling
	xTaskCreate(memory_scanner, "scan", 2048, scan, 2, NULL);
}
#if V4_SIM==0
static TaskHandle_t rgbshot_thread = NULL;

static void rgbshot_control_thread(void *param)
{
	int shanpshot_time = 1000 / NN_EXCUTE_FPS;
	vTaskDelay(shanpshot_time);
	while (1) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 1);
		vTaskDelay(shanpshot_time);
	}
}
#endif

obj_ctrl_s sw_object;
int nn_set_object(void *p)
{
	int i = 0;
	rtknn_res_t *res = (rtknn_res_t *)p;

	if (res->obj_num > 0) {
		sw_object.objDetectNumber = 0;
		for (i = 0; i < res->obj_num; i++) {
			if (sw_object.objDetectNumber == 10) {
				break;
			}

			sw_object.objDetectNumber++;

			sw_object.objTopY[i] = (int)(res->result[6 * i + 3] * RTSP_HEIGHT);
			if (sw_object.objTopY[i] < 0) {
				sw_object.objTopY[i] = 0;
			} else if (sw_object.objTopY[i] >= RTSP_HEIGHT) {
				sw_object.objTopY[i] = RTSP_HEIGHT - 1;
			}

			sw_object.objTopX[i] = (int)(res->result[6 * i + 2] * RTSP_WIDTH);
			if (sw_object.objTopX[i] < 0) {
				sw_object.objTopX[i] = 0;
			} else if (sw_object.objTopX[i] >= RTSP_WIDTH) {
				sw_object.objTopX[i] = RTSP_WIDTH - 1;
			}

			sw_object.objBottomY[i] = (int)(res->result[6 * i + 5] * RTSP_HEIGHT);
			if (sw_object.objBottomY[i] < 0) {
				sw_object.objBottomY[i] = 0;
			} else if (sw_object.objBottomY[i] >= RTSP_HEIGHT) {
				sw_object.objBottomY[i] = RTSP_HEIGHT - 1;
			}

			sw_object.objBottomX[i] = (int)(res->result[6 * i + 4] * RTSP_WIDTH);
			if (sw_object.objBottomX[i] < 0) {
				sw_object.objBottomX[i] = 0;
			} else if (sw_object.objBottomX[i] >= RTSP_WIDTH) {
				sw_object.objBottomX[i] = RTSP_WIDTH - 1;
			}
		}

		hal_video_obj_region(&sw_object, RTSP_CHANNEL);
	} else {
		sw_object.objDetectNumber = 0;
		hal_video_obj_region(&sw_object, RTSP_CHANNEL);
		printf("object num = %d\r\n", sw_object.objDetectNumber);
	}

}

void mmf2_example_nn_rtsp_init(void)
{

	int voe_heap_size = video_voe_presetting(V1_ENA, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0,
						0, 0, 0, 0,
						V4_ENA, NN_WIDTH, NN_HEIGHT);						

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

#if V1_ENA
	//run_memory_scanner(&scan0);
#endif
#if V4_ENA	// move all code to SRAM?
	//run_memory_scanner(&scan1);
#endif
#if V1_ENA
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, 60);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}


	// encode_rc_parm_t rc_parm;
	// rc_parm.minQp = 28;
	// rc_parm.maxQp = 45;

	// mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RCPARAM, (int)&rc_parm);

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
#endif
#if V4_ENA
#if V4_SIM==0
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);	// start channel 4
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
#else
	array_t array;
	array.data_addr = (uint32_t) testRGB_640x360;
	array.data_len = (uint32_t) testRGB_640x360_size;
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&h264_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
#endif

#if 1
	// RTKNN
	rtknn_ctx = mm_module_open(&rtknn_module);
	if (rtknn_ctx) {
#if V1_ENA
		mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_SETOBJECT, (int)nn_set_object);
#endif
		//mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_MODEL_ID, MOBILENETSSD_20OBJ);
		mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_PARAMS, (int)&rtknn_ssd_params);
		//mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_PARAMS, (int)&rtknn_face_params);
		mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_WIDTH, NN_WIDTH);
		mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_HEIGHT, NN_HEIGHT);
		mm_module_ctrl(rtknn_ctx, CMD_RTKNN_SET_FPS, NN_EXCUTE_FPS);

	} else {
		rt_printf("RTKNN open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
	rt_printf("RTKNN opened\n\r");
#endif
#endif

	//--------------Link---------------------------
#if V1_ENA
	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
#endif
#if 1 //V4_ENA
	siso_array_rtknn = siso_create();
	if (siso_array_rtknn) {
#if V4_SIM==0
		siso_ctrl(siso_array_rtknn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_array_rtknn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_array_rtknn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
#else
		siso_ctrl(siso_array_rtknn, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
#endif
		siso_ctrl(siso_array_rtknn, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtknn_ctx, 0);
		siso_start(siso_array_rtknn);
	} else {
		rt_printf("siso_array_rtknn open fail\n\r");
		goto mmf2_example_nn_rtsp_fail;
	}
	rt_printf("siso_array_rtknn started\n\r");
#endif

	if (xTaskCreate(rgbshot_control_thread, ((const char *)"rgbshot_store"), 4096, NULL, tskIDLE_PRIORITY + 1, &rgbshot_thread) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}

	return;
mmf2_example_nn_rtsp_fail:

	return;
}