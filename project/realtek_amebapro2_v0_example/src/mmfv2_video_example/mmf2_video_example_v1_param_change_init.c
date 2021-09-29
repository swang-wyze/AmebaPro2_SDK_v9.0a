/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_video.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/
#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_HD
#define V1_FPS 30
#define V1_GOP 30
#define V1_BPS 1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;

static mm_siso_t *siso_video_rtsp_v1			= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = video_res_w[V1_RESOLUTION],
	.height = video_res_h[V1_RESOLUTION],
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = V1_FPS,
			.bps      = V1_BPS
		}
	}
};

void mmf2_video_example_v1_param_change_init(void)
{
	int voe_heap_size = video_voe_presetting(1, V1_RESOLUTION, V1_BPS, 0,
						0, NULL, NULL,
						0, NULL, NULL,
						0, NULL);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	int i = 0;
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, 60);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_DROP_TIME, 5000);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

#if 1
	rt_printf("changing resolution and fps test\n\r");


	rt_printf("changing rate control mode test\n\r");


	for (i = 0; i < 10; i++) {
		vTaskDelay(10000);
		rt_printf("changing bit rate test = %d\n\r", (1024 * 1024 + 1024 * (1 + i)));
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS, (1024 * 1024 + 1024 * (1 + i)));
	}

	vTaskDelay(20000);
	rt_printf("changing bit rate test = %d\n\r", (1024 * 1024));
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS, 1024 * 1024);


	for (i = 0; i < 10; i++) {
		vTaskDelay(5000);
		rt_printf("changing QP test = %d, %d\n\r", (5 + i * 2), (10 + i * 4));
		encode_rc_parm_t rc_parm;
		rc_parm.minQp = (5 + i * 2);
		rc_parm.maxQp = (10 + i * 4);

		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RCPARAM, (int)&rc_parm);
	}


	for (i = 0; i < 10; i++) {
		vTaskDelay(500);
		rt_printf("changing forcei test\n\r");
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_FORCE_IFRAME, NULL);
	}
#endif

	return;
mmf2_video_exmaple_v1_param_change_fail:

	return;
}