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
* ISP channel : 2
* Video type  : JPEG
*****************************************************************************/

#define V3_CHANNEL 2
#define V3_RESOLUTION VIDEO_FHD
#define V3_FPS 30
#define V3_GOP 30
#define V3_BPS 2*1024*1024
#define V3_RCMODE 2 // 1: CBR, 2: VBR

static mm_context_t *video_v3_ctx			= NULL;
static mm_context_t *rtsp2_v3_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v3			= NULL;

static video_params_t video_v3_params = {
	.stream_id = V3_CHANNEL,
	.type = VIDEO_JPEG,
	.resolution = V3_RESOLUTION,
	.width = video_res_w[V3_RESOLUTION],
	.height = video_res_h[V3_RESOLUTION],
	.fps = V3_FPS,
};


static rtsp2_params_t rtsp2_v3_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_MJPEG,
			.fps      = V3_FPS
		}
	}
};


void mmf2_video_example_v3_init(void)
{
	int voe_heap_size = video_voe_presetting(0, NULL, NULL, 0,
						0, NULL, NULL,
						1, V3_RESOLUTION, V3_BPS,
						0, NULL);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v3_ctx = mm_module_open(&video_module);
	if (video_v3_ctx) {
		mm_module_ctrl(video_v3_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v3_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v3_params);
		mm_module_ctrl(video_v3_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_v3_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v3_ctx, CMD_VIDEO_APPLY, V3_CHANNEL);	// start channel 2
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}


	rtsp2_v3_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v3_ctx) {
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v3_params);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}

	siso_video_rtsp_v3 = siso_create();
	if (siso_video_rtsp_v3) {
		siso_ctrl(siso_video_rtsp_v3, MMIC_CMD_ADD_INPUT, (uint32_t)video_v3_ctx, 0);
		siso_ctrl(siso_video_rtsp_v3, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v3_ctx, 0);
		siso_start(siso_video_rtsp_v3);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}

	mm_module_ctrl(video_v3_ctx, CMD_VIDEO_SNAPSHOT, 2);

	return;
mmf2_video_exmaple_v3_fail:

	return;
}
