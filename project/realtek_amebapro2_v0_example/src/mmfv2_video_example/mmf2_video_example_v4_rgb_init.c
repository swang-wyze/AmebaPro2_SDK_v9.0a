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

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/

#define V4_CHANNEL 4
#define V4_RESOLUTION VIDEO_VGA
#define V4_FPS 30
#define V4_GOP 30
#define V4_BPS 1024*1024

#define VIDEO_TYPE VIDEO_RGB

#if V4_RESOLUTION == VIDEO_VGA
#define V4_WIDTH	640
#define V4_HEIGHT	480
#elif V4_RESOLUTION == VIDEO_WVGA
#define V4_WIDTH	640
#define V4_HEIGHT	360
#endif

mm_context_t *video_v4_ctx			= NULL;

static video_params_t video_v4_params = {
	.stream_id = V4_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V4_RESOLUTION,
	.width = V4_WIDTH,
	.height = V4_HEIGHT,
	.bps = V4_BPS,
	.fps = V4_FPS,
	.gop = V4_GOP,
	.direct_output = 1,
};


void mmf2_video_example_v4_rgb_init(void)
{
	int voe_heap_size = video_voe_presetting(0, 0, 0, 0, 0,
						0, 0, 0, 0,
						0, 0, 0, 0,
						1, V4_WIDTH, V4_HEIGHT);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v4_ctx = mm_module_open(&video_module);
	if (video_v4_ctx) {
		mm_module_ctrl(video_v4_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v4_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_v4_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_v4_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v4_ctx, CMD_VIDEO_APPLY, V4_CHANNEL);	// start channel 4
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_v4_rgb_fail;
	}

	mm_module_ctrl(video_v4_ctx, CMD_VIDEO_YUV, 2);

	return;
mmf2_video_exmaple_v4_rgb_fail:

	return;
}
