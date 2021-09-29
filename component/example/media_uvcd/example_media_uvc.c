/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "basic_types.h"
#include "mmf2_module.h"
#include "module_uvcd.h"
#include "module_array.h"
#include "sample_h264.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "mmf2_miso.h"
#include "mmf2_mimo.h"
//#include "example_media_dual_uvcd.h"


#include "module_video.h"
#include "mmf2_pro2_video_config.h"
#include "example_media_uvcd.h"


/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

#define VIDEO_CHANNEL 0
#define VIDEO_RESOLUTION VIDEO_HD//VIDEO_HD//VIDEO_FHD 
#define VIDEO_FPS 10
#define VIDEO_GOP 10
#define VIDEO_BPS 1024*1024
#define VIDEO_RCMODE 1 // 1: CBR, 2: VBR

#define VIDEO_TYPE VIDEO_NV16//VIDEO_JPEG//VIDEO_NV12//VIDEO_JPEG//VIDEO_H264//VIDEO_HEVC//VIDEO_NV16
#define VIDEO_CODEC AV_CODEC_ID_H264


/* enum encode_type {
	VIDEO_HEVC = 0,
	VIDEO_H264,
	VIDEO_JPEG,
	VIDEO_NV12,
	VIDEO_RGB,
	VIDEO_NV16,
	VIDEO_HEVC_JPEG,
	VIDEO_H264_JPEG
}; */


static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;

static video_params_t video_v1_params = {
	.stream_id = VIDEO_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = VIDEO_RESOLUTION,
	.width = video_res_w[VIDEO_RESOLUTION],
	.height = video_res_h[VIDEO_RESOLUTION],
	.fps = VIDEO_FPS,
	.gop = VIDEO_GOP,
	.bps = VIDEO_BPS,
	.rc_mode = VIDEO_RCMODE,
	.use_static_addr = 0
};

/* static video_params_t video_v3_params = {
	.stream_id = VIDEO_CHANNEL,
	.type = VIDEO_JPEG,
	.resolution = VIDEO_RESOLUTION,
	.fps = VIDEO_FPS
}; */


mm_context_t *uvcd_ctx         = NULL;
mm_siso_t *siso_array_uvcd     = NULL;
mm_context_t *array_h264_ctx   = NULL;


extern struct uvc_format *uvc_format_ptr;

struct uvc_format *uvc_format_local = NULL;;


#define AVMEDIA_TYPE_VIDEO 0
#define AV_CODEC_ID_H264  1
#define ARRAY_MODE_LOOP		1
static array_params_t h264usb_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_H264,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 25,
			.h264_nal_size = 4,
		}
	}
};

void example_media_dual_uvcd_init(void)
{

	uvc_format_ptr = (struct uvc_format *)malloc(sizeof(struct uvc_format));
	memset(uvc_format_ptr, 0, sizeof(struct uvc_format));

	uvc_format_local = (struct uvc_format *)malloc(sizeof(struct uvc_format));
	memset(uvc_format_local, 0, sizeof(struct uvc_format));

	rtw_init_sema(&uvc_format_ptr->uvcd_change_sema, 0);

	printf("type = %d\r\n", VIDEO_TYPE);

	uvcd_ctx = mm_module_open(&uvcd_module);
	//  struct uvc_dev *uvc_ctx = (struct uvc_dev *)uvcd_ctx->priv;

	if (uvcd_ctx) {
		//mm_module_ctrl(uvcd_ctx, CMD_RTSP2_SET_APPLY, 0);
		//mm_module_ctrl(uvcd_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("uvcd open fail\n\r");
		goto mmf2_example_uvcd_fail;
	}
	//

	uvc_format_ptr->format = FORMAT_TYPE_H264;
	uvc_format_ptr->height = MAX_H;//video_v1_params.height;
	uvc_format_ptr->width = MAX_W;//video_v1_params.width;
	//uvc_format_ptr->uvcd_ext_get_cb = NULL;//RTKUSER_USB_GET;
	//uvc_format_ptr->uvcd_ext_set_cb = NULL;//RTKUSER_USB_SET;

	uvc_format_local->format = FORMAT_TYPE_H264;
	uvc_format_local->height = MAX_H;//video_v1_params.height;
	uvc_format_local->width = MAX_W;//video_v1_params.width;

	printf("foramr %d height %d width %d\r\n", uvc_format_local->format, uvc_format_local->height, uvc_format_local->width);

#if 0
#if VIDEO_TYPE == VIDEO_NV12
	video_v1_params.use_static_addr = 1;
#endif

#if VIDEO_TYPE == VIDEO_NV16
	video_v1_params.use_static_addr = 1;
#endif
#endif

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_VOE_HEAP, 32 * 1024 * 1024);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, 2);//Default 30
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		//goto mmf2_video_exmaple_v1_fail;
	}
#if (VIDEO_TYPE == VIDEO_JPEG)
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 2);
#endif

#if (VIDEO_TYPE == VIDEO_NV12)
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 2);
#endif

#if (VIDEO_TYPE == VIDEO_NV16)
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 2);
#endif

	siso_array_uvcd = siso_create();
	if (siso_array_uvcd) {
		siso_ctrl(siso_array_uvcd, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_array_uvcd, MMIC_CMD_ADD_OUTPUT, (uint32_t)uvcd_ctx, 0);
		siso_start(siso_array_uvcd);
	} else {
		rt_printf("siso_array_uvcd open fail\n\r");
		//goto mmf2_example_h264_array_rtsp_fail;
	}
	rt_printf("siso_array_uvcd started\n\r");

	while (1) {
		rtw_down_sema(&uvc_format_ptr->uvcd_change_sema);

		printf("f:%d h:%d s:%d w:%d\r\n", uvc_format_ptr->format, uvc_format_ptr->height, uvc_format_ptr->state, uvc_format_ptr->width);
#if 1
		if ((uvc_format_local->format != uvc_format_ptr->format) || (uvc_format_local->width != uvc_format_ptr->width) ||
			(uvc_format_local->height != uvc_format_ptr->height)) {
			printf("change f:%d h:%d s:%d w:%d\r\n", uvc_format_ptr->format, uvc_format_ptr->height, uvc_format_ptr->state, uvc_format_ptr->width);
			if (uvc_format_local->format == FORMAT_TYPE_MJPEG) {
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 0);
				video_v1_params.use_static_addr = 0;
			}
			if (uvc_format_local->format == FORMAT_TYPE_YUY2) {
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 0);
				video_v1_params.use_static_addr = 0;
			}
			if (uvc_format_local->format == FORMAT_TYPE_NV12) {
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 0);
				video_v1_params.use_static_addr = 0;
			}

			if (uvc_format_ptr->format == FORMAT_TYPE_YUY2) {
				siso_pause(siso_array_uvcd);
				vTaskDelay(1000);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
				vTaskDelay(1000);
				video_v1_params.type = VIDEO_NV16;
				//video_v1_params.use_static_addr = 1;
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 2);
				siso_resume(siso_array_uvcd);
			} else if (uvc_format_ptr->format == FORMAT_TYPE_NV12) {
				siso_pause(siso_array_uvcd);
				vTaskDelay(1000);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
				vTaskDelay(1000);
				video_v1_params.type = VIDEO_NV12;
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_YUV, 2);
				siso_resume(siso_array_uvcd);
			} else if (uvc_format_ptr->format == FORMAT_TYPE_H264) {
				siso_pause(siso_array_uvcd);
				vTaskDelay(1000);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
				vTaskDelay(1000);
				video_v1_params.type = VIDEO_H264;
				video_v1_params.use_static_addr = 1;
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
				siso_resume(siso_array_uvcd);
			} else if (uvc_format_ptr->format == FORMAT_TYPE_MJPEG) {
				siso_pause(siso_array_uvcd);
				vTaskDelay(1000);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
				vTaskDelay(1000);
				video_v1_params.type = VIDEO_JPEG;
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 2);
				siso_resume(siso_array_uvcd);
			} else if (uvc_format_ptr->format == FORMAT_TYPE_H265) {
				siso_pause(siso_array_uvcd);
				vTaskDelay(1000);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
				vTaskDelay(1000);
				video_v1_params.type = VIDEO_HEVC;
				video_v1_params.use_static_addr = 1;
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, VIDEO_CHANNEL);	// start channel 0
				siso_resume(siso_array_uvcd);
			}
			uvc_format_local->format = uvc_format_ptr->format;
			uvc_format_local->width = uvc_format_ptr->width;
			uvc_format_local->height = uvc_format_ptr->height;
		}
#endif
	}

mmf2_example_uvcd_fail:

	return;
}

void example_media_uvcd_main(void *param)
{
	example_media_dual_uvcd_init();
	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(1000);
	}
}

void example_media_uvcd(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_media_uvcd_main, ((const char *)"example_media_dual_uvcd_main"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_media_two_source_main: Create Task Error\n");
	}
}