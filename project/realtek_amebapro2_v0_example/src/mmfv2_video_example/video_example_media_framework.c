/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "video_example_media_framework.h"
#include <FreeRTOS.h>
#include <task.h>
#include "module_video.h"
#include "mmf2_pro2_video_config.h"

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

static void wifi_common_init()
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}

}

//------------------------------------------------------------------------------
// video support examples
//------------------------------------------------------------------------------
static void example_mmf2_video_surport()
{
	//P2P CLOUD
#if CONFIG_EXAMPLE_MEDIA_CLOUD
	// 1 Video (H264/HEVC) 1 Audio -> P2P
	mmf2_video_example_p2p_av_init();
#endif

	//NN
#if CONFIG_EXAMPLE_MEDIA_NN
	mmf2_example_nn_rtsp_init();
#endif

	// CH1 Video -> H264/HEVC -> RTSP
	//mmf2_video_example_v1_init();

	// CH2 Video -> H264/HEVC -> RTSP
	//mmf2_video_example_v2_init();

	// CH3 Video -> JPEG -> RTSP
	//mmf2_video_example_v3_init();

	// CH1 Video -> H264/HEVC -> RTSP + SNAPSHOT
	//mmf2_video_example_v1_shapshot_init();

	// 1 Video (H264/HEVC) -> 2 RTSP (V1, V2)
	//mmf2_video_example_simo_init();

	// 1 Video (H264/HEVC) 1 Audio -> RTSP
	//mmf2_video_example_av_init();

	// 2 Video (H264/HEVC) 1 Audio -> 2 RTSP (V1+A, V2+A)
	//mmf2_video_example_av2_init();

	// 1 Video (H264/HEVC) 1 Audio -> 2 RTSP (V+A)
	//mmf2_video_example_av21_init();

	// 1 Video (H264/HEVC) 1 Audio -> MP4 (SD card)
	//mmf2_video_example_av_mp4_init();

	// 1V1A RTSP MP4
	// H264 -> RTSP and mp4
	// AUDIO -> AAC  -> RTSP and mp4
	//mmf2_video_example_av_rtsp_mp4_init();

	// Joint test
	// H264 -> RTSP (with AUDIO)
	// H264 -> RTSP (with AUDIO)
	// AUDIO -> AAC  -> RTSP
	// RTP   -> AAD  -> AUDIO
	//mmf2_video_example_joint_test_init();

	// Joint test RTSP MP4
	// H264 -> RTSP (V1)
	// H264 -> MP4 (V2)
	// AUDIO -> AAC  -> RTSP and mp4
	// RTP   -> AAD  -> AUDIO
	//mmf2_video_example_joint_test_rtsp_mp4_init();

	// H264 and 2way audio (G711, PCMU)
	// H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	// ARRAY (PCMU) -> G711D -> AUDIO (doorbell)
	//mmf2_video_example_2way_audio_pcmu_doorbell_init();

	// H264 and 2way audio (G711, PCMU)
	// H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	//mmf2_video_example_2way_audio_pcmu_init();

	// ARRAY (H264) -> RTSP (V)
	//mmf2_video_example_array_rtsp_init();

	// V1 parameter change
	//mmf2_video_example_v1_param_change_init();

	// NN inference -> RTSP (V)
	mmf2_example_vipnn_rtsp_init();
}

void video_example_main(void *param)
{
#if CONFIG_EXAMPLE_MEDIA_VIDEO
	wifi_common_init();

	example_mmf2_video_surport();

	//P2P CLOUD
#if CONFIG_EXAMPLE_MEDIA_CLOUD
	skynet_device_run();
#endif
#endif

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(10000);
		// extern mm_context_t *video_v1_ctx;
		// mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRINT_INFO, 0);
	}
}

void video_example_media_framework(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(video_example_main, ((const char *)"mmf2_video"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n video_example_main: Create Task Error\n");
	}
}

extern int isp_ch_buf_num[5];
#define ISP_COMMON_BUF 580*1024
#define ENC_COMMON_BUF 580*1024
#define SNAPSHOT_BUF   300*1024 

int video_voe_presetting(int v1_enable, int v1_w, int v1_h, int v1_bps, int v1_shapshot,
						 int v2_enable, int v2_w, int v2_h, int v2_bps,
						 int v3_enable, int v3_w, int v3_h, int v3_bps,
						 int v4_enable, int v4_w, int v4_h)
{
	int voe_heap_size = 0;
	int v3dnr_w = 1920;
	int v3dnr_h = 1080;
	
	if((v1_w > v3dnr_w) && v1_enable)
	{
		v3dnr_w = v1_w;
		v3dnr_h = v1_h;
	}
	
	if((v2_w > v3dnr_w) && v2_enable)
	{
		v3dnr_w = v2_w;
		v3dnr_h = v2_h;
	}
	
	if((v3_w > v3dnr_w) && v3_enable)
	{
		v3dnr_w = v3_w;
		v3dnr_h = v3_h;
	}
	
	if((v4_w > v3dnr_w) && v4_enable)
	{
		v3dnr_w = v4_w;
		v3dnr_h = v4_h;
	}
	
	//3dnr
	voe_heap_size += ((v3dnr_w * v3dnr_h * 3) / 2);
	printf("3dnr = %d,%d,%d\r\n",v3dnr_w,v3dnr_h,voe_heap_size);

	if (v1_enable) {
		//ISP buffer
		voe_heap_size += ((v1_w * v1_h * 3) / 2) * isp_ch_buf_num[0];
		//ISP common
		voe_heap_size += ISP_COMMON_BUF;
		//enc ref
		voe_heap_size += ((v1_w * v1_h * 3) / 2) * 2;
		//enc common
		voe_heap_size += ENC_COMMON_BUF;
		//enc buffer
		voe_heap_size += ((v1_w * v1_h) / 2 +  v1_bps * 2);
		//shapshot
		if (v1_shapshot) {
			voe_heap_size += ((v1_w * v1_h * 3) / 2) + SNAPSHOT_BUF;
		}
	}

	printf("v1 = %d,%d,%d\r\n",v1_w,v1_h,voe_heap_size);

	if (v2_enable) {
		//ISP buffer
		voe_heap_size += ((v2_w * v2_h * 3) / 2) * isp_ch_buf_num[1];
		//ISP common
		voe_heap_size += ISP_COMMON_BUF;
		//enc ref
		voe_heap_size += ((v2_w * v2_h * 3) / 2) * 2;
		//enc common
		voe_heap_size += ENC_COMMON_BUF;
		//enc buffer
		voe_heap_size += ((v2_w * v2_h) / 2 +  v2_bps * 2);
	}

	printf("v2 = %d,%d,%d\r\n",v2_w,v2_h,voe_heap_size);

	if (v3_enable) {
		//ISP buffer
		voe_heap_size += ((v3_w * v3_h * 3) / 2) * isp_ch_buf_num[2];
		//ISP common
		voe_heap_size += ISP_COMMON_BUF;
		//enc ref
		voe_heap_size += ((v3_w * v3_h * 3) / 2) * 2;
		//enc common
		voe_heap_size += ENC_COMMON_BUF;
		//enc buffer
		voe_heap_size += ((v3_w * v3_h) / 2 +  v3_bps * 2);
	}

	printf("v3 = %d,%d,%d\r\n",v3_w,v3_h,voe_heap_size);

	if (v4_enable) {
		//ISP buffer
		voe_heap_size += v4_w * v4_h * 3 * isp_ch_buf_num[4];
		//ISP common
		voe_heap_size += ISP_COMMON_BUF;
	}
	
	printf("v4 = %d,%d,%d\r\n",v4_w,v4_h,voe_heap_size);

	return voe_heap_size;
}
