/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <osdep_service.h>
#include "mmf2.h"
#include "mmf2_dbg.h"

#include "module_video.h"
#include "module_rtsp2.h"


#include <math.h>
#include "platform_stdlib.h"

#include <unistd.h>
#include <sys/wait.h>

#include "base_type.h"

#include "cmsis.h"
#include "error.h"


#include "hal.h"
#include "hal_video.h"

hal_video_adapter_t *v_adapter;

char *fmt_table[] = {"hevc", "h264", "jpeg", "nv12", "rgb", "nv16", "hevc", "h264"};
char *paramter_table[] = {
	"-l 1 --gopSize=1 -U1 -u0 -q-1 --roiMapDeltaQpBlockUnit 0 --roiMapDeltaQpEnable 1 --dbg 1 --obj 1",	// HEVC
	"-l 1 --gopSize=1 -U1 -u0 -q-1 --roiMapDeltaQpBlockUnit 0 --roiMapDeltaQpEnable 1 --dbg 1 --obj 1",	// H264
	"-g 1 -b 1 -q 7 --dbg 1 --obj 1",			// JPEG
	"--dbg 1 --obj 1",							// NV12
	"--dbg 1 --obj 1",							// RGB
	"--dbg 1 --obj 1",							// NV16
};

int framecnt = 0;
int jpegcnt = 0;
int incb = 0;
int ch1framecnt = 0;
int ch2framecnt = 0;
int rgb_lock = 0;
int isp_ch_buf_num[5] = {2,2,2,2,1};

void video_frame_complete_cb(void *param1, void  *param2, uint32_t arg)
{
	incb = 1;
	enc2out_t *enc2out = (enc2out_t *)param1;
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)param2;
	commandLine_s *cml = v_adp->cmd[enc2out->ch];
	video_ctx_t *ctx = (video_ctx_t *)arg;
	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *output_item;

	u32 timestamp = xTaskGetTickCount();
	int is_output_ready = 0;

	if (ctx->params.direct_output == 1) {
		goto show_log;
	}

	// Snapshot JPEG
	if (enc2out->codec & CODEC_JPEG && enc2out->jpg_len > 0) { // JPEG
		if (ctx->snapshot_cb != NULL) {
			dcache_invalidate_by_addr((uint32_t *)enc2out->jpg_addr, enc2out->jpg_len);
			dcache_invalidate_by_addr((uint32_t *)v_adp->outbuf[enc2out->ch], sizeof(hal_video_buf_s));
			ctx->snapshot_cb(enc2out->jpg_addr, enc2out->jpg_len);
			//if (cml->voe == 1) {
			//hal_video_release(enc2out->ch, enc2out->jpg_len);
			//}
		} else {
			char *tempaddr;
			if (ctx->params.use_static_addr == 0) {
				tempaddr = (char *)malloc(enc2out->jpg_len);
				if (tempaddr == NULL) {
					printf("malloc fail = %d\r\n", enc2out->jpg_len);
					goto show_log;
				}
			}

			is_output_ready = xQueueReceive(mctx->output_recycle, (void *)&output_item, 10);
			if (is_output_ready) {
				dcache_invalidate_by_addr((uint32_t *)enc2out->jpg_addr, enc2out->jpg_len);
				dcache_invalidate_by_addr((uint32_t *)v_adp->outbuf[enc2out->ch], sizeof(hal_video_buf_s));
				if (ctx->params.use_static_addr) {
					output_item->data_addr = (char *)enc2out->jpg_addr;
				} else {
					output_item->data_addr = (char *)tempaddr;//malloc(enc2out->jpg_len);
					memcpy(output_item->data_addr, (char *)enc2out->jpg_addr, enc2out->jpg_len);
				}
				output_item->size = enc2out->jpg_len;
				output_item->timestamp = timestamp;
				output_item->hw_timestamp = enc2out->post_time;
				output_item->type = AV_CODEC_ID_MJPEG;
				output_item->index = 0;

				if (xQueueSend(mctx->output_ready, (void *)&output_item, 10) != pdTRUE) {
					//printf("\r\n xQueueSend fail \r\n");
				}

			} else {
				//printf("\r\n xQueueReceive fail \r\n");
				if (ctx->params.use_static_addr == 0) {
					free(tempaddr);
				}
			}

			//if (cml->voe == 1 && ctx->params.use_static_addr == 0) {
			//hal_video_release(enc2out->ch, enc2out->jpg_len);
			//}
		}
	}

	if (enc2out->enc_len > 0 && (enc2out->codec & CODEC_H264 || enc2out->codec & CODEC_HEVC ||
								 enc2out->codec & CODEC_RGB || enc2out->codec & CODEC_NV12 ||
								 enc2out->codec & CODEC_NV16)) {
		char *tempaddr;
		if (ctx->params.use_static_addr == 0) {
			tempaddr = (char *)malloc(enc2out->enc_len);
			if (tempaddr == NULL) {
				printf("malloc fail = %d\r\n", enc2out->enc_len);
				if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
					hal_video_isp_buf_release(enc2out->ch, enc2out->isp_addr);
				} else {
				hal_video_release(enc2out->ch, enc2out->enc_len);
				}
				goto show_log;
			}
		}

#if 0 //debug
		if (enc2out->enc_len > (1024 * 1024)) {
			printf("\r\n have big frame addr=\r\n", enc2out->enc_addr);
		}

		if (enc2out->ch > 4) {
			printf("\r\n have wrong channel addr=\r\n", enc2out->enc_addr);
		}

		if (enc2out->type == VCENC_INTRA_FRAME) {
			printf("I frame = %d\r\n", enc2out->ch);
		}

		if (enc2out->ch == 0) {
			ch1framecnt++;
			if (ch1framecnt == 30) {
				printf("\r\n ch1 1s frames \r\n");
				ch1framecnt = 0;
			}
		}

		if (enc2out->ch == 1) {
			ch2framecnt++;
			if (ch2framecnt == 15) {
				printf("\r\n ch2 1s frames \r\n");
				ch2framecnt = 0;
			}
		}

#endif

		is_output_ready = xQueueReceive(mctx->output_recycle, (void *)&output_item, 10);
		if (is_output_ready) {
			if (enc2out->codec == CODEC_H264) {
				output_item->type = AV_CODEC_ID_H264;
			} else if (enc2out->codec == CODEC_HEVC) {
				output_item->type = AV_CODEC_ID_H265;
			} else if (enc2out->codec == CODEC_RGB) {
				output_item->type = AV_CODEC_ID_RGB888;
			} else if (enc2out->codec == CODEC_NV12) {
				output_item->type = AV_CODEC_ID_UNKNOWN;
			} else if (enc2out->codec == CODEC_NV16) {
				output_item->type = AV_CODEC_ID_UNKNOWN;
			}

			if (enc2out->codec <= CODEC_JPEG) {
				dcache_invalidate_by_addr((uint32_t *)enc2out->enc_addr, enc2out->enc_len);
				dcache_invalidate_by_addr((uint32_t *)v_adp->outbuf[enc2out->ch], sizeof(hal_video_buf_s));

				if (enc2out->codec == CODEC_H264) {
					uint8_t *ptr = (uint8_t *)enc2out->enc_addr;
					if (ptr[0] != 0 || ptr[1] != 0) {
						printf("\r\nH264 stream error\r\n");
						printf("\r\n(%d/%d) %x %x %x %x\r\n", enc2out->enc_len, enc2out->finish, *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3));
					}
				}

				if (ctx->params.use_static_addr) {
					output_item->data_addr = (char *)enc2out->enc_addr;
				} else {
					output_item->data_addr = (char *)tempaddr;//malloc(enc2out->enc_len);
					memcpy(output_item->data_addr, (char *)enc2out->enc_addr, enc2out->enc_len);
					if (ctx->params.use_static_addr == 0) {
						hal_video_release(enc2out->ch, enc2out->enc_len);
					}
				}

			} else {
				dcache_invalidate_by_addr((uint32_t *)enc2out->isp_addr, enc2out->enc_len);
				dcache_invalidate_by_addr((uint32_t *)v_adp->outbuf[enc2out->ch], sizeof(hal_video_buf_s));
				if (ctx->params.use_static_addr) {
					output_item->data_addr = (char *)enc2out->isp_addr;
				} else {
					output_item->data_addr = (char *)tempaddr;//malloc(enc2out->enc_len);
					memcpy(output_item->data_addr, (char *)enc2out->isp_addr, enc2out->enc_len);
					hal_video_isp_buf_release(enc2out->ch, enc2out->isp_addr);
				}
			}

			output_item->size = enc2out->enc_len;
			output_item->timestamp = timestamp;
			output_item->hw_timestamp = enc2out->post_time;
			output_item->index = 0;

			if (xQueueSend(mctx->output_ready, (void *)&output_item, 10) != pdTRUE) {
				//printf("\r\n xQueueSend fail \r\n");
				if (enc2out->codec <= CODEC_JPEG) {
					hal_video_release(enc2out->ch, enc2out->enc_len);
				} else {
					hal_video_isp_buf_release(enc2out->ch, enc2out->isp_addr);
				}
			}

		} else {
			//printf("\r\n xQueueReceive fail \r\n");

			if (ctx->params.use_static_addr == 0) {
				free(tempaddr);
			}

			if (enc2out->codec <= CODEC_JPEG) {
				hal_video_release(enc2out->ch, enc2out->enc_len);
			} else {
				hal_video_isp_buf_release(enc2out->ch, enc2out->isp_addr);
			}
			}
		}
	
	if (enc2out->codec >= CODEC_RGB) {
		printf("yuv in 0x%x\r\n",enc2out->isp_addr);
	}

show_log:

	if (ctx->params.direct_output == 1) {

		if (enc2out->codec & (CODEC_H264 | CODEC_HEVC)) {
			printf("(%s)(%ldx%ld)(ch%d)(%s) Size(%6ld) QP(%2d) %3d ms %3d ms %3d ms CRC(0x%x)  \n"
				   , (enc2out->codec & CODEC_H264) != 0 ? "H264" : "HEVC"
				   , cml->width, cml->height, enc2out->ch
				   , (enc2out->type == VCENC_INTRA_FRAME) ? "I" : "P"
				   , enc2out->enc_len, enc2out->qp
				   , enc2out->pre_time >> 10
				   , enc2out->enc_time >> 10
				   , enc2out->post_time >> 10
				   , v_adp->crc32);

		}


		if (enc2out->codec & CODEC_JPEG && enc2out->jpg_len > 0) { // JPEG
			printf("(JPEG)(%ldx%ld)(ch%d)(I) Size(%6ld) QP(  ) %3d ms %3d ms %3d ms CRC(0x%x)  \n"
				   , cml->width, cml->height, enc2out->ch
				   , enc2out->jpg_len
				   , enc2out->pre_time >> 10
				   , enc2out->enc_time >> 10
				   , enc2out->post_time >> 10
				   , v_adp->crc32);
		}

		if (enc2out->codec & CODEC_RGB) {
			printf("(%s)(%ldx%ld)(ch%d)(%s) Size(%6ld) QP(%2d) %3d ms %3d ms %3d ms CRC(0x%x)  \n"
				   , "RGB"
				   , cml->width, cml->height, enc2out->ch
				   , (enc2out->type == VCENC_INTRA_FRAME) ? "I" : "P"
				   , enc2out->enc_len, enc2out->qp
				   , enc2out->pre_time >> 10
				   , enc2out->enc_time >> 10
				   , enc2out->post_time >> 10
				   , v_adp->crc32);

		}

		if (enc2out->codec & CODEC_NV12) {
			printf("(%s)(%ldx%ld)(ch%d)(%s) Size(%6ld) QP(%2d) %3d ms %3d ms %3d ms CRC(0x%x)  \n"
				   , "NV12"
				   , cml->width, cml->height, enc2out->ch
				   , (enc2out->type == VCENC_INTRA_FRAME) ? "I" : "P"
				   , enc2out->enc_len, enc2out->qp
				   , enc2out->pre_time >> 10
				   , enc2out->enc_time >> 10
				   , enc2out->post_time >> 10
				   , v_adp->crc32);

		}

		if (enc2out->codec & CODEC_NV16) {
			printf("(%s)(%ldx%ld)(ch%d)(%s) Size(%6ld) QP(%2d) %3d ms %3d ms %3d ms CRC(0x%x)  \n"
				   , "NV16"
				   , cml->width, cml->height, enc2out->ch
				   , (enc2out->type == VCENC_INTRA_FRAME) ? "I" : "P"
				   , enc2out->enc_len, enc2out->qp
				   , enc2out->pre_time >> 10
				   , enc2out->enc_time >> 10
				   , enc2out->post_time >> 10
				   , v_adp->crc32);

		}

	}

	if (ctx->params.direct_output == 1) {
		if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC)) != 0) {
			hal_video_release(enc2out->ch, enc2out->enc_len);
		} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
			hal_video_isp_buf_release(enc2out->ch, enc2out->isp_addr);
		}
	}

	//close output task
	if (enc2out->finish == LAST_FRAME) {

	}
	incb = 0;
}


int video_control(void *p, int cmd, int arg)
{
	video_ctx_t *ctx = (video_ctx_t *)p;
	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *tmp_item;

	switch (cmd) {
	case CMD_VIDEO_SET_PARAMS:
		memcpy(&ctx->params, (void *)arg, sizeof(video_params_t));
		break;
	case CMD_VIDEO_GET_PARAMS:
		memcpy((void *)arg, &ctx->params, sizeof(video_params_t));
		break;
	case CMD_VIDEO_SET_RCPARAM: {
		int ch = ctx->params.stream_id;
		rate_ctrl_s rc_ctrl;
		encode_rc_parm_t *rc_param = (encode_rc_parm_t *)arg;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));

		rc_ctrl.maxqp = rc_param->maxQp;
		rc_ctrl.minqp = rc_param->minQp;
		hal_video_set_rc(&rc_ctrl, ch);
	}
	break;
	case CMD_VIDEO_STREAMID:
		ctx->params.stream_id = arg;
		break;
	case CMD_VIDEO_STREAM_START: {
		int ch = ctx->params.stream_id;
		hal_video_open(ctx->v_adp, ch);
		//hal_video_start(ctx->v_adp, ch);
	}
	break;
	case CMD_VIDEO_STREAM_STOP: {
		int ch = ctx->params.stream_id;
		//wait incb
		do {
			vTaskDelay(1);
		} while (incb);

		printf("leave cb\r\n");

		hal_video_stop(ctx->v_adp, ch);
		printf("hal_video_stop\r\n");
		hal_video_close(ctx->v_adp, ch);
		printf("hal_video_close\r\n");
		vTaskDelay(10);
		if (hal_video_cmd_reset(ctx->v_adp, ch) != OK) {
			printf("hal_video_cmd_reset fail\n");
		}
		printf("hal_video_cmd_reset\r\n");
	}
	break;
	case CMD_VIDEO_FORCE_IFRAME: {
		int ch = ctx->params.stream_id;
		hal_video_froce_i(ch);
	}
	break;
	case CMD_VIDEO_BPS: {
		int ch = ctx->params.stream_id;
		rate_ctrl_s rc_ctrl;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
		rc_ctrl.bps = arg;
		hal_video_set_rc(&rc_ctrl, ch);
	}
	break;
	case CMD_VIDEO_GOP: {
		int ch = ctx->params.stream_id;
		rate_ctrl_s rc_ctrl;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
		rc_ctrl.gop = arg;
		hal_video_set_rc(&rc_ctrl, ch);
	}
	break;
	case CMD_VIDEO_SNAPSHOT: {
		int ch = ctx->params.stream_id;
		hal_video_jpg_out(ch, arg);
	}
	break;
	case CMD_VIDEO_YUV: {
		int ch = ctx->params.stream_id;
		if (ch == 4) {
			if (rgb_lock == 0) {
				hal_video_yuv_out(ch, arg);
				rgb_lock = 1;
			}
		} else {
			hal_video_yuv_out(ch, arg);
		}
	}
	break;
	case CMD_ISP_SET_RAWFMT: {
		int ch = ctx->params.stream_id;
		hal_video_isp_set_rawfmt(ch, arg);
	}
	break;
	case CMD_VIDEO_SNAPSHOT_CB:
		ctx->snapshot_cb = (int (*)(uint32_t, uint32_t))arg;
		break;
	case CMD_VIDEO_UPDATE:
		break;
	case CMD_VIDEO_SET_VOE_HEAP:
		hal_video_voe_open(ctx->v_adp, arg, 1024);
		break;
	case CMD_VIDEO_PRINT_INFO:
		hal_video_mem_info(ctx->v_adp, 0);
		hal_video_buf_info(ctx->v_adp, 0);
		break;
	case CMD_VIDEO_APPLY: {
		int ch = arg;
		ctx->params.stream_id = ch;
		int fps = 1;
		int gop = 30;
		int rcMode = 2;
		int bps = 1024 * 1024;
		int ispfps = 15;

		char cmd1[256];
		char cmd2[256];
		char cmd3[256];
		char fps_cmd1[48];
		char fps_cmd2[48];
		char fps_cmd3[48];
		int type;
		int res = 0;
		int codec;

		int out_rsvd_size = (ctx->params.width * ctx->params.height) / 2;
		int out_buf_size = ctx->params.bps * 2;
		//if(out_buf_size < out_rsvd_size)
		//out_buf_size = out_rsvd_size + ctx->params.bps;
	    printf("video w = %d, video h = %d\r\n",ctx->params.width, ctx->params.height);

		memset(fps_cmd1, 0x0, 48);
		memset(fps_cmd2, 0x0, 48);
		memset(fps_cmd3, 0x0, 48);
		memset(cmd1, 0x0, 256);
		memset(cmd2, 0x0, 256);
		memset(cmd3, 0x0, 256);

		type = ctx->params.type;

		res = ctx->params.resolution;

		if (ctx->params.fps) {
			fps = ctx->params.fps;
		}

		if (ctx->params.bps) {
			bps = ctx->params.bps;
		}

		if (ctx->params.gop) {
			gop = ctx->params.gop;
		}

		if (ctx->params.rc_mode) {
			rcMode = ctx->params.rc_mode - 1;
		}

		switch (type) {
		case 0:
			codec = CODEC_HEVC;
			break;
		case 1:
			codec = CODEC_H264;
			break;
		case 2:
			codec = CODEC_JPEG;
			break;
		case 3:
			codec = CODEC_NV12;
			break;
		case 4:
			codec = CODEC_RGB;
			break;
		case 5:
			codec = CODEC_NV16;
			break;
		case 6:
			codec = CODEC_HEVC | CODEC_JPEG;
			break;
		case 7:
			codec = CODEC_H264 | CODEC_JPEG;
			break;
		// case 8:
			// codec = CODEC_HEVC | CODEC_NV12;
			// break;
		// case 9:
			// codec = CODEC_HEVC | CODEC_JPEG | CODEC_NV12;
			// break;
		// case 10:
			// codec = CODEC_H264 | CODEC_JPEG | CODEC_NV12;
			// break;
		// case 11:
			// codec = CODEC_HEVC | CODEC_JPEG | CODEC_NV16;
			// break;
		// case 12:
			// codec = CODEC_H264 | CODEC_JPEG | CODEC_NV16;
			// break;
		}

		if ((codec & (CODEC_HEVC | CODEC_H264)) != 0) {
			sprintf(fps_cmd1, "-f %d -j %d -R %d -B %d --vbr %d", fps, fps, gop, bps, rcMode);
			sprintf(cmd1, "%s %d %s -w %d -h %d --codecFormat %d %s -i isp"
					, fmt_table[(codec & (CODEC_HEVC | CODEC_H264)) - 1]
					, ch
					, fps_cmd1
					, ctx->params.width
					, ctx->params.height
					, (codec & (CODEC_HEVC | CODEC_H264)) - 1
					, paramter_table[(codec & (CODEC_HEVC | CODEC_H264)) - 1]);
		}

		if ((codec & CODEC_JPEG) != 0) {
			sprintf(fps_cmd2, "-n %d", fps);
			sprintf(cmd2, "%s %d %s -w %d -h %d --codecFormat %d %s -i isp"
					, fmt_table[2]
					, ch
					, fps_cmd2
					, ctx->params.width
					, ctx->params.height
					, 2
					, paramter_table[2]);
		}

		if ((codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
			int value;
			if ((codec & CODEC_NV12) != 0) {
				value = 3;
			} else if ((codec & CODEC_RGB) != 0) {
				value = 4;
			} else {
				value = 5;
			}
			sprintf(fps_cmd3, "-j %d", fps);
			sprintf(cmd3, "%s %d %s -w %d -h %d --codecFormat %d %s -i isp"
					, fmt_table[value]
					, ch
					, fps_cmd3
					, ctx->params.width
					, ctx->params.height
					, 3
					, paramter_table[value]);
			printf("str3 %s\n", cmd3);

		}


		printf("%s\n", cmd1);
		printf("%s\n", cmd2);
		printf("%s\n", cmd3);

		//string to command
		if (hal_video_str2cmd(cmd1, cmd2, cmd3, ctx->v_adp) == NOK) {
			printf("hal_video_str2cmd fail\n");
			return NULL;
		}

		if (hal_video_cb_register(ctx->v_adp, video_frame_complete_cb, (uint32_t)ctx, ch) != OK) {
			printf("hal_video_cb_register fail\n");
		}

		hal_video_isp_buf_num(ctx->v_adp, ch, isp_ch_buf_num[ch]);

		if (codec & (CODEC_HEVC | CODEC_H264 | CODEC_JPEG)) {
			hal_video_enc_buf(ctx->v_adp, ch, out_buf_size, out_rsvd_size);
		}

		/* Encoder initialization */
		if (hal_video_open(ctx->v_adp, ch) != OK) {
			printf("hal_video_open fail\n");
		}

		printf("hal_video_start\r\n");
		if (hal_video_start(ctx->v_adp, ch) != OK) {
			printf("hal_video_start fail\n");
		}
	}
	break;
	}
	return 0;
}

int video_handle(void *ctx, void *input, void *output)
{
	return 0;
}

void *video_destroy(void *p)
{
	video_ctx_t *ctx = (video_ctx_t *)p;

	hal_video_deinit(ctx->v_adp);

	free(ctx);
	return NULL;
}


void *video_create(void *parent)
{
	int ret;

	video_ctx_t *ctx = malloc(sizeof(video_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(video_ctx_t));

	ctx->parent = parent;

	// HW enable & allocation adapter memory
	if (v_adapter == NULL) {
		v_adapter = hal_video_init();
		if (v_adapter == NULL) {
			printf("hal_video_init fail\n");
			return NULL;
		}
		ctx->v_adp = v_adapter;
		if(USE_SENSOR == SENSOR_GC2053)
		{
			extern int _binary_iq_gc2053_bin_start[];
			hal_video_load_iq(ctx->v_adp, _binary_iq_gc2053_bin_start);
		extern int __voe_code_start__[];			// VOE DDR address
		extern int _binary_sensor_gc2053_bin_start[];	// SENSOR binary address
		hal_video_load_sensor((voe_cpy_t)memcpy, _binary_sensor_gc2053_bin_start, __voe_code_start__);
		}
		else if(USE_SENSOR == SENSOR_PS5258)
		{
			extern int _binary_iq_ps5258_bin_start[];
			hal_video_load_iq(ctx->v_adp, _binary_iq_ps5258_bin_start);
			extern int __voe_code_start__[];			// VOE DDR address
			extern int _binary_sensor_ps5258_bin_start[];	// SENSOR binary address
			hal_video_load_sensor((voe_cpy_t)memcpy, _binary_sensor_ps5258_bin_start, __voe_code_start__);	
		}
		else
		{
			printf("unkown sensor\r\n");
		}		
	} else {
		ctx->v_adp = v_adapter;
	}

	return ctx;

video_create_fail:

	if (ctx) {
		free(ctx);
	}
	return NULL;
}

void *video_new_item(void *p)
{
	return NULL;
}

void *video_del_item(void *p, void *d)
{

	video_ctx_t *ctx = (video_ctx_t *)p;
	int ch = ctx->params.stream_id;

	if (ctx->params.use_static_addr == 0) {
		if (d) {
			free(d);
		}
	}

	return NULL;
}

void *video_voe_release_item(void *p, void *d, int length)
{
	video_ctx_t *ctx = (video_ctx_t *)p;
	mm_queue_item_t *free_item = (mm_queue_item_t *)d;
	int ch = ctx->params.stream_id;

	if (free_item->type == AV_CODEC_ID_H264 || free_item->type == AV_CODEC_ID_H265 || free_item->type == AV_CODEC_ID_MJPEG) {
		hal_video_release(ch, length);
	} else if (free_item->type == AV_CODEC_ID_RGB888 ){
		printf("RGB release 0x%x\r\n",free_item->data_addr);
		hal_video_isp_buf_release(ch, free_item->data_addr);
		rgb_lock = 0;
	}
	else{
		printf("YUV release 0x%x\r\n",free_item->data_addr);
		hal_video_isp_buf_release(ch, free_item->data_addr);
		rgb_lock = 0;
	}


	return NULL;
}

int isp_ctrl_cmd(int argc, char **argv)
{

	int id;
	int set_flag;
	int set_value;
	int read_value;
	int ret;
	if (argc >= 2) {
		set_flag = atoi(argv[0]);
		id = atoi(argv[1]);
		if (set_flag == 0) {
			ret = hal_video_isp_ctrl(id, set_flag, &read_value);
			if ( ret == 0 ) {
				printf("result 0x%08x %d \r\n", read_value, read_value);
			} else {
				printf("isp_ctrl get error\r\n");
			}
		} else {

			if (argc >= 3) {
				set_value = atoi(argv[2]);

				ret = hal_video_isp_ctrl(id, set_flag, &set_value);
				if (ret != 0) {
					printf("isp_ctrl set error\r\n");
				}
			} else {
				printf("isp_ctrl set error : need 3 argument: set_flag id  set_value\r\n");
			}
		}

	} else {
		printf("isp_ctrl  error : need 2~3 argument: set_flag id  [set_value] \r\n");
	}
	return OK;
}

int iq_tuning_cmd(int argc, char** argv) {
	int ccmd;
	int vreg_len;
	int vreg_offset;
	uint32_t cmd_data;
	struct isp_tuning_cmd *iq_cmd;
	if (argc < 1) {	// usage
		printf("iqtun cmd\r\n");
		printf("      0: rts_isp_tuning_get_iq\n");
		printf("      1 : rts_isp_tuning_set_iq\r\n");
		printf("      2 : rts_isp_tuning_get_statis\r\n");
		printf("      3 : rts_isp_tuning_get_param\r\n");
		printf("      4 : rts_isp_tuning_set_param\r\n");
		printf("      5 offset lens : rts_isp_tuning_read_vreg \r\n");
		printf("      6 offset lens value1 value2: rts_isp_tuning_write_vreg\r\n");
		return NOK;
	}
	ccmd = atoi(argv[0]);





	cmd_data = malloc(IQ_CMD_DATA_SIZE+32); // for cache alignment
	if (cmd_data == NULL) {
		printf("malloc cmd buf fail\r\n");
		return NOK;
	}
	iq_cmd = (struct isp_tuning_cmd *)((cmd_data+31)&~31); // for cache alignment

	if(ccmd == 0) {
		iq_cmd->addr = ISP_TUNING_IQ_TABLE_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_IQ,iq_cmd);
	} else if(ccmd == 1) {
		iq_cmd->addr = ISP_TUNING_IQ_TABLE_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_SET_IQ,iq_cmd);
	} else if(ccmd == 2) {
		iq_cmd->addr = ISP_TUNING_STATIS_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_STATIS,iq_cmd);
	} else if(ccmd == 3) {
		iq_cmd->addr = ISP_TUNING_PARAM_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_PARAM,iq_cmd);
	} else if(ccmd == 4) {
		iq_cmd->addr = ISP_TUNING_PARAM_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_SET_PARAM,iq_cmd);
	} else if(ccmd == 5) {

		iq_cmd->addr = atoi(argv[1]);
		iq_cmd->len = atoi(argv[2]);
		hal_video_isp_tuning(VOE_ISP_TUNING_READ_VREG,iq_cmd);
		uint32_t *r_data = (uint32_t *)iq_cmd->data;
		for (int i=0; i<(iq_cmd->len/4); i++){
			printf("vreg 0x%08x = 0x%08x \n", iq_cmd->addr + i*4, r_data[i]);
	}

	} else if(ccmd == 6) {
		if (argc < 4 ) {
			printf("      6 offset lens value1 [value2]: rts_isp_tuning_write_vreg\r\n");
		} else {
			iq_cmd->addr = atoi(argv[1]);
			iq_cmd->len = atoi(argv[2]);

			uint32_t *w_data = (uint32_t *)iq_cmd->data;
			w_data[0] = atoi(argv[3]);
			if (argc >4) {
				for (int i=0; i<(argc-4); i++){
					w_data[i+1] = atoi(argv[4+i]);
				}
			}
			hal_video_isp_tuning(VOE_ISP_TUNING_WRITE_VREG,iq_cmd);
		}
	}

	if (cmd_data)
		free(cmd_data);
	return OK;
}

mm_module_t video_module = {
	.create = video_create,
	.destroy = video_destroy,
	.control = video_control,
	.handle = video_handle,

	.new_item = video_new_item,
	.del_item = video_del_item,
	.rsz_item = NULL,
	.vrelease_item = video_voe_release_item,

	.output_type = MM_TYPE_VDSP,    // output for video algorithm
	.module_type = MM_TYPE_VSRC,    // module type is video source
	.name = "VIDEO"
};
