/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <stdint.h>
#include "platform_stdlib.h"
#include "osdep_service.h"
#include "avcodec.h"
#include "mmf2_module.h"
#include "module_filesaver.h"

#include "ff.h"
#include <fatfs_ext/inc/ff_driver.h>
#include "sdio_combine.h"
#include "sdio_host.h"
#include <disk_if/inc/sdcard.h>
#include "fatfs_sdcard_api.h"

static FIL      m_file;
static fatfs_sd_params_t fatfs_sd;

static int SD_file_save_file(char *file_name, char *data_buf, int data_buf_size);

/*-----------------------------------------------------------------------------------*/

static int saver_count = 0;

int filesaver_handle(void *p, void *input, void *output)
{
	filesaver_ctx_t *ctx = (filesaver_ctx_t *)p;

	mm_queue_item_t *input_item = (mm_queue_item_t *)input;

	char sd_fn_out[64];
	memset(&sd_fn_out[0], 0x00, sizeof(sd_fn_out));

	if (input_item->type == AV_CODEC_ID_MP4A_LATM) {
		/* process AAC input_item here */
		snprintf(sd_fn_out, sizeof(sd_fn_out), "%s_%d.aac", ctx->sd_dataset_file_path_out, saver_count + 1);
		SD_file_save_file(sd_fn_out, input_item->data_addr, input_item->size);
	} else if (input_item->type == AV_CODEC_ID_H264) {
		/* process H264 input_item here */
		snprintf(sd_fn_out, sizeof(sd_fn_out), "%s_%d.h264", ctx->sd_dataset_file_path_out, saver_count + 1);
		SD_file_save_file(sd_fn_out, input_item->data_addr, input_item->size);
	} else if (input_item->type == AV_CODEC_ID_NN_RAW) {
		VIPNN_OUT_BUFFER pre_tensor_out;
		memcpy(&pre_tensor_out, input_item->data_addr, input_item->size);

		/* save yolo json result */
		snprintf(sd_fn_out, sizeof(sd_fn_out), "%s_%d.json", ctx->sd_dataset_file_path_out, saver_count + 1);
		if (ctx->parser.nn.nn_get_json_res) {
			char *json_format_out = ctx->parser.nn.nn_get_json_res(&pre_tensor_out.vipnn_res, ctx->params.img_in_param, saver_count + 1, sd_fn_out);
			//printf("\r\njson_format_out: %s\r\n", json_format_out);
			SD_file_save_file(sd_fn_out, json_format_out, strlen(json_format_out));
		}

		/* save tensor */
		for (int i = 0; i < pre_tensor_out.vipnn_out_tensor_num; i++) {
			/* save raw tensor */
			memset(&sd_fn_out[0], 0x00, sizeof(sd_fn_out));
			snprintf(sd_fn_out, sizeof(sd_fn_out), "%s_out_tensor%d_uint8_%d.bin", ctx->sd_dataset_file_path_out, i, saver_count + 1);
			SD_file_save_file(sd_fn_out, (char *)pre_tensor_out.vipnn_out_tensor[i], pre_tensor_out.vipnn_out_tensor_size[i]); /* raw tensor*/

#if 0
			/* save float32 tensor */
			memset(&sd_fn_out[0], 0x00, sizeof(sd_fn_out));
			snprintf(sd_fn_out, sizeof(sd_fn_out), "%s_out_tensor%d_float32_%d.bin", ctx->sd_dataset_file_path_out, i, saver_count + 1);
			float *float_tensor;
			switch (pre_tensor_out.quant_format[i]) {
			case VIP_BUFFER_QUANTIZE_TF_ASYMM:   /* uint8 --> float32 */
				float_tensor = (float *)malloc(pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float));
				for (int k = 0; k < pre_tensor_out.vipnn_out_tensor_size[i]; k++) {
					float_tensor[k] = (*((uint8_t *)pre_tensor_out.vipnn_out_tensor[i] + k) - pre_tensor_out.quant_data[i].affine.zeroPoint) *
									  pre_tensor_out.quant_data[i].affine.scale;
				}
				SD_file_save_file(sd_fn_out, (char *)float_tensor, pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float));
				break;
			case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:   /* int16 --> float32 */
				float_tensor = (float *)malloc(pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float) / sizeof(int16_t));
				for (int k = 0; k < (pre_tensor_out.vipnn_out_tensor_size[i] / sizeof(int16_t)); k++) {
					float_tensor[k] = (float)(*((int16_t *)pre_tensor_out.vipnn_out_tensor[i] + k)) / ((float)(1 << pre_tensor_out.quant_data[i].dfp.fixed_point_pos));
				}
				SD_file_save_file(sd_fn_out, (char *)float_tensor, pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float) / sizeof(int16_t));
				break;
			default:   /* float16 --> float32 */
				float_tensor = (float *)malloc(pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float) / sizeof(__fp16));
				for (int k = 0; k < (pre_tensor_out.vipnn_out_tensor_size[i] / sizeof(__fp16)); k++) {
					float_tensor[k] = (float)(*((__fp16 *)pre_tensor_out.vipnn_out_tensor[i] + k));
				}
				SD_file_save_file(sd_fn_out, (char *)float_tensor, pre_tensor_out.vipnn_out_tensor_size[i] * sizeof(float) / sizeof(__fp16));
			}
			free(float_tensor);
#endif
		}
	}

	saver_count++;

	return 0;
}

static int SD_file_save_file(char *file_name, char *data_buf, int data_buf_size)
{
	int bw;
	FRESULT res;
	char path_all[64];

	snprintf(path_all, sizeof(path_all), "%s%s", fatfs_sd.drv, file_name);

	res = f_open(&m_file, path_all, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (res) {
		printf("open file (%s) fail.\n", file_name);
		return 0;
	}

	printf("\r\nfile name: %s", file_name);

	char *WRBuf = (char *)malloc(data_buf_size);
	memset(WRBuf, 0x00, data_buf_size);
	memcpy(WRBuf, data_buf, data_buf_size);

	do {
		res = f_write(&m_file, WRBuf, data_buf_size, (u32 *)&bw);
		if (res) {
			f_lseek(&m_file, 0);
			printf("Write error.\n");
			return 0;
		}
		printf("\r\nWrite %d bytes.\n", bw);
	} while (bw < data_buf_size);

	free(WRBuf);

	f_lseek(&m_file, 0);
	if (f_close(&m_file)) {
		printf("close file (%s) fail.\n", file_name);
		return 0;
	}
	printf("\r\close file (%s) done.\n\n\r", file_name);

	return 1;
}

/*-----------------------------------------------------------------------------------*/

int filesaver_control(void *p, int cmd, int arg)
{
	filesaver_ctx_t *ctx = (filesaver_ctx_t *)p;

	switch (cmd) {
	case CMD_FILESAVER_SET_PARAMS:
		memcpy(&ctx->params, (void *)arg, sizeof(filesaver_params_t));
		break;
	case CMD_FILESAVER_GET_PARAMS:
		memcpy((void *)arg, &ctx->params, sizeof(filesaver_params_t));
		break;
	case CMD_FILESAVER_SET_IMG_IN_PARAMS:
		ctx->params.img_in_param = (nn_data_param_t *)arg;
		break;
	case CMD_FILESAVER_SET_SAVE_FILE_PATH:
		memset(ctx->sd_dataset_file_path_out, 0x00, sizeof(ctx->sd_dataset_file_path_out));
		memcpy((char *)ctx->sd_dataset_file_path_out, (char *)arg, strlen((char *)arg));
		break;
	case CMD_FILESAVER_SET_NN_PARSER:
		ctx->parser.nn.nn_get_json_res = (nn_get_json_res_t *)arg;
		break;
	case CMD_FILESAVER_APPLY:
		// do nothing
		break;
	}
	return 0;
}

void *filesaver_destroy(void *p)
{
	filesaver_ctx_t *ctx = (filesaver_ctx_t *)p;

	if (ctx) {
		free(ctx);
	}

	fatfs_sd_close();

	return NULL;
}

void *filesaver_create(void *parent)
{
	filesaver_ctx_t *ctx = malloc(sizeof(filesaver_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(filesaver_ctx_t));

	ctx->parent = parent;

	if (fatfs_sd_is_inited() == 0) {
		FRESULT res = fatfs_sd_init();
		if (res < 0) {
			printf("fatfs_sd_init fail (%d)\n", res);
			return NULL;
		}
	}
	fatfs_sd_get_param(&fatfs_sd);

	return ctx;
}

mm_module_t filesaver_module = {
	.create = filesaver_create,
	.destroy = filesaver_destroy,
	.control = filesaver_control,
	.handle = filesaver_handle,

	.new_item = NULL,
	.del_item = NULL,

	.output_type = MM_TYPE_NONE,
	.module_type = MM_TYPE_AVSINK,
	.name = "FILE_SAVER"
};
