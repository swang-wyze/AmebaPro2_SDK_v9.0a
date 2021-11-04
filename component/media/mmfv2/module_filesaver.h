#ifndef _MODULE_FILESAVER_H
#define _MODULE_FILESAVER_H

#include <FreeRTOS.h>
#include <freertos_service.h>
#include <task.h>
#include <stdint.h>

#include "module_vipnn.h"

#define CMD_FILESAVER_SET_PARAMS            MM_MODULE_CMD(0x00)  // set parameter
#define CMD_FILESAVER_GET_PARAMS            MM_MODULE_CMD(0x01)  // get parameter
#define CMD_FILESAVER_SET_IMG_IN_PARAMS     MM_MODULE_CMD(0x02)  // set data in parameter
#define CMD_FILESAVER_SET_SAVE_FILE_PATH    MM_MODULE_CMD(0x03)  // set save file path

#define CMD_FILESAVER_SET_NN_PARSER         MM_MODULE_CMD(0x10)  // set nn parse function

#define CMD_FILESAVER_APPLY                 MM_MODULE_CMD(0x20)  // for hardware module


typedef char *(*nn_get_json_res_t)(void *p, void *img_param, int frame_id, char *file_name);

typedef struct filesaver_param_s {
	uint32_t type;
	uint32_t codec_id;

	nn_data_param_t *img_in_param;

} filesaver_params_t;


typedef struct filesaver_ctx_s {
	void *parent;

	filesaver_params_t params;

	char sd_dataset_file_path_out[64];

	union {
		struct nn_parser_s {
			nn_get_json_res_t nn_get_json_res;
		} nn;
		struct audio_parser_s {
			int dummy;
		} audio;
		struct video_parser_s {
			int dummy;
		} video;
	} parser;

} filesaver_ctx_t;

extern mm_module_t filesaver_module;

#endif