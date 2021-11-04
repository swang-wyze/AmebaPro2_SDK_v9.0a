/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"

#include "module_vipnn.h"
#include "module_fileloader.h"
#include "module_filesaver.h"

#include "avcodec.h"
#include <cJSON.h>

#include "model_yolov3t.h"

#define NUMBER_OF_COCO_CLASS        80
#define MAX_COCO_NAME_STRING_SIZE   20
static char coco_name[NUMBER_OF_COCO_CLASS][MAX_COCO_NAME_STRING_SIZE] =
{   "person",    "bicycle",    "car",    "motorbike",    "aeroplane",    "bus",    "train",    "truck",    "boat",    "traffic light",    "fire hydrant",    "stop sign",    "parking meter",    "bench",    "bird",    "cat",    "dog",    "horse",    "sheep",    "cow",    "elephant",    "bear",    "zebra",    "giraffe",    "backpack",    "umbrella",    "handbag",    "tie",    "suitcase",    "frisbee",    "skis",    "snowboard",    "sports ball",    "kite",    "baseball bat",    "baseball glove",    "skateboard",    "surfboard",    "tennis racket",    "bottle",    "wine glass",    "cup",    "fork",    "knife",    "spoon",    "bowl",    "banana",    "apple",    "sandwich",    "orange",    "broccoli",    "carrot",    "hot dog",    "pizza",    "donut",    "cake",    "chair",    "sofa",    "pottedplant",    "bed",    "diningtable",    "toilet",    "tvmonitor",    "laptop",    "mouse",    "remote",    "keyboard",    "cell phone",    "microwave",    "oven",    "toaster",    "sink",    "refrigerator",    "book",    "clock",    "vase",    "scissors",    "teddy bear",    "hair drier",    "toothbrush"};

#define TEST_IMAGE_WIDTH	416   /* Fix me */
#define TEST_IMAGE_HEIGHT	416   /* Fix me */
nn_data_param_t roi_tester = {
	.img = {
		.width = TEST_IMAGE_WIDTH,
		.height = TEST_IMAGE_HEIGHT,
		.rgb = 1,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = TEST_IMAGE_WIDTH,
			.ymax = TEST_IMAGE_HEIGHT,
		}
	}
};

static fileloader_params_t test_image_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_BMP24,
	.test_data_width = TEST_IMAGE_WIDTH,
	.test_data_height = TEST_IMAGE_HEIGHT
};

static int BMP24toRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size);
static char *nn_get_object_json_format(void *p, void *img_param, int frame_id, char *file_name);

static mm_context_t *fileloader_ctx			= NULL;
static mm_context_t *filesaver_ctx			= NULL;
static mm_context_t *vipnn_ctx              = NULL;

static mm_siso_t *siso_fileloader_vipnn     = NULL;
static mm_siso_t *siso_vipnn_filesaver      = NULL;

#define TEST_FILE_PATH_PREFIX   "test_bmp/image"            /* Fix me */   // test_bmp/image-0001.bmp, test_bmp/image-0002.bmp, test_bmp/image-0003.bmp ...
#define TEST_FILE_NUM           10                          /* Fix me */

#define FILE_OUT_PATH_PREFIX    "yolo_result/yolo_result"   /* Fix me */

void mmf2_example_file_vipnn_tester(void)
{

	fileloader_ctx = mm_module_open(&fileloader_module);
	if (fileloader_ctx) {
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_PARAMS, (int)&test_image_params);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_TEST_FILE_PATH, (int)TEST_FILE_PATH_PREFIX);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILE_NUM, (int)TEST_FILE_NUM);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_DECODE_PROCESS, (int)BMP24toRGB888planar_ConvertInPlace);
		mm_module_ctrl(fileloader_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(fileloader_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_APPLY, 0);
	} else {
		rt_printf("fileloader open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	rt_printf("fileloader opened\n\r");

	// VIPNN
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&yolov3_tiny);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_tester);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, NULL);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_OUTPUT, 1);  //enable module output
		mm_module_ctrl(vipnn_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(vipnn_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		rt_printf("VIPNN open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	rt_printf("VIPNN opened\n\r");

	filesaver_ctx = mm_module_open(&filesaver_module);
	if (filesaver_ctx) {
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_IMG_IN_PARAMS, (int)&roi_tester);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)FILE_OUT_PATH_PREFIX);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_NN_PARSER, (int)nn_get_object_json_format);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_APPLY, 0);
	} else {
		rt_printf("filesaver open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	rt_printf("filesaver opened\n\r");

	//--------------Link---------------------------

	siso_fileloader_vipnn = siso_create();
	if (siso_fileloader_vipnn) {
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)fileloader_ctx, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_fileloader_vipnn);
	} else {
		rt_printf("siso_fileloader_vipnn open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	rt_printf("siso_fileloader_vipnn started\n\r");

	siso_vipnn_filesaver = siso_create();
	if (siso_vipnn_filesaver) {
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)vipnn_ctx, 0);
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_start(siso_vipnn_filesaver);
	} else {
		rt_printf("siso_vipnn_filesaver open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	rt_printf("siso_vipnn_filesaver started\n\r");

	return;
mmf2_example_file_vipnn_tester_fail:

	return;
}

/*-----------------------------------------------------------------------------------*/

static int BMP24toRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size)
{
	char *bmp2rgb_buffer = (char *)pbuffer;
	uint32_t *bmp2rgb_size = (uint32_t *)pbuffer_size;

	int data_offset;
	memcpy(&data_offset, &bmp2rgb_buffer[10], sizeof(int));
	printf("\r\nstart offset of data: %d \n", data_offset);

	int data_size = *bmp2rgb_size - data_offset;
	printf("data_size: %d\n ", data_size);

	int test_im_w, test_im_h;
	memcpy(&test_im_w, &bmp2rgb_buffer[18], sizeof(int));
	memcpy(&test_im_h, &bmp2rgb_buffer[22], sizeof(int));

	printf("\r\nbmp file info: w:%d, h:%d, %s\r\n", test_im_w, test_im_h, (test_im_h > 0 ? "Bottom2Top" : "Top2Bottom"));

	char *top_down_data_buf = (char *)malloc(data_size);
	if (test_im_h > 0) { /* Bottom2Top */
		for (int i = 0; i < test_im_h; i++) {
			memcpy(&top_down_data_buf[ i * test_im_w * 3 ], &bmp2rgb_buffer[data_offset + (test_im_h - 1 - i)*test_im_w * 3], test_im_w * 3);
		}
	} else { /* Top2Bottom */
		memcpy(&top_down_data_buf[0], &bmp2rgb_buffer[data_offset], data_size);
	}

	char *bgr_planar_buf = (char *)malloc(data_size);
	for (int i = 0; i < test_im_w * test_im_h; i++) {
		bgr_planar_buf[i] = top_down_data_buf[i * 3];
		bgr_planar_buf[test_im_w * test_im_h + i] = top_down_data_buf[i * 3 + 1];
		bgr_planar_buf[2 * test_im_w * test_im_h + i] = top_down_data_buf[i * 3 + 2];
	}
	free(top_down_data_buf);

	char *rgb_planar_buf = (char *)malloc(data_size);
	memcpy(&rgb_planar_buf[0], &bgr_planar_buf[2 * test_im_w * test_im_h], test_im_w * test_im_h);
	memcpy(&rgb_planar_buf[test_im_w * test_im_h], &bgr_planar_buf[test_im_w * test_im_h], test_im_w * test_im_h);
	memcpy(&rgb_planar_buf[2 * test_im_w * test_im_h], &bgr_planar_buf[0], test_im_w * test_im_h);
	free(bgr_planar_buf);

	memcpy(bmp2rgb_buffer, rgb_planar_buf, data_size);
	*bmp2rgb_size = (uint32_t) data_size;
	free(rgb_planar_buf);

	return 1;
}

static char *nn_get_object_json_format(void *p, void *img_param, int frame_id, char *file_name)
{
	if (!p || !img_param) {
		return;
	}

	objdetect_res_t *res = (objdetect_res_t *)p;
	nn_data_param_t *im = (nn_data_param_t *)img_param;

	/**** cJSON ****/
	cJSON_Hooks memoryHook;
	memoryHook.malloc_fn = malloc;
	memoryHook.free_fn = free;
	cJSON_InitHooks(&memoryHook);

	cJSON *YoloJSObject = NULL, *yolo_obj_JSObject = NULL;
	char *yolo_json = NULL;
	cJSON *yolo_coor_JSObject = NULL, *yolo_obj_JSArray = NULL;

	YoloJSObject = cJSON_CreateObject();
	cJSON_AddItemToObject(YoloJSObject, "frame_id", cJSON_CreateNumber(frame_id));
	cJSON_AddItemToObject(YoloJSObject, "filename", cJSON_CreateString(file_name));
	cJSON_AddItemToObject(YoloJSObject, "objects", yolo_obj_JSArray = cJSON_CreateArray());

	int im_w = im->img.width;
	int im_h = im->img.height;

	printf("object num = %d\r\n", res->obj_num);
	if (res->obj_num > 0) {
		for (int i = 0; i < res->obj_num; i++) {

			int top_x = (int)(res->result[6 * i + 2] * im_w) < 0 ? 0 : (int)(res->result[6 * i + 2] * im_w);
			int top_y = (int)(res->result[6 * i + 3] * im_h) < 0 ? 0 : (int)(res->result[6 * i + 3] * im_h);
			int bottom_x = (int)(res->result[6 * i + 4] * im_w) > im_w ? im_w : (int)(res->result[6 * i + 4] * im_w);
			int bottom_y = (int)(res->result[6 * i + 5] * im_h) > im_h ? im_h : (int)(res->result[6 * i + 5] * im_h);

			printf("%d,c%d:%d %d %d %d\n\r", i, (int)(res->result[6 * i]), top_x, top_y, bottom_x, bottom_y);

			cJSON_AddItemToArray(yolo_obj_JSArray, yolo_obj_JSObject = cJSON_CreateObject());
			cJSON_AddItemToObject(yolo_obj_JSObject, "class_id", cJSON_CreateNumber((int)res->result[6 * i ]));
			cJSON_AddItemToObject(yolo_obj_JSObject, "name", cJSON_CreateString(coco_name[(int)res->result[6 * i ]]));

			cJSON_AddItemToObject(yolo_obj_JSObject, "relative_coordinates", yolo_coor_JSObject = cJSON_CreateObject());
			cJSON_AddItemToObject(yolo_coor_JSObject, "top_x", cJSON_CreateNumber(top_x));
			cJSON_AddItemToObject(yolo_coor_JSObject, "top_y", cJSON_CreateNumber(top_y));
			cJSON_AddItemToObject(yolo_coor_JSObject, "bottom_x", cJSON_CreateNumber(bottom_x));
			cJSON_AddItemToObject(yolo_coor_JSObject, "bottom_y", cJSON_CreateNumber(bottom_y));

			cJSON_AddItemToObject(yolo_obj_JSObject, "probability", cJSON_CreateNumber((float)res->result[6 * i + 1]));
		}
	}

	yolo_json = cJSON_Print(YoloJSObject);
	cJSON_Delete(YoloJSObject);
	return yolo_json;
}
