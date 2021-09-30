//------------------------------------------------------
// yolo v3 tiny
//------------------------------------------------------
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "nn_api.h"
#include "mmf2_module.h"
#include "module_vipnn.h"
#include "hal_cache.h"


static float anchor_all[6][2] = {
	{81, 82}, {135, 169}, {344, 319},		// 13x13
	{10, 14}, {23, 27}, {37, 58},			// 26x26
};

static float u8_to_f(uint8_t val, uint8_t zero_p, float scale)
{
	return  scale * ((float)val - (float)zero_p);
}

static float s16_to_f(int16_t val, int q)
{
	return (float)val / (float)(1 << q);
}

static float bf16_to_f(__fp16 val)
{
	return (float)val;
}

static float q2f(void *val, nn_tensor_format_t *fmt)
{
	switch (fmt->type) {
	case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:
		// s16
		return s16_to_f(*(int16_t *)val, fmt->fix_point_pos);
	case VIP_BUFFER_QUANTIZE_TF_ASYMM:
		// u8
		return u8_to_f(*(uint8_t *)val, fmt->zero_point, fmt->scale);
	default:
		// bf16
		return bf16_to_f(*(__fp16 *)val);
	}
}

static uint8_t f_to_u8(float val, uint8_t zero_p, float scale)
{
	return (uint8_t)(val / scale) + zero_p;
}

static int16_t f_to_s16(float val, int q)
{
	return (int16_t)(val * (1 << q));
}

static __fp16 f_to_bf16(float val)
{
	return (__fp16)val;
}

static float sigmoid(float x)
{
	return 1 / (1 + exp(-x));
}

static float inv_sigmoid(float x)
{
	return -log((1 - x) / x);
}

#define FLASH_ADDR(x) (0x8000000+x)
void *yolov3t_get_network_binary(void)
{
	//return (void*)yolo_v3_tiny;
	//printf("NN model @ 0x09a000000\n\r");
	return (void *)(FLASH_ADDR(3 * 1024 * 1024) + 128);
	// return (void*)0x0aa00000;	// acuity yolov3t
}

int yolov3t_get_network_size(void)
{
	uint32_t *model = (uint32_t *)FLASH_ADDR(3 * 1024 * 1024);
	//return yolo_v3_tiny_length;
	//return 0x00979800;
	return model[0];
}

static int yolo_in_width, yolo_in_height;

int yolov3t_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param)
{
	void **tensor = (void **)tensor_in;
	rect_t *roi = &data_param->img.roi;

	cvImage img_in, img_out;

	img_in.width  = data_param->img.width;
	img_in.height = data_param->img.height;
	img_out.width  = tensor_param->dim[0].size[0];
	img_out.height = tensor_param->dim[0].size[1];
	yolo_in_width  = tensor_param->dim[0].size[0];
	yolo_in_height = tensor_param->dim[0].size[1];

	img_in.data   = (unsigned char *)data_in;
	img_out.data   = (unsigned char *)tensor[0];
	//printf("src %d %d, dst %d %d\n\r", img_in.width, img_in.height, img_out.width, img_out.height);
	//printf("roi %d %d %d %d \n\r", roi->xmin, roi->ymin, roi->xmax, roi->ymax);

	// resize src ROI area to dst
#if 1
	if (img_in.width == img_out.width && img_in.height == img_out.height) {
		memcpy(img_out.data, img_in.data, img_out.width * img_out.height * 3);
	} else {
		nn_resize_rgb888(&img_in, &img_out, roi, data_param->img.rgb);
	}

#endif
	//memcpy(img_out.data, img_in.data, img_out.width*img_out.height*3);
	dcache_clean_by_addr((uint32_t *)img_out.data, img_out.width * img_out.height * 3);
	//dcache_clean();		// if don't care other hardware dma, you can use this. ugly method, should clean dst address with its data size
	return 0;
}

typedef struct data_format_s {
	nn_tensor_format_t *format;
	nn_tensor_dim_t *dim;
} data_format_t;

static int yolo_get_offset(data_format_t *fmt, int i, int j, int n, int ei, int class_num)
{
	int w = fmt->dim->size[0];
	int h = fmt->dim->size[1];
	int es = 2;	// bf16, int16

	if (fmt->format->type == VIP_BUFFER_QUANTIZE_TF_ASYMM) {
		es = 1;
	}

	return (((5 + class_num) * n + ei) * h * w + w * j + i) * es;
}

static void *yolo_get_data_addr(void *data, data_format_t *fmt, int i, int j, int n, int ei, int class_num)
{
	int offset = yolo_get_offset(fmt, i, j, n, ei, class_num);
	uint8_t *data8 = (uint8_t *)data;

	return (void *)&data8[offset];
}

static float yolo_get_data(void *data, data_format_t *fmt, int i, int j, int n, int ei, int class_num)
{
	void *datax = (void *)yolo_get_data_addr(data, fmt, i, j, n, ei, class_num);
	return q2f(datax, fmt->format);
}

static int yolo_filter_class(void *data, data_format_t *fmt, int i, int j, int n, int *class_idx, float threshold, int class_num)
{
	int count = 0;
	switch (fmt->format->type) {
	case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT: {
		int16_t th16 = f_to_s16(threshold, fmt->format->fix_point_pos);
		for (int x = 0; x < class_num; x++) {
			int16_t tmp = *(int16_t *)yolo_get_data_addr(data, fmt, i, j, n, 5 + x, class_num);
			if (tmp > th16) {
				class_idx[count] = x;
				count++;
			}
		}
		break;
	}
	case VIP_BUFFER_QUANTIZE_TF_ASYMM: {
		uint8_t th8 = f_to_u8(threshold, fmt->format->zero_point, fmt->format->scale);
		for (int x = 0; x < class_num; x++) {
			uint8_t tmp = *(uint8_t *)yolo_get_data_addr(data, fmt, i, j, n, 5 + x, class_num);
			if (tmp > th8) {
				class_idx[count] = x;
				count++;
			}
		}
		break;
	}
	default: {
		__fp16 th16f = f_to_bf16(threshold);
		for (int x = 0; x < class_num; x++) {
			__fp16 tmp = *(__fp16 *)yolo_get_data_addr(data, fmt, i, j, n, 5 + x, class_num);
			if (tmp > th16f) {
				class_idx[count] = x;
				count++;
			}
		}
		break;
	}
	}

	//if(count > 0)WTF

	//	printf("count %d\n\r", count);

	return count;
}


typedef struct box_s {
	float prob;
	int invalid;
	float objectness;
	float class_prob;
	int class_idx;
	float x, y;
	float w, h;
} box_t;

#define MAX_BOX_CNT 64
static box_t res_box[MAX_BOX_CNT];
static box_t *p_res_box[MAX_BOX_CNT];
static int box_idx;

//-----------------------------NMS-----------------------------

float overlap(float x1, float w1, float x2, float w2)
{
	float l1 = x1 - w1 / 2;
	float l2 = x2 - w2 / 2;
	float left = l1 > l2 ? l1 : l2;
	float r1 = x1 + w1 / 2;
	float r2 = x2 + w2 / 2;
	float right = r1 < r2 ? r1 : r2;
	return right - left;
}

float box_intersection(box_t *a, box_t *b)
{
	float w = overlap(a->x, a->w, b->x, b->w);
	float h = overlap(a->y, a->h, b->y, b->h);
	if (w < 0 || h < 0) {
		return 0;
	}
	float area = w * h;
	return area;
}

float box_union(box_t *a, box_t *b)
{
	float i = box_intersection(a, b);
	float u = a->w * a->h + b->w * b->h - i;
	return u;
}

float box_iou(box_t *a, box_t *b)
{
	return box_intersection(a, b) / box_union(a, b);
}

int nms_comparator(const void *pa, const void *pb)
{
	box_t *a = (box_t *)pa;
	box_t *b = (box_t *)pb;
	float diff = a->prob - b->prob;
	if (diff < 0) {
		return 1;
	} else if (diff > 0) {
		return -1;
	}
	return 0;
}

//-----------------------------NMS-----------------------------



static int setup_class_box(int class, int total)
{
	int class_cnt = 0;
	for (int i = 0; i < total; i++) {
		if (res_box[i].class_idx == class) {
			p_res_box[class_cnt] = &res_box[i];
			class_cnt++;
		}
	}
	return class_cnt;
}

static void do_nms(int classes, int total, float threshold)
{
	int class_cnt;
	box_t *a, *b;
	for (int c = 0; c < classes; c++) {
		class_cnt = setup_class_box(c, total);
		qsort(p_res_box, class_cnt, sizeof(box_t *), nms_comparator);
		for (int i = 0; i < class_cnt; i++) {
			a = p_res_box[i];
			for (int j = i + 1; j < class_cnt; j++) {
				b = p_res_box[j];
				if (box_iou(a, b) > threshold) {
					b->invalid = 1;
				}
			}
		}
	}
}

static int __decode_yolo(data_format_t *fmt, void *out, int i, int j, int n, void *p_anchor)
{
	int max_i = 0;
	float cp = 0;
	int class_idx[80];
	int class_cnt = 0;
	int pass_cnt = 0;

	float (*anchor)[2] = (float (*)[2])p_anchor;

	//yolo_13_t *out13 = (yolo_13_t*)out;
	int grid_w = fmt->dim->size[0];
	int grid_h = fmt->dim->size[1];

	int classes = fmt->dim->size[2] / 3 - 5;

	float p = sigmoid(yolo_get_data(out, fmt, i, j, n, 4, classes));
	//int max_i = __find_max_index13(out, i, j, n, 80);

	class_cnt = yolo_filter_class(out, fmt, i, j, n, class_idx, inv_sigmoid(0.5), classes);

	for (int ci = 0; ci < class_cnt; ci++) {
		max_i = class_idx[ci];
		cp = sigmoid(yolo_get_data(out, fmt, i, j, n, 5 + max_i, classes));

		if (cp * p >= 0.5 && box_idx < MAX_BOX_CNT) {

			float tx = sigmoid(yolo_get_data(out, fmt, i, j, n, 0, classes));
			float ty = sigmoid(yolo_get_data(out, fmt, i, j, n, 1, classes));
			float tw = yolo_get_data(out, fmt, i, j, n, 2, classes);
			float th = yolo_get_data(out, fmt, i, j, n, 3, classes);

			float cx = (i + tx) / grid_w;
			float cy = (j + ty) / grid_h;
			float w = exp(tw) * anchor[n][0] / yolo_in_width;
			float h = exp(th) * anchor[n][1] / yolo_in_height;

			float x = cx - w / 2;
			float y = cy - h / 2;
			if (x < 0)	{
				x = 0;
			}
			if (x > 1)	{
				x = 1;
			}
			if (y < 0)	{
				y = 0;
			}
			if (y > 1) {
				y = 1;
			}

			//printf("x y w h = %f %f %f %f\n\r", x, y, w, h);
			//printf("x y w h = %3.0f %3.0f %3.0f %3.0f\n\r", x*yolo_in_width, y*yolo_in_height, w*yolo_in_width, h*yolo_in_height);
			//printf("tx %2.6f ty %2.6f tw %2.6f th %2.6f , p %2.6f, cp %2.6f, c %d, cn %d\n\r", tx, ty, tw, th, p, cp, max_i, class_cnt);

			res_box[box_idx].objectness = p;
			res_box[box_idx].class_prob = cp;
			res_box[box_idx].class_idx = max_i;
			res_box[box_idx].prob = cp * p;
			res_box[box_idx].x = x;
			res_box[box_idx].y = y;
			res_box[box_idx].w = w;
			res_box[box_idx].h = h;
			box_idx++;
			pass_cnt++;
		}
	}

	return pass_cnt;
}

static vipnn_res_t yolo_res;
void *yolov3t_postprocess(void *tensor_out, nn_tensor_param_t *param)
{
	void **tensor = (void **)tensor_out;

	data_format_t fmt;

	uint8_t *bb0 = (uint8_t *)tensor[0];
	fmt.format = &param->format[0];
	fmt.dim = &param->dim[0];
	int w = fmt.dim->size[0];
	int h = fmt.dim->size[1];

	int classes = fmt.dim->size[2] / 3 - 5;

	// reset box index
	box_idx = 0;
	memset(res_box, 0, sizeof(res_box));

	//printf("BB0 %d x %d\n\r", w, h);
	// 13x13x255(85x3)s
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			int ret = 0;
			ret += __decode_yolo(&fmt, bb0, i, j, 0, (void *)&anchor_all[0][0]);
			ret += __decode_yolo(&fmt, bb0, i, j, 1, (void *)&anchor_all[0][0]);
			ret += __decode_yolo(&fmt, bb0, i, j, 2, (void *)&anchor_all[0][0]);
			//if (ret > 0) {
			//	printf("%d %d\n\r-------\n\r", i, j);
			//}
		}
	}

	uint8_t *bb1 = (uint8_t *)tensor[1];
	fmt.format = &param->format[1];
	fmt.dim = &param->dim[1];
	w = fmt.dim->size[0];
	h = fmt.dim->size[1];

	//printf("BB1 %d x %d\n\r", w, h);

	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			int ret = 0;
			ret += __decode_yolo(&fmt, bb1, i, j, 0, (void *)&anchor_all[3][0]);
			ret += __decode_yolo(&fmt, bb1, i, j, 1, (void *)&anchor_all[3][0]);
			ret += __decode_yolo(&fmt, bb1, i, j, 2, (void *)&anchor_all[3][0]);
			//if (ret > 0) {
			//	printf("%d %d\n\r-------\n\r", i, j);
			//}
		}
	}

	do_nms(classes, box_idx, 0.3);

	// dump result
	/*
	for (int i = 0; i < box_idx; i++) {
		box_t *b = &res_box[i];
		printf("x y w h = %f %f %f %f\n\r", b->x, b->y, b->w, b->h);
		printf("x y w h = %3.0f %3.0f %3.0f %3.0f\n\r", b->x * yolo_in_width, b->y * yolo_in_height, b->w * yolo_in_width, b->h * yolo_in_height);
		printf("p %2.6f, obj %2.6f, cp %2.6f, class %d, invalid %d\n\r", b->prob, b->objectness, b->class_prob, b->class_idx, b->invalid);
	}
	*/
	yolo_res.od_res.obj_num  = 0;
	for (int i = 0; i < box_idx; i++) {
		box_t *b = &res_box[i];

		if (b->invalid == 0) {
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 0] = b->class_idx;
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 1] = b->prob;
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 2] = b->x;	// top_x
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 3] = b->y;	// top_y
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 4] = b->x + b->w; // bottom_x
			yolo_res.od_res.result[yolo_res.od_res.obj_num * 6 + 5] = b->y + b->h; // bottom_y
			yolo_res.od_res.obj_num++;
		}
	}

	return &yolo_res;
}

nnmodel_t yolov3_tiny = {
	.nb 		= yolov3t_get_network_binary,
	.nb_size 	= yolov3t_get_network_size,
	.preprocess 	= yolov3t_preprocess,
	.postprocess 	= yolov3t_postprocess,
};