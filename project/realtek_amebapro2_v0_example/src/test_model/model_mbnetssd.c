//-----------------------------------------------------------------
// mobilenet ssd model
//-----------------------------------------------------------------
#include <stdio.h>
#include "nn_api.h"
#include "mmf2_module.h"
#include "module_vipnn.h"
#include "hal_cache.h"

#include "mobilenet_ssd_uint8.h"
#include "mmf2_pro2_video_config.h"

#include "ssd_post_process.h"
#include "hal_video.h"

void *mbnetssd_get_network_binary(void)
{
	return (void *)mobilenet_ssd_uint8;
}

int mbnetssd_get_network_size(void)
{
	return mobilenet_ssd_uint8_size;
}

int mbnetssd_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param)
{
	void **tensor = (void **)tensor_in;
	struct cvRect_S roi = {
		.xmin = data_param->img.roi.xmin,
		.ymin = data_param->img.roi.ymin,
		.xmax = data_param->img.roi.xmax,
		.ymax = data_param->img.roi.ymax,
	};

	cvImage img_in, img_out;

	img_in.width  = data_param->img.width;
	img_in.height = data_param->img.height;
	img_out.width  = tensor_param->dim[0].size[0];
	img_out.height = tensor_param->dim[0].size[1];

	img_in.data   = (unsigned char *)data_in;
	img_out.data   = (unsigned char *)tensor[0];

	//printf("preproc w %d h %d ow %d oh %d\n\r", img_in.width, img_in.height, img_out.width, img_out.height);
	//printf("preproc %x, %x\n\r", img_in.data, img_out.data);

	// resize src ROI area to dst
	nn_resize_rgb888(&img_in, &img_out, &roi, data_param->img.rgb);
	//memcpy(dst->data, src->data, 416*416*3);
	//printf("preprocess finish\n\r");
	dcache_clean_by_addr((uint32_t *)img_out.data, img_out.width * img_out.height * 3);
	//dcache_clean();		// if don't care other hardware dma, you can use this. ugly method, should clean dst address with its data size
	return 0;
}

static vipnn_res_t ssd_res;
void *mbnetssd_postprocess(void *tensor_out, nn_tensor_param_t *param)
{
	void **tensor = (void **)tensor_out;

	vipnn_res_t *res = &ssd_res;
	//float result[6 * 21];
	//int obj_num;

	nn_post_process(0,
					(unsigned char *)tensor[0],  //loc
					(unsigned char *)tensor[1], 	//scores
					res->od_res.result, &res->od_res.obj_num);

	//printf("obj num %d \n\r", res->od_res.obj_num);
	nn_display_results(0, res->od_res.result, &res->od_res.obj_num);

	return (void *)res;
}

nnmodel_t mbnetssd = {
	.nb 		= mbnetssd_get_network_binary,
	.nb_size 	= mbnetssd_get_network_size,
	.preprocess 	= mbnetssd_preprocess,
	.postprocess 	= mbnetssd_postprocess,
};
