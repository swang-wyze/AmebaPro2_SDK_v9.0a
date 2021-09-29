#ifndef _MODULE_VIDEO_H
#define _MODULE_VIDEO_H

#include <stdint.h>
#include <osdep_service.h>
#include "mmf2_module.h"
#include "hal_video.h"

#define CMD_VIDEO_SET_PARAMS     	MM_MODULE_CMD(0x00)  // set parameter
#define CMD_VIDEO_GET_PARAMS     	MM_MODULE_CMD(0x01)  // get parameter
#define CMD_VIDEO_SET_HEIGHT		MM_MODULE_CMD(0x02)
#define CMD_VIDEO_SET_WIDTH			MM_MODULE_CMD(0x03)
#define CMD_VIDEO_BITRATE			MM_MODULE_CMD(0x04)
#define CMD_VIDEO_FPS				MM_MODULE_CMD(0x05)
#define CMD_VIDEO_GOP				MM_MODULE_CMD(0x06)
#define CMD_VIDEO_MEMORY_SIZE		MM_MODULE_CMD(0x07)
#define CMD_VIDEO_BLOCK_SIZE		MM_MODULE_CMD(0x08)
#define CMD_VIDEO_MAX_FRAME_SIZE	MM_MODULE_CMD(0x09)
#define CMD_VIDEO_RCMODE			MM_MODULE_CMD(0x0a)
#define CMD_VIDEO_SET_RCPARAM		MM_MODULE_CMD(0x0b)
#define CMD_VIDEO_GET_RCPARAM		MM_MODULE_CMD(0x0c)
#define CMD_VIDEO_INIT_MEM_POOL		MM_MODULE_CMD(0x0d)
#define CMD_VIDEO_FORCE_IFRAME		MM_MODULE_CMD(0x0e)
#define CMD_VIDEO_SET_RCADVPARAM	MM_MODULE_CMD(0x10)
#define CMD_VIDEO_GET_RCADVPARAM	MM_MODULE_CMD(0x11)
#define CMD_VIDEO_SET_ROIPARM		MM_MODULE_CMD(0x12)
#define CMD_VIDEO_SET_ROI			MM_MODULE_CMD(0x13)
#define CMD_VIDEO_SET_QPCHROMA_OFFSET    MM_MODULE_CMD(0x14)
#define CMD_VIDEO_SET_FORCE_DROP_FRAME   MM_MODULE_CMD(0x15)

#define CMD_SNAPSHOT_ENCODE_CB		MM_MODULE_CMD(0x30)


#define CMD_VIDEO_STREAMID		    MM_MODULE_CMD(0x16)
#define CMD_VIDEO_FORMAT			MM_MODULE_CMD(0x17)
#define CMD_VIDEO_DUMP_STATE		MM_MODULE_CMD(0x18)
#define CMD_VIDEO_BPS               MM_MODULE_CMD(0x19)
#define CMD_VIDEO_SNAPSHOT          MM_MODULE_CMD(0x1a)
#define CMD_VIDEO_SNAPSHOT_CB       MM_MODULE_CMD(0x1b)
#define CMD_VIDEO_YUV               MM_MODULE_CMD(0x1c)
#define CMD_VIDEO_PRINT_INFO        MM_MODULE_CMD(0x1d)


#define CMD_VIDEO_APPLY				MM_MODULE_CMD(0x20)  // apply setting
#define CMD_VIDEO_UPDATE			MM_MODULE_CMD(0x21)  // update new setting
#define CMD_VIDEO_STREAM_START		MM_MODULE_CMD(0x22)  // start stream
#define CMD_VIDEO_STREAM_STOP		MM_MODULE_CMD(0x23)  // stop stream
#define CMD_VIDEO_SET_VOE_HEAP      MM_MODULE_CMD(0x24)


typedef struct encode_params_s {
	uint32_t width;
	uint32_t height;
	uint32_t bps;
	uint32_t fps;
	uint32_t gop;
	uint32_t rc_mode;
	uint32_t auto_qp;			// 1 : enable auto QP, discard qpMin, qpMax
	uint32_t rst_rcerr;			// 1 : reset rc error when rate change
	uint32_t rotation;          // 0 : Rotate 0 1:Rotate 90R 2: Rorate 90L
	uint32_t mem_total_size;
	uint32_t mem_block_size;
	uint32_t mem_frame_size;
} encode_params_t;

typedef struct encode_rc_parm_s {
	unsigned int rcMode;
	unsigned int iQp;		// for fixed QP
	unsigned int pQp;		// for fixed QP
	unsigned int minQp;		// for CBR/VBR
	unsigned int minIQp;	// for CBR/VBR
	unsigned int maxQp;		// for CBR/VBR
} encode_rc_parm_t;

typedef struct encode_rc_adv_parm_s {
	unsigned int rc_adv_enable;
	unsigned int maxBps;		// for VBR
	unsigned int minBps;		// for VBR

	int intraQpDelta;
	int mbQpAdjustment;
	unsigned int mbQpAutoBoost;
} encode_rc_adv_parm_t;

typedef struct encode_roi_parm_s {
	unsigned int enable;
	unsigned int left;		// for fixed QP
	unsigned int right;		// for fixed QP
	unsigned int top;		// for CBR/VBR
	unsigned int bottom;	// for CBR/VBR
} encode_roi_parm_t;


typedef struct video_state_s {
	uint32_t timer_1;
	uint32_t timer_2;
	uint32_t drop_frame;
} video_state_t;


//		printf("vs ch type [resolution] [fps]\n");
//		printf("      ch         : ISP channel 0/1/2:HEVC/JPEG/NV12 4:only RGB \n");
//		printf("      type       : 0:HEVC 1:H264 2:JPEG 3:NV12 4:RGB 9:Disable\n");
//		printf("      resolution : 0:CIF 1:QCIF 2:VGA 3:HD 4:FHD\n");
//		printf("      fps        : set output frame rate\n");
typedef struct video_param_s {
	uint32_t stream_id;
	uint32_t type;
	uint32_t resolution;
	uint32_t width;
	uint32_t height;
	uint32_t bps;
	uint32_t fps;
	uint32_t gop;
	uint32_t rc_mode;
	//uint32_t format;
	//uint32_t boot_mode;
	//uint32_t bayer_type;
	uint32_t out_buf_size;
	uint32_t out_rsvd_size;
	//uint32_t voe_heap_size;
	uint32_t direct_output;
	uint32_t use_static_addr;
} video_params_t;

typedef struct video_ctx_s {
	void *parent;

	hal_video_adapter_t *v_adp;
	void *mem_pool;

	video_params_t params;
	int (*snapshot_cb)(uint32_t, uint32_t);
	void (*change_parm_cb)(void *);
	video_state_t state;

} video_ctx_t;

extern mm_module_t video_module;

#endif