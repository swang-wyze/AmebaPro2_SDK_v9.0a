/**************************************************************************//**
 * @file     hal_video.h
 * @brief    The HAL API implementation for the Video device.
 * @version  V1.00
 * @date     2021-01-14
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/


#ifndef _HAL_VIDEO_H_
#define _HAL_VIDEO_H_


#if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9
#include "platform_stdlib.h"
#include "cmsis.h"
#include "hal.h"
#include "hal_voe.h"

#include "base_type.h"
#if (CONFIG_VERIFY_ENC == 0)
#include "hal_video_common.h"
#else
#include "instance.h"
#include "EncJpegInstance.h"
#endif

#include "rtl8735b_voe_type.h"
#include "rtl8735b_voe_cmd.h"

// converter LF --> CRLF
#define printf(fmt, arg...)				printf(fmt"\r", ##arg)

#else	// Run on VOE

#include "instance.h"
#include "EncJpegInstance.h"

#endif

#define DEBUG_ERROR      0
//#define _ISP_ON_TM

//#define __TTFF__

#include "hal_isp.h"



#ifdef  __cplusplus
extern "C"
{
#endif

#define MAX_PATH				64
#define DEFAULT					-255
#define MAX_BPS_ADJUST			20
#define MAX_SCENE_CHANGE		20
#define MAX_ENC_BUF				16
#define USER_DEFINED_QTABLE		10

#define MOVING_AVERAGE_FRAMES	120
#define MAX_CHANNEL 			5					// Channel 4 is RGB only
#define MAX_OBJECT				10

#define CODEC_HEVC				(ENABLE<<VCENC_VIDEO_CODEC_HEVC)	// 0 0x01
#define CODEC_H264				(ENABLE<<VCENC_VIDEO_CODEC_H264)	// 1 0x02
#define CODEC_JPEG				(ENABLE<<VCENC_VIDEO_CODEC_JPEG)	// 2 0x04
#define CODEC_NV12				(ENABLE<<VCENC_VIDEO_CODEC_NV12)	// 3 0x08
#define CODEC_RGB				(ENABLE<<VCENC_VIDEO_CODEC_RGB)		// 4 0x10
#define CODEC_NV16				(ENABLE<<VCENC_VIDEO_CODEC_NV16)	// 5 0x20

#define JPEG_DISABLE			0
#define JPEG_SNAPSHOT			1
#define JPEG_ENABLE				2

/**
 * @addtogroup hal_enc Encoder
 * @ingroup 8735b_hal
 * @{
 * @brief The Encoder HAL module.
 */

/*  Macros for Encoder module system related configuration  */
/** @defgroup GROUP_ENC_MODULE_SYSTEM_CONFIGURATION ENC SYSTEM CONFIGURATION
 *  enc system related configuration
 *  @{
 */


// enc2out->finish define
enum {
	NON_FRAME		= 0,
	FIRST_FRAME		= 2,
	NORMAL_FRAME	= 5,
	LAST_FRAME		= 9,
	FINISH		    = 10,
	FAIL_FRAME		= -1,
};

enum isp_tuning_iq_type {
	ISP_TUNING_IQ_TABLE_ALL,
	ISP_TUNING_IQ_TABLE_BLC,
	ISP_TUNING_IQ_TABLE_NLSC,
	ISP_TUNING_IQ_TABLE_MLSC,
	ISP_TUNING_IQ_TABLE_AE,
	ISP_TUNING_IQ_TABLE_AWB,
	ISP_TUNING_IQ_TABLE_AF,
	ISP_TUNING_IQ_TABLE_CCM,
	ISP_TUNING_IQ_TABLE_GAMMA,
	ISP_TUNING_IQ_TABLE_WDR,
	ISP_TUNING_IQ_TABLE_YGC,
	ISP_TUNING_IQ_TABLE_UVTUNE,
	ISP_TUNING_IQ_TABLE_SPE,
	ISP_TUNING_IQ_TABLE_TEXTURE,
	ISP_TUNING_IQ_TABLE_DAYNIGHT,
	ISP_TUNING_IQ_TABLE_HIGH_TEMP,
	_ISP_TUNING_IQ_TABLE_NUM
};

enum isp_tuning_statis_type {
	ISP_TUNING_STATIS_ALL,
	ISP_TUNING_STATIS_RAW,
	ISP_TUNING_STATIS_AWB,
	ISP_TUNING_STATIS_AE,
	ISP_TUNING_STATIS_AF,
	ISP_TUNING_STATIS_FLICK,
	ISP_TUNING_STATIS_Y,
	_ISP_TUNING_STATIS_NUM,
};

enum isp_tuning_param_type {
	ISP_TUNING_PARAM_ALL,
	ISP_TUNING_PARAM_AE,
	ISP_TUNING_PARAM_AWB,
	ISP_TUNING_PARAM_FLICK,
	ISP_TUNING_PARAM_WDR,
	_ISP_TUNING_PARAM_NUM,
};

#define IQ_CMD_DATA_SIZE 65536

typedef void (*output_callback_t)(void *, void *, uint32_t);



typedef struct {
	int width;				// output width
	int height;				// output height
	int codec;				// Codec Type Bit0:HEVC Bit1:h264 Bit2:JPEG
							//			  Bit3:NV12 Bit4:RGB  Bit5: ISP_DONE
	int ch;

	u32 enc_len;			// output bit stream size
	u32 jpg_len;			// output bit stream size
	int *enc_addr;			// output bit stream address
	int *jpg_addr;			// output bit stream address
	int *isp_addr;			// output isp address
	int qp;					// output frame Encoder QP value
	int finish;				// output is laster frame
	int type;				// output type I/P frame	//VCENC_INTRA_FRAME/INTER_FRAME ...
	void *queue;			// xQueueHandle Enc to out message queue
	int pre_time;			// pre-process time
	int enc_time;			// HW encode time
	int post_time;			// post-process time
	int cmd;
	int cmd_status;
}enc2out_t;



typedef struct {
	// ringbuffer
	int	out_size;			// out_buf_size
	int	rsvd_size;			// rsvd_size
	int	put_addr;			// ring buffer put pointer address
	int	get_addr;			// ring buffer get pointer address
	int	size_used;			// ring buffer used size
	int	slot_used;			// ring buffer used slot (frame)
	int	*out_buf;			// start buffer address offset
	int	*rsvd_buf;			// reserved buffer address offset
	void *queue;
} hal_video_buf_s;

typedef struct {
	// OSD SW workaround
	int width;
	int height;
	int table_size;

} hal_video_osd_s;

typedef struct {
	// ROI
	int	type;				// ROI type
	int *roi_table;			// ROI table for user
	int block_unit;			// ROI block unit 8, 16, 32, 64
	int table_width;
	int table_height;
	int table_size;

} hal_video_roi_s;

typedef struct {
	i32 bps;
	i32 maxqp;
	i32 minqp;
	i32 gop;
	int forcei;
} rate_ctrl_s;

typedef struct {
	i32 roiAreaEnable;
	i32 roiAreaTop;
	i32 roiAreaLeft;
	i32 roiAreaBottom;
	i32 roiAreaRight;
	i32 roiDeltaQp;
	i32 roiQp;
} roi_ctrl_s;

typedef struct {
	i32 objDetectNumber;
	i32 objTopX[MAX_OBJECT];
	i32 objTopY[MAX_OBJECT];
	i32 objBottomX[MAX_OBJECT];
	i32 objBottomY[MAX_OBJECT];
} obj_ctrl_s;

struct isp_tuning_cmd {
	uint32_t		err;
	uint16_t		opcode;
	uint16_t		status;
	uint32_t		addr;
	uint32_t		len;		/* only data */
	union {
		uint8_t	data[0];
		void	*ptr[0];
	};
} __attribute__((packed));

/* Structure for command line options */
typedef struct {

	char input[MAX_PATH];
	char output[MAX_PATH];

	i32 CodecType;

	i32 outputRateNumer;      /* Output frame rate numerator */
	i32 outputRateDenom;      /* Output frame rate denominator */
	i32 inputRateNumer;      /* Input frame rate numerator */
	i32 inputRateDenom;      /* Input frame rate denominator */
	i32 firstPic;
	i32 lastPic;

	i32 width;
	i32 height;
	i32 lumWidthSrc;
	i32 lumHeightSrc;

	i32 inputFormat;
	VCEncVideoCodecFormat outputFormat;     /* Video Codec Format: HEVC/H264/JPEG/YUV */
	i32 EncMode;                            /* Encode output mode 0: pause 1: one-shot 2: continue */
	i32 JpegMode;                           /* JPEG output mode 0: pause 1: one-shot 2: continue */
	i32 YuvMode;                            /* YUV output mode 0: pause 1: one-shot 2: continue */

	i32 picture_cnt;
	i32 byteStream;
	i32 videoStab;

	i32 max_cu_size;    /* Max coding unit size in pixels */
	i32 min_cu_size;    /* Min coding unit size in pixels */
	i32 max_tr_size;    /* Max transform size in pixels */
	i32 min_tr_size;    /* Min transform size in pixels */
	i32 tr_depth_intra;   /* Max transform hierarchy depth */
	i32 tr_depth_inter;   /* Max transform hierarchy depth */

	i32 min_qp_size;

	i32 enableCabac;      /* [0,1] H.264 entropy coding mode, 0 for CAVLC, 1 for CABAC */
	i32 cabacInitFlag;

	// intra setup
	u32 strong_intra_smoothing_enabled_flag;

	i32 cirStart;
	i32 cirInterval;

	i32 intraAreaEnable;
	i32 intraAreaTop;
	i32 intraAreaLeft;
	i32 intraAreaBottom;
	i32 intraAreaRight;

	i32 pcm_loop_filter_disabled_flag;

	i32 ipcm1AreaTop;
	i32 ipcm1AreaLeft;
	i32 ipcm1AreaBottom;
	i32 ipcm1AreaRight;

	i32 ipcm2AreaTop;
	i32 ipcm2AreaLeft;
	i32 ipcm2AreaBottom;
	i32 ipcm2AreaRight;
	i32 ipcmMapEnable;
	char *ipcmMapFile;

	char *skipMapFile;
	i32 skipMapEnable;
	i32 skipMapBlockUnit;

	roi_ctrl_s roi_ctrl[8];

	/* Rate control parameters */
	i32 hrdConformance;
	i32 cpbSize;
	i32 intraPicRate;   /* IDR interval */

	i32 vbr; /* Variable Bit Rate Control by qpMin */
	i32 qpHdr;
	i32 qpMin;
	i32 qpMax;
	i32 qpMinI;
	i32 qpMaxI;
	i32 bitPerSecond;
	i32 crf; /*CRF constant*/

	i32 bitVarRangeI;

	i32 bitVarRangeP;

	i32 bitVarRangeB;
	u32 u32StaticSceneIbitPercent;

	i32 tolMovingBitRate;/*tolerance of max Moving bit rate */
	i32 monitorFrames;/*monitor frame length for moving bit rate*/
	i32 picRc;
	i32 ctbRc;
	i32 blockRCSize;
	u32 rcQpDeltaRange;
	u32 rcBaseMBComplexity;
	i32 picSkip;
	i32 picQpDeltaMin;
	i32 picQpDeltaMax;
	i32 ctbRcRowQpStep;

	float tolCtbRcInter;
	float tolCtbRcIntra;

	i32 bitrateWindow;
	i32 intraQpDelta;
	i32 fixedIntraQp;
	i32 bFrameQpDelta;

	i32 disableDeblocking;

	i32 enableSao;


	i32 tc_Offset;
	i32 beta_Offset;

	i32 chromaQpOffset;

	i32 profile;              /*main profile or main still picture profile*/
	i32 tier;               /*main tier or high tier*/
	i32 level;              /*main profile level*/

	i32 bpsAdjustFrame[MAX_BPS_ADJUST];
	i32 bpsAdjustBitrate[MAX_BPS_ADJUST];
	i32 smoothPsnrInGOP;

	i32 sliceSize;

	i32 testId;

	i32 rotation;
	i32 mirror;
	i32 horOffsetSrc;
	i32 verOffsetSrc;
	i32 colorConversion;
	i32 scaledWidth;
	i32 scaledHeight;
	i32 scaledOutputFormat;

	i32 enableDeblockOverride;
	i32 deblockOverride;

	i32 enableScalingList;

	u32 compressor;

	i32 interlacedFrame;
	i32 fieldOrder;
	i32 videoRange;
	i32 ssim;
	i32 sei;
	char *userData;
	u32 gopSize;
	char *gopCfg;
	u32 gopLowdelay;
	i32 outReconFrame;
	u32 longTermGap;
	u32 longTermGapOffset;
	u32 ltrInterval;
	i32 longTermQpDelta;

	i32 gdrDuration;
	u32 roiMapDeltaQpBlockUnit;
	u32 roiMapDeltaQpEnable;
	char *roiMapDeltaQpFile;
	char *roiMapDeltaQpBinFile;
	char *roiMapInfoBinFile;
	char *RoimapCuCtrlInfoBinFile;
	char *RoimapCuCtrlIndexBinFile;
	u32 RoiCuCtrlVer;
	u32 RoiQpDeltaVer;
	i32 outBufSizeMax;



	i32 bitDepthLuma;
	i32 bitDepthChroma;

	u32 enableOutputCuInfo;

	u32 rdoLevel;
	u32 hashtype;
	u32 verbose;

	/* constant chroma control */
	i32 constChromaEn;
	u32 constCb;
	u32 constCr;

	i32 sceneChange[MAX_SCENE_CHANGE];

	/* for tile*/
	i32 tiles_enabled_flag;
	i32 num_tile_columns;
	i32 num_tile_rows;
	i32 loop_filter_across_tiles_enabled_flag;

	/*for skip frame encoding ctr*/
	i32 skip_frame_enabled_flag;
	i32 skip_frame_poc;

	/*stride*/
	u32 exp_of_input_alignment;
	u32 exp_of_ref_alignment;
	u32 exp_of_ref_ch_alignment;

	u32 RpsInSliceHeader;
	u32 vui_timing_info_enable;

	u32 picOrderCntType;
	u32 log2MaxPicOrderCntLsb;
	u32 log2MaxFrameNum;

	char *halfDsInput;

	u32 dumpRegister;
	u32 rasterscan;

	u32 lookaheadDepth;

	u32 cuInfoVersion;
	u32 enableRdoQuant;

	u32 AXIAlignment;

	u32 ivf;

	u32 MEVertRange;

	VCEncChromaIdcType codedChromaIdc;
	u32 PsyFactor;

	u32 aq_mode;
	double aq_strength;

	u32 preset;
	u32 writeReconToDDR;

  /* JPEG */
	i32 restartInterval;
//	i32 frameType;
	i32 partialCoding;
	i32 codingMode;
	i32 markerType;
	i32 qLevel;
	i32 unitsType;
	i32 xdensity;
	i32 ydensity;
	// Non support thumbnail

	i32 rcMode;
	u32 qpmin;
	u32 qpmax;
	i32 fixedQP;

  /* AmebaPro VOE/buffer control */
	i32 voe;
	i32 osd;
	i32 obj;

	u32 lumaSize;
	i32	enc_cnt;
	int ch;
	int status;

	int out_buf_size;
	int out_rsvd_size;
	int voe_dbg;


} commandLine_s;





typedef struct {
	VCEncVideoCodecFormat codecFormat;		/* Video Codec Format: HEVC/H264/JPEG */

	commandLine_s 		*cmd[MAX_CHANNEL];		// Channel 3 is RGB only
	struct vcenc_instance* enc_adapter[MAX_CHANNEL];
	struct vcenc_instance* jpg_adapter[MAX_CHANNEL];
	hal_isp_adapter_t 	*isp_adapter;

	void				*tid_output;			// HAL video internal output task PID
	void				*tid_isp;			    // VOE ISP internal output task PID on VOE

	int					*enc_in[MAX_CHANNEL];	// VCEncIn
	int					*enc_out[MAX_CHANNEL];	// VCEncOut

	int					*voe_heap_addr;			// VOE define heap address
	int 				voe_heap_size;			// VOE define heap size
	volatile int		open_ch;				// VOE open channel count
	volatile int		start_ch;				// VOE start channel count

	output_callback_t	out_cb[MAX_CHANNEL];	// Output stream callback function

	enc2out_t			*isp2out;				// ISP 2 OSD/ENC queue
	enc2out_t			*enc2out;				// ENC 2 OUT queue
	hal_video_buf_s		*outbuf[MAX_CHANNEL];
	hal_video_roi_s		*roi[MAX_CHANNEL];

	int					crc32;
	int 				enc_cycle;
	int					pre_cycle;
	int					post_cycle;

	int					enc_time;
	int					pre_time;
	int					post_time;


	i32 				width;
	i32 				height;

	/* SW/HW shared memories for input/output buffers */
#if 0 // next stage implement scale down
	char *halfDsInput;
	int *inDS;
	int *stab_addr;

	u8 *lumDS;
	u8 *cbDS;
	u8 *crDS;
	u32 src_img_size_ds;

	EWLLinearMem_t scaledPictureMem;
	EWLLinearMem_t pictureStabMem;
#endif
	u32 gopSize;
	u32 ctx[MAX_CHANNEL];
} hal_video_adapter_t;

#if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9

struct rts_isp_i2c_reg {
	u16 addr;
	u16 data;
};
#endif




/** @} */ // end of GROUP_ENC_MODULE_SYSTEM_CONFIGURATION

#if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9

hal_video_adapter_t *hal_video_init(void);
void hal_video_deinit(hal_video_adapter_t *enc_adapter);

int hal_video_str_parsing(char* str, hal_video_adapter_t *v_adp);
int hal_video_str2cmd(char *str1, char *str2, char *str3, hal_video_adapter_t *v_adp);
int hal_video_cmd_reset( hal_video_adapter_t *v_adp, int ch);

int hal_video_open(hal_video_adapter_t *enc_adapter, int ch);
int hal_video_close(hal_video_adapter_t *enc_adaptor, int ch);

int hal_video_start(hal_video_adapter_t *v_adp, int ch);
int hal_video_stop(hal_video_adapter_t *v_adp, int ch);

int hal_video_yuv_out(int ch, int mode);
int hal_video_jpg_out(int ch, int mode);

int hal_video_release(int ch, int len);
int hal_video_isp_buf_release(int ch, uint32_t buf_addr);
int hal_video_isp_set_rawfmt(int ch, uint32_t rawfmt);


int hal_video_froce_i(int ch);
int hal_video_set_rc(rate_ctrl_s *rc, int ch);
int hal_video_cb_register(hal_video_adapter_t *v_adp, output_callback_t output_cb, u32 arg, int ch);
int hal_video_isp_ctrl(int ctrl_id, int set_flag, int *value);

int hal_video_roi_region(int ch, int x, int y, int width, int height, int value);
int hal_video_obj_region(obj_ctrl_s *obj_r, int ch);

// Show VOE information
int hal_video_mem_info(hal_video_adapter_t *v_adp, int ch);
int hal_video_buf_info(hal_video_adapter_t *v_adp, int ch);
int hal_video_print(hal_video_adapter_t *v_adp, int mode);



static __inline__ int hal_video_set_wdt(int sec)
{
	return hal_voe_set_wdt(sec);
}

// VOE bring up flow
int hal_video_voe_open(hal_video_adapter_t *v_adp, int heap_size, int queue);
int hal_video_voe_close(hal_video_adapter_t *v_adp);

static __inline__ int hal_video_load_fw(voe_cpy_t voe_cpy, int *fw_addr, int *voe_ddr_addr)
{
	return hal_voe_load_fw(voe_cpy, fw_addr, voe_ddr_addr);
}

static __inline__ int hal_video_load_sensor(voe_cpy_t voe_cpy, int *fw_addr, int *voe_ddr_addr)
{
	return hal_voe_load_sensor(voe_cpy, fw_addr, voe_ddr_addr);
}

static __inline__ int hal_video_load_iq(hal_video_adapter_t *v_adp, int *iq_addr)
{
	if (v_adp->isp_adapter != NULL) {
		v_adp->isp_adapter->iq_addr = iq_addr;
		return OK;
	} else {
		printf("IQ address set fail\n");
		return NOK;
	}

}

// VOE!@ISP/ENC buffer setting
static __inline__ void hal_video_enc_buf(hal_video_adapter_t *v_adp, int ch, int buf_size, int rsvd_size)
{
	commandLine_s *cml =v_adp->cmd[ch];
	cml->out_buf_size = buf_size;
	cml->out_rsvd_size = rsvd_size;
}

static __inline__ int hal_video_isp_buf_num(hal_video_adapter_t *v_adp, int ch, int buf_num)
{
	if (v_adp->isp_adapter != NULL) {
		v_adp->isp_adapter->video_stream[ch].buff_num = buf_num;
		printf ("%d buff %d\n", ch, v_adp->isp_adapter->video_stream[ch].buff_num);
		return OK;
	} else {
		printf("Set stream ISP buffer fail\n");
		return NOK;
	}
}

#endif // #if !defined (CONFIG_VOE_PLATFORM) || !CONFIG_VOE_PLATFORM // Run on TM9
/** @} */ /* End of group hal_enc */

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _HAL_VIDEO_H_"

