#if defined(CONFIG_PLATFORM_8735B)
#include "ff.h"
#include <fatfs_ext/inc/ff_driver.h>
#include <disk_if/inc/sdcard.h>
#include "basic_types.h"
#include "section_config.h"
#include "mp4_muxer.h"
#include "osdep_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "avcodec.h"
//#include "mmf2_dbg.h"

//DMA DEFINE
#include "device.h"
#include "diag.h"

#define DMA_CONFIG_MP4 0

#if DMA_CONFIG_MP4
#include "dma_api.h"
#endif

//int moov_size = 0;

#define MDAT_DEFAULT_SIZE		32
#define MDAT_HEADER_SIZE		8
#define MEDIA_TIME_SCALE		1000
#define VIDEO_TRACK_TIME_SCALE	30000

//FOR DMA SETTING
//static gdma_t gdma;

#define MP4_DMA_BLOCK_LENGTH 4092
#define MP4_DMA_BLOCK_NUM 16
#define MP4_DMA_SIZE (MP4_DMA_BLOCK_LENGTH*MP4_DMA_BLOCK_NUM)

#define ORIGINAL_VERSION 0

#define VIDEO_TRACK_ID 2
#define AUDIO_TRACK_ID 1
#define NEXT_TRACK_ID 3

//int audio_appear_first = 0;
//int video_appear_first = 0;

//int audio_start = 0;
//int video_start = 0;

#define AUDIO_SKIP_FRAME 2

#if 0
struct BlockInfo {
	u32 SrcAddr;
	u32 DstAddr;
	u32 BlockLength;
	u32 SrcOffset;
	u32 DstOffset;
};

SDRAM_DATA_SECTION struct BlockInfo block_info[MP4_DMA_BLOCK_NUM];
#endif
//_sema	MP4_DMA_DONE;

static void dma_done_handler(uint32_t id)
{
	//DiagPrintf("DMA Copy Done = %d!!\r\n",id);
	pmp4_context mp4_ctx = (pmp4_context)id;
	rtw_up_sema_from_isr(&mp4_ctx->MP4_DMA_DONE);
}

static void set_dma(pmp4_context mp4_ctx, unsigned char *dst, unsigned char *src, int length)
{
	//printf("length = %d\r\n",length);
	int i = 0, j = 0;
	int byte_left = length, offset = 0;
	if (byte_left >= MP4_DMA_SIZE) {
		dma_memcpy_init(&mp4_ctx->gdma, dma_done_handler, (uint32_t)mp4_ctx);
		mp4_ctx->gdma.hal_gdma_adaptor.block_num = MP4_DMA_BLOCK_NUM;
		mp4_ctx->gdma.hal_gdma_adaptor.gdma_cb_para = mp4_ctx;
		for (i = 0; i < (byte_left / MP4_DMA_SIZE); i++) {
			for (j = 0; j < MP4_DMA_BLOCK_NUM; j++) {
				mp4_ctx->block_info[j].SrcOffset = 0;
				mp4_ctx->block_info[j].DstOffset = 0;
				mp4_ctx->block_info[j].SrcAddr = (uint32_t) &src[ i * MP4_DMA_SIZE + j * MP4_DMA_BLOCK_LENGTH]; //SRC
				mp4_ctx->block_info[j].DstAddr = (uint32_t) &dst[ i * MP4_DMA_SIZE + j * MP4_DMA_BLOCK_LENGTH]; //Dest
				mp4_ctx->block_info[j].BlockLength = MP4_DMA_BLOCK_LENGTH;
			}
			dma_multiblk_memcpy(&mp4_ctx->gdma, (phal_gdma_block_t) &mp4_ctx->block_info, MP4_DMA_BLOCK_NUM);
			rtw_down_sema(&mp4_ctx->MP4_DMA_DONE);
		}
		dma_memcpy_deinit(&(mp4_ctx->gdma));
		byte_left %= MP4_DMA_SIZE;
		offset = length - byte_left;
	}

	if (byte_left >= MP4_DMA_BLOCK_LENGTH) {
		memset(mp4_ctx->block_info, 0, sizeof(mp4_ctx->block_info));
		dma_memcpy_init(&mp4_ctx->gdma, dma_done_handler, (uint32_t) &mp4_ctx->gdma);
		mp4_ctx->gdma.hal_gdma_adaptor.gdma_cb_para = mp4_ctx;
		mp4_ctx->gdma.hal_gdma_adaptor.block_num = byte_left / MP4_DMA_BLOCK_LENGTH;
		for (j = 0; j < (byte_left / MP4_DMA_BLOCK_LENGTH); j++) {
			mp4_ctx->block_info[j].SrcOffset = 0;
			mp4_ctx->block_info[j].DstOffset = 0;
			mp4_ctx->block_info[j].SrcAddr = (uint32_t) &src[ offset + j * MP4_DMA_BLOCK_LENGTH]; //SRC
			mp4_ctx->block_info[j].DstAddr = (uint32_t) &dst[ offset + j * MP4_DMA_BLOCK_LENGTH]; //Dest
			mp4_ctx->block_info[j].BlockLength = MP4_DMA_BLOCK_LENGTH;
		}
		byte_left %= MP4_DMA_BLOCK_LENGTH;
		offset = length - byte_left;

		if (byte_left) {
			mp4_ctx->block_info[mp4_ctx->gdma.hal_gdma_adaptor.block_num].SrcOffset = 0;
			mp4_ctx->block_info[mp4_ctx->gdma.hal_gdma_adaptor.block_num].DstOffset = 0;
			mp4_ctx->block_info[mp4_ctx->gdma.hal_gdma_adaptor.block_num].SrcAddr = (uint32_t) &src[offset];//SRC
			mp4_ctx->block_info[mp4_ctx->gdma.hal_gdma_adaptor.block_num].DstAddr = (uint32_t) &dst[offset];//Dest
			mp4_ctx->block_info[mp4_ctx->gdma.hal_gdma_adaptor.block_num].BlockLength = byte_left;
			mp4_ctx->gdma.hal_gdma_adaptor.block_num ++;
		}

		dma_multiblk_memcpy(&mp4_ctx->gdma, (phal_gdma_block_t) &mp4_ctx->block_info, mp4_ctx->gdma.hal_gdma_adaptor.block_num);
		rtw_down_sema(&mp4_ctx->MP4_DMA_DONE);
		dma_memcpy_deinit(&(mp4_ctx->gdma));
	} else {
		memcpy(&dst[offset], &src[offset], byte_left);
	}
}

void mp4_muxer_init(pmp4_context mp4_ctx)
{
#if DMA_CONFIG_MP4
	rtw_init_sema(&mp4_ctx->MP4_DMA_DONE, 0);
	//printf("SEMA INIT\r\n");
	memset(mp4_ctx->block_info, 0, sizeof(mp4_ctx->block_info));
#endif
	mp4_ctx->storage_state = STORAGE_IDLE;
	mp4_ctx->fatfs_buf_size = 0;
	mp4_ctx->loop_mode = 0;
}
void mp4_muxer_close(pmp4_context mp4_ctx)
{
#if DMA_CONFIG_MP4
	rtw_free_sema(&mp4_ctx->MP4_DMA_DONE);
#endif
	printf("mp4_muxer_close\r\n");
}


void set_mp4_fatfs_param(pmp4_context mp4_ctx, fatfs_sd_params_t *fatfs_param)
{
	mp4_ctx-> drv_num = fatfs_param->drv_num;
	memcpy(mp4_ctx->_drv, fatfs_param->drv, sizeof(mp4_ctx->_drv));
	mp4_ctx->m_fs = fatfs_param->fs;
}


#if DMA_CONFIG_MP4
void copy(pmp4_context mp4_ctx, unsigned char *dst, unsigned char *src, int length)
{

	set_dma(mp4_ctx, dst, src, length);
}
#else
void copy(pmp4_context mp4_ctx, unsigned char *dst, unsigned char *src, int length)
{
	memcpy(dst, src, length);
}
#endif

#if defined(__GNUC__)
#define MUX_HTONL(x) ((((x) & 0x000000ffUL) << 24) | \
                      (((x) & 0x0000ff00UL) <<  8) | \
                      (((x) & 0x00ff0000UL) >>  8) | \
                      (((x) & 0xff000000UL) >> 24))
#elif defined(__ICCARM__)
#include <intrinsics.h>
#define MUX_HTONL(x) __REV(x)
#endif
void Save32BigEndian(unsigned int src, void *DES)
{
	/* 	unsigned int *des = (unsigned int *)DES;
		*des = MUX_HTONL(src);
		return; */

	char tbuf[4] = {0};
	unsigned int *temp = (unsigned int *)tbuf;
	*temp = src;
	char *des = (char *)DES;
	des[0] = tbuf[3];
	des[1] = tbuf[2];
	des[2] = tbuf[1];
	des[3] = tbuf[0];
	return;

}

void Save32BigEndianPartial(long unsigned int src, void *DES, int size)
{

	char tbuf[4] = {0};
	long unsigned int *temp = (long unsigned int *)tbuf;
	*temp = src;
	char *des = (char *)DES;
	for (int i = 0 ; i < size ; i++) {
		des[i] = tbuf[3 - i];
	}
	return;
}

void Save32BigEndianPartialEnd(long unsigned int src, void *DES, int endian_convertion_pos)
{

	char tbuf[4] = {0};
	long unsigned int *temp = (long unsigned int *)tbuf;
	*temp = src;
	char *des = (char *)DES;
	for (int i = endian_convertion_pos ; i < 4; i++) {
		des[i] = tbuf[3 - i];
	}
	return;
}
static int fatfs_write_flush(pmp4_context mp4_ctx)
{
	//printf("\n\rfatfs_write_flush: %d\n\r",mp4_ctx->fatfs_buf_pos);
	int bw = 0;
	int ret = 0;
	if (mp4_ctx->fatfs_buf_pos != 0) {
#if 0
		ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_pos, (u32 *)&bw);
		if (ret != FR_OK) {
			return -1;
		}
#else
		if (mp4_ctx->sd_card_is_mounted_cb) {
			if (mp4_ctx->sd_card_is_mounted_cb()) {
				ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_pos, (u32 *)&bw);
				if (bw != mp4_ctx->fatfs_buf_pos) {
					printf("bw %d mp4_ctx->fatfs_buf_pos %d\r\n", bw, mp4_ctx->fatfs_buf_pos);
					return -1;
				}
				if (ret != FR_OK) {
					return -1;
				}
			} else {
				printf("The sd card is removed\r\n");
				return -1;
			}
		} else {
			ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_pos, (u32 *)&bw);
			if (bw != mp4_ctx->fatfs_buf_pos) {
				printf("bw %d mp4_ctx->fatfs_buf_pos %d\r\n", bw, mp4_ctx->fatfs_buf_pos);
				return -1;
			}
			if (ret != FR_OK) {
				return -1;
			}
		}
#endif
	}
	mp4_ctx->fatfs_buf_pos = 0;
	return FR_OK;

}

static int fatfs_write(unsigned char *buf, unsigned int length, pmp4_context mp4_ctx, char write_header, int mp4_frame_size)
{
	//printf("fatfs_write length= %d \n\r",length);
	int ret = 0;
	int bw = 0, offset = 0, copy_size = 0, endian_convertion_pos = 0;
	while (length > 0) {
		if (mp4_ctx->fatfs_buf_pos + length >= mp4_ctx->fatfs_buf_size) {
			copy_size = mp4_ctx->fatfs_buf_size - mp4_ctx->fatfs_buf_pos;
			copy(mp4_ctx, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos, buf + offset, copy_size);

			if (write_header && offset == 0) {
				if (copy_size < 4) {

					endian_convertion_pos = copy_size;
					Save32BigEndianPartial(mp4_frame_size, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos, copy_size);
					// buffer not enough for Endian modification (4 byte)

				} else {
					Save32BigEndian(mp4_frame_size, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos);
				}
			}
#if 0
			ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_size, (u32 *)&bw);
			if (ret != FR_OK) {
				return -1;
			}
#else
			if (mp4_ctx->sd_card_is_mounted_cb) {
				if (mp4_ctx->sd_card_is_mounted_cb()) {
					ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_size, (u32 *)&bw);
					if (bw != mp4_ctx->fatfs_buf_size) {
						printf("bw %d mp4_ctx->fatfs_buf_size %d\r\n", bw, mp4_ctx->fatfs_buf_size);
						return -1;
					}
					if (ret != FR_OK) {
						return -1;
					}
				} else {
					printf("The sd card is removed\r\n");
					return -1;
				}
			} else {
				ret = f_write(&mp4_ctx->m_file, mp4_ctx->fatfs_buf, mp4_ctx->fatfs_buf_size, (u32 *)&bw);
				if (bw != mp4_ctx->fatfs_buf_size) {
					printf("bw %d mp4_ctx->fatfs_buf_size %d\r\n", bw, mp4_ctx->fatfs_buf_size);
					return -1;
				}
				if (ret != FR_OK) {
					return -1;
				}
			}

#endif
			mp4_ctx->fatfs_buf_pos = 0;
			offset += copy_size;
			length -= copy_size;
		} else {
			copy(mp4_ctx, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos, buf + offset, length);
			if (write_header && offset == 0) {
				Save32BigEndian(mp4_frame_size, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos);
			} else if (write_header && endian_convertion_pos) {
				Save32BigEndianPartialEnd(mp4_frame_size, mp4_ctx->fatfs_buf + mp4_ctx->fatfs_buf_pos, endian_convertion_pos);
				endian_convertion_pos = 0;
			}
			mp4_ctx->fatfs_buf_pos += length;
			length = 0;
		}
	}
	return FR_OK;
}

int mp4_update_data(pmp4_context mp4_ctx, char write_header, int mp4_frame_size)
{
	int ret = 0;
	int bw = 0;
	if (mp4_ctx->cb_fwrite) {
		if (write_header) {
			Save32BigEndian(mp4_frame_size, mp4_ctx->payload.addr);
		}
		ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, mp4_ctx->payload.addr, mp4_ctx->payload.len, (u32 *)&bw);
	} else {
		ret = fatfs_write(mp4_ctx->payload.addr, mp4_ctx->payload.len, mp4_ctx,  write_header, mp4_frame_size);
	}
	if (ret != FR_OK) {
		return -1;
	}
	return FR_OK;
}

void init_mp4_root(PMP4root root)
{
	root->keyindex = 0;
	root->video_len = 0;
	root->audio_len = 0;
	root->total = 0;
}

void Append_video_frame(pmp4_context mp4_ctx, int size)
{
	if (mp4_ctx->root.video_len < mp4_ctx->video_size) {
		mp4_ctx->video_buffer_size[mp4_ctx->root.video_len] = size;
		mp4_ctx->video_buffer_index[mp4_ctx->root.video_len] = mp4_ctx->root.total + MDAT_DEFAULT_SIZE;
		mp4_ctx->root.video_len++;
		mp4_ctx->root.total += size;
	}
}

void Append_audio_frame(pmp4_context mp4_ctx, int size)
{
	if (mp4_ctx->root.audio_len < mp4_ctx->audio_size) {
		mp4_ctx->audio_buffer_size[mp4_ctx->root.audio_len] = size;
		mp4_ctx->audio_buffer_index[mp4_ctx->root.audio_len] = mp4_ctx->root.total + MDAT_DEFAULT_SIZE;
		mp4_ctx->root.audio_len++;
		mp4_ctx->root.total += size;
	}
}

void Append_video_keyframe(pmp4_context mp4_ctx)
{
	if (mp4_ctx->root.keyindex < mp4_ctx->key_frame_size) {
		mp4_ctx->key_frame[mp4_ctx->root.keyindex] = mp4_ctx->root.video_len;
		mp4_ctx->root.keyindex++;
	}
}

void copy_sps_pps(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{
	unsigned int sps_offset = 0;
	unsigned int pps_offset = 0;
	unsigned int frame_offset = 0;
	unsigned char type = 0;
	for (int i = 0; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if ((buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01 && mp4_ctx->nal_len == 4)
				|| (buffer[i + 2] == 0x01 && mp4_ctx->nal_len == 3)) {
				type = buffer[i + mp4_ctx->nal_len] & 0x1f;
				if (type == 0x08) {
					pps_offset = i;
					i += mp4_ctx->nal_len;
				} else {
					frame_offset = i;
					break;
				}
			}
		}
	}

	mp4_ctx->sps_len = pps_offset - sps_offset;
	mp4_ctx->pps_len = frame_offset - pps_offset - mp4_ctx->nal_len;

	memcpy((char *)mp4_ctx->sps_str, (char *)(buffer), mp4_ctx->sps_len);
	memcpy((char *)mp4_ctx->pps_str, (char *)(buffer + pps_offset + mp4_ctx->nal_len), mp4_ctx->pps_len);
}

int get_sps_pps(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{
	int type = 0;
	unsigned int i = 0;
	unsigned int first_start_code = 0;
	for (i = 0; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if (buffer[i + 2] == 0x01) {
				mp4_ctx->nal_len = 3;
			} else if (buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01) {
				mp4_ctx->nal_len = 4;
			} else {
				continue;
			}
			first_start_code = i;
			break;
		}
	}

	for (i = first_start_code; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if ((buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01 && mp4_ctx->nal_len == 4)
				|| (buffer[i + 2] == 0x01 && mp4_ctx->nal_len == 3)) {
				type = buffer[i + mp4_ctx->nal_len] & 0x1f;

				if (type == 0x07) {	// SPS
					mp4_ctx->h264_extra_len = i + mp4_ctx->nal_len;
					copy_sps_pps(mp4_ctx, buffer + i + mp4_ctx->nal_len, size);
					//printf("sps_len = %d pps _len = %d start_index = %d nal_len = %d\r\n",mp4_ctx->sps_len,mp4_ctx->pps_len,mp4_ctx->h264_extra_len,mp4_ctx->nal_len);
					mp4_ctx->sps_pps_mark = 1;
					return 1;
				} else if (type == 0x01) {	// Slice
					return 0;
				} else {	// irrelevant type, continue
					i += mp4_ctx->nal_len;
				}
			}
		}
	}
	return 0;
}

void copy_vps_sps_pps(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{
	unsigned int sps_offset = 0;
	unsigned int pps_offset = 0;
	unsigned int vps_offset = 0;
	unsigned int frame_offset = 0;
	unsigned char type = 0;
	//unsigned char vps_buf[64]={0};
	//unsigned char sps_buf[64]={0};
	//unsigned char pps_buf[64]={0};

	//int sps_len = 0;
	//int pps_len = 0;
	//int vps_len = 0;
	//int nal_len = 0;
	for (unsigned int i = 0; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if ((buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01 && mp4_ctx->nal_len == 4)
				|| (buffer[i + 2] == 0x01 && mp4_ctx->nal_len == 3)) {
				type = buffer[i + mp4_ctx->nal_len];
				if (type == 0x42) {	//SPS
					vps_offset = i;
					i += mp4_ctx->nal_len;
					printf("vps_offset %d\r\n", vps_offset);
				} else if (type == 0x44) { //PPS
					sps_offset = i;
					i += mp4_ctx->nal_len;
					printf("sps_offset %d\r\n", sps_offset);
				} else {
					pps_offset = i;
					printf("frame_offset %d\r\n", pps_offset);
					break;
				}
			}
		}
	}
	mp4_ctx->vps_len = vps_offset;
	mp4_ctx->sps_len = sps_offset - vps_offset - mp4_ctx->nal_len;
	mp4_ctx->pps_len = pps_offset - sps_offset - mp4_ctx->nal_len;

	printf("vps_len %d sps_len %d pps_len %d\r\n", mp4_ctx->vps_len, mp4_ctx->sps_len, mp4_ctx->pps_len);

	memcpy((char *)mp4_ctx->vps_str, (char *)(buffer), mp4_ctx->vps_len);
	memcpy((char *)mp4_ctx->sps_str, (char *)(buffer) + vps_offset + mp4_ctx->nal_len, mp4_ctx->sps_len);
	memcpy((char *)mp4_ctx->pps_str, (char *)(buffer) + sps_offset + mp4_ctx->nal_len, mp4_ctx->pps_len);
	printf("vps start\r\n");
#if 0
	for (int i = 0; i < mp4_ctx->vps_len; i++) {
		printf("%x,", mp4_ctx->vps_str[i]);
	}
	printf("\r\n");
	printf("sps start\r\n");
	for (int i = 0; i < mp4_ctx->sps_len; i++) {
		printf("%x,", mp4_ctx->sps_str[i]);
	}
	printf("\r\n");
	printf("pps start\r\n");
	for (int i = 0; i < mp4_ctx->pps_len; i++) {
		printf("%x,", mp4_ctx->pps_str[i]);
	}
	printf("\r\n");
#endif
}

int get_hevc_info(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{
	int type = 0;
	unsigned int i = 0;
	//int nal_len = 0;
	unsigned int first_start_code = 0;
	//int h264_extra_len = 0;
	int sps_pps_mark = 0;
	//for(i=0;i<size;i++){
	//printf("%d = %d\r\n",i,buffer[i]);
	//}
	for (i = 0; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if (buffer[i + 2] == 0x01) {
				mp4_ctx->nal_len = 3;
			} else if (buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01) {
				mp4_ctx->nal_len = 4;
			} else {
				continue;
			}
			first_start_code = i;
			break;
		}
	}

	for (i = first_start_code; i < size ; i++) {
		if (buffer[i] == 0x00 && buffer[i + 1] == 0x00) {
			if ((buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01 && mp4_ctx->nal_len == 4)
				|| (buffer[i + 2] == 0x01 && mp4_ctx->nal_len == 3)) {
				type = buffer[i + mp4_ctx->nal_len];

				if (type == 0x40) {	// VPS
					mp4_ctx->h264_extra_len = i + mp4_ctx->nal_len;
					printf("h264_extra_len %d\r\n", mp4_ctx->h264_extra_len);
					copy_vps_sps_pps(mp4_ctx, buffer + i + mp4_ctx->nal_len, size);
					//printf("sps_len = %d pps _len = %d start_index = %d nal_len = %d\r\n",mp4_ctx->sps_len,mp4_ctx->pps_len,mp4_ctx->h264_extra_len,mp4_ctx->nal_len);
					mp4_ctx->sps_pps_mark = 1;
					return 1;
				} else if (type == 0x26) {	// i frame
					return 0;
				} else {	// irrelevant type, continue
					i += mp4_ctx->nal_len;
				}
			}
		}
	}

	//printf("%d\r\n",first_start_code);
	return 0;
}

int Write_mdat_data_video(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{

	int type = 0;
	int index = 0;
	int ret = 0;
	if (mp4_ctx->encoder_type == AV_CODEC_ID_H264) {
		type = buffer[mp4_ctx->h264_extra_len] & 0x1f;
		if (type == 0x07) {
			index = mp4_ctx->h264_extra_len + mp4_ctx->sps_len + mp4_ctx->nal_len + mp4_ctx->pps_len;
		} else {
			index = mp4_ctx->h264_extra_len - mp4_ctx->nal_len;
		}

		if (mp4_ctx->sps_start == 0) {
			if (type != 0x07) {
				return 0;
			} else {
				//printf("get first\r\n");
				mp4_ctx->start_time = rtw_get_current_time();
				mp4_ctx->sps_start++;
			}
		}
		Append_video_frame(mp4_ctx, size - index);
		if (type != 1) {
			Append_video_keyframe(mp4_ctx);
		}
	} else {
		type = buffer[mp4_ctx->h264_extra_len];
		if (type == 0x40) {
			index = mp4_ctx->h264_extra_len + mp4_ctx->vps_len + mp4_ctx->nal_len + mp4_ctx->sps_len + mp4_ctx->nal_len + mp4_ctx->pps_len;
		} else {
			index = mp4_ctx->h264_extra_len - mp4_ctx->nal_len;
		}

		if (mp4_ctx->sps_start == 0) {
			if (type != 0x40) {
				return 0;
			} else {
				//printf("get first\r\n");
				mp4_ctx->start_time = rtw_get_current_time();
				mp4_ctx->sps_start++;
			}
		}
		Append_video_frame(mp4_ctx, size - index);
		if (type != 0x02) {
			Append_video_keyframe(mp4_ctx);
		}

	}
	//Save32BigEndian((size-index-4), buffer+index);



	mp4_ctx->payload.addr = buffer + index;
	mp4_ctx->payload.len = size - index;
	ret = mp4_update_data(mp4_ctx, 1, (size - index - 4));
	if (ret != FR_OK) {
		return -1;
	}

	return 0;
}

int Write_mdat_data_audio(pmp4_context mp4_ctx, unsigned char *buffer, unsigned int size)
{
	int len = 0;
	int ret = 0;
#if 1
	if (buffer[0x01] & 0x01) {
		len = 0x07;    //no error bit length eqaul 7 bytes
	} else {
		len = 0x09;    //has error bit length equal 9 bytes
	}
	Append_audio_frame(mp4_ctx, size - len);

	mp4_ctx->payload.addr = buffer + len;
	mp4_ctx->payload.len = size - len;
	ret = mp4_update_data(mp4_ctx, 0, 0);
#else
	Append_audio_frame(mp4_ctx, size);

	mp4_ctx->payload.addr = buffer;
	mp4_ctx->payload.len = size;
	ret = mp4_update_data(mp4_ctx, 0, 0);
#endif
	if (ret != FR_OK) {
		return -1;
	}
	return 0;
}

void check_file_exist(const char *mp4path, pmp4_context mp4_ctx)
{
	FRESULT fr = FR_OK;
	FILINFO fno = {0};
	char file[_MAX_LFN + 1];

#if _USE_LFN
	char fname_lfn[_MAX_LFN + 1] = {0};
	fno.lfname = fname_lfn;
	fno.lfsize = sizeof(fname_lfn);
#endif

	printf("Test for '%s'...\n", mp4path);
	fr = f_stat(mp4path, &fno);
	//printf("mp4path = %s _1\r\n",mp4path);

	char *filename;
#if _USE_LFN
	filename = *fno.lfname ? fno.lfname : fno.fname;
#else
	filename = fno.fname;
#endif
	sprintf((char *)file, "%s/%s", mp4_ctx->_drv, filename);
	switch (fr) {
	case FR_OK:
		printf("File Exist & Size: %s %lu\n", filename, fno.fsize);
		//printf("mp4path = %s _2\r\n",mp4path);
		fr = f_unlink(file);
		if (fr) {
			printf("remove file (%s) %x fail. result = %d\n", filename, fr);
			f_close(&mp4_ctx->m_file);
		} else {
			printf("Remove path = %s\r\n", filename);
		}
		break;
	case FR_NO_FILE:
		printf("It does not exist.\n");
		break;

	default:
		printf("An error occured. (%d)\n", fr);
	}
}

//static char buffer[128]={0};
int mp4_open_file(pmp4_context mp4_ctx)
{
	int res = 0;
	//char *mp4path = buffer;
	char *mp4path = malloc(128);
	if (mp4path == NULL) {
		printf("%s can't allocate the mp4path buffer%d\r\n", __FUNCTION__);
		return -1;
	}
#if 0
	printf("mp4_ctx->remove_append_name = %d mp4_ctx->file_total = %d\r\n", mp4_ctx->remove_append_name, mp4_ctx->file_total);
	if (mp4_ctx->file_total <= 1) {
		if (mp4_ctx->remove_append_name) {
			sprintf(mp4path, "%s", mp4_ctx->filename);
		} else {
			sprintf(mp4path, "%s%s.mp4", mp4_ctx->_drv, mp4_ctx->filename);
		}
	} else {
		sprintf(mp4path, "%s%s_%d.mp4", mp4_ctx->_drv, mp4_ctx->filename, mp4_ctx->file_name_index);
	}
#endif
	//check_file_exist(mp4path,mp4_ctx);
	memset(mp4path, 0, sizeof(mp4path));
	//printf("sizeof(mp4path) = %d\r\n",sizeof(buffer));
	//strcpy(mp4path, mp4_ctx->_drv);
	if (mp4_ctx->file_total <= 1) {
		if (mp4_ctx->remove_append_name) {
			sprintf(mp4path, "%s", mp4_ctx->filename);
		} else {
			strcpy(mp4path, mp4_ctx->_drv);
			sprintf(mp4path + strlen(mp4path), "%s.mp4", mp4_ctx->filename);
		}
	} else {
		strcpy(mp4path, mp4_ctx->_drv);
		sprintf(mp4path + strlen(mp4path), "%s_%d.mp4", mp4_ctx->filename, mp4_ctx->file_name_index);
	}
	if (mp4_ctx->cb_fopen) {
		res = mp4_ctx->cb_fopen(&mp4_ctx->m_file, mp4path, FA_OPEN_ALWAYS | FA_WRITE);
	} else {
		res = f_open(&mp4_ctx->m_file, mp4path, FA_OPEN_ALWAYS | FA_WRITE);
	}
	if (res) {
		printf("open file (%s) fail.\n", mp4path);
		if (mp4path) {
			free(mp4path);
		}
		if (mp4_ctx->cb_fclose) {
			mp4_ctx->cb_fclose(&mp4_ctx->m_file);
		} else {
			f_close(&mp4_ctx->m_file);
		}
		return -1;
	} else {
		printf("open file (%s) len = %d seconds\r\n", mp4path, mp4_ctx->period_time);
	}
	if (mp4path) {
		free(mp4path);
	}
	return 0;
}

int create_box(unsigned char *buf, char *str, int size)
{
	Save32BigEndian(size, buf);
	strncpy((char *)(buf + 4), str, strlen(str));
	return size;
}

int create_box_full(unsigned char *buf, char *str, int size, unsigned int version, unsigned int flag)
{
	Save32BigEndian(size, buf);
	strncpy((char *)(buf + 4), str, strlen(str));
	Save32BigEndian(version, (char *)(buf + 8));
	return size;
}

void update_box_size(pmp4_context mp4_ctx, unsigned char *buf, int size)
{
	Save32BigEndian(size, buf);
	mp4_ctx->moov_size = size;
}

///////////////////////////////////////////////
int tkhd_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int media_timescale = 600;//the number of time units that pass per second
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "tkhd", 92, 0, 1);
	Save32BigEndian(1, moov_buf + moov_position + 8); //Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, tkhd->data+12);//Creation Time
	//Save32BigEndian(present_time, tkhd->data+16);//Modification Time
	Save32BigEndian(VIDEO_TRACK_ID, moov_buf + moov_position + 20); //Track ID
	//Save32BigEndian(0, tkhd->data+24);//Reserved(4Bytes)
#if 0
	Save32BigEndian(mp4_ctx->root.video_len * MEDIA_TIME_SCALE / mp4_ctx->frame_rate, moov_buf + moov_position + 28); //Duration
#else
	//Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len-1]-mp4_ctx->video_timestamp_buffer[0], moov_buf+moov_position+24);
	Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[0], moov_buf + moov_position + 28);
	//Save32BigEndian(mp4_ctx->root.video_len*MEDIA_TIME_SCALE/mp4_ctx->frame_rate, moov_buf+moov_position+28);//Duration
#endif
	//Save32BigEndian(0, tkhd->data+32);//Reserved(8Bytes)
	//Save32BigEndian(0, tkhd->data+40);//Layer(2Bytes)=0 + Alternate Group(2Bytes)=0
	//Save32BigEndian(0, tkhd->data+44);//Volume(2Bytes)=0 + Reserved(2Bytes)
	//long unsigned int matrix[9]= { 0x00010000,0,0,0,0x00010000,0,0,0,0x40000000 };
	Save32BigEndian(0x00010000, moov_buf + moov_position + 48); //Matrix structure
	Save32BigEndian(0x00010000, moov_buf + moov_position + 64); //Matrix structure
	Save32BigEndian(0x40000000, moov_buf + moov_position + 80); //Matrix structure
	Save32BigEndian(mp4_ctx->width << 16, moov_buf + moov_position + 84); //Track width
	Save32BigEndian(mp4_ctx->height << 16, moov_buf + moov_position + 88); //Track height

	moov_position += size;
	return moov_position;
}

int tkhd_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int media_timescale = 600;//the number of time units that pass per second
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "tkhd", 92, 7, 0);
	//Save32BigEndian(1, moov_buf+moov_position+8);//Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, tkhd->data+12);//Creation Time
	//Save32BigEndian(present_time, tkhd->data+16);//Modification Time
#if 0
	Save32BigEndian(1, moov_buf + moov_position + 20); //Track ID
#else
	Save32BigEndian(AUDIO_TRACK_ID, moov_buf + moov_position + 20); //Track ID
#endif
	//Save32BigEndian(0, tkhd->data+24);//Reserved(4Bytes)
#if 0
	Save32BigEndian(mp4_ctx->root.audio_len * 1000 * 1024 / mp4_ctx->sample_rate,
					moov_buf + moov_position + 28); //Save32BigEndian(44628, moov_buf+moov_position+28);//Duration
#else
	if (mp4_ctx->type == STORAGE_ALL) {
		float diff = (mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 1] - mp4_ctx->audio_timestamp_buffer[0]);
		float multi = (float)mp4_ctx->video_clock_rate / (float)mp4_ctx->audio_clock_rate;
		float temp = diff * multi;
		//printf("audio tkhd = %d multi = %f\r\n",(int)temp,multi);
		//printf("diff = %d clock = %x clock = %d\r\n",mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len-1]-mp4_ctx->audio_timestamp_buffer[0],mp4_ctx->audio_clock_rate,mp4_ctx->video_clock_rate);
		Save32BigEndian((int)temp, moov_buf + moov_position + 28); //Save32BigEndian(44628, moov_buf+moov_position+28);//Duration
	} else {
		//Save32BigEndian(mp4_ctx->root.audio_len*1000*1024/mp4_ctx->sample_rate, moov_buf+moov_position+28);
		Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 1] - mp4_ctx->audio_timestamp_buffer[0], moov_buf + moov_position + 28);
	}
#endif
	//Save32BigEndian(0, tkhd->data+32);//Reserved(8Bytes)
	//Save32BigEndian(0, tkhd->data+40);//Layer(2Bytes)=0 + Alternate Group(2Bytes)=0
	//Save32BigEndian(0, tkhd->data+44);//Volume(2Bytes)=0 + Reserved(2Bytes)
	//long unsigned int matrix[9]= { 0x00010000,0,0,0,0x00010000,0,0,0,0x40000000 };
	Save32BigEndian(0x01000000, moov_buf + moov_position + 44); //Volume(2Bytes)=0 + Reserved(2Bytes)
	Save32BigEndian(0x00010000, moov_buf + moov_position + 48); //Matrix structure
	Save32BigEndian(0x00010000, moov_buf + moov_position + 64); //Matrix structure
	Save32BigEndian(0x40000000, moov_buf + moov_position + 80); //Matrix structure
	Save32BigEndian(0 << 16, moov_buf + moov_position + 84); //Track width
	Save32BigEndian(0 << 16, moov_buf + moov_position + 88); //Track height

	moov_position += size;
	return moov_position;
}

int hdlr_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	const char *hdlname = "Ameba ISO Video Handler";
	size = create_box_full(moov_buf + moov_position, "hdlr", 32 + strlen(hdlname) + 1, 0, 0);
	strncpy((char *)(moov_buf + moov_position + 16), "vide", strlen("vide")); //handler type:'vide' for video data, 'soun' for sound data or ¡¥subt¡¦ for subtitles
	//3*4 = 12 Bytes reserverd, set to 0
	strcpy((char *)(moov_buf + moov_position + 32), hdlname); //handler name
	moov_position += size;
	return moov_position;
}

int hdlr_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	const char *hdlname = "Ameba ISO Video Handler";
	size = create_box_full(moov_buf + moov_position, "hdlr", 32 + strlen(hdlname) + 1, 0, 0);
	strncpy((char *)(moov_buf + moov_position + 16), "soun", strlen("soun")); //handler type:'vide' for video data, 'soun' for sound data or ¡¥subt¡¦ for subtitles
	//3*4 = 12 Bytes reserverd, set to 0
	strcpy((char *)(moov_buf + moov_position + 32), hdlname); //handler name
	moov_position += size;
	return moov_position;
}

int smhd_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "smhd", 16, 0, 0);
	moov_position += size;
	return moov_position;
}

int vmhd_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "vmhd", 20, 1, 0);
	moov_position += size;
	return moov_position;
}

int esds_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "esds", 41, 0, 0);
	unsigned int i = 0;
	unsigned int sample_rate_type = 0;
	unsigned int audio_Object_Type = 2;//1;//2;//1;
	unsigned int freq_idx_map[] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350};
	//mp4_ctx->sample_rate = 16000;
	for (i = 0; i < sizeof(freq_idx_map) / sizeof(freq_idx_map[0]); i++) {
		if (mp4_ctx->sample_rate == freq_idx_map[i]) {
			sample_rate_type = i;
		}
	}
	//tag 03
	moov_buf[moov_position + 12] = 0x03;
	moov_buf[moov_position + 13] = 0x19;
	moov_buf[moov_position + 14] = 0x00;
	moov_buf[moov_position + 15] = 0x00;
	moov_buf[moov_position + 16] = 0x00;
	//tag 04
	moov_buf[moov_position + 16 + 1] = 0x04;
	moov_buf[moov_position + 17 + 1] = 0x11;
	moov_buf[moov_position + 18 + 1] = 0x40; //mp4
	moov_buf[moov_position + 19 + 1] = 0x15;

	moov_buf[moov_position + 20 + 1] = 0x00;
	moov_buf[moov_position + 21 + 1] = 0x01;
	moov_buf[moov_position + 22 + 1] = 0x05;
#if 0
	//max bitrate
	moov_buf[moov_position + 23 + 1] = 0x00;
	moov_buf[moov_position + 24 + 1] = 0x01;
	moov_buf[moov_position + 25 + 1] = 0xf4;
	moov_buf[moov_position + 26 + 1] = 0xf8;
	//avg bitrate
	moov_buf[moov_position + 27 + 1] = 0x00;
	moov_buf[moov_position + 28 + 1] = 0x01;
	moov_buf[moov_position + 29 + 1] = 0x73;
	moov_buf[moov_position + 30 + 1] = 0xF8;
#endif
	//tag 05
	//moov_buf[moov_position+31+1]=0x00;
	moov_buf[moov_position + 32] = 0x05;
	moov_buf[moov_position + 33] = 0x02;
	moov_buf[moov_position + 34] = (audio_Object_Type  << 3) | (sample_rate_type >> 1);
	mp4_ctx->channel_count = 1;
	moov_buf[moov_position + 35] = ((sample_rate_type & 0x01) << 7) | (mp4_ctx->channel_count << 3) | (0x00) | (0x00) | (0x00);
#if 0
	moov_buf[moov_position + 34] = 0x0a;
	moov_buf[moov_position + 35] = 0x10;
#endif
	moov_buf[moov_position + 36] = 0x00;
	moov_buf[moov_position + 37] = 0x00;
	moov_buf[moov_position + 38] = 0x06;
	moov_buf[moov_position + 39] = 0x01;
	moov_buf[moov_position + 40] = 0x02;
	//Save32BigEndian(1, moov_buf+moov_position+12);
	moov_position += size;
	return moov_position;
}

int mp4a_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	//int i = 0;
	int moov_position = 0;
	int stsdbox_pos = 0;
	size = create_box(moov_buf + moov_position, "mp4a", 36);

	//
	stsdbox_pos = 8;
	//reserved_format[6]
	//moov_buf[stsdbox_pos] = 0x00;
	//moov_buf[stsdbox_pos + 1] = 0x00;
	//moov_buf[stsdbox_pos + 2] = 0x00;
	//moov_buf[stsdbox_pos + 3] = 0x00;
	//moov_buf[stsdbox_pos + 4] = 0x00;
	//moov_buf[stsdbox_pos + 5] = 0x00;
	stsdbox_pos += 6;
	//data_reference_index  0x01;
	moov_buf[stsdbox_pos] = 0x00;
	moov_buf[stsdbox_pos + 1] = 0x01;
	stsdbox_pos += 2;
	//reserved[2]
	//moov_buf[stsdbox_pos] = 0x00;
	//moov_buf[stsdbox_pos + 1] = 0x00;
	//moov_buf[stsdbox_pos + 2] = 0x00;
	//moov_buf[stsdbox_pos + 3] = 0x00;
	stsdbox_pos += 4;
	//moov_buf[stsdbox_pos] = 0x00;
	//moov_buf[stsdbox_pos + 1] = 0x00;
	//moov_buf[stsdbox_pos + 2] = 0x00;
	//moov_buf[stsdbox_pos + 3] = 0x00;
	stsdbox_pos += 4;
	//channelcount = 2
	moov_buf[stsdbox_pos] = 0x00;
	moov_buf[stsdbox_pos + 1] = 0x02;
	stsdbox_pos += 2;
	//samplesize = 16 0x10
	moov_buf[stsdbox_pos] = 0x00;
	moov_buf[stsdbox_pos + 1] = 0x10;
	stsdbox_pos += 2;
	//pre_defined
	//moov_buf[stsdbox_pos] = 0x00;
	//moov_buf[stsdbox_pos + 1] = 0x00;
	stsdbox_pos += 2;
	//reserved_1
	//moov_buf[stsdbox_pos] = 0x00;
	//moov_buf[stsdbox_pos + 1] = 0x00;
	stsdbox_pos += 2;
	//samplerate samplerate = {timescale of media}<<16;
	Save32BigEndian(mp4_ctx->sample_rate, moov_buf + stsdbox_pos);
	stsdbox_pos += 4;
	//
	moov_position += size;
	size = esds_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int url_box(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "url ", 12, 1, 0);
	moov_position += size;
	return moov_position;
}

int dref_box(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "dref", 12 + 4 * 1, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12);
	moov_position += size;
	size = url_box(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int dinf_box(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "dinf", 8);
	moov_position += size;
	size = dref_box(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int avcC_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "avcC", 16 + mp4_ctx->sps_len + 3 + mp4_ctx->pps_len);
	Save32BigEndian(((0x01 << 24) | (mp4_ctx->sps_str[1] << 16 & 0x00FF0000) | (mp4_ctx->sps_str[2] << 8 & 0x0000FF00) | (mp4_ctx->sps_str[3] & 0x000000FF)),
					moov_buf + moov_position + 8); //configurationVersion=01; AVCProfileIndication; profile_compatibility; AVCLevelIndication
	Save32BigEndian(((0xFFE1 << 16) | (mp4_ctx->sps_len & 0x0000FFFF)),
					moov_buf + moov_position + 12); //6bits reserved + 2bits(lengthSizeMinusOne-1) +3bits reserved +5 bits numOfSequenceParameterSets
	memcpy(moov_buf + moov_position + 16, mp4_ctx->sps_str, mp4_ctx->sps_len);
	Save32BigEndian(((0x01 << 24) | (mp4_ctx->pps_len << 8 & 0x00FFFF00)), moov_buf + moov_position + 16 + mp4_ctx->sps_len);
	memcpy(moov_buf + moov_position + 16 + mp4_ctx->sps_len + 3, mp4_ctx->pps_str, mp4_ctx->pps_len);
	moov_position += size;
	return moov_position;
}

int avc1_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "avc1", 86);
	Save32BigEndian(1, moov_buf + moov_position + 12); //reserved (6Bytes) + data_reference_index(2Bytes) = 1
	Save32BigEndian(mp4_ctx->width << 16, moov_buf + moov_position + 32); //Track width, media player will use sps/pps rather than this
	Save32BigEndian(mp4_ctx->height << 16, moov_buf + moov_position + 34); //Track height
	Save32BigEndian(0x00480000, moov_buf + moov_position + 36); //horizresolution 72dpi
	Save32BigEndian(0x00480000, moov_buf + moov_position + 40); //vertresolution 72dpi
	//32bytes string: compressor name//leave empty
	Save32BigEndian(0x0018ffff, moov_buf + moov_position + 82); //0x0018 :depth, -1:pre_defined
	moov_position += size;
	size = avcC_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int hvcC_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)//»Ý­n­×§ï
{
	int size = 0;
	int moov_position = 0;
	int numOfNALArray = 0x03;//VPS SPS PPS //8 :Lenght+ name 23: Command config 5*3 :  array_completeness+nal_num+nal_length ,VPS SPS PPS LENGHT
	size = create_box(moov_buf + moov_position, "hvcC", 8 + 23 + 5 * 3 + mp4_ctx->vps_len + mp4_ctx->sps_len + mp4_ctx->pps_len);
	// 8  - bit version
	moov_buf[0 + 8]  = 1;
	//Profile space 2 bit, tier flag 1 bit and profile IDC 5 bit
	moov_buf[1 + 8]  = 0x00;
	// 32 - bit compatibility flag
	moov_buf[2 + 8]  = 0x00;
	moov_buf[3 + 8]  = 0x00;
	moov_buf[4 + 8]  = 0x00;
	moov_buf[5 + 8]  = 0x00;
	// 48 - bit general constraint indicator flag
	moov_buf[6 + 8]  = moov_buf[7 + 8]  = moov_buf[8 + 8]  = 0x00;
	moov_buf[9 + 8 + 8] = moov_buf[10 + 8] = moov_buf[11 + 8] = 0x00;
	// 8  - bit general IDC level
	moov_buf[12 + 8] = 0x00;
	// 4  - bit reserved '1111'
	// 12 - bit spatial segmentation idc
	moov_buf[13 + 8] = 0xf0;
	moov_buf[14 + 8] = 0x00;
	// 6  - bit reserved '111111'
	// 2  - bit parallelism Type
	moov_buf[15 + 8] = 0xfc;
	// 6  - bit reserved '111111'
	// 2  - bit chromaFormat
	moov_buf[16 + 8] = 0xfc;
	// 5  - bit reserved '11111'
	// 3  - bit DepthLumaMinus8
	moov_buf[17 + 8] = 0xf8;
	// 5  - bit reserved '11111'
	// 3  - bit DepthChromaMinus8
	moov_buf[18 + 8] = 0xf8;
	// 16 - bit average frame rate
	moov_buf[19 + 8] = moov_buf[20 + 8] = 0x00;
	// 2  - bit constant frame rate
	// 3  - bit num temporal layers
	// 1  - bit temoral nested
	// 2  - bit lengthSizeMinusOne
	moov_buf[21 + 8] = 0x07;

	moov_buf[22 + 8] = (uint8_t)numOfNALArray;
	int size_index = 23 + 8;
	for (int i = 0; i < numOfNALArray; i++) {
		if (i == 0) { //VPS//0X32
			//0xa0(1 byte)array_completeness(1)=1+reserved(1)=0+NAL_UNIT_TYPE(6)  1+0+100000 = 10100000
			//0x0001 (2 bytes)
			//vps len(2 bytes)
			//vps content
			moov_buf[size_index] = 0xa0;//23
			size_index += 1;
			moov_buf[size_index] = (0x0001 >> 8) & 0xff; //24
			size_index += 1;
			moov_buf[size_index] = 0x0001 & 0xff; //25
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->vps_len >> 8) & 0xff; //26
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->vps_len) & 0xff; //27
			size_index += 1;
			memcpy(moov_buf + size_index, mp4_ctx->vps_str, mp4_ctx->vps_len);
			size_index += mp4_ctx->vps_len;
			//moov_buf[23+8] = 0xa0;
			//moov_buf[24+8] = (0x0001>>8)&0xff;
			//moov_buf[25+8] = 0x0001&0xff;
			//moov_buf[26+8] = (vps_len>>8)&0xff;
			//moov_buf[27+8] = vps_len&0xff;
			//memcpy(moov_buf+28+8,vps_buf,vps_len);
		} else if (i == 1) { //SPS//0X33
			//0xa1 (1byte)array_completeness(1)=1+reserved(1)=0+NAL_UNIT_TYPE(6) 1+0+100001 = 10100001
			//0x0001 (2 bytes)
			//vps len(2 bytes)
			//vps content
			moov_buf[size_index] = 0xa1;//23
			size_index += 1;
			moov_buf[size_index] = (0x0001 >> 8) & 0xff; //24
			size_index += 1;
			moov_buf[size_index] = 0x0001 & 0xff; //25
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->sps_len >> 8) & 0xff; //26
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->sps_len) & 0xff; //27
			size_index += 1;
			memcpy(moov_buf + size_index, mp4_ctx->sps_str, mp4_ctx->sps_len);
			size_index += mp4_ctx->sps_len;//Original vps_len passed
			//moov_buf[23+8] = 0xa0;
			//moov_buf[24+8] = (0x0001>>8)&0xff;
			//moov_buf[25+8] = 0x0001&0xff;
			//moov_buf[26+8] = (sps_len>>8)&0xff;
			//moov_buf[27+8] = 0x0001&0xff;
		} else { //pps//0X34
			//0xa2 (1byte)array_completeness(1)=1+reserved(1)=0+NAL_UNIT_TYPE(6) 1+0+100010 = 10100010
			//array_completeness(1)=1+reserved(1)=0+NAL_UNIT_TYPE(6)
			//0x0001 (2 bytes)
			//vps len(2 bytes)
			//vps content
			moov_buf[size_index] = 0xa1;//23
			size_index += 1;
			moov_buf[size_index] = (0x0001 >> 8) & 0xff; //24
			size_index += 1;
			moov_buf[size_index] = 0x0001 & 0xff; //25
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->pps_len >> 8) & 0xff; //26
			size_index += 1;
			moov_buf[size_index] = (mp4_ctx->pps_len) & 0xff; //27
			size_index += 1;
			memcpy(moov_buf + size_index, mp4_ctx->pps_str, mp4_ctx->pps_len);
			size_index += mp4_ctx->pps_len;
			//moov_buf[23+8] = 0xa0;
			//moov_buf[24+8] = (0x0001>>8)&0xff;
			//moov_buf[25+8] = 0x0001&0xff;
			//moov_buf[26+8] = (sps_len>>8)&0xff;
			//moov_buf[27+8] = 0x0001&0xff;
		}
	}
	printf("hvcc %d %d\r\n", 8 + 23 + 5 * 3 + mp4_ctx->vps_len + mp4_ctx->sps_len + mp4_ctx->pps_len, size_index);
	//moov_position += size_index;
	moov_position = 8 + 23 + 5 * 3 + mp4_ctx->vps_len + mp4_ctx->sps_len + mp4_ctx->pps_len;
	return moov_position;
}

int hvc1_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "hvc1", 86);
	Save32BigEndian(1, moov_buf + moov_position + 12); //reserved (6Bytes) + data_reference_index(2Bytes) = 1
	Save32BigEndian(mp4_ctx->width << 16, moov_buf + moov_position + 32); //Track width, media player will use sps/pps rather than this
	Save32BigEndian(mp4_ctx->height << 16, moov_buf + moov_position + 34); //Track height
	Save32BigEndian(0x00480000, moov_buf + moov_position + 36); //horizresolution 72dpi
	Save32BigEndian(0x00480000, moov_buf + moov_position + 40); //vertresolution 72dpi
	//32bytes string: compressor name//leave empty
	Save32BigEndian(0x0018ffff, moov_buf + moov_position + 82); //0x0018 :depth, -1:pre_defined
	moov_position += size;
	size = hvcC_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int stsd_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "stsd", 16, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12); //Entry count = 1: only video-avc-h264
	moov_position += size;
	if (mp4_ctx->encoder_type == AV_CODEC_ID_H264) {
		size = avc1_box_video(moov_buf + moov_position, mp4_ctx);
	} else {
		size = hvc1_box_video(moov_buf + moov_position, mp4_ctx);
	}
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int stsd_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "stsd", 16, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12); //Entry count = 1: only video-avc-h264
	moov_position += size;
	//size = avc1_box(moov_buf+moov_position,root);
	size = mp4a_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}
#if ORIGINAL_VERSION
int stts_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int track_timescale = 30000;//the number of time units that pass per second
	//int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "stts", 24, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12); //entry count = 1
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 16); //Sample count
	Save32BigEndian(VIDEO_TRACK_TIME_SCALE / mp4_ctx->frame_rate, moov_buf + moov_position + 20); //Sample delta
	moov_position += size;
	return moov_position;
}
#else
int stts_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)//New version
{
	int size = 0;
	int moov_position = 0;
	//int track_timescale = 30000;//the number of time units that pass per second
	//int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "stts", 16 + 8 * mp4_ctx->root.video_len, 0, 0);
	//Save32BigEndian(1, moov_buf+moov_position+12);//entry count = 1
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 12);

	for (int i = 0; i < mp4_ctx->root.video_len; i++) {
		Save32BigEndian(1, moov_buf + moov_position + 16 + 8 * i); //Sample count
		//if(mp4_ctx->type == STORAGE_ALL){
		if (0) {
			if (i == 0) {
				Save32BigEndian(1, moov_buf + moov_position + 20 + 8 * i); //Sample delta
			} else {
				Save32BigEndian(mp4_ctx->video_timestamp_buffer[i] - mp4_ctx->video_timestamp_buffer[i - 1], moov_buf + moov_position + 20 + 8 * i); //Sample delta
			}
		} else {
			if (i == (mp4_ctx->root.video_len - 1)) {
				Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 2],
								moov_buf + moov_position + 20 + 8 * i);
			} else {
				if ((mp4_ctx->type == STORAGE_ALL) && (i == 0)) {
					if (mp4_ctx->video_timestamp_first > mp4_ctx->audio_timestamp_first) {
						int delta = ((mp4_ctx->video_timestamp_first - mp4_ctx->audio_timestamp_first) * mp4_ctx->video_clock_rate) / configTICK_RATE_HZ;
						//printf("video delta = %d\r\n",delta);
						Save32BigEndian((mp4_ctx->video_timestamp_buffer[i + 1] - mp4_ctx->video_timestamp_buffer[i]) + delta, moov_buf + moov_position + 20 + 8 * i);
					} else {
						Save32BigEndian(mp4_ctx->video_timestamp_buffer[i + 1] - mp4_ctx->video_timestamp_buffer[i], moov_buf + moov_position + 20 + 8 * i);
					}
				} else {
					Save32BigEndian(mp4_ctx->video_timestamp_buffer[i + 1] - mp4_ctx->video_timestamp_buffer[i], moov_buf + moov_position + 20 + 8 * i);
				}

			}
		}
		//Save32BigEndian(mp4_ctx->video_timestamp_buffer[i], moov_buf+moov_position+20+8*i);//Sample delta
	}
	moov_position += size;
	return moov_position;
}
#endif
#if ORIGINAL_VERSION
int stts_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int track_timescale = 30000;//the number of time units that pass per second
	//int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "stts", 24, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12); //entry count = 1
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 16); //Sample count
	//Save32BigEndian(track_timescale/fps, moov_buf+moov_position+20);//Sample delta
	Save32BigEndian((1024 * 1000) / mp4_ctx->sample_rate, moov_buf + moov_position + 20); //Save32BigEndian(23, moov_buf+moov_position+20);//Sample delta
	moov_position += size;
	return moov_position;
}
#else
int stts_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)//New version
{
	int size = 0;
	int moov_position = 0;
	//int track_timescale = 30000;//the number of time units that pass per second
	//int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "stts", 16 + 8 * mp4_ctx->root.audio_len, 0, 0);
	//Save32BigEndian(1, moov_buf+moov_position+12);//entry count = 1
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 12);
#if 0
	for (int i = 0; i < mp4_ctx->root.audio_len; i++) {
		Save32BigEndian(1, moov_buf + moov_position + 16 + 8 * i); //Sample count
		if (i == 0) {
			Save32BigEndian(1024, moov_buf + moov_position + 20 + 8 * i);
		} else {
			Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i] - mp4_ctx->audio_timestamp_buffer[i - 1], moov_buf + moov_position + 20 + 8 * i);
		}
		//Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i], moov_buf+moov_position+20+8*i);//Sample delta
	}
#else
	for (int i = 0; i < mp4_ctx->root.audio_len; i++) {
		Save32BigEndian(1, moov_buf + moov_position + 16 + 8 * i); //Sample count
		//if(mp4_ctx->type == STORAGE_ALL){
		if (0) {
			if (i <= AUDIO_SKIP_FRAME) { // 10 FOR FIRST 4 FOR SECOND
				Save32BigEndian(0, moov_buf + moov_position + 20 + 8 * i);
			} else {
				Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i] - mp4_ctx->audio_timestamp_buffer[i - 1], moov_buf + moov_position + 20 + 8 * i);
			}
		} else {
			if (i == (mp4_ctx->root.audio_len - 1)) {
				Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 1] - mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 2],
								moov_buf + moov_position + 20 + 8 * i);
			} else {
				if ((mp4_ctx->type == STORAGE_ALL) && (i == 0)) {
					if (mp4_ctx->video_timestamp_first < mp4_ctx->audio_timestamp_first) {
						int delta = ((mp4_ctx->audio_timestamp_first - mp4_ctx->video_timestamp_first) * mp4_ctx->audio_clock_rate) / configTICK_RATE_HZ;
						//printf("audio delta = %d\r\n",delta);
						Save32BigEndian((mp4_ctx->audio_timestamp_buffer[i + 1] - mp4_ctx->audio_timestamp_buffer[i]) + delta, moov_buf + moov_position + 20 + 8 * i);
					} else {
						Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i + 1] - mp4_ctx->audio_timestamp_buffer[i], moov_buf + moov_position + 20 + 8 * i);
					}
				} else {
					Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i + 1] - mp4_ctx->audio_timestamp_buffer[i], moov_buf + moov_position + 20 + 8 * i);
				}
			}
			//Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i]-mp4_ctx->audio_timestamp_buffer[i-1], moov_buf+moov_position+20+8*i);
		}
		//Save32BigEndian(mp4_ctx->audio_timestamp_buffer[i], moov_buf+moov_position+20+8*i);//Sample delta
	}
#endif
	moov_position += size;
	return moov_position;
}
#endif

int stss_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	int i = 0;
	size = create_box_full(moov_buf + moov_position, "stss", 16 + 4 * mp4_ctx->root.keyindex, 0, 1);
	Save32BigEndian(mp4_ctx->root.keyindex, moov_buf + moov_position + 12);
	for (i = 0; i < mp4_ctx->root.keyindex; i++) {
		Save32BigEndian(mp4_ctx->key_frame[i], moov_buf + moov_position + 16 + i * 4);
	}
	moov_position += size;
	return moov_position;
}
#if 1//ORIGINAL_VERSION //s
int stsc_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	//int chunk_count = 1;//only one chunk
	size = create_box_full(moov_buf + moov_position, "stsc", 16 + 12 * mp4_ctx->root.video_len, 0, 1);
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 12);
	for (int i = 0; i < mp4_ctx->root.video_len; i++) {
		Save32BigEndian(i + 1, moov_buf + moov_position + 16 + 12 * i); //first_chunk
		Save32BigEndian(1, moov_buf + moov_position + 20 + 12 * i); //samples_per_chunk;
		Save32BigEndian(1, moov_buf + moov_position + 24 + 12 * i); //sample_description_index;
	}
	moov_position += size;
	return moov_position;
}
#else
int stsc_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)// For new version
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	//int chunk_count = 1;//only one chunk
	size = create_box_full(moov_buf + moov_position, "stsc", 16 + 12 * 1, 0, 1);
	Save32BigEndian(1, moov_buf + moov_position + 12);

	Save32BigEndian(1, moov_buf + moov_position + 16); //first_chunk
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 20); //samples_per_chunk;
	Save32BigEndian(1, moov_buf + moov_position + 24); //sample_description_index;

	moov_position += size;
	return moov_position;
}
#endif
#if 1//ORIGINAL_VERSION
int stsc_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	//int chunk_count = 1;//only one chunk
	size = create_box_full(moov_buf + moov_position, "stsc", 16 + 12 * mp4_ctx->root.audio_len, 0, 1);
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 12);
	for (int i = 0; i < mp4_ctx->root.audio_len; i++) {
		Save32BigEndian(i + 1, moov_buf + moov_position + 16 + 12 * i); //first_chunk
		Save32BigEndian(1, moov_buf + moov_position + 20 + 12 * i); //samples_per_chunk;
		Save32BigEndian(1, moov_buf + moov_position + 24 + 12 * i); //sample_description_index;
	}
	moov_position += size;
	return moov_position;
}
#else
int stsc_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)// For new version
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	//int chunk_count = 1;//only one chunk
	size = create_box_full(moov_buf + moov_position, "stsc", 16 + 12 * 1, 0, 1);
	Save32BigEndian(1, moov_buf + moov_position + 12);

	Save32BigEndian(1, moov_buf + moov_position + 16); //first_chunk
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 20); //samples_per_chunk;
	Save32BigEndian(1, moov_buf + moov_position + 24); //sample_description_index;

	moov_position += size;
	return moov_position;
}
#endif

int stsz_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	size = create_box_full(moov_buf + moov_position, "stsz", 20 + 4 * mp4_ctx->root.video_len, 0, 0);
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 16); //sample_count
	for (int i = 0; i < mp4_ctx->root.video_len; i++) {
		//Save32BigEndian(video_buffer[MP4_SIZE][i], moov_buf+moov_position+20+4*i);//entry_size//sample size
		Save32BigEndian(mp4_ctx->video_buffer_size[i], moov_buf + moov_position + 20 + 4 * i); //entry_size//sample size
	}
	moov_position += size;
	return moov_position;
}

int stsz_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int i = 0;
	size = create_box_full(moov_buf + moov_position, "stsz", 20 + 4 * mp4_ctx->root.audio_len, 0, 0);
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 16); //sample_count
	for (int i = 0; i < mp4_ctx->root.audio_len; i++) {
		//Save32BigEndian(audio_buffer[MP4_SIZE][i], moov_buf+moov_position+20+4*i);//entry_size//sample size
		Save32BigEndian(mp4_ctx->audio_buffer_size[i], moov_buf + moov_position + 20 + 4 * i); //entry_size//sample size
	}
	moov_position += size;
	return moov_position;
}

#if 1//ORIGINAL_VERSION
int stco_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int accu = 24+8;//ftype + mdat header
	size = create_box_full(moov_buf + moov_position, "stco", 16 + 4 * mp4_ctx->root.video_len, 0, 0);
	Save32BigEndian(mp4_ctx->root.video_len, moov_buf + moov_position + 12);
	for (int i = 0; i < mp4_ctx->root.video_len; i++) {
		//Save32BigEndian(video_buffer[MP4_INDEX][i], moov_buf+moov_position+16+4*i);//chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)
		Save32BigEndian(mp4_ctx->video_buffer_index[i], moov_buf + moov_position + 16 + 4 *
						i); //chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)
	}
	moov_position += size;
	return moov_position;
}
#else
int stco_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)// For new version
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "stco", 16 + 4 * 1, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12);

	//Save32BigEndian(video_buffer[MP4_INDEX][i], moov_buf+moov_position+16+4*i);//chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)
	Save32BigEndian(mp4_ctx->video_buffer_index[0], moov_buf + moov_position +
					16); //chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)

	moov_position += size;
	return moov_position;
}
#endif
#if 1//ORIGINAL_VERSION
int stco_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//int accu = 24+8;//ftype + mdat header
	size = create_box_full(moov_buf + moov_position, "stco", 16 + 4 * mp4_ctx->root.audio_len, 0, 0);
	Save32BigEndian(mp4_ctx->root.audio_len, moov_buf + moov_position + 12);
	for (int i = 0; i < mp4_ctx->root.audio_len; i++) {
		//Save32BigEndian(audio_buffer[MP4_INDEX][i], moov_buf+moov_position+16+4*i);//chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)
		Save32BigEndian(mp4_ctx->audio_buffer_index[i], moov_buf + moov_position + 16 + 4 *
						i); //chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)

	}
	moov_position += size;
	return moov_position;
}
#else
int stco_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)// For new version
{
	int size = 0;
	int moov_position = 0;
	size = create_box_full(moov_buf + moov_position, "stco", 16 + 4 * 1, 0, 0);
	Save32BigEndian(1, moov_buf + moov_position + 12);

	//Save32BigEndian(video_buffer[MP4_INDEX][i], moov_buf+moov_position+16+4*i);//chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)
	Save32BigEndian(mp4_ctx->audio_buffer_index[0], moov_buf + moov_position +
					16); //chunk offset, first chunk start offset = ftyp.boxsize(24) + moov.boxsize + mdat.datasize(8)

	moov_position += size;
	return moov_position;
}
#endif

int stbl_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "stbl", 8);
	moov_position += size;
	size = stsd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stts_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stss_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stsc_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stsz_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stco_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int stbl_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "stbl", 8);
	moov_position += size;
	size = stsz_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stsd_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	size = stts_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stsc_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stco_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}



int minf_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "minf", 8);
	moov_position += size;
	size = vmhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = dinf_box(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stbl_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int minf_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "minf", 8);
	moov_position += size;
	//size = vmhd_box(moov_buf+moov_position,root);
	size = smhd_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = dinf_box(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = stbl_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int mdhd_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int track_timescale = 30000;//the number of time units that pass per second
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "mdhd", 32, 0, 1);
	//Save32BigEndian(0, mdhd->data+8);//Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, mdhd->data+12);//Creation Time
	//Save32BigEndian(present_time, mdhd->data+16);//Modification Time
#if ORIGINAL_VERSION
	Save32BigEndian(VIDEO_TRACK_TIME_SCALE, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->root.video_len * VIDEO_TRACK_TIME_SCALE / mp4_ctx->frame_rate, moov_buf + moov_position + 24); //Duration, will get filled later
#else
	Save32BigEndian(mp4_ctx->video_clock_rate, moov_buf + moov_position + 20);
	Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[0], moov_buf + moov_position + 24);
#endif
	Save32BigEndian(0x55c40000, moov_buf + moov_position + 28); //Language(2Bytes)==undetermined('u'-0x60, 'n'-0x60, 'd'-0x60) + Quality(2Bytes) == 0
	moov_position += size;
	return moov_position;
}

int mdhd_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int track_timescale = 44100;//the number of time units that pass per second
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "mdhd", 32, 0, 1);
	//Save32BigEndian(0, mdhd->data+8);//Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, mdhd->data+12);//Creation Time
	//Save32BigEndian(present_time, mdhd->data+16);//Modification Time
#if ORIGINAL_VERSION
	Save32BigEndian(MEDIA_TIME_SCALE, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->root.audio_len * 1000 * 1024 / mp4_ctx->sample_rate,
					moov_buf + moov_position + 24); //Save32BigEndian(44628, moov_buf+moov_position+24);//Duration, will get filled later
#else
	Save32BigEndian(mp4_ctx->audio_clock_rate, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 1] - mp4_ctx->audio_timestamp_buffer[0], moov_buf + moov_position + 24);
#endif
	Save32BigEndian(0x55c40000, moov_buf + moov_position + 28); //Language(2Bytes)==undetermined('u'-0x60, 'n'-0x60, 'd'-0x60) + Quality(2Bytes) == 0
	moov_position += size;
	return moov_position;
}

int mdia_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "mdia", 8);
	moov_position += size;
	size = mdhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = hdlr_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = minf_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int mdia_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "mdia", 8);
	moov_position += size;
	size = mdhd_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = hdlr_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = minf_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int mvhd_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int media_timescale = 600;
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "mvhd", 108, 0, 0);
	//Save32BigEndian(0, mvhd->data+8);//Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, mvhd->data+12);//Creation Time
	//Save32BigEndian(present_time, mvhd->data+16);//Modification Time
#if 0
#if 0
	Save32BigEndian(MEDIA_TIME_SCALE, moov_buf + moov_position + 20); //the number of time units that pass per second
#else
	Save32BigEndian(mp4_ctx->video_clock_rate, moov_buf + moov_position + 20);
#endif
#if 0
	Save32BigEndian(mp4_ctx->root.video_len * MEDIA_TIME_SCALE / mp4_ctx->frame_rate, moov_buf + moov_position + 24); //Duration
#else
	Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[0], moov_buf + moov_position + 24);
#endif
#endif

#if ORIGINAL_VERSION
	Save32BigEndian(MEDIA_TIME_SCALE, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->root.video_len * MEDIA_TIME_SCALE / mp4_ctx->frame_rate, moov_buf + moov_position + 24); //Duration
#else
	if (mp4_ctx->type == STORAGE_ALL) {
		Save32BigEndian(mp4_ctx->video_clock_rate, moov_buf + moov_position + 20);
		Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[0], moov_buf + moov_position + 24);
		//Save32BigEndian(mp4_ctx->audio_clock_rate, moov_buf+moov_position+20);
		//Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len-1]-mp4_ctx->audio_timestamp_buffer[0], moov_buf+moov_position+24);
	} else {
		Save32BigEndian(mp4_ctx->video_clock_rate, moov_buf + moov_position + 20);
		Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len - 1] - mp4_ctx->video_timestamp_buffer[0], moov_buf + moov_position + 24);
	}
	//Save32BigEndian(mp4_ctx->audio_clock_rate, moov_buf+moov_position+20);
	//Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len-1]-mp4_ctx->audio_timestamp_buffer[0], moov_buf+moov_position+24);
	//Save32BigEndian(mp4_ctx->video_clock_rate, moov_buf+moov_position+20);
	//Save32BigEndian(mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len-1]-mp4_ctx->video_timestamp_buffer[0], moov_buf+moov_position+24);
#endif
	Save32BigEndian(0x00010000, moov_buf + moov_position + 28); //Preferred Rate -> normal rate
	Save32BigEndian(0x01000000, moov_buf + moov_position + 32); //Preferred Volume(2Bytes)+Reserved(10Bytes)
	//long unsigned int matrix[9]= { 0x00010000,0,0,0,0x00010000,0,0,0,0x40000000 };
	Save32BigEndian(0x00010000, moov_buf + moov_position + 44); //Matrix structure
	Save32BigEndian(0x00010000, moov_buf + moov_position + 60); //Matrix structure
	Save32BigEndian(0x40000000, moov_buf + moov_position + 76); //Matrix structure
#if 0
	Save32BigEndian(0x00000002, moov_buf + moov_position + 104); //Next Track ID = 2
#else
	Save32BigEndian(0x00000003, moov_buf + moov_position + 104); //Next Track ID = 2
#endif
	moov_position += size;
	return size;
}

int mvhd_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	//unsigned int audio_media_timescale = 1000;
	//unsigned int fps = 30;//frame per second
	size = create_box_full(moov_buf + moov_position, "mvhd", 108, 0, 0);
	//Save32BigEndian(0, mvhd->data+8);//Version(1Byte)+Flags(3Bytes)
	//Save32BigEndian(present_time, mvhd->data+12);//Creation Time
	//Save32BigEndian(present_time, mvhd->data+16);//Modification Time
#if ORIGINAL_VERSION
	Save32BigEndian(MEDIA_TIME_SCALE, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->root.audio_len * 1000 * 1024 / mp4_ctx->sample_rate, moov_buf + moov_position + 24); //Duration
#else
	Save32BigEndian(mp4_ctx->audio_clock_rate, moov_buf + moov_position + 20); //the number of time units that pass per second
	Save32BigEndian(mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len - 1] - mp4_ctx->audio_timestamp_buffer[0], moov_buf + moov_position + 24); //Duration
#endif
	Save32BigEndian(0x00010000, moov_buf + moov_position + 28); //Preferred Rate -> normal rate
	Save32BigEndian(0x01000000, moov_buf + moov_position + 32); //Preferred Volume(2Bytes)+Reserved(10Bytes)
	//long unsigned int matrix[9]= { 0x00010000,0,0,0,0x00010000,0,0,0,0x40000000 };
	Save32BigEndian(0x00010000, moov_buf + moov_position + 44); //Matrix structure
	Save32BigEndian(0x00010000, moov_buf + moov_position + 60); //Matrix structure
	Save32BigEndian(0x40000000, moov_buf + moov_position + 76); //Matrix structure
	Save32BigEndian(0x00000002, moov_buf + moov_position + 104); //Next Track ID = 2
	moov_position += size;
	return size;
}

int trak_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "trak", 8);
	moov_position += size;
	size = tkhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = mdia_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int trak_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "trak", 8);
	moov_position += size;
	size = tkhd_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = mdia_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int Set_moov_box_video(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "moov", 8);
	moov_position += size;
	size = mvhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int Set_moov_box_all(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "moov", 8);
	moov_position += size;
#if 0
	size = mvhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
#else
	size = mvhd_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_video(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
#endif

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int Set_moov_box_audio(unsigned char *moov_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int moov_position = 0;
	size = create_box(moov_buf + moov_position, "moov", 8);
	moov_position += size;
	size = mvhd_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;
	size = trak_box_audio(moov_buf + moov_position, mp4_ctx);
	moov_position += size;

	update_box_size(mp4_ctx, moov_buf, moov_position);
	return moov_position;
}

int Set_ftyp_box(unsigned char *ftyp_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	size = create_box(ftyp_buf, "ftyp", 24);
	strncpy((char *)(ftyp_buf + 8), "isom", strlen("isom"));
	//strcpy(ftyp->data+8, "isom");//Major brand
	Save32BigEndian(1, ftyp_buf + 12); //Minor version
	//strcpy(ftyp->data+16, "isomavc1");//Compatible brands: isom, avc1
	if (mp4_ctx->encoder_type == AV_CODEC_ID_H264) {
		strncpy((char *)(ftyp_buf + 16), "isomavc1", strlen("isomavc1"));
	} else {
		strncpy((char *)(ftyp_buf + 16), "isomhvc1", strlen("isomhvc1"));
	}
	return size;
}

int Set_mdat_box(unsigned char *mda_buf)
{
	int size = 0;
	size = create_box(mda_buf, "mdat", 8);
	return size;
}
////////////////////////////////////////////////
#if 0
void mp4_free_buffer(pmp4_context mp4_ctx)
{
	if (mp4_ctx->fatfs_buf != NULL) {
		free(mp4_ctx->fatfs_buf);
	}

	if (mp4_ctx->video_buffer_index != NULL) {
		free(mp4_ctx->video_buffer_index);
	}
	if (mp4_ctx->video_buffer_size != NULL) {
		free(mp4_ctx->video_buffer_size);
	}
	if (mp4_ctx->key_frame != NULL) {
		free(mp4_ctx->key_frame);
	}

	if (mp4_ctx->audio_buffer_index != NULL) {
		free(mp4_ctx->audio_buffer_index);
	}
	if (mp4_ctx->audio_buffer_size != NULL) {
		free(mp4_ctx->audio_buffer_size);
	}

	if (mp4_ctx->moov_box != NULL) {
		free(mp4_ctx->moov_box);
	}

	if (mp4_ctx->audio_timestamp_buffer != NULL) {
		free(mp4_ctx->audio_timestamp_buffer);
	}
	if (mp4_ctx->video_timestamp_buffer != NULL) {
		free(mp4_ctx->video_timestamp_buffer);
	}
}
#else
void mp4_free_buffer(pmp4_context mp4_ctx)
{
	if (mp4_ctx->fatfs_buf != NULL) {
		free(mp4_ctx->fatfs_buf);
		mp4_ctx->fatfs_buf = NULL;
	}

	if (mp4_ctx->video_buffer_index != NULL) {
		free(mp4_ctx->video_buffer_index);
		mp4_ctx->video_buffer_index = NULL;
	}
	if (mp4_ctx->video_buffer_size != NULL) {
		free(mp4_ctx->video_buffer_size);
		mp4_ctx->video_buffer_size = NULL;
	}
	if (mp4_ctx->key_frame != NULL) {
		free(mp4_ctx->key_frame);
		mp4_ctx->key_frame = NULL;
	}
	if (mp4_ctx->audio_buffer_index != NULL) {
		free(mp4_ctx->audio_buffer_index);
		mp4_ctx->audio_buffer_index = NULL;
	}
	if (mp4_ctx->audio_buffer_size != NULL) {
		free(mp4_ctx->audio_buffer_size);
		mp4_ctx->audio_buffer_size = NULL;
	}

	if (mp4_ctx->moov_box != NULL) {
		free(mp4_ctx->moov_box);
		mp4_ctx->moov_box = NULL;
	}

	if (mp4_ctx->audio_timestamp_buffer != NULL) {
		free(mp4_ctx->audio_timestamp_buffer);
		mp4_ctx->audio_timestamp_buffer = NULL;
	}
	if (mp4_ctx->video_timestamp_buffer != NULL) {
		free(mp4_ctx->video_timestamp_buffer);
		mp4_ctx->video_timestamp_buffer = NULL;
	}
}
#endif

//static int init_num = 0;
int mp4_init_buffer(pmp4_context mp4_ctx)
{
	int rec_time_sec = 0;;
	mp4_free_buffer(mp4_ctx);

	if (mp4_ctx->fatfs_buf_size == 0) {
		printf("ERROR: mp4_ctx->fatfs_buf_size not set\n\r");
		goto error_exit;
	} else if (mp4_ctx->fatfs_buf_size % (32 * 1024) != 0) {
		printf("ERROR: mp4_ctx->fatfs_buf_size must be 32kB multiple\n\r");
		goto error_exit;
	}

	mp4_ctx->fatfs_buf = (unsigned char *)malloc(mp4_ctx->fatfs_buf_size);
	if (mp4_ctx->fatfs_buf == NULL) {
		printf("mp4_init_buffer error: allocate mp4_ctx->fatfs_buf fail\r\n");
		goto error_exit;
	}

	if (mp4_ctx->period_time == 0) {
		printf("ERROR: mp4_ctx->period_time not set\n\r");
		goto error_exit;
	}
	rec_time_sec = mp4_ctx->period_time;

	if (mp4_ctx->type == STORAGE_ALL || mp4_ctx->type == STORAGE_VIDEO) {
		mp4_ctx->video_size = mp4_ctx->frame_rate * rec_time_sec;
		mp4_ctx->video_buffer_index = (unsigned int *)malloc(mp4_ctx->video_size * sizeof(unsigned int));
		if (mp4_ctx->video_buffer_index == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->video_buffer_index fail\r\n");
			goto error_exit;
		}
		mp4_ctx->video_buffer_size = (unsigned int *)malloc(mp4_ctx->video_size * sizeof(unsigned int));
		if (mp4_ctx->video_buffer_size == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->video_buffer_size fail\r\n");
			goto error_exit;
		}

		mp4_ctx->key_frame_size = (mp4_ctx->video_size + (mp4_ctx->gop - 1)) / mp4_ctx->gop;
		mp4_ctx->key_frame = (unsigned int *)malloc(mp4_ctx->key_frame_size * sizeof(unsigned int));
		if (mp4_ctx->key_frame == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->key_frame fail\r\n");
			goto error_exit;
		}

		mp4_ctx->video_timestamp_buffer = (unsigned int *)malloc(mp4_ctx->video_size * sizeof(unsigned int));
		if (mp4_ctx->video_timestamp_buffer == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->video_timestamp_buffer fail\r\n");
			goto error_exit;
		}
	}

	if (mp4_ctx->type == STORAGE_ALL || mp4_ctx->type == STORAGE_AUDIO) {
		// AAC encodes 1024 samples at a time
		mp4_ctx->audio_size = ((mp4_ctx->sample_rate + 1023) / 1024) * rec_time_sec;
		mp4_ctx->audio_buffer_index = (unsigned int *)malloc(mp4_ctx->audio_size * sizeof(unsigned int));
		if (mp4_ctx->audio_buffer_index == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->audio_buffer_index fail\r\n");
			goto error_exit;
		}
		mp4_ctx->audio_buffer_size = (unsigned int *)malloc(mp4_ctx->audio_size * sizeof(unsigned int));
		if (mp4_ctx->audio_buffer_size == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->audio_buffer_size fail\r\n");
			goto error_exit;
		}

		mp4_ctx->audio_timestamp_buffer = (unsigned int *)malloc(mp4_ctx->audio_size * sizeof(unsigned int));
		if (mp4_ctx->audio_timestamp_buffer == NULL) {
			printf("mp4_init_buffer error: allocate mp4_ctx->audio_timestamp_buffer fail\r\n");
			goto error_exit;
		}
	}

	// Set_moov_box_all = 8 + mvhd_box_video (108) + trak_box_video (650 + 4* key_frame_size + 20 * mp4_ctx->video_size)
	//												+ trak_box_audio (474 + 20 * audio_size)
	// Set_moov_box_video = 8 + mvhd_box_video(108) + trak_box_video (650 + 4* key_frame_size + 20 * mp4_ctx->video_size)
	// Set_moov_box_audio = 8 + mvhd_box_audio(108) + trak_box_audio (474 + 20 * audio_size)
	if (mp4_ctx->type == STORAGE_ALL) {
		mp4_ctx->moov_box_size = 1240 + 4 * mp4_ctx->key_frame_size + 28 * mp4_ctx->video_size + 28 * mp4_ctx->audio_size + 1024 * 100;
	} else if (mp4_ctx->type == STORAGE_VIDEO) {
		mp4_ctx->moov_box_size = 766 + 4 * mp4_ctx->key_frame_size + 28 * mp4_ctx->video_size;
	} else if (mp4_ctx->type == STORAGE_AUDIO) {
		mp4_ctx->moov_box_size = 590 + 28 * mp4_ctx->audio_size;
	}

	mp4_ctx->moov_box = (unsigned char *)malloc(mp4_ctx->moov_box_size);
	if (mp4_ctx->moov_box == NULL) {
		printf("mp4_init_buffer error: allocate mp4_ctx->moov_box fail\r\n");
		goto error_exit;
	}

	//printf("\n\rmp4_init_buffer: video_size=%d, key_frame_size=%d\n\r",mp4_ctx->video_size,mp4_ctx->key_frame_size);
	//printf("\n\rmp4_init_buffer: moov_box_size=%d, fatfs_buf_size=%d, audio_size=%d\n\r",mp4_ctx->moov_box_size,mp4_ctx->fatfs_buf_size, mp4_ctx->audio_size);
	return 0;
error_exit:
	mp4_free_buffer(mp4_ctx);
	return -1;

}

int mp4_moov_start(pmp4_context mp4_ctx)
{
	int ret = 0;
	int bw = 0;
	memset(mp4_ctx->fatfs_buf, 0, mp4_ctx->fatfs_buf_size);
	memset(mp4_ctx->video_buffer_index, 0, mp4_ctx->video_size * sizeof(unsigned int));
	memset(mp4_ctx->video_buffer_size, 0, mp4_ctx->video_size * sizeof(unsigned int));
	memset(mp4_ctx->key_frame, 0, mp4_ctx->key_frame_size * sizeof(unsigned int));
	memset(mp4_ctx->audio_buffer_index, 0, mp4_ctx->audio_size * sizeof(unsigned int));
	memset(mp4_ctx->audio_buffer_size, 0, mp4_ctx->audio_size * sizeof(unsigned int));
	memset(mp4_ctx->moov_box, 0, mp4_ctx->moov_box_size);
	memset(mp4_ctx->ftyp_box, 0, sizeof(mp4_ctx->ftyp_box));
	memset(mp4_ctx->mdat_box, 0, sizeof(mp4_ctx->mdat_box));
	memset(mp4_ctx->sps_str, 0, sizeof(mp4_ctx->sps_str));
	memset(mp4_ctx->pps_str, 0, sizeof(mp4_ctx->pps_str));

	//setting clock rate

	mp4_ctx->audio_clock_rate = mp4_ctx->sample_rate;
	mp4_ctx->video_clock_rate = 90000;
	mp4_ctx->video_timestamp_first = 0;
	mp4_ctx->audio_timestamp_first = 0;

	mp4_ctx->fatfs_buf_pos = 0;
	//Verify
	mp4_ctx->audio_appear_first = 0;
	mp4_ctx->video_appear_first = 0;
	mp4_ctx->audio_start = 0;
	mp4_ctx->video_start = 0;

	mp4_ctx->moov_size = 0;
	mp4_ctx->sps_pps_mark = 0;
	init_mp4_root(&mp4_ctx->root);
	Set_ftyp_box(mp4_ctx->ftyp_box, mp4_ctx);
	Set_mdat_box(mp4_ctx->mdat_box);
	if (mp4_ctx->cb_fwrite) {
		ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, mp4_ctx->ftyp_box, sizeof(mp4_ctx->ftyp_box), (u32 *)&bw);
	} else {
		ret = fatfs_write(mp4_ctx->ftyp_box, sizeof(mp4_ctx->ftyp_box), mp4_ctx, 0, 0);
	}
	if (ret != FR_OK) {
		return -1;
	}
	if (mp4_ctx->cb_fwrite) {
		ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, mp4_ctx->mdat_box, sizeof(mp4_ctx->mdat_box), (u32 *)&bw);
	} else {
		ret = fatfs_write(mp4_ctx->mdat_box, sizeof(mp4_ctx->mdat_box), mp4_ctx, 0, 0);
	}
	if (ret != FR_OK) {
		return -1;
	}
	return 0;
}

int mp4_moov_end(pmp4_context mp4_ctx, int type)
{
	UINT bw = 0;
	unsigned int temp = 0;
	int ret = 0;
	switch (type) {
	case STORAGE_ALL:
		Set_moov_box_all(mp4_ctx->moov_box, mp4_ctx);
		break;
	case STORAGE_VIDEO:
		Set_moov_box_video(mp4_ctx->moov_box, mp4_ctx);
		break;
	case STORAGE_AUDIO:
		Set_moov_box_audio(mp4_ctx->moov_box, mp4_ctx);
		break;
	default:
		printf("not support type = %d\r\n", type);
		break;
	}
	if (mp4_ctx->cb_fwrite) {
		ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, mp4_ctx->moov_box, mp4_ctx->moov_size, &bw);
	} else {
		ret = fatfs_write(mp4_ctx->moov_box, mp4_ctx->moov_size, mp4_ctx, 0, 0);
	}
	if (ret != FR_OK) {
		return -1;
	}
	if (mp4_ctx->cb_fwrite) {
		//No need to flush
	} else {
		ret = fatfs_write_flush(mp4_ctx);
	}
	if (ret != FR_OK) {
		return -1;
	}
	if (mp4_ctx->cb_fseek) {
		ret = mp4_ctx->cb_fseek(&mp4_ctx->m_file, 0x18);
	} else {
		ret = f_lseek(&mp4_ctx->m_file, 0x18);
	}
	if (ret != FR_OK) {
		return -1;
	}
	temp = mp4_ctx->root.total;
	Save32BigEndian(mp4_ctx->root.total + MDAT_HEADER_SIZE, &mp4_ctx->root.total);
#if 0
	ret = f_write(&mp4_ctx->m_file, &mp4_ctx->root.total, sizeof(int), &bw);
	if (ret != FR_OK) {
		return -1;
	}
#else
	if (mp4_ctx->sd_card_is_mounted_cb) {
		if (mp4_ctx->sd_card_is_mounted_cb()) {
			if (mp4_ctx->cb_fwrite) {
				ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, &mp4_ctx->root.total, sizeof(int), &bw);
			} else {
				ret = f_write(&mp4_ctx->m_file, &mp4_ctx->root.total, sizeof(int), &bw);
				if (bw != sizeof(int)) {
					printf("bw %d mp4_ctx->fatfs_buf_size %d\r\n", bw, sizeof(int));
					return -1;
				}
			}
			if (ret != FR_OK) {
				return -1;
			}
		} else {
			printf("The sd card is removed\r\n");
			return -1;
		}
	} else {
		if (mp4_ctx->cb_fwrite) {
			ret = mp4_ctx->cb_fwrite(&mp4_ctx->m_file, &mp4_ctx->root.total, sizeof(int), &bw);
		} else {
			ret = f_write(&mp4_ctx->m_file, &mp4_ctx->root.total, sizeof(int), &bw);
			if (bw != sizeof(int)) {
				printf("bw %d mp4_ctx->fatfs_buf_size %d\r\n", bw, sizeof(int));
				return -1;
			}
		}
		if (ret != FR_OK) {
			return -1;
		}
	}
#endif
	mp4_ctx->root.total = temp;
	return 0;
}

int mp4_close_file(pmp4_context mp4_ctx)
{
	//fatfs_write_flush(mp4_ctx);
	int ret = 0;
	if (mp4_ctx->cb_fclose) {
		ret = mp4_ctx->cb_fclose(&mp4_ctx->m_file);
	} else {
		ret = f_close(&mp4_ctx->m_file);
	}
	if (ret != FR_OK) {
		return -1;
	}
	mp4_ctx->file_name_index++;
	return 0;
}

u8 mp4_is_recording(pmp4_context mp4_ctx)
{
	if (mp4_ctx->storage_state == STORAGE_IDLE) {
		return 0;
	} else {
		return 1;
	}
}

int mp4_start_record(pmp4_context mp4_ctx, int file_num)
{
	if (mp4_ctx->mp4_user_callback) {
		if (!mp4_ctx->cb_fclose) {
			printf("It need to add the cb_fclose\r\n");
			return -1;
		}
		if (!mp4_ctx->cb_fopen) {
			printf("It need to add the cb_fopen\r\n");
			return -1;
		}
		if (!mp4_ctx->cb_fseek) {
			printf("It need to add the cb_fseek\r\n");
			return -1;
		}
		if (!mp4_ctx->cb_fwrite) {
			printf("It need to add the cb_fwrite\r\n");
			return -1;
		}
	}
	if (file_num == 0) {
		printf("[Warning] mp4_start_record: file_num = 0\n\r");
		return -1;
	} else if (mp4_is_recording(mp4_ctx)) {
		printf("[Error] MP4 is recording\r\n");
		return -1;
	} else {
		printf("Start MP4 recording (%d files)\r\n", file_num);
		mp4_ctx->file_total = mp4_ctx->file_name_index + file_num;
		mp4_ctx->storage_state = STORAGE_INIT;
		return 0;
	}
}

int mp4_stop_record(pmp4_context mp4_ctx)
{
	if (mp4_is_recording(mp4_ctx)) {
		printf("Stop MP4 recording\r\n");
		mp4_ctx->file_total = 0;
		mp4_ctx->loop_mode = 0;
		mp4_ctx->storage_state = STORAGE_STOP;
		return 0;
	} else {
		printf("[Error] MP4 is not recording\r\n");
		return -1;
	}
}

void mp4_set_timestamp(pmp4_context mp4_ctx, u32 current_clock_tick, u32 type)
{
	u32 delta_clock_tick;

	if (type == AV_CODEC_ID_H264) {
		delta_clock_tick = current_clock_tick - mp4_ctx->video_old_depend_clock_tick;
		mp4_ctx->video_old_depend_clock_tick = current_clock_tick;
		mp4_ctx->video_timestamp += (delta_clock_tick * mp4_ctx->video_clock_rate) / configTICK_RATE_HZ;
		if (mp4_ctx->video_size <= mp4_ctx->root.video_len) {
			printf("[D]mp4_ctx->audio_size > mp4_ctx->root.video_len\n");
		} else {
			mp4_ctx->video_timestamp_buffer[mp4_ctx->root.video_len] = mp4_ctx->video_timestamp;
		}
	} else if (type == AV_CODEC_ID_MP4A_LATM) {
		delta_clock_tick = current_clock_tick - mp4_ctx->audio_old_depend_clock_tick;
		mp4_ctx->audio_old_depend_clock_tick = current_clock_tick;
		mp4_ctx->audio_timestamp += (delta_clock_tick * mp4_ctx->audio_clock_rate) / configTICK_RATE_HZ;
		if (mp4_ctx->audio_size <= mp4_ctx->root.audio_len) {
			printf("[D]mp4_ctx->audio_size > mp4_ctx->root.video_len\n");
		} else {
			mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len] = mp4_ctx->audio_timestamp;
		}
		//mp4_ctx->audio_timestamp_buffer[mp4_ctx->root.audio_len] = mp4_ctx->audio_timestamp;
	}
}

int atcmd_stop_recording = 0;
#if 0
int check_is_iframe(pmp4_context mp4_ctx, unsigned char *buffer)
{
	int type = 0;
	type = buffer[mp4_ctx->h264_extra_len] & 0x1f;
	if (type == 0x07) {
		return 1;
	} else {
		return 0;
	}
}
#else
int check_is_iframe(pmp4_context mp4_ctx, unsigned char *buffer)
{
	int type = 0;
	if (mp4_ctx->encoder_type == AV_CODEC_ID_H264) {
		type = buffer[mp4_ctx->h264_extra_len] & 0x1f;
		if (type == 0x07) {
			return 1;
		} else {
			return 0;
		}
	} else if (mp4_ctx->encoder_type == AV_CODEC_ID_H265) {
		type = buffer[mp4_ctx->h264_extra_len];
		if (type == 0x40) {
			return 1;
		} else {
			return 0;
		}
	}
}
#endif
void mp4_muxer_handle(pmp4_context mp4_ctx, unsigned char *buf, unsigned int size, int type, unsigned int timestamp)
{
	int first = 0;
	int second = 0;
	int ret = 0;
Reaction:
	switch (mp4_ctx->storage_state) {
	case STORAGE_IDLE:
		//printf("STORAGE_IDLE\r\n");
		break;
	case STORAGE_INIT:
		//printf("init = %d\r\n",rtw_get_current_time());
		printf("STORAGE_INIT\r\n");

		if (mp4_init_buffer(mp4_ctx) < 0) {
			printf("mp4_init_buffer error\n\r");
			mp4_ctx->storage_state = STORAGE_IDLE;
			break;
		}

		ret = mp4_open_file(mp4_ctx);
		if (ret != 0) {
			printf("Can't open the file\r\n");
			mp4_ctx->storage_state = STORAGE_ERROR;
			goto Reaction;
		}
		ret = mp4_moov_start(mp4_ctx);
		if (ret != 0) {
			mp4_ctx->storage_state = STORAGE_ERROR;
			goto Reaction;
		}
		mp4_ctx->start_time = rtw_get_current_time();
		mp4_ctx->storage_state = STORAGE_START;
		if (mp4_ctx->cb_start) {
			mp4_ctx->cb_start(NULL);
		}
		printf("STORAGE_INIT -> STORAGE_START\r\n");
		goto Reaction;
		break;
	case STORAGE_START:
		if (mp4_ctx->type == STORAGE_ALL || mp4_ctx->type == STORAGE_VIDEO) {
			if (mp4_ctx->sps_pps_mark > 0) {
				printf("STORAGE_START\r\n");
				if (mp4_ctx->type == STORAGE_ALL) {
					mp4_ctx->storage_state = STORAGE_WRITE_ALL;
				} else {
					mp4_ctx->storage_state = STORAGE_WRITE_VIDEO;
				}
				mp4_ctx->start_time = rtw_get_current_time();
			} else if (type == AV_CODEC_ID_H264) {
				mp4_ctx->encoder_type = AV_CODEC_ID_H264;
				if (get_sps_pps(mp4_ctx, buf, size)) {
					//printf("STORAGE_START\r\n");
					if (mp4_ctx->type == STORAGE_ALL) {
						mp4_ctx->storage_state = STORAGE_WRITE_ALL;
					} else {
						mp4_ctx->storage_state = STORAGE_WRITE_VIDEO;
					}
					mp4_ctx->start_time = rtw_get_current_time();
					goto Reaction;
				}
			} else if (type == AV_CODEC_ID_H265) {
				mp4_ctx->encoder_type = AV_CODEC_ID_H265;
				//printf("HVCC_TYPE\r\n");
				if (get_hevc_info(mp4_ctx, buf, size)) {
					printf("STORAGE_START\r\n");
					if (mp4_ctx->type == STORAGE_ALL) {
						mp4_ctx->storage_state = STORAGE_WRITE_ALL;
					} else {
						mp4_ctx->storage_state = STORAGE_WRITE_VIDEO;
					}
					mp4_ctx->start_time = rtw_get_current_time();
					goto Reaction;
				}
			}
		} else {
			printf("STORAGE_START %d\r\n", type);
			mp4_ctx->storage_state = STORAGE_WRITE_AUDIO;
			mp4_ctx->start_time = rtw_get_current_time();
		}
		break;
	case STORAGE_WRITE_ALL:
		if ((mp4_ctx->root.video_len < mp4_ctx->video_size) && (mp4_ctx->root.audio_len < mp4_ctx->audio_size)) {
#if 0
			if (audio_appear_first == 0 && type == AV_CODEC_ID_MP4A_LATM) {
				audio_appear_first = 1;
				printf("audio appear\r\n");
			}

			if (audio_appear_first) {
				if (type == AV_CODEC_ID_MP4A_LATM) {
					if (audio_start == 0) {
						printf("audio_ts = %x\r\n", timestamp);
						audio_start = 1;
					}
					mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_MP4A_LATM);
					Write_mdat_data_audio(mp4_ctx, buf, size);
				} else if ((type == AV_CODEC_ID_H264)) {
					if (video_start == 0) {
						printf("video_ts = %x\r\n", timestamp);
						video_start = 1;
					}
					mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_H264);
					Write_mdat_data_video(mp4_ctx, buf, size);
				}
			}
#else
			if (mp4_ctx->video_appear_first == 0 && (type == AV_CODEC_ID_H264 || type == AV_CODEC_ID_H265)) {
				if (check_is_iframe(mp4_ctx, buf)) {
					mp4_ctx->video_appear_first = 1;
					mp4_ctx->video_timestamp_first = timestamp;
					//printf("vfirst = %d\r\n",timestamp);
					//printf("video appear\r\n");
				}
			}

			if (mp4_ctx->video_appear_first == 1 && mp4_ctx->audio_appear_first == 0 && type == AV_CODEC_ID_MP4A_LATM) {
				mp4_ctx->audio_appear_first = 1;
				mp4_ctx->audio_timestamp_first = timestamp;
				//printf("afirst = %d\r\n",timestamp);
				//printf("audio appear\r\n");
			}

			if (mp4_ctx->video_appear_first) {
				if (type == AV_CODEC_ID_MP4A_LATM) {
					if (mp4_ctx->audio_start == 0) {
						//printf("audio_ts = %x\r\n",timestamp);
						mp4_ctx->audio_start = 1;
					}
					mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_MP4A_LATM);
					ret = Write_mdat_data_audio(mp4_ctx, buf, size);
					if (ret != 0) {
						mp4_ctx->storage_state = STORAGE_ERROR;
						goto Reaction;
					}
				} else if ((type == AV_CODEC_ID_H264) || (type == AV_CODEC_ID_H265)) {
					if (mp4_ctx->video_start == 0) {
						// printf("video_ts = %x\r\n",timestamp);
						mp4_ctx->video_start = 1;
					}
					mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_H264);
					ret = Write_mdat_data_video(mp4_ctx, buf, size);
					if (ret != 0) {
						mp4_ctx->storage_state = STORAGE_ERROR;
						goto Reaction;
					}
					//printf("v_ts = %d\r\n",timestamp);
				}
			}

#endif
		} else {
			mp4_ctx->storage_state = STORAGE_STOP;
			goto Reaction;
		}
		break;
	case STORAGE_WRITE_AUDIO:
		if (type == AV_CODEC_ID_MP4A_LATM) {
			if (mp4_ctx->root.audio_len < mp4_ctx->audio_size) {
				mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_MP4A_LATM);
				ret = Write_mdat_data_audio(mp4_ctx, buf, size);
				if (ret != 0) {
					mp4_ctx->storage_state = STORAGE_ERROR;
					goto Reaction;
				}
			} else {
				mp4_ctx->storage_state = STORAGE_STOP;
				goto Reaction;
			}
		}
		break;
	case STORAGE_WRITE_VIDEO:
		if (type == AV_CODEC_ID_H264 || type == AV_CODEC_ID_H265) {
			if (mp4_ctx->root.video_len < mp4_ctx->video_size) {
				//printf("size %d\r\n",size);
				mp4_set_timestamp(mp4_ctx, timestamp, AV_CODEC_ID_H264);
				ret = Write_mdat_data_video(mp4_ctx, buf, size);
				if (ret != 0) {
					mp4_ctx->storage_state = STORAGE_ERROR;
					goto Reaction;
				}
			} else {
				mp4_ctx->storage_state = STORAGE_STOP;
				goto Reaction;
			}
		}
		break;
	case STORAGE_STOP:
		ret = mp4_moov_end(mp4_ctx, mp4_ctx->type);
		if (ret != 0) {
			mp4_ctx->storage_state = STORAGE_ERROR;
			goto Reaction;
		}
		ret = mp4_close_file(mp4_ctx);
		if (ret != 0) {
			printf("Can't close the file\r\n");
			mp4_ctx->storage_state = STORAGE_ERROR;
			goto Reaction;
		}
		printf("STORAGE_STOP\r\n");
		printf("video_len = %d audio_len = %d moov_len = %d total_size = %u\r\n", mp4_ctx->root.video_len, mp4_ctx->root.audio_len, mp4_ctx->moov_size,
			   mp4_ctx->root.total);
		if (mp4_ctx->cb_stop) {
			mp4_ctx->cb_stop(NULL);
		}

		if (mp4_ctx->file_name_index < mp4_ctx->file_total) {
			mp4_ctx->storage_state = STORAGE_INIT;
			printf("STORAGE_STOP -> STORAGE_INIT\r\n");
		} else {

			//mp4_ctx->storage_state = STORAGE_IDLE;
			if (mp4_ctx->loop_mode) {
				// restart, for long run test
				mp4_ctx->file_name_index = 0;
				mp4_ctx->storage_state = STORAGE_INIT;
				printf("STORAGE_STOP -> STORAGE_INIT[Loop]\r\n");
			} else {
				mp4_ctx->storage_state = STORAGE_IDLE;
				mp4_ctx->file_name_index = 0;
				mp4_free_buffer(mp4_ctx);
				printf("STORAGE_STOP -> STORAGE_IDLE\r\n");
				if (mp4_ctx->cb_end) {
					mp4_ctx->cb_end(NULL);
				}
			}

		}
		break;
	case STORAGE_ERROR:
		mp4_close_file(mp4_ctx);
		mp4_ctx->file_total = 0;
		mp4_ctx->loop_mode = 0;
		mp4_ctx->file_name_index = 0;
		mp4_ctx->storage_state = STORAGE_IDLE;
		mp4_free_buffer(mp4_ctx);
		printf("STORAGE_ERROR -> STORAGE_IDLE\r\n");
		if (mp4_ctx->cb_error) {
			mp4_ctx->cb_error(NULL);
		}
		break;

	}
}
#define MP4_STOP_THRESHOLD 100
int mp4_stop_record_immediately(pmp4_context mp4_ctx)
{
	if (mp4_is_recording(mp4_ctx)) {
		printf("Stop MP4 recording\r\n");
		mp4_ctx->file_total = 0;
		mp4_ctx->loop_mode = 0;
		mp4_ctx->storage_state = STORAGE_STOP;
		mp4_muxer_handle(mp4_ctx, NULL, 0, 0, 0);
		int i = 0;
		for (i = 0; i < MP4_STOP_THRESHOLD; i++) {
			if (mp4_ctx->storage_state == STORAGE_IDLE) {
				return 0;
			} else {
				vTaskDelay(10);
			}
		}
		printf("It can't finish the mp4 file\r\n");
		return -1;
	} else {
		printf("[Error] MP4 is not recording\r\n");
		return -1;
	}
}

#endif
