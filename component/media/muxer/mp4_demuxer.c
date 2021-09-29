#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#include "ff.h"
#include <fatfs_ext/inc/ff_driver.h>
#include <disk_if/inc/sdcard.h>
#include "basic_types.h"
#include "section_config.h"
#include "osdep_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mp4_demuxer.h"

static FRESULT fseek_func(FIL *stream, long int offset, int origin)
{
	return f_lseek(stream, offset);
}

static FRESULT fread_func(void *ptr, size_t size, size_t count, FIL *stream)
{
	UINT bw = 0;
	return f_read(stream, ptr, size, &bw);
}

static FRESULT fwrite_func(const void *ptr, size_t size, size_t count, FIL *stream)
{
	UINT bw = 0;
	return f_write(stream, ptr, size, &bw);
}

static FRESULT fseek_memory(mp4_demux *mp4_demuxer, long int offset, int origin)
{
	mp4_demuxer->info_offset = offset - mp4_demuxer->info_moov_offset;
	return 0;
}

static FRESULT fread_memory(void *ptr, size_t size, size_t count, mp4_demux *mp4_demuxer)
{
	memcpy(ptr, mp4_demuxer->info_buf + mp4_demuxer->info_offset, size); //memcpy(dst,src,size)
	mp4_demuxer->info_offset += size;
	return 0;
}

static FRESULT fwrite_memory(const void *ptr, size_t size, size_t count, mp4_demux *mp4_demuxer)
{
	memcpy(mp4_demuxer->info_buf + mp4_demuxer->info_offset, ptr, size);
	mp4_demuxer->info_offset += size;
	return 0;
}

unsigned int swap_uint32(unsigned int val)
{
	val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
	return (val << 16) | (val >> 16);
}

unsigned short swap_uint16(unsigned short val)
{
	return (val << 8) | (val >> 8);
}



void create_adts_header(int rate_idx, int channels, int length, unsigned char *adts_header) //need 7 byte
{
	//unsigned char adts_header[7]={0};
	unsigned int obj_type = 0x01;
	unsigned int num_data_block = 0;
	unsigned int frame_length = 0;
	frame_length = 7 + length;

	//unsigned int num_data_block = frame_length / 1024;

	// include the header length also

	//frame_length += 7;

	/* We want the same metadata */

	/* Generate ADTS header */

	//if(adts_header == NULL) return;

	/* Sync point over a full byte */

	adts_header[0] = 0xFF;

	/* Sync point continued over first 4 bits + static 4 bits

	* (ID, layer, protection)*/

	adts_header[1] = 0xF1;

	/* Object type over first 2 bits */

	adts_header[2] = obj_type << 6;//

	/* rate index over next 4 bits */

	adts_header[2] |= (rate_idx << 2);

	/* channels over last 2 bits */

	adts_header[2] |= (channels & 0x4) >> 2;

	/* channels continued over next 2 bits + 4 bits at zero */

	adts_header[3] = (channels & 0x3) << 6;

	/* frame size over last 2 bits */

	adts_header[3] |= (frame_length & 0x1800) >> 11;

	/* frame size continued over full byte */

	adts_header[4] = (frame_length & 0x1FF8) >> 3;

	/* frame size continued first 3 bits */

	adts_header[5] = (frame_length & 0x7) << 5;

	/* buffer fullness (0x7FF for VBR) over 5 last bits*/

	adts_header[5] |= 0x1F;

	/* buffer fullness (0x7FF for VBR) continued over 6 first bits + 2 zeros

	* number of raw data blocks */

	adts_header[6] = 0xFC;// one raw data blocks .

	adts_header[6] |= num_data_block & 0x03; //Set raw Data blocks.
}

int get_moov_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = 0;
	//fype
	fseek_func(mp4_demuxer->m_file, 0, SEEK_SET);
	//printf("get_moov_index\r\n");
	fread_func(&h, sizeof(header), 1, mp4_demuxer->m_file); //ftype
	h.size = swap_uint32(h.size);
	if (strncmp("ftyp", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//mdat
	fseek_func(mp4_demuxer->m_file, count, SEEK_SET);
	fread_func(&h, sizeof(header), 1, mp4_demuxer->m_file); //mdat
	h.size = swap_uint32(h.size);
	if (strncmp("mdat", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//moov
	fseek_func(mp4_demuxer->m_file, count, SEEK_SET);
	fread_func(&h, sizeof(header), 1, mp4_demuxer->m_file); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("moov", (const char *)h.str, 4) != 0) {
		return -1;
	} else {
		if (mp4_demuxer->info_buf == NULL) {
			mp4_demuxer->info_buf = (unsigned char *)malloc(h.size);
			if (mp4_demuxer->info_buf == NULL) {
				printf("It can't be allocated the buffer\r\n");
				return -1;
			} else {
				fseek_func(mp4_demuxer->m_file, count, SEEK_SET);
				fread_func(mp4_demuxer->info_buf, h.size, 1, mp4_demuxer->m_file); //moov
				//fread_func(buf+mp4_demuxer->sps_length+mp4_demuxer->pps_length,mp4_demuxer->video_size_buffer[index],1,mp4_demuxer->m_file);
				mp4_demuxer->info_size = h.size;
				mp4_demuxer->info_offset = 0;
				mp4_demuxer->info_moov_offset = count;
			}
		}
	}
	return count;
}

int get_trak_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to mvhd
	count += mvhd_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("mvhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	return count;
}

int get_trak_index_second(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to mvhd
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	count += h.size;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	if (strncmp("trak", (const char *)h.str, 4) != 0) {
		return -1;
	}
	return count;
}

int get_mdia_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to mvhd
	count += trak_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("tkhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	return count;
}

int get_minf_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to mdhd
	count += mdia_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("mdhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//jump to hdlr
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("hdlr", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;

	return count;
}

int get_stbl_video_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to vmhd
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("vmhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//jump to dinf
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("dinf", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;

	return count;
}

int get_stbl_audio_index(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to vmhd
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("smhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//jump to dinf
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("dinf", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;

	return count;
}

int get_trak_data_size(mp4_demux *mp4_demuxer, unsigned int index, int type)
{
	unsigned int value = 0;
	unsigned int len = 0;
	unsigned int i = 0;
	fseek_memory(mp4_demuxer, index + 0x10, SEEK_SET);
	fread_memory(&len, sizeof(int), 1, mp4_demuxer);
	len = swap_uint32(len);
	fseek_memory(mp4_demuxer, index + 0x14, SEEK_SET);
	if (type == video_type) {
		mp4_demuxer->video_len = len;
		if (mp4_demuxer->video_size_buffer == NULL) {
			mp4_demuxer->video_size_buffer = (unsigned int *)malloc(mp4_demuxer->video_len * sizeof(int));
			if (mp4_demuxer->video_size_buffer == NULL) {
				printf("It can't allocate memory at %s\r\n", __FUNCTION__);
				return -1;
			}
		}
	} else if (type == audio_type) {
		mp4_demuxer->audio_len = len;
		if (mp4_demuxer->audio_size_buffer == NULL) {
			mp4_demuxer->audio_size_buffer = (unsigned int *)malloc(mp4_demuxer->audio_len * sizeof(int));
			if (mp4_demuxer->audio_size_buffer == NULL) {
				printf("It can't allocate memory at %s\r\n", __FUNCTION__);
				return -1;
			}
		}
	}

	for (i = 0; i < len; i++) {
		fread_memory(&value, sizeof(int), 1, mp4_demuxer);
		value = swap_uint32(value);
		if (type == video_type) {
			if (value > mp4_demuxer->video_max_size) {
				mp4_demuxer->video_max_size = value;
			}

			mp4_demuxer->video_size_buffer[i] = value;
		} else if (type == audio_type) {
			if (value > mp4_demuxer->audio_max_size) {
				mp4_demuxer->audio_max_size = value;
			}
			mp4_demuxer->audio_size_buffer[i] = value;
		}
	}
	return 0;
}

int get_trak_data_offset(mp4_demux *mp4_demuxer, unsigned int index, int type)
{
	unsigned int value = 0;
	unsigned int len = 0;

	fseek_memory(mp4_demuxer, index + 0x0c, SEEK_SET);
	fread_memory(&len, sizeof(int), 1, mp4_demuxer);
	len = swap_uint32(len);

	fseek_memory(mp4_demuxer, index + 0x10, SEEK_SET);

	if (type == video_type) {
		if (mp4_demuxer->video_offset_buffer == NULL) {
			mp4_demuxer->video_offset_buffer = (unsigned int *)malloc(mp4_demuxer->video_len * sizeof(int));
			if (mp4_demuxer->video_offset_buffer == NULL) {
				printf("It can't be alloocate %s\r\n", __FUNCTION__);
				return -1;
			}
		}
	} else if (type == audio_type) {
		if (mp4_demuxer->audio_offset_buffer == NULL) {
			mp4_demuxer->audio_offset_buffer = (unsigned int *)malloc(mp4_demuxer->audio_len * sizeof(int));
			if (mp4_demuxer->audio_offset_buffer == NULL) {
				printf("It can't be alloocate %s\r\n", __FUNCTION__);
				return -1;
			}
		}
	}

	for (unsigned int i = 0; i < len; i++) {
		fread_memory(&value, sizeof(int), 1, mp4_demuxer);
		value = swap_uint32(value);
		if (type == video_type) {
			mp4_demuxer->video_offset_buffer[i] = value;
		} else if (type == audio_type) {
			mp4_demuxer->audio_offset_buffer[i] = value;
		}
	}
	return 0;
}

int get_timestamp_info(mp4_demux *mp4_demuxer, int index, int type)
{
	unsigned int len = 0;
	unsigned int value = 0;
	unsigned int i = 0;

	fseek_memory(mp4_demuxer, index + 0x0c, SEEK_SET);
	fread_memory(&len, sizeof(int), 1, mp4_demuxer);
	len = swap_uint32(len);
	if (type == video_type) {
		if (mp4_demuxer->video_timestamp_buf == NULL) {
			mp4_demuxer->video_timestamp_buf = (unsigned int *)malloc(len * sizeof(int));
			if (mp4_demuxer->video_timestamp_buf == NULL) {
				printf("It can't be allocated %s\r\n", __FUNCTION__);
				return -1;
			}
			mp4_demuxer->video_len = len;
		}
	} else if (type == audio_type) {
		if (mp4_demuxer->audio_timestamp_buf == NULL) {
			mp4_demuxer->audio_timestamp_buf = (unsigned int *)malloc(len * sizeof(int));
			if (mp4_demuxer->audio_timestamp_buf == NULL) {
				printf("It can't be allocated %s\r\n", __FUNCTION__);
				return -1;
			}
			mp4_demuxer->audio_len = len;
		}
	}

	for (i = 0; i < len; i++) {
		fseek_memory(mp4_demuxer, index + 0x14 + (i * 8), SEEK_SET);
		fread_memory(&value, sizeof(int), 1, mp4_demuxer);
		value = swap_uint32(value);
		if (type == video_type) {
			mp4_demuxer->video_timestamp_buf[i] = value;
		} else if (type == audio_type) {
			mp4_demuxer->audio_timestamp_buf[i] = value;
		}
	}
	return 0;
}

int get_avc1_info(mp4_demux *mp4_demuxer, int index)
{
	header h;
	fseek_memory(mp4_demuxer, index + 0x10, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("avc1", (const char *)h.str, 4) != 0) {
		return -1;
	}
	fseek_memory(mp4_demuxer, index + 0x66, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("avcC", (const char *)h.str, 4) != 0) {
		return -1;
	}

	//To get the resolution
	fseek_memory(mp4_demuxer, index + 0x30, SEEK_SET);
	fread_memory(&mp4_demuxer->width, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->width = swap_uint16(mp4_demuxer->width);

	fseek_memory(mp4_demuxer, index + 0x32, SEEK_SET);
	fread_memory(&mp4_demuxer->height, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->height = swap_uint16(mp4_demuxer->height);
	//

	fseek_memory(mp4_demuxer, index + 0x66 + 0x0e, SEEK_SET);
	fread_memory(&mp4_demuxer->sps_length, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->sps_length = swap_uint16(mp4_demuxer->sps_length);
	//printf("sps = %d\r\n",mp4_demuxer->sps_length);

	fseek_memory(mp4_demuxer, index + 0x66 + 0x0e + 0x02, SEEK_SET);

	fread_memory(mp4_demuxer->sps + 4, mp4_demuxer->sps_length, 1, mp4_demuxer);
	mp4_demuxer->sps[0] = 0x00;
	mp4_demuxer->sps[1] = 0x00;
	mp4_demuxer->sps[2] = 0x00;
	mp4_demuxer->sps[3] = 0x01;

	fseek_memory(mp4_demuxer, index + 0x66 + 0x10 + mp4_demuxer->sps_length + 0x01, SEEK_SET);
	fread_memory(&mp4_demuxer->pps_length, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->pps_length = swap_uint16(mp4_demuxer->pps_length);
	//printf("pps = %d\r\n",mp4_demuxer->pps_length);

	fseek_memory(mp4_demuxer, index + 0x66 + 0x10 + mp4_demuxer->sps_length + 0x01 + 0x02, SEEK_SET);
	fread_memory(mp4_demuxer->pps + 4, mp4_demuxer->pps_length, 1, mp4_demuxer);
	mp4_demuxer->pps[0] = 0x00;
	mp4_demuxer->pps[1] = 0x00;
	mp4_demuxer->pps[2] = 0x00;
	mp4_demuxer->pps[3] = 0x01;
	mp4_demuxer->sps_length += 4; //for nalu
	mp4_demuxer->pps_length += 4; //for nalu

	return 0;
}

int get_hvc1_info(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int type = 0;//0 for avcc 1 for hvcc
	fseek_memory(mp4_demuxer, index + 0x10, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("hvc1", (const char *)h.str, 4) != 0) {
		return -1;
	}
	fseek_memory(mp4_demuxer, index + 0x66, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("hvcC", (const char *)h.str, 4) != 0) {
		return -1;
	}


	//To get the resolution
	fseek_memory(mp4_demuxer, index + 0x30, SEEK_SET);
	fread_memory(&mp4_demuxer->width, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->width = swap_uint16(mp4_demuxer->width);
	//printf("width %d\r\n",mp4_demuxer->width);
	fseek_memory(mp4_demuxer, index + 0x32, SEEK_SET);
	fread_memory(&mp4_demuxer->height, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->height = swap_uint16(mp4_demuxer->height);
	//printf("height %d\r\n",mp4_demuxer->height);
	//

	//VPS part
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22, SEEK_SET);
	fread_memory(&mp4_demuxer->vps_length, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->vps_length = swap_uint16(mp4_demuxer->vps_length);
	printf("vps_length %d\r\n", mp4_demuxer->vps_length);
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22 + 0x02, SEEK_SET);
	fread_memory(mp4_demuxer->vps + 4, mp4_demuxer->vps_length, 1, mp4_demuxer);
	mp4_demuxer->vps[0] = 0x00;
	mp4_demuxer->vps[1] = 0x00;
	mp4_demuxer->vps[2] = 0x00;
	mp4_demuxer->vps[3] = 0x01;
	//SPS part
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22 + 0x02 + mp4_demuxer->vps_length + 0x03, SEEK_SET);
	fread_memory(&mp4_demuxer->sps_length, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->sps_length = swap_uint16(mp4_demuxer->sps_length);
	printf("sps_length %d\r\n", mp4_demuxer->sps_length);
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22 + 0x02 + mp4_demuxer->vps_length + 0x03 + 0x02, SEEK_SET);
	fread_memory(mp4_demuxer->sps + 4, mp4_demuxer->sps_length, 1, mp4_demuxer);
	mp4_demuxer->sps[0] = 0x00;
	mp4_demuxer->sps[1] = 0x00;
	mp4_demuxer->sps[2] = 0x00;
	mp4_demuxer->sps[3] = 0x01;
	//PPS part
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22 + 0x02 + mp4_demuxer->vps_length + 0x03 + 0x02 + mp4_demuxer->sps_length + 0x03, SEEK_SET);
	fread_memory(&mp4_demuxer->pps_length, sizeof(unsigned short), 1, mp4_demuxer);
	mp4_demuxer->pps_length = swap_uint16(mp4_demuxer->pps_length);
	printf("pps_length %d\r\n", mp4_demuxer->pps_length);
	fseek_memory(mp4_demuxer, index + 0x66 + 0x22 + 0x02 + mp4_demuxer->vps_length + 0x03 + 0x02 + mp4_demuxer->sps_length + 0x03 + 0x02, SEEK_SET);
	fread_memory(mp4_demuxer->pps + 4, mp4_demuxer->pps_length, 1, mp4_demuxer);
	mp4_demuxer->pps[0] = 0x00;
	mp4_demuxer->pps[1] = 0x00;
	mp4_demuxer->pps[2] = 0x00;
	mp4_demuxer->pps[3] = 0x01;

	mp4_demuxer->vps_length += 4; //for nalu
	mp4_demuxer->sps_length += 4; //for nalu
	mp4_demuxer->pps_length += 4; //for nalu
	return 0;
}

int get_mp4a_info(mp4_demux *mp4_demuxer, int index) //audio get the channel and sample rate
{
	header h;
	unsigned short value = 0;
	int i = 0;
	unsigned int freq_idx_map[] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350};
	fseek_memory(mp4_demuxer, index + 0x10, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("mp4a", (const char *)h.str, 4) != 0) {
		return -1;
	}
	fseek_memory(mp4_demuxer, index + 0x10 + 0x24, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer);
	h.size = swap_uint32(h.size);
	if (strncmp("esds", (const char *)h.str, 4) != 0) {
		return -1;
	}
	fseek_memory(mp4_demuxer, index + 0x10 + 0x24 + 0x22, SEEK_SET);
	fread_memory(&value, sizeof(unsigned short), 1, mp4_demuxer);
	value = swap_uint16(value);
	mp4_demuxer->sample_index = (value & 0x0780) >> 7;
	mp4_demuxer->channel_count = (value & 0x0078) >> 3 + 1; //It need to add one
	mp4_demuxer->audio_sample_rate = freq_idx_map[mp4_demuxer->sample_index];
	printf("sample_index = %d channel_count = %d\r\n", mp4_demuxer->sample_index, mp4_demuxer->channel_count);

	return 0;
}

int get_keyframe_info(mp4_demux *mp4_demuxer, int index)
{
	unsigned int len = 0;
	unsigned int value = 0;
	unsigned int i = 0;
	fseek_memory(mp4_demuxer, index + 0x0c, SEEK_SET);
	fread_memory(&len, sizeof(int), 1, mp4_demuxer);
	len = swap_uint32(len);
	mp4_demuxer->key_len = len;
	if (mp4_demuxer->video_keyframe_buffer == NULL) {
		mp4_demuxer->video_keyframe_buffer = (unsigned int *)malloc(len * sizeof(int));
		if (mp4_demuxer->video_keyframe_buffer == NULL) {
			printf("It can't be allocated %s\r\n", __FUNCTION__);
			return -1;
		}
		//printf("get_keyframe_info malloc\r\n");
	}
	for (i = 0; i < len; i++) {
		fread_memory(&value, sizeof(int), 1, mp4_demuxer);
		value = swap_uint32(value);
		mp4_demuxer->video_keyframe_buffer[i] = value;
	}
	return 0;
}

int get_mp4_video_info(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int ret = 0;
	int count = index;
	unsigned int value = 0;
	//jump to stsd
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stts
	h.size = swap_uint32(h.size);
	if (strncmp("stsd", (const char *)h.str, 4) != 0) {
		return -1;
	}

	ret = get_avc1_info(mp4_demuxer, count);
	if (ret < 0) {
		printf("Change to hvc1\r\n");
		ret = get_hvc1_info(mp4_demuxer, count);
		if (ret < 0) {
			printf("It cause error at get_avc1_info\r\n");
			return -1;
		} else {
			mp4_demuxer->encode_type = H265_TYPE;
		}
	} else {
		mp4_demuxer->encode_type = H264_TYPE;
	}

	//jump the stts
	count += h.size;
	fseek_memory(mp4_demuxer, count, SEEK_SET);

	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stts
	h.size = swap_uint32(h.size);
	if (strncmp("stts", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_timestamp_info(mp4_demuxer, count, video_type);
	if (ret < 0) {
		printf("It cause error at get_timestamp_info\r\n");
		return -1;
	}
	count += h.size;
	//jump to stss
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stss", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_keyframe_info(mp4_demuxer, count);
	if (ret < 0) {
		printf("It cause error at get_keyframe_info\r\n");
		return -1;
	}
	count += h.size;
	//jump to stsc
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stsc", (const char *)h.str, 4) != 0) {
		return -1;
	}
	count += h.size;
	//jump to stsz
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stsz", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_trak_data_size(mp4_demuxer, count, video_type);
	if (ret < 0) {
		printf("It cause error at get_trak_data_size\r\n");
		return -1;
	}
	count += h.size;
	//jump to stco
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stco", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_trak_data_offset(mp4_demuxer, count, video_type);
	if (ret < 0) {
		printf("It cause error at get_trak_data_offset\r\n");
		return -1;
	}
	count += h.size;
	return count;
}

int get_mp4_audio_info(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int ret = 0;
	int count = index;
	unsigned int value = 0;
	//jump to stsz
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("stsz", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_trak_data_size(mp4_demuxer, count, audio_type);
	if (ret < 0) {
		printf("It cause error at get_trak_data_size\r\n");
		return -1;
	}
	//jump the stts
	count += h.size;

	//jump to stsd
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stsd", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_mp4a_info(mp4_demuxer, count);
	if (ret < 0) {
		printf("It cause error at get_mp4a_info\r\n");
		return -1;
	}
	count += h.size;

	//jump to stts
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stts", (const char *)h.str, 4) != 0) {
		return -1;
	}
	ret = get_timestamp_info(mp4_demuxer, count, audio_type);
	if (ret < 0) {
		printf("It cause error at get_timestamp_info\r\n");
		return -1;
	}
	count += h.size;
	//jump to stsc
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stsc", (const char *)h.str, 4) != 0) {
		return -1;
	}
	//get_trak_data_size(fp,count);
	count += h.size;

	//jump to stco
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //stsc
	h.size = swap_uint32(h.size);
	if (strncmp("stco", (const char *)h.str, 4) != 0) {
		return -1;
	}

	ret = get_trak_data_offset(mp4_demuxer, count, audio_type);
	if (ret < 0) {
		printf("It cause error at get_trak_data_offset\r\n");
		return -1;
	}
	count += h.size;

	return count ;
}

int check_video_type(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to vmhd
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("vmhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
}
int check_audio_type(mp4_demux *mp4_demuxer, int index)
{
	header h;
	int count = index;
	//jump to vmhd
	count += minf_length;
	fseek_memory(mp4_demuxer, count, SEEK_SET);
	fread_memory(&h, sizeof(header), 1, mp4_demuxer); //moov
	h.size = swap_uint32(h.size);
	if (strncmp("smhd", (const char *)h.str, 4) != 0) {
		return -1;
	}
}

int get_initial_offset(mp4_demux *mp4_demuxer)
{
	int index = 0;
	if ((index = get_moov_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("moov index = %x\r\n",index);
	if ((index = get_trak_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("trak index = %x\r\n",index);
	return index;
}



int get_video_trak(mp4_demux *mp4_demuxer)
{
	int track_index = 0;
	int index = 0;

	if ((index = get_initial_offset(mp4_demuxer)) < 0) {
		return -1;
	}
	track_index = index;
	//printf("trak index = %x\r\n",index);

	if ((index = get_mdia_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("mdia index = %x\r\n",index);
	if ((index = get_minf_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("minf index = %x\r\n",index);
	if (check_video_type(mp4_demuxer, index) < 0) {
		if ((index = get_trak_index_second(mp4_demuxer, track_index)) < 0) {
			return -1;
		}
		if ((index = get_mdia_index(mp4_demuxer, index)) < 0) {
			return -1;
		}
		//printf("mdia index = %x\r\n",index);
		if ((index = get_minf_index(mp4_demuxer, index)) < 0) {
			return -1;
		}
		//printf("minf index = %x\r\n",index);
	}
	if ((index = get_stbl_video_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("stbl index = %x\r\n",index);
	if ((index = get_mp4_video_info(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("trak index = %x\r\n",index);
	mp4_demuxer->video_max_size += 100; //Keep the buffer size for sps and pps
	mp4_demuxer->video_exist = 1;
	return index;
}

int get_audio_trak(mp4_demux *mp4_demuxer)
{
	int track_index = 0;
	int index = 0;
	if ((index = get_initial_offset(mp4_demuxer)) < 0) {
		return -1;
	}
	track_index = index;
	//printf("trak index = %x\r\n",index);

	if ((index = get_mdia_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("mdia index = %x\r\n",index);
	if ((index = get_minf_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("minf index = %x\r\n",index);
	if (check_audio_type(mp4_demuxer, index) < 0) {
		if ((index = get_trak_index_second(mp4_demuxer, track_index)) < 0) {
			return -1;
		}
		if ((index = get_trak_index_second(mp4_demuxer, track_index)) < 0) {
			return -1;
		}
		if ((index = get_mdia_index(mp4_demuxer, index)) < 0) {
			return -1;
		}
		//printf("mdia index = %x\r\n",index);
		if ((index = get_minf_index(mp4_demuxer, index)) < 0) {
			return -1;
		}
		//printf("minf index = %x\r\n",index);
	}
	if ((index = get_stbl_audio_index(mp4_demuxer, index)) < 0) {
		return -1;
	}
	//printf("stbl index = %x\r\n",index);
	if ((index = get_mp4_audio_info(mp4_demuxer, index)) < 0) {
		return -1;
	}
	mp4_demuxer->audio_max_size += 64;
	mp4_demuxer->audio_exist = 1;
	return index;
}

void mp4_demuxer_deinit(mp4_demux *mp4_demuxer)
{
	if (mp4_demuxer != NULL) {
		free(mp4_demuxer);
	}
}
extern bool h264_decode_sps(unsigned char *buf, unsigned int nLen, unsigned short *width, unsigned short *height, int *fps);
int mp4_demuxer_open(mp4_demux *mp4_demuxer, char *filename)
{
	int ret = 0;
	mp4_demuxer->m_file = malloc(sizeof(FIL));

	if (f_open(mp4_demuxer->m_file, filename, FA_READ) != FR_OK) {
		printf("open %s error \r\n", filename);
		return -1;
	} else {
		printf("open %s ok\r\n", filename);
	}
	ret = get_video_trak(mp4_demuxer);
	if (ret < 0) {
		printf("Can't find the video track\r\n");
		return -1;
	}
	ret = get_audio_trak(mp4_demuxer);
	if (ret < 0) {
		printf("Can't find the audio track\r\n");
		return -1;
	}
	if (mp4_demuxer->audio_exist == 0 && mp4_demuxer->video_exist == 0) {
		return -1;
	}
	if (mp4_demuxer->video_exist) {
		h264_decode_sps(mp4_demuxer->sps, mp4_demuxer->sps_length, &mp4_demuxer->width, &mp4_demuxer->height, &mp4_demuxer->fps);
		printf("widht = %d height = %d fps = %d\r\n", mp4_demuxer->width, mp4_demuxer->height, mp4_demuxer->fps);
	}
	if (mp4_demuxer->audio_exist) {
		printf("audio sample rate = %d channel num = %d\r\n", mp4_demuxer->audio_sample_rate, mp4_demuxer->audio_channel);
	}
	return 0;
}

void mp4_demuxer_close(mp4_demux *mp4_demuxer)
{
	//For video
	if (mp4_demuxer->m_file) {
		f_close(mp4_demuxer->m_file);
	}
	if (mp4_demuxer->video_keyframe_buffer) {
		free(mp4_demuxer->video_keyframe_buffer);
	}
	if (mp4_demuxer->video_offset_buffer) {
		free(mp4_demuxer->video_offset_buffer);
	}
	if (mp4_demuxer->video_size_buffer) {
		free(mp4_demuxer->video_size_buffer);
	}
	if (mp4_demuxer->video_timestamp_buf) {
		free(mp4_demuxer->video_timestamp_buf);
	}
	//For audio
	if (mp4_demuxer->audio_offset_buffer) {
		free(mp4_demuxer->audio_offset_buffer);
	}
	if (mp4_demuxer->audio_size_buffer) {
		free(mp4_demuxer->audio_size_buffer);
	}
	if (mp4_demuxer->audio_timestamp_buf) {
		free(mp4_demuxer->audio_timestamp_buf);
	}
	if (mp4_demuxer->info_buf) {
		free(mp4_demuxer->info_buf);
	}
	memset(mp4_demuxer, 0, sizeof(mp4_demux));
}


int check_video_key_frame(mp4_demux *mp4_demuxer, int index)
{
	int i = 0;
	int key_frame_num = 0;
	for (i = 0; i < mp4_demuxer->key_len; i++) {
		if (mp4_demuxer->video_keyframe_buffer[i] == (index + 1)) {
			return 1;
		}
	}
	return 0;
}

int get_timestamp(mp4_demux *mp4_demuxer, unsigned int type, unsigned int index)
{
	int i = 0;
	int timestamp = 0;
	if (type == video_type) {
		for (i = 0; i <= index; i++) {
			timestamp += mp4_demuxer->video_timestamp_buf[i];
		}
	} else {
		for (i = 0; i <= index; i++) {
			timestamp += mp4_demuxer->audio_timestamp_buf[i];
		}
	}
	return timestamp;
}

int get_video_frame(mp4_demux *mp4_demuxer, unsigned char *buf, int index, unsigned char *key_frame, unsigned int *duration_time, unsigned int *timestamp)
{
	int size = 0;
	static int count = 0;
	fseek_func(mp4_demuxer->m_file, mp4_demuxer->video_offset_buffer[index], SEEK_SET);
	if (check_video_key_frame(mp4_demuxer, index) == 1 && count == 0) {
		if (mp4_demuxer->encode_type == H264_TYPE) {
			fread_func(buf + mp4_demuxer->sps_length + mp4_demuxer->pps_length, mp4_demuxer->video_size_buffer[index], 1, mp4_demuxer->m_file);
			memcpy(buf, mp4_demuxer->sps, mp4_demuxer->sps_length);
			memcpy(buf + mp4_demuxer->sps_length, mp4_demuxer->pps, mp4_demuxer->pps_length);

			buf[mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x00] = 0x00;
			buf[mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x01] = 0x00;
			buf[mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x02] = 0x00;
			buf[mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x03] = 0x01;
			*key_frame = 1;
			count = 0;
			size = mp4_demuxer->sps_length + mp4_demuxer->pps_length + mp4_demuxer->video_size_buffer[index];
		} else {
			fread_func(buf + mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length, mp4_demuxer->video_size_buffer[index], 1, mp4_demuxer->m_file);
			memcpy(buf, mp4_demuxer->vps, mp4_demuxer->vps_length);
			memcpy(buf + mp4_demuxer->vps_length, mp4_demuxer->sps, mp4_demuxer->sps_length);
			memcpy(buf + mp4_demuxer->vps_length + mp4_demuxer->sps_length, mp4_demuxer->pps, mp4_demuxer->pps_length);

			buf[mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x00] = 0x00;
			buf[mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x01] = 0x00;
			buf[mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x02] = 0x00;
			buf[mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length + 0x03] = 0x01;
			*key_frame = 1;
			count = 0;
			size = mp4_demuxer->vps_length + mp4_demuxer->sps_length + mp4_demuxer->pps_length + mp4_demuxer->video_size_buffer[index];
		}

	} else {
		fread_func(buf, mp4_demuxer->video_size_buffer[index], 1, mp4_demuxer->m_file);
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x01;
		*key_frame = 0;
		size = mp4_demuxer->video_size_buffer[index];
	}
	*duration_time = mp4_demuxer->video_timestamp_buf[index];
	*timestamp = get_timestamp(mp4_demuxer, video_type, index);
	return size;
}
int get_audio_frame(mp4_demux *mp4_demuxer, unsigned char *buf, int index, unsigned int *duration_time, unsigned int *timestamp)
{
	int size = 0;

	fseek_func(mp4_demuxer->m_file, mp4_demuxer->audio_offset_buffer[index], SEEK_SET);
	create_adts_header(mp4_demuxer->sample_index, mp4_demuxer->channel_count, mp4_demuxer->audio_size_buffer[index], buf);
	fread_func(buf + 7, mp4_demuxer->audio_size_buffer[index], 1, mp4_demuxer->m_file);
	size = mp4_demuxer->audio_size_buffer[index] + 7;
	*duration_time = mp4_demuxer->audio_timestamp_buf[index];
	*timestamp = get_timestamp(mp4_demuxer, audio_type, index);
	return size;
}
void set_mp4_demuxer_fatfs_param(mp4_demux *mp4_demuxer, fatfs_sd_params_t *fatfs_param)
{
	mp4_demuxer-> drv_num = fatfs_param->drv_num;
	memcpy(mp4_demuxer->_drv, fatfs_param->drv, sizeof(mp4_demuxer->_drv));
	mp4_demuxer->m_fs = fatfs_param->fs;
}
#endif