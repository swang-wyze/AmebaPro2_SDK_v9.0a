//#include "platform_opts.h"
#if 1//FATFS_DISK_SD
#include "platform_stdlib.h"
#include "ff.h"
#include <fatfs_ext/inc/ff_driver.h>
#include "sdio_combine.h"
#include "sdio_host.h"
#include <disk_if/inc/sdcard.h>
#include "fatfs_sdcard_api.h"
extern phal_sdhost_adapter_t psdioh_adapter;

static fatfs_sd_params_t fatfs_sd_param;
static uint8_t fatfs_sd_init_done = 0;
static FIL     fatfs_sd_file;
static char *fatfs_sd_buf = NULL;
static uint32_t fatfs_sd_buf_size;
static uint32_t fatfs_sd_buf_pos;


int fatfs_sd_close(void)
{
	if (fatfs_sd_init_done) {
		if (f_mount(NULL, fatfs_sd_param.drv, 1) != FR_OK) {
			printf("FATFS unmount logical drive fail.\n");
		}

		if (FATFS_UnRegisterDiskDriver(fatfs_sd_param.drv_num)) {
			printf("Unregister disk driver from FATFS fail.\n");
		}

		//sdio_deinit_host(psdioh_adapter);
		//deinit_combine(psdioh_adapter);
		fatfs_sd_init_done = 0;
	}
	return 0;
}

int fatfs_sd_init(void)
{
	int ret = 0;

	if (!fatfs_sd_init_done) {
		int Fatfs_ok = 0;
		FRESULT res;
		sdio_driver_init();

		// Register disk driver to Fatfs
		printf("Register disk driver to Fatfs.\n\r");
		fatfs_sd_param.drv_num = FATFS_RegisterDiskDriver(&SD_disk_Driver);

		if (fatfs_sd_param.drv_num < 0) {
			printf("Rigester disk driver to FATFS fail.\n\r");
		} else {
			Fatfs_ok = 1;
			fatfs_sd_param.drv[0] = fatfs_sd_param.drv_num + '0';
			fatfs_sd_param.drv[1] = ':';
			fatfs_sd_param.drv[2] = '/';
			fatfs_sd_param.drv[3] = 0;
		}
		if (!Fatfs_ok) {
			ret = -1;
			goto fatfs_init_err;
		}
		res = f_mount(&fatfs_sd_param.fs, fatfs_sd_param.drv, 1);
		if (res) {
			if (f_mount(&fatfs_sd_param.fs, fatfs_sd_param.drv, 0) != FR_OK) {
				printf("FATFS mount logical drive on sd card fail.\n\r");
				ret = -2;
				goto fatfs_init_err;
			}
		} else {
			fatfs_sd_init_done = 1;
		}
	} else {

	}

	return 0;

fatfs_init_err:
	fatfs_sd_close();
	return ret;
}

int fatfs_sd_get_param(fatfs_sd_params_t *param)
{
	if (fatfs_sd_init_done) {
		memcpy(param, &fatfs_sd_param, sizeof(fatfs_sd_params_t));
		return 0;
	} else {
		memset(param, 0, sizeof(fatfs_sd_params_t));
		return -1;
	}
}

int fatfs_sd_open_file(char *filename)
{
	if (fatfs_sd_init_done) {
		int res;
		char path[64];

		strcpy(path, fatfs_sd_param.drv);
		sprintf(&path[strlen(path)], "%s", filename);

		res = f_open(&fatfs_sd_file, path, FA_OPEN_ALWAYS | FA_WRITE);
		if (res) {
			printf("open file (%s) fail. res = %d\n\r", filename, res);
			return -1;
		}
		return 0;
	} else {
		return -2;
	}
}

int fatfs_sd_close_file(void)
{
	int res;
	res = f_close(&fatfs_sd_file);
	if (res) {
		printf("close file fail.\n\r");
		return -1;
	}

	return 0;
}

void fatfs_sd_write(char *buf, uint32_t len)
{
	int res = 0;
	uint32_t bw;
	int offset = 0;

	//printf("fatfs_sd_write length= %d\n\r",len);

	while (len > 0) {
		if (fatfs_sd_buf_pos + len >= fatfs_sd_buf_size) {
			memcpy(fatfs_sd_buf + fatfs_sd_buf_pos, buf + offset, fatfs_sd_buf_size - fatfs_sd_buf_pos);

			res = f_write(&fatfs_sd_file, fatfs_sd_buf, fatfs_sd_buf_size, &bw);
			if (res) {
				printf("Write error (%d)\n\r", res);
				f_lseek(&fatfs_sd_file, 0);
			}
			//printf("Write %d bytes.\n\r", bw);
			//vTaskDelay(1);
			offset += fatfs_sd_buf_size - fatfs_sd_buf_pos;
			len -= fatfs_sd_buf_size - fatfs_sd_buf_pos;
			fatfs_sd_buf_pos = 0;
		} else {
			memcpy(fatfs_sd_buf + fatfs_sd_buf_pos, buf + offset, len);
			fatfs_sd_buf_pos = fatfs_sd_buf_pos + len;
			len = 0;
		}
	}
}

void fatfs_sd_flush_buf(void)
{
	int res = 0;
	uint32_t bw;

	if (fatfs_sd_buf_pos != 0) {
		//printf("flush %d bytes before close file\n\r",fatfs_sd_buf_pos);
		res = f_write(&fatfs_sd_file, fatfs_sd_buf, fatfs_sd_buf_pos, (UINT *)&bw);
		if (res) {
			printf("Write error (%d)\n\r", res);
			f_lseek(&fatfs_sd_file, 0);
		}
		fatfs_sd_buf_pos = 0;
	}
}

void fatfs_sd_free_write_buf(void)
{
	if (fatfs_sd_buf) {
		free(fatfs_sd_buf);
	}
}

int fatfs_sd_create_write_buf(uint32_t buf_size)
{
	if (buf_size == 0) {
		printf("ERROR: buf_size can't be 0\n\r");
		return -1;
	}
	fatfs_sd_free_write_buf();
	fatfs_sd_buf = (char *)malloc(buf_size);
	if (fatfs_sd_buf == NULL) {
		printf("allocate fatfs_sd_buf fail\r\n");
		return -2;
	}
	memset(fatfs_sd_buf, 0, buf_size);
	fatfs_sd_buf_size = buf_size;
	fatfs_sd_buf_pos = 0;
	return 0;
}

#endif //FATFS_DISK_SD
