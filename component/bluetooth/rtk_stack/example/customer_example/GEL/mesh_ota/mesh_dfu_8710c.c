/******************************************************************************
 *
 * Copyright(c) 2019 - 2021 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#include "mesh_dfu_8710c.h"

extern uint32_t sys_update_ota_get_curr_fw_idx(void);
extern uint32_t sys_update_ota_prepare_addr(void);
flash_t mesh_flash_ota;

uint32_t mesh_dfu_prepare_addr(void)
{
	uint32_t new_fw_addr = 0;

	new_fw_addr = sys_update_ota_prepare_addr();
	if (new_fw_addr == -1) {
		printf("mesh_dfu_prepare_addr: prepare new firmware address fail \r\n");
		return 0;
	}

	return new_fw_addr;
}

uint32_t mesh_dfu_get_curr_fw_idx(void)
{
	uint32_t curr_fw_index = 0;

	curr_fw_index = sys_update_ota_get_curr_fw_idx();

	return curr_fw_index;
}

void mesh_dfu_platform_reset(void)
{
	sys_disable_fast_boot();
	hal_misc_rst_by_wdt();
	while (1) {
		osDelay(1000);
	}
}

int mesh_dfu_erase_upg_region(uint32_t img_len, uint32_t new_fw_len, uint32_t new_fw_addr)
{
	uint32_t NewFWBlkSize = 0;

	if (new_fw_len == 0) {
		new_fw_len = img_len;
		printf("[%s] new_fw_len %d\r\n", __FUNCTION__, new_fw_len);
		if ((int)new_fw_len > 0) {
			NewFWBlkSize = ((new_fw_len - 1) / 4096) + 1;
			printf("[%s] NewFWBlkSize %d  0x%x\r\n", __FUNCTION__, NewFWBlkSize, new_fw_addr);
			device_mutex_lock(RT_DEV_LOCK_FLASH);
			for (int i = 0; i < NewFWBlkSize; i++) {
				flash_erase_sector(&mesh_flash_ota, new_fw_addr + i * 4096);
			}
			device_mutex_unlock(RT_DEV_LOCK_FLASH);
		} else {
			printf("[%s] Size INVALID\r\n", __FUNCTION__);
			return -1;
		}
	}

	return new_fw_len;
}

uint8_t mesh_dfu_flash_write(uint32_t address, uint32_t length, uint8_t *data)
{
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	if (flash_burst_write(&mesh_flash_ota, address, length, data) < 0) {
		printf("[%s] Write stream failed\r\n", __FUNCTION__);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);
		return 0;
	}
	device_mutex_unlock(RT_DEV_LOCK_FLASH);

	return 1;
}

int mesh_update_ota_signature(unsigned char *sig_backup, uint32_t NewFWAddr)
{
	int ret = 0;
	unsigned char sig_readback[32];
	printf("[%s] Append OTA signature\r\n", __FUNCTION__);
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	if (flash_burst_write(&mesh_flash_ota, NewFWAddr + 16, 16, sig_backup + 16) < 0) {
		printf("[%s] Write stream failed\r\n", __FUNCTION__);
		ret = -1;
	} else {
		if (flash_burst_write(&mesh_flash_ota, NewFWAddr, 16, sig_backup) < 0) {
			printf("[%s] Write stream failed\r\n", __FUNCTION__);
			ret = -1;
		}
	}
	flash_stream_read(&mesh_flash_ota, NewFWAddr, 32, sig_readback);
	device_mutex_unlock(RT_DEV_LOCK_FLASH);

	printf("[%s] signature:\r\n", __FUNCTION__);
	for (int i = 0; i < 32; i++) {
		printf(" %02X", sig_readback[i]);
		if (i == 15) {
			printf("\r\n");
		}
	}
	return ret;
}