/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/

#include <dct.h>
#include <flash_api.h>
#include <device_lock.h>

#ifdef FLASH_SECTOR_SIZE
#define DCT_FLASH_SECTOR_SIZE	FLASH_SECTOR_SIZE
#else
#define DCT_FLASH_SECTOR_SIZE	0x1000
#endif

flash_t	dct_flash;

//------------------------------------------------------------------------------
int32_t dct_flash_init(void)
{
	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
int32_t dct_flash_read(uint32_t address, uint8_t *buffer, uint32_t length)
{
	int32_t ret = DCT_SUCCESS;

	device_mutex_lock(RT_DEV_LOCK_FLASH);
	if (flash_stream_read(&dct_flash, address, length, buffer) < 0) {
		ret = DCT_ERR_FLASH_RW;
//		DCT_DBG_ERROR("read error %d\n", ret);
	}
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_flash_write(uint32_t address, uint8_t *buffer, uint32_t length)
{
	int32_t ret = DCT_SUCCESS;

	device_mutex_lock(RT_DEV_LOCK_FLASH);
	if (flash_stream_write(&dct_flash, address, length, buffer) < 0) {
		ret = DCT_ERR_FLASH_RW;
//		DCT_DBG_ERROR("write error %d\n", ret);
	}
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_flash_erase(uint32_t address, uint32_t length)
{
	uint32_t sector_cnt = 0, i;

	sector_cnt = ((length - 1) / DCT_FLASH_SECTOR_SIZE) + 1;

	device_mutex_lock(RT_DEV_LOCK_FLASH);
	for (i = 0; i < sector_cnt; i++) {
		flash_erase_sector(&dct_flash, address + i * DCT_FLASH_SECTOR_SIZE);
	}
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
void dct_mutex_init(_mutex *mutex)
{
	rtw_mutex_init(mutex);
}

//------------------------------------------------------------------------------
void dct_mutex_free(_mutex *mutex)
{
	rtw_mutex_free(mutex);
}

//------------------------------------------------------------------------------
int32_t dct_mutex_lock(_mutex *mutex, uint32_t timeout_ms)
{
	// Wait for the mutex
	if (rtw_mutex_get_timeout(mutex, timeout_ms) < 0) {
		//DCT_DBG_ERROR("get mutex timeout\n");
		return DCT_ERR_MODULE_BUSY;
	}
	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
void dct_mutex_unlock(_mutex *mutex)
{
	rtw_mutex_put(mutex);
}

//------------------------------------------------------------------------------
