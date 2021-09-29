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
#include <dct_util.h>
#include <crc32.h>
#include <osdep_service.h>

#define DCT_CRC_ENABLE			1			/*!< enable CRC */
#define DCT_BACKUP_ENABLE		1			/*!< enable backup, it needs double module size */
#define DCT_WEAR_LEVEL_ENABLE	1			/*!< enable wear leveling, it needs sextuple module size */
#define DCT_CACHE_ENABLE		0			/*!< enable cache, it need 4096 bytes memory */
#define DCT_MUTEX_TIMEOUT		5000		/*!< 5 second */
#define FLASH_SECTOR_SIZE		0x1000
#define MAX_MODULE_NUM			512			/*!< due to BBT is 4K byte, 4096/2/3=682 */

#define DCT_BEGIN_ADDR			(g_dct_adapter->dct_begin_address)		/*!< DCT begin address of flash, ex: 0x100000 = 1M */
#define BACKUP_ENABLE_FLAG		(g_dct_adapter->enable_backup)			/*!< enable backup, it needs double module size */
#define WEAR_LEVELING_FLAG		(g_dct_adapter->wear_leveling)			/*!< enable wear leveling, it needs sextuple module size */
#define	MODULE_NUM				(g_dct_adapter->module_number)			/*!< number of module */
#define	MODULE_BACKUP_NUM		(g_dct_adapter->module_backup_number)	/*!< backup number of module */
#define	MODULE_REPLACE_NUM		(g_dct_adapter->module_replace_number)	/*!< replace number of module */
#define VARIABLE_NAME_SIZE		(g_dct_adapter->variable_name_size)		/*!< max size of the variable name */
#define VARIABLE_VALUE_SIZE		(g_dct_adapter->variable_value_size)	/*!< max size of the variable value */
#define MODULE_MUTEX_POINT		(g_dct_adapter->module_mutex)
#define VARIABLE_NAME_BUF		(g_dct_adapter->variable_name_buf)
#define VARIABLE_VALUE_BUF		(g_dct_adapter->variable_value_buf)

/**
  * @brief  Below definitions are the Device Configuration Table settings.
  * @note   !!! Do not modify below definitions !!!
  */
#define	MODULE_SIZE				FLASH_SECTOR_SIZE	/*!< size of a module, the same as flash sector size */
#define MODULE_SIGNATURE_SIZE	4					/*!< size of the module signature */
#define MODULE_INFO_SIZE		32					/*!< size of the module infomation */
#define MODULE_STATE_SIZE		4					/*!< size of the module state */
#define BBT_SIGNATURE_SIZE		4					/*!< size of the BBT signature */

#define MODULE_SIGNATURE_OFFSET	0													/*!< offset of module signature */
#define MODULE_STATE_OFFSET		(MODULE_SIGNATURE_OFFSET + MODULE_SIGNATURE_SIZE)	/*!< offset of module state */
#define MODULE_NAME_OFFSET		(MODULE_STATE_OFFSET + MODULE_STATE_SIZE)			/*!< offset of module name */
#define MODULE_INFO_OFFSET		(MODULE_NAME_OFFSET + MODULE_NAME_SIZE)				/*!< offset of module infomation */
#define VARIABLE_BUF_OFFSET		(MODULE_INFO_OFFSET + MODULE_INFO_SIZE)				/*!< offset of module variable start */

#define VARIABLE_ITME_SIZE		(VARIABLE_NAME_SIZE + VARIABLE_VALUE_SIZE)			/*!< size of variable name + variable vaule */
#define VARIABLE_MAX_NUM		((MODULE_SIZE - MODULE_SIGNATURE_SIZE - MODULE_STATE_SIZE - MODULE_NAME_SIZE - MODULE_INFO_SIZE)	\
								/ (VARIABLE_ITME_SIZE))								/*!< the max variable number of module */
#define BLOCK_ADDRESS(idx)		(DCT_BEGIN_ADDR + idx*MODULE_SIZE)					/*!< block start address, if wear level BBT = block0 and module0 = block1, else module0 = block0 */
#define MODULE_MUTEX(idx)		(MODULE_MUTEX_POINT + idx)
#define BBT_ADDRESS				(DCT_BEGIN_ADDR)									/*!< BBT start address = block 0 */

//#define MODULE_SIGNATURE		"DCT1"				/*!< module signature, DCT: signature, number: version, 1: add wear leveling */
//#define BBT_SIGNATURE			"BBT0"

u8 MODULE_SIGNATURE[] = "DCT1";
u8 BBT_SIGNATURE[] = "BBT0";

/**
  * @brief  DCT debug message level.
  */
#define DCT_DBG_MSG_OFF			0	/*!< all off */
#define DCT_DBG_MSG_RESIDENT	1	/*!< resident message*/
#define DCT_DBG_MSG_ERROR		2	/*!< error message */
#define DCT_DBG_MSG_WARNING		3	/*!< warning message */
#define DCT_DBG_MSG_INFO		4	/*!< info message */
#define DCT_DBG_MSG_LEVEL		DCT_DBG_MSG_INFO

#define DCT_PRINTK	printf
#define DCT_DBG_MSG(level, fmt)						\
		do{											\
			if(level <= g_dct_debug_level){			\
				DCT_PRINTK("[%s]: ", __FUNCTION__);	\
				DCT_PRINTK fmt;						\
			}										\
		}while(0)
#define DCT_DBG_NONE(...)							\
		do{											\
		}while(0)

#if (DCT_DBG_MSG_RESIDENT <= DCT_DBG_MSG_LEVEL)
#define DCT_DBG_RESIDENT(...)	DCT_DBG_MSG(DCT_DBG_MSG_RESIDENT, (__VA_ARGS__))
#else
#define DCT_DBG_RESIDENT		DCT_DBG_NONE
#endif
#if (DCT_DBG_MSG_ERROR <= DCT_DBG_MSG_LEVEL)
#define DCT_DBG_ERROR(...)		DCT_DBG_MSG(DCT_DBG_MSG_ERROR, (__VA_ARGS__))
#else
#define DCT_DBG_ERROR			DCT_DBG_NONE
#endif
#if (DCT_DBG_MSG_WARNING <= DCT_DBG_MSG_LEVEL)
#define DCT_DBG_WARNING(...)	DCT_DBG_MSG(DCT_DBG_MSG_WARNING, (__VA_ARGS__))
#else
#define DCT_DBG_WARNING			DCT_DBG_NONE
#endif
#if (DCT_DBG_MSG_INFO <= DCT_DBG_MSG_LEVEL)
#define DCT_DBG_INFO(...)		DCT_DBG_MSG(DCT_DBG_MSG_INFO, (__VA_ARGS__));
#else
#define DCT_DBG_INFO			DCT_DBG_NONE
#endif

/**
  * @brief  DCT module state value.
  */
typedef enum _dct_module_state_e {
	DCT_MODULE_STATE_INIT			= 0xFFFFFFFF,	/*!< initail state */
	DCT_MODULE_STATE_VALID			= 0xFFFFFFFE,	/*!< valid state */
	DCT_MODULE_STATE_DELETING		= 0xFFFFFFFC,	/*!< deleting state */
	DCT_MODULE_STATE_DELETED		= 0xFFFFFFF8,	/*!< deleted state */
} dct_module_state_e;

/**
  * @brief  The structure is the information of DCT module.
  * @note   !!! The structure size must is MODULE_INFO_SIZE !!!
  */
typedef struct _dct_module_info_t {
	uint32_t	variable_crc;						/*!< the crc of all variables */
	uint16_t	module_number;						/*!< the number of module */
	uint16_t	variable_name_size;					/*!< max size of the variable name (VARIABLE_NAME_SIZE) */
	uint16_t	variable_value_size;				/*!< max size of the variable value (VARIABLE_VALUE_SIZE) */
	uint16_t	used_variable_num;					/*!< the number of variable had used */
	uint8_t		enable_backup;						/*!< backup function */
	uint8_t		wear_leveling;						/*!< wear leveling */
	uint8_t		reserved[MODULE_INFO_SIZE - 1 * sizeof(uint32_t) - 4 * sizeof(uint16_t) - 2 * sizeof(uint8_t)];	/*!< the reserved size of information structure */
} dct_module_info_t;

/**
  * @brief  The structure is the dct module.
  */
typedef struct _dct_module_t {
	uint8_t				dct_signature[MODULE_SIGNATURE_SIZE];	/*!< DCT signature */
	uint32_t			module_state;							/*!< the module state */
	uint8_t				module_name[MODULE_NAME_SIZE];			/*!< the module name */
	dct_module_info_t	module_info;							/*!< the module information structure */
	uint8_t				*dct_vars_ptr;							/*!< the point of variable buffer */
} dct_module_t;

typedef struct _dct_adapter_t {
	uint32_t	dct_begin_address;
	uint16_t	module_number;
	uint16_t	module_backup_number;
	uint16_t	module_replace_number;
	uint16_t	variable_name_size;
	uint16_t	variable_value_size;
	_mutex		*module_mutex;
	uint8_t		*variable_name_buf;
	uint8_t		*variable_value_buf;
	uint8_t		enable_backup;
	uint8_t		wear_leveling;
} dct_adapter_t;

//------------------------------------------------------------------------------
dct_adapter_t	*g_dct_adapter = NULL;
uint8_t			g_dct_debug_level = DCT_DBG_MSG_ERROR;

//------------------------------------------------------------------------------
static int32_t _dct_check_bbt(void)
{
	int32_t 	ret = DCT_ERROR;
	uint32_t	bbt_signature = 0;
	uint16_t	buf_size, i, block_idx;
	uint8_t	*buf;
	uint8_t		verify_fail = 0;

	// check signature
	ret = dct_flash_read(BBT_ADDRESS, (uint8_t *)&bbt_signature, BBT_SIGNATURE_SIZE);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	if (memcmp(&bbt_signature, BBT_SIGNATURE, 3)) {
		DCT_DBG_RESIDENT("BBT signature not match.\n");
		verify_fail = 1;
	} else {
		// check BBT block index
		ret = dct_flash_read(BBT_ADDRESS + BBT_SIGNATURE_SIZE, (uint8_t *)&block_idx, 2);
		if (block_idx == 0xffff) { // index0 = 0xffff
			buf_size = ((MODULE_NUM + MODULE_BACKUP_NUM) * 2) + 2;
			buf = rtw_malloc(buf_size);
			if (!buf) {
				DCT_DBG_ERROR("BBT can't alloc memory!\n");
				return DCT_ERR_NO_MEMORY;
			}
			ret = dct_flash_read(BBT_ADDRESS + BBT_SIGNATURE_SIZE + 2, buf, buf_size);
			if (ret != DCT_SUCCESS) {
				rtw_mfree(buf, buf_size);
				return ret;
			}
			for (i = 0; i < (MODULE_NUM + MODULE_BACKUP_NUM); i++) {
				block_idx = *(uint16_t *)&buf[i * 2];
				DCT_DBG_INFO("BBT block index %d.\n", block_idx);
				if (block_idx == 0 || block_idx > (MODULE_NUM + MODULE_BACKUP_NUM + MODULE_REPLACE_NUM)) {
					DCT_DBG_ERROR("BBT block index %d error %d!\n", i, block_idx);
					verify_fail = 1;
				}
			}
			// check end
			block_idx = *(uint16_t *)&buf[(MODULE_NUM + MODULE_BACKUP_NUM) * 2];
			if (block_idx != 0xffff) {
				DCT_DBG_ERROR("BBT check end fail!\n");
				verify_fail = 1;
			}
			rtw_mfree(buf, buf_size);
		} else {
			verify_fail = 1;
		}
	}
	if (verify_fail) {
		// init BBT
		dct_flash_erase(BBT_ADDRESS, MODULE_SIZE);
		DCT_DBG_RESIDENT("erase BBT 0x%08x\n", (unsigned int)BBT_ADDRESS);
		// write signature
		ret = dct_flash_write(BBT_ADDRESS, BBT_SIGNATURE, BBT_SIGNATURE_SIZE);
		// write block index
		for (i = 0; i < (MODULE_NUM + MODULE_BACKUP_NUM); i++) {
			block_idx = i + 1;
			ret = dct_flash_write(BBT_ADDRESS + BBT_SIGNATURE_SIZE + 2 + (i * 2), (uint8_t *)&block_idx, 2);
		}
	}
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_update_bbt(uint16_t module_idx)
{
	int32_t 	ret = DCT_ERROR;
	uint16_t	block_idx, free_block_idx = 0, old_block_idx;
	uint16_t	buf_size, i;
	uint8_t	*buf;

	// find free block index
	buf_size = (MODULE_NUM + MODULE_BACKUP_NUM) * 2;
	buf = rtw_malloc(buf_size);
	if (!buf) {
		DCT_DBG_ERROR("update BBT can't alloc memory!\n");
		return DCT_ERR_NO_MEMORY;
	}
	ret = dct_flash_read(BBT_ADDRESS + BBT_SIGNATURE_SIZE + 2, buf, buf_size);
	if (ret != DCT_SUCCESS) {
		rtw_mfree(buf, buf_size);
		return ret;
	}
	for (i = 0; i < (MODULE_NUM + MODULE_BACKUP_NUM); i++) {
		block_idx = *(uint16_t *)&buf[i * 2];
		DCT_DBG_INFO("BBT block index %d.\n", block_idx);
		if (block_idx > free_block_idx) {
			free_block_idx = block_idx;
		}
	}
	old_block_idx = *(uint16_t *)&buf[module_idx * 2];
	free_block_idx++;
	*(uint16_t *)&buf[module_idx * 2] = free_block_idx;
	DCT_DBG_RESIDENT("BBT free block index %d.\n", free_block_idx);

	// init BBT
	dct_flash_erase(BBT_ADDRESS, MODULE_SIZE);
	// write signature
	ret = dct_flash_write(BBT_ADDRESS, BBT_SIGNATURE, BBT_SIGNATURE_SIZE);
	// write block index
	for (i = 0; i < (MODULE_NUM + MODULE_BACKUP_NUM); i++) {
		block_idx = *(uint16_t *)&buf[i * 2];
		ret = dct_flash_write(BBT_ADDRESS + BBT_SIGNATURE_SIZE + 2 + (i * 2), (uint8_t *)&block_idx, 2);
	}
	if (ret == DCT_SUCCESS) {
		DCT_DBG_RESIDENT("change module(%d) from block index %d to %d.\n", module_idx, old_block_idx, free_block_idx);
	}
	rtw_mfree(buf, buf_size);
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_get_block_idx_from_bbt(uint16_t module_idx, uint16_t *block_idx)
{
	int32_t 	ret = DCT_ERROR;

	ret = dct_flash_read(BBT_ADDRESS + BBT_SIGNATURE_SIZE + 2 + module_idx * 2, (uint8_t *)block_idx, 2);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	if (*block_idx == 0 || *block_idx > (MODULE_NUM + MODULE_BACKUP_NUM + MODULE_REPLACE_NUM)) {
		DCT_DBG_ERROR("BBT module index %d, error block index %d!\n", module_idx, *block_idx);
		return DCT_ERROR;
	} else {
		DCT_DBG_INFO("BBT module index %d, block index %d.\n", module_idx, *block_idx);
	}
	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
static int32_t _dct_find_module(uint8_t *module_name, uint16_t *module_idx)
{
	int32_t 	ret = DCT_ERROR;
	uint16_t 	idx, block_idx;
	uint8_t 	name[MODULE_NAME_SIZE + 1] = {0};
	uint8_t		avail = 0;
	uint32_t	total_module_number;

	total_module_number = MODULE_NUM;
	// check module name
	for (idx = 0; idx < total_module_number; idx++) {
		memset(name, 0, sizeof(name));
		// read module name
		block_idx = idx;
#if DCT_WEAR_LEVEL_ENABLE
		if (WEAR_LEVELING_FLAG) {
			if (_dct_get_block_idx_from_bbt(idx, &block_idx) != DCT_SUCCESS) {
				continue;
			}
		}
#endif
		if (dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_NAME_OFFSET, name, MODULE_NAME_SIZE) != DCT_SUCCESS) {
			continue;
		}
		// blank flash: 0xff
		if (avail == 0 && name[0] == 0xff) {
			*module_idx = idx;
			avail = 1;
			continue;
		}
		if (!strcmp((const char *)name, (const char *)module_name)) {
			*module_idx = idx;
			ret = DCT_SUCCESS;
			break;
		}
	}
	if (avail == 0 && ret != DCT_SUCCESS) {
		DCT_DBG_WARNING("module used out\n");
		ret = DCT_ERR_NO_SPACE;
	}
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_find_variable(dct_handle_t *dct_handle, uint8_t *variable_name, uint8_t *variable_value)
{
	int32_t		ret = DCT_ERROR;
	uint16_t	idx;

	DCT_DBG_INFO("find variable in module(%d).\n", dct_handle->module_idx);
	for (idx = 0; idx < VARIABLE_MAX_NUM; idx++) {
#if DCT_CACHE_ENABLE
		// blank flash: 0xff
		if (*(dct_handle->variable_cache + idx * VARIABLE_ITME_SIZE) == 0xff) {
#else
		memset(VARIABLE_NAME_BUF, 0, VARIABLE_NAME_SIZE + 1);
		// read variable name
		if (dct_flash_read(BLOCK_ADDRESS(dct_handle->block_idx) + VARIABLE_BUF_OFFSET + idx * VARIABLE_ITME_SIZE, VARIABLE_NAME_BUF,
						   VARIABLE_NAME_SIZE) != DCT_SUCCESS) {
			continue;
		}
		// blank flash: 0xff
		if (VARIABLE_NAME_BUF[0] == 0xff) {
#endif
			DCT_DBG_INFO("first empty variable idx: %d.\n", idx);
			break;
		}
#if DCT_CACHE_ENABLE
		if (!strcmp(dct_handle->variable_cache + idx * VARIABLE_ITME_SIZE, variable_name)) {
			memcpy(variable_value, dct_handle->variable_cache + idx * VARIABLE_ITME_SIZE + VARIABLE_NAME_SIZE, VARIABLE_VALUE_SIZE);
#else
		if (!strcmp((const char *)VARIABLE_NAME_BUF, (const char *)variable_name)) {
			dct_flash_read(BLOCK_ADDRESS(dct_handle->block_idx) + VARIABLE_BUF_OFFSET + idx * VARIABLE_ITME_SIZE + VARIABLE_NAME_SIZE, variable_value, VARIABLE_VALUE_SIZE);
#endif
			DCT_DBG_INFO("find variable idx: %d.\n", idx);
			ret = idx;
			break;
		}
	}
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_init_valid_module(uint16_t module_idx, uint16_t module_number, uint16_t variable_name_size, uint16_t variable_value_size,
									  uint8_t enable_backup, uint8_t wear_leveling)
{
	int32_t				ret = DCT_SUCCESS;
	dct_module_info_t	module_info;
	uint16_t			used_variable_num = 0, block_idx;

	block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		if (module_idx < (MODULE_NUM + MODULE_BACKUP_NUM)) {
			ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
			if (ret != DCT_SUCCESS) {
				return ret;
			}
		} else {
			block_idx++;
		}
	}
#endif
	// erase module if no signature or info not match
	dct_flash_erase(BLOCK_ADDRESS(block_idx), MODULE_SIZE);
	DCT_DBG_RESIDENT("erase flash address 0x%08x\n", (unsigned int)BLOCK_ADDRESS(block_idx));
	// write signature
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_SIGNATURE_OFFSET, MODULE_SIGNATURE, MODULE_SIGNATURE_SIZE);
	// write module info: module number
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.module_number - (uint32_t)&module_info),
						  (uint8_t *)&module_number, sizeof(uint16_t));
	// write module info: variable name size
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.variable_name_size - (uint32_t)&module_info),
						  (uint8_t *)&variable_name_size, sizeof(uint16_t));
	// write module info: variable value size
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.variable_value_size - (uint32_t)&module_info),
						  (uint8_t *)&variable_value_size, sizeof(uint16_t));
	// write module info: used variable number
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.used_variable_num - (uint32_t)&module_info),
						  (uint8_t *)&used_variable_num, sizeof(uint16_t));
#if DCT_BACKUP_ENABLE
	// write module info: enable backup function
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.enable_backup - (uint32_t)&module_info),
						  (uint8_t *)&enable_backup, sizeof(uint8_t));
#endif
#if DCT_WEAR_LEVEL_ENABLE
	// write module info: enable backup function
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET + ((uint32_t)&module_info.wear_leveling - (uint32_t)&module_info),
						  (uint8_t *)&wear_leveling, sizeof(uint8_t));
#endif
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_verify_module(void)
{
	int32_t				ret = DCT_SUCCESS;
	uint16_t			module_idx, block_idx;
	uint32_t			module_signature = 0;
	dct_module_info_t	module_info;
	uint8_t				verify_fail;
	uint32_t			total_module_number;

	total_module_number = MODULE_NUM + MODULE_BACKUP_NUM + MODULE_REPLACE_NUM;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		// check BBT
		ret = _dct_check_bbt();
		if (ret != DCT_SUCCESS) {
			return ret;
		}
	}
#endif
	for (module_idx = 0; module_idx < total_module_number; module_idx++) {
		verify_fail = 0;
		block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
		if (WEAR_LEVELING_FLAG) {
			if (module_idx < (MODULE_NUM + MODULE_BACKUP_NUM)) {
				ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
				if (ret != DCT_SUCCESS) {
					return ret;
				}
			} else {
				block_idx++;
			}
		}
#endif
		// check signature
		ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_SIGNATURE_OFFSET, (uint8_t *)&module_signature, MODULE_SIGNATURE_SIZE);
		if (ret != DCT_SUCCESS) {
			continue;
		}
		if (memcmp(&module_signature, MODULE_SIGNATURE, 3)) {
			DCT_DBG_RESIDENT("module(%d) signature not match.\n", module_idx);
			verify_fail = 1;
		} else {
			// check module info
			ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, MODULE_INFO_SIZE);
			if (ret != DCT_SUCCESS) {
				continue;
			}
#if DCT_BACKUP_ENABLE
			if (module_info.enable_backup == 0xff) {
				module_info.enable_backup = 0;
			}
#endif
#if DCT_WEAR_LEVEL_ENABLE
			if (module_info.wear_leveling == 0xff) {
				module_info.wear_leveling = 0;
			}
#endif
			if ((module_info.module_number != MODULE_NUM) ||
#if DCT_BACKUP_ENABLE
				(module_info.enable_backup != BACKUP_ENABLE_FLAG) ||
#endif
#if DCT_WEAR_LEVEL_ENABLE
				(module_info.wear_leveling != WEAR_LEVELING_FLAG) ||
#endif
				(module_info.variable_name_size != VARIABLE_NAME_SIZE) ||
				(module_info.variable_value_size != VARIABLE_VALUE_SIZE)) {
				DCT_DBG_RESIDENT("module(%d) info not match.\n", module_idx);
				verify_fail = 1;
			}
		}
		if (verify_fail) {
			// erase and initial to valid module
			ret = _dct_init_valid_module(module_idx, MODULE_NUM, VARIABLE_NAME_SIZE, VARIABLE_VALUE_SIZE, BACKUP_ENABLE_FLAG, WEAR_LEVELING_FLAG);
		}
	}
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_init_adapter(uint32_t begin_address, uint16_t module_number, uint16_t variable_name_size, uint16_t variable_value_size,
								 uint8_t enable_backup, uint8_t enable_wear_leveling)
{
	DCT_BEGIN_ADDR = begin_address;
	MODULE_NUM = module_number;
	VARIABLE_NAME_SIZE = variable_name_size;
	VARIABLE_VALUE_SIZE = variable_value_size;
	MODULE_BACKUP_NUM = 0;
	MODULE_REPLACE_NUM = 0;
#if DCT_BACKUP_ENABLE
	BACKUP_ENABLE_FLAG = enable_backup;
	if (enable_backup) {
		MODULE_BACKUP_NUM = module_number;
	}
#endif
#if DCT_WEAR_LEVEL_ENABLE
	WEAR_LEVELING_FLAG = enable_wear_leveling;
	if (enable_wear_leveling) {
		MODULE_BACKUP_NUM = module_number * 2;
		MODULE_REPLACE_NUM = module_number * 3;
	}
#endif
	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
static int32_t _dct_update_module_all(uint8_t *module_name, uint16_t module_idx, uint32_t module_state, uint32_t variable_crc, uint16_t used_variable_num,
									  uint8_t *variable_buf)
{
	int32_t				ret;
	uint8_t				name[MODULE_NAME_SIZE + 1] = {0};
	dct_module_info_t	module_info;
	uint16_t			block_idx;

	block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
		if (ret != DCT_SUCCESS) {
			return ret;
		}
	}
#endif
	// erease module
	ret = dct_flash_erase(BLOCK_ADDRESS(block_idx), MODULE_SIZE);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	// write signature
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_SIGNATURE_OFFSET, MODULE_SIGNATURE, MODULE_SIGNATURE_SIZE);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	// write module state
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	// write module name
	memset(name, 0, sizeof(name));
	strncpy((char *)name, (char *)module_name, MODULE_NAME_SIZE);
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_NAME_OFFSET, name, MODULE_NAME_SIZE);
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	// write module info
	memset(&module_info, 0, sizeof(module_info));
	module_info.variable_crc = variable_crc;
	module_info.module_number = MODULE_NUM;
	module_info.variable_name_size = VARIABLE_NAME_SIZE;
	module_info.variable_value_size = VARIABLE_VALUE_SIZE;
	module_info.used_variable_num = used_variable_num;
#if DCT_BACKUP_ENABLE
	module_info.enable_backup = BACKUP_ENABLE_FLAG;
#endif
#if DCT_WEAR_LEVEL_ENABLE
	module_info.wear_leveling = WEAR_LEVELING_FLAG;
#endif
	ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, sizeof(module_info));
	if (ret != DCT_SUCCESS) {
		return ret;
	}
	// write variable
	if (variable_buf && used_variable_num) {
		ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, variable_buf, used_variable_num * VARIABLE_ITME_SIZE);
	}

#if DCT_WEAR_LEVEL_ENABLE
	// check bad block
	if (WEAR_LEVELING_FLAG) {
		uint8_t		bad_block = 0;
		uint32_t	check_module_signature = 0;
		uint32_t 	check_module_state, variable_buf_size;
		uint8_t		*check_variable_buf;

		// check signature
		dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_SIGNATURE_OFFSET, (uint8_t *)&check_module_signature, MODULE_SIGNATURE_SIZE);
		if (memcmp(&check_module_signature, MODULE_SIGNATURE, 4)) {
			bad_block = 1;
		}
		// check module state
		dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&check_module_state, MODULE_STATE_SIZE);
		if (check_module_state != module_state) {
			bad_block = 1;
		}
		// check module name
		dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_NAME_OFFSET, name, MODULE_NAME_SIZE);
		if (strcmp((const char *)name, (const char *)module_name)) {
			bad_block = 1;
		}
		// check module info
		dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, MODULE_INFO_SIZE);
		if ((module_info.variable_crc != variable_crc) ||
			(module_info.module_number != MODULE_NUM) ||
			(module_info.variable_name_size != VARIABLE_NAME_SIZE) ||
			(module_info.variable_value_size != VARIABLE_VALUE_SIZE) ||
			(module_info.used_variable_num != used_variable_num) ||
#if DCT_BACKUP_ENABLE
			(module_info.enable_backup != BACKUP_ENABLE_FLAG) ||
#endif
			(module_info.wear_leveling != WEAR_LEVELING_FLAG)) {
			DCT_DBG_RESIDENT("module(%d) info not match.\n", module_idx);
			bad_block = 1;
		}
		// check variable
		variable_buf_size = used_variable_num * VARIABLE_ITME_SIZE;
		check_variable_buf = rtw_malloc(variable_buf_size);
		if (!check_variable_buf) {
			DCT_DBG_ERROR("can't alloc memory!\n");
			return DCT_ERR_NO_MEMORY;
		}
		dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, check_variable_buf, variable_buf_size);
		if (memcmp(check_variable_buf, variable_buf, variable_buf_size)) {
			bad_block = 1;
		}
		rtw_mfree(check_variable_buf, variable_buf_size);
		// update BBT
		if (bad_block) {
			ret = _dct_update_bbt(module_idx);
			if (ret != DCT_SUCCESS) {
				DCT_DBG_ERROR("update module(%d) fail.\n", module_idx);
				return ret;
			}
			ret = _dct_update_module_all(module_name, module_idx, module_state, variable_crc, used_variable_num, variable_buf);
			if (ret != DCT_SUCCESS) {
				DCT_DBG_ERROR("update module(%d) from BBT fail.\n", module_idx);
				return ret;
			}
		}
	}
#endif
	DCT_DBG_INFO("update module(%d) success.\n", module_idx);
	return ret;
}

//------------------------------------------------------------------------------
static int32_t _dct_update_variable(dct_handle_t *dct_handle, uint16_t used_variable_num, uint32_t variable_idx, char *variable_name, char *variable_value)
{
	int32_t		ret = DCT_SUCCESS;
	uint8_t		*variable_buf;
#if !DCT_CACHE_ENABLE
	uint32_t	variable_buf_size;
	uint16_t	module_idx, block_idx;
	uint32_t	module_state;
#endif

#if DCT_CACHE_ENABLE
	variable_buf = dct_handle->variable_cache;
#else
	// get all variable from flash
	variable_buf_size = used_variable_num * VARIABLE_ITME_SIZE;
	variable_buf = rtw_malloc(variable_buf_size);
	if (!variable_buf) {
		DCT_DBG_ERROR("can't alloc memory!\n");
		return DCT_ERR_NO_MEMORY;
	}
	ret = dct_flash_read(BLOCK_ADDRESS(dct_handle->block_idx) + VARIABLE_BUF_OFFSET, variable_buf, variable_buf_size);
	if (ret == DCT_SUCCESS) {
#endif
	if (variable_name && variable_value) {
		// set new variable
		memset(&variable_buf[variable_idx * VARIABLE_ITME_SIZE], 0, VARIABLE_ITME_SIZE);
		strcpy((char *)&variable_buf[variable_idx * VARIABLE_ITME_SIZE], (char *)variable_name);
		strcpy((char *)&variable_buf[variable_idx * VARIABLE_ITME_SIZE + VARIABLE_NAME_SIZE], (char *)variable_value);
	} else {
		// delete variable
		if (used_variable_num > variable_idx) {
			memcpy(&variable_buf[variable_idx * VARIABLE_ITME_SIZE], &variable_buf[(variable_idx + 1)*VARIABLE_ITME_SIZE],
				   (used_variable_num - variable_idx - 1)*VARIABLE_ITME_SIZE); // move variable
			memset(&variable_buf[(used_variable_num - 1)*VARIABLE_ITME_SIZE], 0xFF, VARIABLE_ITME_SIZE);	// clear last one
			used_variable_num--;
		}
	}
#if !DCT_CACHE_ENABLE
} else {
	rtw_mfree(variable_buf, variable_buf_size);
	return ret;
}
#endif

#if DCT_CRC_ENABLE
dct_handle->variable_crc = crc32(variable_buf, used_variable_num * VARIABLE_ITME_SIZE, 0);
DCT_DBG_INFO("module(%d) crc(0x%08x)\n", dct_handle->module_idx, (unsigned int)dct_handle->variable_crc);
#else
dct_handle->variable_crc = 0xFFFFFFFF;
#endif

#if DCT_CACHE_ENABLE
dct_handle->cache_dirty = 1;
#else
// update module with new variable into flash
#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
if (BACKUP_ENABLE_FLAG || WEAR_LEVELING_FLAG) {
	// change handle module state to DCT_MODULE_STATE_DELETED
	module_state = DCT_MODULE_STATE_DELETED;
	ret = dct_flash_write(BLOCK_ADDRESS(dct_handle->block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
	// update to backup module
	module_idx = dct_handle->module_idx;
	module_idx += MODULE_NUM;
	if (module_idx >= (MODULE_NUM + MODULE_BACKUP_NUM)) {
		module_idx %= MODULE_NUM;
	}
} else {
	// update to handle module
	module_idx = dct_handle->module_idx;
}
#else
// update to handle module
module_idx = dct_handle->module_idx;
#endif
// update module into flash
ret = _dct_update_module_all(dct_handle->module_name, module_idx, DCT_MODULE_STATE_VALID, dct_handle->variable_crc, used_variable_num, variable_buf);
if (ret != DCT_SUCCESS) {
	DCT_DBG_ERROR("update module to flash fail!\n");
	rtw_mfree(variable_buf, variable_buf_size);
	return ret;
}
#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
if (BACKUP_ENABLE_FLAG || WEAR_LEVELING_FLAG) {
	// change handle module idx to backup module idx
	DCT_DBG_INFO("change handle module idx from %d to %d.\n", dct_handle->module_idx, module_idx);
	dct_handle->module_idx = module_idx;
	block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		_dct_get_block_idx_from_bbt(module_idx, &block_idx);
	}
#endif
	dct_handle->block_idx = block_idx;
}
#endif

rtw_mfree(variable_buf, variable_buf_size);
#endif	// #if DCT_CACHE_ENABLE

// update used_variable_num of dct_handle
dct_handle->used_variable_num = used_variable_num;

return ret;
}

//------------------------------------------------------------------------------
#if DCT_CACHE_ENABLE
static int32_t _dct_commit_variable(dct_handle_t *dct_handle)
{
	int32_t		ret;
	uint8_t		*variable_buf = dct_handle->variable_cache;
	uint16_t	module_idx, block_idx;
	uint32_t	module_state;

	// update module with new variable into flash
#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
	if (BACKUP_ENABLE_FLAG || WEAR_LEVELING_FLAG) {
		// change handle module state to DCT_MODULE_STATE_DELETED
		module_state = DCT_MODULE_STATE_DELETED;
		ret = dct_flash_write(BLOCK_ADDRESS(dct_handle->block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
		// update to backup module
		module_idx = dct_handle->module_idx;
		module_idx += MODULE_NUM;
		if (module_idx >= (MODULE_NUM + MODULE_BACKUP_NUM)) {
			module_idx %= MODULE_NUM;
		}
	} else {
		// update to handle module
		module_idx = dct_handle->module_idx;
	}
#else
	// update to handle module
	module_idx = dct_handle->module_idx;
#endif
	// update module into flash
	ret = _dct_update_module_all(dct_handle->module_name, module_idx, DCT_MODULE_STATE_VALID, dct_handle->variable_crc, dct_handle->used_variable_num,
								 variable_buf);
	if (ret != DCT_SUCCESS) {
		DCT_DBG_ERROR("update module to flash fail!\n");
		return ret;
	}
	return ret;
}
#endif

//------------------------------------------------------------------------------
static int32_t _dct_get_used_variable_num(uint16_t block_idx)
{
	int32_t		ret = DCT_ERROR;
	uint16_t	idx;
	uint8_t		name;

	for (idx = 0; idx < VARIABLE_MAX_NUM; idx++) {
		name = 0;
		// read variable name
		if (dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET + idx * VARIABLE_ITME_SIZE, &name, 1) != DCT_SUCCESS) {
			return ret;
		}
		// blank flash: 0xff
		if (name == 0xff) {
			break;
		}
	}
	ret = idx;
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_remain_variable(dct_handle_t *dct_handle)
{
	if (dct_handle == NULL || dct_handle->module_state != DCT_MODULE_STATE_VALID) {
		DCT_DBG_WARNING("open module to get handle first.\n");
		return DCT_ERR_INVALID;
	}
	return (VARIABLE_MAX_NUM - dct_handle->used_variable_num);
}

//------------------------------------------------------------------------------
int32_t dct_delete_variable(dct_handle_t *dct_handle, char *variable_name)
{
	int32_t		idx, ret;
	uint32_t 	variable_idx;

	if (dct_handle == NULL || dct_handle->module_state != DCT_MODULE_STATE_VALID) {
		DCT_DBG_WARNING("open module to get handle first.\n");
		return DCT_ERR_INVALID;
	}
	memset(VARIABLE_VALUE_BUF, 0, VARIABLE_VALUE_SIZE + 1);
	idx = _dct_find_variable(dct_handle, (uint8_t *)variable_name, VARIABLE_VALUE_BUF);
	if (idx < 0) {
		DCT_DBG_WARNING("not find variable.\n");
		return DCT_ERR_NOT_FIND;
	}
	variable_idx = idx;
	if (variable_idx >= dct_handle->used_variable_num) {
		DCT_DBG_ERROR("used variable number small than variable index(%d).\n", (int)idx);
		return DCT_ERR_INVALID;
	}
	// update variable to flash
	ret = _dct_update_variable(dct_handle, dct_handle->used_variable_num, variable_idx, 0, 0);

	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_set_variable(dct_handle_t *dct_handle, char *variable_name, char *variable_value)
{
	int32_t		idx, ret;
	uint16_t	len, used_variable_num;
	uint32_t	variable_idx;

	if (dct_handle == NULL || dct_handle->module_state != DCT_MODULE_STATE_VALID) {
		DCT_DBG_WARNING("open module to get handle first.\n");
		return DCT_ERR_INVALID;
	}
	memset(VARIABLE_VALUE_BUF, 0, VARIABLE_VALUE_SIZE + 1);
	len = strlen(variable_name);
	if (len > VARIABLE_NAME_SIZE) {
		DCT_DBG_ERROR("variable name length(%d) over than max size(%d)!\n", len, VARIABLE_NAME_SIZE);
		return DCT_ERR_SIZE_OVER;
	}
	len = strlen(variable_value);
	if (len > VARIABLE_VALUE_SIZE) {
		DCT_DBG_ERROR("variable value length(%d) over than max size(%d)!\n", len, VARIABLE_VALUE_SIZE);
		return DCT_ERR_SIZE_OVER;
	}

	idx = _dct_find_variable(dct_handle, (uint8_t *)variable_name, VARIABLE_VALUE_BUF);
	if (idx >= 0) {
		// check value if changed
		if (!strcmp((const char *)VARIABLE_VALUE_BUF, (const char *)variable_value)) {
			DCT_DBG_INFO("variable value is the same.\n");
			return DCT_SUCCESS;
		}
		DCT_DBG_INFO("variable already exit, update new value.\n");
		// update original one
		used_variable_num = dct_handle->used_variable_num;
		variable_idx = idx;
	} else {
		// add new one
		if (dct_handle->used_variable_num == VARIABLE_MAX_NUM) { // full
			DCT_DBG_ERROR("variable space full(%d)!\n", dct_handle->used_variable_num);
			return DCT_ERR_NO_SPACE;
		}
		used_variable_num = dct_handle->used_variable_num + 1;
		variable_idx = dct_handle->used_variable_num;
	}
	// update variable to flash
	ret = _dct_update_variable(dct_handle, used_variable_num, variable_idx, variable_name, variable_value);

	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_get_variable(dct_handle_t *dct_handle, char *variable_name, char *buffer, uint16_t buffer_size)
{
	int32_t		idx;
	uint16_t	len;

	if (dct_handle == NULL || dct_handle->module_state != DCT_MODULE_STATE_VALID) {
		DCT_DBG_WARNING("open module to get handle first.\n");
		return DCT_ERR_INVALID;
	}

	memset(VARIABLE_VALUE_BUF, 0, VARIABLE_VALUE_SIZE + 1);
	idx = _dct_find_variable(dct_handle, (uint8_t *)variable_name, VARIABLE_VALUE_BUF);
	if (idx < 0) {
		DCT_DBG_WARNING("not find variable.\n");
		return DCT_ERR_NOT_FIND;
	}
	len = strlen((const char *)VARIABLE_VALUE_BUF);
	if (len > buffer_size) {
		DCT_DBG_WARNING("buffer size(%d) is less than variable length(%d).\n", buffer_size, len);
		len = buffer_size;
	}
	memcpy(buffer, VARIABLE_VALUE_BUF, len);

	return DCT_SUCCESS;
}

//------------------------------------------------------------------------------
int32_t dct_open_module(dct_handle_t *dct_handle, char *module_name)
{
	int32_t				ret;
	uint16_t			module_idx, block_idx, i;
	dct_module_info_t	module_info;
	uint32_t			module_state;
#if DCT_CRC_ENABLE
	uint8_t				*variable_buf = NULL;
	uint32_t			variable_buf_size, variable_crc;
#endif

	if (!g_dct_adapter) {
		DCT_DBG_RESIDENT("init DCT first\n");
		return DCT_ERROR;
	}

	// initial handle
	memset(dct_handle, 0, sizeof(dct_handle_t));

	// find module
	ret = _dct_find_module((uint8_t *)module_name, &module_idx);
	if (ret != DCT_SUCCESS) {
		DCT_DBG_WARNING("not find module\n");
		return DCT_ERR_NOT_FIND;
	}

	// get module mutex lock
	if (!*(MODULE_MUTEX(module_idx % MODULE_NUM))) {
		dct_mutex_init(MODULE_MUTEX(module_idx % MODULE_NUM));
	}
	if (dct_mutex_lock(MODULE_MUTEX(module_idx % MODULE_NUM), DCT_MUTEX_TIMEOUT) < 0) {
		DCT_DBG_ERROR("get module(%d) mutex timeout\n", module_idx);
		return DCT_ERROR;
	}

#if DCT_CACHE_ENABLE
	// alloc cache
	dct_handle->variable_cache = rtw_malloc(VARIABLE_ITME_SIZE * VARIABLE_MAX_NUM);
	if (!dct_handle->variable_cache) {
		DCT_DBG_ERROR("can't alloc memory!\n");
		ret = DCT_ERR_NO_MEMORY;
		goto open_error;
	}
	memset(dct_handle->variable_cache, 0xFF, VARIABLE_ITME_SIZE * VARIABLE_MAX_NUM);
	dct_handle->cache_dirty = 0;
#endif

	// check module state
	block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
		if (ret != DCT_SUCCESS) {
			goto open_error;
		}
	}
#endif
	ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
	if (ret != DCT_SUCCESS) {
		goto open_error;
	}

#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		if (module_state != DCT_MODULE_STATE_VALID) {
			DCT_DBG_INFO("module(%d) state(0x%08x) invalid.\n", module_idx, (unsigned int)module_state);
			for (i = 0; i < (MODULE_BACKUP_NUM / MODULE_NUM); i++) {
				module_idx += MODULE_NUM;
				if (module_idx >= (MODULE_NUM + MODULE_BACKUP_NUM)) {
					module_idx %= MODULE_NUM;
				}
				DCT_DBG_INFO("change backup module(%d).\n", module_idx);
				// check backup module state
				ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
				if (ret != DCT_SUCCESS) {
					goto open_error;
				}
				ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
				if (ret != DCT_SUCCESS) {
					goto open_error;
				}
				if (module_state == DCT_MODULE_STATE_VALID) {
					break;
				}
			}
			if (i == (MODULE_BACKUP_NUM / MODULE_NUM)) {
				DCT_DBG_ERROR("all backup module state(0x%08x) not vaild!\n", (unsigned int)module_state);
			}
		}
	} else if (BACKUP_ENABLE_FLAG) {
		if (module_state != DCT_MODULE_STATE_VALID) {
			DCT_DBG_INFO("module(%d) state(0x%08x) invalid.\n", module_idx, (unsigned int)module_state);
			module_idx += MODULE_NUM;
			if (module_idx >= (MODULE_NUM + MODULE_BACKUP_NUM)) {
				module_idx %= MODULE_NUM;
			}
			DCT_DBG_INFO("change backup module(%d).\n", module_idx);
			// check backup module state
			block_idx = module_idx;
			ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
			if (ret != DCT_SUCCESS) {
				goto open_error;
			}
			if (module_state != DCT_MODULE_STATE_VALID) {
				DCT_DBG_ERROR("backup module state(0x%08x)!\n", (unsigned int)module_state);
			}
		}
	} else {
		if (module_state != DCT_MODULE_STATE_VALID) {
			DCT_DBG_ERROR("module state(0x%08x) invalid, unregister first!\n", (unsigned int)module_state);
			ret = DCT_ERR_INVALID;
			goto open_error;
		}
	}
#else
	if (module_state != DCT_MODULE_STATE_VALID) {
		DCT_DBG_ERROR("module state(0x%08x) invalid, unregister first!\n", module_state);
		ret = DCT_ERR_INVALID;
		goto open_error;
	}
#endif
	// get module info
	ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, sizeof(module_info));
	if (ret != DCT_SUCCESS) {
		goto open_error;
	}
	// check used variable number
	if (module_info.used_variable_num > VARIABLE_MAX_NUM) {
		DCT_DBG_ERROR("module used variable number(%d) bigger than MAX(%d)!\n", module_info.used_variable_num, VARIABLE_MAX_NUM);
		ret = DCT_ERR_INVALID;
		goto open_error;
	}

#if DCT_CACHE_ENABLE
	// cache variables
	ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, dct_handle->variable_cache, module_info.used_variable_num * VARIABLE_ITME_SIZE);
	if (ret != DCT_SUCCESS) {
		goto open_error;
	}
#endif

#if DCT_CRC_ENABLE
	// check crc
	if (module_info.used_variable_num != 0) {
		// calculate variable crc
		variable_buf_size = module_info.used_variable_num * VARIABLE_ITME_SIZE;
#if DCT_CACHE_ENABLE
		variable_crc = crc32(dct_handle->variable_cache, variable_buf_size, 0);
#else
		variable_buf = rtw_malloc(variable_buf_size);
		if (!variable_buf) {
			DCT_DBG_ERROR("can't alloc memory!\n");
			ret = DCT_ERR_NO_MEMORY;
			goto open_error;
		}
		ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, variable_buf, variable_buf_size);
		if (ret != DCT_SUCCESS) {
			goto open_error;
		}
		variable_crc = crc32(variable_buf, variable_buf_size, 0);
#endif
		if (variable_crc != module_info.variable_crc) {
			DCT_DBG_ERROR("module(%d) crc(0x%08x, 0x%08x) not match!\n",
						  module_idx, (unsigned int)variable_crc, (unsigned int)module_info.variable_crc);
#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
			if (BACKUP_ENABLE_FLAG || WEAR_LEVELING_FLAG) {
				uint16_t	module_idx_old;
				module_idx_old = module_idx;
				// go back to previous one
				if (module_idx < MODULE_NUM) {
					module_idx += MODULE_BACKUP_NUM;
				} else {
					module_idx -= MODULE_NUM;
				}
				// get module info
				block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
				if (WEAR_LEVELING_FLAG) {
					ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
					if (ret != DCT_SUCCESS) {
						goto open_error;
					}
				}
#endif
				ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, sizeof(module_info));
				if (ret == DCT_SUCCESS) {
					// check used variable number
					if (module_info.used_variable_num > VARIABLE_MAX_NUM) {
						DCT_DBG_ERROR("module used variable number(%d) bigger than MAX(%d)!\n", module_info.used_variable_num, VARIABLE_MAX_NUM);
						ret = DCT_ERR_INVALID;
						goto open_error;
					}
					// calculate variable crc
#if DCT_CACHE_ENABLE
					memset(dct_handle->variable_cache, 0xFF, variable_buf_size);
					variable_buf_size = module_info.used_variable_num * VARIABLE_ITME_SIZE;
					ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, dct_handle->variable_cache, variable_buf_size);
					if (ret != DCT_SUCCESS) {
						goto open_error;
					}
					variable_crc = crc32(dct_handle->variable_cache, module_info.used_variable_num * VARIABLE_ITME_SIZE, 0);
#else
					rtw_mfree(variable_buf, variable_buf_size);
					variable_buf_size = module_info.used_variable_num * VARIABLE_ITME_SIZE;
					variable_buf = rtw_malloc(variable_buf_size);
					if (!variable_buf) {
						DCT_DBG_ERROR("can't alloc memory!\n");
						ret = DCT_ERR_NO_MEMORY;
						goto open_error;
					}
					ret = dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET, variable_buf, variable_buf_size);
					if (ret != DCT_SUCCESS) {
						goto open_error;
					}
					variable_crc = crc32(variable_buf, variable_buf_size, 0);
#endif
					if (variable_crc == module_info.variable_crc) {
						// overwrite crc error's module from backup module
#if DCT_CACHE_ENABLE
						ret = _dct_update_module_all((uint8_t *)module_name, module_idx_old, DCT_MODULE_STATE_VALID, variable_crc, module_info.used_variable_num,
													 dct_handle->variable_cache);
#else
						ret = _dct_update_module_all((uint8_t *)module_name, module_idx_old, DCT_MODULE_STATE_VALID, variable_crc, module_info.used_variable_num, variable_buf);
#endif
						if (ret != DCT_SUCCESS) {
							DCT_DBG_ERROR("update module to flash fail!\n");
							goto open_error;
						}
						DCT_DBG_RESIDENT("overwrite crc error module(%d) from backup module(%d)!\n", module_idx_old, module_idx);
						module_idx = module_idx_old;
					} else {
						DCT_DBG_ERROR("badkup module(%d) crc(0x%08x, 0x%08x) not match!\n",
									  module_idx, (unsigned int)variable_crc, (unsigned int)module_info.variable_crc);
						ret = DCT_ERR_CRC;
						goto open_error;
					}
				} else {
					ret = DCT_ERR_CRC;
					goto open_error;
				}
			} else {
				ret = DCT_ERR_CRC;
				goto open_error;
			}
#else
			ret = DCT_ERR_CRC;
			goto open_error;
#endif
		}
	}
#endif
	// setup handle
	strncpy((char *)dct_handle->module_name, module_name, MODULE_NAME_SIZE);
	dct_handle->module_idx = module_idx;
	dct_handle->block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		dct_handle->block_idx = block_idx;
	}
#endif
	dct_handle->module_state = DCT_MODULE_STATE_VALID;
	dct_handle->used_variable_num = module_info.used_variable_num;
#if DCT_CRC_ENABLE
	if (variable_buf) {
		rtw_mfree(variable_buf, variable_buf_size);
	}
#endif
	return DCT_SUCCESS;

open_error:
#if DCT_CRC_ENABLE
	if (variable_buf) {
		rtw_mfree(variable_buf, variable_buf_size);
	}
#endif
#if DCT_CACHE_ENABLE
	if (dct_handle->variable_cache) {
		rtw_mfree(dct_handle->variable_cache, VARIABLE_ITME_SIZE * VARIABLE_MAX_NUM);
		dct_handle->variable_cache = NULL;
	}
#endif
	// free module mutex lock
	dct_mutex_unlock(MODULE_MUTEX(module_idx % MODULE_NUM));
	dct_mutex_free(MODULE_MUTEX(module_idx % MODULE_NUM));
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_close_module(dct_handle_t *dct_handle)
{
	int32_t	ret = DCT_SUCCESS;

#if DCT_CACHE_ENABLE
	// commit variable
	if (dct_handle->cache_dirty) {
		ret = _dct_commit_variable(dct_handle);
		if (ret != DCT_SUCCESS) {
			DCT_DBG_ERROR("commit module(%s) fail!\n", dct_handle->module_name);
		}
	}
	// free cache
	if (dct_handle->variable_cache) {
		rtw_mfree(dct_handle->variable_cache, MODULE_SIZE);
		dct_handle->variable_cache = NULL;
	}
#endif
	dct_handle->module_state = 0;
	// free module mutex lock
	dct_mutex_unlock(MODULE_MUTEX(dct_handle->module_idx % MODULE_NUM));
	dct_mutex_free(MODULE_MUTEX(dct_handle->module_idx % MODULE_NUM));
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_register_module(char *module_name)
{
	int32_t				ret;
	uint16_t			module_idx, block_idx;
	uint8_t				name[MODULE_NAME_SIZE + 1] = {0};
	uint32_t			module_state;
	dct_module_info_t	module_info;

	if (!g_dct_adapter) {
		DCT_DBG_RESIDENT("init DCT first\n");
		return DCT_ERROR;
	}
	if (strlen(module_name) > MODULE_NAME_SIZE) {
		DCT_DBG_ERROR("length of module name is more than %d!\n", MODULE_NAME_SIZE);
		return DCT_ERROR;
	}
	// find module
	ret = _dct_find_module((uint8_t *)module_name, &module_idx);
	switch (ret) {
	case DCT_SUCCESS:
		DCT_DBG_INFO("%s had registed %d\n", module_name, module_idx);
		ret = DCT_SUCCESS;
		break;
	case DCT_ERR_NO_SPACE:
		DCT_DBG_ERROR("no freed module!\n");
		ret = DCT_ERR_NO_SPACE;
		break;
	case DCT_ERROR:	// get freed module_idx to add
		block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
		if (WEAR_LEVELING_FLAG) {
			ret = _dct_get_block_idx_from_bbt(module_idx, &block_idx);
			if (ret != DCT_SUCCESS) {
				return ret;
			}
		}
#endif
		// get module mutex lock
		if (!*(MODULE_MUTEX(module_idx % MODULE_NUM))) {
			dct_mutex_init(MODULE_MUTEX(module_idx % MODULE_NUM));
		}
		if (dct_mutex_lock(MODULE_MUTEX(module_idx % MODULE_NUM), DCT_MUTEX_TIMEOUT) < 0) {
			DCT_DBG_ERROR("get module(%d) mutex timeout\n", module_idx);
			ret = DCT_ERROR;
		} else {
			// write module state
			module_state = DCT_MODULE_STATE_VALID;
			ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
			// write module name
			memset(name, 0, sizeof(name));
			strncpy((char *)name, module_name, MODULE_NAME_SIZE);
			ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_NAME_OFFSET, name, MODULE_NAME_SIZE);
			// write module info
			memset(&module_info, 0, sizeof(module_info));
			module_info.variable_crc = 0xFFFFFFFF;
			module_info.module_number = MODULE_NUM;
			module_info.variable_name_size = VARIABLE_NAME_SIZE;
			module_info.variable_value_size = VARIABLE_VALUE_SIZE;
			module_info.used_variable_num = 0;
#if DCT_BACKUP_ENABLE
			module_info.enable_backup = BACKUP_ENABLE_FLAG;
#endif
#if DCT_WEAR_LEVEL_ENABLE
			module_info.wear_leveling = WEAR_LEVELING_FLAG;
#endif
			ret = dct_flash_write(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, sizeof(module_info));
			// free module mutex lock
			dct_mutex_unlock(MODULE_MUTEX(module_idx % MODULE_NUM));
			dct_mutex_free(MODULE_MUTEX(module_idx % MODULE_NUM));
			DCT_DBG_INFO("add new module %d\n", module_idx);
			ret = DCT_SUCCESS;
		}
		break;
	}
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_unregister_module(char *module_name)
{
	int32_t		ret;
	uint16_t	module_idx;

	if (!g_dct_adapter) {
		DCT_DBG_RESIDENT("init DCT first\n");
		return DCT_ERROR;
	}

	// find module
	ret = _dct_find_module((uint8_t *)module_name, &module_idx);
	if (ret != DCT_SUCCESS) {
		DCT_DBG_WARNING("not find module\n");
		return DCT_SUCCESS;
	}
	// get module mutex lock
	if (!*(MODULE_MUTEX(module_idx % MODULE_NUM))) {
		dct_mutex_init(MODULE_MUTEX(module_idx % MODULE_NUM));
	}
	if (dct_mutex_lock(MODULE_MUTEX(module_idx % MODULE_NUM), DCT_MUTEX_TIMEOUT) < 0) {
		DCT_DBG_ERROR("get module(%d) mutex timeout\n", module_idx);
		return DCT_ERROR;
	}
	// erase and initial to valid module
	ret = _dct_init_valid_module(module_idx, MODULE_NUM, VARIABLE_NAME_SIZE, VARIABLE_VALUE_SIZE, BACKUP_ENABLE_FLAG, WEAR_LEVELING_FLAG);

#if DCT_BACKUP_ENABLE || DCT_WEAR_LEVEL_ENABLE
	if (BACKUP_ENABLE_FLAG || WEAR_LEVELING_FLAG) {
		uint8_t		i;
		// erase backup module
		for (i = 0; i < (MODULE_BACKUP_NUM / MODULE_NUM); i++) {
			module_idx += MODULE_NUM;
			if (module_idx >= (MODULE_NUM + MODULE_BACKUP_NUM)) {
				module_idx %= MODULE_NUM;
			}
			// erase and initial to valid module
			ret = _dct_init_valid_module(module_idx, MODULE_NUM, VARIABLE_NAME_SIZE, VARIABLE_VALUE_SIZE, BACKUP_ENABLE_FLAG, WEAR_LEVELING_FLAG);
		}
	}
#endif
	// free module mutex lock
	dct_mutex_unlock(MODULE_MUTEX(module_idx % MODULE_NUM));
	dct_mutex_free(MODULE_MUTEX(module_idx % MODULE_NUM));
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_format(uint32_t begin_address, uint16_t module_number, uint16_t variable_name_size, uint16_t variable_value_size, uint8_t enable_backup,
				   uint8_t enable_wear_leveling)
{
	int32_t		ret;
	uint16_t	module_idx;
	uint32_t	total_module_number;

	if (g_dct_adapter) {
		dct_deinit();
	}
	// initial DCT adapter
	g_dct_adapter = (dct_adapter_t *)rtw_malloc(sizeof(dct_adapter_t));
	if (!g_dct_adapter) {
		DCT_DBG_ERROR("can't alloc memory!\n");
		return DCT_ERR_NO_MEMORY;
	}
	memset(g_dct_adapter, 0, sizeof(dct_adapter_t));
	ret = _dct_init_adapter(begin_address, module_number, variable_name_size, variable_value_size, enable_backup, enable_wear_leveling);

#if DCT_WEAR_LEVEL_ENABLE
	if (enable_wear_leveling) {
		// check BBT
		ret = _dct_check_bbt();
		if (ret != DCT_SUCCESS) {
			goto dct_format_fail;
		}
	}
#endif

	total_module_number = MODULE_NUM + MODULE_BACKUP_NUM + MODULE_REPLACE_NUM;
	for (module_idx = 0; module_idx < total_module_number; module_idx++) {
		ret = _dct_init_valid_module(module_idx, module_number, variable_name_size, variable_value_size, enable_backup, enable_wear_leveling);
		if (ret != DCT_SUCCESS) {
			goto dct_format_fail;
		}
	}
	DCT_DBG_RESIDENT("DCT format success, please do dct_init() again.\n");

dct_format_fail:
	rtw_mfree((uint8_t *)g_dct_adapter, sizeof(dct_adapter_t));
	g_dct_adapter = NULL;
	return ret;
}

//------------------------------------------------------------------------------
int32_t dct_init(uint32_t begin_address, uint16_t module_number, uint16_t variable_name_size, uint16_t variable_value_size, uint8_t enable_backup,
				 uint8_t enable_wear_leveling)
{
	int32_t	ret;

	if (g_dct_adapter) {
		return DCT_SUCCESS;
	}

#if DCT_WEAR_LEVEL_ENABLE
	if (enable_wear_leveling) {
		if (module_number > MAX_MODULE_NUM) { // due to BBT is 4K bytes
			DCT_DBG_ERROR("module number can't be more than %d!\n", MAX_MODULE_NUM);
			return DCT_ERR_NO_SPACE;
		}
	}
#endif
	// initial flash
	dct_flash_init();

	// initial CRC
	crc32_init();

	// initial DCT adapter
	g_dct_adapter = (dct_adapter_t *)rtw_malloc(sizeof(dct_adapter_t));
	if (!g_dct_adapter) {
		DCT_DBG_ERROR("can't alloc memory!\n");
		return DCT_ERR_NO_MEMORY;
	}
	memset(g_dct_adapter, 0, sizeof(dct_adapter_t));
	ret = _dct_init_adapter(begin_address, module_number, variable_name_size, variable_value_size, enable_backup, enable_wear_leveling);

	MODULE_MUTEX_POINT = (_mutex *)rtw_malloc(sizeof(_mutex) * MODULE_NUM);
	if (!MODULE_MUTEX_POINT)	{
		goto init_error;
	}
	// clear module mutex
	memset(MODULE_MUTEX_POINT, 0, sizeof(_mutex)*MODULE_NUM);
	VARIABLE_NAME_BUF = rtw_malloc(VARIABLE_NAME_SIZE + 1);
	if (!VARIABLE_NAME_BUF) {
		goto init_error;
	}
	VARIABLE_VALUE_BUF = rtw_malloc(VARIABLE_VALUE_SIZE + 1);
	if (!VARIABLE_VALUE_BUF) {
		goto init_error;
	}

	// verify flash layout
	ret = _dct_verify_module();
	return ret;

init_error:
	DCT_DBG_ERROR("can't alloc memory!\n");
	if (g_dct_adapter) {
		if (VARIABLE_VALUE_BUF) {
			rtw_mfree(VARIABLE_VALUE_BUF, VARIABLE_VALUE_SIZE + 1);
		}
		if (VARIABLE_NAME_BUF) {
			rtw_mfree(VARIABLE_NAME_BUF, VARIABLE_NAME_SIZE + 1);
		}
		if (MODULE_MUTEX_POINT) {
			rtw_mfree((uint8_t *)MODULE_MUTEX_POINT, sizeof(_mutex)*MODULE_NUM);
		}
		rtw_mfree((uint8_t *)g_dct_adapter, sizeof(dct_adapter_t));
		g_dct_adapter = NULL;
		ret = DCT_ERR_NO_MEMORY;
	}
	return ret;
}

//------------------------------------------------------------------------------
void dct_deinit(void)
{
	if (g_dct_adapter) {
		rtw_mfree(VARIABLE_VALUE_BUF, VARIABLE_VALUE_SIZE + 1);
		rtw_mfree(VARIABLE_NAME_BUF, VARIABLE_NAME_SIZE + 1);
		rtw_mfree((uint8_t *)MODULE_MUTEX_POINT, sizeof(_mutex)*MODULE_NUM);
		rtw_mfree((uint8_t *)g_dct_adapter, sizeof(dct_adapter_t));
		g_dct_adapter = NULL;
	}
}

//------------------------------------------------------------------------------
void dct_dump_flash(uint32_t flash_address, uint16_t module_idx)
{
	uint32_t	i, j, module_address;
	uint8_t		buf[32] = {0};
	uint16_t	block_idx;

	if (flash_address) {
		module_address = flash_address;
	} else {
		if (!g_dct_adapter) {
			DCT_DBG_RESIDENT("init DCT first\n");
			return;
		}
		block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
		if (WEAR_LEVELING_FLAG) {
			if (_dct_get_block_idx_from_bbt(module_idx, &block_idx) != DCT_SUCCESS) {
				return;
			}
			DCT_DBG_RESIDENT("block idx = %d\n", block_idx);
		}
#endif
		module_address = BLOCK_ADDRESS(block_idx);
	}
	for (i = 0; i < MODULE_SIZE; i += 32) {
		dct_flash_read(module_address + i, buf, sizeof(buf));
		for (j = 0; j < 32; j++) {
			DCT_PRINTK("%02x ", buf[j]);
		}
		DCT_PRINTK("\n");
	}
}

//------------------------------------------------------------------------------
void dct_dump_module(uint16_t module_idx, uint16_t variable_num)
{
	uint32_t			module_signature, module_state, idx;
	uint8_t 			name[MODULE_NAME_SIZE + 1] = {0};
	dct_module_info_t	module_info;
	uint16_t			block_idx;

	if (!g_dct_adapter) {
		DCT_DBG_RESIDENT("init DCT first\n");
		return;
	}
	block_idx = module_idx;
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		if (_dct_get_block_idx_from_bbt(module_idx, &block_idx) != DCT_SUCCESS) {
			return;
		}
		DCT_DBG_RESIDENT("block idx: %d\n", block_idx);
	}
#endif
	DCT_DBG_RESIDENT("module idx: %d [0x%08x]\n", module_idx, (unsigned int)BLOCK_ADDRESS(block_idx));

	// dump signature
	dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_SIGNATURE_OFFSET, (uint8_t *)&module_signature, MODULE_SIGNATURE_SIZE);
	DCT_DBG_RESIDENT("signature: %c%c%c%c\n", *(uint8_t *)&module_signature, *((uint8_t *)&module_signature + 1), *((uint8_t *)&module_signature + 2),
					 *((uint8_t *)&module_signature + 3));

	// dump module state
	dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_STATE_OFFSET, (uint8_t *)&module_state, MODULE_STATE_SIZE);
	switch (module_state) {
	case DCT_MODULE_STATE_INIT:
		strcpy((char *)name, "init");
		break;
	case DCT_MODULE_STATE_VALID:
		strcpy((char *)name, "valid");
		break;
	case DCT_MODULE_STATE_DELETING:
		strcpy((char *)name, "deleting");
		break;
	case DCT_MODULE_STATE_DELETED:
		strcpy((char *)name, "deleted");
		break;
	default:
		strcpy((char *)name, "unknow");
		break;
	}
	DCT_DBG_RESIDENT("module state: 0x%08x (%s)\n", (unsigned int)module_state, name);

	// dump module name
	dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_NAME_OFFSET, name, MODULE_NAME_SIZE);
	DCT_DBG_RESIDENT("module name: %s\n", name);

	// dump module info
	dct_flash_read(BLOCK_ADDRESS(block_idx) + MODULE_INFO_OFFSET, (uint8_t *)&module_info, MODULE_INFO_SIZE);
	DCT_DBG_RESIDENT("crc: 0x%08x\n", (unsigned int)module_info.variable_crc);
	DCT_DBG_RESIDENT("module number: %d, backup: %d, wear leveling: %d\n", module_info.module_number, module_info.enable_backup, module_info.wear_leveling);
	DCT_DBG_RESIDENT("variable name size: %d, variable value size: %d, used variable: %d\n", module_info.variable_name_size, module_info.variable_value_size,
					 module_info.used_variable_num);

	// dump variable
	DCT_DBG_RESIDENT("real used variable num in flash: %d, total variable amount: %d\n", (int)_dct_get_used_variable_num(block_idx), (int)VARIABLE_MAX_NUM);
	for (idx = 0; idx < variable_num; idx++) {
		memset(VARIABLE_NAME_BUF, 0, VARIABLE_NAME_SIZE + 1);
		memset(VARIABLE_VALUE_BUF, 0, VARIABLE_VALUE_SIZE + 1);
		dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET + idx * VARIABLE_ITME_SIZE, VARIABLE_NAME_BUF, VARIABLE_NAME_SIZE);
		dct_flash_read(BLOCK_ADDRESS(block_idx) + VARIABLE_BUF_OFFSET + idx * VARIABLE_ITME_SIZE + VARIABLE_NAME_SIZE, VARIABLE_VALUE_BUF, VARIABLE_VALUE_SIZE);
		if (VARIABLE_NAME_BUF[0] == 0xff) {
			VARIABLE_NAME_BUF[0] = '\0';
		}
		if (VARIABLE_VALUE_BUF[0] == 0xff) {
			VARIABLE_VALUE_BUF[0] = '\0';
		}
		DCT_DBG_RESIDENT("name: %s \t\t value: %s\n", VARIABLE_NAME_BUF, VARIABLE_VALUE_BUF);
	}
}

//------------------------------------------------------------------------------
int32_t dct_dump_bbt(void)
{
	uint16_t			block_idx, module_idx;

	if (!g_dct_adapter) {
		DCT_DBG_RESIDENT("init DCT first\n");
		return DCT_ERROR;
	}
#if DCT_WEAR_LEVEL_ENABLE
	if (WEAR_LEVELING_FLAG) {
		for (module_idx = 0; module_idx < (MODULE_NUM + MODULE_BACKUP_NUM); module_idx++) {
			if (_dct_get_block_idx_from_bbt(module_idx, &block_idx) != DCT_SUCCESS) {
				return DCT_ERROR;
			}
			DCT_DBG_RESIDENT("module idx: %d, block idx: %d [0x%08x]\n", module_idx, block_idx, (unsigned int)BLOCK_ADDRESS(block_idx));
		}
	}
#endif
	return DCT_SUCCESS;
}
//------------------------------------------------------------------------------
void dct_setting(uint8_t debug_level)
{
	// dbg
	g_dct_debug_level = debug_level;
	// cache
}

