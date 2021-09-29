/******************************************************************************
 *
 * Copyright(c) 2019 - 2021 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#include "dfudep_service.h"
#include "os_sched.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mesh_dfu_8710c.h"

uint32_t mesh_dfu_address = 0;
uint32_t image_left_size = 0;
uint32_t new_fw_addr = 0;
uint32_t idx = 0;
unsigned char sig_backup[32];

static const uint32_t dfu_crc32_table[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t dfu_crc32_get(uint8_t *buf, uint16_t len)
{
	uint8_t *p;
	uint32_t crc;

	crc = 0xffffffff;       /* preload shift register, per CRC-32 spec */

	for (p = buf; len > 0; ++p, --len) {
		crc = dfu_crc32_table[(crc ^ *p) & 0xff] ^ (crc >> 8);
	}

	return ~crc;    /* transmit complement, per CRC-32 spec */
}

uint8_t customer_dfu_metadata_check(void *pmetadata, uint32_t len)
{
	return DFU_RESULT_OK;
}

uint8_t customer_dfu_resources_check(void)
{
	return DFU_RESULT_OK;
}

#include "blob_transfer.h"

extern blob_transfer_server_ctx_t blob_transfer_server_ctx;

uint8_t customer_dfu_init_dfu_resources(void)
{
	uint32_t curr_fw_idx = 0, img_len = 0, new_fw_len = 0;

	new_fw_addr = mesh_dfu_prepare_addr();
	if (new_fw_addr == 0) {
		printf("customer_dfu_init_dfu_resources: get new firmware address fail \r\n");
		return DFU_RESULT_FAIL;
	}
	mesh_dfu_address = new_fw_addr;
	image_left_size = blob_transfer_server_ctx.blob_size;
	idx = 0;
	memset(sig_backup, 0, 32);
	curr_fw_idx = mesh_dfu_get_curr_fw_idx();
	img_len = blob_transfer_server_ctx.blob_size - blob_transfer_server_ctx.total_blocks * DFU_BLOCK_SIGNATURE_SIZE;
	printf("customer_dfu_init_dfu_resources: Current firmware index is %d\r\n", curr_fw_idx);
	printf("customer_dfu_init_dfu_resources: New firmware address is 0x%x\r\n", new_fw_addr);
	new_fw_len = mesh_dfu_erase_upg_region(img_len, new_fw_len, new_fw_addr);
	if (new_fw_len == -1) {
		mesh_dfu_address = 0;
		image_left_size = 0;
		new_fw_addr = 0;
		idx = 0;
		printf("customer_dfu_init_dfu_resources: Fail \r\n");
		return DFU_RESULT_FAIL;
	}

	return DFU_RESULT_OK;
}

uint8_t customer_dfu_block_verify(uint8_t *data, uint16_t len, uint8_t signature_size)
{
	uint32_t crc = 0;

	for (uint32_t i = 0; i < len; i++) {
		if (i % 12 == 0) {
			printf("\r\n");
		}
		printf(" 0x%02x ", data[i]);
	}

	if (signature_size == 4) {
		uint8_t signature[4] = {0};

		crc = dfu_crc32_get(data, len - signature_size);
		// printf(" customer_dfu_block_signature crc = 0x%08x \r\n", crc);
		signature[0] = (uint8_t)(crc & 0x000000FF);
		signature[1] = (uint8_t)((crc & 0x0000FF00) >> 8);
		signature[2] = (uint8_t)((crc & 0x00FF0000) >> 16);
		signature[3] = (uint8_t)((crc & 0xFF000000) >> 24);

		printf("\r\n customer_dfu_block_verify %x, %x, %x, %x \r\n", signature[0], signature[1], signature[2], signature[3]);
		if (signature[0] == data[len - 4] && signature[1] == data[len - 3] && \
			signature[2] == data[len - 2] && signature[3] == data[len - 1]) {
			printf("customer_dfu_block_verify block verify success");
			return DFU_RESULT_OK;
		} else {
			printf("dfu_transfer_client_data: ========FAIL=========\r\n");
			return DFU_RESULT_FAIL;
		}
	}

	return DFU_RESULT_OK;
}

uint8_t customer_dfu_verify(uint16_t image_id)
{
	return DFU_RESULT_OK;
}

uint8_t customer_dfu_cancel(void)
{
	return DFU_RESULT_OK;
}

uint8_t customer_dfu_verify_cancel(void)
{
	return DFU_RESULT_OK;
}

uint8_t customer_dfu_apply(uint8_t reason, uint32_t delay_ms)
{
	os_delay(delay_ms);
	mesh_dfu_platform_reset();

	return DFU_RESULT_OK;
}

uint8_t customer_dfu_block_data_restore(uint8_t *data, uint32_t len, uint8_t signature_size)
{
	// printf("dfu_transfer_client_data: ========Received one block========= len = %d \r\n", len);
	// for (uint32_t i = 0; i < len; i++) {
	//     if (i % 12 == 0) {
	//         printf("\r\n");
	//     }
	//     printf(" 0x%02x ", data[i]);
	// }

	/* backup the signature of the first 32 bytes, in case ota fial */
	if (idx < 32) {
		memcpy(sig_backup + idx, data, (idx + len - signature_size > 32 ? (32 - idx) : len - signature_size));
		memset(data, 0xFF, (idx + len - signature_size > 32 ? (32 - idx) : len - signature_size));
		printf("[%s] sig_backup for %d bytes from %d index\r\n", __FUNCTION__, (idx + len - signature_size > 32 ? (32 - idx) : len - signature_size), idx);
	}

	if (!mesh_dfu_flash_write(mesh_dfu_address, len - signature_size, data)) {
		printf("customer_dfu_block_data_restore: fail \r\n");
		mesh_dfu_address = 0;
		image_left_size = 0;
		new_fw_addr = 0;
		idx = 0;
		return DFU_RESULT_FAIL;
	}
	printf("customer_dfu_block_data_restore: write addr 0x%x , block size %d success  \r\n", mesh_dfu_address, len - signature_size);
	mesh_dfu_address += len - signature_size;
	idx += len - signature_size;
	if (image_left_size > len) {
		image_left_size -= len;
	} else {
		image_left_size = 0;
	}

	if (image_left_size == 0) {
		printf("customer_dfu_block_data_restore: Complete receiving firmware \r\n");
		printf("customer_dfu_block_data_restore: Start updating signature \r\n");
		// update ota signature at the end of OTA process
		if (mesh_update_ota_signature(sig_backup, new_fw_addr) == -1) {
			printf("customer_dfu_block_data_restore: Update signature fail \r\n");
			mesh_dfu_address = 0;
			image_left_size = 0;
			new_fw_addr = 0;
			idx = 0;
			return DFU_RESULT_FAIL;
		}
	}

	return DFU_RESULT_OK;
}

void customer_dfu_failsafe(void)
{
	mesh_dfu_address = 0;
	image_left_size = 0;
	new_fw_addr = 0;
	idx = 0;
	memset(sig_backup, 0, 32);

	return;
}

uint8_t customer_is_dfu_enabled(void)
{
	return DFU_RESULT_OK;
}

uint8_t customer_dfu_get_fw_info(fw_info_t *pfw_info)
{
	pfw_info->fw_id.company_id = 0x005D;
	pfw_info->fw_id.version[0] = 0x00;
	pfw_info->fw_id.version[1] = 0x01;
	pfw_info->fw_id_len = 4;
	pfw_info->update_uri_len = 0;

	return DFU_RESULT_OK;
}
#include <flash_api.h>

extern const unsigned char rtlbt_fw[];
extern uint32_t test_image_addr_base;
extern flash_t mesh_flash_ota;

uint32_t customer_dfu_fw_image_data_get(uint32_t len, uint8_t *pout)
{
	// static uint32_t i = 0;

	// memcpy(pout, &rtlbt_fw[i], len);
	// i += len;
	// printf("customer_dfu_fw_image_data_get: len = %d", len);

	// return DFU_RESULT_OK;

	static uint32_t i = 0;

	printf("get flash stream \r\n");
	flash_stream_read(&mesh_flash_ota, 0x104000 + i, len, pout);
	i += len;
	printf("customer_dfu_fw_image_data_get: len = %d", len);

	return DFU_RESULT_OK;
}

uint8_t customer_dfu_block_signature(uint8_t *data, uint16_t len, uint8_t signature_size)
{
	uint32_t crc = 0;

	// printf("\r\n customer_dfu_block_signature previous data \r\n");
	// for (uint32_t i = 0; i <  len; i++) {
	//     if (i % 12 == 0) {
	//         printf("\r\n");
	//     }
	//     printf(" 0x%02x ", data[i]);
	// }
	// printf("\r\n");

	if (signature_size == 4) {
		crc = dfu_crc32_get(data, len);
		// printf(" customer_dfu_block_signature crc = 0x%08x \r\n", crc);
		data[len] = (uint8_t)(crc & 0x000000FF);
		data[len + 1] = (uint8_t)((crc & 0x0000FF00) >> 8);
		data[len + 2] = (uint8_t)((crc & 0x00FF0000) >> 16);
		data[len + 3] = (uint8_t)((crc & 0xFF000000) >> 24);
	}

	// printf("\r\n customer_dfu_block_signature CRC32 data \r\n");
	// for (uint32_t i = 0; i <  len + signature_size; i++) {
	//     if (i % 12 == 0) {
	//         printf("\r\n");
	//     }
	//     printf(" 0x%02x ", data[i]);
	// }
	// printf("\r\n");

	return DFU_RESULT_OK;
}

uint8_t customer_dfu_check_fw_info(fw_info_t *pfw_info)
{
	return DFU_RESULT_OK;
}

const struct dfudep_service_ops dfudep_service = {
	/* updater server api */
	customer_dfu_metadata_check,                //dfu_metadata_check
	customer_dfu_resources_check,               //dfu_resources_check
	customer_dfu_init_dfu_resources,            //dfu_init_dfu_resources
	customer_dfu_block_verify,                  //dfu_block_verify
	customer_dfu_verify,                        //dfu_verify
	customer_dfu_cancel,                        //dfu_cancel
	customer_dfu_verify_cancel,                 //dfu_verify_cancel
	customer_dfu_apply,                         //dfu_apply
	customer_dfu_block_data_restore,            //dfu_block_data_restore
	customer_dfu_failsafe,                      //dfu_failsafe
	customer_is_dfu_enabled,                    //is_dfu_enabled
	customer_dfu_get_fw_info,                   //dfu_get_fw_info
	/* updater client api */
	customer_dfu_fw_image_data_get,             //dfu_fw_image_data_get
	customer_dfu_block_signature,               //dfu_block_signature
	customer_dfu_check_fw_info                  //dfu_check_fw_info
};