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

#include "crc32.h"

#define TABLE_SIZE 256
#define POLYNOMIAL 0x04c11db7L      // Standard CRC-32 ppolynomial

static uint32_t crc_table[TABLE_SIZE];

void crc32_init(void)
{
	unsigned int i, crc_accum = 0;
	unsigned char j;

	for (i = 0;  i < TABLE_SIZE;  i++) {
		crc_accum = ((unsigned int) i << 24);
		for (j = 0;  j < 8;  j++) {
			if (crc_accum & 0x80000000) {
				crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
			} else {
				crc_accum = (crc_accum << 1);
			}
		}
		crc_table[i] = crc_accum;
	}
}

uint32_t crc32(void *data, uint32_t data_size, uint32_t crc)
{
	unsigned int i, j;
	uint8_t *u8_data = data;

	for (j = 0; j < data_size; j++) {
		i = ((uint32_t)(crc >> 24) ^ (*u8_data)) & 0xFF;
		crc = (crc << 8) ^ crc_table[i];
		u8_data++;
	}
	crc = ~crc;
	return crc;
}

