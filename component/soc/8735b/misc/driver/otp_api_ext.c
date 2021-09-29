/**************************************************************************//**
* @file 	   otp_api_ext.c
* @brief 	  This file implements the OTP Mbed HAL API functions.
*
* @version	 V1.00
* @date 	   2019-01-15
*
* @note
*
******************************************************************************
*
* Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
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
#include "hal_otp.h"
#include "hal_otp_nsc.h"
#include "hal.h"
#include "memory.h"
#include "otp_api_ext.h"

//#define OTP_LOGICAL_SIM

#define OTP_LOGICAL_MAP				0x04
#define OTP_LOGICAL_MAP_SIZE		0x1D0 // 464
#define OTP_LOGICAL_MAP_HW_SIZE		0x210 // 528

#define OTP_PHY_WIFI_RF_K			0x4C0
#define OTP_PHY_WIFI_RF_K_SIZE		80
#define OTP_PHY_BT_PARA				0x510
#define OTP_PHY_BT_PARA_SIZE		48

u8 otp_inited = 0;

#ifdef OTP_LOGICAL_SIM
u8 otp_sim_map[OTP_LOGICAL_MAP_HW_SIZE];
#endif

static int otp_pg_packet(u8 offset, u8 wden, u8 *data)
{
	u16 idx = 4;//the first 4 addresses are reserved
	u8 temp0, temp1, WordEn;
	u8 Len = 0;
	u8 word_idx;

	/* WordEnable bit=0 means write this bit */
	if ((wden & 0xF) == 0xF) {
		return -1;
	}

	if (offset == 0) {
		if (!(wden & BIT(0)) || !(wden & BIT(1))) {
			dbg_printf("\r\nCan't program Word0 for Offset 0,1\r\n");
			return -1;
		}
	}
	//count the physical written num of word
	while (idx < OTP_LOGICAL_MAP_HW_SIZE) {
#ifdef OTP_LOGICAL_SIM
		temp0 = otp_sim_map[idx];
#else
		temp0 = hal_otp_byte_rd_syss(idx);
#endif
		if (temp0 != 0xff) {//used
			idx++;
#ifdef OTP_LOGICAL_SIM
			temp1 = otp_sim_map[idx];
#else
			temp1 = hal_otp_byte_rd_syss(idx);
#endif
			//Addr = ( ((temp0&0x0f)<<4) | ((temp1&0xf0)>>4) );//logical addr
			WordEn = ((~temp1) & 0x0f); //~ write enbale

			// bit=0: word not write
			// bit=1: word have write
			while (WordEn != 0) {
				if (WordEn & BIT0) {
					idx = idx + 2; //word have write
				}
				WordEn = WordEn >> 1;
			}
		} else {//find address not written
			break;
		}
		idx++;
	}
	//CODE WORD
	wden = wden & 0xf;

	if ((wden & BIT(0)) == 0)	{
		Len = Len + 2;    // 0 : write enable
	}
	if ((wden & BIT(1)) == 0)	{
		Len = Len + 2;
	}
	if ((wden & BIT(2)) == 0)	{
		Len = Len + 2;
	}
	if ((wden & BIT(3)) == 0)	{
		Len = Len + 2;
	}

	if ((idx + Len) < (OTP_LOGICAL_MAP_HW_SIZE)) {
#ifdef OTP_LOGICAL_SIM
		otp_sim_map[idx] = (((offset >> 4) & 0x0f) | 0x30);
#else
		hal_otp_byte_wr_syss(idx, ((offset >> 4) & 0x0f) | 0x30); //addr[7:4]
#endif
		idx++;
#ifdef OTP_LOGICAL_SIM
		otp_sim_map[idx] = (((offset & 0x0f) << 4) | wden);
#else
		hal_otp_byte_wr_syss(idx, ((offset & 0x0f) << 4) | wden); //addr[3:0]
#endif
		idx++;
		for (word_idx = 0; word_idx < 4; word_idx ++) {
			if ((wden & BIT(word_idx)) == 0) {
#ifdef OTP_LOGICAL_SIM
				otp_sim_map[idx] = *(data + word_idx * 2);
#else
				hal_otp_byte_wr_syss(idx, *(data + word_idx * 2));
#endif
				idx++;
#ifdef OTP_LOGICAL_SIM
				otp_sim_map[idx] = *(data + word_idx * 2 + 1);
#else
				hal_otp_byte_wr_syss(idx, *(data + word_idx * 2 + 1));
#endif
				idx++;
			}
		}
	} else {
		dbg_printf("OTP PG No Enough Space!\n");
		return -1;
	}
	return 0;
}

int otp_logical_write(u16 addr, u16 cnts, u8 *data)
{
	u8	offset, word_en;
	u8	map[OTP_LOGICAL_MAP_SIZE];
	u16	mapLen = OTP_LOGICAL_MAP_SIZE;
	u8	newdata[8];
	s32	i, idx;
	int	ret = 0, used_bytes;

	if ((addr + cnts) > mapLen) {
		return -1;
	}

	used_bytes = otp_logical_read(0, mapLen, map);
	if (used_bytes < 0) {
		return -1;
	}

	offset = (addr >> 3);
	word_en = 0xF;
	memset(newdata, 0xFF, 8);
	i = addr & 0x7;	// index of one package
	idx = 0;	// data index

	if (i & 0x1) {
		// odd start
		if (data[idx] != map[addr + idx]) {
			word_en &= ~(BIT(i >> 1));
			newdata[i - 1] = map[addr + idx - 1];
			newdata[i] = data[idx];
		}
		i++;
		idx++;
	}
	do {
		for (; i < 8; i += 2) {
			if (cnts == idx) {
				break;
			}
			if ((cnts - idx) == 1) {
				if (data[idx] != map[addr + idx]) {
					word_en &= ~(BIT(i >> 1));
					newdata[i] = data[idx];
					newdata[i + 1] = map[addr + idx + 1];
				}
				idx++;
				break;
			} else {
				if ((data[idx] != map[addr + idx]) ||
					(data[idx + 1] != map[addr + idx + 1])) {
					word_en &= ~(BIT(i >> 1));
					newdata[i] = data[idx];
					newdata[i + 1] = data[idx + 1];
				}
				idx += 2;
			}
			if (idx == cnts) {
				break;
			}
		}

		if (word_en != 0xF) {
			ret = otp_pg_packet(offset, word_en, newdata);
			/*
			dbg_printf("offset=%x \n",offset);
			dbg_printf("word_en=%x \n",word_en);

			for(i=0;i<8;i++)
			{
			dbg_printf("data=%x \t",newdata[i]);
			}
			*/
			if (ret < 0) {
				return ret;
			}
		}

		if (idx == cnts) {
			break;
		}

		offset++;
		i = 0;
		word_en = 0xF;
		memset(newdata, 0xFF, 8);
	} while (1);
#ifdef OTP_LOGICAL_SIM
	dbg_printf("\n\r");
	for (i = 0; i < OTP_LOGICAL_MAP_HW_SIZE; i += 16) {
		int j;
		dbg_printf("0x%03x\t", i);
		for (j = 0; j < 8; j++) {
			dbg_printf("%02X ", otp_sim_map[i + j]);
		}
		dbg_printf("\t");
		for (; j < 16; j++) {
			dbg_printf("%02X ", otp_sim_map[i + j]);
		}
		dbg_printf("\n\r");
	}
#endif
	return 0;
}

int otp_logical_read(u16 laddr, u16 size, u8 *pbuf)
{
	u16 phy_addr = 0;
	u8 offset, wden;
	u8 header, extheader, data;
	u16 i;

	if (!pbuf) {
		return -1;
	}
	memset(pbuf, 0xFF, size);

	if (!otp_inited) {
		hal_otp_init();
		otp_inited = 1;
	}

	/*Logical map size 464 (0x1D0)*/
	/*Logical map region on physical otp: 0x0 ~ 0x210*/
	while (phy_addr < OTP_LOGICAL_MAP_HW_SIZE) {
		/*First 4 bytes are reserved for physical*/
		if (phy_addr == 0 || phy_addr == 1 || phy_addr == 2 || phy_addr == 3) {
			phy_addr++;
			continue;
		}
#ifdef OTP_LOGICAL_SIM
		static int map_inited = 0;
		if (!map_inited) {
			u32 ret;
			map_inited = 1;
#if 1
			ret = hal_otp_rd_syss(0, 0, otp_sim_map, OTP_LOGICAL_MAP_HW_SIZE);
			if (ret != OTPStsSuccess) {
				return -1;
			}
#else
			memset(otp_sim_map, 0xff, sizeof(otp_sim_map));
#endif
		}
		header = otp_sim_map[phy_addr++];
#else
		header = hal_otp_byte_rd_syss(phy_addr++);
#endif
		if (header == 0xFF) {/* not write */
			break;
		}

		/* Check PG header for section num. */
#ifdef OTP_LOGICAL_SIM
		extheader = otp_sim_map[phy_addr++];
#else
		extheader = hal_otp_byte_rd_syss(phy_addr++);
#endif
		offset = (header & 0x0F) << 4;
		offset |= ((extheader & 0xF0) >> 4);
		wden = (extheader & 0x0F);

		/*One section has 8 bytes data, logical map has 512/8 = 64 sections*/
		if (offset < (OTP_LOGICAL_MAP_SIZE >> 3)) {
			u16 addr = 0;
			/* Get word enable value from PG header */
			addr = offset * 8;
			/*Each section has 4 words data*/
			for (i = 0; i < 4; i++) {
				/* Check word enable condition in the section */
				if (!(wden & (0x01 << i))) {
#ifdef OTP_LOGICAL_SIM
					data = otp_sim_map[phy_addr++];
#else
					data = hal_otp_byte_rd_syss(phy_addr++);
#endif
					if (addr >= laddr && addr < (laddr + size)) {
						pbuf[addr - laddr] = data;
					}
#ifdef OTP_LOGICAL_SIM
					data = otp_sim_map[phy_addr++];
#else
					data = hal_otp_byte_rd_syss(phy_addr++);
#endif
					if ((addr + 1) >= laddr && (addr + 1) < (laddr + size)) {
						pbuf[addr + 1 - laddr] = data;
					}
				}
				addr += 2;
			}
		} else {
			u8 word_cnts = 0;
			if (!(wden & BIT(0)))	{
				word_cnts++;    // 0 : write enable
			}
			if (!(wden & BIT(1)))	{
				word_cnts++;
			}
			if (!(wden & BIT(2)))	{
				word_cnts++;
			}
			if (!(wden & BIT(3)))	{
				word_cnts++;
			}
			phy_addr += word_cnts * 2; //word
		}
	}

	/*return used bytes*/
	return phy_addr - 1;
}

int otp_logical_remain(void)
{
	u16 phy_addr = 0;
	u8 header, extheader, data;
	u8 wden, word_cnts;

	/*First 4 bytes are reserved for physical*/
	phy_addr = 4;
	while (phy_addr < OTP_LOGICAL_MAP_HW_SIZE) {
#ifdef OTP_LOGICAL_SIM
		header = otp_sim_map[phy_addr];
#else
		header = hal_otp_byte_rd_syss(phy_addr);
#endif
		if (header == 0xFF) {/* not write */
			break;
		}
		phy_addr++;
#ifdef OTP_LOGICAL_SIM
		extheader = otp_sim_map[phy_addr];
#else
		extheader = hal_otp_byte_rd_syss(phy_addr);
#endif

		wden = (extheader & 0x0F);
		word_cnts = 0;
		if (!(wden & BIT(0)))	{
			word_cnts++;    // 0 : write enable
		}
		if (!(wden & BIT(1)))	{
			word_cnts++;
		}
		if (!(wden & BIT(2)))	{
			word_cnts++;
		}
		if (!(wden & BIT(3)))	{
			word_cnts++;
		}
		phy_addr += word_cnts * 2; //word

		phy_addr++;
	}
	return (OTP_LOGICAL_MAP_HW_SIZE - phy_addr);
}

u8 otp_byte_write(u32 addr, u8 data)
{
	// only read valid block to protect hidden zone
	if (((addr >= OTP_LOGICAL_MAP) && (addr < (OTP_LOGICAL_MAP + OTP_LOGICAL_MAP_SIZE))) ||
		((addr >= OTP_PHY_WIFI_RF_K) && (addr < (OTP_PHY_WIFI_RF_K + OTP_PHY_WIFI_RF_K_SIZE))) ||
		((addr >= OTP_PHY_BT_PARA) && (addr < (OTP_PHY_BT_PARA + OTP_PHY_BT_PARA_SIZE)))) {
		hal_otp_byte_wr_syss(addr, data);
	} else {
		return 0;
	}
	return 1;
}

u8 otp_byte_read(u32 addr)
{
	// only read valid block to protect hidden zone
	if (((addr >= OTP_LOGICAL_MAP) && (addr < (OTP_LOGICAL_MAP + OTP_LOGICAL_MAP_SIZE))) ||
		((addr >= OTP_PHY_WIFI_RF_K) && (addr < (OTP_PHY_WIFI_RF_K + OTP_PHY_WIFI_RF_K_SIZE))) ||
		((addr >= OTP_PHY_BT_PARA) && (addr < (OTP_PHY_BT_PARA + OTP_PHY_BT_PARA_SIZE)))) {
		return hal_otp_byte_rd_syss(addr);
	} else {
		return 0xFF;
	}
}

