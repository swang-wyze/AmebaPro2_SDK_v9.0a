/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#include "osif.h"
#include "hci/hci_common.h"
#include "hci_uart.h"
#include "hci_platform.h"
#include "hci_dbg.h"
#include "flash_api.h"
#include "efuse_logical_api.h"
#include "hal_efuse_nsc.h"
#include "bt_intf.h"
#include "wifi_conf.h"
#include "device_lock.h"
#include "rtl8710c_bt_calibration.h"

#define HCI_LGC_EFUSE_LEN          0x20
#define HCI_PHY_EFUSE_LEN          0x13
#define HCI_PHY_EFUSE_BASE         0x100
#define HCI_LGC_EFUSE_OFFSET       0x190

#define HCI_MAC_ADDR_LEN           6
#define HCI_CONFIG_SIGNATURE       0x8723ab55
#define HCI_CONFIG_HDR_LEN         6

#define HCI_MERGE_PATCH_ADDRESS    0x110000

#define HCI_PATCH_FRAG_SIZE        252

#define HCI_CFG_BAUDRATE           BIT0
#define HCI_CFG_FLOWCONTROL        BIT1
#define HCI_CFG_BD_ADDR            BIT2
#define LEFUSE(x)                  ((x) - HCI_LGC_EFUSE_OFFSET)
#define HCI_CHIP_VER               (((HAL_READ32(0x400001F0, 0) & (BIT4|BIT5|BIT6|BIT7)) >> 4) + 1)

#define LE_ARRAY_TO_UINT16(u16, a)          \
    {                                       \
        u16 = ((uint16_t)(*(a + 0)) << 0) + \
              ((uint16_t)(*(a + 1)) << 8);  \
    }

#define LE_ARRAY_TO_UINT32(u32, a)           \
    {                                        \
        u32 = ((uint32_t)(*(a + 0)) << 0) +  \
              ((uint32_t)(*(a + 1)) << 8) +  \
              ((uint32_t)(*(a + 2)) << 16) + \
              ((uint32_t)(*(a + 3)) << 24);  \
    }

#define LE_UINT32_TO_ARRAY(a, u32)                      \
    {                                                   \
        *((uint8_t *)(a) + 0) = (uint8_t)((u32) >> 0);  \
        *((uint8_t *)(a) + 1) = (uint8_t)((u32) >> 8);  \
        *((uint8_t *)(a) + 2) = (uint8_t)((u32) >> 16); \
        *((uint8_t *)(a) + 3) = (uint8_t)((u32) >> 24); \
    }

typedef struct {
    uint8_t *fw_buf;
    uint8_t fw_is_alloced;
    uint16_t fw_len;
    uint8_t *config_buf;
    uint8_t cfg_is_alloced;
    uint16_t config_len;
    uint8_t cur_index;
    uint8_t end_index;
    uint8_t last_pkt;
    uint32_t sent_len;
} HCI_PATCH_INFO;

static BT_Cali_TypeDef iqk_data = {0x100, 0x00, 0x20, 0x20, 0x20, 0x20};

static HCI_PATCH_INFO* hci_patch_info = NULL;

HCI_IQK_DATA hci_iqk_data[HCI_START_IQK_TIMES] = {
    {0x00, 0x4000}, {0x01, 0x0f88}, {0x02, 0x3400}, {0x3f, 0x0700},
};

uint32_t hci_sw_val = 0xffffffff;

static uint8_t hci_phy_efuse[HCI_PHY_EFUSE_LEN]  = {0};
static uint8_t hci_lgc_efuse[HCI_LGC_EFUSE_LEN]  = {0};
static uint8_t hci_cfg_bd_addr[HCI_MAC_ADDR_LEN] = {0};
static uint8_t hci_chipid_in_fw                  = 0;

static uint32_t hci_cfg_flag                     = 0;
static uint32_t hci_cfg_init_uart_baudrate       = 115200;
static uint32_t hci_cfg_work_uart_baudrate       = 0;
static uint8_t  hci_cfg_work_bt_baudrate[4]      = {0};

static uint8_t hci_init_config[] =
{
  0x55, 0xab, 0x23, 0x87,                                 /* header */
  0x32, 0x00,                                             /* Config length: header + len + preload */
  0x30, 0x00, 0x06, 0x99, 0x88, 0x77, 0x44, 0x55, 0x66,   /* BT MAC address */
  0x0c, 0x00, 0x04, 0x04, 0x50, 0xF7, 0x05,               /* Baudrate 921600 */
  0x18, 0x00, 0x01, 0x5c,                                 /* flow control */
  0x94, 0x01, 0x06, 0x08, 0x00, 0x00, 0x00, 0x2e, 0x07,   /* phy flatk */
  0x9f, 0x01, 0x05, 0x2a, 0x2a, 0x2a, 0x2a, 0x50,         /* unknow 1 */
  0xA4, 0x01, 0x04, 0xfe, 0xfe, 0xfe, 0xfe,               /* unknow 2 */
};

#if 0
void bt_only_enable_func(void)
{
	HCI_DBG("BT HCI UART OUT ONLY ...\n");
	set_reg_value(0x40000244, BIT9 | BIT8, 3);
	osif_delay(5);

	//BT LOG ENABLE PA14
	set_reg_value(0x400000cc, BIT2 | BIT1 | BIT0, 6);
	osif_delay(5);
	set_reg_value(0x400000cc, BIT8, 1);
	osif_delay(5);
	//PA15  BT_UART_IN
	set_reg_value(0x400000CC, BIT18 | BIT17 | BIT16, 6);
	osif_delay(5);
	set_reg_value(0x400000CC, BIT24, 1);
	osif_delay(5);

	//PA16 BT_UART_OUT
	set_reg_value(0x400000D0, BIT2 | BIT1 | BIT0, 6);
	osif_delay(5);
	set_reg_value(0x400000D0, BIT8, 1);
	osif_delay(5);
	set_reg_value(0x40000214, BIT29, 1);
	osif_delay(5);
	//HCI
	//Enale BT block
	set_reg_value(0x40000214, BIT24 | BIT25, 0);
	osif_delay(200);
	set_reg_value(0x40000214, BIT24 | BIT25, 3);
	HCI_DBG("BT Reset ok\n");
	osif_delay(200);
}

void bt_enable_func_uart_only(void)
{
	HCI_DBG("BT HCI UART OUT ONLY ...\n");

	//PA15  BT_UART_IN
	set_reg_value(0x400000CC, BIT18 | BIT17 | BIT16, 6);
	osif_delay(5);
	set_reg_value(0x400000CC, BIT24, 1);
	osif_delay(5);

	//PA16 BT_UART_OUT
	set_reg_value(0x400000D0, BIT2 | BIT1 | BIT0, 6);
	osif_delay(5);
	set_reg_value(0x400000D0, BIT8, 1);
	osif_delay(5);
	set_reg_value(0x40000214, BIT29, 1);
	osif_delay(5);
	//HCI
}

void hci_uart_out(void)
{
	HCI_DBG("HCI UART OUT OK: PA15 RX, PA16 TX");
	osif_delay(100);

	//PA15  BT_UART_IN
	set_reg_value(0x400000CC, BIT18 | BIT17 | BIT16, 6);
	osif_delay(5);
	set_reg_value(0x400000CC, BIT24, 1);
	osif_delay(5);

	//PA16 BT_UART_OUT
	set_reg_value(0x400000D0, BIT2 | BIT1 | BIT0, 6);
	osif_delay(5);
	set_reg_value(0x400000D0, BIT8, 1);
	osif_delay(5);
	set_reg_value(0x40000214, BIT29, 1);
	osif_delay(5);
}

static uint8_t mp_driver_init_done = 0;
uint8_t bt_mp_driver_init_done(void)
{
	return mp_driver_init_done;
}

void bt_write_lgc_efuse_value(void)
{
	hci_lgc_efuse[0x16] = iqk_data.IQK_xx & 0xff;
	hci_lgc_efuse[0x17] = (iqk_data.IQK_xx >> 8) & 0xff;
	hci_lgc_efuse[0x18] = iqk_data.IQK_yy & 0xff;
	hci_lgc_efuse[0x19] = (iqk_data.IQK_yy >> 8) & 0xff;
	hci_lgc_efuse[0x1a] = iqk_data.QDAC;
	hci_lgc_efuse[0x1b] = iqk_data.IDAC;
	hci_lgc_efuse[0x1c] = iqk_data.QDAC2;
	hci_lgc_efuse[0x1d] = iqk_data.IDAC2;

	HCI_DBG(" write logic efuse 0x1A6 =0x%02x", hci_lgc_efuse[0x16]);
	HCI_DBG(" write logic efuse 0x1A7 =0x%02x", hci_lgc_efuse[0x17]);
	HCI_DBG(" write logic efuse 0x1A8 =0x%02x", hci_lgc_efuse[0x18]);
	HCI_DBG(" write logic efuse 0x1A9 =0x%02x", hci_lgc_efuse[0x19]);
	HCI_DBG(" write logic efuse 0x1Aa =0x%02x", hci_lgc_efuse[0x1a]);
	HCI_DBG(" write logic efuse 0x1Ab =0x%02x", hci_lgc_efuse[0x1b]);
	HCI_DBG(" write logic efuse 0x1Ac =0x%02x", hci_lgc_efuse[0x1c]);
	HCI_DBG(" write logic efuse 0x1Ad =0x%02x", hci_lgc_efuse[0x1d]);
	//EFUSE_LMAP_WRITE(0x1A4,8,(uint8_t *)&hci_lgc_efuse[0x14]);
}
#endif

/******************************************************************************
 * Read Register About
 */
static uint32_t cal_bit_shift(uint32_t Mask)
{
    uint32_t i;
    for (i = 0; i < 31; i++) {
            if (((Mask >> i) & 0x1) == 1)
                    break;
    }
    return (i);
}

static void set_reg_value(uint32_t reg_address, uint32_t Mask, uint32_t val)
{
    uint32_t shift = 0;
    uint32_t data = 0;

    data = HAL_READ32(reg_address, 0);
    shift = cal_bit_shift(Mask);
    data = ((data & (~Mask)) | (val << shift));
    HAL_WRITE32(reg_address, 0, data);
    data = HAL_READ32(reg_address, 0);
}

/******************************************************************************
 * Efuse About
 */
static uint8_t hci_platform_read_efuse(void)
{
	device_mutex_lock(RT_DEV_LOCK_EFUSE);
    /* Read phy_efuse */
	for (int i = 0; i < 16; i++)
		hal_efuse_read(HCI_PHY_EFUSE_BASE + i, hci_phy_efuse + i, LDO_OUT_DEFAULT_VOLT);

	hal_efuse_read(0xF8, hci_phy_efuse + 16, LDO_OUT_DEFAULT_VOLT);
	hal_efuse_read(0xF9, hci_phy_efuse + 17, LDO_OUT_DEFAULT_VOLT);

	/* Read logic_efuse */
	efuse_logical_read(HCI_LGC_EFUSE_OFFSET, 0x20, hci_lgc_efuse);
	device_mutex_unlock(RT_DEV_LOCK_EFUSE);

    /* Dump phy_efuse and logic_efuse */
	if (!CHECK_SW(EFUSE_SW_DRIVER_DEBUG_LOG)) {
		HCI_PRINT("Dump physical efuse: \n\r");
		for (int i = 0; i < 18; i++)
			HCI_PRINT("%02x ", hci_phy_efuse[i]);
		
		HCI_PRINT("\n\rlogic efuse:\n\r");
		for (int i = 0; i < 0x20; i++)
			HCI_PRINT("%02x ", hci_lgc_efuse[i]);

        HCI_PRINT("\n\r");
	}

	return HCI_SUCCESS;
}

/******************************************************************************
 * Baudrate About
 */
static void hci_platform_convert_baudrate(uint32_t *bt_baudrate, uint32_t *uart_baudrate, uint8_t bt_to_uart)
{
    uint8_t i;

    const struct {
        uint32_t bt_baudrate;
        uint32_t uart_baudrate;
    } baudrate_map[] = {
        {0x0000701d, 115200},
        {0x0252C00A, 230400},
        {0x05F75004, 921600},
        {0x00005004, 1000000},
        {0x04928002, 1500000},
        {0x00005002, 2000000},
        {0x0000B001, 2500000},
        {0x04928001, 3000000},
        {0x052A6001, 3500000},
        {0x00005001, 4000000},
    };

    const uint32_t baudrate_map_len = sizeof(baudrate_map)/sizeof(baudrate_map[0]);

    if (bt_to_uart)
    {
        for (i = 0; i < baudrate_map_len; i++)
        {
            if (*bt_baudrate == baudrate_map[i].bt_baudrate)
                break;
        }

        if (i == baudrate_map_len) {
            HCI_ERR("Wrong Baudrate Selection! Use Default 115200!");
            i = 0;
        }
        *uart_baudrate = baudrate_map[i].uart_baudrate;
    }
    else
    {
        for (i = 0; i < baudrate_map_len; i++)
        {
            if (*uart_baudrate == baudrate_map[i].uart_baudrate)
                break;
        }

        if (i == baudrate_map_len) {
            HCI_ERR("Wrong Baudrate Selection! Use Default 115200!");
            i = 0;
        }
        *bt_baudrate = baudrate_map[i].bt_baudrate;
    }
}

/******************************************************************************
 * Confifg Parse About
 */
static uint8_t hci_platform_parse_config(void)
{
    uint8_t *p, i;
    uint16_t entry_offset, entry_len;
    uint16_t tx_flatk;

    if (sizeof(hci_init_config) <= HCI_CONFIG_HDR_LEN)
        return HCI_IGNORE;

    p = hci_init_config;
    if (HCI_CONFIG_SIGNATURE != *(uint32_t*)(p))
        return HCI_FAIL;

    if (*(uint16_t *)(p + 4) != (uint16_t)(sizeof(hci_init_config) - HCI_CONFIG_HDR_LEN))
    {
        /* Fix the len, just avoid the length is not corect */
        *(uint16_t *)(p + 4) = (uint16_t)(sizeof(hci_init_config) - HCI_CONFIG_HDR_LEN);
    }

    p += HCI_CONFIG_HDR_LEN;
    while (p < hci_init_config + sizeof(hci_init_config))
    {
        entry_offset = *(uint16_t*)(p);
        entry_len = *(uint8_t*)(p + 2);
        p += 3;

        switch (entry_offset)
        {
        case 0x000c:
            /* MP Mode, Use Default: 115200 */
            if ((wifi_driver_is_mp()) || (!CHECK_SW(EFUSE_SW_UPPERSTACK_SWITCH)))
                hci_platform_convert_baudrate((uint32_t *)p, &hci_cfg_init_uart_baudrate, 0);

            hci_platform_convert_baudrate((uint32_t *)p, &hci_cfg_work_uart_baudrate, 1);
            hci_platform_convert_baudrate((uint32_t *)hci_cfg_work_bt_baudrate, &hci_cfg_work_uart_baudrate, 0);
            /* TODO: Config BaudRate */
            break;
        case 0x0018:
            /* MP Mode, Close Flow Control */
            if ((wifi_driver_is_mp()) || (!CHECK_SW(EFUSE_SW_UPPERSTACK_SWITCH)))
                p[0] = p[0] & (~BIT2);
            /* TODO: Config Flow Control */
            break;
        case 0x0030:
            /* Set ConfigBuf MacAddr, Use Customer Assign or Efuse */
            if (hci_cfg_flag & HCI_CFG_BD_ADDR)
            {
                for (i = 0; i < HCI_MAC_ADDR_LEN; i++)
                    p[i] = hci_cfg_bd_addr[i];
            }
            else
            {
                if ((hci_lgc_efuse[0] != 0xff) && (hci_lgc_efuse[1] != 0xff))
                {
                    for (i = 0; i < HCI_MAC_ADDR_LEN; i++)
                        p[i] = hci_lgc_efuse[HCI_MAC_ADDR_LEN - 1 - i];
                }
            }
            HCI_PRINT("Bluetooth init BT_ADDR in cfgbuf [%02x:%02x:%02x:%02x:%02x:%02x]\n\r",
                      p[5], p[4], p[3], p[2], p[1], p[0]);
            break;
        case 0x194:
            if (hci_lgc_efuse[LEFUSE(0x196)] == 0xff)
            {
                if (!(hci_phy_efuse[2] & BIT0))
                {
                    tx_flatk = hci_phy_efuse[0x0a] | hci_phy_efuse[0x0b] << 8;
                    bt_flatk_8710c(tx_flatk);
                }
                break;
            }
            else
            {
                p[0] = hci_lgc_efuse[LEFUSE(0x196)];
                if (hci_lgc_efuse[LEFUSE(0x196)] & BIT1)
                    p[1] = hci_lgc_efuse[LEFUSE(0x197)];

                if (hci_lgc_efuse[LEFUSE(0x196)] & BIT2)
                {
                    p[2] = hci_lgc_efuse[LEFUSE(0x198)];
                    p[3] = hci_lgc_efuse[LEFUSE(0x199)];
                    tx_flatk = hci_lgc_efuse[LEFUSE(0x198)] | hci_lgc_efuse[LEFUSE(0x199)] << 8;
                    bt_flatk_8710c(tx_flatk);
                }
                else
                {
                    if (!(hci_phy_efuse[2] & BIT0))
                    {
                        tx_flatk = hci_phy_efuse[0xa] | hci_phy_efuse[0xb] << 8;
                        bt_flatk_8710c(tx_flatk);
                    }
                }

                if (hci_lgc_efuse[LEFUSE(0x196)] & BIT5)
                {
                    p[4] = hci_lgc_efuse[LEFUSE(0x19a)];
                    p[5] = hci_lgc_efuse[LEFUSE(0x19b)];
                }
            }
            break;
        case 0x19f:
            for (i = 0; i < entry_len; i++)
            {
                if (hci_lgc_efuse[LEFUSE(0x19c + i)] != 0xff)
                    p[i] = hci_lgc_efuse[LEFUSE(0x19c + i)];
            }
            break;
        case 0x1A4:
            for (i = 0; i < entry_len; i++)
            {
                if (hci_lgc_efuse[LEFUSE(0x1a2 + i)] != 0xff)
                    p[i] = hci_lgc_efuse[LEFUSE(0x1A2 + i)];
            }
            break;
        default:
            break;
        }

        p +=  entry_len;
    }

    return HCI_SUCCESS;
}

/******************************************************************************
 * IQK About
 */
static void bt_dump_iqk(BT_Cali_TypeDef *iqk_data)
{
    if (!CHECK_SW(EFUSE_SW_DRIVER_DEBUG_LOG)) {
        HCI_PRINT("Dump iqk_data: \n\r");
        HCI_PRINT("The IQK_xx data is 0x%x\n\r", (unsigned int)iqk_data->IQK_xx);
        HCI_PRINT("The IQK_yy data is 0x%x\n\r", (unsigned int)iqk_data->IQK_yy);
        HCI_PRINT("The QDAC   data is 0x%x\n\r", iqk_data->QDAC);
        HCI_PRINT("The IDAC   data is 0x%x\n\r", iqk_data->IDAC);
        HCI_PRINT("The QDAC2  data is 0x%x\n\r", iqk_data->QDAC2);
        HCI_PRINT("The IDAC2  data is 0x%x\n\r", iqk_data->IDAC2);
    }
}

static uint8_t bt_iqk_efuse_valid(BT_Cali_TypeDef* bt_iqk_data)
{
	if ((hci_phy_efuse[3] == 0xff) && (hci_phy_efuse[4] == 0xff) &&
		(hci_phy_efuse[5] == 0xff) && (hci_phy_efuse[6] == 0xff)) {
		HCI_WARN("%s: no data", __FUNCTION__);
		return HCI_FAIL;
	} else {
		bt_iqk_data->IQK_xx = hci_phy_efuse[3] | hci_phy_efuse[4] << 8;
		bt_iqk_data->IQK_yy = hci_phy_efuse[5] | hci_phy_efuse[6] << 8;
		bt_iqk_data->QDAC   = hci_phy_efuse[0x0c];
		bt_iqk_data->IDAC   = hci_phy_efuse[0x0d];
		bt_iqk_data->QDAC2  = hci_phy_efuse[0x0e];
		bt_iqk_data->IDAC2  = hci_phy_efuse[0x0f];
		HCI_DBG("%s: has data", __FUNCTION__);
		return HCI_SUCCESS;
	}
}

static uint8_t bt_iqk_logic_efuse_valid(BT_Cali_TypeDef* bt_iqk_data)
{
	if ((hci_lgc_efuse[0x16] == 0xff) && (hci_lgc_efuse[0x17] == 0xff) &&
		(hci_lgc_efuse[0x18] == 0xff) && (hci_lgc_efuse[0x19] == 0xff)) {
		HCI_WARN("%s: no data", __FUNCTION__);
		return HCI_FAIL;
	} else {
		bt_iqk_data->IQK_xx = (uint32_t)(((uint32_t)hci_lgc_efuse[0x17]) << 8) | hci_lgc_efuse[0x16];
		bt_iqk_data->IQK_yy = (uint32_t)(((uint32_t)hci_lgc_efuse[0x19]) << 8) | hci_lgc_efuse[0x18];
		bt_iqk_data->QDAC   = hci_lgc_efuse[0x1a];
		bt_iqk_data->IDAC   = hci_lgc_efuse[0x1b];
		bt_iqk_data->QDAC2  = hci_lgc_efuse[0x1c];
		bt_iqk_data->IDAC2  = hci_lgc_efuse[0x1d];
		HCI_DBG("%s: has data", __FUNCTION__);
		return HCI_SUCCESS;
	}
}

uint8_t hci_platform_check_iqk(void)
{
	BT_Cali_TypeDef bt_iqk_data;

	if (!hci_lgc_efuse[LEFUSE(0x1a1)] & BIT0) {
		HCI_DBG("%s: USE FIX LOGIC EFUSE", __FUNCTION__);

        if (HCI_FAIL == bt_iqk_logic_efuse_valid(&bt_iqk_data)) {
            HCI_WARN("%s: LOGIC EFUSE HAS NO DATA", __FUNCTION__);
            return HCI_FAIL;
        }

        bt_dump_iqk(&bt_iqk_data);
        bt_lok_write(bt_iqk_data.IDAC, bt_iqk_data.QDAC, bt_iqk_data.IDAC2, bt_iqk_data.QDAC2);
        hci_phy_efuse[0] = 0;
        hci_phy_efuse[1] = hci_phy_efuse[1] & (~BIT0);
        //hci_phy_efuse[2] = 0xff;
        hci_phy_efuse[3] = bt_iqk_data.IQK_xx & 0xff;
        hci_phy_efuse[4] = (bt_iqk_data.IQK_xx >> 8) & 0xff;
        hci_phy_efuse[5] = bt_iqk_data.IQK_yy & 0xff;
        hci_phy_efuse[6] = (bt_iqk_data.IQK_yy >> 8) & 0xff;
        return HCI_SUCCESS;
	}

	if (HCI_SUCCESS == bt_iqk_efuse_valid(&bt_iqk_data)) {
		if (hci_phy_efuse[0] != 0)
			bt_dck_write(hci_phy_efuse[0x0e], hci_phy_efuse[0x0f]);
		else
			HCI_DBG("hci_tp_phy_efuse[0]=0");

		bt_dump_iqk(&bt_iqk_data);
		bt_lok_write(bt_iqk_data.IDAC, bt_iqk_data.QDAC, bt_iqk_data.IDAC2, bt_iqk_data.QDAC2);
		return HCI_SUCCESS;
	} else if (HCI_SUCCESS == bt_iqk_logic_efuse_valid(&bt_iqk_data)) {
		bt_dump_iqk(&bt_iqk_data);
		bt_lok_write(bt_iqk_data.IDAC, bt_iqk_data.QDAC, bt_iqk_data.IDAC2, bt_iqk_data.QDAC2);
		hci_phy_efuse[0] = 0;
		hci_phy_efuse[1] = hci_phy_efuse[1] & (~BIT0);
		//hci_phy_efuse[2] = 0xff;
		hci_phy_efuse[3] = bt_iqk_data.IQK_xx & 0xff;
		hci_phy_efuse[4] = (bt_iqk_data.IQK_xx >> 8) & 0xff;
		hci_phy_efuse[5] = bt_iqk_data.IQK_yy & 0xff;
		hci_phy_efuse[6] = (bt_iqk_data.IQK_yy >> 8) & 0xff;
		hci_phy_efuse[0x0e] = hci_lgc_efuse[0x1E];
		hci_phy_efuse[0x0f] = hci_lgc_efuse[0x1f];
		bt_dck_write(hci_phy_efuse[0x0e], hci_phy_efuse[0x0f]);
		return HCI_SUCCESS;
	} else {
		HCI_WARN("%s: NO IQK LOK DATA need start LOK", __FUNCTION__);
		return HCI_FAIL;
	}
}

uint8_t hci_platform_start_iqk(void)
{
	uint32_t ret = 0;

	ret = bt_iqk_8710c(&iqk_data, 0);
	if (_FAIL == ret) {
		HCI_ERR("%s: Warning: IQK Fail, please connect driver!", __FUNCTION__);
		return HCI_FAIL;
	}

	bt_dump_iqk(&iqk_data);
	bt_lok_write(iqk_data.IDAC, iqk_data.QDAC, iqk_data.IDAC2, iqk_data.QDAC2);

	hci_phy_efuse[0] = 0;
	hci_phy_efuse[1] = hci_phy_efuse[1] & (~BIT0);
	//hci_phy_efuse[2] = 0xff;
	hci_phy_efuse[3] = iqk_data.IQK_xx & 0xff;
	hci_phy_efuse[4] = (iqk_data.IQK_xx >> 8) & 0xff;
	hci_phy_efuse[5] = iqk_data.IQK_yy & 0xff;
	hci_phy_efuse[6] = (iqk_data.IQK_yy >> 8) & 0xff;

	return HCI_SUCCESS;
}

int hci_platform_get_iqk_data(uint8_t *data, uint8_t len)
{
  memcpy(data, hci_phy_efuse, len);
  return HCI_SUCCESS;
}

/******************************************************************************
 * ROM Version About
 */
void hci_platform_record_chipid(uint8_t chipid)
{
    hci_chipid_in_fw = chipid;

    /* Just Config parse here, left find patch later */
    hci_platform_parse_config();
}

/******************************************************************************
 * Baudrate About
 */
void hci_platform_get_baudrate(uint8_t* baudrate, uint8_t len)
{
    /* memcpy */
    for (uint8_t i = 0; i < len; i++)
        baudrate[i] = hci_cfg_work_bt_baudrate[i];
}

uint8_t hci_platform_set_baudrate(void)
{
    hci_uart_set_bdrate(hci_cfg_work_uart_baudrate);
    osif_delay(10);

    return HCI_SUCCESS;
}

/******************************************************************************
 * Download Patch About
 */
uint8_t hci_platform_dl_patch_init(void)
{
    hci_patch_info = osif_mem_alloc(0, sizeof(HCI_PATCH_INFO));
    if (!hci_patch_info)
        return HCI_FAIL;

    memset(hci_patch_info, 0, sizeof(HCI_PATCH_INFO));

    return HCI_SUCCESS;
}

void hci_platform_dl_patch_done(void)
{
    if(hci_patch_info->fw_is_alloced && hci_patch_info->fw_buf)
        osif_mem_free(hci_patch_info->fw_buf);
    hci_patch_info->fw_buf = NULL;

    if(hci_patch_info)
        osif_mem_free(hci_patch_info);
    hci_patch_info = NULL;
}

static uint8_t hci_platform_get_patch_info(void)
{
    const uint8_t no_patch_sig[] = {0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t single_patch_sig[] = {0xFD, 0x63, 0x05, 0x62};
    const uint8_t merged_patch_sig[] = {0x52, 0x65, 0x61, 0x6C, 0x74, 0x65, 0x63, 0x68};
    HCI_PATCH_INFO *patch_info = hci_patch_info;
    uint16_t lmp_subversion, num_of_patch, fw_chip_id, fw_len, i;
    uint32_t fw_offset;
    flash_t flash;

    if (!CHECK_SW(EFUSE_SW_USE_FLASH_PATCH))
    {
        /* Check flash img */
        uint8_t flash_patch_head[8];
        flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS, 8, flash_patch_head);
        if (!memcmp(flash_patch_head, merged_patch_sig, sizeof(merged_patch_sig)))
        {
            /* use the changed patch */
            flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + 8, 4, (uint8_t *)&lmp_subversion);
            flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + 12, 2, (uint8_t *)&num_of_patch);
            for (i = 0; i < num_of_patch; i++)
            {
                flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + 0x0e + 2 * i, 2, (uint8_t *)&fw_chip_id);
                if (fw_chip_id == hci_chipid_in_fw)
                {
                    flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + 0x0e + 2 * num_of_patch + 2 * i, 2, (uint8_t *)&fw_len);
                    flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + 0x0e + 4 * num_of_patch + 4 * i, 4, (uint8_t *)&fw_offset);
                    break;
                }
            }
            if (i >= num_of_patch)
            {
                HCI_ERR("Use flash patch but no match patch");
                return HCI_FAIL;
            }

            patch_info->fw_buf = (uint8_t *)osif_mem_alloc(0, fw_len);
            if (!patch_info->fw_buf)
                return HCI_FAIL;
            patch_info->fw_is_alloced = 1;

            flash_stream_read(&flash, HCI_MERGE_PATCH_ADDRESS + fw_offset, fw_len, patch_info->fw_buf);
            LE_UINT32_TO_ARRAY(patch_info->fw_buf + fw_len - 4, lmp_subversion);
            goto CONFIG_PATCH_INFO;
        }
    }

    patch_info->fw_buf = (uint8_t *)rltk_bt_get_patch_code();

    if (!memcmp(patch_info->fw_buf, single_patch_sig, sizeof(single_patch_sig)))
    {
        HCI_DBG("Use single patch");
        patch_info->fw_len = rltk_bt_get_patch_code_len();
        patch_info->fw_is_alloced = 0;
    }
    else if (!memcmp(patch_info->fw_buf, merged_patch_sig, sizeof(merged_patch_sig)))
    {
        HCI_DBG("Use merged patch");
        LE_ARRAY_TO_UINT32(lmp_subversion, patch_info->fw_buf + 8);
        LE_ARRAY_TO_UINT16(num_of_patch, patch_info->fw_buf + 12);

        for (i = 0; i < num_of_patch; i++)
        {
            LE_ARRAY_TO_UINT16(fw_chip_id, patch_info->fw_buf + 0x0e + 2 * i);
            if (fw_chip_id == hci_chipid_in_fw)
            {
                LE_ARRAY_TO_UINT16(fw_len, patch_info->fw_buf + 0x0e + 2 * num_of_patch + 2 * i);
                LE_ARRAY_TO_UINT32(fw_offset, patch_info->fw_buf + 0x0e + 4 * num_of_patch + 4 * i);
                break;
            }
        }

        if (i >= num_of_patch)
        {
            HCI_ERR("Use normal patch but no match patch");
            return HCI_FAIL;
        }

        patch_info->fw_buf = patch_info->fw_buf + fw_offset;
        patch_info->fw_len = fw_len;
        patch_info->fw_is_alloced = 0;
    }
    else if (!memcmp(patch_info->fw_buf, no_patch_sig, sizeof(no_patch_sig)))
    {
        HCI_WARN("NO patch!");
        return HCI_IGNORE;
    }
    else
    {
        HCI_ERR("Something Wrong when finding patch");
        return HCI_FAIL;
    }

CONFIG_PATCH_INFO:
    /* Set config info */
    patch_info->config_buf = hci_init_config;
    patch_info->config_len = sizeof(hci_init_config);

    /* Calculate patch info */
    patch_info->end_index = (patch_info->fw_len + patch_info->config_len - 1) / HCI_PATCH_FRAG_SIZE;
    patch_info->last_pkt = (patch_info->fw_len + patch_info->config_len) % HCI_PATCH_FRAG_SIZE;
    if (patch_info->last_pkt == 0)
        patch_info->last_pkt = HCI_PATCH_FRAG_SIZE;

    return HCI_SUCCESS;
}

uint8_t hci_platform_get_patch_cmd_len(uint8_t *cmd_len)
{
	uint8_t ret;
    HCI_PATCH_INFO *patch_info = hci_patch_info;

    /* Download FW partial patch first time, get patch and info */
    if (0 == patch_info->cur_index)
    {
        ret = hci_platform_get_patch_info();
        if (HCI_SUCCESS != ret)
            return ret;
    }

    if (patch_info->cur_index == patch_info->end_index)
    {
        *cmd_len = patch_info->last_pkt + 1;
        return HCI_SUCCESS;
    }

    *cmd_len = HCI_PATCH_FRAG_SIZE + 1;

    return HCI_SUCCESS;
}

uint8_t hci_platform_get_patch_cmd_buf(uint8_t *cmd_buf, uint8_t cmd_len)
{
    HCI_PATCH_INFO* patch_info = hci_patch_info;
    uint8_t*        data_buf   = &cmd_buf[1];
    uint8_t         data_len   = cmd_len - 1;

    /* first byte is index */
    cmd_buf[0] = patch_info->cur_index % 0x80;
    if (patch_info->cur_index == patch_info->end_index)
        cmd_buf[0] |= 0x80;

    if (patch_info->sent_len + data_len <= patch_info->fw_len)
    {
        /* within fw patch domain */
        memcpy(data_buf, patch_info->fw_buf + patch_info->sent_len, data_len);
    }
    else if ((patch_info->sent_len < patch_info->fw_len) && (patch_info->sent_len + data_len > patch_info->fw_len))
    {
        /* need copy fw patch domain and config domain */
        memcpy(data_buf, patch_info->fw_buf + patch_info->sent_len, patch_info->fw_len - patch_info->sent_len);

        memcpy(data_buf + (patch_info->fw_len - patch_info->sent_len), patch_info->config_buf,
               data_len - (patch_info->fw_len - patch_info->sent_len));
    }
    else
    {
        memcpy(data_buf, patch_info->config_buf + (patch_info->sent_len - patch_info->fw_len), data_len);
    }

    patch_info->sent_len += data_len;
    patch_info->cur_index++;

    return HCI_SUCCESS;
}

/******************************************************************************
 * Init and Deinit About
 */
static inline void bt_power_on(void)
{
	set_reg_value(0x40000214, BIT24 | BIT25, 3);
    osif_delay(50);
}

static inline void bt_power_off(void)
{
	set_reg_value(0x40000214, BIT24 | BIT25, 0);
    osif_delay(50);
}

static void bt_reset(void)
{
    /* PowerSaving */
	if (!wifi_driver_is_mp())
		wifi_set_powersave_mode(IPS_MODE_NONE, LPS_MODE_NONE);

	HCI_DBG("BT Resetting...\n");

	set_reg_value(0x40000244, BIT9 | BIT8, 3);
	osif_delay(5);

    /* FW Log open if SW is Set */
	if (!CHECK_SW(EFUSE_SW_BT_FW_LOG)) {
        HCI_PRINT("BT FW Log Open\n\r");
		set_reg_value(0x400000cc, BIT2 | BIT1 | BIT0, 6);
		osif_delay(5);
		set_reg_value(0x400000cc, BIT8, 1);
		osif_delay(5);
	}
	//bt_only_enable_func();

    /* Power off and on */
    bt_power_off();
    bt_power_on();

	HCI_DBG("BT Reset OK!");	
}

static void hci_normal_start(void)
{
	if (wifi_driver_is_mp())
		rtlk_bt_set_gnt_bt(PTA_BT);
	else
		rltk_coex_bt_enable(1);
}

uint8_t hci_platform_init(void)
{
	if (!(wifi_is_running(WLAN0_IDX) || wifi_is_running(WLAN1_IDX))) {
		HCI_ERR("WiFi is OFF! Please Restart BT after Wifi on!");
		return HCI_FAIL;
	}

    READ_SW(hci_sw_val);

	if (wifi_driver_is_mp())
		HCI_DBG("This is BT MP Driver, AmebaZ2 %x Cut", HCI_CHIP_VER);
    else
        HCI_DBG("This is BT Normal Driver, AmebaZ2 %x Cut", HCI_CHIP_VER);

	HCI_DBG("BT BUILD Date: %s, %s", __DATE__, __TIME__);
	HCI_DBG("We use Debug Val: 0x%x", HAL_READ32(SPI_FLASH_BASE, FLASH_BT_PARA_ADDR));

	//rtlk_bt_set_gnt_bt(PTA_BT);

    /* Read efuse */
    if (HCI_FAIL == hci_platform_read_efuse())
        return HCI_FAIL;

    /* BT Reset */
    bt_reset();

    /* BT Coex */
    hci_normal_start();

    /* BT UART Open */
    if (HCI_FAIL == hci_uart_open())
        return HCI_FAIL;

	return HCI_SUCCESS;
}

uint8_t hci_platform_deinit(void)
{
    /* BT UART Close */
    hci_uart_close();

    /* Power off */
    bt_power_off();

    /* BT Coex */
	rltk_coex_bt_enable(0);

    /* PowerSaving */
	if (!wifi_driver_is_mp())
		wifi_set_powersave_mode(IPS_MODE_RESUME, LPS_MODE_RESUME);

    return HCI_SUCCESS;
}

uint8_t hci_platform_init_done(void)
{
	if (wifi_driver_is_mp()) {
		HCI_DBG("EFUSE_SW_MP_MODE: UPPERSTACK NOT UP! GNT_BT (%x)", (unsigned int)HAL_READ32(0x40080000, 0x0764));
		//mp_driver_init_done = 1;
		return HCI_FAIL;
	}

	HCI_DBG("Start upperStack.");
	return HCI_SUCCESS;
}
