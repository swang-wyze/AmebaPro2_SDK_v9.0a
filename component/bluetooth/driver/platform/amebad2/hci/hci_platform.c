/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#include "osif.h"
#include "hci_config.h"
#include "hci/hci_common.h"
#include "hci_uart.h"
#include "hci_platform.h"
#include "hci_dbg.h"

#define HCI_LGC_EFUSE_LEN          0x20
#define HCI_PHY_EFUSE_LEN          0x13
#define HCI_PHY_EFUSE_BASE         0x100
#define HCI_LGC_EFUSE_OFFSET       0x190
#define HCI_MAC_ADDR_LEN           6
#define HCI_CONFIG_SIGNATURE       0x8730ab55
#define HCI_CONFIG_HDR_LEN         6
#define HCI_PATCH_FRAG_SIZE        252
#define HCI_PATCH_ADDRESS_OTA1     0x080F8000
#define HCI_PATCH_ADDRESS_OTA2     0x081F8000

#define HCI_CFG_BAUDRATE           BIT0
#define HCI_CFG_FLOWCONTROL        BIT1
#define HCI_CFG_BD_ADDR            BIT2
#define LEFUSE(x)                  ((x)-HCI_LGC_EFUSE_OFFSET)

static const uint8_t hci_patch_buf[] = {0xff, 0xff, 0xff, 0xff};
static uint32_t hci_patch_buf_len    = sizeof(hci_patch_buf);
#if 0
static uint8_t hci_phy_efuse[HCI_PHY_EFUSE_LEN]  = {0};
static uint8_t hci_lgc_efuse[HCI_LGC_EFUSE_LEN]  = {0};
#endif
static uint8_t hci_chipid_in_fw  = 0;
static uint8_t hci_init_config[] = {
    0, /*LE trx on delay*/
};
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
static HCI_PATCH_INFO *hci_patch_info = NULL;

static uint8_t  hci_cfg_bd_addr[HCI_MAC_ADDR_LEN] = {0};
static uint32_t hci_cfg_init_uart_baudrate        = 115200;
static uint32_t hci_cfg_work_uart_baudrate        = 1500000;
static uint8_t  hci_cfg_work_bt_baudrate[4]       = {0x02, 0x80, 0x92, 0x04};
static uint32_t hci_cfg_flag                      = 0;

extern void bt_power_on(void);
extern void bt_power_off(void);
extern void rltk_coex_bt_enable(u8 enable);

static uint32_t cal_bit_shift(uint32_t Mask)
{
    uint32_t i;
    for (i = 0; i < 31; i++) {
            if (((Mask >> i) & 0x1) == 1) {
                    break;
            }
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

void hci_platform_cfg_bd_addr(uint8_t *bdaddr)
{
    for (uint8_t i = 0; i < HCI_MAC_ADDR_LEN; i++)
        hci_cfg_bd_addr[i] = bdaddr[i];

    hci_cfg_flag |= HCI_CFG_BD_ADDR;
}

static void hci_platform_convert_baudrate(uint32_t *bt_baudrate, uint32_t *uart_baudrate, uint8_t bt_to_uart)
{
    uint8_t i;

    const struct {
        uint32_t bt_baudrate;
        uint32_t uart_baudrate;
    } baudrate_map[] = {
        {0x0000701d, 115200},
        {0x0252C00A, 230400},
        {0x03F75004, 921600},
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

#if 0
static uint8_t hci_platform_read_efuse(void)
{
    uint8_t i, *pbuf;

    /* Read Logic Efuse */
    pbuf = osif_mem_alloc(1024);
    if (!pbuf || _FAIL == EFUSE_LMAP_READ(pbuf))
    {
        HCI_ERR("EFUSE_LMAP_READ failed");
        if (pbuf)
            osif_mem_free(pbuf);
        return HCI_FAIL;
    }

    memcpy(hci_lgc_efuse, pbuf + HCI_LGC_EFUSE_OFFSET, HCI_LGC_EFUSE_LEN);

    /* Read Physical Efuse */
    for (i = 0; i < HCI_PHY_EFUSE_LEN; i++)
        EFUSE_PMAP_READ8(0, 0x120 + i, hci_phy_efuse + i, L25EOUTVOLTAGE);

    HCI_DBG("\n\rRead Logic Efuse: \n\r");
    for (i = 0; i < HCI_LGC_EFUSE_LEN; i++)
        HCI_DBG("%x:", hci_lgc_efuse[i]);
    HCI_DBG("\n\rRead Phy Efuse: \n\r");
    for (i = 0; i < HCI_PHY_EFUSE_LEN; i++)
        HCI_DBG("%x:", hci_phy_efuse[i]);
    HCI_DBG("\n\r");

    if (pbuf)
        osif_mem_free(pbuf);

    return HCI_SUCCESS;
}
#endif

static uint8_t hci_platform_parse_config(void)
{
    uint8_t *p, i;
    uint16_t entry_offset, entry_len;
    //uint16_t tx_flatk;

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
            if ((wifi_driver_is_mp()) || (!CHECK_CFG_SW(EFUSE_SW_UPPERSTACK_SWITCH)))
                hci_platform_convert_baudrate((uint32_t *)p, &hci_cfg_init_uart_baudrate, 0);

            hci_platform_convert_baudrate((uint32_t *)p, &hci_cfg_work_uart_baudrate, 1);
            hci_platform_convert_baudrate((uint32_t *)hci_cfg_work_bt_baudrate, &hci_cfg_work_uart_baudrate, 0);
            /* TODO: Config BaudRate */
            break;
        case 0x0018:
            /* MP Mode, Close Flow Control */
            if ((wifi_driver_is_mp()) || (!CHECK_CFG_SW(EFUSE_SW_UPPERSTACK_SWITCH)))
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
#if 0
            else
            {
                if ((hci_lgc_efuse[0] != 0xff) && (hci_lgc_efuse[1] != 0xff))
                {
                    for (i = 0; i < HCI_MAC_ADDR_LEN; i++)
                        p[i] = hci_lgc_efuse[HCI_MAC_ADDR_LEN - 1 - i];
                }
            }
#endif
            break;
#if 0
        case 0x194:

            if (hci_lgc_efuse[LEFUSE(0x196)] == 0xff)
            {
                if (!(hci_phy_efuse[2] & BIT0))
                {
                    tx_flatk = hci_phy_efuse[0x0a] | hci_phy_efuse[0x0b] << 8;
                    bt_flatk_8730d(tx_flatk);
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
                    bt_flatk_8730d(tx_flatk);
                }
                else
                {
                    if (!(hci_phy_efuse[2] & BIT0))
                    {
                        tx_flatk = hci_phy_efuse[0xa] | hci_phy_efuse[0xb] << 8;
                        bt_flatk_8730d(tx_flatk);
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
#endif
        default:
            break;
        }

        p +=  entry_len;
    }

    return HCI_SUCCESS;
}

static void hci_borad_controller_reset(void)
{
#if 0
    /* BT Controller Power */
    //bt_power_on();
    osif_delay(5);

    /* Isolation */
    set_reg_value(0x40000000, BIT16, 0);
    osif_delay(5);

    /* BT Function Enable */
    set_reg_value(0x40000204, BIT24, 0);
    osif_delay(5);
    set_reg_value(0x40000204, BIT24, 1);
    osif_delay(50);

    /* BT Clock Enable */
    set_reg_value(0x40000214, BIT24, 1);
    osif_delay(5);

    HCI_DBG("\n\rBT Reset OK!");
#endif
}

uint8_t hci_platform_init(void)
{
#if 0
    /* Efuse, MP, Trace_Setting */
    if (!(wifi_is_up(RTW_STA_INTERFACE) || wifi_is_up(RTW_AP_INTERFACE)))
    {
        HCI_ERR("Wifi is OFF! Restart Wifi First!");
        return HCI_FAIL;
    }

    HCI_DBG("\n\rThis is AmebaD %x CUT", SYSCFG_CUTVersion() + 10);

    is_mp_driver = wifi_driver_is_mp();
    if (is_mp_driver)
    {
        HCI_DBG("\r\nThis is BT MP Driver");
    }
    else
    {
        HCI_DBG("\r\nThis is BT Normal Driver");
        wifi_set_powersave_mode(IPS_MODE_NONE, LPS_MODE_NONE);
    }
#endif
    if (!CHECK_CFG_SW(EFUSE_SW_BT_FW_LOG))
    {
        /* if want to close, Use Cmd: EFUSE wmap 1a1 1 fe */
        set_reg_value(0x48000440, BIT0 | BIT1 | BIT2 | BIT3 | BIT4, 17);
    }

    /* Read Efuse and Parse Configbuf */
#if 0
    if (HCI_FAIL == hci_platform_read_efuse())
        return HCI_FAIL;
#endif
    if (HCI_FAIL == hci_platform_parse_config())
        return HCI_FAIL;

    /* BT Controller Reset */
    hci_borad_controller_reset();

    /* UART Init */
    if (HCI_FAIL == hci_uart_open())
        return HCI_FAIL;
    /* Coex: TODO */
    //uint8_t is_mp_driver = 0;
    //if (!is_mp_driver)
    //   rltk_coex_bt_enable(1);

    return HCI_SUCCESS;
}

uint8_t hci_platform_deinit(void)
{
    /* BT Controller Power Off */
    //bt_power_off();

    /* UART Deinit */
    hci_uart_close();

    /* Coex */
    //if (!wifi_driver_is_mp())
    //    rltk_coex_bt_enable(0);

    return HCI_SUCCESS;
}

void hci_platform_record_chipid(uint8_t chipid)
{
    hci_chipid_in_fw = chipid;
}

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

static inline void hci_platform_flash_stream_read(uint8_t *address, uint32_t len, uint8_t *data)
{
	memcpy(data, (const void *)address, len);
}

static uint8_t hci_platform_get_patch_info(void)
{
    const uint8_t   no_patch_sig[]     = {0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t   merged_patch_sig[] = {0x52, 0x65, 0x61, 0x6C, 0x74, 0x65, 0x63, 0x68};
    HCI_PATCH_INFO* patch_info         = hci_patch_info;
    uint16_t        num_of_patch, fw_chip_id, fw_len, i;
    uint32_t        fw_offset;

    if (CHECK_CFG_SW(EFUSE_SW_USE_FLASH_PATCH))
    {
        patch_info->fw_buf = (uint8_t*)(void*)hci_patch_buf;
        patch_info->fw_len = hci_patch_buf_len;
    }
    else
    {
        //if (ota_get_cur_index() == OTA_INDEX_1)
        //    patch_info->fw_buf = (uint8_t *)HCI_PATCH_ADDRESS_OTA2;
        //else
            patch_info->fw_buf = (uint8_t *)HCI_PATCH_ADDRESS_OTA1;
    }

    if (!memcmp(patch_info->fw_buf, no_patch_sig, sizeof(no_patch_sig)))
        return HCI_IGNORE;

    if (!memcmp(patch_info->fw_buf, merged_patch_sig, sizeof(no_patch_sig)))
    {
        /* Merged Patch */
        hci_platform_flash_stream_read(patch_info->fw_buf + 0x0c, 2, (uint8_t *)&num_of_patch);

        for (i = 0; i < num_of_patch; i++)
        {
            hci_platform_flash_stream_read(patch_info->fw_buf + 0x0e + 2 * i, 2, (uint8_t *)&fw_chip_id);
            if (fw_chip_id == hci_chipid_in_fw)
            {
                hci_platform_flash_stream_read(patch_info->fw_buf + 0x0e + 2 * num_of_patch + 2 * i, 2, (uint8_t *)&fw_len);
                hci_platform_flash_stream_read(patch_info->fw_buf + 0x0e + 4 * num_of_patch + 4 * i, 4, (uint8_t *)&fw_offset);
                break;
            }
        }

        if (i >= num_of_patch)
        {
            HCI_ERR("No Match Patch");
            return HCI_FAIL;
        }

        patch_info->fw_buf = patch_info->fw_buf + fw_offset;
        patch_info->fw_len = fw_len;
        patch_info->fw_is_alloced = 0;
    }
    else
    {
        /* Single Patch, Do Nothing */
        patch_info->fw_is_alloced = 0;
    }

    patch_info->config_buf = hci_init_config;
    patch_info->config_len = sizeof(hci_init_config);
    
    /* Calculate patch info */
    patch_info->end_index = (patch_info->fw_len + patch_info->config_len - 1) / HCI_PATCH_FRAG_SIZE;
    patch_info->last_pkt = (patch_info->fw_len + patch_info->config_len) % HCI_PATCH_FRAG_SIZE;
    if(patch_info->last_pkt == 0)
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
