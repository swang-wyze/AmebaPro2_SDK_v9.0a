/******************************************************************************
 * Copyright (c) 2013-2016 Realtek Semiconductor Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef AUTOCONF_8735B_H
#define AUTOCONF_8735B_H

#define CONFIG_RTL8735B
#undef RTL8735B_SUPPORT
#define RTL8735B_SUPPORT 1
/* Configure for bus */
#define CONFIG_AXI_HCI
/* PHY layer band config */
#define CONFIG_DFS
#define NOT_SUPPORT_40M
/* 0: 20 MHz, 1: 40 MHz, 2: 80 MHz, 3: 160MHz, 4: 80+80MHz
* 2.4G use bit 0 ~ 3, 5G use bit 4 ~ 7
* 0x21 means enable 2.4G 40MHz & 5G 80MHz */
#define  CONFIG_BW_MODE	0x00
#define SUPPORT_5G_CHANNEL
#define CONFIG_INIT_CHAN 1
/* PHY layer band config end */
/* For DM support */
#define RATE_ADAPTIVE_SUPPORT 1
#define TX_CHECK_DSEC_ALWAYS 1
/* For phydm configurations */
#define CONFIG_FW_C2H_PKT
#define PHYDM_LINUX_CODING_STYLE 1
#define PHYDM_NEW_INTERFACE 1
#undef DBG
#define DBG 1 /* for phydm debug */
/* config in concurrent mode */
#ifdef CONFIG_CONCURRENT_MODE
#define CONFIG_RUNTIME_PORT_SWITCH
#endif
//#define CONFIG_NO_FW
//#define CONFIG_EX_FW_BIN
/* Configurations for power saving */
//#define CONFIG_WOWLAN
//#define CONFIG_WOWLAN_SD1
//#define CONFIG_WOWLAN_HW_CAM
//#define CONFIG_WOWLAN_CUSTOM_PATTERN
#define FW_IQK

#define RTW_HALMAC		/* Use HALMAC architecture */
/* For efuse or flash config start */
#define CONFIG_EFUSE_SEPARATE

#define CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B 0 // 1: HAL+MAC LOOPBACK, 2: HAL+MAC+BB LOOPBACK 3: DRV+HAL+MAC LOOPBACK
#if defined(CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B) && (CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B == 3)
#define CONFIG_MAC_LOOPBACK_DRIVER_AMEBA
#endif

#if (defined(CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B) && ((CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B == 1) || (CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B == 2)))
//Enable mac loopback for test mode (Ameba)
#ifdef CONFIG_WIFI_NORMAL
#define CONFIG_TWO_MAC_DRIVER // for test mode
#else //CONFIG_WIFI_VERIFY
#define ENABLE_MAC_LB_FOR_TEST_MODE
#endif
#endif

#ifdef ENABLE_MAC_LB_FOR_TEST_MODE
#define CONFIG_SUDO_PHY_SETTING
#define INT_HANDLE_IN_ISR 1
#define CONFIG_LWIP_LAYER 0
#define CONFIG_WLAN_HAL_TEST
#define CONFIG_WLAN_HAL_RX_TASK
#define CONFIG_MAC_LOOPBACK_DRIVER_AMEBA 1
#define HAL_MAC_ENABLE 1
#if (defined(CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B) && (CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B == 2))
// Enable BB loopback test
#define HAL_BB_ENABLE 1
#define HAL_RF_ENABLE 1
#define DISABLE_BB_RF 0
#else
#define DISABLE_BB_RF 1
#endif
#else
//#define CONFIG_TWO_MAC_DRIVER //for mornal driver; two mac
#if defined(CONFIG_TWO_MAC_DRIVER) || defined(CONFIG_MAC_LOOPBACK_DRIVER_AMEBA)
#define CONFIG_SUDO_PHY_SETTING
#define HAL_MAC_ENABLE 1
#define DISABLE_BB_RF 1
#else
#define HAL_MAC_ENABLE 1
#define HAL_BB_ENABLE 1
#define HAL_RF_ENABLE 1
#define DISABLE_BB_RF 0
#endif
//#define INT_HANDLE_IN_ISR 1
#endif

#define CONFIG_REG_ENABLE_KFREE 0	// 0: Depend on efuse(flash), 1: enable, 2: disable

#endif  /*#ifndef AUTOCONF_8735B_H  */
