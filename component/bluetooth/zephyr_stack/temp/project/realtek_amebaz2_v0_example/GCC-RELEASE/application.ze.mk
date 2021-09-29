
# Initialize tool chain
# -------------------------------------------------------------------
AMEBAZ2_TOOLDIR	      = ../../../component/soc/8710c/misc/iar_utility
AMEBAZ2_GCCTOOLDIR	  = ../../../component/soc/8710c/misc/gcc_utility
AMEBAZ2_BSPDIR        = ../../../component/soc/8710c/misc/bsp
AMEBAZ2_BOOTLOADERDIR = $(AMEBAZ2_BSPDIR)/image
AMEBAZ2_ROMSYMDIR     = $(AMEBAZ2_BSPDIR)/ROM
DUMP_START_ADDRESS    = 0x98000000
DUMP_END_ADDRESS      = 0x98200000
CROSS_COMPILE         = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-

# Compilation tools
AR        = $(CROSS_COMPILE)ar
CC        = $(CROSS_COMPILE)gcc
AS        = $(CROSS_COMPILE)as
NM        = $(CROSS_COMPILE)nm
LD        = $(CROSS_COMPILE)gcc
GDB       = $(CROSS_COMPILE)gdb
OBJCOPY   = $(CROSS_COMPILE)objcopy
OBJDUMP   = $(CROSS_COMPILE)objdump

OS       := $(shell uname)
LDSCRIPT := ./rtl8710c_ram.ld


# Initialize target name and target object files
# -------------------------------------------------------------------
all: build_info application_ze manipulate_images
mp : build_info application_ze manipulate_images

TARGET       = application_ze
OBJ_DIR      = $(TARGET)/Debug/obj
BIN_DIR      = $(TARGET)/Debug/bin
INFO_DIR     = $(TARGET)/Debug/info
BOOT_BIN_DIR = bootloader/Debug/bin
ROMIMG       = 


# Decide if 64 bit time wrapper is to be included
# -------------------------------------------------------------------
#SYSTEM_TIME64_MAKE_OPTION = 1


# Include folder list
# -------------------------------------------------------------------
COMP_DIR = ../../../component
INCLUDES =
INCLUDES += \
	-I../inc \
	-I$(COMP_DIR)/at_cmd \
	-I$(COMP_DIR)/stdlib \
	-I$(COMP_DIR)/wifi/api \
	-I$(COMP_DIR)/wifi/wifi_config \
	-I$(COMP_DIR)/wifi/wifi_fast_connect \
	-I$(COMP_DIR)/wifi/wpa_supplicant/src \
	-I$(COMP_DIR)/wifi/wpa_supplicant/src/crypto \
	-I$(COMP_DIR)/application \
	-I$(COMP_DIR)/network/mqtt/MQTTClient \
	-I$(COMP_DIR)/example \
	-I$(COMP_DIR)/file_system \
	-I$(COMP_DIR)/file_system/dct \
	-I$(COMP_DIR)/file_system/fatfs \
	-I$(COMP_DIR)/file_system/fatfs/r0.10c/include \
	-I$(COMP_DIR)/file_system/ftl \
	-I$(COMP_DIR)/mbed/hal \
	-I$(COMP_DIR)/network/cJSON \
	-I$(COMP_DIR)/network/httplite \
	-I$(COMP_DIR)/network/xml \
	-I$(COMP_DIR)/mbed/hal_ext \
	-I$(COMP_DIR)/mbed/targets/hal/rtl8710c \
	-I$(COMP_DIR)/network \
	-I$(COMP_DIR)/network/coap/include \
	-I$(COMP_DIR)/network/libcoap/include \
	-I$(COMP_DIR)/network/http2/nghttp2-1.31.0/includes \
	-I$(COMP_DIR)/lwip/api \
	-I$(COMP_DIR)/lwip/lwip_v2.0.2/src/include \
	-I$(COMP_DIR)/lwip/lwip_v2.0.2/src/include/lwip \
	-I$(COMP_DIR)/lwip/lwip_v2.0.2/port/realtek \
	-I$(COMP_DIR)/lwip/lwip_v2.0.2/port/realtek/freertos \
	-I$(COMP_DIR)/ssl/mbedtls-2.4.0/include \
	-I$(COMP_DIR)/ssl/ssl_ram_map/rom \
	-I$(COMP_DIR)/wifi/driver/include \
	-I$(COMP_DIR)/wifi/driver/src/osdep \
	-I$(COMP_DIR)/wifi/driver/src/core/option \
	-I$(COMP_DIR)/bluetooth \
	-I$(COMP_DIR)/bluetooth/driver \
	-I$(COMP_DIR)/bluetooth/driver/hci \
	-I$(COMP_DIR)/bluetooth/driver/inc \
	-I$(COMP_DIR)/bluetooth/driver/inc/hci \
	-I$(COMP_DIR)/bluetooth/driver/platform/amebaz2/inc \
	-I$(COMP_DIR)/bluetooth/os/osif \
	-I$(COMP_DIR)/bluetooth/zephyr_stack/example \
	-I$(COMP_DIR)/bluetooth/zephyr_stack/platform/amebaz2 \
	-I$(COMP_DIR)/media/rtp_codec \
	-I$(COMP_DIR)/media/mmfv2 \
	-I$(COMP_DIR)/application/airsync/1.0.4 \
	-I$(COMP_DIR)/soc/8710c/cmsis/rtl8710c/include \
	-I$(COMP_DIR)/soc/8710c/cmsis/rtl8710c/lib/include \
	-I$(COMP_DIR)/soc/8710c/fwlib/include \
	-I$(COMP_DIR)/soc/8710c/fwlib/lib/include \
	-I$(COMP_DIR)/soc/8710c/cmsis/cmsis-core/include \
	-I$(COMP_DIR)/soc/8710c/app/rtl_printf/include \
	-I$(COMP_DIR)/soc/8710c/app/shell \
	-I$(COMP_DIR)/soc/8710c/app/stdio_port \
	-I$(COMP_DIR)/soc/8710c/misc/utilities/include \
	-I$(COMP_DIR)/soc/8710c/mbed-drivers/include \
	-I$(COMP_DIR)/soc/8710c/misc/platform \
	-I$(COMP_DIR)/soc/8710c/misc/driver \
	-I$(COMP_DIR)/soc/8710c/misc/os \
	-I$(COMP_DIR)/os/freertos \
	-I$(COMP_DIR)/os/freertos/freertos_v10.0.1/Source/include \
	-I$(COMP_DIR)/os/freertos/freertos_v10.0.1/Source/portable/GCC/ARM_RTL8710C \
	-I$(COMP_DIR)/os/os_dep/include \
	-I$(COMP_DIR)/application/amazon/amazon-ffs/libffs/include \
	-I$(COMP_DIR)/application/amazon/amazon-ffs/ffs_linux/libffs/include \
	-I$(COMP_DIR)/wifi/driver/include \
	-I$(COMP_DIR)/wifi/driver/src/osdep \
	-I$(COMP_DIR)/wifi/driver/src/hal \
	-I$(COMP_DIR)/wifi/driver/src/hci \
	-I$(COMP_DIR)/wifi/driver/src/hal/phydm/rtl8710c \
	-I$(COMP_DIR)/wifi/driver/src/hal/phydm \
#	Temporary change to fix compile error, will remove later \

# Source file list
# -------------------------------------------------------------------
SRC_C  =
SRAM_C =
ERAM_C =

APP_DIR  = $(COMP_DIR)/application
AT_DIR   = $(COMP_DIR)/at_cmd
BT_DIR   = $(COMP_DIR)/bluetooth
BT_ZE_DIR= $(COMP_DIR)/bluetooth/zephyr_stack
EXA_DIR  = $(COMP_DIR)/example
HAL_DIR  = $(COMP_DIR)/mbed/targets/hal/rtl8710c
LWIP_DIR = $(COMP_DIR)/lwip/lwip_v2.0.2
NET_DIR  = $(COMP_DIR)/network
RTOS_DIR = $(COMP_DIR)/os/freertos
SOC_DIR  = $(COMP_DIR)/soc/8710c
TLS_DIR  = $(COMP_DIR)/ssl/mbedtls-2.4.0
OS_DIR   = $(COMP_DIR)/os
WIFI_DIR = $(COMP_DIR)/wifi

#bluetooth
SRC_C += \
	$(COMP_DIR)/file_system/ftl/ftl.c \

#bluetooth - hci
SRC_C += \
    $(BT_DIR)/driver/hci/hci_process/hci_process.c \
    $(BT_DIR)/driver/hci/hci_process/hci_standalone.c \
    $(BT_DIR)/driver/hci/hci_transport/hci_h4.c \
    $(BT_DIR)/driver/hci/hci_if_zephyr.c \
    $(BT_DIR)/driver/platform/amebaz2/hci/hci_fwconfig.c \
    $(BT_DIR)/driver/platform/amebaz2/hci/hci_normal_patch.c \
    $(BT_DIR)/driver/platform/amebaz2/hci/hci_platform.c \
    $(BT_DIR)/driver/platform/amebaz2/hci/hci_uart.c \
    $(BT_DIR)/os/freertos/osif_freertos.c \

#bluetooth - platform
SRC_C += \
	$(BT_ZE_DIR)/platform/amebaz2/btsnoop_uart.c \
#bluetooth - example - ble_peripheral
#SRC_C +=  \
#bluetooth - example - ble_central
#SRC_C +=  \
#bluetooth - example - ble_scatternet
#SRC_C +=  \
#bluetooth - example - entry

SRC_C +=  \
	$(BT_ZE_DIR)/example/bt_example_entry.c \

#cmsis
SRC_C += \
	$(SOC_DIR)/cmsis/rtl8710c/source/ram_s/app_start.c \
	$(SOC_DIR)/cmsis/rtl8710c/source/ram/mpu_config.c \
	$(SOC_DIR)/cmsis/rtl8710c/source/ram/sys_irq.c \

#libc api wrapper
SRC_C +=  \
	$(SOC_DIR)/misc/utilities/source/ram/libc_wrap.c \

#console
SRC_C += \
	$(AT_DIR)/atcmd_bt_ze.c \
	$(AT_DIR)/atcmd_lwip.c \
	$(AT_DIR)/atcmd_mp.c \
	$(AT_DIR)/atcmd_mp_ext2.c \
	$(AT_DIR)/atcmd_sys.c \
	$(AT_DIR)/atcmd_wifi.c \
	$(AT_DIR)/log_service.c \
	$(SOC_DIR)/app/shell/cmd_shell.c \
	$(SOC_DIR)/app/shell/ram_s/consol_cmds.c \
	$(SOC_DIR)/misc/driver/rtl_console.c \
#	$(AT_DIR)/atcmd_bt.c \

#network - api
SRC_C += \
	$(LWIP_DIR)/../api/lwip_netconf.c \
	$(WIFI_DIR)/api/wifi_conf.c \
	$(WIFI_DIR)/api/wifi_conf_inter.c \
	$(WIFI_DIR)/api/wifi_conf_ext.c \
	$(WIFI_DIR)/promisc/wifi_conf_promisc.c \
	$(WIFI_DIR)/promisc/wifi_promisc.c \
	$(WIFI_DIR)/api/wifi_conf_wowlan.c \
	$(WIFI_DIR)/api/wifi_conf_mesh.c \
	$(WIFI_DIR)/api/wifi_ind.c \
	$(WIFI_DIR)/wifi_config/wifi_simple_config.c \
	$(WIFI_DIR)/wifi_fast_connect/wifi_fast_connect.c \
	$(WIFI_DIR)/wpa_supplicant/src/crypto/tls_polarssl.c \
	$(WIFI_DIR)/wpa_supplicant/wpa_supplicant/wifi_eap_config.c \
	$(WIFI_DIR)/wpa_supplicant/wpa_supplicant/wifi_p2p_config.c \
	$(WIFI_DIR)/wpa_supplicant/wpa_supplicant/wifi_wps_config.c \
#$(WIFI_DIR)/api/wifi_util.c \

#network - app
SRC_C += \
	$(EXA_DIR)/ssl_client/ssl_client.c \
	$(EXA_DIR)/ssl_client/ssl_client_ext.c \
	$(NET_DIR)/iperf/iperf.c \
	$(NET_DIR)/ping/ping_test.c \
	$(SOC_DIR)/misc/platform/ota_8710c.c \
	$(WIFI_DIR)/api/wlan_network.c \

#utilities
SRC_C += \
	$(APP_DIR)/gb2unicode/gb2unicode.c \
	$(NET_DIR)/cJSON/cJSON.c \
	$(NET_DIR)/httplite/http_client.c \
	$(NET_DIR)/xml/xml.c \

#network - app - mqtt
SRC_C += \
	$(NET_DIR)/mqtt/MQTTClient/MQTTClient.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTConnectClient.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTConnectServer.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTDeserializePublish.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTFormat.c \
	$(NET_DIR)/mqtt/MQTTClient/MQTTFreertos.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTPacket.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTSerializePublish.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTSubscribeClient.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTSubscribeServer.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTUnsubscribeClient.c \
	$(NET_DIR)/mqtt/MQTTPacket/MQTTUnsubscribeServer.c \

#network - coap
SRC_C += \
	$(NET_DIR)/coap/sn_coap_ameba_port.c \
	$(NET_DIR)/coap/sn_coap_builder.c \
	$(NET_DIR)/coap/sn_coap_header_check.c \
	$(NET_DIR)/coap/sn_coap_parser.c \
	$(NET_DIR)/coap/sn_coap_protocol.c \

#network - http
SRC_C +=  \
	$(NET_DIR)/httpc/httpc_tls.c \
	$(NET_DIR)/httpd/httpd_tls.c \

#network
SRC_C += \
	$(NET_DIR)/dhcp/dhcps.c \
	$(NET_DIR)/sntp/sntp.c \

#network - lwip
SRC_C += \
	$(LWIP_DIR)/src/api/api_lib.c \
	$(LWIP_DIR)/src/api/api_msg.c \
	$(LWIP_DIR)/src/api/err.c \
	$(LWIP_DIR)/src/api/netbuf.c \
	$(LWIP_DIR)/src/api/netdb.c \
	$(LWIP_DIR)/src/api/netifapi.c \
	$(LWIP_DIR)/src/api/sockets.c \
	$(LWIP_DIR)/src/api/tcpip.c \
	$(LWIP_DIR)/src/core/def.c \
	$(LWIP_DIR)/src/core/dns.c \
	$(LWIP_DIR)/src/core/inet_chksum.c \
	$(LWIP_DIR)/src/core/init.c \
	$(LWIP_DIR)/src/core/ip.c \
	$(LWIP_DIR)/src/core/mem.c \
	$(LWIP_DIR)/src/core/memp.c \
	$(LWIP_DIR)/src/core/netif.c \
	$(LWIP_DIR)/src/core/pbuf.c \
	$(LWIP_DIR)/src/core/raw.c \
	$(LWIP_DIR)/src/core/stats.c \
	$(LWIP_DIR)/src/core/sys.c \
	$(LWIP_DIR)/src/core/tcp.c \
	$(LWIP_DIR)/src/core/tcp_in.c \
	$(LWIP_DIR)/src/core/tcp_out.c \
	$(LWIP_DIR)/src/core/timeouts.c \
	$(LWIP_DIR)/src/core/udp.c \
	$(LWIP_DIR)/src/core/ipv4/autoip.c \
	$(LWIP_DIR)/src/core/ipv4/dhcp.c \
	$(LWIP_DIR)/src/core/ipv4/etharp.c \
	$(LWIP_DIR)/src/core/ipv4/icmp.c \
	$(LWIP_DIR)/src/core/ipv4/igmp.c \
	$(LWIP_DIR)/src/core/ipv4/ip4.c \
	$(LWIP_DIR)/src/core/ipv4/ip4_addr.c \
	$(LWIP_DIR)/src/core/ipv4/ip4_frag.c \
	$(LWIP_DIR)/src/core/ipv6/dhcp6.c \
	$(LWIP_DIR)/src/core/ipv6/ethip6.c \
	$(LWIP_DIR)/src/core/ipv6/icmp6.c \
	$(LWIP_DIR)/src/core/ipv6/inet6.c \
	$(LWIP_DIR)/src/core/ipv6/ip6.c \
	$(LWIP_DIR)/src/core/ipv6/ip6_addr.c \
	$(LWIP_DIR)/src/core/ipv6/ip6_frag.c \
	$(LWIP_DIR)/src/core/ipv6/mld6.c \
	$(LWIP_DIR)/src/core/ipv6/nd6.c \
	$(LWIP_DIR)/src/netif/ethernet.c \
	$(LWIP_DIR)/port/realtek/freertos/ethernetif.c \
	$(LWIP_DIR)/port/realtek/freertos/sys_arch.c \
	$(WIFI_DIR)/driver/src/osdep/lwip_intf.c \

#network - mdns
SRC_C += \
	$(NET_DIR)/mDNS/mDNSPlatform.c \

#network - ssl - mbedtls
SRC_C += \
	$(TLS_DIR)/library/aesni.c \
	$(TLS_DIR)/library/blowfish.c \
	$(TLS_DIR)/library/camellia.c \
	$(TLS_DIR)/library/ccm.c \
	$(TLS_DIR)/library/certs.c \
	$(TLS_DIR)/library/cipher.c \
	$(TLS_DIR)/library/cipher_wrap.c \
	$(TLS_DIR)/library/cmac.c \
	$(TLS_DIR)/library/debug.c \
	$(TLS_DIR)/library/error.c \
	$(TLS_DIR)/library/gcm.c \
	$(TLS_DIR)/library/havege.c \
	$(TLS_DIR)/library/md.c \
	$(TLS_DIR)/library/md2.c \
	$(TLS_DIR)/library/md4.c \
	$(TLS_DIR)/library/md_wrap.c \
	$(TLS_DIR)/library/memory_buffer_alloc.c \
	$(TLS_DIR)/library/net_sockets.c \
	$(TLS_DIR)/library/padlock.c \
	$(TLS_DIR)/library/pkcs11.c \
	$(TLS_DIR)/library/pkcs12.c \
	$(TLS_DIR)/library/pkcs5.c \
	$(TLS_DIR)/library/pkparse.c \
	$(TLS_DIR)/library/platform.c \
	$(TLS_DIR)/library/ripemd160.c \
	$(TLS_DIR)/library/sha256.c \
	$(TLS_DIR)/library/ssl_cache.c \
	$(TLS_DIR)/library/ssl_ciphersuites.c \
	$(TLS_DIR)/library/ssl_cli.c \
	$(TLS_DIR)/library/ssl_cookie.c \
	$(TLS_DIR)/library/ssl_srv.c \
	$(TLS_DIR)/library/ssl_ticket.c \
	$(TLS_DIR)/library/ssl_tls.c \
	$(TLS_DIR)/library/threading.c \
	$(TLS_DIR)/library/timing.c \
	$(TLS_DIR)/library/version.c \
	$(TLS_DIR)/library/version_features.c \
	$(TLS_DIR)/library/x509.c \
	$(TLS_DIR)/library/x509_create.c \
	$(TLS_DIR)/library/x509_crl.c \
	$(TLS_DIR)/library/x509_crt.c \
	$(TLS_DIR)/library/x509_csr.c \
	$(TLS_DIR)/library/x509write_crt.c \
	$(TLS_DIR)/library/x509write_csr.c \
	$(TLS_DIR)/library/xtea.c \

#network - ssl - ssl_ram_map
SRC_C += \
	$(TLS_DIR)/../ssl_ram_map/rom/rom_ssl_ram_map.c \
	$(TLS_DIR)/../ssl_func_stubs/ssl_func_stubs.c \

#network - websocket
SRC_C += \
	$(NET_DIR)/websocket/wsclient_tls.c \
	$(NET_DIR)/websocket/wsserver_tls.c \

#os
SRC_C += \
	$(RTOS_DIR)/cmsis_os.c \
	$(RTOS_DIR)/freertos_cb.c \
	$(RTOS_DIR)/freertos_service.c \
	$(RTOS_DIR)/freertos_pmu.c \
	$(RTOS_DIR)/freertos_static_mem.c \
	$(OS_DIR)/os_dep/device_lock.c \
	$(OS_DIR)/os_dep/osdep_service.c \
	$(OS_DIR)/os_dep/timer_service.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/croutine.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/event_groups.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/list.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/queue.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/stream_buffer.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/tasks.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/timers.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/portable/MemMang/heap_5.c \
	$(RTOS_DIR)/freertos_v10.0.1/Source/portable/GCC/ARM_RTL8710C/port.c \

#peripheral - api
SRC_C += \
	$(HAL_DIR)/crypto_api.c \
	$(HAL_DIR)/dma_api.c \
	$(HAL_DIR)/efuse_api.c \
	$(HAL_DIR)/gpio_api.c \
	$(HAL_DIR)/gpio_irq_api.c \
	$(HAL_DIR)/i2c_api.c \
	$(HAL_DIR)/pinmap.c \
	$(HAL_DIR)/pinmap_common.c \
	$(HAL_DIR)/port_api.c \
	$(HAL_DIR)/pwmout_api.c \
	$(HAL_DIR)/rtc_api.c \
	$(HAL_DIR)/serial_api.c \
	$(HAL_DIR)/spdio_api.c \
	$(HAL_DIR)/spi_api.c \
	$(HAL_DIR)/sys_api.c \
	$(HAL_DIR)/timer_api.c \
	$(HAL_DIR)/us_ticker.c \
	$(HAL_DIR)/us_ticker_api.c \
	$(HAL_DIR)/wait_api.c \
	$(HAL_DIR)/wdt_api.c \
	$(HAL_DIR)/reset_reason_api.c \

#peripheral - hal
SRC_C += \
	$(SOC_DIR)/fwlib/source/ram_s/hal_efuse.c \
	$(SOC_DIR)/fwlib/source/ram/hal_gdma.c \
	$(SOC_DIR)/fwlib/source/ram/hal_gpio.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_i2c.c \
	$(SOC_DIR)/fwlib/source/ram/hal_misc.c \
	$(SOC_DIR)/fwlib/source/ram_s/hal_pinmux_nsc.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_pwm.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_sdio_dev.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_ssi.c \
	$(SOC_DIR)/fwlib/source/ram/hal_uart.c \

#peripheral - wlan
#SRC_C += \
	$(WIFI_DIR)/driver/src/core/option/rtw_opt_rf_para_rtl8710c.c

#file_system - fatfs
SRC_C += \
	$(COMP_DIR)/file_system/fatfs/fatfs_ext/src/ff_driver.c \
	$(COMP_DIR)/file_system/fatfs/r0.10c/src/diskio.c \
	$(COMP_DIR)/file_system/fatfs/r0.10c/src/ff.c \
	$(COMP_DIR)/file_system/fatfs/r0.10c/src/option/ccsbcs.c \
	$(COMP_DIR)/file_system/fatfs/disk_if/src/flash_fatfs.c \
	
#utilities - ffs and example
#SRC_C += \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_base64.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_base85.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_configuration_map.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_hex.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_json.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_logging.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_result.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_stream.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/common/ffs_wifi.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_device_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_json_value.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_registration_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_registration_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_connection_attempt.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_connection_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_connection_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_credentials.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_provisionee_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_scan_result.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/conversion/ffs_convert_wifi_security_protocol.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_compute_configuration_data_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_compute_configuration_data_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_configuration.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_device_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_error_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_get_wifi_credentials_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_get_wifi_credentials_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_post_wifi_scan_data_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_post_wifi_scan_data_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_registration_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_registration_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_report_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_report_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_report_result.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_start_pin_based_setup_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_start_pin_based_setup_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_start_provisioning_session_request.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_start_provisioning_session_response.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_connection_attempt.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_connection_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_connection_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_credentials.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_provisionee_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_scan_result.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/model/ffs_dss_wifi_security_protocol.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_client.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_compute_configuration_data.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_get_wifi_credentials.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_post_wifi_scan_data.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_report.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_start_pin_based_setup.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/dss/ffs_dss_operation_start_provisioning_session.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/raspbian/ffs_raspbian_iwlist.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/raspbian/ffs_raspbian_wifi_manager.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs_api.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_circular_buffer.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_linked_list.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_configuration_map.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_crypto.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_linux_crypto_common.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_linux_error_details.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_http_client.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_logging.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_user_context.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/compat/ffs_linux_wifi.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_wifi_configuration_list.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_wifi_connection_attempt_list.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_wifi_context.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_wifi_manager.c  \
	$(APP_DIR)/amazon/amazon-ffs/ffs_linux/libffs/src/ffs/linux/ffs_wifi_scan_list.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/wifi_provisionee/ffs_wifi_provisionee_encoded_setup_network.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/wifi_provisionee/ffs_wifi_provisionee_setup_network.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/wifi_provisionee/ffs_wifi_provisionee_state.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/wifi_provisionee/ffs_wifi_provisionee_task.c  \
	$(APP_DIR)/amazon/amazon-ffs/libffs/src/ffs/wifi_provisionee/ffs_wifi_provisionee_user_network.c  \
	$(EXA_DIR)/amazon_ffs/example_ffs.c

#utilities - example
SRC_C += \
	$(EXA_DIR)/bcast/example_bcast.c \
	$(EXA_DIR)/cJSON/example_cJSON.c \
	$(EXA_DIR)/coap/example_coap.c \
	$(EXA_DIR)/dct/example_dct.c \
	$(EXA_DIR)/example_entry.c \
	$(EXA_DIR)/high_load_memory_use/example_high_load_memory_use.c \
	$(EXA_DIR)/http_client/example_http_client.c \
	$(EXA_DIR)/http_download/example_http_download.c \
	$(EXA_DIR)/httpc/example_httpc.c \
	$(EXA_DIR)/httpd/example_httpd.c \
	$(EXA_DIR)/mcast/example_mcast.c \
	$(EXA_DIR)/mqtt/example_mqtt.c \
	$(EXA_DIR)/nonblock_connect/example_nonblock_connect.c \
	$(EXA_DIR)/ota_http/example_ota_http.c \
	$(EXA_DIR)/rarp/example_rarp.c \
	$(EXA_DIR)/sntp_showtime/example_sntp_showtime.c \
	$(EXA_DIR)/socket_select/example_socket_select.c \
	$(EXA_DIR)/socket_tcp_trx/example_socket_tcp_trx_1.c \
	$(EXA_DIR)/socket_tcp_trx/example_socket_tcp_trx_2.c \
	$(EXA_DIR)/ssl_download/example_ssl_download.c \
	$(EXA_DIR)/ssl_server/example_ssl_server.c \
	$(EXA_DIR)/tcp_keepalive/example_tcp_keepalive.c \
	$(EXA_DIR)/uart_atcmd/example_uart_atcmd.c \
	$(EXA_DIR)/wifi_roaming/example_wifi_roaming.c \
	$(EXA_DIR)/wlan_scenario/example_wlan_scenario.c \
	$(EXA_DIR)/websocket_client/example_wsclient.c \
	$(EXA_DIR)/xml/example_xml.c \
	$(EXA_DIR)/fatfs/example_fatfs.c \
#	$(EXA_DIR)/wlan_fast_connect/example_wlan_fast_connect.c \
#	$(EXA_DIR)/amazon_awsiot/example_amazon_awsiot.c \
#	$(EXA_DIR)/coap_client/example_coap_client.c \
#	$(EXA_DIR)/coap_server/example_coap_server.c \
#	$(EXA_DIR)/eap/example_eap.c \
#	$(EXA_DIR)/googlenest/example_google.c \

#user
SRC_C += ../src/main.c
#SRC_CPP = ../src/main.cpp


# SRAM
# -------------------------------------------------------------------
# @SRAM
SRAM_C += \
	$(HAL_DIR)/flash_api.c \
	$(HAL_DIR)/power_mode_api.c \
	$(SOC_DIR)/misc/driver/flash_api_ext.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_flash.c \
	$(SOC_DIR)/fwlib/source/ram_ns/hal_spic.c \


# Generate obj list
# -------------------------------------------------------------------
SRC_O   = $(patsubst %.c,%_$(TARGET).o,$(SRC_C))
SRAM_O  = $(patsubst %.c,%_$(TARGET).o,$(SRAM_C))
ERAM_O  = $(patsubst %.c,%_$(TARGET).o,$(ERAM_C))
SRC_OO += $(patsubst %.cpp,%_$(TARGET).oo,$(SRC_CPP))

SRC_C_LIST       = $(notdir $(SRC_C)) $(notdir $(SRAM_C)) $(notdir $(ERAM_C))
OBJ_LIST         = $(addprefix $(OBJ_DIR)/,$(patsubst %.c,%_$(TARGET).o,$(SRC_C_LIST)))
DEPENDENCY_LIST  = $(addprefix $(OBJ_DIR)/,$(patsubst %.c,%_$(TARGET).d,$(SRC_C_LIST)))

SRC_CPP_LIST     = $(notdir $(SRC_CPP))
OBJ_CPP_LIST     = $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp,%_$(TARGET).oo,$(SRC_CPP_LIST)))
DEPENDENCY_LIST += $(addprefix $(OBJ_DIR)/,$(patsubst %.cpp,%_$(TARGET).d,$(SRC_CPP_LIST)))


# Compile options
# -------------------------------------------------------------------
CFLAGS  =
CFLAGS += \
	-march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=soft -D__thumb2__ -g -gdwarf-3 -Os \
	-D__ARM_ARCH_8M_MAIN__=1 -gdwarf-3 -fstack-usage -fdata-sections -ffunction-sections  \
	-fdiagnostics-color=always -Wall -Wpointer-arith -Wundef -Wno-write-strings --save-temps \
	-Wno-maybe-uninitialized -c -MMD \
	-DCONFIG_PLATFORM_8710C -DCONFIG_BUILD_RAM=1 \
	-DV8M_STKOVF -Wstrict-prototypes \
	-DCONFIG_USE_ZEPHYR_BT_STACK \

#for time64 
ifdef SYSTEM_TIME64_MAKE_OPTION
CFLAGS += -DCONFIG_SYSTEM_TIME64=1 -include time64.h
else
CFLAGS += -DCONFIG_SYSTEM_TIME64=0
endif

CPPFLAGS := $(CFLAGS)
CPPFLAGS += -std=c++11 -fno-use-cxa-atexit

# libc api wrapper
LFLAGS  = 
LFLAGS += \
	-Os -march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=soft -nostartfiles -nodefaultlibs -nostdlib -specs=nosys.specs \
	-Wl,--gc-sections -Wl,--warn-section-align -Wl,--cref -Wl,--build-id=none -Wl,--use-blx \
	-Wl,-Map=$(BIN_DIR)/$(TARGET).map \
	-Wl,-wrap,strcat  -Wl,-wrap,strchr   -Wl,-wrap,strcmp \
	-Wl,-wrap,strncmp -Wl,-wrap,strnicmp -Wl,-wrap,strcpy \
	-Wl,-wrap,strncpy -Wl,-wrap,strlcpy  -Wl,-wrap,strlen \
	-Wl,-wrap,strnlen -Wl,-wrap,strncat  -Wl,-wrap,strpbrk \
	-Wl,-wrap,strspn  -Wl,-wrap,strstr   -Wl,-wrap,strtok \
	-Wl,-wrap,strxfrm -Wl,-wrap,strsep   -Wl,-wrap,strtod \
	-Wl,-wrap,strtof  -Wl,-wrap,strtold  -Wl,-wrap,strtoll \
	-Wl,-wrap,strtoul -Wl,-wrap,strtoull -Wl,-wrap,atoi \
	-Wl,-wrap,atoui   -Wl,-wrap,atol     -Wl,-wrap,atoul \
	-Wl,-wrap,atoull  -Wl,-wrap,atof \
	-Wl,-wrap,malloc  -Wl,-wrap,realloc \
	-Wl,-wrap,calloc  -Wl,-wrap,free \
	-Wl,-wrap,_malloc_r  -Wl,-wrap,_calloc_r \
	-Wl,-wrap,memcmp  -Wl,-wrap,memcpy \
	-Wl,-wrap,memmove -Wl,-wrap,memset \
	-Wl,-wrap,printf  -Wl,-wrap,sprintf \
	-Wl,-wrap,puts  -Wl,-wrap,putc -Wl,-wrap,putchar \
	-Wl,-wrap,snprintf  -Wl,-wrap,vsnprintf \
	-Wl,-wrap,aesccmp_construct_mic_iv \
	-Wl,-wrap,aesccmp_construct_mic_header1 \
	-Wl,-wrap,aesccmp_construct_ctr_preload \
	-Wl,-wrap,rom_psk_CalcGTK \
	-Wl,-wrap,rom_psk_CalcPTK \
	-Wl,-wrap,aes_80211_encrypt \
	-Wl,-wrap,aes_80211_decrypt \

ifdef SYSTEM_TIME64_MAKE_OPTION
LFLAGS += -Wl,-wrap,localtime -Wl,-wrap,mktime -Wl,-wrap,ctime
endif

LIBFLAGS  =
LIBFLAGS += \
	-L$(AMEBAZ2_ROMSYMDIR) \
	-Wl,--start-group ../../../component/soc/8710c/fwlib/lib/lib/hal_pmc.a -Wl,--end-group \
	-L../../../component/soc/8710c/misc/bsp/lib/common/GCC -l_http -l_dct -l_eap -l_p2p -l_websocket -l_wps \
	-Wl,--start-group -L../../../component/soc/8710c/misc/bsp/lib/common/GCC -l_bt -Wl,--end-group \
#	-Wl,--start-group ../../../component/bluetooth/board/amebaz2/lib/btgap.a -Wl,--end-group \
#	-Wl,--start-group ../../../component/bluetooth/example/bt_mesh/lib/lib/amebaz2/btmesh_prov.a -Wl,--end-group \
#	-Wl,--start-group ../../../component/bluetooth/example/bt_mesh/lib/lib/amebaz2/btmesh_dev.a -Wl,--end-group \
#	-L../../../component/soc/8710c/misc/bsp/lib/common/GCC -l_coap \

all: LIBFLAGS += -Wl,--start-group -L../../../component/soc/8710c/misc/bsp/lib/common/GCC -l_soc_is -l_wlan -Wl,--end-group
mp:  LIBFLAGS += -Wl,--start-group -L../../../component/soc/8710c/misc/bsp/lib/common/GCC -l_soc_is -l_wlan_mp -Wl,--end-group

RAMALL_BIN =
OTA_BIN    = 

include toolchain.mk


# Compile
# -------------------------------------------------------------------
.PHONY: application_ze
application_ze: prerequirement $(SRC_O) $(SRAM_O) $(ERAM_O) $(SRC_OO)
	$(LD) $(LFLAGS) -o $(BIN_DIR)/$(TARGET).axf $(OBJ_CPP_LIST) -lstdc++ $(OBJ_LIST) $(ROMIMG) $(LIBFLAGS) -T$(LDSCRIPT)  
	$(OBJCOPY) -j .bluetooth_trace.text -Obinary $(BIN_DIR)/$(TARGET).axf $(BIN_DIR)/APP.trace
	$(OBJCOPY) -R .bluetooth_trace.text $(BIN_DIR)/$(TARGET).axf 
	$(OBJDUMP) -d $(BIN_DIR)/$(TARGET).axf > $(BIN_DIR)/$(TARGET).asm


# Manipulate Image
# -------------------------------------------------------------------
.PHONY: manipulate_images
manipulate_images: | application_ze
	@echo ===========================================================
	@echo Image manipulating
	@echo ===========================================================
	cp $(AMEBAZ2_BOOTLOADERDIR)/bootloader.axf $(BOOT_BIN_DIR)/bootloader.axf
ifeq ($(findstring Linux, $(OS)), Linux)
	chmod 0774 $(ELF2BIN) $(CHKSUM)
endif
	$(ELF2BIN) keygen keycfg.json
	$(ELF2BIN) convert amebaz2_bootloader.json BOOTLOADER secure_bit=0
	$(ELF2BIN) convert amebaz2_bootloader.json PARTITIONTABLE secure_bit=0
	$(ELF2BIN) convert amebaz2_firmware_ze.json FIRMWARE secure_bit=0
	$(CHKSUM) $(BIN_DIR)/firmware_ze.bin
	$(ELF2BIN) combine $(BIN_DIR)/flash_ze.bin PTAB=partition.bin,BOOT=$(BOOT_BIN_DIR)/bootloader.bin,FW1=$(BIN_DIR)/firmware_ze.bin


# Generate build info
# -------------------------------------------------------------------
.PHONY: build_info
build_info:
	@echo \#define RTL_FW_COMPILE_TIME RTL8710CFW_COMPILE_TIME\ > .ver
	@echo \#define RTL_FW_COMPILE_DATE RTL8710CFW_COMPILE_DATE\ >> .ver
	@echo \#define UTS_VERSION \"`date +%Y/%m/%d-%T`\" >> .ver
	@echo \#define RTL8710CFW_COMPILE_TIME \"`date +%Y/%m/%d-%T`\" >> .ver
	@echo \#define RTL8710CFW_COMPILE_DATE \"`date +%Y%m%d`\" >> .ver
	@echo \#define RTL8710CFW_COMPILE_BY \"`id -u -n`\" >> .ver
	@echo \#define RTL8710CFW_COMPILE_HOST \"`$(HOSTNAME_APP)`\" >> .ver
	@if [ -x /bin/dnsdomainname ]; then \
		echo \#define RTL8710CFW_COMPILE_DOMAIN \"`dnsdomainname`\"; \
	elif [ -x /bin/domainname ]; then \
		echo \#define RTL8710CFW_COMPILE_DOMAIN \"`domainname`\"; \
	else \
		echo \#define RTL8710CFW_COMPILE_DOMAIN ; \
	fi >> .ver

	@echo \#define RTL8710CFW_COMPILER \"gcc `$(CC) $(CFLAGS) -dumpversion | tr --delete '\r'`\" >> .ver
	@mv -f .ver ../inc/$@.h

.PHONY: prerequirement
prerequirement:
	@if [ ! -d $(ARM_GCC_TOOLCHAIN) ]; then \
		echo ===========================================================; \
		echo Toolchain not found, \"make toolchain\" first!; \
		echo ===========================================================; \
		exit -1; \
	fi
	@echo ===========================================================
	@echo Build $(TARGET)
	@echo ===========================================================
	mkdir -p $(OBJ_DIR)
	mkdir -p $(BIN_DIR)
	mkdir -p $(BOOT_BIN_DIR)
	mkdir -p $(INFO_DIR)

$(SRC_OO): %_$(TARGET).oo : %.cpp | prerequirement
	$(CC) $(CPPFLAGS) -c $< -o $@
	$(CC) $(CPPFLAGS) -c $< -MM -MT $@ -MF $(OBJ_DIR)/$(notdir $(patsubst %.oo,%.d,$@))
	cp $@ $(OBJ_DIR)/$(notdir $@)
	mv $(notdir $*.ii) $(INFO_DIR)
	chmod 777 $(OBJ_DIR)/$(notdir $@)
#	mv $(notdir $*.s) $(INFO_DIR)

$(SRC_O): %_$(TARGET).o : %.c | prerequirement
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -MM -MT $@ -MF $(OBJ_DIR)/$(notdir $(patsubst %.o,%.d,$@))
	cp $@ $(OBJ_DIR)/$(notdir $@)
	mv $(notdir $*.i) $(INFO_DIR)
	mv $(notdir $*.s) $(INFO_DIR)
	chmod 777 $(OBJ_DIR)/$(notdir $@)

$(SRAM_O): %_$(TARGET).o : %.c | prerequirement
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(OBJCOPY) --prefix-alloc-sections .sram $@
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -MM -MT $@ -MF $(OBJ_DIR)/$(notdir $(patsubst %.o,%.d,$@))
	cp $@ $(OBJ_DIR)/$(notdir $@)
	mv $(notdir $*.i) $(INFO_DIR)
	mv $(notdir $*.s) $(INFO_DIR)
	chmod 777 $(OBJ_DIR)/$(notdir $@)

$(ERAM_O): %_$(TARGET).o : %.c | prerequirement
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	$(OBJCOPY) --prefix-alloc-sections .psram $@
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -MM -MT $@ -MF $(OBJ_DIR)/$(notdir $(patsubst %.o,%.d,$@))
	cp $@ $(OBJ_DIR)/$(notdir $@)
	mv $(notdir $*.i) $(INFO_DIR)
	mv $(notdir $*.s) $(INFO_DIR)
	chmod 777 $(OBJ_DIR)/$(notdir $@)

-include $(DEPENDENCY_LIST)

# Only needed for FPGA phase
#	$(FLASH_TOOLDIR)/Check_Jtag.sh
.PHONY: romdebug
romdebug:
ifeq ($(findstring CYGWIN, $(OS)), CYGWIN) 
	cmd /c start $(GDB) -x ./rtl_gdb_debug_jlink.txt
else
	$(GDB) -x ./rtl_gdb_debug_jlink.txt	
endif

#	@if [ ! -f $(FLASH_TOOLDIR)/rtl_gdb_flash_write.txt ] ; then echo Please do /"make setup GDB_SERVER=[jlink or openocd]/" first; echo && false ; fi
#ifeq ($(findstring CYGWIN, $(OS)), CYGWIN) 
#	$(FLASH_TOOLDIR)/Check_Jtag.sh
#endif
.PHONY: flash
flash:
	@if [ ! -e $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader.txt ]; then \
		echo ===========================================================; \
		echo gdb script not found, \"make setup GDB_SERVER=[jlink, pyocd or openocd]\" first!; \
		echo ===========================================================; \
		exit -1; \
	fi
	chmod +rx $(AMEBAZ2_GCCTOOLDIR)/flashloader.sh
	$(AMEBAZ2_GCCTOOLDIR)/flashloader.sh application_ze/Debug/bin/flash_ze.bin
	$(GDB) -x $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader.txt

.PHONY: debug
debug:
	@if [ ! -e $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug.txt ]; then \
		echo ===========================================================; \
		echo gdb script not found, \"make setup GDB_SERVER=[jlink, pyocd or openocd]\" first!; \
		echo ===========================================================; \
		exit -1; \
	fi
	chmod +rx $(AMEBAZ2_GCCTOOLDIR)/debug.sh
	$(AMEBAZ2_GCCTOOLDIR)/debug.sh $(BIN_DIR)/$(TARGET).axf
	$(GDB) -x $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug.txt

.PHONY: dump
dump:
	chmod +rx $(AMEBAZ2_GCCTOOLDIR)/dump.sh
	$(AMEBAZ2_GCCTOOLDIR)/dump.sh $(BIN_DIR)/flash_ze_dump.bin $(DUMP_START_ADDRESS) $(DUMP_END_ADDRESS)
	$(GDB) -x $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_dump_jlink.txt

.PHONY: setup
setup:
	@echo "----------------"
	@echo Setup $(GDB_SERVER)
	@echo "----------------"
ifeq ($(GDB_SERVER), pyocd)
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug_pyocd.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug.txt
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader_pyocd.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader.txt
else ifeq ($(GDB_SERVER), openocd)
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug_openocd.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug.txt
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader_openocd.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader.txt
else
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug_jlink.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_debug.txt
	cp -p $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader_jlink.txt $(AMEBAZ2_GCCTOOLDIR)/rtl_gdb_flashloader.txt
endif

.PHONY: clean
clean:
	rm -rf $(TARGET)
	rm -f $(SRC_O) $(SRAM_O) $(ERAM_O) $(SRC_OO)
	rm -f $(patsubst %.o,%.d,$(SRC_O)) $(patsubst %.o,%.d,$(SRAM_O)) $(patsubst %.o,%.d,$(ERAM_O)) $(patsubst %.oo,%.d,$(SRC_OO))
	rm -f $(patsubst %.o,%.su,$(SRC_O)) $(patsubst %.o,%.su,$(SRAM_O)) $(patsubst %.o,%.su,$(ERAM_O)) $(patsubst %.oo,%.su,$(SRC_OO))
	rm -f *.i
	rm -f *.s
	rm -f *.d
	rm -f *.o
