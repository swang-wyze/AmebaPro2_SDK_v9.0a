cmake_minimum_required(VERSION 3.6)

project(app_ntz)

enable_language(C ASM)

# root of realtek_amebapro2_v0_example
set (prj_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")
# root of SDK
set (sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(app_ntz application.ntz)

set(freertos "freertos_v202012.00")
set(lwip "lwip_v2.1.2")

include(../config.cmake)
link_directories(${prj_root}/GCC-RELEASE/application_ntz/output)




ADD_LIBRARY (skynet_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET skynet_lib PROPERTY IMPORTED_LOCATION ${prj_root}/src/doorbell-chime/skynet_iot/GCC/libSkynetAPI_iot.a )

ADD_LIBRARY (bt_upperstack_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET bt_upperstack_lib PROPERTY IMPORTED_LOCATION ${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/lib/btgap.a )

message(STATUS "Build libraries ${BUILD_LIB}")
message(STATUS "Build FPGA ${BUILD_FPGA}")
message(STATUS "Build PXP ${BUILD_PXP}")



#HAL
list(
    APPEND app_ntz_sources
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_flash.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_flash_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_flash_sec.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_hkdf.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_otp_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_pinmux_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_snand.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_spic.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_spic_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_sys_ctrl.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_wdt.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_rtc.c
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_audio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_adc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_comp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_crypto.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_init.c	
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_scan.c	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eddsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gdma.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2c.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2s.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_otp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_pwm.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_rsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sdhost.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_ssi.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_timer.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_trng.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_uart.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sgpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_i2s.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sgpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sport.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_ssi.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_audio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eth.c
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_eth.c
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/hal_video.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/hal_video_cmd.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/hal_video_peri.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/hal_voe.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/semihost/SEGGER_SEMIHOST_CortexM.S	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/semihost/SEGGER_SEMIHOST.c
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram_s/rtl8735b_rtc.c
	
)

#MBED
list(
    APPEND app_ntz_sources
	${sdk_root}/component/mbed/targets/hal/rtl8735b/audio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/crypto_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/dma_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/flash_api.c
	${sdk_root}/component/soc/8735b/misc/driver/flash_api_ext.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/i2c_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/i2s_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pwmout_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/sgpio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/spi_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/timer_api.c
	${sdk_root}/component/soc/8735b/mbed-drivers/source/wait_api.c
	${sdk_root}/component/soc/8735b/mbed-drivers/source/us_ticker_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/us_ticker.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/gpio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/gpio_irq_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/serial_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/wdt_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/rtc_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/analogin_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pinmap_common.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pinmap.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/ethernet_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/trng_api.c


)


#RTOS
list(
    APPEND app_ntz_sources
	${sdk_root}/component/os/freertos/${freertos}/Source/croutine.c
	${sdk_root}/component/os/freertos/${freertos}/Source/event_groups.c
	${sdk_root}/component/os/freertos/${freertos}/Source/list.c
	${sdk_root}/component/os/freertos/${freertos}/Source/queue.c
	${sdk_root}/component/os/freertos/${freertos}/Source/stream_buffer.c
	${sdk_root}/component/os/freertos/${freertos}/Source/tasks.c
	${sdk_root}/component/os/freertos/${freertos}/Source/timers.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/MemMang/heap_4_2.c
	
	${sdk_root}/component/os/freertos/freertos_cb.c
	${sdk_root}/component/os/freertos/freertos_service.c
	${sdk_root}/component/os/freertos/cmsis_os.c
	
	${sdk_root}/component/os/os_dep/osdep_service.c
	${sdk_root}/component/os/os_dep/device_lock.c
	${sdk_root}/component/os/os_dep/timer_service.c
)

#CMSIS
list(
    APPEND app_ntz_sources
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram_s/app_start.c
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram/mpu_config.c
	
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_add_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_bitreversal2.S
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_cfft_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_cfft_radix8_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/ComplexMathFunctions/arm_cmplx_mag_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/CommonTables/arm_common_tables.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/CommonTables/arm_const_structs.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/StatisticsFunctions/arm_max_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_mult_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_rfft_fast_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_rfft_fast_init_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_scale_f32.c
)

#at_cmd
list(
	APPEND app_ntz_sources
	${sdk_root}/component/at_cmd/atcmd_sys.c
	${sdk_root}/component/at_cmd/atcmd_wifi.c
	${sdk_root}/component/at_cmd/atcmd_bt.c
	${sdk_root}/component/at_cmd/log_service.c
	${sdk_root}/component/soc/8735b/misc/driver/rtl_console.c
)

#wifi
list(
	APPEND app_ntz_sources
	#api
	${sdk_root}/component/wifi/api/wifi_conf.c
	${sdk_root}/component/wifi/api/wifi_conf_inter.c
	${sdk_root}/component/wifi/api/wifi_conf_ext.c
	${sdk_root}/component/wifi/api/wifi_ind.c
	${sdk_root}/component/wifi/api/wlan_network.c
	#promisc
	${sdk_root}/component/wifi/promisc/wifi_conf_promisc.c
	${sdk_root}/component/wifi/promisc/wifi_promisc.c
	#fast_connect
	#${sdk_root}/component/wifi/wifi_fast_connect/wifi_fast_connect.c
	#wpa_supplicant
	${sdk_root}/component/wifi/wpa_supplicant/wpa_supplicant/wifi_wps_config.c
	#option
	${sdk_root}/component/wifi/driver/src/core/option/rtw_opt_crypto_ssl.c
)

#network
list(
	APPEND app_ntz_sources
	#dhcp
	${sdk_root}/component/network/dhcp/dhcps.c
	#ping
	${sdk_root}/component/network/ping/ping_test.c
	#iperf
	${sdk_root}/component/network/iperf/iperf.c
	#sntp
	${sdk_root}/component/network/sntp/sntp.c
	#http
	${sdk_root}/component/network/httpc/httpc_tls.c
	${sdk_root}/component/network/httpd/httpd_tls.c
)

#lwip
list(
	APPEND app_ntz_sources
	#api
	${sdk_root}/component/lwip/api/lwip_netconf.c
	#lwip - api
	${sdk_root}/component/lwip/${lwip}/src/api/api_lib.c
	${sdk_root}/component/lwip/${lwip}/src/api/api_msg.c
	${sdk_root}/component/lwip/${lwip}/src/api/err.c
	${sdk_root}/component/lwip/${lwip}/src/api/netbuf.c
	${sdk_root}/component/lwip/${lwip}/src/api/netdb.c
	${sdk_root}/component/lwip/${lwip}/src/api/netifapi.c
	${sdk_root}/component/lwip/${lwip}/src/api/sockets.c
	${sdk_root}/component/lwip/${lwip}/src/api/tcpip.c
	#lwip - core
	${sdk_root}/component/lwip/${lwip}/src/core/def.c
	${sdk_root}/component/lwip/${lwip}/src/core/dns.c
	${sdk_root}/component/lwip/${lwip}/src/core/inet_chksum.c
	${sdk_root}/component/lwip/${lwip}/src/core/init.c
	${sdk_root}/component/lwip/${lwip}/src/core/ip.c
	${sdk_root}/component/lwip/${lwip}/src/core/mem.c
	${sdk_root}/component/lwip/${lwip}/src/core/memp.c
	${sdk_root}/component/lwip/${lwip}/src/core/netif.c
	${sdk_root}/component/lwip/${lwip}/src/core/pbuf.c
	${sdk_root}/component/lwip/${lwip}/src/core/raw.c
	${sdk_root}/component/lwip/${lwip}/src/core/stats.c
	${sdk_root}/component/lwip/${lwip}/src/core/sys.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp_in.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp_out.c
	${sdk_root}/component/lwip/${lwip}/src/core/timeouts.c
	${sdk_root}/component/lwip/${lwip}/src/core/udp.c
	#lwip - core - ipv4
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/autoip.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/dhcp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/etharp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/icmp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/igmp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4_addr.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4_frag.c
	#lwip - core - ipv6
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/dhcp6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ethip6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/icmp6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/inet6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6_addr.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6_frag.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/mld6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/nd6.c
	#lwip - netif
	${sdk_root}/component/lwip/${lwip}/src/netif/ethernet.c
	#lwip - port
	${sdk_root}/component/lwip/${lwip}/port/realtek/freertos/ethernetif.c
	${sdk_root}/component/wifi/driver/src/osdep/lwip_intf.c
	${sdk_root}/component/lwip/${lwip}/port/realtek/freertos/sys_arch.c
)

#ssl
list(
	APPEND app_ntz_sources
	#mbedtls
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/aesni.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/blowfish.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/camellia.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ccm.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/certs.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cipher.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cipher_wrap.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cmac.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/debug.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/gcm.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/havege.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/md2.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/md4.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/memory_buffer_alloc.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/net_sockets.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/padlock.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs11.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs12.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs5.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkparse.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/platform.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ripemd160.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cache.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_ciphersuites.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cli.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cookie.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_srv.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_ticket.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_tls.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/threading.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/timing.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/version.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/version_features.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_create.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_crl.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_crt.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_csr.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509write_crt.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509write_csr.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/xtea.c
	#ssl_ram_map
	${sdk_root}/component/ssl/ssl_ram_map/rom/rom_ssl_ram_map.c
	${sdk_root}/component/ssl/ssl_func_stubs/ssl_func_stubs.c
)

#example
list(
	APPEND app_ntz_sources
	${sdk_root}/component/example/example_entry.c
	${sdk_root}/component/example/ssl_client/ssl_client.c
	${sdk_root}/component/example/ssl_client/ssl_client_ext.c
	${sdk_root}/component/example/fatfs/example_fatfs.c
	${sdk_root}/component/example/sntp_showtime/example_sntp_showtime.c
	${sdk_root}/component/example/httpc/example_httpc.c
	${sdk_root}/component/example/httpd/example_httpd.c
	${sdk_root}/component/example/media_mp4_demuxer/example_media_mp4_demuxer.c
)

#FATFS
list(
	APPEND app_ntz_sources
	${sdk_root}/component/file_system/fatfs/disk_if/src/sdcard.c
	${sdk_root}/component/file_system/fatfs/disk_if/src/flash_fatfs.c
	${sdk_root}/component/file_system/fatfs/fatfs_ext/src/ff_driver.c
	
	${sdk_root}/component/file_system/fatfs/r0.14/diskio.c
	${sdk_root}/component/file_system/fatfs/r0.14/ff.c
	${sdk_root}/component/file_system/fatfs/r0.14/ffsystem.c
	${sdk_root}/component/file_system/fatfs/r0.14/ffunicode.c
	${sdk_root}/component/file_system/fatfs/fatfs_flash_api.c
	${sdk_root}/component/file_system/fatfs/fatfs_reent.c
	${sdk_root}/component/file_system/fatfs/fatfs_sdcard_api.c
	${sdk_root}/component/file_system/fatfs/fatfs_ramdisk_api.c
	${prj_root}/src/sd/example_sd_hot_plug.c
)

#USER
list(
    APPEND app_ntz_sources
	${prj_root}/src/main.c
	#${prj_root}/src/main_nn.c
)

#RTSP
list(
	APPEND app_ntz_sources	
	${sdk_root}/component/network/rtsp/rtp_api.c
	${sdk_root}/component/network/rtsp/rtsp_api.c
	${sdk_root}/component/network/rtsp/sdp.c
)

#MMF_MODULE
list(
	APPEND app_ntz_sources	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin/sensor_gc2053.s
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin/sensor_fixp.s
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin/iq_bin.s
	${sdk_root}/component/media/mmfv2/module_video.c
	${sdk_root}/component/media/mmfv2/module_rtsp2.c
	${sdk_root}/component/media/mmfv2/module_array.c
	${sdk_root}/component/media/mmfv2/module_audio.c
	${sdk_root}/component/media/mmfv2/module_aac.c
	${sdk_root}/component/media/mmfv2/module_aad.c
	${sdk_root}/component/media/mmfv2/module_g711.c
	#${sdk_root}/component/media/mmfv2/module_httpfs.c
	${sdk_root}/component/media/mmfv2/module_i2s.c
	${sdk_root}/component/media/mmfv2/module_mp4.c
	${sdk_root}/component/media/mmfv2/module_rtp.c
    ${sdk_root}/component/media/mmfv2/module_opusc.c
	${sdk_root}/component/media/mmfv2/module_opusd.c
)

#UVCD AUTO TEST
list(
	APPEND app_ntz_sources
	${sdk_root}/component/example/media_uvcd/example_media_uvc.c
	${sdk_root}/component/media/mmfv2/module_array.c
	${sdk_root}/component/media/mmfv2/module_uvcd.c
	${sdk_root}/component/example/media_uvcd/usbd_uvc_parm.c
)

#VOE TEST
list(
	APPEND app_ntz_sources	
	${prj_root}/src/mmfv2_video_example/video_example_media_framework.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_array_rtsp_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_v1_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_v2_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_v3_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_v1_snapshot_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_simo_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_av_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_av2_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_av21_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_av_mp4_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_av_rtsp_mp4_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_joint_test_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_joint_test_rtsp_mp4_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_2way_audio_pcmu_doorbell_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_2way_audio_pcmu_init.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_v1_param_change_init.c
)

#AUDIO TEST
list(
	APPEND app_ntz_sources

	${sdk_root}/component/example/media_framework/example_media_framework.c
	${sdk_root}/component/example/media_framework/mmf2_example_a_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_aacloop_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_audioloop_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_g711loop_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_2way_audio_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_aac_array_rtsp_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_pcmu_array_rtsp_init.c
	${sdk_root}/component/example/media_framework/mmf2_example_rtp_aad_init.c
    	${sdk_root}/component/example/media_framework/mmf2_example_i2s_audio_init.c
    ${sdk_root}/component/example/media_framework/mmf2_example_2way_audio_opus_init.c
    ${sdk_root}/component/example/media_framework/mmf2_example_a_opus_init.c
    ${sdk_root}/component/example/media_framework/mmf2_example_rtp_opusd_init.c
    ${sdk_root}/component/example/media_framework/mmf2_example_opusloop_init.c
)

#CLOUD TEST
list(
	APPEND app_ntz_sources

    #test skynet
	${prj_root}/src/doorbell-chime/skynet_device.c
	${prj_root}/src/doorbell-chime/module_p2p.c
	${prj_root}/src/doorbell-chime/module_p2p_aac.c
	${prj_root}/src/mmfv2_video_example/mmf2_video_example_p2p_av_init.c
	#${prj_root}/src/doorbell-chime/skynet_wakeup.c
)

#MISC
list(
	APPEND app_ntz_sources

	${sdk_root}/component/soc/8735b/misc/utilities/source/ram/libc_wrap.c
	${sdk_root}/component/soc/8735b/app/shell/cmd_shell.c	
)

#LIB
list(
	APPEND app_ntz_sources
	
	# move libsoc_ntz
)

#FTL
list(
	APPEND app_ntz_sources
	
	${sdk_root}/component/file_system/ftl/ftl.c
)

#BLUETOOTH
list(
	APPEND app_ntz_sources

	${sdk_root}/component/bluetooth/driver/hci/hci_process/hci_process.c
	${sdk_root}/component/bluetooth/driver/hci/hci_process/hci_standalone.c
	${sdk_root}/component/bluetooth/driver/hci/hci_transport/hci_h4.c
	${sdk_root}/component/bluetooth/driver/hci/hci_if_rtk.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/bt_normal_patch.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_dbg.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_platform.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_uart.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/platform_utils.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/rtk_coex.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/trace_uart.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/src/cycle_queue.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/src/trace_task.c
	${sdk_root}/component/bluetooth/os/freertos/osif_freertos.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_example_entry.c

	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/server/simple_ble_service.c
	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/server/bas.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/ble_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/peripheral_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/ble_peripheral_at_cmd.c

	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/client/gcs_client.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_client_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_link_mgr.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_at_cmd.c

	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_link_mgr.c

	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app_task.c
)

if(BUILD_KVS_DEMO)
	include(./libkvs_amazon.cmake)
endif()

if(SIMULATION)
	if(BUILD_PXP)
		set(sim_rom_path ${prj_root}/GCC-RELEASE/ROM/GCC/PXP)
	endif()
	if(BUILD_FPGA)
		set(sim_rom_path ${prj_root}/GCC-RELEASE/ROM/GCC/FPGA)
	endif()

	#add command for build PXP or FPGA
	add_custom_command(
		OUTPUT rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/rom.s -I${sim_rom_path} -o rom.o
	)

	add_custom_command(
		OUTPUT dtcm_rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/dtcm_rom.s -I${sim_rom_path} -o dtcm_rom.o
	)

	add_custom_command(
		OUTPUT itcm_rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/itcm_rom.s -I${sim_rom_path} -o itcm_rom.o
	)

	add_executable(
		${app_ntz}
		${app_ntz_sources}
		#$<TARGET_OBJECTS:rom> 
		#$<TARGET_OBJECTS:soc> 
		rom.o
		dtcm_rom.o
		itcm_rom.o
	)
	
	set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_sim.ld )
else()
	add_executable(
		${app_ntz}
		${app_ntz_sources}
		#$<TARGET_OBJECTS:rom> 
		#$<TARGET_OBJECTS:soc> 
	)
	
	if(NODDR)
		message(STATUS "WITHOUT DDR")
		set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_noddr.ld ) 
	else()
		message(STATUS "WITH DDR")
		set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram.ld )
	endif()
endif()


list(
	APPEND app_ntz_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1
)

target_compile_definitions(${app_ntz} PRIVATE ${app_ntz_flags} )

# HEADER FILE PATH
include(../includepath.cmake)
target_include_directories(
	${app_ntz}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
	${prj_root}/src/mmfv2_video_example/
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin/
	
	${sdk_root}/component/media/mmfv2
	${sdk_root}/component/media/rtp_codec
	${sdk_root}/component/example/media_framework/inc
	${sdk_root}/component/audio/3rdparty/AEC
	${sdk_root}/component/mbed/hal_ext
	${sdk_root}/component/example/media_framework
	${sdk_root}/component/file_system/ftl
	${prj_root}/src/doorbell-chime

	${sdk_root}/component/bluetooth/driver
	${sdk_root}/component/bluetooth/driver/hci
	${sdk_root}/component/bluetooth/driver/inc
	${sdk_root}/component/bluetooth/driver/inc/hci
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/inc
	${sdk_root}/component/bluetooth/os/osif
	${sdk_root}/component/bluetooth/rtk_stack/example
	${sdk_root}/component/bluetooth/rtk_stack/inc/app
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/gap
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile/client
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile/server
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/bt
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/remote
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/sys
	${sdk_root}/component/bluetooth/rtk_stack/inc/os
	${sdk_root}/component/bluetooth/rtk_stack/inc/platform
	${sdk_root}/component/bluetooth/rtk_stack/inc/stack
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/inc
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/lib
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/inc
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_airsync_config
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh/provisioner
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh/device
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_multiple_profile/provisioner_multiple_profile
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_multiple_profile/device_multiple_profile
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_test
	
	${sdk_root}/component/wifi/wpa_supplicant/src
	${sdk_root}/component/wifi/driver/src/core/option
	${sdk_root}/component/ssl/ssl_ram_map/rom
	${sdk_root}/component/audio/3rdparty/faac/libfaac
	${sdk_root}/component/soc/8735b/app/file_system/drivers/sdio/realtek/sdio_host/inc
)

if(BUILD_KVS_DEMO)
target_include_directories(
	${app_ntz}
	PUBLIC

	${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/producer/src/include
	${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/include
	${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/include/portable/realtek/rtl8195
	${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/include
	${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/include/portable
	${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/include/private
)

target_link_libraries(
	${app_ntz} 
	kvs_webrtc
	kvs_producer_c
	kvs_pic
	websockets
	srtp2
	kvs_producer
)
endif()

target_link_libraries(
	${app_ntz} 
	wlan
	wps
    opus
    opusenc
    opusfile
    hmp3
	g711
	http
	aec
	mmf
	skynet_lib
	bt_upperstack_lib
	sdcard
	faac
	haac
	muxer
	usbd
	soc_ntz
#	-Wl,--no-whole-archive
#	soc 
#	-Wl,--whole-archive
#	rom
	m
	c
	gcc
	nosys
)

target_link_options(
	${app_ntz} 
	PUBLIC
	"LINKER:SHELL:-L ${CMAKE_CURRENT_SOURCE_DIR}/../ROM/GCC"
	"LINKER:SHELL:-T ${ld_script}"
	"LINKER:SHELL:-Map=${app_ntz}.map"
)

#add command for build PXP
#add_custom_target(
#
#)

add_custom_command(TARGET ${app_ntz} POST_BUILD 
	COMMAND ${CMAKE_NM} $<TARGET_FILE:${app_ntz}> | sort > ${app_ntz}.nm.map
	COMMAND ${CMAKE_OBJEDUMP} -d $<TARGET_FILE:${app_ntz}> > ${app_ntz}.asm
	COMMAND cp $<TARGET_FILE:${app_ntz}> ${app_ntz}.axf
	COMMAND cp ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin/voe.bin voe.bin
	COMMAND ${CMAKE_OBJCOPY} -j .bluetooth_trace.text -Obinary ${app_ntz}.axf APP.trace
	COMMAND ${CMAKE_OBJCOPY} -R .bluetooth_trace.text ${app_ntz}.axf 
	#COMMAND cp ${app} ${app}_pure
	#COMMAND ${CMAKE_STRIP} --strip-unneeded ${app}_pure
	#COMMAND ${CMAKE_OBJCOPY} -j .rom.vector_tbl -j .rom.ns_vector_tbl -j .rom.text -j .ARM.extab -j .ARM.exidx -j .data_of_rom -j .note.gnu.build-id -j .rom.stubs -j .rom.ns4s_stubs -Obinary ${app}_pure rom.bin
	#COMMAND ${CMAKE_OBJCOPY} -j .gnu.sgstubs  -Obinary ${app}_pure nsc.bin
	#COMMAND ${CMAKE_OBJCOPY} -j .ram.func.table -j .ram.img.signature -j .data -j .ram.code_text -j .ram.code_rodata  -Obinary ${app}_pure ram_ld.bin
	#COMMAND ${CMAKE_OBJCOPY} -j .ram.vector -j .ram.func.table -j .ram.img.signature -j .ram.func.table -j .data -j .ram.code_text -j .ram.code_rodata  -Obinary ${app}_pure ram.bin
	#COMMAND rm -rf info
	#COMMAND mkdir info
	#COMMAND mv *.{i,s} info
	
	#COMMAND ${prj_root}/GCC-RELEASE/elf2bin.exe convert ${prj_root}/GCC-RELEASE/amebapro2_partition.json PARTITIONTABLE
	
	COMMAND cp ${prj_root}/GCC-RELEASE/keycfg.json keycfg.json
	COMMAND cp ${prj_root}/GCC-RELEASE/partition.json partition.json
	COMMAND cp ${prj_root}/GCC-RELEASE/amebapro2_firmware.json amebapro2_firmware.json
	COMMAND ${prj_root}/GCC-RELEASE/elf2bin.exe keygen keycfg.json
	COMMAND ${prj_root}/GCC-RELEASE/elf2bin.exe convert amebapro2_firmware.json FIRMWARE
	COMMAND rm -rf output && mkdir output
	COMMAND cp firmware_ntz.bin output
	COMMAND cp ${app_ntz}.nm.map output
	COMMAND cp ${app_ntz}.map output
	COMMAND cp ${app_ntz}.asm output
	COMMAND cp ${app_ntz}.axf output
	#COMMAND cp *.a output
	#COMMAND rm -rf output && mkdir output
	#COMMAND cp ${boot} output
	#COMMAND python ../split_binary.py output
	#COMMAND cd .. && tar zcvf build.tar.gz build/output
)

if(BUILD_PXP)
add_custom_command(TARGET ${app_ntz} POST_BUILD 
	COMMAND cp ${app_ntz}.axf output/application.axf
	COMMAND python ${prj_root}/GCC-RELEASE/split_binary.py output Ignore
	COMMAND rm output/application.axf
	COMMAND cd .. && tar zcvf build.tar.gz build/output
)
endif()

if(BUILD_KVS_DEMO)
add_custom_command(TARGET ${app_ntz} POST_BUILD 
	COMMAND cp *.a output
)
endif()
