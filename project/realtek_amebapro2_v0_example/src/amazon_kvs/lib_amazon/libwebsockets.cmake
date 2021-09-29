cmake_minimum_required(VERSION 3.6)

project(websockets)

set(websockets websockets)

list(
    APPEND websockets_sources

###core
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/alloc.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/buflist.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/context.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/libwebsockets.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/logs.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core/lws_dll2.c
###core-net
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/adopt.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/client.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/close.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/connect.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/dummy-callback.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/network.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/output.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/pollfd.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/sequencer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/service.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/sorted-usec-list.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/state.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/vhost.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/wsi-timeout.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net/wsi.c
###event-libs
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/event-libs/poll/poll.c
###misc
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/base64-decode.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/lejp.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/lws-ring.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/lwsac/lwsac.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/romfs.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/misc/sha-1.c
###plat
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-fds.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-init.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-misc.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-pipe.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-service.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/freertos-sockets.c
###roles
##h1
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h1/ops-h1.c
##h2
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h2/hpack.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h2/http2.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h2/ops-h2.c
##http
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http/client/client-handshake.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http/client/client-http.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http/header.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http/parsers.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http/server/ranges.c
##pipe
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/pipe/ops-pipe.c
##raw
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/raw-file/ops-raw-file.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/raw-skt/ops-raw-skt.c
##ws
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/ws/client-parser-ws.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/ws/client-ws.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/ws/ops-ws.c
###system
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/system/smd/smd.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/system/system.c
###tls
##mbedtls
#wrapper
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_cert.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_lib.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_methods.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_pkey.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/platform/ssl_pm.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/platform/ssl_port.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_stack.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/library/ssl_x509.c
##mbedtls
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/mbedtls-client.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/mbedtls-ssl.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/mbedtls-tls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/mbedtls-x509.c
###tls
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/tls-client.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/tls-network.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/tls.c
)


add_library(
    ${websockets} STATIC
    ${websockets_sources}
)

list(
	APPEND websockets_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
    LWS_AMAZON_RTOS
    LWS_RTK_PLATFORM
)

target_compile_definitions(${websockets} PRIVATE ${websockets_flags} )

target_include_directories(
	${websockets}
	PUBLIC

	# ${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure

    ${prj_root}/inc
    ${sdk_root}/component/mbed/hal
    ${sdk_root}/component/mbed/hal_ext
    ${sdk_root}/component/mbed/targets/hal/rtl8735b
	${sdk_root}/component/mbed/api
    ${sdk_root}/component/stdlib
    ${sdk_root}/component/at_cmd
    ${sdk_root}/component/example
    ${sdk_root}/component/network
    ${sdk_root}/component/soc/8735b/cmsis/cmsis-core/include
    ${sdk_root}/component/soc/8735b/cmsis/rtl8735b/lib/include
    ${sdk_root}/component/soc/8735b/cmsis/rtl8735b/include

    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/include
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/host/storage/inc/quirks
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/device/class/ethernet/inc
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx/halmac_8822b
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/include
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/device/class/ethernet/src
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/device/core/inc
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx/halmac_8735b
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx_v1
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/device/class/vendor/inc
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/host/storage/inc
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/host/storage/inc/scatterlist
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx/halmac_8821c
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/host/vendor_spec
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx_v1/halmac_8814b
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/host/storage/inc/scsi
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/inc
    ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/halmac/halmac_88xx/halmac_8195b
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/usb_otg/device

	${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/include
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/core-net
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/event-libs
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/event-libs/poll
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/plat/freertos/esp32
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h1
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/h2
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/http
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/roles/ws
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/system/smd
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/include
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/include/internal
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/include/openssl
	${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/lib/tls/mbedtls/wrapper/include/platform

    ${sdk_root}/component/os/os_dep/include
    ${sdk_root}/component/os/freertos
    ${sdk_root}/component/os/freertos/${freertos}/Source/include

    ${sdk_root}/component/lwip/api
    ${sdk_root}/component/lwip/${lwip}/src/include
    ${sdk_root}/component/lwip/${lwip}/src/include/lwip
    ${sdk_root}/component/lwip/${lwip}/port/realtek
    ${sdk_root}/component/lwip/${lwip}/port/realtek/freertos

    ${sdk_root}/component/ssl/mbedtls-2.16.6/include
    ${sdk_root}/component/ssl/mbedtls_ram_map/rom
    
    ${sdk_root}/component/file_system/fatfs
    ${sdk_root}/component/file_system/fatfs/r0.14
    
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/include
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/include/portable/realtek/rtl8195
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/include
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/include/private
    
)