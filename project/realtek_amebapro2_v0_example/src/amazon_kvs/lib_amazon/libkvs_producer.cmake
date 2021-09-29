cmake_minimum_required(VERSION 3.6)

project(kvs_producer)

set(kvs_producer kvs_producer)

list(
    APPEND kvs_producer_sources

##c-utility
#pal
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/pal/freertos/lock.c
#src
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/buffer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/consolelogger.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/crt_abstractions.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/doublylinkedlist.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/httpheaders.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/map.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/strings.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/src/xlogging.c
##llhttp
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/llhttp/src/api.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/llhttp/src/http.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/llhttp/src/llhttp.c
##parson
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/parson/parson.c
##producer
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/aws_signer_v4.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/http_helper.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/iot_credential_provider.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/json_helper.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/port/port_amebapro.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/mkv_generator.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/nalu.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/netio.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/restapi.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/sps_decode.c
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/source/stream.c
)


add_library(
    ${kvs_producer} STATIC
    ${kvs_producer_sources}
)

list(
	APPEND kvs_producer_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
    ENABLE_LOG_ERROR
    ENABLE_LOG_WARN
    ENABLE_LOG_INFO
)

target_compile_definitions(${kvs_producer} PRIVATE ${kvs_producer_flags} )

target_include_directories(
	${kvs_producer}
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

    ${sdk_root}/component/os/os_dep/include
    ${sdk_root}/component/os/freertos
    ${sdk_root}/component/os/freertos/${freertos}/Source/include

    ${sdk_root}/component/lwip/api
    ${sdk_root}/component/lwip/${lwip}/src/include
    ${sdk_root}/component/lwip/${lwip}/src/include/lwip
    ${sdk_root}/component/lwip/${lwip}/src/include/compat/posix
    ${sdk_root}/component/lwip/${lwip}/port/realtek
    ${sdk_root}/component/lwip/${lwip}/port/realtek/freertos

    ${sdk_root}/component/ssl/mbedtls-2.16.6/include
    ${sdk_root}/component/ssl/mbedtls_ram_map/rom
    
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/parson
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/src/include
    ${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/llhttp/include
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/inc
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/deps/umock-c/inc
    ${prj_root}/src/amazon_kvs/lib_amazon/producer/libraries/3rdparty/c-utility/deps/azure-macro-utils-c/inc
    
)