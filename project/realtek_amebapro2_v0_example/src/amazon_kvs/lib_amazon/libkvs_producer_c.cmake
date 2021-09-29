cmake_minimum_required(VERSION 3.6)

project(kvs_producer_c)

set(kvs_producer_c kvs_producer_c)

list(
    APPEND kvs_producer_c_sources

#lws
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/Lws/LwsCall.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/Lws/LwsIotCredentialProvider.c
#common
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/Auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/AwsV4Signer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/FileCredentialProvider.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/IotCredentialProvider.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/jsmn.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/RequestInfo.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/StaticCredentialProvider.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/Util.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/source/Common/Version.c
)


add_library(
    ${kvs_producer_c} STATIC
    ${kvs_producer_c_sources}
)

list(
	APPEND kvs_producer_c_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
	__IEEE_LITTLE_ENDIAN
    KVS_USE_MBEDTLS
    KVS_BUILD_WITH_LWS
    LWS_AMAZON_RTOS
)

target_compile_definitions(${kvs_producer_c} PRIVATE ${kvs_producer_c_flags} )

target_include_directories(
	${kvs_producer_c}
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
    
    ${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-producer-c/src/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/client/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/common/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/heap/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/state/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/trace/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/view/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/mkvgen/include
    
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets/include
    ${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets
    
)