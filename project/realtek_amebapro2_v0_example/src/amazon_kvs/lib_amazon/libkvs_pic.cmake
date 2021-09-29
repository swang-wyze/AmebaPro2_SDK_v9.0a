cmake_minimum_required(VERSION 3.6)

project(kvs_pic)

set(kvs_pic kvs_pic)

list(
    APPEND kvs_pic_sources

#state
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/state/src/State.c
#utils
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Allocators.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Atomics.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Base64.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/BitField.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/BitReader.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Crc32.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Directory.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/DoubleLinkedList.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/DynamicLibrary.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Endianness.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/FileIo.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/FileLogger.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/HashTable.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Hex.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/InstrumentedAllocators.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Logger.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Mutex.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Semaphore.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/SingleLinkedList.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/StackQueue.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/String.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Tags.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Thread.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Time.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/TimerQueue.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/src/Version.c

)


add_library(
    ${kvs_pic} STATIC
    ${kvs_pic_sources}
)

list(
	APPEND kvs_pic_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
	__IEEE_LITTLE_ENDIAN
)

target_compile_definitions(${kvs_pic} PRIVATE ${kvs_pic_flags} )

target_include_directories(
	${kvs_pic}
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
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/common/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/state/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/utils/include
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-pic/src/mkvgen/include
    
)