cmake_minimum_required(VERSION 3.6)

project(srtp2)

set(srtp2 srtp2)

list(
    APPEND srtp2_sources

##crypto
#cipher
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes_gcm_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes_icm_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/cipher.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/null_cipher.c
#hash
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/hmac.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/hmac_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/null_auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/sha1.c
#kernal
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/alloc.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/crypto_kernel.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/err.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/key.c
#math
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/math/datatypes.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/math/stat.c
#reply
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/rdb.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/rdbx.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/ut_sim.c
##srtp
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/srtp/ekt.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/srtp/srtp.c
)


add_library(
    ${srtp2} STATIC
    ${srtp2_sources}
)

list(
	APPEND srtp2_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
    HAVE_CONFIG_H
    KVS_PLAT_RTK_FREERTOS
)

target_compile_definitions(${srtp2} PRIVATE ${srtp2_flags} )

target_include_directories(
	${srtp2}
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
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/include
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/include
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp

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
    
)