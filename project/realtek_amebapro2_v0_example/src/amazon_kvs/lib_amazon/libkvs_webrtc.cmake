cmake_minimum_required(VERSION 3.6)

project(kvs_webrtc)

set(kvs_webrtc kvs_webrtc)

list(
    APPEND kvs_webrtc_sources

##crypto
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Crypto/Dtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Crypto/Dtls_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Crypto/IOBuffer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Crypto/Tls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Crypto/Tls_mbedtls.c
##ice
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/ConnectionListener.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/IceAgent.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/IceAgentStateMachine.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/IceUtils.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/NatBehaviorDiscovery.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/Network.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/SocketConnection.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Ice/TurnConnection.c
##metrics
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Metrics/Metrics.c
##PeerConnection
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/DataChannel.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/JitterBuffer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/jsmn.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/PeerConnection.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/Retransimitter.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/Rtcp.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/Rtp.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/SessionDescription.c
##Rtcp
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtcp/RollingBuffer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtcp/RtcpPacket.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtcp/RtpRollingBuffer.c
##Rtp
#Codecs
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs/RtpG711Payloader.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs/RtpH264Payloader.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs/RtpOpusPayloader.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs/RtpVP8Payloader.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/RtpPacket.c
##Sctp
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Sctp/Sctp.c
##Sdp
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Sdp/Deserialize.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Sdp/Serialize.c
##signaling
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/ChannelInfo.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/Client.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/FileCache.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/LwsApiCalls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/Signaling.c
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Signaling/StateMachine.c
##Srtp
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Srtp/SrtpSession.c
##Stun
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Stun/Stun.c

)


add_library(
    ${kvs_webrtc} STATIC
    ${kvs_webrtc_sources}
)

list(
	APPEND kvs_webrtc_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	ARM_MATH_ARMV8MML
	__IEEE_LITTLE_ENDIAN
    KVS_USE_MBEDTLS
    KVS_BUILD_WITH_LWS
    KVS_PLAT_RTK_FREERTOS
    LWS_RTK_PLATFORM
    LWS_AMAZON_RTOS
    BUILD_CLIENT
    ENABLE_STREAMING
)

target_compile_definitions(${kvs_webrtc} PRIVATE ${kvs_webrtc_flags} )

target_include_directories(
	${kvs_webrtc}
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
    ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/include
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/include
    
)