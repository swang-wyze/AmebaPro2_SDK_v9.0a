### libkvs_amazon.cmake ###

include(${prj_root}/src/amazon_kvs/lib_amazon/libsrtp2.cmake)
include(${prj_root}/src/amazon_kvs/lib_amazon/libwebsockets.cmake)
include(${prj_root}/src/amazon_kvs/lib_amazon/libkvs_producer.cmake)
include(${prj_root}/src/amazon_kvs/lib_amazon/libkvs_pic.cmake)
include(${prj_root}/src/amazon_kvs/lib_amazon/libkvs_producer_c.cmake)
include(${prj_root}/src/amazon_kvs/lib_amazon/libkvs_webrtc.cmake)

list(FILTER app_ntz_sources EXCLUDE REGEX "mbedtls-") #remove the existing mbedtls in project, then add mbedtls-2.16.6
list(FILTER app_ntz_sources EXCLUDE REGEX "ssl_func_stubs.c")
list(
	APPEND app_ntz_sources
    #mbedtls-2.16.6
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aes.c
    ### ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aes_alt.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aesni.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/arc4.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aria.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/asn1parse.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/asn1write.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/base64.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/bignum.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/blowfish.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/camellia.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ccm.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/certs.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/chacha20.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/chachapoly.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/cipher.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/cipher_wrap.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/cmac.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ctr_drbg.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/debug.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/des.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/dhm.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ecdh.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ecdsa.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ecjpake.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ecp.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ecp_curves.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/entropy.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/entropy_alt.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/entropy_poll.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/error.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/gcm.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/havege.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/hkdf.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/hmac_drbg.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/md.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/md2.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/md4.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/md5.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/md_wrap.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/memory_buffer_alloc.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/net_sockets.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/nist_kw.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/oid.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/padlock.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pem.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pk.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pk_wrap.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pkcs11.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pkcs12.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pkcs5.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pkparse.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/pkwrite.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/platform.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/platform_util.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/poly1305.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ripemd160.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/rsa.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/rsa_internal.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/sha1.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/sha256.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/sha512.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_cache.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_ciphersuites.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_cli.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_cookie.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_srv.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_ticket.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/ssl_tls.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/threading.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/timing.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/version.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/version_features.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509_create.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509_crl.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509_crt.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509_csr.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509write_crt.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/x509write_csr.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/xtea.c
)

#AMAZON KVS PRODUCER & WEBRTC
list(
	APPEND app_ntz_sources

    #kvs mmf module
    ${sdk_root}/component/example/kvs_producer_mmf/module_kvs_producer.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/module_kvs_webrtc.c

    #kvs example
	${sdk_root}/component/example/kvs_producer_mmf/example_kvs_producer_mmf.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/Common.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/example_kvs_webrtc_mmf.c

    #posix
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_clock.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_mqueue.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_barrier.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_cond.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_mutex.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_sched.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_semaphore.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_timer.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_unistd.c
    ${prj_root}/src/amazon_kvs/lib_amazon/posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_utils.c
)
