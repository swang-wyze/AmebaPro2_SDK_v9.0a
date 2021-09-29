# Amazon KVS WebRTC demo on AmebaPro2 #

## Download the necessary source code from Github
- Go to `project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon`
    ```
    cd project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon
    ```
- Clone the following repository for KVS webrtc
	- amazon-kinesis-video-streams-webrtc-sdk-c
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/amazon-kinesis-video-streams-webrtc-sdk-c.git
    ```
    - amazon-kinesis-video-streams-producer-c
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/amazon-kinesis-video-streams-producer-c.git
    ```
    - amazon-kinesis-video-streams-pic
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/amazon-kinesis-video-streams-pic.git
    ```
    - cisco/libsrtp
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/libsrtp.git
    ```
    - warmcat/libwebsockets
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/libwebsockets.git
    ```
- Clone the following repository for KVS producer
	- amazon-kinesis-video-streams-producer-embedded-c
    ```
    cd project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon
    git clone --recursive https://github.com/aws-samples/amazon-kinesis-video-streams-producer-embedded-c.git producer
    cd producer
    git reset --hard 8c9d4b29cb95ddcfad816e50a34e770222bb8ff5
    ```

## Add KVS demo code to your project
- Add the KVS demo code to project, we do it in `project/realtek_amebapro2_v0_example/GCC-RELEASE/application_ntz/libkvs_amazon.cmake`:  
```
list(
	APPEND app_ntz_sources

    #kvs mmf module
    ${sdk_root}/component/example/kvs_producer_mmf/module_kvs_producer.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/module_kvs_webrtc.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/module_kvs_webrtc_audio.c

    #kvs example
	${sdk_root}/component/example/kvs_producer_mmf/example_kvs_producer_mmf.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/Common.c
    ${sdk_root}/component/example/kvs_webrtc_mmf/example_kvs_webrtc_mmf.c
)
```

## Using mbedtls-2.16.6 to replace the existing mbedtls version in project
- In KVS webrtc project, we have to use some function in mbedtls-2.16.6  
we have to remove original mbedtls src add the src of mbedtls-2.16.6 to project, and we already do it in `project/realtek_amebapro2_v0_example/GCC-RELEASE/application_ntz/libkvs_amazon.cmake`:
```
...
list(FILTER app_ntz_sources EXCLUDE REGEX "mbedtls-") #remove the existing mbedtls in project, then add mbedtls-2.16.6
list(FILTER app_ntz_sources EXCLUDE REGEX "ssl_func_stubs.c")
list(
	APPEND app_ntz_sources
    #mbedtls-2.16.6
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aes.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aesni.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/arc4.c
    ${sdk_root}/component/ssl/mbedtls-2.16.6/library/aria.c
    ...
)
...
```
also, modify the include path for mbedtls-2.16.6 in `project\realtek_amebapro2_v0_example\GCC-RELEASE\includepath.cmake`  
```
...
if(BUILD_KVS_DEMO)
    list(FILTER inc_path EXCLUDE REGEX "mbedtls-") #remove the existing mbedtls in project, then add mbedtls-2.16.6
    list (
        APPEND inc_path
        "${sdk_root}/component/ssl/mbedtls-2.16.6/include"
    )
endif()
...
```

## Replace the file
- replace the following file in SDK with the files in `project\realtek_amebapro2_v0_example\src\amazon_kvs\replace\`
    - `project/realtek_amebapro2_v0_example/src/main.c`
    - `project/realtek_amebapro2_v0_example/inc/FreeRTOSConfig.h`
    - `component/os/freertos/freertos_cb.c`
    - `component/os/freertos/freertos_v202012.00/Source/portable/MemMang/heap_4_2.c`
    - `component/lwip/api/lwipopts.h`

## No using the wrapper function for snprintf 
- In `project/realtek_amebapro2_v0_example/GCC-RELEASE/toolchain.cmake`, comment the following wrapper function
    ```
    # "-Wl,-wrap,sprintf"
    # "-Wl,-wrap,snprintf"
    # "-Wl,-wrap,vsnprintf"
    ```

## Congiure the example
- configure AWS key channel name in `component/example/kvs_webrtc_mmf/example_kvs_webrtc.h`
    ```
    /* Enter your AWS KVS key here */
    #define KVS_WEBRTC_ACCESS_KEY   "xxxxxxxxxx"
    #define KVS_WEBRTC_SECRET_KEY   "xxxxxxxxxx"

    /* Setting your signaling channel name */
    #define KVS_WEBRTC_CHANNEL_NAME "xxxxxxxxxx"
    ```
- configure video parameter in `component/example/kvs_webrtc_mmf/example_kvs_webrtc_mmf.c`
    ```
    ...
    #define V1_RESOLUTION VIDEO_HD
    #define V1_FPS 30
    #define V1_GOP 30
    #define V1_BPS 1024*1024
    ```
- add following code to the end of example_entry.c:  
    ```
    #if CONFIG_EXAMPLE_KVS_WEBRTC_MMF
        example_kvs_webrtc_mmf();
    #endif
    ```
- define the demo you want to run in platform_opts.h  
    ```
    /* For KVS WebRTC mmf module example*/
    #define CONFIG_EXAMPLE_KVS_WEBRTC_MMF           1
    ```

## Build the project
- run following commands to build the image with option `-DBUILD_KVS_DEMO=ON`
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DBUILD_KVS_DEMO=ON
    cmake --build . --target flash
    ```

- use image tool to download the image to AmebaPro2 and reboot

- configure WiFi Connection  
    While runnung the example, you may need to configure WiFi connection by using these commands in uart terminal.  
    ```
    ATW0=<WiFi_SSID> : Set the WiFi AP to be connected
    ATW1=<WiFi_Password> : Set the WiFi AP password
    ATWC : Initiate the connection
    ```

- if everything works fine, you should see the following log
    ```
    ...
    wifi connected
    [KVS Master] Using trickleICE by default
    cert path:0://cert.pem
    look for ssl cert successfully
    [KVS Master] Created signaling channel My_KVS_Signaling_Channel
    [KVS Master] Finished setting audio and video handlers
    [KVS Master] KVS WebRTC initialization completed successfully
    N:  mem: platform fd map:   120 bytes
    N: lws_tls_client_create_vhost_context: using mem client CA cert 1424
    [KVS Master] Signaling client created successfully
    [KVS Master] Signaling client connection to socket established
    [KVS Master] Channel My_KVS_Signaling_Channel set up done
    ...
    ```

## Validate result
- we can use KVS WebRTC Test Page to test the result  
https://awslabs.github.io/amazon-kinesis-video-streams-webrtc-sdk-js/examples/index.html
