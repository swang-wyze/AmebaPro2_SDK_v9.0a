#ifndef _EXAMPLE_KVS_WEBRTC_H_
#define _EXAMPLE_KVS_WEBRTC_H_

void example_kvs_webrtc(void);

/* Enter your AWS KVS key here */
#define KVS_WEBRTC_ACCESS_KEY   "xxxxxxxxxxxxxxxxxxxx"
#define KVS_WEBRTC_SECRET_KEY   "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

/* Setting your signaling channel name */
#define KVS_WEBRTC_CHANNEL_NAME "xxxxxxxxxxxxxxxxxxxx"

/* Setting your AWS region */
#define KVS_WEBRTC_REGION       "us-west-2"

/* Cert path */
#define TEMP_CERT_PATH          "0://cert.pem"  // path to CA cert

/* log level */
#define KVS_WEBRTC_LOG_LEVEL    LOG_LEVEL_ERROR  //LOG_LEVEL_VERBOSE  LOG_LEVEL_WARN

/* Video output buffer size */
#define KVS_VIDEO_OUTPUT_BUFFER_SIZE    1920*1080/10

/* Audio format setting */
#define AUDIO_G711_MULAW        1
#define AUDIO_G711_ALAW         0
#define AUDIO_OPUS              0

/* Enable two-way audio communication (not support opus format now)*/
#define ENABLE_AUDIO_SENDRECV

/*
 * Testing Amazon KVS WebRTC with IAM user key is easy but it is not recommended.
 * With AWS IoT Thing credentials, it can be managed more securely.(https://iotlabtpe.github.io/Amazon-KVS-WebRTC-WorkShop/lab/lab-4.html)
 * Script for generate iot credential: https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/scripts/generate-iot-credential.sh
 */
#define ENABLE_KVS_WEBRTC_IOT_CREDENTIAL    0

/* IoT credential configuration */
#if ENABLE_KVS_WEBRTC_IOT_CREDENTIAL

#define KVS_WEBRTC_IOT_CREDENTIAL_ENDPOINT      "xxxxxxxxxxxxxx.credentials.iot.us-west-2.amazonaws.com"  // IoT credentials endpointiot
#define KVS_WEBRTC_ROLE_ALIAS                   "webrtc_iot_role_alias"  // IoT role alias
#define KVS_WEBRTC_THING_NAME                   KVS_WEBRTC_CHANNEL_NAME  // iot thing name, recommended to be same as your channel name

#define KVS_WEBRTC_ROOT_CA \
"-----BEGIN CERTIFICATE-----\n" \
"......\n" \
"-----END CERTIFICATE-----\n"

#define KVS_WEBRTC_CERTIFICATE \
"-----BEGIN CERTIFICATE-----\n" \
"......\n" \
"-----END CERTIFICATE-----\n"

#define KVS_WEBRTC_PRIVATE_KEY \
"-----BEGIN RSA PRIVATE KEY-----\n" \
"......\n" \
"-----END RSA PRIVATE KEY-----\n"
#endif /* ENABLE_KVS_WEBRTC_IOT_CREDENTIAL */

#endif /* _EXAMPLE_KVS_WEBRTC_H_ */

