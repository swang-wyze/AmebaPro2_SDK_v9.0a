/*******************************************
Shared include file for the samples
*******************************************/
#ifndef __KINESIS_VIDEO_SAMPLE_INCLUDE__
#define __KINESIS_VIDEO_SAMPLE_INCLUDE__

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <com/amazonaws/kinesis/video/webrtcclient/Include.h>
#include "example_kvs_webrtc.h"

#define NUMBER_OF_H264_FRAME_FILES               1500
#define NUMBER_OF_OPUS_FRAME_FILES               618
#define DEFAULT_FPS_VALUE                        30
#define DEFAULT_MAX_CONCURRENT_STREAMING_SESSION 10

#define SAMPLE_MASTER_CLIENT_ID "ProducerMaster"
#define SAMPLE_VIEWER_CLIENT_ID "ConsumerViewer"
#define SAMPLE_CHANNEL_NAME     (PCHAR) CONFIG_AWS_KVS_CHANNEL

#define SAMPLE_AUDIO_FRAME_DURATION (20 * HUNDREDS_OF_NANOS_IN_A_MILLISECOND)
#define SAMPLE_STATS_DURATION       (60 * HUNDREDS_OF_NANOS_IN_A_SECOND)
#define SAMPLE_VIDEO_FRAME_DURATION (HUNDREDS_OF_NANOS_IN_A_SECOND / DEFAULT_FPS_VALUE)

#define SAMPLE_SESSION_CLEANUP_WAIT_PERIOD (5 * HUNDREDS_OF_NANOS_IN_A_SECOND)

#define ASYNC_ICE_CONFIG_INFO_WAIT_TIMEOUT (3 * HUNDREDS_OF_NANOS_IN_A_SECOND)
#define ICE_CONFIG_INFO_POLL_PERIOD        (20 * HUNDREDS_OF_NANOS_IN_A_MILLISECOND)

#define CA_CERT_PEM_FILE_EXTENSION ".pem"

#define FILE_LOGGING_BUFFER_SIZE (100 * 1024)
#define MAX_NUMBER_OF_LOG_FILES  5

#define SAMPLE_HASH_TABLE_BUCKET_COUNT  50
#define SAMPLE_HASH_TABLE_BUCKET_LENGTH 2

#define SAMPLE_VIDEO_THREAD_NAME "videosource"
#define SAMPLE_VIDEO_THREAD_SIZE 20*1024

#define SAMPLE_AUDIO_THREAD_NAME "audiosource"
#define SAMPLE_AUDIO_THREAD_SIZE 4096

#define SAMPLE_TIMER_NAME "sampleTimer"
#define SAMPLE_TIMER_SIZE 10240

typedef enum {
	SAMPLE_STREAMING_VIDEO_ONLY,
	SAMPLE_STREAMING_AUDIO_VIDEO,
} SampleStreamingMediaType;

typedef struct __SampleStreamingSession SampleStreamingSession;
typedef struct __SampleStreamingSession *PSampleStreamingSession;

typedef struct {
	UINT64 prevNumberOfPacketsSent;
	UINT64 prevNumberOfPacketsReceived;
	UINT64 prevNumberOfBytesSent;
	UINT64 prevNumberOfBytesReceived;
	UINT64 prevPacketsDiscardedOnSend;
	UINT64 prevTs;
} RtcMetricsHistory, *PRtcMetricsHistory;

typedef struct {
	volatile ATOMIC_BOOL appTerminateFlag;
	volatile ATOMIC_BOOL interrupted;
	volatile ATOMIC_BOOL mediaThreadStarted;
	volatile ATOMIC_BOOL recreateSignalingClient;
	volatile ATOMIC_BOOL connected;
	BOOL useTestSrc;
	ChannelInfo channelInfo;
	PCHAR pCaCertPath;
	PAwsCredentialProvider pCredentialProvider;
	SIGNALING_CLIENT_HANDLE signalingClientHandle;
	PBYTE pAudioFrameBuffer;
	UINT32 audioBufferSize;
	PBYTE pVideoFrameBuffer;
	UINT32 videoBufferSize;
	TID mediaSenderTid;
	TIMER_QUEUE_HANDLE timerQueueHandle;
	UINT32 iceCandidatePairStatsTimerId;
	SampleStreamingMediaType mediaType;//!< the control of video only, or video/audio.
	startRoutine audioSource;//!< the thread handler of audio transmission.
	startRoutine videoSource;//!< the thread handler of video transmission.
	startRoutine receiveAudioVideoSource;
#ifdef ENABLE_DATA_CHANNEL
	RtcOnDataChannel onDataChannel;
#endif

	TID signalingProcessor;
	PHashTable pPendingSignalingMessageForRemoteClient;
	PHashTable pRtcPeerConnectionForRemoteClient;

	MUTEX sampleConfigurationObjLock;
	CVAR cvar;
	BOOL trickleIce;
	BOOL useTurn;
	BOOL enableFileLogging;
	UINT64 customData;
	PSampleStreamingSession sampleStreamingSessionList[DEFAULT_MAX_CONCURRENT_STREAMING_SESSION];
	UINT32 streamingSessionCount;
	MUTEX streamingSessionListReadLock;
	UINT32 iceUriCount;
	SignalingClientCallbacks signalingClientCallbacks;
	SignalingClientInfo clientInfo;
	RtcStats rtcIceCandidatePairMetrics;

	MUTEX signalingSendMessageLock;
} SampleConfiguration, *PSampleConfiguration;

typedef VOID (*StreamSessionShutdownCallback)(UINT64, PSampleStreamingSession);

struct __SampleStreamingSession {
	volatile ATOMIC_BOOL terminateFlag;
	volatile ATOMIC_BOOL candidateGatheringDone;
	volatile ATOMIC_BOOL peerIdReceived;
	volatile SIZE_T frameIndex;
	PRtcPeerConnection pPeerConnection;
	PRtcRtpTransceiver pVideoRtcRtpTransceiver;
	PRtcRtpTransceiver pAudioRtcRtpTransceiver;
	RtcSessionDescriptionInit answerSessionDescriptionInit;
	PSampleConfiguration pSampleConfiguration;
	UINT32 audioTimestamp;
	UINT32 videoTimestamp;
	CHAR peerId[MAX_SIGNALING_CLIENT_ID_LEN + 1];
	TID receiveAudioVideoSenderTid;
	UINT64 offerReceiveTime;
	UINT64 startUpLatency;
	BOOL firstFrame;
	RtcMetricsHistory rtcMetricsHistory;
	BOOL remoteCanTrickleIce;

	// this is called when the SampleStreamingSession is being freed
	StreamSessionShutdownCallback shutdownCallback;
	UINT64 shutdownCallbackCustomData;
	PRtcDataChannel pRtcDataChannel;
};

VOID sigintHandler(INT32);
STATUS readFrameFromDisk(PBYTE, PUINT32, PCHAR);
PVOID sendVideoPackets(PVOID);
PVOID sendAudioPackets(PVOID);
PVOID sendGstreamerAudioVideo(PVOID);
PVOID sampleReceiveVideoFrame(PVOID args);
PVOID getPeriodicIceCandidatePairStats(PVOID);
STATUS getIceCandidatePairStatsCallback(UINT32 timerId, UINT64 currentTime, UINT64 customData);
STATUS createSampleConfiguration(PCHAR, SIGNALING_CHANNEL_ROLE_TYPE, BOOL, BOOL, PSampleConfiguration *);
STATUS freeSampleConfiguration(PSampleConfiguration *);
STATUS signalingClientStateChanged(UINT64, SIGNALING_CLIENT_STATE);
STATUS signalingMessageReceived(UINT64 customData, PReceivedSignalingMessage pReceivedSignalingMessage);
STATUS handleAnswer(PSampleConfiguration, PSampleStreamingSession, PSignalingMessage);
STATUS handleOffer(PSampleConfiguration, PSampleStreamingSession, PSignalingMessage);
STATUS handleRemoteCandidate(PSampleStreamingSession, PSignalingMessage);
STATUS initializePeerConnection(PSampleConfiguration, PRtcPeerConnection *);
STATUS lookForSslCert(PSampleConfiguration *);
STATUS createSampleStreamingSession(PSampleConfiguration, PCHAR, BOOL, PSampleStreamingSession *);
STATUS freeSampleStreamingSession(PSampleStreamingSession *);
STATUS streamingSessionOnShutdown(PSampleStreamingSession, UINT64, StreamSessionShutdownCallback);
STATUS sendSignalingMessage(PSampleStreamingSession, PSignalingMessage);
STATUS respondWithAnswer(PSampleStreamingSession);
STATUS resetSampleConfigurationState(PSampleConfiguration);
VOID sampleFrameHandler(UINT64, PFrame);
VOID sampleBandwidthEstimationHandler(UINT64, DOUBLE);
#ifdef ENABLE_DATA_CHANNEL
VOID onDataChannel(UINT64, PRtcDataChannel);
#endif
VOID onConnectionStateChange(UINT64, RTC_PEER_CONNECTION_STATE);
STATUS sessionCleanupWait(PSampleConfiguration);
STATUS awaitGetIceConfigInfoCount(SIGNALING_CLIENT_HANDLE, PUINT32);
STATUS logSignalingClientStats(PSignalingClientMetrics);
STATUS logSelectedIceCandidatesInformation(PSampleStreamingSession);
STATUS logStartUpLatency(PSampleConfiguration);
INT32 kvsWebRTCClientMaster(void);
uint64_t getEpochTimestampInHundredsOfNanos(void *);
#ifdef __cplusplus
}
#endif
#endif /* __KINESIS_VIDEO_SAMPLE_INCLUDE__ */
