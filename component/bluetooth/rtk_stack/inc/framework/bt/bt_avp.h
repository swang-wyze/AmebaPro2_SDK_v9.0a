/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */


#ifndef _BT_AVP_H_
#define _BT_AVP_H_

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef enum {
	BT_AVP_BUD_IN_EAR,
	BT_AVP_BUD_OUT_OF_CASE,
	BT_AVP_BUD_IN_CASE,
} T_BT_AVP_BUD_LOCATION;

typedef enum {
	BT_AVP_CONTROL_NONE                 = 0x00,
	BT_AVP_CONTROL_SIRI                 = 0x01,
	BT_AVP_CONTROL_PLAY_PAUSE           = 0x02,
	BT_AVP_CONTROL_FORWARD              = 0x03,
	BT_AVP_CONTROL_BACKWARD             = 0x04,
} T_BT_AVP_CONTROL;

typedef enum {
	BT_AVP_ANC_CLOSE                  = 0x01,
	BT_AVP_ANC_OPEN                   = 0x02,
	BT_AVP_ANC_TRANSPARENCY_MODE      = 0x03,
} T_BT_AVP_ANC;

typedef enum {
	BT_AVP_MIC_AUTO                  = 0x00,
	BT_AVP_MIC_ALLWAYS_RIGHT         = 0x01,
	BT_AVP_MIC_ALLWAYS_LEFT          = 0x02,
} T_BT_AVP_MIC;

typedef enum {
	BT_AVP_CLICK_SPEED_DEFAULT        = 0x00,
	BT_AVP_CLICK_SPEED_SLOW           = 0x01,
	BT_AVP_CLICK_SPEED_SLOWEST        = 0x02,
} T_BT_AVP_CLICK_SPEED;

typedef enum {
	BT_AVP_LONG_PRESS_TIME_DEFAULT    = 0x00,
	BT_AVP_LONG_PRESS_TIME_SHORT      = 0x01,
	BT_AVP_LONG_PRESS_TIME_SHORTEST   = 0x02,
} T_BT_AVP_LONG_PRESS_TIME;

typedef enum {
	BT_AVP_EVENT_SET_NAME                          = 0x01,
	BT_AVP_EVENT_CONTROL_SETTINGS                  = 0x02,
	BT_AVP_EVENT_IN_EAR_DETECTION                  = 0x03,
	BT_AVP_EVENT_MIC_SETTINGS                      = 0x04,
	BT_AVP_EVENT_ANC_SETTINGS                      = 0x05,
	BT_AVP_EVENT_CLICK_SETTINGS                    = 0x06,
	BT_AVP_EVENT_COMPACTNESS_TEST                  = 0x07,
	BT_AVP_EVENT_CLICK_SPEED                       = 0x08,
	BT_AVP_EVENT_LONG_RESS_TIME                    = 0x09,
	BT_AVP_EVENT_ONE_BUD_ANC                       = 0x0a,
	BT_AVP_EVENT_CONN_CMPL                         = 0x0b,
	BT_AVP_EVENT_VERSION_SYNC                      = 0x0c,
	BT_AVP_EVENT_VOICE_RECOGNITION_START           = 0x0d,
	BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_START    = 0x0e,
	BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_STOP     = 0x0f,
	BT_AVP_EVENT_VOICE_RECOGNITION_STOP            = 0x10,
	BT_AVP_EVENT_VOICE_RECOGNITION_ERROR           = 0x11,
	BT_AVP_EVENT_DISCONN_CMPL                      = 0x12,
	BT_AVP_EVENT_GATT_CONN_CMPL                    = 0x13,
	BT_AVP_EVENT_GATT_DISCONN_CMPL                 = 0x14,
} T_BT_AVP_EVENT;

typedef struct {
	uint8_t bd_addr[6];
	uint16_t len;
	uint8_t *data;
} T_BT_AVP_EVENT_SET_NAME;

typedef struct {
	uint8_t bd_addr[6];
	T_BT_AVP_CONTROL left_ear_control;
	T_BT_AVP_CONTROL right_ear_control;
} T_BT_AVP_EVENT_CONTROL_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	bool open;
} T_BT_AVP_EVENT_IN_EAR_DETECTION;

typedef struct {
	uint8_t bd_addr[6];
	T_BT_AVP_MIC setting;
} T_BT_AVP_EVENT_MIC_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	T_BT_AVP_ANC setting;
} T_BT_AVP_EVENT_ANC_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	bool left_ear_control_anc;
	bool right_ear_control_anc;
	uint8_t setting;
} T_BT_AVP_EVENT_CLICK_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_COMPACTNESS_TEST;

typedef struct {
	uint8_t bd_addr[6];
	T_BT_AVP_CLICK_SPEED speed;
} T_BT_AVP_EVENT_CLICK_SPEED_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	T_BT_AVP_LONG_PRESS_TIME time;
} T_BT_AVP_EVENT_LONG_PRESS_TIME_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	bool is_open;
} T_BT_AVP_EVENT_ONE_BUD_ANC;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_CONN_CMPL;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VERSION_SYNC;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOICE_RECOGNITION_START;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_START;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_STOP;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOICE_RECOGNITION_STOP;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOICE_RECOGNITION_ERROR;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BALANCED_TONE_SLIGHT;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BALANCED_TONE_MODERATE;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BALANCED_TONE_STRONG;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOCAL_RANGE_SLIGHT;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOCAL_RANGE_MODERATE;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_VOCAL_RANGE_STRONG;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BRIGHTNESS_SLIGHT;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BRIGHTNESS_MODERATE;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_BRIGHTNESS_STRONG;

typedef struct {
	uint8_t bd_addr[6];
	uint32_t left_ear_gian;
	uint32_t right_ear_gian;
} T_BT_AVP_EVENT_APT_GAIN_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
	uint32_t left_ear_tone;
	uint32_t right_ear_tone;
} T_BT_AVP_EVENT_APT_TONE_SETTINGS;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_DISCONN_CMPL;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_GATT_CONN_CMPL;

typedef struct {
	uint8_t bd_addr[6];
} T_BT_AVP_EVENT_GATT_DISCONN_CMPL;

typedef union {
	T_BT_AVP_EVENT_SET_NAME                         set_name;
	T_BT_AVP_EVENT_CONTROL_SETTINGS                 control_settings;
	T_BT_AVP_EVENT_IN_EAR_DETECTION                 in_ear_detection;
	T_BT_AVP_EVENT_MIC_SETTINGS                     mic_settings;
	T_BT_AVP_EVENT_ANC_SETTINGS                     anc_settings;
	T_BT_AVP_EVENT_CLICK_SETTINGS                   click_settings;
	T_BT_AVP_EVENT_COMPACTNESS_TEST                 compactness_test;
	T_BT_AVP_EVENT_CLICK_SPEED_SETTINGS             click_speed_settings;
	T_BT_AVP_EVENT_LONG_PRESS_TIME_SETTINGS         long_press_time_settings;
	T_BT_AVP_EVENT_ONE_BUD_ANC                      one_bud_anc;
	T_BT_AVP_EVENT_CONN_CMPL                        conn_cmpl;
	T_BT_AVP_EVENT_VERSION_SYNC                     version_sync;
	T_BT_AVP_EVENT_VOICE_RECOGNITION_START          voice_recognition_start;
	T_BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_START   voice_recognition_encode_start;
	T_BT_AVP_EVENT_VOICE_RECOGNITION_ENCODE_STOP    voice_recognition_encode_stop;
	T_BT_AVP_EVENT_VOICE_RECOGNITION_STOP           voice_recognition_stop;
	T_BT_AVP_EVENT_VOICE_RECOGNITION_ERROR          voice_recognition_error;
	T_BT_AVP_EVENT_BALANCED_TONE_SLIGHT             balanced_tone_slight;
	T_BT_AVP_EVENT_BALANCED_TONE_MODERATE           balanced_tone_moderate;
	T_BT_AVP_EVENT_BALANCED_TONE_STRONG             balanced_tone_strong;
	T_BT_AVP_EVENT_VOCAL_RANGE_SLIGHT               vocal_range_slight;
	T_BT_AVP_EVENT_VOCAL_RANGE_MODERATE             vocal_range_moderate;
	T_BT_AVP_EVENT_VOCAL_RANGE_STRONG               vocal_range_strong;
	T_BT_AVP_EVENT_BRIGHTNESS_SLIGHT                brightness_slight;
	T_BT_AVP_EVENT_BRIGHTNESS_MODERATE              brightness_moderate;
	T_BT_AVP_EVENT_BRIGHTNESS_STRONG                brightness_strong;
	T_BT_AVP_EVENT_APT_GAIN_SETTINGS                apt_gain_settings;
	T_BT_AVP_EVENT_APT_TONE_SETTINGS                apt_tone_settings;
	T_BT_AVP_EVENT_DISCONN_CMPL                     disconn_cmpl;
	T_BT_AVP_EVENT_GATT_CONN_CMPL                   gatt_conn_cmpl;
	T_BT_AVP_EVENT_GATT_DISCONN_CMPL                gatt_disconn_cmpl;
} T_BT_AVP_EVENT_PARAM;

typedef void (* P_BT_AVP_CBACK)(T_BT_AVP_EVENT event_type, void *event_buf, uint16_t buf_len);

bool bt_avp_init(P_BT_AVP_CBACK cback);

bool bt_avp_connect_req(uint8_t *bd_addr);

bool bt_avp_disconnect_req(uint8_t *bd_addr);

bool bt_att_connect_req(uint8_t *bd_addr);

bool bt_att_disconnect_req(uint8_t *bd_addr);

bool bt_avp_send_data(uint8_t *bd_addr, uint8_t *p_data, uint16_t data_len, bool flushable);

bool bt_avp_bud_location_report(uint8_t *bd_addr, T_BT_AVP_BUD_LOCATION right_ear_location,
								T_BT_AVP_BUD_LOCATION left_ear_location);

bool bt_avp_battery_level_report(uint8_t *bd_addr, uint8_t right_ear_level,
								 uint8_t right_ear_charging,
								 uint8_t left_ear_level, uint8_t left_ear_charging,
								 uint8_t case_level, uint8_t case_status);

bool bt_avp_anc_setting_report(uint8_t *bd_addr, T_BT_AVP_ANC anc_setting);

bool bt_avp_compactness_test_report(uint8_t *bd_addr, bool right_ear_result, bool left_ear_result);

bool bt_avp_voice_recognition_enable_req(uint8_t *bd_addr);

bool bt_avp_voice_recognition_disable_req(uint8_t *bd_addr);

bool bt_avp_voice_recognition_encode_start(uint8_t *bd_addr);

bool bt_avp_voice_recognition_encode_stop(uint8_t *bd_addr);

bool bt_avp_send_notification(uint8_t *bd_addr, uint8_t *p_data, uint16_t data_len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

#endif
