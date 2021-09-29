/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */
#include "btm.h"
#include "app_link_util.h"
#include "bt_a2dp.h"
#include "app_a2dp.h"
#include "trace.h"
#include <string.h>
#include "a2dp_sink_app_main.h"

#include "trace_app.h"
#include "ameba_soc.h"

uint8_t stream_data[512];

#include "a2dp_sink_entity.h"
#include "audio_framework_api.h"
extern void audio_framework_app_main(uint8_t audio_type, audio_entity_register_func register_func);
extern bool rtk_a2dp_sink_profile_register(uint8_t type, PAUDIO_APP_ENTITY p_entity);
extern void audio_framework_app_main(uint8_t audio_type, audio_entity_register_func register_func);

static void app_a2dp_bt_cback(T_BT_EVENT event_type, void *event_buf, uint16_t buf_len)
{
	(void)buf_len;
	T_BT_EVENT_PARAM *param = event_buf;
	T_APP_BR_LINK *p_link;
	bool handle = true;

	switch (event_type) {
	case BT_EVENT_A2DP_CONN_IND: {
		p_link = app_find_br_link(param->a2dp_conn_ind.bd_addr);
		if (p_link != NULL) {
			bt_a2dp_connect_cfm(p_link->bd_addr, true);
			APP_PRINT_INFO0("A2DP p_link confirmed");
		} else {
			APP_PRINT_INFO0("A2DP p_link is NULL");
		}
	}
	break;

	case BT_EVENT_A2DP_CONFIG_CMPL: {
		p_link = app_find_br_link(param->a2dp_config_cmpl.bd_addr);
		if (p_link != NULL) {
			T_BT_EVENT_PARAM_A2DP_CONFIG_CMPL *cfg = &param->a2dp_config_cmpl;
			p_link->a2dp_codec_type = param->a2dp_config_cmpl.codec_type;

			if (p_link->a2dp_codec_type == BT_A2DP_CODEC_TYPE_SBC) {
				APP_PRINT_INFO0("A2DP config: SBC ");
				p_link->a2dp_codec_info.sbc.sampling_frequency = cfg->codec_info.sbc.sampling_frequency;
				p_link->a2dp_codec_info.sbc.channel_mode = cfg->codec_info.sbc.channel_mode;
				p_link->a2dp_codec_info.sbc.block_length = cfg->codec_info.sbc.block_length;
				p_link->a2dp_codec_info.sbc.subbands = cfg->codec_info.sbc.subbands;
				p_link->a2dp_codec_info.sbc.allocation_method = cfg->codec_info.sbc.allocation_method;
				p_link->a2dp_codec_info.sbc.min_bitpool = cfg->codec_info.sbc.min_bitpool;
				p_link->a2dp_codec_info.sbc.max_bitpool = cfg->codec_info.sbc.max_bitpool;
				audio_framework_app_main(RTK_A2DP_SINK, rtk_a2dp_sink_profile_register);
			} else if (p_link->a2dp_codec_type == BT_A2DP_CODEC_TYPE_AAC) {
				APP_PRINT_INFO0("A2DP config: AAC ");
				p_link->a2dp_codec_info.aac.object_type = cfg->codec_info.aac.object_type;
				p_link->a2dp_codec_info.aac.sampling_frequency = cfg->codec_info.aac.sampling_frequency;
				p_link->a2dp_codec_info.aac.channel_number = cfg->codec_info.aac.channel_number;
				p_link->a2dp_codec_info.aac.vbr_supported = cfg->codec_info.aac.vbr_supported;
				p_link->a2dp_codec_info.aac.bit_rate = cfg->codec_info.aac.bit_rate;
			} else {
				APP_PRINT_INFO0("A2DP config: Vendor ");
				memcpy(p_link->a2dp_codec_info.vendor.info, cfg->codec_info.vendor.info, 12);
			}
		}
	}
	break;

	case BT_EVENT_A2DP_STREAM_OPEN: {
		APP_PRINT_INFO0("A2DP STREAM is opened ");
	}
	break;

	case BT_EVENT_A2DP_STREAM_START_IND: {
		if ((app_db.br_link[0].streaming_fg == false ||
			 app_db.br_link[0].avrcp_play_status == BT_AVRCP_PLAY_STATUS_PAUSED) ||
			(memcmp(app_db.br_link[0].bd_addr, param->a2dp_stream_start_ind.bd_addr, 6) == 0)) {
			APP_PRINT_INFO2("app_a2dp_bt_cback: BT_EVENT_A2DP_STREAM_START_IND active_a2dp_idx %d, streaming_fg %d",
							0, app_db.br_link[0].streaming_fg);
			bt_a2dp_stream_start_cfm(param->a2dp_stream_start_ind.bd_addr, true);
		}
	}
	break;

	case BT_EVENT_A2DP_STREAM_START_RSP: {
		if (app_db.br_link[0].streaming_fg == false ||
			(memcmp(app_db.br_link[0].bd_addr,
					param->a2dp_stream_start_ind.bd_addr, 6) == 0)) {
			APP_PRINT_INFO2("app_a2dp_bt_cback: BT_EVENT_A2DP_STREAM_START_IND active_a2dp_idx %d, streaming_fg %d",
							0, app_db.br_link[0].streaming_fg);
		}
	}
	break;

	case BT_EVENT_A2DP_STREAM_STOP: {
		if (memcmp(app_db.br_link[0].bd_addr,
				   param->a2dp_stream_stop.bd_addr, 6) == 0) {
			APP_PRINT_INFO0("BT_EVENT_A2DP_STREAM_STOP");
		}
	}
	break;

	case BT_EVENT_A2DP_STREAM_CLOSE: {
		if (memcmp(app_db.br_link[0].bd_addr,
				   param->a2dp_stream_close.bd_addr, 6) == 0) {
			APP_PRINT_INFO0("BT_EVENT_A2DP_STREAM_CLOSE");
		}
	}
	break;

	case BT_EVENT_A2DP_STREAM_DATA_IND: {
		APP_PRINT_INFO1("stream data len %d %d", param->a2dp_stream_data_ind.len);
		if (param->a2dp_stream_data_ind.len <= 512) {
			// memcpy(stream_data, param->a2dp_stream_data_ind.payload, param->a2dp_stream_data_ind.len);
			//TODO decode
			rtk_audio_data_in(RTK_A2DP_SINK, param->a2dp_stream_data_ind.payload, param->a2dp_stream_data_ind.len, NULL);
		}
	}
	break;

	default: {
		handle = false;
	}
	break;
	}

	if (handle == true) {
		APP_PRINT_INFO1("app_a2dp_bt_cback: event_type 0x%04x", event_type);
	}
}

void app_a2dp_init(void)
{
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "app_a2dp_init\n");

	bt_a2dp_init(1, 180);
	bt_a2dp_role_set(BT_A2DP_ROLE_SNK);

	{
		T_BT_A2DP_STREAM_END_POINT sep;

		sep.codec_type = BT_A2DP_CODEC_TYPE_SBC;
		sep.u.codec_sbc.sampling_frequency_mask = 0x0f;
		sep.u.codec_sbc.channel_mode_mask = 0x0f;
		sep.u.codec_sbc.block_length_mask = 0x0f;
		sep.u.codec_sbc.subbands_mask = 0x03;
		sep.u.codec_sbc.allocation_method_mask = 0x03;
		sep.u.codec_sbc.min_bitpool = 2;
		sep.u.codec_sbc.max_bitpool = 53;
		bt_a2dp_stream_end_point_set(sep);
	}

	{
		T_BT_A2DP_STREAM_END_POINT sep;

		sep.codec_type = BT_A2DP_CODEC_TYPE_AAC;
		sep.u.codec_aac.object_type_mask = 0x80;
		sep.u.codec_aac.sampling_frequency_mask = 0x0180;
		sep.u.codec_aac.channel_number_mask = 0x0c;
		sep.u.codec_aac.vbr_supported = 1;
		sep.u.codec_aac.bit_rate = 0;
		bt_a2dp_stream_end_point_set(sep);
	}

	bt_mgr_cback_register(app_a2dp_bt_cback);
}
