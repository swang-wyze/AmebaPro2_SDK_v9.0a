/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#include "trace.h"
#include "string.h"
#include "btm.h"
#include "bt_avrcp.h"
#include "bt_bond.h"
#include "app_link_util.h"
#include "app_avrcp.h"
#include "a2dp_sink_app_main.h"

#include "trace_app.h"
#include "ameba_soc.h"

#define VOL_MAX     0x7f
#define VOL_MIN     0x02

uint8_t curr_volume = 0x05;

static void app_avrcp_bt_cback(T_BT_EVENT event_type, void *event_buf, uint16_t buf_len)
{
	T_BT_EVENT_PARAM *param = event_buf;
	T_APP_BR_LINK *p_link;
	bool handle = true;

	switch (event_type) {
	case BT_EVENT_AVRCP_CONN_IND: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_CONN_IND");
		p_link = app_find_br_link(param->avrcp_conn_ind.bd_addr);
		if (p_link != NULL) {
			bt_avrcp_connect_cfm(p_link->bd_addr, true);
		}
	}
	break;

	case BT_EVENT_AVRCP_VOLUME_CHANGED: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_VOLUME_CHANGED");
		p_link = app_find_br_link(param->avrcp_volume_changed.bd_addr);

		if (p_link != NULL) {
			if (param->avrcp_volume_changed.volume > VOL_MAX) {
				curr_volume = VOL_MAX;
			} else if (param->avrcp_volume_changed.volume < VOL_MIN) {
				curr_volume = VOL_MIN;
			} else {
				curr_volume = param->avrcp_volume_changed.volume;
			}
		}
	}
	break;

	case BT_EVENT_AVRCP_VOLUME_UP: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_VOLUME_UP");
		p_link = app_find_br_link(param->avrcp_volume_up.bd_addr);
		if (p_link != NULL) {
			if (curr_volume < VOL_MAX) {
				curr_volume++;
			} else {
				curr_volume = VOL_MAX;
			}
		}
	}
	break;

	case BT_EVENT_AVRCP_VOLUME_DOWN: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_VOLUME_DOWN");
		p_link = app_find_br_link(param->avrcp_volume_down.bd_addr);
		if (p_link != NULL) {
			if (curr_volume > VOL_MIN) {
				curr_volume--;
			} else {
				curr_volume = VOL_MIN;
			}
		}
	}
	break;

	case BT_EVENT_AVRCP_REG_VOLUME_CHANGED: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_REG_VOLUME_CHANGED");
		p_link = app_find_br_link(param->avrcp_volume_down.bd_addr);
		if (p_link != NULL) {

		}
	}
	break;

	case BT_EVENT_AVRCP_CONN_CMPL: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_CONN_CMPL");
	}
	break;

	case BT_EVENT_AVRCP_PLAY_STATUS_CHANGED: {
		APP_PRINT_INFO0("BT_EVENT_AVRCP_PLAY_STATUS_CHANGED");;
		p_link = app_find_br_link(param->avrcp_play_status_changed.bd_addr);
		if (p_link != NULL) {
			p_link->avrcp_play_status = param->avrcp_play_status_changed.play_status;
		}
	}
	break;

	default: {
		handle = false;
	}
	break;
	}

	if (handle == true) {
		APP_PRINT_INFO1("app_avrcp_bt_cback: event_type 0x%04x", event_type);
	}
}

void app_avrcp_init(void)
{
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "app_avrcp_init\n");

	bt_avrcp_init(2);
	bt_mgr_cback_register(app_avrcp_bt_cback);
}

