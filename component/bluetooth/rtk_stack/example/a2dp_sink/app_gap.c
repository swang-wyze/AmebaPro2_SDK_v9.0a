/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <string.h>
#include "trace.h"
#include "gap.h"
#include "gap_bond_legacy.h"
#include "app_gap.h"
#include "a2dp_sink_app_main.h"
#include "remote.h"
#include "btm.h"
#include "trace_app.h"
#include "ameba_soc.h"

// const uint8_t null_addr[6] = {0};

static void app_gap_common_callback(uint8_t cb_type, void *p_cb_data)
{
	T_GAP_CB_DATA cb_data;
	memcpy(&cb_data, p_cb_data, sizeof(T_GAP_CB_DATA));
	APP_PRINT_INFO1("app_gap_common_callback: cb_type = %d", cb_type);
	return;
}

static void app_gap_bt_cback(T_BT_EVENT event_type, void *event_buf, uint16_t buf_len)
{
	T_BT_EVENT_PARAM *param = event_buf;

	switch (event_type) {
	case BT_EVENT_READY: {
		memcpy(app_db.factory_addr, param->ready.bd_addr, 6);
		APP_PRINT_INFO1("app_gap_bt_cback: bt_ready, bd_addr %b",
						TRACE_BDADDR(param->ready.bd_addr));
	}
	break;

	case BT_EVENT_USER_CONFIRMATION_REQ: {
		legacy_bond_user_cfm(param->user_confirmation_req.bd_addr, GAP_CFM_CAUSE_ACCEPT);
	}
	break;

	case BT_EVENT_ACL_LINK_STATUS:
		switch (param->acl_link_status.info->status) {
		case GAP_ACL_AUTHEN_SUCCESS: {
			if (app_alloc_br_link(param->acl_link_status.info->bd_addr)) {
				APP_PRINT_INFO0("link alloc success");
			} else {
				APP_PRINT_ERROR0("link alloc failed");
			}
		}
		break;

		case GAP_ACL_CONN_DISCONN: {
			T_APP_BR_LINK *plink = NULL;
			plink = app_find_br_link(param->acl_link_status.info->bd_addr);
			if (app_free_br_link(plink)) {
				APP_PRINT_INFO0("link delete success");
			} else {
				APP_PRINT_ERROR0("link delete failed");
			}
		}
		break;

		default:
			break;

		}
		break;

	case BT_EVENT_LINK_KEY_INFO: {
		bt_bond_key_set(param->link_key_info.bd_addr, param->link_key_info.link_key,
						param->link_key_info.key_type);
	}
	break;

	case BT_EVENT_LINK_KEY_REQ: {
		uint8_t link_key[16];
		T_BT_LINK_KEY_TYPE type;

		if (bt_bond_key_get(param->link_key_req.bd_addr, link_key, (uint8_t *)&type)) {
			bt_link_key_cfm(param->link_key_req.bd_addr, true, type, link_key);
		} else {
			bt_link_key_cfm(param->link_key_req.bd_addr, false, type, link_key);
		}
	}
	break;

	default:
		break;
	}
}

void app_gap_init(void)
{
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "app_gap_init\n");

	gap_register_app_cb(app_gap_common_callback);

	bt_mgr_cback_register(app_gap_bt_cback);
}
