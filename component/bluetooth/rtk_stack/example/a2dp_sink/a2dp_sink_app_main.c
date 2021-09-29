
/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <stdint.h>
#include <string.h>
#include "os_mem.h"
#include "os_sched.h"
#include "os_msg.h"
#include "os_task.h"
#include "os_sched.h"

#include "trace.h"

#include "gap_legacy.h"
#include "gap_bond_legacy.h"
#include "gap_timer.h"
#include "gap.h"

#include "sysm.h"
#include "trace_app.h"
#include "app_msg.h"

#include "a2dp_sink_app_main.h"
#include "app_gap.h"
#include "app_sdp.h"
#include "app_a2dp.h"
#include "app_avrcp.h"
#include "ameba_soc.h"
#include "hci_tp_dbg.h"
#include "timers.h"

#define MAX_NUMBER_OF_GAP_MESSAGE       0x20    //!< indicate BT stack message queue size
#define MAX_NUMBER_OF_IO_MESSAGE        0x40    //!< indicate io queue size, extra 0x20 for data uart
#define MAX_NUMBER_OF_GAP_TIMER         0x10    //!< indicate gap timer queue size
#define MAX_NUMBER_OF_DSP_MSG           0x20    //!< number of dsp message reserved for DSP message handling.
#define MAX_NUMBER_OF_CODEC_MSG         0x20    //!< number of codec message reserved for CODEC message handling.
#define MAX_NUMBER_OF_SYS_MSG           0x20    //!< indicate SYS timer queue size

/** indicate rx event queue size*/
#define MAX_NUMBER_OF_RX_EVENT      \
    (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE  +  MAX_NUMBER_OF_DSP_MSG + MAX_NUMBER_OF_CODEC_MSG + MAX_NUMBER_OF_GAP_TIMER + MAX_NUMBER_OF_SYS_MSG)

#define DEFAULT_PAGESCAN_WINDOW             0x48
#define DEFAULT_PAGESCAN_INTERVAL           0x800 //0x800
#define DEFAULT_PAGE_TIMEOUT                0x2000
#define DEFAULT_SUPVISIONTIMEOUT            0x1f40 //0x7D00
#define DEFAULT_INQUIRYSCAN_WINDOW          0x48
#define DEFAULT_INQUIRYSCAN_INTERVAL        0x800 //0x1000

void *audio_evt_queue_handle;
void *audio_io_queue_handle;

T_APP_DB app_db;

static void bt_gap_init(void)
{
	uint8_t device_name_legacy[40] = "12345";
	uint32_t class_of_device = 0x0C025A;
	uint16_t supervision_timeout = DEFAULT_SUPVISIONTIMEOUT;

	uint16_t link_policy = GAP_LINK_POLICY_ROLE_SWITCH | GAP_LINK_POLICY_SNIFF_MODE;

	uint8_t radio_mode = GAP_RADIO_MODE_VISIABLE_CONNECTABLE;
	bool limited_discoverable = false;
	bool auto_accept_acl = true;

	uint8_t pagescan_type = GAP_PAGE_SCAN_TYPE_INTERLACED;
	uint16_t pagescan_interval = DEFAULT_PAGESCAN_INTERVAL;
	uint16_t pagescan_window = DEFAULT_PAGESCAN_WINDOW;
	uint16_t page_timeout = DEFAULT_PAGE_TIMEOUT;

	uint8_t inquiryscan_type = GAP_INQUIRY_SCAN_TYPE_INTERLACED;
	uint16_t inquiryscan_window = DEFAULT_INQUIRYSCAN_WINDOW;
	uint16_t inquiryscan_interval = DEFAULT_INQUIRYSCAN_INTERVAL;
	uint8_t inquiry_mode = GAP_INQUIRY_MODE_EXTENDED_RESULT;

	uint8_t pair_mode = GAP_PAIRING_MODE_PAIRABLE;
	uint16_t auth_flags = GAP_AUTHEN_BIT_GENERAL_BONDING_FLAG | GAP_AUTHEN_BIT_SC_FLAG;
	uint8_t io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
	uint8_t oob_enable = false;
	uint8_t bt_mode = GAP_BT_MODE_21ENABLED;
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "bt_gap_init\n");
	legacy_gap_init();

	//0: to be master
	legacy_cfg_accept_role(1);

	legacy_set_gap_param(GAP_PARAM_LEGACY_NAME, GAP_DEVICE_NAME_LEN, device_name_legacy);

	gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(uint8_t), &pair_mode);
	gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(uint16_t), &auth_flags);
	gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(uint8_t), &io_cap);
	gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(uint8_t), &oob_enable);

	legacy_set_gap_param(GAP_PARAM_BT_MODE, sizeof(uint8_t), &bt_mode);
	legacy_set_gap_param(GAP_PARAM_COD, sizeof(uint32_t), &class_of_device);
	legacy_set_gap_param(GAP_PARAM_LINK_POLICY, sizeof(uint16_t), &link_policy);
	legacy_set_gap_param(GAP_PARAM_SUPV_TOUT, sizeof(uint16_t), &supervision_timeout);
	legacy_set_gap_param(GAP_PARAM_AUTO_ACCEPT_ACL, sizeof(bool), &auto_accept_acl);


	legacy_set_gap_param(GAP_PARAM_RADIO_MODE, sizeof(uint8_t), &radio_mode);
	legacy_set_gap_param(GAP_PARAM_LIMIT_DISCOV, sizeof(bool), &limited_discoverable);

	legacy_set_gap_param(GAP_PARAM_PAGE_SCAN_TYPE, sizeof(uint8_t), &pagescan_type);
	legacy_set_gap_param(GAP_PARAM_PAGE_SCAN_INTERVAL, sizeof(uint16_t), &pagescan_interval);
	legacy_set_gap_param(GAP_PARAM_PAGE_SCAN_WINDOW, sizeof(uint16_t), &pagescan_window);
	legacy_set_gap_param(GAP_PARAM_PAGE_TIMEOUT, sizeof(uint16_t), &page_timeout);

	legacy_set_gap_param(GAP_PARAM_INQUIRY_SCAN_TYPE, sizeof(uint8_t), &inquiryscan_type);
	legacy_set_gap_param(GAP_PARAM_INQUIRY_SCAN_INTERVAL, sizeof(uint16_t), &inquiryscan_interval);
	legacy_set_gap_param(GAP_PARAM_INQUIRY_SCAN_WINDOW, sizeof(uint16_t), &inquiryscan_window);
	legacy_set_gap_param(GAP_PARAM_INQUIRY_MODE, sizeof(uint8_t), &inquiry_mode);

}

static void framework_init(void)
{
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "framework_init\n");
	/* System Manager */
	sys_mgr_init(audio_evt_queue_handle);

	/* Initialize remote control manager*/
	remote_mgr_init(REMOTE_SESSION_ROLE_SINGLE);

	/* Bluetooth Manager */
	bt_mgr_init();

	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "framework_init ok\n");
}

static void app_task(void *pvParameters)
{
	(void)pvParameters;
	uint8_t event;

	gap_start_bt_stack(audio_evt_queue_handle, audio_io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

	APP_PRINT_TRACE4("app_task: data on %d, data off %d, buf on %d, buf off %d",
					 os_mem_peek(RAM_TYPE_DATA_ON), os_mem_peek(RAM_TYPE_DATA_OFF),
					 os_mem_peek(RAM_TYPE_BUFFER_ON), os_mem_peek(RAM_TYPE_BUFFER_OFF));

	while (true) {
		if (os_msg_recv(audio_evt_queue_handle, &event, 0xFFFFFFFF) == true) {
			if (EVENT_GROUP(event) == EVENT_GROUP_IO) {
				T_IO_MSG io_msg;

				if (os_msg_recv(audio_io_queue_handle, &io_msg, 0) == true) {
					if (event == EVENT_IO_TO_APP) {
						//app_io_handle_msg(io_msg);
					}
				}
			} else if (EVENT_GROUP(event) == EVENT_GROUP_STACK) {
				gap_handle_msg(event);
			} else if (EVENT_GROUP(event) == EVENT_GROUP_FRAMEWORK) {
				sys_mgr_event_handle(event);
			}
		}
	}
}

int a2dp_sink_app_init(void)
{
	void *app_task_handle;
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "==>a2dp_sink_app_init\n");

	APP_PRINT_INFO2("APP COMPILE TIME: [%s - %s]", TRACE_STRING(__DATE__), TRACE_STRING(__TIME__));
	memset(&app_db, 0, sizeof(T_APP_DB));

	os_msg_queue_create(&audio_io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
	os_msg_queue_create(&audio_evt_queue_handle, MAX_NUMBER_OF_RX_EVENT, sizeof(unsigned char));

	gap_init_timer(audio_evt_queue_handle, MAX_NUMBER_OF_GAP_TIMER);

	bt_gap_init();
	framework_init();

	app_gap_init();

	app_avrcp_init();
	app_a2dp_init();
	app_sdp_init();

	os_task_create(&app_task_handle, "app_task", app_task, NULL, 1024 * 4, 1);
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "<==a2dp_sink_app_init\n");

	return 0;
}

extern void bt_trace_set_switch(bool flag);
extern bool bte_init(void);

int bt_app_main(void)
{
	DBG_PRINTF(MODULE_BOOT, LEVEL_INFO, "a2dp bt_app_main \n");

	// hci_tp_debug_set_level(HCI_TP_DEBUG_WARN);

	uint32_t tmp = 0;
	tmp = HAL_READ32(0x42008200, 0x50);
	tmp &= ~0x80000;
	HAL_WRITE32(0x42008200, 0x50, tmp);

	bt_trace_set_switch(true);
	bt_trace_init();
	bte_init();
	a2dp_sink_app_init();
#if 1
	while (1) {
		os_delay(20000);
		printf("bt_app_main, free heap3: %x \r\n", os_mem_peek(3));
	}
#endif
	os_task_delete(NULL);

	return 0;
}
