#include "platform_opts.h"
#include "platform_os.h"
#include "os_task.h"
#include <platform_stdlib.h>
#include <wifi_conf.h>
#include <osdep_service.h>
#include <platform_opts_bt.h>
#include "bt_config_wifi.h"

#include <gap_conn_le.h>

#if defined(CONFIG_BT_MESH_PROVISIONER_RTK_DEMO) && CONFIG_BT_MESH_PROVISIONER_RTK_DEMO

//media
extern void amebacam_broadcast_demo_thread(void *param);
//httpd
extern void httpd_demo_init_thread(void *param);
extern int bt_config_app_init(void);
extern uint8_t get_bt_config_state(void);

void bt_mesh_example_init_thread(void *param)
{
	/* avoid gcc compile warning */
	(void)param;

	/*Wait WIFI init complete*/
	while (!(wifi_is_running(WLAN0_IDX) || wifi_is_running(WLAN1_IDX))) {
		os_delay(1000);
	}
	/*Init BT config*/
	bt_config_app_init();
	/*Wait BT config complete*/
	while (get_bt_config_state() != BC_DEV_DISABLED) {
		os_delay(500);
	}

	os_task_delete(NULL);
}

void *httpd_demo_init_task = NULL;
void *amebacam_broadcast_demo_task = NULL;
void *bt_mesh_example_init_task = NULL;

void example_bt_mesh(void)
{
	//init httpd
	if (os_task_create(&httpd_demo_init_task, "httpd_demo_init_thread", httpd_demo_init_thread,
					   NULL, 2048, 1) != true) {
		printf("%s xTaskCreate(httpd_demo_init_thread) failed\r\n", __FUNCTION__);
	}
	//init amebacam_broadcast
	if (os_task_create(&amebacam_broadcast_demo_task, "amebacam_broadcast_demo_thread", amebacam_broadcast_demo_thread,
					   NULL, 2048, 1) != true) {
		printf("%s xTaskCreate(amebacam_broadcast_demo_thread) failed\r\n", __FUNCTION__);
	}
	//init bt config/bt mesh
	if (os_task_create(&bt_mesh_example_init_task, "bt_mesh_example_demo_init_thread", bt_mesh_example_init_thread,
					   NULL, 1024, 5) != true) {
		printf("%s xTaskCreate(bt_mesh_example_demo_init_thread) failed\r\n", __FUNCTION__);
	}
}

#endif /* CONFIG_EXAMPLE_BT_MESH_DEMO */


