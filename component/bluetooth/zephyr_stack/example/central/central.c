/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

static void start_scan(void);

static struct bt_conn *default_conn = NULL;
static char target_addr[6] = {0};
static uint8_t target_type = 0xff;
static struct bt_conn* central_conn[CONFIG_BT_MAX_CONN] = {0};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

#if 1
	/* Check target type and addr before connect */
	if ((target_type != addr->type) || 
		(memcmp(target_addr, addr->a.val, 6)))
		return;
#else
	/* connect only to devices in close proximity */
	if (rssi < -70)
		return;
#endif
	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
		start_scan();
		return;
	}

	printk("Create conn to %s OK\n", addr_str);
}

static void start_scan(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void stop_scan(void)
{
	int err;

	err = bt_le_scan_stop();
	if (err) {
		printk("Scanning failed to stop (err %d)\n", err);
		return;
	}

	printk("Scanning successfully stopped\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

    for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
        if (0 == central_conn[i]) {
            central_conn[i] = conn;
            break;
        }
    }

	printk("Connected: %s\n", addr);

    /* Start pairing */
    //bt_conn_set_security(conn, BT_SECURITY_L2);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	//start_scan();
}

static struct bt_conn_cb conn_callbacks = {
		.connected = connected,
		.disconnected = disconnected,
};

static int ze_util_ascii_to_hex(char* str)
{
    //char2hex
    char* ch = str;

    while (*ch) {
        if ('0' <= *ch && *ch <= '9')
            *ch -= '0';
        else if ('a' <= *ch && *ch <= 'f')
            *ch = *ch - 'a' + 10;
        else if ('A' <= *ch && *ch <= 'F')
            *ch = *ch - 'A' + 10;
        else
            return -1;

        ch++;
    }

    return 0;
}

int example_bt_ze_central(int argc, char **argv)
{
	int err = 0;

    if (argc == 1) {
        int en = atoi(argv[0]);
        if (en == 1) {
            err = bt_enable(NULL);
            if (err) {
                printk("Bluetooth init failed (err %d)\n", err);
                return err;
            }
            printk("Central Started!\n");
        } else if (en == 0) {
            stop_scan();
            bt_conn_cb_unregister(&conn_callbacks);
            bt_set_bondable(false);
            for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
                if (central_conn[i]) {
                    err = bt_conn_disconnect(central_conn[i], BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    if (err) {
                        printk("Disconnection failed (err %d)", err);
                        return err;
                    }
                    central_conn[i] = 0;
                }
            }
            printk("Central Stopped!\n");
        } else {
            printk("Wrong parameters!\n");
            return -EINVAL;
        }
    } else if (argc == 2) {
        char* type  = argv[0];

        if (!strcmp(type, "P")) {
            target_type = BT_ADDR_LE_PUBLIC;
        } else if (!strcmp(type, "R")) {
            target_type = BT_ADDR_LE_RANDOM;
        } else if (!strcmp(type, "PID")) {
            target_type = BT_ADDR_LE_PUBLIC_ID;
        } else if (!strcmp(type, "RID")) {
            target_type = BT_ADDR_LE_RANDOM_ID;
        } else {
            printk("Wrong addr type (%s)\n", type);
            return -EINVAL;
        }
        
        if (ze_util_ascii_to_hex(argv[1])) {
            printk("Wrong addr format\n");
            return -EINVAL;
        }
        
        for (char i = 0; i < 6; i++)
            target_addr[6-1-i] = argv[1][2*i]*16 + argv[1][2*i+1];
        
        printk("Try starting a central, then connect to [%02x:%02x:%02x:%02x:%02x:%02x]\n",
               target_addr[5], target_addr[4], target_addr[3], 
               target_addr[2], target_addr[1], target_addr[0]);

        bt_set_bondable(true);
        bt_conn_cb_register(&conn_callbacks);
        start_scan();
    } else {
        printk("Wrong parameters!\n");
        return -EINVAL;
    }

    return 0;
}
