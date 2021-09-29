/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <kernel.h>
#include <btsnoop.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <hci_core.h>
#include <log.h>
#include <init.h>
#include <proxy.h>

/******************************************************************************
 * FOR INTERNAL PRIV
 */

unsigned char g_log_format = LOG_FORMAT_TAG|LOG_FORMAT_LEVEL;
unsigned char g_log_level  = LOG_LEVEL_ERR;

int bt_priv_set_dbg(uint8_t level, uint8_t format)
{
    if (level > LOG_LEVEL_ALL || format > 0x0f)
        return -EINVAL;

    g_log_level = level;
    g_log_format = format;

    return 0;
}

int bt_priv_set_drv(uint8_t enable, uint8_t drv_only)
{
    int err = -1;
    static int _drv_only = 0;
    _drv_only = drv_only;

    if (1 == enable) {
        err = hci_drv_init();
        if (err) {
            BT_ERR("hci_drv_init failed (err %d)", err);
            return err;
        }

        err = bt_uart_init();
        if (err) {
            BT_ERR("bt_uart_init failed (err %d)", err);
            return err;
        }

        if (!_drv_only) {
            err = bt_enable(NULL);
            if (err) {
                BT_ERR("bt_enable failed (err %d)", err);
                return err;
            }
        }
    } else if (0 == enable) {
        if (!_drv_only) {
            _drv_only = 0;
            err = bt_disable();
            if (err) {
                BT_ERR("bt_disable failed (err %d)", err);
                return err;
            }
        }

        hci_drv_stop();
        bt_uart_deinit();

        err = hci_drv_deinit();
        if (err) {
            BT_ERR("hci_drv_deinit failed (err %d)", err);
            return err;
        }
    } else {
        BT_ERR("No such operation to hci_drv!");
        return err;
    }
    
    return 0;
}

/******************************************************************************
 * FOR SYS_INIT
 */

#ifdef CONFIG_BT_BAS
extern const struct init_entry _CONCAT(__init_, bas_init);
#endif
#ifdef CONFIG_BT_HRS
extern const struct init_entry _CONCAT(__init_, hrs_init);
#endif
#ifdef CONFIG_BT_OTS
extern const struct init_entry _CONCAT(__init_, bt_gatt_ots_instances_prepare);
extern const struct init_entry _CONCAT(__init_, bt_gatt_ots_l2cap_init);
#endif
//extern const struct init_entry _CONCAT(__init_, bt_uart_init);
//extern const struct init_entry _CONCAT(__init_, bt_monitor_init);
//extern const struct init_entry _CONCAT(__init_, hci_driver_init);
//extern const struct init_entry _CONCAT(__init_, bt_rpmsg_init);
//extern const struct init_entry _CONCAT(__init_, bt_uc_init);

void bt_sys_init_internel(void)
{
#ifdef CONFIG_BT_BAS
    _CONCAT(__init_, bas_init).init(_CONCAT(__init_, bas_init).dev);
#endif
#ifdef CONFIG_BT_HRS
    _CONCAT(__init_, hrs_init).init(_CONCAT(__init_, hrs_init).dev);
#endif
#ifdef CONFIG_BT_OTS
    _CONCAT(__init_, bt_gatt_ots_instances_prepare).init(_CONCAT(__init_, bt_gatt_ots_instances_prepare).dev);
    _CONCAT(__init_, bt_gatt_ots_l2cap_init).init(_CONCAT(__init_, bt_gatt_ots_l2cap_init).dev);
#endif
}

/******************************************************************************
 * FOR HOST
 */
extern struct bt_dev bt_dev;
struct k_work_q k_sys_work_q;
static K_KERNEL_STACK_DEFINE(work_q_stack, CONFIG_BT_WORK_Q_STACK_SIZE);
k_msg_queue g_tx_message_queue;

#if !defined(CONFIG_BT_TINYCRYPT_ECC)
int default_CSPRNG(uint8_t *dst, unsigned int len)
{
	return !bt_rand(dst, len);
}
#endif /* CONFIG_BT_TINYCRYPT_ECC */

int bt_enable_dependence(void)
{
	int err = 0;

    bt_sys_init_internel();

#ifdef CONFIG_BT_HCI_BTSNOOP
	btsnoop_init();
#endif /* CONFIG_BT_HCI_BTSNOOP */

#ifndef CONFIG_USE_STATIC_MEM
	k_sys_q_init();
#endif

	struct k_work_queue_config cfg = {
		.name = "bt_work_q",
		.no_yield = IS_ENABLED(CONFIG_BT_WORK_Q_NO_YIELD),
	};

	k_work_queue_start(&k_sys_work_q, work_q_stack,
			    K_KERNEL_STACK_SIZEOF(work_q_stack),
			    K_PRIO_COOP(CONFIG_BT_WORK_Q_PRIO), &cfg);

	err = k_msg_init(&g_tx_message_queue, 100, sizeof(k_tx_message_t));

	return err;
}

static int bt_conn_deinit(void)
{
    int err = 0;

#if 0
    /* Deinitialize background scan */
    if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
        for (int i = 0; i < ARRAY_SIZE(acl_conns); i++) {
            struct bt_conn *conn = bt_conn_ref(&acl_conns[i]);

            if (!conn)
                continue;

#if !defined(CONFIG_BT_WHITELIST)
            if (atomic_test_bit(conn->flags, BT_CONN_AUTO_CONNECT))
                bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
#endif

            bt_conn_unref(conn);
        }
    }

    bt_l2cap_deinit();

    err = bt_smp_deinit();
    if (err)
        return err;

    bt_att_deinit();

    k_fifo_init(&free_tx);
#endif

    return 0;
}

static int bt_id_deinit(void)
{
    int err = 0;

	if (!bt_dev.id_count) {
		BT_DBG("No user identity. No need to deinit.");
        return 0;
	}

#if defined(CONFIG_BT_PRIVACY)
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&bt_dev.rpa_update, &sync);
#endif

    for (uint8_t id = 1; id < bt_dev.id_count; id++) {
        err = bt_id_delete(id);
        if (err < 0) {
            BT_ERR("Deleting ID %u failed (err %d)", id, err);
            return err;
        }
    }

    return 0;
}

static int hci_deinit(void)
{
    int err = 0;

    err = bt_id_deinit();
    if (err)
        return err;

    /* All down here may don't need to be deinit
     * because code reset them when init (for now)
     */
#if 0
#if defined(CONFIG_BT_HCI_VS_EXT)
    hci_vs_deinit();
#endif

    err = clear_event_mask();
    if (err)
        return err;

	if (BT_FEAT_BREDR(bt_dev.features)) {
		err = bt_br_deinit();
		if (err)
			return err;
	}

    err = le_deinit();
    if (err)
        return err;

    err = common_deinit();
    if (err)
        return err;
#endif

    return 0;
}

static int settings_unload(void)
{
    return 0;
}

int bt_deinit(void)
{
	int err = 0;

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		settings_unload();
		atomic_clear_bit(bt_dev.flags, BT_DEV_PRESET_ID);
	}

	if (IS_ENABLED(CONFIG_BT_CONN)) {
		err = bt_conn_deinit();
		if (err)
			return err;
	}

	err = hci_deinit();
	if (err)
		return err;

	return 0;
}

int bt_disable_dependence(void)
{
	int err = 0;

	k_work_queue_stop(&k_sys_work_q);

	k_msg_free(&g_tx_message_queue);

#ifndef CONFIG_USE_STATIC_MEM	
	k_sys_q_free();
#endif	

#ifdef CONFIG_BT_HCI_BTSNOOP
	btsnoop_deinit();
#endif /* CONFIG_BT_HCI_BTSNOOP */

	return err;
}

void bt_del_hci_tx_thread(void* ctx)
{
	ARG_UNUSED(ctx);

	k_msg_send(&g_tx_message_queue, NULL, BT_EVENT_CMD_TX, K_NO_WAIT);
}

#if !defined(CONFIG_BT_RECV_IS_RX_THREAD)
void bt_del_hci_rx_thread(void* ctx)
{
	struct net_buf* buf = (struct net_buf*)ctx;

	bt_buf_set_type(buf, 0xff);
	net_buf_put(&bt_dev.rx_queue, buf);
}
#endif

/******************************************************************************
 * FOR Compiling
 */
#if !defined(CONFIG_BT_SHELL) && !defined(CONFIG_BT_SHELL_PRIV)
int execute_shell_cmds(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    BT_ERR("NO shell cmds in BT Lib!\n\r");
    return 0;
}
#endif

/******************************************************************************
 * FOR Z_STRUCT_SECTION_FOREACH
 */
/* bt_l2cap_br_fixed_chan */
extern struct bt_l2cap_br_fixed_chan br_fixed_chan;
const struct bt_l2cap_br_fixed_chan* _CONCAT(_bt_l2cap_br_fixed_chan, _list)[] =
{
#if defined(CONFIG_BT_BREDR)
	&br_fixed_chan,
#endif
	NULL
};

/* bt_l2cap_fixed_chan */
extern struct bt_l2cap_fixed_chan att_fixed_chan;
extern struct bt_l2cap_fixed_chan le_fixed_chan;
extern struct bt_l2cap_fixed_chan smp_fixed_chan;
extern struct bt_l2cap_fixed_chan smp_br_fixed_chan;
const struct bt_l2cap_fixed_chan* _CONCAT(_bt_l2cap_fixed_chan, _list)[] =
{
#if defined(CONFIG_BT_CONN)
	&att_fixed_chan,
	&le_fixed_chan,
#endif
#if defined(CONFIG_BT_SMP)
	&smp_fixed_chan,
#endif
#if defined(CONFIG_BT_BREDR)
	&smp_br_fixed_chan,
#endif
	NULL
};

/* bt_gatt_service_static */
extern struct bt_gatt_service_static _1_gatt_svc;
extern struct bt_gatt_service_static _2_gap_svc;
extern struct bt_gatt_service_static bas;
extern struct bt_gatt_service_static dis_svc;
extern struct bt_gatt_service_static hrs_svc;
const struct bt_gatt_service_static* _CONCAT(_bt_gatt_service_static, _list)[] =
{
#if defined(CONFIG_BT_CONN)
	&_1_gatt_svc,
	&_2_gap_svc,
#endif
#if defined(CONFIG_BT_BAS)
	&bas,
#endif
#if defined(CONFIG_BT_DIS)
	&dis_svc,
#endif
#if defined(CONFIG_BT_HRS)
	&hrs_svc,
#endif
	NULL
};

/* settings_handler_static START */
#if defined(CONFIG_BT_MESH)
extern struct bt_mesh_app_key_cb _CONCAT(bt_mesh_app_key_cb_, app_key_evt);
const struct bt_mesh_app_key_cb* _CONCAT(_bt_mesh_app_key_cb, _list)[] =
{
    &_CONCAT(bt_mesh_app_key_cb_, app_key_evt),
	NULL
};

const struct bt_mesh_hb_cb* _CONCAT(_bt_mesh_hb_cb, _list)[] =
{
	NULL
};

//extern struct bt_mesh_friend_cb _CONCAT(bt_mesh_friend_cb_, friend_cb);
const struct bt_mesh_friend_cb* _CONCAT(_bt_mesh_friend_cb, _list)[] =
{
    //&_CONCAT(bt_mesh_friend_cb_, friend_cb),
	NULL
};

//extern struct bt_mesh_hb_cb _CONCAT(bt_mesh_hb_cb, xxx);

extern struct bt_mesh_lpn_cb _CONCAT(bt_mesh_lpn_cb, lpn_cb);
//extern struct bt_mesh_lpn_cb _CONCAT(bt_mesh_lpn_cb, _mesh_test);
const struct bt_mesh_lpn_cb* _CONCAT(_bt_mesh_lpn_cb, _list)[] =
{
    &_CONCAT(bt_mesh_lpn_cb, lpn_cb),
	NULL
};

extern struct bt_mesh_subnet_cb _CONCAT(bt_mesh_subnet_cb_, app_keys);
extern struct bt_mesh_subnet_cb _CONCAT(bt_mesh_subnet_cb_, beacon);
#if defined(CONFIG_BT_MESH_FRIEND)
extern struct bt_mesh_subnet_cb _CONCAT(bt_mesh_subnet_cb_, friend);
#endif
#if defined(CONFIG_BT_MESH_LOW_POWER)
extern struct bt_mesh_subnet_cb _CONCAT(bt_mesh_subnet_cb_, lpn);
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
extern struct bt_mesh_subnet_cb _CONCAT(bt_mesh_subnet_cb_, proxy);
#endif
const struct bt_mesh_subnet_cb* _CONCAT(_bt_mesh_subnet_cb, _list)[] =
{
    &_CONCAT(bt_mesh_subnet_cb_, app_keys),
    &_CONCAT(bt_mesh_subnet_cb_, beacon),
#if defined(CONFIG_BT_MESH_FRIEND)
    &_CONCAT(bt_mesh_subnet_cb_, friend),
#endif
#if defined(CONFIG_BT_MESH_LOW_POWER)
    &_CONCAT(bt_mesh_subnet_cb_, lpn),
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
    &_CONCAT(bt_mesh_subnet_cb_, proxy),
#endif
    NULL
};

const struct bt_mesh_proxy_cb _CONCAT(_bt_mesh_proxy_cb, _list)[] = 
{
	NULL
};
#endif

/* settings_handler_static */
#ifdef CONFIG_BT_SETTINGS
extern struct settings_handler_static settings_handler_bt_dis;
extern struct settings_handler_static settings_handler_bt_mesh;
extern struct settings_handler_static settings_handler_bt_link_key;
extern struct settings_handler_static settings_handler_bt_keys;
extern struct settings_handler_static settings_handler_bt_ccc;
extern struct settings_handler_static settings_handler_bt_sc;
extern struct settings_handler_static settings_handler_bt_cf;
extern struct settings_handler_static settings_handler_bt_hash;
extern struct settings_handler_static settings_handler_bt;

const struct settings_handler_static* _CONCAT(_settings_handler_static, _list)[] =
{
	&settings_handler_bt_keys,
#if defined(CONFIG_BT_SETTINGS) && defined(CONFIG_BT_DIS_SETTINGS)
	&settings_handler_bt_dis,
#endif
#if !IS_ENABLED(CONFIG_BT_SETTINGS_CCC_LAZY_LOADING)
	&settings_handler_bt_ccc,
#endif
	&settings_handler_bt_sc,
#if defined(CONFIG_BT_GATT_CACHING)
	&settings_handler_bt_cf,
	&settings_handler_bt_hash,
#endif
#if defined(CONFIG_BT_BREDR)
	&settings_handler_bt_link_key,
#endif
	&settings_handler_bt,
	&settings_handler_bt_mesh,
	NULL,
};
#endif
