/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <bluetooth/bluetooth.h>
#include <hci_if_zephyr.h>
#include <shell/shell.h>
#include <ctype.h>

extern unsigned char g_log_format;
extern unsigned char g_log_level;

static int cmd_priv_set_dbg(const struct shell *shell, size_t argc, char *argv[])
{
    int err = 0;
    char* level_str[] = {
        "LOG_LEVEL_NONE", 
        "LOG_LEVEL_ERR", 
        "LOG_LEVEL_WRN", 
        "LOG_LEVEL_INF", 
        "LOG_LEVEL_DBG", 
        "LOG_LEVEL_ALL", 
    };

	if (argc < 2) {
        shell_error(shell, "Wrong argument Number");
		return -EINVAL;
    }

    err = bt_priv_set_dbg(atoi(argv[1]), atoi(argv[2]));
    if (err) {
        shell_help(shell);
        return -EINVAL;
    }

    shell_print(shell, "Set BT Debug Level to: %s", level_str[g_log_level]);
    shell_print(shell, "Set BT Debug Format to: %s%s%s%s ", 
        (g_log_format&0x08)?"[tag]":"",    (g_log_format&0x04)?"[level]":"", 
        (g_log_format&0x02)?"[module]":"", (g_log_format&0x01)?"[function]":"");  

	return 0;
}

static int cmd_priv_bt_drv(const struct shell *shell, size_t argc, char *argv[])
{
    int err = 0;
    int drv_only = 0;

	if (argc < 2) {
        shell_error(shell, "Wrong argument Number");
		return -EINVAL;
    }

	if (!strcmp(argv[1], "on")) {
        if (argc >= 3)
            drv_only = *argv[2] - '0';

        err = bt_priv_set_drv(1, drv_only);
        if (err) {
            shell_error(shell, "Bluetooth init failed (err %d)", err);
            return err;
        }

        shell_print(shell, "Bluetooth init OK");
	} else if (!strcmp(argv[1], "off")) {
        err = bt_priv_set_drv(0, 0);
        if (err) {
            shell_error(shell, "Bluetooth deinit failed (err %d)", err);
            return err;
        }
        shell_print(shell, "Bluetooth deinit OK");
	} else {
		shell_error(shell, "Invalid argument: %s", argv[1]);
		return -EINVAL;
	}
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(priv_cmds,
	SHELL_CMD_ARG(set_dbg, NULL, "<level: 0-5> <format>. Smaller level, less log.", cmd_priv_set_dbg, 1, 0),
	SHELL_CMD_ARG(bt_drv,  NULL, "<on, off> <drv_only 0-1>", cmd_priv_bt_drv, 1, 0),
	SHELL_SUBCMD_SET_END
);

