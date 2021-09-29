#include "log_service.h"
#include "atcmd_bt_ze.h"
#include <platform_stdlib.h>
#include <platform_opts_bt.h>

#if (defined(CONFIG_BT_PERIPHERAL) && CONFIG_BT_PERIPHERAL)
char bt_at_cmd_buf[256] = {0};
#endif

#if (defined(CONFIG_BT_PERIPHERAL) && CONFIG_BT_PERIPHERAL)
int example_bt_ze_peripheral(int argc, char **argv);
static void fATBp(void *arg)
{
    int argc = 0;
    int param = 0;
    char *argv[MAX_ARGC] = {0};

    if (arg) {
        argc = parse_param(arg, argv);
    } else {
        goto exit;
    }

    if (argc != 2) {
        AT_PRINTK("[AT_PRINTK] ERROR: input parameter error!\n\r");
        goto exit;
    }

    if (example_bt_ze_peripheral(argc-1, &argv[1]))
        AT_PRINTK("[AT_PRINTK] ERROR: Start BLE Peripheral failed!\n\r");

    return;

exit:
    AT_PRINTK("[ATBp] Start BLE Peripheral: ATBp=1");
    AT_PRINTK("[ATBp] Stop  BLE Peripheral: ATBp=0 (NOT SUPPORT NOW!)");
}
#endif

#if (defined(CONFIG_BT_CENTRAL) && CONFIG_BT_CENTRAL)
void example_bt_ze_central(int argc, char **argv);
static void fATBc(void *arg)
{
	int argc = 0;
	int param = 0;
	char *argv[MAX_ARGC] = {0};

	if (!arg)
        goto exit;

    argc = parse_param(arg, argv);

    if (argc != 2) {
		AT_PRINTK("[AT_PRINTK] ERROR: input parameter error!\n\r");
		goto exit;
	}

    example_bt_ze_central(argc-1, &argv[1]);
	return;

exit:
	AT_PRINTK("[ATBc] Start BLE Central: ATBc=1");
	AT_PRINTK("[ATBc] Stop  BLE Central: ATBc=0");
}

static void fATBC(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	memset(bt_at_cmd_buf, 0, 256);

	if (arg) {
		strncpy(bt_at_cmd_buf, arg, sizeof(bt_at_cmd_buf));
		argc = parse_param(bt_at_cmd_buf, argv);
	} else {
		goto exit;
	}

	if (argc != 3) {
		AT_PRINTK("[AT_PRINTK] ERROR: input parameter error!\n\r");
		goto exit;
	}

	example_bt_ze_central(argc-1, &argv[1]);
	return;

exit:
	AT_PRINTK("[ATBC] Connect to remote device: ATBC=P/R,BLE_BD_ADDR");
	AT_PRINTK("[ATBC] P=public, R=random");
	AT_PRINTK("[ATBC] eg:ATBC=P,001122334455");
}

#endif

extern int execute_shell_cmds(int argc, char* argv[]);
static void fATBZ(void *arg)
{
    int argc = 0;
    int param = 0;
    char *argv[MAX_ARGC] = {0};

    memset(bt_at_cmd_buf, 0, 256);

    if (arg) {
        strncpy(bt_at_cmd_buf, arg, sizeof(bt_at_cmd_buf));
        argc = parse_param(bt_at_cmd_buf, argv);
    } else {
        goto exit;
    }

    if (argc < 2) {
        AT_PRINTK("[AT_PRINTK] ERROR: input parameter error!\n\r");
        goto exit;
    }

    if (execute_shell_cmds(argc-1, argv+1))
        goto exit;

    return;

exit:
    AT_PRINTK("[ATBZ]: Execute SHELL Wrong!");
    AT_PRINTK("[ATBZ]: If Usage Wrong! Use [ATBZ=help] to Check!");
}

static log_item_t at_bt_items[ ] = {
#if (defined(CONFIG_BT_PERIPHERAL) && CONFIG_BT_PERIPHERAL)
    {"ATBp", fATBp, {NULL, NULL}},
#endif
#if (defined(CONFIG_BT_CENTRAL) && CONFIG_BT_CENTRAL)
    {"ATBc", fATBc, {NULL, NULL}},
    {"ATBC", fATBC, {NULL, NULL}},
#endif
#if ((defined(CONFIG_BT) && CONFIG_BT))
    /* All Zephyr stack shell cmds will be gathed here */
    {"ATBZ", fATBZ, {NULL, NULL}}, 
#endif
};

void at_bt_init(void)
{
#if ((defined(CONFIG_BT) && CONFIG_BT))
	log_service_add_table(at_bt_items, sizeof(at_bt_items) / sizeof(at_bt_items[0]));
#endif
}

