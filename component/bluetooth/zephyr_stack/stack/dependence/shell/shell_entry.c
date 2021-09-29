/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <shell/shell.h>
#include <ctype.h>

#if defined(CONFIG_BT_SHELL_PRIV)
extern cmd_entry priv_cmds_table[];
#endif
#if defined(CONFIG_BT_SHELL)
#if defined(CONFIG_BT)
extern cmd_entry bt_cmds_table[];
#endif
#if defined(CONFIG_BT_CONN)
extern cmd_entry gatt_cmds_table[];
#endif
#if defined(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
extern cmd_entry l2cap_cmds_table[];
#endif
#if defined(CONFIG_BT_BREDR)
extern cmd_entry br_cmds_table[];
#endif
#if defined(CONFIG_BT_ISO)
extern cmd_entry iso_cmds_table[];
#endif
#if defined(CONFIG_BT_WHITELIST) && defined(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
extern cmd_entry whitelist_cmds_table[];
#endif
#if defined(CONFIG_BT_RFCOMM)
extern cmd_entry rfcomm_cmds_table[];
#endif
#if defined(CONFIG_BT_MESH) && defined(CONFIG_BT_MESH_SHELL)
extern cmd_entry mesh_cmds_table[];
#endif
#endif

static struct {
    const char* cmds_hint;
    const cmd_entry* cmds_table;
} shell_cmds_table[] = {
#if defined(CONFIG_BT_SHELL_PRIV)
    {"priv", priv_cmds_table},
#endif
#if defined(CONFIG_BT_SHELL)
#if defined(CONFIG_BT)
    {"bt", bt_cmds_table}, 
#endif
#if defined(CONFIG_BT_CONN)
    {"gatt", gatt_cmds_table},     
#endif
#if defined(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
    {"l2cap", l2cap_cmds_table},
#endif
#if defined(CONFIG_BT_BREDR)
    {"br", br_cmds_table}, 
#endif
#if defined(CONFIG_BT_ISO)
    {"iso", iso_cmds_table}, 
#endif
#if defined(CONFIG_BT_WHITELIST) && defined(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL)
    {"wl", whitelist_cmds_table}, 
#endif
#if defined(CONFIG_BT_RFCOMM)
    {"rfcomm", rfcomm_cmds_table}, 
#endif
#if defined(CONFIG_BT_MESH) && defined(CONFIG_BT_MESH_SHELL)
    {"mesh", mesh_cmds_table}, 
#endif
#endif
};

int execute_shell_cmds(int argc, char* argv[])
{
    struct shell inner_shell = {0, 0};
    struct shell_ctx inner_ctx = {0};
    int i = 0, j = 0, found = 0, err = 0;

    int table_count = sizeof(shell_cmds_table) / sizeof(shell_cmds_table[0]);

    for (j = 0; j < table_count; j++)
    {
        if(strcmp((const char *)argv[0], shell_cmds_table[j].cmds_hint) == 0) {
            if (strcmp((const char *)argv[1], "help") == 0)
            {
                found = 1;
                while (shell_cmds_table[j].cmds_table[i].name) {
                    shell_fprintf(NULL, SHELL_INFO, "%s\n\r", shell_cmds_table[j].cmds_table[i].name);
                    shell_fprintf(NULL, SHELL_INFO, "   %s %s\n\r\n\r", shell_cmds_table[j].cmds_table[i].name, 
                                  shell_cmds_table[j].cmds_table[i].help);
                    i++;
                }

                break;
            }

            while (shell_cmds_table[j].cmds_table[i].name)
            {
                if(strcmp((const char *)argv[1], (const char *)(shell_cmds_table[j].cmds_table[i].name)) == 0)
                {
                    inner_ctx.active_cmd.syntax = shell_cmds_table[j].cmds_table[i].name;
                    inner_ctx.active_cmd.help = shell_cmds_table[j].cmds_table[i].help;
                    inner_shell.ctx = &inner_ctx;
                    err = shell_cmds_table[j].cmds_table[i].handle(&inner_shell, argc-1, &argv[1]);
                    found = 1;
                    break;
                }
                i++;
            }
        }
    }

    if (!found) {
        err = -EINVAL;
        shell_fprintf(NULL, SHELL_ERROR, "NOT Found Shell Cmds!\n\r");
    }

    return err;
}

