/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#ifndef _ZEPHYR_BT_EXAMPLE_ENTRY_H_
#define _ZEPHYR_BT_EXAMPLE_ENTRY_H_

#if defined(CONFIG_PLATFORM_8710C) && defined(CONFIG_FTL_ENABLED)
void app_ftl_init(void);
#endif

#endif