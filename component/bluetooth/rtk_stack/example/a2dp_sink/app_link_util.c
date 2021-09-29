/*
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <string.h>

#include "os_mem.h"
//#include "remote.h"
#include "a2dp_sink_app_main.h"

uint32_t app_connected_profiles(void)
{
	uint32_t i, connected_profiles = 0;

	for (i = 0; i < MAX_BR_LINK_NUM; i++) {
		connected_profiles |= app_db.br_link[i].connected_profile;
	}
	return connected_profiles;
}


uint8_t app_connected_profile_link_num(uint32_t profile_mask)
{
	uint8_t i, link_number = 0;

	for (i = 0; i < MAX_BR_LINK_NUM; i++) {
		if (app_db.br_link[i].connected_profile & profile_mask) {
			link_number++;
		}

	}
	return link_number;
}

T_APP_BR_LINK *app_find_br_link(uint8_t *bd_addr)
{
	T_APP_BR_LINK *p_link = NULL;
	uint8_t        i;

	if (bd_addr != NULL) {
		for (i = 0; i < MAX_BR_LINK_NUM; i++) {
			if (app_db.br_link[i].used == true &&
				!memcmp(app_db.br_link[i].bd_addr, bd_addr, 6)) {
				p_link = &app_db.br_link[i];
				break;
			}
		}
	}

	return p_link;
}

T_APP_BR_LINK *app_alloc_br_link(uint8_t *bd_addr)
{
	T_APP_BR_LINK *p_link = NULL;
	uint8_t        i;

	if (bd_addr != NULL) {
		for (i = 0; i < MAX_BR_LINK_NUM; i++) {
			if (app_db.br_link[i].used == false) {
				p_link = &app_db.br_link[i];

				p_link->used = true;
				p_link->id   = i;
				memcpy(p_link->bd_addr, bd_addr, 6);
				break;
			}
		}
	}

	return p_link;
}

bool app_free_br_link(T_APP_BR_LINK *p_link)
{
	if (p_link != NULL) {
		if (p_link->used == true) {
			memset(p_link, 0, sizeof(T_APP_BR_LINK));
			return true;
		}
	}

	return false;
}
