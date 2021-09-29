#ifndef _SDIO_HOST_H
#define _SDIO_HOST_H
#include "basic_types.h"

#if defined(CONFIG_PLATFORM_8195A)
#include "rtl8195a_sdio_host.h"
#include "header8195b.h"
#endif

#if defined(CONFIG_PLATFORM_8195BHP)
#include "rtl8195bhp.h"
#include "rtl8195bhp_sdio_host.h"
#endif

#include "rtl8735b.h"
#include "rtl8735b_sdhost.h"

typedef enum {
	SDIO_INIT_NONE = -1,
	SDIO_INIT_FAIL = 0,
	SDIO_INIT_OK = 1,
	SDIO_SD_NONE = 2,
	SDIO_SD_INSERT = 3,
	SDIO_SD_OK = 4,
} _sdio_init_s;



uint32_t sdio_init_host(hal_sdhost_adapter_t *psdioh_adapter);	// init sdio host interface
void sdio_deinit_host(hal_sdhost_adapter_t *psdioh_adapter);

uint32_t sdio_sd_init(hal_sdhost_adapter_t *psdioh_adapter);	// init sd card through sdio
void sdio_sd_deinit(void);	//de-init sd card through sdio
uint32_t sdio_sd_status(hal_sdhost_adapter_t *psdioh_adapter);
u32 sdio_sd_getCapacity(void);
s8 sdio_sd_getProtection(void);
s8 sdio_sd_setProtection(bool protection);
s8 sdio_sd_getCSD(u8 *CSD);
s8 sdio_sd_isReady(void);

#if defined(CONFIG_PLATFORM_8195A)
s8 sdio_sd_setClock(SD_CLK_FREQUENCY SDCLK);
#endif

uint32_t sdio_read_blocks(hal_sdhost_adapter_t *psdioh_adapter, u64 sector,  u16 count, u8 *buffer);
uint32_t sdio_write_blocks(hal_sdhost_adapter_t *psdioh_adapter, u64 sector, u16 count, const u8 *buffer);

#endif
