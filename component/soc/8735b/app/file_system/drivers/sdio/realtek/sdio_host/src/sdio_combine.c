#include "sdio_combine.h"

sdio_sd_init_combine_type      sdio_sd_host_init_combine = hal_sdhost_init_host;
sdio_sd_read_combine_type      sdio_sd_read_combine = hal_sdhost_read_data;
sdio_sd_write_combine_type     sdio_sd_write_combine = hal_sdhost_write_data;
deinit_combine_type            deinit_combine = hal_sdhost_deinit;
sdio_sd_card_combine_type      sdio_sd_card_combine = hal_sdhost_init_card;
hal_sdioh_speed_type           hal_sdioh_speed_combine = hal_sdhost_switch_bus_speed;
hal_sdioh_get_card_status_type hal_sdioh_get_card_status_combine = hal_sdhost_get_card_status;

void sdio_driver_init(void)
{
#ifdef CONFIG_PLATFORM_8735B
	sdio_sd_host_init_combine = hal_sdhost_init_host;
	sdio_sd_read_combine = hal_sdhost_read_data;
	sdio_sd_write_combine = hal_sdhost_write_data;
	deinit_combine = hal_sdhost_deinit;
	sdio_sd_card_combine = hal_sdhost_init_card;
	hal_sdioh_speed_combine = hal_sdhost_switch_bus_speed;
	hal_sdioh_get_card_status_combine = hal_sdhost_get_card_status;
#endif
}
