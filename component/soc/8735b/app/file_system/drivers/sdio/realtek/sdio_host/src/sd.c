#include "sd.h"
#include <stdio.h>
#include "portable.h"
#define malloc  pvPortMalloc
#define free    vPortFree

#if CONFIG_SD_SDIO
#include "sdio_host.h"
#include "cmsis.h"
#include "sdio_combine.h"
#endif
#if CONFIG_SD_SPI
#include "sd_spi.h"
#endif

SD_RESULT SD_WaitReady(void)
{
	char ret = 0;

#if CONFIG_SD_SDIO
	// TODO:
#elif CONFIG_SD_SPI
	ret = SPI_SD_WaitReady();
#endif
	if (ret) {
		return SD_ERROR;
	}
	return SD_OK;
}
#define SD_DEFAULT_SPEED SdHostSpeedDS
//SdHostSpeedSDR25

hal_sdhost_adapter_t test_sdioh;
phal_sdhost_adapter_t psdioh_adapter = &test_sdioh;

u8 new_rdata[SD_BLK_LEN]__attribute__((aligned(32))); // use another memory to avoid cache issue
u8 new_wdata[SD_BLK_LEN]__attribute__((aligned(32))); // use another memory to avoid cache issue

__weak
void sdh_card_insert_callback(void *pdata)
{
	dbg_printf("[In]\r\n");
	for (int i = 0; i < 50000; i++) {
		asm("nop");
	}
}

__weak
void sdh_card_remove_callback(void *pdata)
{
	dbg_printf("[Out]\r\n");
	for (int i = 0; i < 50000; i++) {
		asm("nop");
	}
}

#if 0
enum sdioh_access_mode_e clk_mode;
int set_clk_flag = 0;
SD_RESULT SD_Set_Clk(enum sdioh_access_mode_e clk)
{
	clk_mode = clk;
	set_clk_flag = 1;
}
#endif

// Wrtie retry max count
int retry_max = 1;

SD_RESULT SD_Write_Threshold(int input_times)
{
	retry_max = input_times;
	return SD_OK;
}

SD_RESULT SD_Init(void)
{
	SD_RESULT ret = SD_OK;

#if CONFIG_SD_SDIO
	u8 sd_bus_speed_v33[] = {SdHostSpeedHS, SdHostSpeedDS};
	u8 sd_bus_speed_v18[] = {SdHostSpeedSDR50, SdHostSpeedSDR25, SdHostSpeedSDR12};
	u8 *sd_bus_speed_sel;
	u8 sd_bus_speed_count;
	ret = sdio_sd_host_init_combine(psdioh_adapter);
	if (ret) {
		return SD_INITERR;
	}
	ret = sdio_sd_card_combine(psdioh_adapter);
	if (ret) {
		return SD_INITERR;
	}
	if (psdioh_adapter->curr_sig_level == SdHostSigVol18) {
		sd_bus_speed_sel = sd_bus_speed_v18;
		sd_bus_speed_count = sizeof(sd_bus_speed_v18);
	} else {
		sd_bus_speed_sel = sd_bus_speed_v33;
		sd_bus_speed_count = sizeof(sd_bus_speed_v33);
	}
	// try to use max speed
	for (u8 sel = 0; sel < sd_bus_speed_count; sel++) {
		ret = hal_sdioh_speed_combine(psdioh_adapter, sd_bus_speed_sel[sel]);
		if (ret == HAL_OK) {
			break;
		}
	}
	if (ret != HAL_OK) {
		return SD_INITERR;
	}
#elif CONFIG_SD_SPI
	if (SPI_SD_Init() == 0) {
		// get sd write protection
		if (SPI_SD_GetProtection()) {
			ret = SD_PROTECTED;
		} else {
			ret = SD_OK;
		}
	} else {
		ret = SD_INITERR;
	}
#endif
	return SD_OK;
}

SD_RESULT SD_DeInit(void)
{
	SD_RESULT ret = SD_OK;

#if CONFIG_SD_SDIO
	deinit_combine(psdioh_adapter);
	ret = SD_OK;
#elif CONFIG_SD_SPI
	SPI_SD_DeInit();
	ret = SD_OK;
#endif
	return ret;
}
/*
SD_RESULT SD_SetCLK(SD_CLK CLK){
    SD_RESULT ret;
    char status;
#if CONFIG_SD_SDIO
    switch(CLK){
        case SD_CLK_HIGH:
            status = sdio_sd_setClock(SD_CLK_41_6MHZ);
            break;
        case SD_CLK_MID:
            status = sdio_sd_setClock(SD_CLK_20_8MHZ);
            break;
        case SD_CLK_LOW:
            status = sdio_sd_setClock(SD_CLK_10_4MHZ);
            break;
        case SD_CLK_RSV:
            status = sdio_sd_setClock(SD_CLK_5_2MHZ);
            break;
        default:
            status = -1;
    }
    if(status == 0)
        ret = SD_OK;
    else
        ret = SD_ERROR;
#endif
    return ret;
}
*/
SD_RESULT SD_Status(void)
{
	SD_RESULT ret = SD_OK;

#if CONFIG_SD_SDIO
	if (0 == psdioh_adapter->card_inited) {
		return SD_INITERR;
	}
	ret = hal_sdioh_get_card_status_combine(psdioh_adapter);
	if (ret != HAL_OK) {
		return SD_NODISK;
	}
#elif CONFIG_SD_SPI
	status = SPI_SD_IsReady();
	if (status == 0) {
		ret = SD_OK;
		// get sd write protection
		if (SPI_SD_GetProtection()) {
			ret = SD_PROTECTED;
		}
	} else {
		ret = SD_NODISK;
	}
#endif
	return ret;
}

SD_RESULT SD_GetCID(u8 *cid_data)
{
	// TODO
	return SD_OK;
}

SD_RESULT SD_GetCSD(u8 *csd_data)
{
	// TODO
	return SD_OK;
}


SD_RESULT SD_GetCapacity(u32 *sector_count)
{
	u32 block_capacity = 0;
#if CONFIG_SD_SDIO
	// CSD 2.0
	if (((sd_csd_v2_t *)test_sdioh.csd)->csd_structure == 1) {
		sd_csd_v2_t *csd = (sd_csd_v2_t *)test_sdioh.csd;
		block_capacity = (csd->c_size_1 | (csd->c_size_2 << 16)) * 1024;
	} else {
		sd_csd_v1_t *csd = (sd_csd_v1_t *)test_sdioh.csd;
		u32 block_nr, mult, block_len;
		mult = 1 << (csd->c_size_mult + 2);
		block_nr = mult * ((csd->c_size_1 | (csd->c_size_2 << 2)) + 1);
		block_len = 1 << csd->read_bl_len;
		block_capacity = block_nr * block_len / SD_BLK_LEN;
	}
	dbg_printf("Block capacity = %u blocks\r\n", block_capacity);

#elif CONFIG_SD_SPI
	block_capacity = SPI_SD_GetCapacity();
#endif

	*sector_count = block_capacity;

	if (*sector_count) {
		return SD_OK;
	}
	return SD_ERROR;
}

SD_RESULT SD_ReadBlocks(u32 sector, u8 *data, u32 count)
{
	SD_RESULT ret = SD_OK;
#if CONFIG_SD_SDIO
	u8 *buff_new = NULL, *buff_alignment = data;
	// FIXME too slow
	if (((u32)buff_alignment) % 32 != 0) { //check address 32 byte alignment for SDIO host
		buff_new = (u8 *) malloc(count * SD_BLK_LEN + 31);
		if (buff_new == NULL) {
			dbg_printf("Fail to malloc cache for SDIO host!!\n");
			return SD_ERROR;
		}
		buff_alignment = (u8 *)((u32)buff_new & (~(u32)31));
	}
	ret = sdio_sd_read_combine(psdioh_adapter, (u64)sector * SD_BLK_LEN, count,  buff_alignment);

	if (buff_new) {
		memcpy(data, buff_alignment, count * SD_BLK_LEN);
		free(buff_new);
	}

#elif CONFIG_SD_SPI
	ret = SPI_Read_Blocks(sector, data, count);
#endif
	if (ret == 0) {
		return SD_OK;
	}
	return SD_ERROR;
}

SD_RESULT SD_WriteBlocks(u32 sector, const u8 *data, u32 count)
{
	char ret;

#if CONFIG_SD_SDIO
	u8 *buff_new = NULL;
	const u8 *buff_alignment = data;

	if (((u32)buff_alignment) % 32 != 0) { //check address 32 byte alignment for SDIO host
		// FIXME why retry
		buff_new = (u8 *) malloc(count * SD_BLK_LEN + 31);
		if (buff_new == NULL) {
			dbg_printf("Fail to malloc cache for SDIO host!!\n");
			return SD_ERROR;
		}
		u8 *buff_new_align = (u8 *)((u32)buff_new & (~(u32)31));
		memcpy(buff_new_align, data, count * SD_BLK_LEN);
		buff_alignment = buff_new_align;
	}

	ret = sdio_sd_write_combine(psdioh_adapter, (u64)sector * SD_BLK_LEN, count, (u8 *)buff_alignment);
	if (ret != HAL_OK) {
		for (int i = 0; i <= retry_max ; i++) {
			i++;
			ret = sdio_sd_write_combine(psdioh_adapter, (u64)sector * SD_BLK_LEN, count, (u8 *)buff_alignment);
			dbg_printf("sdio_sd_write_combine again ret = %d\n", ret);
			if (ret == HAL_OK) {
				dbg_printf("retry write sucess!!!!!!\n");
				break;
			} else {
				dbg_printf("try %d times\n", i);
				if (i >= retry_max) {
					return SD_ERROR;
				}
			}
		}
	}

	if (buff_new) {
		free(buff_new);
	}

#elif CONFIG_SD_SPI
	ret = SPI_Write_Blocks(sector, data, count);
#endif
	if (ret == 0) {
		return SD_OK;
	}
	return SD_ERROR;
}

