/**************************************************************************//**
 * @file     hal_snand_nsc.c
 * @brief    Execute Serial NAND Flash commands in non-secure mode through non-secure callable functions.
 * @version  1.00
 * @date     2021-06-25
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#include "hal_snand.h"
#include "hal_snand_nsc.h"
#include "rtl8735b_snand.h"

#if defined(CONFIG_BUILD_SECURE)

/**
    \addtogroup hs_hal_snand_nsc_api FLASH HAL NSC APIs
    \ingroup hs_hal_snand
    \brief The Serial-NAND FLASH HAL NSC (Non-Secure Callable) APIs. Non-secure domain is allowed to access secure functions through NSC APIs.
    @{
*/
SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_init_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_init(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_deinit_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_deinit(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_reset_to_spi_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_reset_to_spi(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_read_id_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_read_id(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_set_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_set_quad_enable(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_unset_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_unset_quad_enable(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_set_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data)
{
	hal_snand_set_status(pAdaptor, cmd, data);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_set_status_no_check_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data)
{
	hal_snand_set_status_no_check(pAdaptor, cmd, data);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_get_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd)
{
	hal_snand_get_status(pAdaptor, cmd);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_wait_ready_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_wait_ready(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_set_write_enable_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_set_write_enable(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY
hal_snand_set_write_disable_nsc(hal_snafc_adaptor_t *pAdaptor)
{
	hal_snand_set_write_disable(pAdaptor);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_block_erase_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr)
{
	hal_snand_block_erase(pAdaptor, blkPageAddr);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_page_program_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr)
{
	hal_snand_page_program(pAdaptor, blkPageAddr);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_pio_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr)
{
	hal_snand_pio_read(pAdaptor, memAddr, dataLens, blkPageAddr);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_pio_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr)
{
	hal_snand_pio_write(pAdaptor, memAddr, dataLens, blkPageAddr);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_dma_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr)
{
	hal_snand_dma_read(pAdaptor, memAddr, dataLens,  blkPageAddr);
}

SECTION_NS_ENTRY_FUNC
uint32_t NS_ENTRY
hal_snand_dma_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr)
{
	hal_snand_dma_write(pAdaptor, memAddr, dataLens, blkPageAddr);
}

/** *@} */ /* End of group hs_hal_flash_nsc_api */

#endif
