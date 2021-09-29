/**************************************************************************//**
 * @file     hal_snand_nsc.h
 * @brief    The header file of hal_snand_nsc.c.
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

#ifndef _HAL_SNAND_NSC_H_
#define _HAL_SNAND_NSC_H_
#include "cmsis.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

#ifdef  __cplusplus
extern "C"
{
#endif

#if defined(CONFIG_BUILD_SECURE)
/**

        \addtogroup hs_hal_snand_nsc_api Serial-NAND FLASH HAL NSC APIs
        \ingroup hs_hal_snand
        \brief The Serial-NAND FLASH HAL NSC (Non-Secure Callable) APIs. Non-secure domain is allowed to access secure functions through NSC APIs.
        @{
*/

void NS_ENTRY hal_snand_init_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_deinit_nsc(hal_snafc_adaptor_t *pAdaptor);
uint32_t NS_ENTRY hal_snand_reset_to_spi_nsc(hal_snafc_adaptor_t *pAdaptor);
uint32_t NS_ENTRY hal_snand_read_id_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_set_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_unset_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_set_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data);
void NS_ENTRY hal_snand_set_status_no_check_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data);
uint32_t NS_ENTRY hal_snand_get_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd);
uint32_t NS_ENTRY hal_snand_wait_ready_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_set_write_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void NS_ENTRY hal_snand_set_write_disable_nsc(hal_snafc_adaptor_t *pAdaptor);
uint32_t NS_ENTRY hal_snand_block_erase_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr);
uint32_t NS_ENTRY hal_snand_page_program_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr);
uint32_t NS_ENTRY hal_snand_pio_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
uint32_t NS_ENTRY hal_snand_pio_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
uint32_t NS_ENTRY hal_snand_dma_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
uint32_t NS_ENTRY hal_snand_dma_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);

/** *@} */ /* End of group hs_hal_snand_nsc_api */

#endif // end of "#if defined(CONFIG_BUILD_SECURE)"


#if defined(CONFIG_BUILD_NONSECURE)

void hal_snand_init_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_deinit_nsc(hal_snafc_adaptor_t *pAdaptor);
uint32_t hal_snand_reset_to_spi_nsc(hal_snafc_adaptor_t *pAdaptor);
uint32_t hal_snand_read_id_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_set_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_unset_quad_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_set_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data);
void hal_snand_set_status_no_check_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd, uint8_t data);
uint8_t hal_snand_get_status_nsc(hal_snafc_adaptor_t *pAdaptor, uint8_t cmd);
void hal_snand_wait_ready_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_set_write_enable_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_set_write_disable_nsc(hal_snafc_adaptor_t *pAdaptor);
void hal_snand_block_erase_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr);
void hal_snand_page_program_nsc(hal_snafc_adaptor_t *pAdaptor, uint32_t blkPageAddr);
void hal_snand_pio_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
void hal_snand_pio_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
void hal_snand_dma_read_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);
void hal_snand_dma_write_nsc(hal_snafc_adaptor_t *pAdaptor, void *memAddr, uint32_t dataLens, uint32_t blkPageAddr);

#define hal_snand_init                      hal_snand_init_nsc
#define hal_snand_deinit                    hal_snand_deinit_nsc
#define hal_snand_reset_to_spi              hal_snand_reset_to_spi_nsc
#define hal_snand_read_id                   hal_snand_read_id_nsc
#define hal_snand_set_quad_enable           hal_snand_set_quad_enable_nsc
#define hal_snand_unset_quad_enable         hal_snand_unset_quad_enable_nsc
#define hal_snand_set_status                hal_snand_set_status_nsc
#define hal_snand_set_status_no_check       hal_snand_set_status_no_check_nsc
#define hal_snand_get_status                hal_snand_get_status_nsc
#define hal_snand_wait_ready                hal_snand_wait_ready_nsc
#define hal_snand_set_write_enable          hal_snand_set_write_enable_nsc
#define hal_snand_set_write_disable         hal_snand_set_write_disable_nsc
#define hal_snand_block_erase               hal_snand_block_erase_nsc
#define hal_snand_page_program              hal_snand_page_program_nsc
#define hal_snand_pio_read                  hal_snand_pio_read_nsc
#define hal_snand_pio_write                 hal_snand_pio_write_nsc
#define hal_snand_dma_read                  hal_snand_dma_read_nsc
#define hal_snand_dma_write                 hal_snand_dma_write_nsc

#endif  // end of "#if defined(CONFIG_BUILD_NONSECURE)"

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_SNAND_NSC_H_"

