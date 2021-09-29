/**************************************************************************//**
 * @file     rtl8735b_flash_sec.c
 * @brief    Implement flash security IP (SEC) ROM code functions.
 *
 * @version  V1.00
 * @date     2021-07-13
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
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

// TODO add doxygen
/**
  \brief todo
*/
#include "cmsis.h"

#ifdef CONFIG_FLASH_SEC_EN

#include "basic_types.h"
#include "rtl8735b.h"
#include "rtl8735b_flash_sec.h"

#define SECTION_FLASHSEC_TEXT           SECTION(".rom.hal_flash_sec.text")
#define SECTION_FLASHSEC_DATA           SECTION(".rom.hal_flash_sec.data")
#define SECTION_FLASHSEC_RODATA         SECTION(".rom.hal_flash_sec.rodata")
#define SECTION_FLASHSEC_BSS            SECTION(".rom.hal_flash_sec.bss")
#define SECTION_FLASHSEC_STUBS          SECTION(".rom.hal_flash_sec.stubs")
#define SECTION_FLASHSEC_EXT_STUBS      SECTION(".rom.hal_flash_sec_extend.stubs")


// TODO notes
// volatile
// API for BootLoader
// No use READ32 and WRITE32

// TODO No magic number
#define FLASH_SEC_SEC_LOCK_CTRL_PTN     0x0DC

// FIXME Remove me??
//SECTION_FLASHSEC_RODATA
//FLASH_SEC_TypeDef *flash_sec_regs = FLASH_SEC;
#define flash_sec_regs                  FLASH_SEC

// TODO add more api disable/enable, init

SECTION_FLASHSEC_STUBS
const hal_flash_sec_func_stubs_t hal_flash_sec_stubs = {
	.hal_flash_sec_get_status = hal_rtl_flash_sec_get_status,
	.hal_flash_sec_get_cid = hal_rtl_flash_sec_get_cid,
	.hal_flash_sec_get_err_addr = hal_rtl_flash_sec_get_err_addr,

	.hal_flash_sec_aes_enable = hal_rtl_flash_sec_aes_enable,
	.hal_flash_sec_aes_disable = hal_rtl_flash_sec_aes_disable,

	.hal_flash_sec_clr_intr = hal_rtl_flash_sec_clr_intr,
	.hal_flash_sec_set_ctrl = hal_rtl_flash_sec_set_ctrl,
	.hal_flash_sec_gcm_key_init = hal_rtl_flash_sec_gcm_key_init,
	.hal_flash_sec_dec_key_init = hal_rtl_flash_sec_dec_key_init,
	.hal_flash_sec_clean_cache = hal_rtl_flash_sec_clean_cache,
	.hal_flash_sec_lock = hal_rtl_flash_sec_lock,

	.hal_flash_sec_remap_region = hal_rtl_flash_sec_remap_region,
	.hal_flash_sec_disable_remap_region = hal_rtl_flash_sec_disable_remap_region,

	.hal_flash_sec_decrypt_region = hal_rtl_flash_sec_decrypt_region,
	.hal_flash_sec_disable_decrypt_region = hal_rtl_flash_sec_disable_decrypt_region,

	.hal_flash_sec_set_tag_base = hal_rtl_flash_sec_set_tag_base,
	.hal_flash_sec_set_iv = hal_rtl_flash_sec_set_iv,

	// Get reg
	.hal_flash_sec_get_remap_region = hal_rtl_flash_sec_get_remap_region,
	.hal_flash_sec_get_decrypt_region = hal_rtl_flash_sec_get_decrypt_region,
	.hal_flash_sec_get_tag_base = hal_rtl_flash_sec_get_tag_base,
	.hal_flash_sec_get_iv = hal_rtl_flash_sec_get_iv
};

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
SECTION_FLASHSEC_EXT_STUBS
const hal_flash_sec_extend_func_stubs_t hal_flash_sec_ext_stubs = {
	.hal_flash_sec_enable_remap_region = hal_rtl_flash_sec_enable_remap_region,
	.hal_flash_sec_calculate_tag_base = hal_rtl_flash_sec_calculate_tag_base,
	.hal_flash_sec_decrypt_init = hal_rtl_flash_sec_decrypt_init,
	.hal_flash_sec_decrypt_region_init = hal_rtl_flash_sec_decrypt_region_init,
	.hal_flash_sec_decrypt_region_enable = hal_rtl_flash_sec_decrypt_region_enable,
	.hal_flash_sec_set_word_from_byte_bigen = hal_rtl_flash_sec_set_word_from_byte_bigen,
};
#endif

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_get_status(hal_flash_sec_status_t *status)
{
	u32 reg_val = flash_sec_regs->FLASH_SEC_SEC_STS_REG;
	status->idle                = (reg_val & FLASH_SEC_BIT_IDLE) >> FLASH_SEC_SHIFT_IDLE;
	status->key_init_done       = (reg_val & FLASH_SEC_BIT_KEY_INIT_DONE) >> FLASH_SEC_SHIFT_KEY_INIT_DONE;
	status->intr_err_tag        = (reg_val & FLASH_SEC_BIT_INTR_ERR_TAG) >> FLASH_SEC_SHIFT_INTR_ERR_TAG;
	status->intr_err_resp       = (reg_val & FLASH_SEC_BIT_INTR_ERR_RESP) >> FLASH_SEC_SHIFT_INTR_ERR_RESP;
	status->intr_err_apb        = (reg_val & FLASH_SEC_BIT_INTR_ERR_APB) >> FLASH_SEC_SHIFT_INTR_ERR_APB;
	status->intr_err_init       = (reg_val & FLASH_SEC_BIT_INTR_ERR_INIT) >> FLASH_SEC_SHIFT_INTR_ERR_INIT;
	status->remap_region_error  = (reg_val & FLASH_SEC_BIT_REMAP_REGN_ERR) >> FLASH_SEC_SHIFT_REMAP_REGN_ERR;
	status->remap_addr_error    = (reg_val & FLASH_SEC_BIT_REMAP_ADDR_ERR) >> FLASH_SEC_SHIFT_REMAP_ADDR_ERR;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_get_cid(hal_flash_sec_cid_t *cid)
{
	u32 reg_val = flash_sec_regs->FLASH_SEC_SEC_CID_REG;
	cid->decrypt_region_num = (reg_val & FLASH_SEC_MASK_REGN_NUMBER) >> FLASH_SEC_SHIFT_REGN_NUMBER;
	cid->iv_num             = (reg_val & FLASH_SEC_MASK_IV_NUMBER) >> FLASH_SEC_SHIFT_IV_NUMBER;
	cid->support_ctr        = (reg_val & FLASH_SEC_BIT_SPRT_CTR) >> FLASH_SEC_SHIFT_SPRT_CTR;
	cid->support_mix        = (reg_val & FLASH_SEC_BIT_SPRT_ECB_MIX) >> FLASH_SEC_SHIFT_SPRT_ECB_MIX;
	cid->support_gcm        = (reg_val & FLASH_SEC_BIT_SPRT_GCM) >> FLASH_SEC_SHIFT_SPRT_GCM;
	cid->key_source         = (reg_val & FLASH_SEC_BIT_KEY_SRC) >> FLASH_SEC_SHIFT_KEY_SRC;
	u8 cache_line_bits      = (reg_val & FLASH_SEC_MASK_LINE_SIZE) >> FLASH_SEC_SHIFT_LINE_SIZE;
	switch (cache_line_bits) {
	case 0:
		cid->cache_line_size    = FLASH_SEC_CACHE_16;
		break;
	case 1:
		cid->cache_line_size    = FLASH_SEC_CACHE_32;
		break;
	case 2:
		cid->cache_line_size    = FLASH_SEC_CACHE_64;
		break;
	case 3:
		cid->cache_line_size    = FLASH_SEC_CACHE_128;
		break;
	}
}

SECTION_FLASHSEC_TEXT
u32 hal_rtl_flash_sec_get_err_addr(void)
{
	return flash_sec_regs->FLASH_SEC_SEC_ERR_ADDR_REG;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_clr_intr(void)
{
	flash_sec_regs->FLASH_SEC_SEC_CLR_INT_REG |= 0x01;
}

// Control reg
SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_aes_enable(void)
{
	HAL_SET_BIT(flash_sec_regs->FLASH_SEC_SEC_CTRL_REG, FLASH_SEC_BIT_AES_EN);
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_aes_disable(void)
{
	HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CTRL_REG, FLASH_SEC_BIT_AES_EN);
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_set_ctrl(hal_flash_sec_adapter_t *adtr)
{
	u32 reg_val = flash_sec_regs->FLASH_SEC_SEC_CTRL_REG;

	// Save only enable
	reg_val &= FLASH_SEC_BIT_AES_EN;

	u32 key_size_bits, tag_size_bits;

	switch (adtr->tag_size) {
	case FLASH_SEC_TAG_4:
		tag_size_bits = 0;
		break;
	case FLASH_SEC_TAG_8:
		tag_size_bits = 1;
		break;
	case FLASH_SEC_TAG_16:
		tag_size_bits = 2;
		break;
	default:
		return HAL_ERR_PARA;
	}
	switch (adtr->key_size) {
	case FLASH_SEC_KEY_128:
		key_size_bits = 0;
		break;
	case FLASH_SEC_KEY_192:
		key_size_bits = 1;
		break;
	case FLASH_SEC_KEY_256:
		key_size_bits = 2;
		break;
	default:
		return HAL_ERR_PARA;
	}
	reg_val |= (tag_size_bits << FLASH_SEC_SHIFT_TAG_SIZE) & FLASH_SEC_MASK_TAG_SIZE;
	reg_val |= (adtr->axi_byte_swap << FLASH_SEC_SHIFT_AXI_BYTE_SWAP) & FLASH_SEC_BIT_AXI_BYTE_SWAP;
	reg_val |= (adtr->axi_word_swap << FLASH_SEC_SHIFT_AXI_WORD_SWAP) & FLASH_SEC_BIT_AXI_WORD_SWAP;
	reg_val |= (key_size_bits << FLASH_SEC_SHIFT_KEY_SIZE) & FLASH_SEC_MASK_KEY_SIZE;
	// Set enable bit with other apis
	//reg_val |= (adtr->aes_enable << FLASH_SEC_SHIFT_AES_EN) & FLASH_SEC_BIT_AES_EN;
	reg_val |= (adtr->line_buffer_icg_dis << FLASH_SEC_SHIFT_ICG_CTRL_0) & FLASH_SEC_BIT_ICG_CTRL_0;
	reg_val |= (adtr->fetch_unit_icg_dis << FLASH_SEC_SHIFT_ICG_CTRL_1) & FLASH_SEC_BIT_ICG_CTRL_1;
	reg_val |= (adtr->apb_ctrl_icg_dis << FLASH_SEC_SHIFT_ICG_CTRL_2) & FLASH_SEC_BIT_ICG_CTRL_2;
	reg_val |= (adtr->key_reg_icg_dis << FLASH_SEC_SHIFT_ICG_CTRL_3) & FLASH_SEC_BIT_ICG_CTRL_3;
	flash_sec_regs->FLASH_SEC_SEC_CTRL_REG = reg_val;
	return HAL_OK;
}

// Maintenance reg
SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_gcm_key_init(void)
{
	flash_sec_regs->FLASH_SEC_SEC_MAINTAIN_OP_REG |= FLASH_SEC_BIT_GCM_KEY_INIT;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_dec_key_init(void)
{
	flash_sec_regs->FLASH_SEC_SEC_MAINTAIN_OP_REG |= FLASH_SEC_BIT_DEC_KEY_INIT;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_clean_cache(void)
{
	flash_sec_regs->FLASH_SEC_SEC_MAINTAIN_OP_REG |= FLASH_SEC_BIT_CLEAN_CACHE_LINE;
}
// End of Maintenance


SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_lock(void)
{
	flash_sec_regs->FLASH_SEC_SEC_LOCK_CTRL_REG = FLASH_SEC_SEC_LOCK_CTRL_PTN;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_set_tag_base(u32 tag_base_addr)
{
	flash_sec_regs->FLASH_SEC_SEC_GCM_TAG_BASE_REG = tag_base_addr;
}

// TODO move the adtr to ram??
SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_remap_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->remap_region_sel;
	u8 enable = adtr->remap_region_enable;
	u32 va_base = adtr->va_base;
	u32 pa_base = adtr->pa_base;
	u32 va_end = (u32)adtr->va_base + (u32)adtr->remap_region_size - 1;

	u32 va_base_page = GET_PAGE(va_base);
	u32 va_end_page = GET_PAGE(va_end);
	volatile u32 addr_val = 0;
	addr_val |= (va_base_page << FLASH_SEC_SHIFT_REMAP_BASE_0_ADDR) & FLASH_SEC_MASK_REMAP_BASE_0_ADDR;
	addr_val |= (va_end_page << FLASH_SEC_SHIFT_REMAP_END_0_ADDR) & FLASH_SEC_MASK_REMAP_END_0_ADDR;

	u32 pa_base_page = GET_PAGE(pa_base);
	volatile u32 offset_val = 0;
	offset_val |= ((u32)enable << FLASH_SEC_SHIFT_REMAP_0_ENABLE) & FLASH_SEC_BIT_REMAP_0_ENABLE;
	offset_val |= (pa_base_page << FLASH_SEC_SHIFT_REMAP_0_OFFSET) & FLASH_SEC_MASK_REMAP_0_OFFSET;

	switch (region_id) {
	case 0:
		flash_sec_regs->FLASH_SEC_SEC_REMAP0_ADDR_REG = addr_val;
		flash_sec_regs->FLASH_SEC_SEC_REMAP0_OFFSET_REG = offset_val;
		break;
	case 1:
		flash_sec_regs->FLASH_SEC_SEC_REMAP1_ADDR_REG = addr_val;
		flash_sec_regs->FLASH_SEC_SEC_REMAP1_OFFSET_REG = offset_val;
		break;
	case 2:
		flash_sec_regs->FLASH_SEC_SEC_REMAP2_ADDR_REG = addr_val;
		flash_sec_regs->FLASH_SEC_SEC_REMAP2_OFFSET_REG = offset_val;
		break;
	case 3:
		flash_sec_regs->FLASH_SEC_SEC_REMAP3_ADDR_REG = addr_val;
		flash_sec_regs->FLASH_SEC_SEC_REMAP3_OFFSET_REG = offset_val;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	return HAL_OK;
}

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)

SECTION_FLASHSEC_TEXT
uint8_t hal_rtl_flash_sec_check_addr_region(uint32_t base, uint32_t size)
{
	uint32_t end = 0x0;
	if ((base & PAGE_OFFSET_MASK) || (base < FLASH_ACCESS_BASE)) {
		return 1;
	}

	if (size & PAGE_OFFSET_MASK) {
		return 1;
	}
	end = base + size;
	if (end > FLASH_MAX_END) {
		return 1;
	}
	return 0;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_check_rom_error(hal_flash_sec_adapter_t *adtr)
{
	hal_flash_sec_status_t *psec_status = NULL;
	psec_status = &(adtr->sec_status);
	hal_rtl_flash_sec_get_status(psec_status);

	if (1 == psec_status->remap_region_error) {
		return HAL_ERR_UNKNOWN;
	} else if (1 == psec_status->remap_addr_error) {
		return HAL_ERR_UNKNOWN;
	} else if (1 == psec_status->intr_err_apb) {
		return HAL_ERR_UNKNOWN;
	} else if (1 == psec_status->intr_err_tag) {
		return HAL_ERR_UNKNOWN;
	} else if (1 == psec_status->intr_err_resp) {
		return HAL_ERR_UNKNOWN;
	} else if (1 == psec_status->intr_err_init) {
		return HAL_ERR_UNKNOWN;
	}
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_calculate_tag_base(uint32_t cache_line_size, uint32_t tag_size,
		uint32_t flash_addr, uint32_t region_size, uint32_t tag_region_addr,
		uint32_t *tag_base, uint32_t *tag_region_size)
{
	hal_status_t ret = HAL_OK;
	uint32_t ratio = 0x0, tag_offset = 0x0;

	if (cache_line_size % tag_size != 0) {
		dbg_printf("calculate_tag_base arg error: cache %% tag != 0");
		ret = HAL_ERR_PARA;
		return ret;
	}
	ratio = cache_line_size / tag_size;
	if (NULL != tag_region_size) {
		*tag_region_size = region_size / ratio;
	}

	tag_offset = (flash_addr & FLASH_OFFSET_MASK) / ratio;
	if (NULL != tag_base) {
		*tag_base = tag_region_addr - tag_offset;
	}
	return ret;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_default_calculate_tag_base(uint32_t flash_addr, uint32_t region_size,
		uint32_t tag_region_addr, uint32_t *tag_base,
		uint32_t *tag_region_size)
{
	hal_status_t ret = HAL_OK;
	ret = hal_rtl_flash_sec_calculate_tag_base(FLASH_SEC_CACHE_32, FLASH_SEC_TAG_4, flash_addr, region_size,
			tag_region_addr, tag_base, tag_region_size);
	return ret;
}


SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_enable_remap_region(hal_flash_sec_adapter_t *adtr, uint8_t rmp_region_sel,
		uint32_t va_base, uint32_t pa_base, uint32_t region_size)
{
	hal_status_t ret;
	uint32_t va_end = 0xFFFFFFFF;
	uint32_t pa_end = 0xFFFFFFFF;

	// check options
	if (adtr == NULL) {
		return HAL_ERR_MEM;
	}
	if (adtr->remap_region_sel >= FLASH_SEC_REMAP_REGION_MAX) {
		return HAL_ERR_PARA;
	}
	if (hal_rtl_flash_sec_check_addr_region(va_base, region_size)) {
		return HAL_ERR_PARA;
	}
	if (hal_rtl_flash_sec_check_addr_region(pa_base, region_size)) {
		return HAL_ERR_PARA;
	}

	va_end = va_base + region_size;
	pa_end = pa_base + region_size;

	// check overlap
	if (va_base > pa_base) {
		if (pa_end > va_base) {
			return HAL_ERR_PARA;
		}
	} else if (va_base < pa_base) {
		if (va_end > pa_base) {
			return HAL_ERR_PARA;
		}
	} else {
		return HAL_ERR_PARA;
	}

	adtr->va_base = va_base;
	adtr->pa_base = pa_base;
	adtr->remap_region_size   = region_size;
	adtr->remap_region_sel    = rmp_region_sel;
	adtr->remap_region_enable = ENABLE;

	ret = hal_rtl_flash_sec_remap_region(adtr);
	if (ret == HAL_OK) {
		return hal_rtl_flash_sec_check_rom_error(adtr);
	}
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_disable_rmp_region(hal_flash_sec_adapter_t *adtr, uint8_t rmp_region_sel)
{
	hal_status_t ret = HAL_OK;
	if (adtr == NULL) {
		return HAL_ERR_MEM;
	}
	adtr->remap_region_sel = rmp_region_sel;
	ret = hal_rtl_flash_sec_disable_remap_region(adtr);
}

#endif

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_disable_remap_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->remap_region_sel;
	switch (region_id) {
	case 0:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_REMAP0_OFFSET_REG, FLASH_SEC_BIT_REMAP_0_ENABLE);
		break;
	case 1:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_REMAP1_OFFSET_REG, FLASH_SEC_BIT_REMAP_1_ENABLE);
		break;
	case 2:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_REMAP2_OFFSET_REG, FLASH_SEC_BIT_REMAP_2_ENABLE);
		break;
	case 3:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_REMAP3_OFFSET_REG, FLASH_SEC_BIT_REMAP_3_ENABLE);
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_set_iv(hal_flash_sec_adapter_t *adtr)
{
	u8 iv_sel = adtr->iv_sel;
	u32 iv_low = adtr->iv_low;
	u32 iv_high = adtr->iv_high;

	switch (iv_sel) {
	case 0:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV0_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV0_HIGH_REG = iv_high;
		break;
	case 1:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV1_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV1_HIGH_REG = iv_high;
		break;
	case 2:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV2_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV2_HIGH_REG = iv_high;
		break;
	case 3:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV3_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV3_HIGH_REG = iv_high;
		break;
	case 4:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV4_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV4_HIGH_REG = iv_high;
		break;
	case 5:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV5_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV5_HIGH_REG = iv_high;
		break;
	case 6:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV6_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV6_HIGH_REG = iv_high;
		break;
	case 7:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV7_LOW_REG = iv_low;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV7_HIGH_REG = iv_high;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	return HAL_OK;
}

// change arg to region size
SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_decrypt_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->decrypt_region_sel;
	u8 enable = adtr->decrypt_region_enable;
	u8 cipher_mode = adtr->cipher;
	u8 iv_sel = adtr->iv_sel;
#if IS_CUT_TEST(CONFIG_CHIP_VER)
	u32 va_base = adtr->va_base;
	u32 va_end = (u32)adtr->va_base + (u32)adtr->decrypt_region_size - 1;

	u32 va_base_page = GET_PAGE(va_base);
	u32 region_base_val = (va_base_page << FLASH_SEC_SHIFT_REGN_0_BASE) & FLASH_SEC_MASK_REGN_0_BASE;

	u32 va_end_page = GET_PAGE(va_end);
	u32 region_end_val = 0;
	region_end_val |= ((u32)enable << FLASH_SEC_SHIFT_REGN_0_ENABLE) & FLASH_SEC_BIT_REGN_0_ENABLE;
	region_end_val |= ((u32)cipher_mode << FLASH_SEC_SHIFT_REGN_0_MODE) & FLASH_SEC_MASK_REGN_0_MODE;
	region_end_val |= ((u32)iv_sel << FLASH_SEC_SHIFT_REGN_0_IV_NUM) & FLASH_SEC_MASK_REGN_0_IV_NUM;
	region_end_val |= (va_end_page << FLASH_SEC_SHIFT_REGN_0_END) & FLASH_SEC_MASK_REGN_0_END;
#else
	u32 dec_base = adtr->dec_base;
	u32 dec_end = (u32)adtr->dec_base + (u32)adtr->decrypt_region_size - 1;

	u32 dec_base_page = GET_PAGE(dec_base);
	u32 region_base_val = (dec_base_page << FLASH_SEC_SHIFT_REGN_0_BASE) & FLASH_SEC_MASK_REGN_0_BASE;

	u32 dec_end_page = GET_PAGE(dec_end);
	u32 region_end_val = 0;
	region_end_val |= ((u32)enable << FLASH_SEC_SHIFT_REGN_0_ENABLE) & FLASH_SEC_BIT_REGN_0_ENABLE;
	region_end_val |= ((u32)cipher_mode << FLASH_SEC_SHIFT_REGN_0_MODE) & FLASH_SEC_MASK_REGN_0_MODE;
	region_end_val |= ((u32)iv_sel << FLASH_SEC_SHIFT_REGN_0_IV_NUM) & FLASH_SEC_MASK_REGN_0_IV_NUM;
	region_end_val |= (dec_end_page << FLASH_SEC_SHIFT_REGN_0_END) & FLASH_SEC_MASK_REGN_0_END;
#endif
	switch (region_id) {
	case 0:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION0_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION0_END_REG = region_end_val;
		break;
	case 1:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION1_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION1_END_REG = region_end_val;
		break;
	case 2:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION2_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION2_END_REG = region_end_val;
		break;
	case 3:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION3_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION3_END_REG = region_end_val;
		break;
	case 4:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION4_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION4_END_REG = region_end_val;
		break;
	case 5:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION5_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION5_END_REG = region_end_val;
		break;
	case 6:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION6_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION6_END_REG = region_end_val;
		break;
	case 7:
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION7_BASE_REG = region_base_val;
		flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION7_END_REG = region_end_val;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	return HAL_OK;
}

#if IS_AFTER_CUT_A(CONFIG_CHIP_VER)
SECTION_FLASHSEC_TEXT
uint32_t hal_rtl_flash_sec_set_word_from_byte_bigen(const unsigned char *s_value)
{
	volatile uint32_t val = 0x0;
	(val) = (((uint32_t)(s_value)[0] << 24)
			 | ((uint32_t)(s_value)[1] << 16)
			 | ((uint32_t)(s_value)[2] <<  8)
			 | ((uint32_t)(s_value)[3]));
	return val;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_decrypt_key_sel(hal_flash_sec_adapter_t *adtr, uint8_t key_sel)
{
	volatile uint32_t reg_val = 0x0;
	reg_val |= SYSON_S_BIT_FLASH_SEC_KEY_RDY_EN;
	reg_val |= (key_sel << SYSON_S_SHIFT_SCE_KEY_SEL) & SYSON_S_MASK_SCE_KEY_SEL;
	SYSON_S->SYSON_S_REG_SYS_SEC_CTRL = reg_val;
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_decrypt_init(hal_flash_sec_adapter_t *adtr, const uint8_t key_sel, uint32_t key_size,
		uint32_t cache_line_size, uint32_t tag_size)
{
	hal_status_t ret = HAL_OK;
	if (adtr == NULL) {
		ret = HAL_ERR_MEM;
		return ret;
	}

	adtr->key_size = key_size;
	adtr->cache_line_size = cache_line_size;
	adtr->tag_size = tag_size;

	ret = hal_rtl_flash_sec_set_ctrl(adtr);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = hal_rtl_flash_sec_decrypt_key_sel(adtr, key_sel);
	if (ret != HAL_OK) {
		return ret;
	}
	return ret;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_default_decrypt_init(hal_flash_sec_adapter_t *adtr, const uint8_t key_sel)
{
	hal_status_t ret = HAL_OK;
	ret = hal_rtl_flash_sec_decrypt_init(adtr, key_sel, FLASH_SEC_KEY_256, FLASH_SEC_CACHE_32, FLASH_SEC_TAG_4);
	return ret;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_decrypt_region_init(hal_flash_sec_adapter_t *adtr, const uint32_t iv_low_ptn, const uint32_t iv_high_ptn,
		uint8_t dec_region_sel)
{
	hal_status_t ret = HAL_OK;
	if (adtr == NULL) {
		ret = HAL_ERR_MEM;
		return ret;
	}
	if (dec_region_sel >= FLASH_SEC_DECRYPT_REGION_MAX) {
		ret = HAL_ERR_PARA;
		return ret;
	}

	adtr->iv_low  = iv_low_ptn;
	adtr->iv_high = iv_high_ptn;
	adtr->iv_sel  = dec_region_sel;
	ret = hal_rtl_flash_sec_set_iv(adtr);
	return ret;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_decrypt_region_enable(hal_flash_sec_adapter_t *adtr, const uint8_t cipher_sel,
		uint32_t dec_base, uint32_t dec_size, uint32_t tag_base)
{
	hal_status_t ret = HAL_OK;
	uint8_t dec_region_sel = 0x0;
	if (adtr == NULL) {
		ret = HAL_ERR_MEM;
		return ret;
	}

	dec_region_sel = (adtr->iv_sel);
	// check options
	if (dec_region_sel >= FLASH_SEC_DECRYPT_REGION_MAX) {
		ret = HAL_ERR_PARA;
		return ret;
	}
	if (hal_rtl_flash_sec_check_addr_region(dec_base, dec_size)) {
		return HAL_ERR_PARA;
	}
	if (adtr->cipher >= FLASH_SEC_CIPHER_MAX) {
		return HAL_ERR_PARA;
	}

	adtr->decrypt_region_sel = dec_region_sel;
	adtr->decrypt_region_enable = ENABLE;
	adtr->cipher   = cipher_sel;
	adtr->dec_base = dec_base;
	adtr->decrypt_region_size = dec_size;
	ret = hal_rtl_flash_sec_decrypt_region(adtr);
	if (ret != HAL_OK) {
		return ret;
	}

	if (FLASH_SEC_CIPHER_GCM == cipher_sel) {
		hal_rtl_flash_sec_set_tag_base(tag_base);
	}

	// Set Flash SEC Cipher engine enable
	hal_rtl_flash_sec_aes_enable();
	return ret;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_disable_dec_region(hal_flash_sec_adapter_t *adtr, uint8_t dec_region_sel)
{
	hal_status_t ret = HAL_OK;
	if (adtr == NULL) {
		ret = HAL_ERR_MEM;
		return ret;
	}
	adtr->decrypt_region_sel = dec_region_sel;
	ret = hal_rtl_flash_sec_disable_decrypt_region(adtr);
	return ret;
}
#endif

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_disable_decrypt_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->decrypt_region_sel;
	switch (region_id) {
	case 0:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION0_END_REG, FLASH_SEC_BIT_REGN_0_ENABLE);
		break;
	case 1:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION1_END_REG, FLASH_SEC_BIT_REGN_1_ENABLE);
		break;
	case 2:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION2_END_REG, FLASH_SEC_BIT_REGN_2_ENABLE);
		break;
	case 3:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION3_END_REG, FLASH_SEC_BIT_REGN_3_ENABLE);
		break;
	case 4:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION4_END_REG, FLASH_SEC_BIT_REGN_4_ENABLE);
		break;
	case 5:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION5_END_REG, FLASH_SEC_BIT_REGN_5_ENABLE);
		break;
	case 6:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION6_END_REG, FLASH_SEC_BIT_REGN_6_ENABLE);
		break;
	case 7:
		HAL_CLEAR_BIT(flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION7_END_REG, FLASH_SEC_BIT_REGN_7_ENABLE);
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_get_remap_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->remap_region_sel;
	u32 addr_val = 0;
	u32 offset_val = 0;
	switch (region_id) {
	case 0:
		addr_val   = flash_sec_regs->FLASH_SEC_SEC_REMAP0_ADDR_REG;
		offset_val = flash_sec_regs->FLASH_SEC_SEC_REMAP0_OFFSET_REG;
		break;
	case 1:
		addr_val   = flash_sec_regs->FLASH_SEC_SEC_REMAP1_ADDR_REG;
		offset_val = flash_sec_regs->FLASH_SEC_SEC_REMAP1_OFFSET_REG;
		break;
	case 2:
		addr_val   = flash_sec_regs->FLASH_SEC_SEC_REMAP2_ADDR_REG;
		offset_val = flash_sec_regs->FLASH_SEC_SEC_REMAP2_OFFSET_REG;
		break;
	case 3:
		addr_val   = flash_sec_regs->FLASH_SEC_SEC_REMAP3_ADDR_REG;
		offset_val = flash_sec_regs->FLASH_SEC_SEC_REMAP3_OFFSET_REG;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	adtr->remap_region_enable = (offset_val & FLASH_SEC_BIT_REMAP_0_ENABLE) >> FLASH_SEC_SHIFT_REMAP_0_ENABLE;
	adtr->va_base = (addr_val & FLASH_SEC_MASK_REMAP_BASE_0_ADDR) >> FLASH_SEC_SHIFT_REMAP_BASE_0_ADDR << PAGE_SHIFT;
	adtr->pa_base = (offset_val & FLASH_SEC_MASK_REMAP_0_OFFSET) >> FLASH_SEC_SHIFT_REMAP_0_OFFSET << PAGE_SHIFT;
	u32 va_end = (addr_val & FLASH_SEC_MASK_REMAP_END_0_ADDR) >> FLASH_SEC_SHIFT_REMAP_END_0_ADDR << PAGE_SHIFT;
	adtr->remap_region_size = va_end - adtr->va_base;
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_get_decrypt_region(hal_flash_sec_adapter_t *adtr)
{
	u8 region_id = adtr->decrypt_region_sel;
	u32 base_val = 0;
	u32 end_val = 0;
	switch (region_id) {
	case 0:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION0_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION0_END_REG;
		break;
	case 1:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION1_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION1_END_REG;
		break;
	case 2:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION2_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION2_END_REG;
		break;
	case 3:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION3_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION3_END_REG;
		break;
	case 4:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION4_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION4_END_REG;
		break;
	case 5:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION5_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION5_END_REG;
		break;
	case 6:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION6_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION6_END_REG;
		break;
	case 7:
		base_val = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION7_BASE_REG;
		end_val  = flash_sec_regs->FLASH_SEC_SEC_CIPHER_REGION7_END_REG;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	adtr->decrypt_region_enable = (end_val & FLASH_SEC_BIT_REMAP_0_ENABLE) >> FLASH_SEC_SHIFT_REGN_0_ENABLE;
	adtr->cipher = (end_val & FLASH_SEC_MASK_REGN_0_MODE) >> FLASH_SEC_SHIFT_REGN_0_MODE;
	adtr->iv_sel = (end_val & FLASH_SEC_MASK_REGN_0_IV_NUM) >> FLASH_SEC_SHIFT_REGN_0_IV_NUM;
	adtr->va_base = (base_val & FLASH_SEC_MASK_REGN_0_BASE) >> FLASH_SEC_SHIFT_REGN_0_BASE << PAGE_SHIFT;
	u32 va_end = (base_val & FLASH_SEC_MASK_REGN_0_END) >> FLASH_SEC_SHIFT_REGN_0_END << PAGE_SHIFT;
	adtr->decrypt_region_size = va_end - adtr->va_base;
	return HAL_OK;
}

SECTION_FLASHSEC_TEXT
void hal_rtl_flash_sec_get_tag_base(u32 *tag_base)
{
	*tag_base = flash_sec_regs->FLASH_SEC_SEC_GCM_TAG_BASE_REG;
}

SECTION_FLASHSEC_TEXT
hal_status_t hal_rtl_flash_sec_get_iv(hal_flash_sec_adapter_t *adtr)
{
	u8 iv_sel = adtr->iv_sel;
	u32 iv_low = 0;
	u32 iv_high = 0;

	switch (iv_sel) {
	case 0:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV0_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV0_HIGH_REG;
		break;
	case 1:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV1_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV1_HIGH_REG;
		break;
	case 2:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV2_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV2_HIGH_REG;
		break;
	case 3:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV3_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV3_HIGH_REG;
		break;
	case 4:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV4_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV4_HIGH_REG;
		break;
	case 5:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV5_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV5_HIGH_REG;
		break;
	case 6:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV6_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV6_HIGH_REG;
		break;
	case 7:
		iv_low =  flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV7_LOW_REG;
		iv_high = flash_sec_regs->FLASH_SEC_SEC_CIPHER_IV7_HIGH_REG;
		break;
	default:
		return HAL_ERR_PARA;
		break;
	}
	adtr->iv_low = iv_low;
	adtr->iv_high = iv_high;
	return HAL_OK;
}

#endif // end of "#if CONFIG_FLASHSEC_EN"
