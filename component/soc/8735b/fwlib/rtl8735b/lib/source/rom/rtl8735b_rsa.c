/**************************************************************************//**
 * @file     rtl8735b_rsa.c
 * @brief    This file implements the HW RSA HAL functions.
 *
 * @version  V1.00
 * @date     2021-03-26
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#include "cmsis.h"
#include "rtl8735b_rsa.h"


#if CONFIG_RSA_EN

#define SECTION_RSA_TEXT           SECTION(".rom.hal_rsa.text")
#define SECTION_RSA_DATA           SECTION(".rom.hal_rsa.data")
#define SECTION_RSA_RODATA         SECTION(".rom.hal_rsa.rodata")
#define SECTION_RSA_BSS            SECTION(".rom.hal_rsa.bss")
#define SECTION_RSA_STUBS          SECTION(".rom.hal_rsa.stubs")




#if !defined(CONFIG_BUILD_NONSECURE)
#undef RSA_REG_BASE
#define RSA_REG_BASE (RSA_S)

#undef RSA_DATA_BUFFER_ADDR
#define RSA_DATA_BUFFER_ADDR (0x50067000UL)

#define HW_RSA        ((HW_RSA_TypeDef *) 0x50067000UL)
#else
#undef RSA_REG_BASE
#define RSA_REG_BASE (RSA)

#undef RSA_DATA_BUFFER_ADDR
#define RSA_DATA_BUFFER_ADDR (0x40067000UL)

#define HW_RSA        ((HW_RSA_TypeDef *) 0x40067000UL)
#endif





SECTION_RSA_DATA uint8_t rsa_key_word_cnt = 64;

SECTION_RSA_BSS hal_rsa_adapter_t *rsa_comm_adapter;

#define RSA_Data_Buffer        ((RSA_DataBuffer_Type *) RSA_DATA_BUFFER_ADDR)

SECTION_RSA_STUBS const hal_rsa_func_stubs_t hal_rsa_stubs = {
	.rsa_comm_adp = &rsa_comm_adapter,
	.hal_rsa_set_key_size = hal_rtl_rsa_set_key_size,
	.hal_rsa_set_byte_swap = hal_rtl_rsa_set_byte_swap,
	.hal_rsa_set_endian = hal_rtl_rsa_set_endian,
	.hal_rsa_set_operands = hal_rtl_rsa_set_operands,
	.hal_rsa_check_status = hal_rtl_rsa_check_status,
	.hal_rsa_compute = hal_rtl_rsa_compute,
	.hal_rsa_process = hal_rtl_rsa_process,
	.hal_rsa_config = hal_rtl_rsa_config,

#if !defined(CONFIG_BUILD_NONSECURE)
	.hal_rsa_clock_init = hal_rtl_rsa_clock_init
#endif
};



SECTION_RSA_TEXT
int hal_rtl_rsa_set_key_size(hal_rsa_adapter_t *rsa_adp)
{
	if (!((rsa_adp->key_type == 0) | (rsa_adp->key_type == 2) | (rsa_adp->key_type == 5))) {
		DBG_RSA_ERR("Input key type wrong number %x\r\n", rsa_adp->key_type);
		return FAIL;
	}

	//rsa_adp->key_type = type;
	HW_RSA->Mode_control_field.Key_size_sel = rsa_adp->key_type;

	switch (rsa_adp->key_type) {
	case RSA_KEY_SEL_2048:
		rsa_adp->rsa_key_word_cnt = 64;
		//rsa_key_word_cnt = 64;
		break;

	case RSA_KEY_SEL_1024:
		rsa_adp->rsa_key_word_cnt = 32;
		//rsa_key_word_cnt = 32;
		break;

	case RSA_KEY_SEL_3072:
		rsa_adp->rsa_key_word_cnt = 96;
		//rsa_key_word_cnt = 96;
		break;
	}

//    u32 RSA_KEY_SIZE_SEL;
//    RSA_TypeDef* RSA_OBJ = (RSA_TypeDef*)(rsa_comm_adapter->base_addr);
//
//    RSA_KEY_SIZE_SEL = (RSA_MASK_KEY_SIZE_SEL & (type << RSA_SHIFT_KEY_SIZE_SEL));
//    RSA_OBJ->RSA_MODE_CONTROL_FIELD = RSA_OBJ->RSA_MODE_CONTROL_FIELD | RSA_KEY_SIZE_SEL;
//
//    switch (type)
//    {
//        case RSA_KEY_SEL_2048:
//        rsa_key_word_cnt = 64;
//        break;
//
//        case RSA_KEY_SEL_1024:
//        rsa_key_word_cnt = 32;
//        break;
//
//        case RSA_KEY_SEL_3072:
//        rsa_key_word_cnt = 96;
//        break;
//    }

	return SUCCESS;
}

SECTION_RSA_TEXT
int hal_rtl_rsa_set_byte_swap(hal_rsa_adapter_t *rsa_adp)
{
	if (rsa_adp->byte_endian > 1) {
		DBG_RSA_ERR("Input endian wrong number %x\r\n", rsa_adp->byte_endian);
		return FAIL;
	}

	//rsa_adp->byte_endian = byte_endian;
	HW_RSA->Mode_control_field.Byte_swap = rsa_adp->byte_endian;

//    u32 BYTE_SWAP;
//    RSA_TypeDef* RSA_OBJ = (RSA_TypeDef*)(rsa_comm_adapter->base_addr);
//
//    BYTE_SWAP = (RSA_BIT_BYTE_SWAP & (byte_endian << RSA_SHIFT_BYTE_SWAP));
//    RSA_OBJ->RSA_MODE_CONTROL_FIELD = RSA_OBJ->RSA_MODE_CONTROL_FIELD | BYTE_SWAP;

	return SUCCESS;
}

SECTION_RSA_TEXT
int hal_rtl_rsa_set_endian(hal_rsa_adapter_t *rsa_adp)
{
	if (rsa_adp->endian > 1) {
		DBG_RSA_ERR("Input endian wrong number %x\r\n", rsa_adp->endian);
		return FAIL;
	}

	//rsa_adp->endian = endian;
	HW_RSA->Mode_control_field.Endian = rsa_adp->endian;

//    u32 ENDIAN;
//    RSA_TypeDef* RSA_OBJ = (RSA_TypeDef*)(rsa_comm_adapter->base_addr);
//
//    ENDIAN = (RSA_BIT_ENDIAN & (endian << RSA_SHIFT_ENDIAN));
//    RSA_OBJ->RSA_MODE_CONTROL_FIELD = RSA_OBJ->RSA_MODE_CONTROL_FIELD | ENDIAN;

	return SUCCESS;
}

SECTION_RSA_TEXT
int hal_rtl_rsa_set_operands(hal_rsa_adapter_t *rsa_adp)
{
	int ret;
	ret = SUCCESS;
	if ((rsa_adp->Message == NULL) || (rsa_adp->Exponent == NULL) || (rsa_adp->Modulus == NULL)) {
		DBG_RSA_ERR("Invaild parameters\r\n");
		return FAIL;
	}

//    rsa_adp->exp_word_cnt = exp_word_cnt;
//    rsa_adp->Message = M;
//    rsa_adp->Exponent = e;
//    rsa_adp->Modulus = N;

	for (uint16_t i = 0; i < rsa_adp->rsa_key_word_cnt; i ++) {
		HW_RSA->N[i] = (i < rsa_adp->rsa_key_word_cnt) ? rsa_adp->Modulus[i]  : 0;
		HW_RSA->M[i] = (i < rsa_adp->rsa_key_word_cnt) ? rsa_adp->Message[i]  : 0;
		HW_RSA->e[i] = (i < rsa_adp->exp_word_cnt)     ? rsa_adp->Exponent[i] : 0;
	}

//    for (uint16_t i = 0; i < rsa_key_word_cnt; i ++)
//    {
//        HW_RSA->N[i] = (i < rsa_key_word_cnt) ? N[i] : 0;
//        HW_RSA->M[i] = (i < rsa_key_word_cnt) ? M[i] : 0;
//        HW_RSA->e[i] = (i < exp_word_cnt) ? e[i] : 0;
//    }

//    for (uint16_t i = 0; i < rsa_key_word_cnt; i ++)
//    {
//        RSA_Data_Buffer->N[i] = (i < rsa_key_word_cnt) ? N[i] : 0;
//        RSA_Data_Buffer->M[i] = (i < rsa_key_word_cnt) ? M[i] : 0;
//        RSA_Data_Buffer->e[i] = (i < exp_word_cnt) ? e[i] : 0;
//    }

	return ret;
}

SECTION_RSA_TEXT
RSA_ERR_CODE hal_rtl_rsa_check_status(hal_rsa_adapter_t *rsa_adp)
{
	RSA_ERR_CODE status = RSA_ERR_UNFINISHED;

	if (HW_RSA->Status_field.RSA_finish == 0) {
		if (HW_RSA->Status_field.NPINV_error) {
			status |= RSA_ERR_NPINV;
			DBG_RSA_ERR("RSA_ERR_NPINV\r\n");
		}

		if (HW_RSA->Status_field.M_range_error) {
			status |= RSA_ERR_M_RANGE;
			DBG_RSA_ERR("RSA_ERR_M_RANGE\r\n");
		}

		if (HW_RSA->Status_field.Exp_error) {
			status |= RSA_ERR_EXP;
			DBG_RSA_ERR("RSA_ERR_EXP\r\n");
		}
	} else {
		status = RSA_ERR_NONE;
	}

	if (status == RSA_ERR_UNFINISHED) {
		DBG_RSA_ERR("RSA_ERR_UNFINISHED\r\n");
	}

	rsa_adp->status = status;

//    RSA_TypeDef* RSA_OBJ = (RSA_TypeDef*)(rsa_comm_adapter->base_addr);
//    RSA_ERR_CODE status = RSA_ERR_UNFINISHED;
//
//    if ( ((RSA_OBJ->RSA_STATUS_FIELD & RSA_BIT_FINISH) >> RSA_SHIFT_FINISH) == 0 )
//    {
//        if ( ((RSA_OBJ->RSA_STATUS_FIELD & RSA_BIT_NPINV_ERROR) >> RSA_SHIFT_NPINV_ERROR) == 1 )
//        {
//            status |= RSA_ERR_NPINV;
//            DBG_RSA_ERR("ERR_NPINV status error\r\n");
//        }
//
//        if ( ((RSA_OBJ->RSA_STATUS_FIELD & RSA_BIT_M_RANGE_ERROR) >> RSA_SHIFT_M_RANGE_ERROR) == 1 )
//        {
//            status |= RSA_ERR_M_RANGE;
//            DBG_RSA_ERR("ERR_M_RANGE status error\r\n");
//        }
//
//        if ( ((RSA_OBJ->RSA_STATUS_FIELD & RSA_BIT_EXP_ERROR) >> RSA_SHIFT_EXP_ERROR) == 1 )
//        {
//            status |= RSA_ERR_EXP;
//            DBG_RSA_ERR("ERR_EXP status error\r\n");
//        }
//    }
//    else
//    {
//        status = RSA_ERR_NONE;
//    }

	return status;
}


SECTION_RSA_TEXT
RSA_ERR_CODE hal_rtl_rsa_compute(hal_rsa_adapter_t *rsa_adp, uint32_t *result)
{

	//RSA_TypeDef* RSA_OBJ = (RSA_TypeDef*)(rsa_comm_adapter->base_addr);

	HW_RSA->Start_and_interrupt_control_field.Interrupt_enable = 1;
	HW_RSA->Start_and_interrupt_control_field.Go = 1;

//    RSA_OBJ->RSA_START_AND_INTERRUPT_CONTROL_FIELD |= RSA_BIT_INTERRUPT_ENABLE;
//    RSA_OBJ->RSA_START_AND_INTERRUPT_CONTROL_FIELD |= RSA_BIT_GO;



//    while ( ((RSA_OBJ->RSA_START_AND_INTERRUPT_CONTROL_FIELD & RSA_BIT_INTERRUPT_STATUS_CLEAR) >> RSA_SHIFT_INTERRUPT_STATUS_CLEAR) == 0 );
	while (HW_RSA->Start_and_interrupt_control_field.Interrupt_status_clear == 0);

	for (uint16_t i = 0; i < rsa_key_word_cnt; i ++) {
		result[i] = RSA_Data_Buffer->X[i];
	}

	rsa_adp->result = result;

	RSA_ERR_CODE status = hal_rtl_rsa_check_status(rsa_adp);

	/* bcuz hw rsa's interrupt and status are cleared at once, read status before clearing */
//    RSA_OBJ->RSA_START_AND_INTERRUPT_CONTROL_FIELD |= RSA_BIT_INTERRUPT_STATUS_CLEAR;
	HW_RSA->Start_and_interrupt_control_field.Interrupt_status_clear = 1;

	return status;
}

SECTION_RSA_TEXT
int hal_rtl_rsa_config(hal_rsa_adapter_t *rsa_adp, RSA_KEY_SEL_TYPE type, RSA_ENDIAN_TYPE byte_endian, RSA_ENDIAN_TYPE endian,
					   uint32_t *M, uint32_t *e, uint32_t *N, uint32_t exp_word_cnt)
{

	if (!((type == 0) | (type == 2) | (type == 5))) {
		DBG_RSA_ERR("Input key type wrong number %x\r\n", type);
		return FAIL;
	}

	if (byte_endian > 1) {
		DBG_RSA_ERR("Input endian wrong number %x\r\n", byte_endian);
		return FAIL;
	}

	if (endian > 1) {
		DBG_RSA_ERR("Input endian wrong number %x\r\n", endian);
		return FAIL;
	}

	if ((M == NULL) || (e == NULL) || (N == NULL)) {
		DBG_RSA_ERR("Invaild parameters\r\n");
		return FAIL;
	}

	rsa_adp->key_type = type;
	rsa_adp->byte_endian = byte_endian;
	rsa_adp->endian = endian;

	rsa_adp->exp_word_cnt = exp_word_cnt;
	rsa_adp->Message = M;
	rsa_adp->Exponent = e;
	rsa_adp->Modulus = N;


	return SUCCESS;
}



SECTION_RSA_TEXT
int hal_rtl_rsa_process(hal_rsa_adapter_t *rsa_adp, uint32_t *result)
{
	int ret;

	ret = hal_rtl_rsa_set_key_size(rsa_adp);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = hal_rtl_rsa_set_byte_swap(rsa_adp);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = hal_rtl_rsa_set_endian(rsa_adp);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = hal_rtl_rsa_set_operands(rsa_adp);
	if (ret != SUCCESS) {
		return ret;
	}
	ret = hal_rtl_rsa_compute(rsa_adp, result);
	if (ret != SUCCESS) {
		return ret;
	}

	return ret;
}

#if !defined(CONFIG_BUILD_NONSECURE)
SECTION_RSA_TEXT
void hal_rtl_rsa_clock_init(int en)
{
	hal_rtl_sys_peripheral_en(RSA_SYS, en);
}
#endif




#endif  // end of "#if CONFIG_RSA_EN"

