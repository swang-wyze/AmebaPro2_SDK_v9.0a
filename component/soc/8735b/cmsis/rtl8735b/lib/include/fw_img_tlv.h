/**************************************************************************//**
 * @file     fw_img_tlv.h
 * @brief    This file defines the image tlv(type-length-value) format for boot flow and some secure info type
 *           for secure boot.
 *
 * @version  V1.00
 * @date     2021-08-03
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

#ifndef _FW_IMG_TLV_H_
#define _FW_IMG_TLV_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#define IMG_VRF_PBK_MAX_SIZE                        (384)
#define IMG_VRF_SIGNDATA_MAX_SIZE                   (384)
#define IMG_HASH_CHK_DIGEST_MAX_SIZE                (64)
#define IMG_HDR_START_ALIGN_SHIFT                   (12)
#define IMG_HDR_START_ALIGN_SIZE                    (0x1 << IMG_HDR_START_ALIGN_SHIFT)
#define IMG_HDR_START_ALIGN_MASK                    (IMG_HDR_START_ALIGN_SIZE - 0x1)
#define SB_PBK_HASH_SIZE                            (32)
#define SB_HUK_SIZE                                 (32)
#define SB_HKDF_KEY_SALT_SIZE                       (32)
#define SB_HKDF_KEY_NONCE_SIZE                      (32)
#define SB_HKDF_HMAC_SHA256_SIZE                    (32)
#define SB_CRYPTO_MAX_MSG_LENGTH                    (65536)
#define SB_SEC_KEY_SIZE                             (32)
#define SB_SEC_INIT_ADDR                            (0xFFFFFFFF)

#define MAX_CERTI_TBL_RECORD                        (2)
#define KEYCERTI_TBL_HDR_RESV_SIZE                  (12)
#define KEYCERTI_TBL_RECRD_RESV_SIZE                (5)
#define KEYCERTI_HDR_SIZE                           (16)
#define KEYCERTI_ROTPKDATA_SIZE                     (IMG_VRF_PBK_MAX_SIZE)
#define KEYCERTI_SIGNDATA_SIZE                      (IMG_VRF_SIGNDATA_MAX_SIZE)
#define MAX_KEYCERTI_HSH_TBL_RECORD                 (12)
#define KEYCERTI_PKHASH_SIZE                        (32)
#define KEYCERTI_SJTAG_KEY_SALT_SIZE                (32)
#define KEYCERTI_SJTAG_KEY_NONCE_SIZE               (32)
#define SB_SJTAG_KEY_SIZE                           (32)

#define KEYCERTI_RESV1_SIZE                         (13)
#define KEYCERTI_RESV2_SIZE                         (12)
#define KEYCERTI_RESV3_SIZE                         (15)
#define KEYCERTI_HSH_TBL_RESV_SIZE                  (6)
#define KEYCERTI_TYPEID_SIZE                        (2)
#define KEYCERTI_VERSION_SIZE                       (2)
#define KEYCERTI_TIMST_SIZE                         (8)


#define MANIFEST_HDR_SIZE                           (32)
//#define MANIFEST_HDR_RESERV1_SIZE                   (4)
#define MANIFEST_HDR_RESERV2_SIZE                   (16)
#define MANIFEST_IEDATA_SIZE                        (704)
#define MANIFEST_SIGNDATA_SIZE                      (IMG_VRF_SIGNDATA_MAX_SIZE)
#define MANIFEST_TOTAL_SIZE                         (MANIFEST_HDR_SIZE+MANIFEST_IEDATA_SIZE+MANIFEST_SIGNDATA_SIZE)
#define MAX_SEC_ENC_RECORD                          (8)
#define MANIFEST_MAX_LABEL_SIZE                     (8)
#define MANIFEST_IE_TBL_MAX_NUM                     (88)
#define MANIFEST_IE_ID_MAX_NUM                      (22)
#define MANIFEST_IE_ALIGNED_SIZE                    (4)
#define MANIFEST_IE_RESERV_SIZE                     (32)
#define MANIFEST_UNIE_START_ALIGN_SHIFT             (11)
#define MANIFEST_UNIE_START_ALIGN_SIZE              (0x1 << MANIFEST_UNIE_START_ALIGN_SHIFT)
#define MANIFEST_UNIE_CHK_SIZE                      (4)
#define IMG_MANIFEST_IE_TYPEID_SIZE                 (2)
#define IMG_MANIFEST_IE_VERSION_SIZE                (32)
#define IMG_MANIFEST_IE_TIMEST_SIZE                 (8)
#define IMG_MANIFEST_LD_SEL_MAX_IE_CNT              (3)

#define PARTITION_TABLE_ID_MAX_NUM                  (12)
#define PARTITION_RECORD_MAX                        (12)
#define PARTITION_TBL_FST_SIZE                      (32)
#define PARTITION_TBL_RECORD_SIZE                   (32*PARTITION_RECORD_MAX)
#define PARTITION_TBL_USER_DATA_SIZE                (256)
#define PARTITION_TBL_FST_RESV1_SIZE                (8)
#define PARTITION_TBL_FST_RESV2_SIZE                (8)
#define PARTITION_TBL_RECORD_RESV1_SIZE             (5)
#define PARTITION_TBL_RECORD_RESV2_SIZE             (16)

#define FW_IMG_HDR_MAX_SIZE                         (256)

#define FW_JTAG_KEY_SIZE                            (32)
#define FW_HDR_RESV1_SIZE                           (4)
#define FW_HDR_RESV2_SIZE                           (3)
#define FW_HDR_RESV3_SIZE                           (4)
#define FW_HDR_RESV4_SIZE                           (4)
#define FW_HDR_RESV5_SIZE                           (4)
#define FW_HDR_RESV6_SIZE                           (16)
#define FW_HDR_RESV7_SIZE                           (16)
#define FW_HDR_NXTOFFSET_NULL                       (0xFFFFFFFF)

#define SECT_HDR_RESV2_SIZE                         (4)
#define SECT_HDR_RESV3_SIZE                         (4)
#define SECT_HDR_RESV4_SIZE                         (4)
#define SECT_HDR_RESV5_SIZE                         (16)
#define SECT_HDR_RESV6_SIZE                         (16)
#define SECT_HDR_SEC_INFO_SIZE                      (32)
#define SECT_HDR_NXTOFFSET_NULL                     (0xFFFFFFFF)
//#define BL_SECT_SECLEN                              (50319)

enum {
	LD_IMF_FT_TLV    =   0x0,
	LD_IMF_FT_NTLV   =   0x1,
};

enum {
	IMG_SEL_LD       =   0x0,
	IMG_SEL_UPDATE   =   0x1,
};

enum {
	LD_SEL_IMG_FW           =   0x0,
	LD_SEL_IMG_BOOT         =   0x1,
	LD_SEL_IMG_KEYCERTI     =   0x2,
};

enum {
	OTA_UPDATE_IMG_FW       =   0x0,
	OTA_UPDATE_IMG_BOOT     =   0x1,
	OTA_UPDATE_IMG_KEYCERTI =   0x2,
};

enum {
	OTA_UPDATE_IMG_NO_SPECI_IDX =   0x0,
	OTA_UPDATE_IMG_SPECI_IDX1   =   0x1,
	OTA_UPDATE_IMG_SPECI_IDX2   =   0x2,
};

typedef enum {
	FW_PT_INI_VAL_ID       = 0xF8E0,
	FW_PT_KEY_CER_TBL_ID   = 0xF1C1,
	FW_PT_KEY_CER1_ID      = 0xE9C2,
	FW_PT_KEY_CER2_ID      = 0xE1C3,
	FW_PT_PT_ID            = 0xD9C4,
	FW_PT_BL_PRI_ID        = 0xD1C5,
	FW_PT_BL_SEC_ID        = 0xC9C6,
	FW_PT_FW1_ID           = 0xC1C7,
	FW_PT_FW2_ID           = 0xB9C8,
	FW_PT_ISP_IQ_ID        = 0x89CE,
	FW_PT_NN_MDL_ID        = 0x81CF,
	FW_PT_NAND_CTRL_ID     = 0x79D0,
	FW_PT_NAND_BBT_ID      = 0x71D1,
	FW_PT_RESV_ID          = 0x01DF,
} PART_TYPE_ID_T;

typedef enum {
	IE_PK_ID        = 0x01,
	IE_VERSION_ID   = 0x02,
	IE_IMGSZ_ID     = 0x03,
	IE_TYPEID_ID    = 0x04,
	IE_ENCALG_ID    = 0x05,
	IE_ENCKN_ID     = 0x06,
	IE_ENCKS_ID     = 0x07,
	IE_ENCIV_ID     = 0x08,
	IE_HSHALG_ID    = 0x09,
	IE_HSHKN_ID     = 0x0A,
	IE_HSHKS_ID     = 0x0B,
	IE_HASH_ID      = 0x0C,
	IE_TIMST_ID     = 0x0D,
	IE_VID_ID       = 0x0E,
	IE_PID_ID       = 0x0F,
	IE_IMGLST_ID    = 0x10,
	IE_DEP_ID       = 0x11,
	IE_RMATKN_ID    = 0x12,
	IE_BATLV_ID     = 0x13,
	IE_ACPW_ID      = 0x14,
	IE_IERESV_ID    = 0x15,
	IE_NAN_ID       = 0xFF,
} IE_ID_T;


#define    IE_TLV_TYPE_ID_SIZE     (1)
#define    IE_TLV_SIZE_SIZE        (3)
#define    IE_TLV_TL_TOTAL_SIZE    (4)


enum {
	IE_DATA_PADDING_ONEBYTE =   0x1,
	IE_DATA_PADDING_TWOBYTE =   0x2,
	IE_DATA_PADDING_THRBYTE =   0x3,
};

enum {
	IMG_PARSE_BL =   0x1,
	IMG_PARSE_FW =   0x2
};

enum {
	IMG_LOAD_BL     =   0x1,
	IMG_LOAD_FW     =   0x2,
	IMG_LOAD_ISP_IQ =   0x3
};

enum {
	INFO_INVALID =   0x0,
	INFO_VALID   =   0x1
};

enum {
	IMG_SIGN_VRF_ALG_NONE          =   0x0,
	IMG_SIGN_VRF_ALG_HMAC_SHA256   =   0x1,
	IMG_SIGN_VRF_ALG_EDDSA_ED25519 =   0x2,
	IMG_SIGN_VRF_ALG_ECDSA256      =   0x3,
	IMG_SIGN_VRF_ALG_RSA2048       =   0x4,
	IMG_SIGN_VRF_ALG_RSA3072       =   0x5,
};

enum {
	IMG_HSH_CHK_ALG_NONE           =   0x0,
	IMG_HSH_CHK_ALG_SHA256         =   0x1,
	IMG_HSH_CHK_ALG_HMAC_SHA256    =   0x2
};

enum {
	IMG_SEC_ENC_ALG_NONE           =   0xFF,
	IMG_SEC_ENC_ALG_AES256_GCM     =   0x1,
	IMG_SEC_ENC_ALG_AES256_CTR     =   0x2,
	IMG_SEC_ENC_ALG_AES256_ECB_MIX =   0x3,
};

typedef enum {
	FW_IMG_INI_VAL_ID       = 0xF8E0,
	FW_IMG_KEY_CER_ID       = 0xF2A1,
	FW_IMG_PT_ID            = 0xEAA2,
	FW_IMG_BL_ID            = 0xE2A3,
	FW_IMG_FWHS_S_ID        = 0xDAA4,
	FW_IMG_FWHS_NS_ID       = 0xD2A5,
	FW_IMG_ISP_ID           = 0xCAA6,
	FW_IMG_VOE_ID           = 0xC2A7,
	FW_IMG_WLAN_ID          = 0xBAA8,
	FW_IMG_XIP_ID           = 0xB2A9,
	FW_IMG_CPFW_ID          = 0xAAAA,
	FW_IMG_WOWLN_ID         = 0xA2AB,
	FW_IMG_CINIT_ID         = 0x9AAC,
	FW_IMG_FWUPD_ID         = 0x92AD,
	FW_IMG_RESV_ID          = 0x02BF,
	FW_IMG_NOTSET_ID        = 0xFFFF,

} FW_IMG_TYPE_ID_T;

typedef enum {
	FW_SIMG_INI_VAL_ID      = 0xF8E0,
	FW_SIMG_DTCM_ID         = 0xF381,
	FW_SIMG_ITCM_ID         = 0xEB82,
	FW_SIMG_SRAM_ID         = 0xE383,
	FW_SIMG_PSRAM_ID        = 0xDB84,
	FW_SIMG_LPDDR_ID        = 0xD385,
	FW_SIMG_DDR_ID          = 0xCB86,
	FW_SIMG_XIP_ID          = 0xC387,
	FW_SIMG_RESV_ID         = 0x039F,

} FW_SECT_TYPE_ID_T;

typedef struct certi_tbl_record_s {
	uint32_t start_addr;
	uint32_t length;
	uint16_t type_id;
	uint8_t  resv[5];
	uint8_t  valid;
} certi_tbl_record_t, *pcerti_tbl_record_t;

typedef struct certi_tbl_s {
	uint8_t certi1_idx;
	uint8_t certi2_idx;
	uint8_t resv[2];
	certi_tbl_record_t  key_cer_tbl_rec[MAX_CERTI_TBL_RECORD];
} certi_tbl_t, *pcerti_tbl_t;

typedef struct img_keycerti_tbl_s {
	uint16_t type_id;
	uint16_t vrf_al;
	uint8_t  resv[12];
	certi_tbl_record_t  key_cer_tbl_rec[MAX_CERTI_TBL_RECORD];
} img_keycerti_tbl_t, *pimg_keycerti_tbl_t;

typedef struct key_certi_rotpk_s {
	uint8_t data[KEYCERTI_ROTPKDATA_SIZE];
} key_certi_rotpk_t, *pkey_certi_rotpk_t;

typedef struct key_certi_hsh_tbl_s {
	uint16_t type_id;
	uint8_t  resv[6];
} key_certi_hsh_tbl_t, *pkey_certi_hsh_tbl_t;

typedef struct key_certi_pkhsh_s {
	uint8_t data[KEYCERTI_PKHASH_SIZE];
} key_certi_pkhsh_t, *pkey_certi_pkhsh_t;

typedef struct key_certi_sign_s {
	uint8_t data[KEYCERTI_SIGNDATA_SIZE];
} key_certi_sign_t, *pkey_certi_sign_t;

typedef struct s_jtag_sec_nonfixed_key_s {
	uint8_t kn[KEYCERTI_SJTAG_KEY_NONCE_SIZE];
	uint8_t ks[KEYCERTI_SJTAG_KEY_SALT_SIZE];
} s_jtag_sec_nonfixed_key_t, *ps_jtag_sec_nonfixed_key_t;

typedef struct s_jtag_nsec_nonfixed_key_s {
	uint8_t kn[KEYCERTI_SJTAG_KEY_NONCE_SIZE];
	uint8_t ks[KEYCERTI_SJTAG_KEY_SALT_SIZE];
} s_jtag_nsec_nonfixed_key_t, *ps_jtag_nsec_nonfixed_key_t;

typedef struct img_keycerti_s {
	uint8_t  rotpk_hsh_idx;
	uint8_t  huk_idx;
	uint8_t  sec_key_idx;
	uint8_t  resv1[13];
	key_certi_rotpk_t   rotpk;
	uint16_t type_id;
	uint16_t key_version;
	uint8_t  timest[8];
	uint32_t imgsz;
	uint32_t s_jtag_ctrl;
	uint8_t  resv2[12];
	s_jtag_sec_nonfixed_key_t jtag_s_nonfixed_key_info;
	s_jtag_nsec_nonfixed_key_t jtag_ns_nonfixed_key_info;
	uint8_t  key_hash_rec_num;
	uint8_t  resv3[15];
	key_certi_hsh_tbl_t key_hsh_tbl[MAX_KEYCERTI_HSH_TBL_RECORD];
	key_certi_pkhsh_t   key_pkhsh[MAX_KEYCERTI_HSH_TBL_RECORD];
	key_certi_sign_t    signature;
} img_keycerti_t, *pimg_keycerti_t;

typedef struct ie_tlv_const_tbl_s {
	IE_ID_T   ie_id;
	char      *ie_name;
	uint32_t  max_size;
} ie_tlv_const_tbl_t, *pie_tlv_const_tbl_t;

typedef struct part_type_id_tbl_s {
	PART_TYPE_ID_T   id;
	char             *name;
} part_type_id_tbl_t, *ppart_type_id_tbl_t;


typedef struct manif_ie_tlv_s {
	uint8_t  id;
	uint8_t  resv[3];
	uint32_t size;
	uint8_t *p_val;
} manif_ie_tlv_t, *pmanif_ie_tlv_t;

typedef struct manif_ie_tlv_tbl_s {
	manif_ie_tlv_t tbl_info[MANIFEST_IE_TBL_MAX_NUM];
} manif_ie_tlv_tbl_t, *pmanif_ie_tlv_tbl_t;

typedef struct manif_hdr_s {
	uint8_t  lbl[MANIFEST_MAX_LABEL_SIZE];
	uint16_t size;
	uint16_t vrf_al;
	uint32_t enc_rmp_base_addr;
	uint8_t resv2[MANIFEST_HDR_RESERV2_SIZE];
} manif_hdr_t, *pmanif_hdr_t;

typedef struct manif_sec_enc_record_s {
	uint8_t enc_en;
	uint8_t encalg_sel;
	uint8_t add_ref;
	uint8_t xip_en;

	uint8_t resv2[4];
	uint8_t tag_size_sel;
	uint8_t resv3[3];
	uint32_t tag_base_addr;
	uint32_t iv_ptn_low;
	uint32_t iv_ptn_high;
	uint32_t base_addr;
	uint32_t end_addr;
} manif_sec_enc_record_t, *pmanif_sec_enc_record_t;

typedef struct manif_ie_data_s {
	uint8_t data[MANIFEST_IEDATA_SIZE];
} manif_ie_data_t, *pmanif_ie_data_t;

typedef struct manif_sign_s {
	uint8_t data[MANIFEST_SIGNDATA_SIZE];
} manif_sign_t, *pmanif_sign_t;

typedef struct img_manifest_s {
	manif_hdr_t             hdr;
	manif_sec_enc_record_t  sec_enc_record[MAX_SEC_ENC_RECORD];
	manif_ie_data_t         ie_data;
	manif_sign_t            signature;
} img_manifest_t, *pimg_manifest_t;

typedef struct img_manifest_ld_sel_s {
	uint16_t type_id;
	uint8_t  version[IMG_MANIFEST_IE_VERSION_SIZE];
	uint8_t  timest[IMG_MANIFEST_IE_TIMEST_SIZE];
} img_manifest_ld_sel_t, *pimg_manifest_ld_sel_t;

typedef struct gpio_pwr_on_trap_pin_s {
	uint16_t pin: 5;            /*!< bit: 4...0 the GPIO pin number */
	uint16_t port: 3;           /*!< bit: 7...6 the  GPIO port number */
	uint16_t io_lev: 1;         /*!< bit:  8 the IO level to trigger the trap */
	uint16_t reserved: 6;       /*!< bit: 14...9 reserved */
	uint16_t valid: 1;          /*!< bit:  15 is this trap valid */
} gpio_pwr_on_trap_pin_t;

typedef struct part_fst_info_s {
	uint8_t rec_num;
	uint8_t bl_p_idx;
	uint8_t bl_s_idx;
	uint8_t fw1_idx;
	uint8_t fw2_idx;
	uint8_t iq_idx;
	uint8_t nn_m_idx;
	uint8_t mp_idx;
	uint8_t resv1[8];
	gpio_pwr_on_trap_pin_t ota_trap;
	gpio_pwr_on_trap_pin_t mp_trap;
	uint32_t udl;
	uint8_t resv2[8];
} part_fst_info_t, *ppart_fst_info_t;

typedef struct part_record_t_s {
	uint32_t start_addr;        /*!< The start address of the image partition, it should be 4K-bytes aligned */
	uint32_t length;            /*!< The size of the image partition, it should be times of 4K-bytes */
	uint16_t type_id;           /*!< t he image type of the partition */
	uint8_t  resv1[5];          /*!< reserved */
	uint8_t  valid;
	uint8_t  resv2[16];         /*!< reserved */
} part_record_t, *ppart_record_t;

typedef struct partition_tbl_s {
	part_fst_info_t     fst;
	part_record_t       partition_record[PARTITION_RECORD_MAX];
} partition_tbl_t, *ppartition_tbl_t;

typedef struct part_user_data_s {
	uint8_t data[PARTITION_TBL_USER_DATA_SIZE];
} part_user_data_t, *ppart_user_data_t;

typedef struct img_partition_tbl_s {
	img_manifest_t      manifest;
	partition_tbl_t     tbl_info;
	part_user_data_t    user_data;
} img_partition_tbl_t, *pimg_partition_tbl_t;

typedef struct fw_img_jtag_key_s {
	uint8_t s_jtag_s_key[FW_JTAG_KEY_SIZE];
	uint8_t s_jtag_ns_key[FW_JTAG_KEY_SIZE];
} fw_img_jtag_key_t, *pfw_img_jtag_key_t;

typedef struct fw_img_hdr_s {
	uint32_t imglen;
	uint32_t nxtoffset;
	uint16_t type_id;
	uint16_t nxt_type_id;
	uint8_t  s_jtag_ctrl;
	uint8_t  resv2[3];
	uint8_t  resv3[4];
	uint8_t  resv4[4];
	uint32_t str_tbl;
	uint8_t  resv5[4];
	fw_img_jtag_key_t   jtag_key;
	uint8_t  resv6[16];
	uint8_t  resv7[16];
} fw_img_hdr_t, *pfw_img_hdr_t;

typedef struct img_fw_img_s {
	img_manifest_t      manifest;
	fw_img_hdr_t        fw_img_hdr;
} img_fw_img_t, *pfw_img_t;

typedef struct sec_ctrl_s {
	uint32_t ctrl_val;
} sec_ctrl_t, *psec_ctrl_t;

typedef struct sec_info_s {
	uint8_t sec_info1[32];
	uint8_t sec_info2[32];
} sec_info_t, *psec_info_t;

typedef struct sect_hdr_s {
	uint32_t    seclen;
	uint32_t    nxtoffset;
	uint16_t    type_id;
	uint16_t    resv1;
	sec_ctrl_t  sec_ctrl;
	uint8_t     resv2[4];
	uint32_t    dest;
	uint8_t     resv3[4];
	uint8_t     resv4[4];
	sec_info_t  sec_info;
	uint8_t     resv5[16];
	uint8_t     resv6[16];
} sect_hdr_t, *psect_hdr_t;

typedef struct img_pkhsh_info_s {
	key_certi_hsh_tbl_t key_hsh_tbl;
	key_certi_pkhsh_t   key_pkhsh;
} img_pkhsh_info_t, *pimg_pkhsh_info_t;

typedef struct sec_boot_keycerti_s {
	uint8_t  rotpk_hsh_idx;
	uint8_t  huk_idx;
	uint8_t  sec_key_idx;
	uint8_t  resv1[13];
	uint8_t  rotpk_hsh_data[SB_PBK_HASH_SIZE];
	uint32_t s_jtag_ctrl;
	uint8_t  resv2[12];
	s_jtag_sec_nonfixed_key_t jtag_s_nonfixed_key_info;
	s_jtag_nsec_nonfixed_key_t jtag_ns_nonfixed_key_info;
	uint8_t  key_hash_rec_num;
	uint8_t  resv3[15];
	img_pkhsh_info_t img_pkhsh_info[MAX_KEYCERTI_HSH_TBL_RECORD];
} sec_boot_keycerti_t, *psec_boot_keycerti_t;

typedef struct tb_img_sign_vrf_info_s {
	uint8_t vrf_alg;
	uint8_t *p_pbk;
	uint8_t *p_sign;
	uint8_t *p_msg;
	uint32_t msglen;
} tb_img_sign_vrf_info_t, *ptb_img_sign_vrf_info_t;

typedef struct tb_img_hash_chk_info_s {
	uint8_t hash_alg;
	uint8_t *p_key_huk;
	uint8_t *p_key_salt;
	uint8_t *p_key_nonce;
	uint8_t *p_msg;
	uint8_t *p_digest;
	uint32_t msglen;
} tb_img_hash_chk_info_t, *ptb_img_hash_chk_info_t;

typedef struct sb_sec_enc_info_s {
	uint32_t enc_rmp_base_addr;
	uint8_t *p_msg;
	uint8_t enc_sts;
	uint8_t resv[3];
	//uint32_t msglen;
	manif_sec_enc_record_t sec_enc_record[MAX_SEC_ENC_RECORD];
} sb_sec_enc_info_t, *psb_sec_enc_info_t;

/**
  \brief  The data structure for a SEC remap configuration of a XIP memory block.
*/
typedef struct hal_sec_rmp_region_cfg_s {
	uint8_t is_used;
	uint8_t resv[3];
	uint32_t phy_addr;      // The image physical address (flash image offset)
	uint32_t remap_addr;    // The re-mapping address (image virtual address)
	uint32_t remap_size;    // re-mapping size
} hal_sec_rmp_region_cfg_t, *phal_sec_rmp_region_cfg_t;

/**
  \brief  The data structure for a SEC decrypt configuration of a XIP memory block.
*/
typedef struct hal_sec_dec_region_cfg_s {
	uint8_t is_used;
	uint8_t cipher_sel;
	uint8_t resv[2];
	uint32_t iv_ptn_low;
	uint32_t iv_ptn_high;
	uint32_t dec_base;      // The image decrypt address (flash address)
	uint32_t dec_size;      // decrypt size
	uint32_t tag_base_addr; // aes-gcm flash tag_base_addr
	uint32_t tag_flh_addr;  // aes-gcm flash tag located addr
	uint32_t hdr_total_size;// IMG header total size
} hal_sec_dec_region_cfg_t, *phal_sec_dec_region_cfg_t;

#define MAX_PENDING_SEC_RMP_REGION_CFG          (4)
#define MAX_PENDING_SEC_DEC_REGION_CFG          (8)
#define IMG_REGION_LOOKUP_TBL_MAX_SIZE          (10)
#define RGN_IDX_INITVAL                         (0xFF)

typedef struct img_region_lookup_s {
	uint8_t  is_xip;
	uint8_t  rng_idx;
} img_region_lookup_t, *pimg_region_lookup_t;

typedef struct hal_sec_region_cfg_f_s {
	hal_status_t (*enable_remap_region)(uint8_t rmp_region_sel,
										uint32_t va_base, uint32_t pa_base, uint32_t region_size);
	hal_status_t (*disable_rmp_region)(uint8_t rmp_region_sel);
	hal_status_t (*default_decrypt_init)(const uint8_t key_sel);
	hal_status_t (*decrypt_region_init)(const uint32_t iv_low_ptn, const uint32_t iv_high_ptn, uint8_t dec_region_sel);
	hal_status_t (*decrypt_region_enable)(const uint8_t cipher_sel,
										  uint32_t dec_base, uint32_t dec_size, uint32_t tag_base);
	hal_status_t (*disable_dec_region)(uint8_t dec_region_sel);
	void (*aes_disable)(void);
	uint32_t (*set_word_from_byte_bigen)(const unsigned char *s_value);
	hal_status_t (*default_calculate_tag_base)(uint32_t flash_addr, uint32_t region_size,
			uint32_t tag_region_addr, uint32_t *tag_base,
			uint32_t *tag_region_size);
	uint32_t resv[3];
} hal_sec_region_cfg_f_t, *phal_sec_region_cfg_f_t;

/**
  \brief  The data structure for pending SEC configuration job.
*/
typedef struct hal_sec_region_cfg_s {
	uint8_t key_set;
	uint8_t key_id;
	uint8_t rmp_not_set_sts;
	uint8_t dec_not_set_sts;

	uint8_t rmp_cfg_cnt;    // the configured memory block count
	uint8_t dec_cfg_cnt;    // the configured memory block count
	uint8_t rmp_sts;
	uint8_t dec_sts;

	hal_sec_rmp_region_cfg_t sec_rmp_region_cfg[MAX_PENDING_SEC_RMP_REGION_CFG];
	hal_sec_dec_region_cfg_t sec_dec_region_cfg[MAX_PENDING_SEC_DEC_REGION_CFG];
	hal_sec_region_cfg_f_t sec_cfg_f;
} hal_sec_region_cfg_t, *phal_sec_region_cfg_t;

typedef struct sec_boot_info_s {
	uint32_t sb_level;
	sec_boot_keycerti_t *psb_keycerti;
	sb_sec_enc_info_t *pimg_sec_enc_info;
	hal_sec_region_cfg_t *psec_region_ctrl;
	tb_img_sign_vrf_info_t img_sign_vrf_info;
	tb_img_hash_chk_info_t img_hash_chk_info;

} sec_boot_info_t, *psec_boot_info_t;

typedef enum {
	KEY_CERTI_INFO    = 0x1,
	PT_TBL_INFO       = 0x2,
	BL_INFO           = 0x3,
	FW_IMG_INFO       = 0x4,
	FW_ISP_INFO       = 0x5,
	FW_VOE_INFO       = 0x6

} IMG_INFO_T;

enum {
	SB_HIGH_VAL_RTOPK_HSH       =   0x1,
	SB_HIGH_VAL_HUK             =   0x2,
	SB_HIGH_VAL_SEC_KEY         =   0x3,
	SB_HIGH_VAL_SJTAG_S_KEY     =   0x4,
	SB_HIGH_VAL_SJTAG_NS_KEY    =   0x5
};

enum {
	SB_HIGH_VAL_IDX1      =   0x1,
	SB_HIGH_VAL_IDX2      =   0x2,
	SB_HIGH_VAL_RMA_IDX   =   0x3
};

enum {
	ROTPK_IDX1      =   0x1,
	ROTPK_IDX2      =   0x2,
	ROTPK_RMA_IDX   =   0x3
};

enum {
	HUK_IDX1        =   0x1,
	HUK_IDX2        =   0x2,
	HUK_RMA_IDX     =   0x3
};

enum {
	SEC_KEY_IDX1    =   0x1,
	SEC_KEY_IDX2    =   0x2,
	SEC_KEY_RMA_IDX =   0x3
};

enum {
	TOP_SEC_KEY1_LD_SEC_KEY_ALL  =   0x0,
	TOP_SEC_KEY2_LD_SEC_KEY_ALL  =   0x1,
	TOP_SEC_KEY_LD_SEC_KEY_MATCH =   0x2,
	TOP_SEC_KEY_LD_SEC_KEY_CROSS =   0x3
};


typedef struct part_tbl_info_s {

#if 0
	// not sure members
	uint16_t mani_size;
	uint16_t vrf_al;
	manif_ie_tlv_t *ie_cache_tbl;
	uint8_t *ie_raw_data;
	manif_sign_t *manif_sign;
#endif
	partition_tbl_t *part_tbl_raw_data;

} part_tbl_info_t, *ppart_tbl_info_t;

typedef struct hal_flash_boot_tlv_stubs_s {
	void *ppartition_tbl;
	void *pkeycerti;
	uint8_t *fast_boot;
	void (*clear_export_partition_tbl_info)(void);
	void (*clear_export_sb_keycerti_info)(void);
	void (*erase_boot_loader)(uint32_t code_start, uint32_t code_size, uint32_t img2_entry);
	int32_t (*otu_fw_download)(hal_uart_adapter_t *potu_uart, uint32_t flash_sel, uint32_t flash_offset);
	int (*verify_manif_f)(const uint8_t *img_offset, const uint8_t info_type, sec_boot_info_t *p_sb_info);
	void (*load_img_hdr_f)(const uint8_t *img_offset, fw_img_hdr_t *img_hdr_buf, const uint8_t sel_img_load);
	uint8_t (*search_available_idx)(uint8_t *p_sts, const uint8_t cfg_max_size);
	int (*xip_pending_cfg_add_rmp)(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_rmp_lkp_tbl,
								   uint32_t phy_addr, uint32_t remap_addr, uint32_t remap_sz);
	int (*xip_pending_process_rmp)(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info);
	int (*xip_disable_rmp_config)(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel);

	int (*xip_pending_cfg_add_dec_key)(hal_sec_region_cfg_t *psb_sec_cfg_pending, const uint8_t key_id);
	int (*xip_pending_cfg_add_dec)(hal_sec_region_cfg_t *psb_sec_cfg_pending, img_region_lookup_t *pimg_dec_lkp_tbl,
								   uint8_t cipher_sel, uint32_t dec_base, uint32_t dec_sz,
								   uint32_t iv_ptn_low, uint32_t iv_ptn_high,
								   uint32_t tag_base_addr, uint32_t tag_flh_addr, uint32_t total_hdr_size);
	int (*xip_pending_process_dec)(hal_sec_region_cfg_t *psb_sec_cfg_pending, sec_boot_info_t *p_sb_info);
	int (*xip_disable_dec_config)(hal_sec_region_cfg_t *psb_sec_cfg_pending, uint8_t region_sel);
	int (*img_rmp_and_dec_lkp_tbl_insert)(img_region_lookup_t *plkp_tbl, uint8_t tbl_size, uint8_t is_xip, uint8_t *tbl_cnt);
	void (*img_rmp_and_dec_lkp_tbl_remove)(img_region_lookup_t *plkp_tbl, uint8_t *tbl_cnt);
	int (*img_get_ld_sel_info_from_ie)(const uint8_t img_obj, const uint8_t *ptr, img_manifest_ld_sel_t *pld_sel_info);
	uint32_t (*get_ld_version)(const uint8_t img_obj, uint8_t *p_img_version);
	uint32_t (*get_ld_timst)(uint8_t *p_img_timest);
	int (*img_get_ld_sel_idx)(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx,
							  uint8_t img2_idx);
	int (*img_get_update_sel_idx)(const uint8_t img_obj, img_manifest_ld_sel_t *pld_sel_info1, img_manifest_ld_sel_t *pld_sel_info2, uint8_t img1_idx,
								  uint8_t img2_idx);
	int (*img_sel_op_idx)(void *p_tbl_info, const uint8_t img_obj, const uint8_t img_sel_op);
	uint32_t reserved[15];  // reserved space for next ROM code version function table extending.
} hal_flash_boot_tlv_stubs_t;

int verify_manif_f(const uint8_t *img_offset, const uint8_t info_type, sec_boot_info_t *p_sb_info); //TOdo info_type may not use
hal_status_t fw_spic_init(phal_spic_adaptor_t phal_spic_adaptor, u8 spic_bit_mode, u8 io_pin_sel);
hal_status_t fw_spic_deinit(phal_spic_adaptor_t phal_spic_adaptor);


#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _FW_IMG_TLV_H_"
