#ifndef _GAP_ISO_DATA_H_
#define _GAP_ISO_DATA_H_

#include "platform_opts_bt.h"

#if UPPER_STACK_VERSION == VERSION_2021

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

bool gap_iso_data_cfm(void *p_buf);

T_GAP_CAUSE gap_iso_send_data(uint8_t *p_data, uint16_t handle, uint16_t iso_sdu_len, bool ts_flag,
							  uint32_t time_stamp, uint16_t pkt_seq_num);


#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif

#endif
