#include <drv_types.h>
#include "rtl8710c_bt.h"
#include "rtl8710c/rtl8710c_hal_efuse.h"

extern Rltk_wlan_t	rltk_wlan_info[NET_IF_NUM];
_WEAK const unsigned char rtlbt_mp_fw[] ={0};
_WEAK const unsigned char rtlbt_fw[] ={0};

const unsigned char *bt_get_patch_code_8710c(_adapter *padapter)
{
	if (Hal_BT_Is_Supported(padapter)) {
#ifdef CONFIG_MP_INCLUDED
		return rtlbt_mp_fw;
#else
		return rtlbt_fw;
#endif
	}
	else {
		return NULL;
	}
}

unsigned int bt_get_patch_code_len_8710c(_adapter *padapter)
{
	if (Hal_BT_Is_Supported(padapter)) {
#ifdef CONFIG_MP_INCLUDED
		return sizeof(rtlbt_mp_fw);
#else
		return sizeof(rtlbt_fw);
#endif
	}
	else {
		return 0;
	}
}