#include "rom_faac_func_stubs.h"
#include "basic_types.h"

extern const float _pow43_res_table[];
extern const float _adj43_res_table[];

SECTION(".rom.faac.stubs")
const faac_func_stubs_t faac_func_stubs = {
	.pow43_res_table = _pow43_res_table,
	.adj43_res_table = _adj43_res_table
};