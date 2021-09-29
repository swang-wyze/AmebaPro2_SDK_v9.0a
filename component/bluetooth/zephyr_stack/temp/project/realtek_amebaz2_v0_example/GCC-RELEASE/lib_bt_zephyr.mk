
# Initialize tool chain
# -------------------------------------------------------------------
AMEBAZ2_TOOLDIR	      = $(COMP_DIR)/soc/8710c/misc/iar_utility
AMEBAZ2_BSPDIR        = $(COMP_DIR)/soc/8710c/misc/bsp
AMEBAZ2_BOOTLOADERDIR = $(AMEBAZ2_BSPDIR)/image
AMEBAZ2_ROMSYMDIR     = $(AMEBAZ2_BSPDIR)/ROM

OS := $(shell uname)

# Compilation tools
CROSS_COMPILE = $(ARM_GCC_TOOLCHAIN)/arm-none-eabi-
AR            = $(CROSS_COMPILE)ar
CC            = $(CROSS_COMPILE)gcc
AS            = $(CROSS_COMPILE)as
NM            = $(CROSS_COMPILE)nm
LD            = $(CROSS_COMPILE)gcc
GDB           = $(CROSS_COMPILE)gdb
OBJCOPY       = $(CROSS_COMPILE)objcopy
OBJDUMP       = $(CROSS_COMPILE)objdump


# Initialize target name and target object files
# -------------------------------------------------------------------
all: check_config lib_bt

TARGET   = lib_bt
OBJ_DIR  = $(TARGET)/Debug/obj
BIN_DIR  = $(TARGET)/Debug/bin
INFO_DIR = $(TARGET)/Debug/info

ROMIMG   =


# Include folder list
# -------------------------------------------------------------------
REL_PATH       = ../../..
COMP_DIR       = $(REL_PATH)/component
BT_DIR         = $(COMP_DIR)/bluetooth
BT_ZE_DIR      = $(BT_DIR)/zephyr_stack
INC_DIR        = $(BT_ZE_DIR)/include
STACK_DIR      = $(BT_ZE_DIR)/stack
EXAMP_DIR      = $(BT_ZE_DIR)/example
PLATFORM_DIR   = $(BT_ZE_DIR)/platform
KCONFIG_CONFIG = $(PLATFORM_DIR)/amebaz2/.config
KCONFIG_DIR    = $(shell pwd)/tools/kconfigtool_python
INCLUDES       = 

INCLUDES += \
	-I../inc \
	-I$(INC_DIR) \
	-I$(INC_DIR)/bluetooth/mesh \
	-I$(INC_DIR)/bluetooth/services \
	-I$(PLATFORM_DIR)/amebaz2 \
	-I$(STACK_DIR) \
	-I$(STACK_DIR)/common \
	-I$(STACK_DIR)/dependence/hci/btsnoop \
	-I$(STACK_DIR)/dependence/include \
	-I$(STACK_DIR)/dependence/include/toolchain \
	-I$(STACK_DIR)/dependence/settings/include \
	-I$(STACK_DIR)/host \
	-I$(STACK_DIR)/mesh \
	-I$(STACK_DIR)/services \
	-I$(STACK_DIR)/shell \
	-I$(STACK_DIR)/audio \

INCLUDES += \
	-I$(BT_DIR)/driver/hci \
	-I$(BT_DIR)/driver/inc \
	-I$(BT_DIR)/driver/platform/amebaz2/inc \

INCLUDES += \
	-I$(COMP_DIR)/stdlib \
	-I$(COMP_DIR)/os/os_dep/include \
	-I$(COMP_DIR)/os/os_dep/include \
	-I$(COMP_DIR)/os/freertos \
	-I$(COMP_DIR)/os/freertos/freertos_v10.0.1/Source/include \
	-I$(COMP_DIR)/os/freertos/freertos_v10.0.1/Source/portable/GCC/ARM_RTL8710C \
	-I$(COMP_DIR)/soc/8710c/app/shell \
	-I$(COMP_DIR)/soc/8710c/app/stdio_port \
	-I$(COMP_DIR)/soc/8710c/app/rtl_printf/include \
	-I$(COMP_DIR)/soc/8710c/cmsis/rtl8710c/lib/include \
	-I$(COMP_DIR)/soc/8710c/fwlib/include \
	-I$(COMP_DIR)/soc/8710c/fwlib/lib/include \
	-I$(COMP_DIR)/soc/8710c/cmsis/rtl8710c/include \
	-I$(COMP_DIR)/soc/8710c/cmsis/cmsis-core/include \
	-I$(COMP_DIR)/soc/8710c/mbed-drivers/include \
	-I$(COMP_DIR)/soc/8710c/misc/utilities/include \

.PHONY: check_config
check_config:
ifneq (, $(wildcard $(KCONFIG_CONFIG)))
include $(KCONFIG_CONFIG)
else
	$(error "Error: Please make lib_bt_menuconfig first to generate .config file")
endif

# Source file list
# -------------------------------------------------------------------

SRC_C  = 
# -------------------------- subdir common/ -------------------------
SRC_C += \
	$(STACK_DIR)/common/dummy.c \
	$(STACK_DIR)/common/log.c 

ifeq ($(CONFIG_BT_RPA), y)
	SRC_C += $(STACK_DIR)/common/rpa.c 
endif

# ----------------------- subdir dependence/ -------------------
SRC_C += \
	$(STACK_DIR)/dependence/hci/hci/hci_h4.c \
	$(STACK_DIR)/dependence/kernel/atomic_c.c \
	$(STACK_DIR)/dependence/kernel/buf.c \
	$(STACK_DIR)/dependence/kernel/hex.c \
	$(STACK_DIR)/dependence/kernel/kernel.c \
	$(STACK_DIR)/dependence/kernel/queue.c \
	$(STACK_DIR)/dependence/kernel/work_q.c \
	$(STACK_DIR)/dependence/stack/stack.c \

ifeq ($(CONFIG_BT_SHELL_PRIV), y)
	SRC_C += \
	$(STACK_DIR)/dependence/shell/priv.c
endif
ifeq ($(findstring y, $(CONFIG_BT_SHELL) $(CONFIG_BT_SHELL_PRIV)), y)
	SRC_C += \
	$(STACK_DIR)/dependence/shell/shell.c \
	$(STACK_DIR)/dependence/shell/shell_entry.c 
endif

# ----------------------- subdir dependence/hci/btsnoop/ -------------------
ifeq ($(CONFIG_BT_HCI_BTSNOOP), y)
	SRC_C += $(STACK_DIR)/dependence/hci/btsnoop/btsnoop.c 
endif

# ----------------------- subdir dependence/settings/ -------------------
SRC_C += \
	$(STACK_DIR)/dependence/settings/settings_store.c \
	$(STACK_DIR)/dependence/settings/settings.c \
	$(STACK_DIR)/dependence/settings/settings_init.c  \
	$(STACK_DIR)/dependence/settings/settings_line.c

ifeq ($(CONFIG_SETTINGS_RUNTIME), y)
	SRC_C += $(STACK_DIR)/dependence/settings/settings_runtime.c 
endif

ifeq ($(CONFIG_SETTINGS_FS), y)
	SRC_C += $(STACK_DIR)/dependence/settings/settings_file.c
endif

ifeq ($(CONFIG_SETTINGS_FCB), y)
	SRC_C += $(STACK_DIR)/dependence/settings/settings_fcb.c
endif

ifeq ($(CONFIG_SETTINGS_NVS), y)
	SRC_C += $(STACK_DIR)/dependence/settings/settings_nvs.c
endif

ifeq ($(CONFIG_SETTINGS_NONE), y)
	SRC_C += $(STACK_DIR)/dependence/settings/settings_none.c 
endif

# ------------------ subdir dependence/tinycrypt/ -------------------
ifeq ($(CONFIG_TINYCRYPT), y)
	SRC_C += \
		$(STACK_DIR)/dependence/tinycrypt/ecc.c \
		$(STACK_DIR)/dependence/tinycrypt/utils.c 

# ifneq ($(CONFIG_BT_TINYCRYPT_ECC), y)
# 	SRC_C += $(STACK_DIR)/dependence/tinycrypt/ecc_platform_specific.c
# endif

	ifeq ($(CONFIG_TINYCRYPT_ECC_DH), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/ecc_dh.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_ECC_DSA), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/ecc_dsa.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_AES), y)
		SRC_C += \
		$(STACK_DIR)/dependence/tinycrypt/aes_decrypt.c \
		$(STACK_DIR)/dependence/tinycrypt/aes_encrypt.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_AES_CBC), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/cbc_mode.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_AES_CTR), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/ctr_mode.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_AES_CCM), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/ccm_mode.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_AES_CMAC), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/cmac_mode.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_SHA256), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/sha256.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_SHA256_HMAC), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/hmac.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_SHA256_HMAC_PRNG), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/hmac_prng.c 
	endif

	ifeq ($(CONFIG_TINYCRYPT_CTR_PRNG), y)
		SRC_C += $(STACK_DIR)/dependence/tinycrypt/ctr_prng.c
	endif
endif # CONFIG_TINYCRYPT

# -------------------------- subdir host/ -------------------------
ifeq ($(CONFIG_BT_HCI), y)

	ifeq ($(CONFIG_BT_HCI_RAW), y)
		SRC_C += \
		$(STACK_DIR)/host/hci_raw.c \
		$(STACK_DIR)/host/hci_common.c 
	endif

	ifeq ($(CONFIG_BT_MONITOR), y)
		SRC_C += $(STACK_DIR)/host/monitor.c
	endif

	ifeq ($(CONFIG_BT_TINYCRYPT_ECC), y)
		SRC_C += $(STACK_DIR)/host/hci_ecc.c 
	endif

	ifeq ($(CONFIG_BT_A2DP), y)
		SRC_C += $(STACK_DIR)/host/a2dp.c
	endif

	ifeq ($(CONFIG_BT_AVDTP), y)
		SRC_C += $(STACK_DIR)/host/avdtp.c
	endif

	ifeq ($(CONFIG_BT_RFCOMM), y)
		SRC_C += $(STACK_DIR)/host/rfcomm.c 
	endif

	ifeq ($(CONFIG_BT_TESTING), y)
		SRC_C += $(STACK_DIR)/host/testing.c 
	endif

	ifeq ($(CONFIG_BT_SETTINGS), y)
		SRC_C += $(STACK_DIR)/host/settings.c 
	endif

	ifeq ($(CONFIG_BT_HOST_CCM), y)
		SRC_C += $(STACK_DIR)/host/aes_ccm.c 
	endif

	ifeq ($(CONFIG_BT_BREDR), y)
		SRC_C += \
		$(STACK_DIR)/host/br.c   \
		$(STACK_DIR)/host/keys_br.c \
		$(STACK_DIR)/host/l2cap_br.c \
		$(STACK_DIR)/host/sdp.c  \
		$(STACK_DIR)/host/ssp.c \	
	endif

	ifeq ($(CONFIG_BT_HFP_HF), y)
		SRC_C += \
		$(STACK_DIR)/host/hfp_hf.c \
		$(STACK_DIR)/host/at.c 
	endif

	ifeq ($(CONFIG_BT_HCI_HOST), y)
		SRC_C += \
		$(STACK_DIR)/host/uuid.c \
		$(STACK_DIR)/host/addr.c \
		$(STACK_DIR)/host/buf.c \
		$(STACK_DIR)/host/hci_core.c \
		$(STACK_DIR)/host/hci_common.c \
		$(STACK_DIR)/host/id.c 

		ifeq ($(CONFIG_BT_BROADCASTER), y)
			SRC_C += $(STACK_DIR)/host/adv.c 
		endif

		ifeq ($(CONFIG_BT_OBSERVER), y)
			SRC_C += $(STACK_DIR)/host/scan.c 
		endif
		
		ifeq ($(CONFIG_BT_HOST_CRYPTO), y)
			SRC_C += $(STACK_DIR)/host/crypto.c 
		endif

		ifeq ($(CONFIG_BT_ECC), y)
			SRC_C += $(STACK_DIR)/host/ecc.c 
		endif
	endif # CONFIG_BT_HCI_HOST

	ifeq ($(CONFIG_BT_CONN), y)
		SRC_C += \
		$(STACK_DIR)/host/conn.c \
		$(STACK_DIR)/host/l2cap.c \
		$(STACK_DIR)/host/att.c \
		$(STACK_DIR)/host/gatt.c 

		ifeq ($(CONFIG_BT_SMP), y)
			SRC_C += \
			$(STACK_DIR)/host/smp.c \
			$(STACK_DIR)/host/keys.c 
		else
			SRC_C += $(STACK_DIR)/host/smp_null.c 
		endif # CONFIG_BT_SMP
	endif # CONFIG_BT_CONN

	ifeq ($(CONFIG_BT_ISO), y)
		SRC_C += $(STACK_DIR)/host/iso.c
	endif

	ifeq ($(CONFIG_BT_DF), y)
		SRC_C += $(STACK_DIR)/host/direction.c 
	endif
endif # CONFIG_BT_HCI

# ------------------------ subdir services/ -------------------------
ifeq ($(CONFIG_BT_DIS), y)
	SRC_C += $(STACK_DIR)/services/dis.c 
endif

ifeq ($(CONFIG_BT_BAS), y)
	SRC_C += $(STACK_DIR)/services/bas.c 
endif

ifeq ($(CONFIG_BT_HRS), y)
	SRC_C += $(STACK_DIR)/services/hrs.c 
endif

ifeq ($(CONFIG_BT_TPS), y)
	SRC_C += $(STACK_DIR)/services/tps.c 
endif
# ------------------------ subdir services/ots -------------------------
ifeq ($(CONFIG_BT_OTS), y)
	SRC_C += \
	$(STACK_DIR)/services/ots/ots.c \
	$(STACK_DIR)/services/ots/ots_l2cap.c \
	$(STACK_DIR)/services/ots/ots_obj_manager.c \
	$(STACK_DIR)/services/ots/ots_oacp.c \
	$(STACK_DIR)/services/ots/ots_olcp.c 
	ifeq ($(CONFIG_BT_OTS_DIR_LIST_OBJ), y)
		SRC_C += ots_dir_list.c
	endif
endif
# ------------------------ subdir shell/ -------------------------
ifeq ($(CONFIG_BT_SHELL), y)
	SRC_C += \
	$(STACK_DIR)/shell/bt.c \
	$(STACK_DIR)/shell/hci.c 

	ifeq ($(CONFIG_BT_CONN), y)
		SRC_C += $(STACK_DIR)/shell/gatt.c 
	endif

	ifeq ($(CONFIG_BT_CONN), y)
		SRC_C += $(STACK_DIR)/shell/bredr.c 
	endif

	ifeq ($(CONFIG_BT_L2CAP_DYNAMIC_CHANNEL), y)
		SRC_C += $(STACK_DIR)/shell/l2cap.c  
	endif

	ifeq ($(CONFIG_BT_RFCOMM), y)
		SRC_C += $(STACK_DIR)/shell/rfcomm.c   
	endif

	ifeq ($(CONFIG_BT_ISO), y)
		SRC_C += $(STACK_DIR)/shell/iso.c  
	endif

	ifeq ($(CONFIG_BT_VCS), y)
		SRC_C += $(STACK_DIR)/shell/vcs.c  
	endif

	ifeq ($(CONFIG_BT_VCS_CLIENT), y)
		SRC_C += $(STACK_DIR)/shell/vcs_client.c  
	endif

	ifeq ($(CONFIG_BT_MICS), y)
		SRC_C += $(STACK_DIR)/shell/mics.c  
	endif

	ifeq ($(CONFIG_BT_MICS)_$(CONFIG_BT_LL_SW_SPLIT), y_y)
		SRC_C +=  \
		$(STACK_DIR)/shell/ticker.c \
		$(STACK_DIR)/shell/ll.c 
	endif
endif # CONFIG_BT_SHELL

# ------------------ subdir mesh/ ---------------------------------
ifeq ($(CONFIG_BT_MESH), y)
	SRC_C += \
	$(STACK_DIR)/mesh/access.c \
	$(STACK_DIR)/mesh/adv.c \
	$(STACK_DIR)/mesh/app_keys.c \
	$(STACK_DIR)/mesh/beacon.c \
	$(STACK_DIR)/mesh/cfg.c \
	$(STACK_DIR)/mesh/cfg_srv.c \
	$(STACK_DIR)/mesh/crypto.c \
	$(STACK_DIR)/mesh/health_srv.c \
	$(STACK_DIR)/mesh/heartbeat.c \
	$(STACK_DIR)/mesh/main.c \
	$(STACK_DIR)/mesh/msg.c \
	$(STACK_DIR)/mesh/net.c \
	$(STACK_DIR)/mesh/rpl.c \
	$(STACK_DIR)/mesh/subnet.c \
	$(STACK_DIR)/mesh/transport.c 

	ifeq ($(CONFIG_BT_MESH_ADV_LEGACY), y)
		SRC_C += $(STACK_DIR)/mesh/adv_legacy.c 
	endif

	ifeq ($(CONFIG_BT_MESH_ADV_EXT), y)
		SRC_C += $(STACK_DIR)/mesh/adv_ext.c
	endif

	ifeq ($(CONFIG_BT_SETTINGS), y)
		SRC_C += $(STACK_DIR)/mesh/settings.c
	endif

	ifeq ($(CONFIG_BT_MESH_LOW_POWER), y)
		SRC_C += $(STACK_DIR)/mesh/lpn.c 
	endif

	ifeq ($(CONFIG_BT_MESH_FRIEND), y)
		SRC_C += $(STACK_DIR)/mesh/friend.c
	endif

	ifeq ($(CONFIG_BT_MESH_PROV), y)
		SRC_C += $(STACK_DIR)/mesh/prov.c
	endif

	ifeq ($(CONFIG_BT_MESH_PROV_DEVICE), y)
		SRC_C += $(STACK_DIR)/mesh/prov_device.c
	endif

	ifeq ($(CONFIG_BT_MESH_PROVISIONER), y)
		SRC_C += $(STACK_DIR)/mesh/provisioner.c
	endif

	ifeq ($(CONFIG_BT_MESH_PB_ADV), y)
		SRC_C += $(STACK_DIR)/mesh/pb_adv.c
	endif

	ifeq ($(CONFIG_BT_MESH_PB_GATT), y)
		SRC_C += $(STACK_DIR)/mesh/pb_gatt.c
	endif

	ifeq ($(CONFIG_BT_MESH_GATT_SERVER), y)
		SRC_C += $(STACK_DIR)/mesh/gatt_services.c
	endif

	ifeq ($(CONFIG_BT_MESH_GATT), y)
		SRC_C += $(STACK_DIR)/mesh/proxy_msg.c
	endif

	ifeq ($(CONFIG_BT_MESH_CFG_CLI), y)
		SRC_C += $(STACK_DIR)/mesh/cfg_cli.c
	endif

	ifeq ($(CONFIG_BT_MESH_HEALTH_CLI), y)
		SRC_C += $(STACK_DIR)/mesh/health_cli.c
	endif

	ifeq ($(CONFIG_BT_MESH_SELF_TEST), y)
		SRC_C += $(STACK_DIR)/mesh/test.c
	endif

	ifeq ($(CONFIG_BT_MESH_SHELL), y)
		SRC_C += $(STACK_DIR)/mesh/shell.c
	endif

	ifeq ($(CONFIG_BT_MESH_CDB), y)
		SRC_C += $(STACK_DIR)/mesh/cdb.c
	endif

endif

# ------------------ subdir mesh/ ---------------------------------
ifneq ($(findstring $(CONFIG_BT_VOCS)$(CONFIG_BT_VOCS_CLIENT),  y),)
	SRC_C += $(STACK_DIR)/audio/vocs.c 
endif

ifeq ($(CONFIG_BT_VOCS_CLIENT), y)
	SRC_C += $(STACK_DIR)/audio/vocs_client.c
endif

ifneq ($(findstring $(CONFIG_BT_AICS)$(CONFIG_BT_AICS_CLIENT),  y),)
	SRC_C += $(STACK_DIR)/audio/aics.c
endif

ifeq ($(CONFIG_BT_AICS_CLIENT), y)
	SRC_C += $(STACK_DIR)/audio/aics_client.c
endif

ifneq ($(findstring $(CONFIG_BT_VCS)$(CONFIG_BT_VCS_CLIENT),  y),)
	SRC_C += $(STACK_DIR)/audio/vcs.c 
endif

ifeq ($(CONFIG_BT_VCS_CLIENT), y)
	SRC_C += $(STACK_DIR)/audio/vcs_client.c 
endif

ifneq ($(findstring $(CONFIG_BT_MICS)$(CONFIG_BT_MICS_CLIENT),  y),)
	SRC_C += $(STACK_DIR)/audio/mics.c 
endif

ifeq ($(CONFIG_BT_MICS_CLIENT), y)
	SRC_C += $(STACK_DIR)/audio/mics_client.c 
endif

#$(STACK_DIR)/dependence/tester/ \

#example
SRC_C += \
	$(EXAMP_DIR)/peripheral/cts.c \
	$(EXAMP_DIR)/peripheral/peripheral.c \
	$(EXAMP_DIR)/central/central.c \

#lib_version
VER_C += $(TARGET)_version.c


# Generate obj list
# -------------------------------------------------------------------
#SRC_O = $(patsubst %.c,%_$(TARGET).o,$(SRC_C))
VER_O = $(addprefix $(OBJ_DIR)/,$(patsubst %.c,%_$(TARGET).o,$(VER_C)))

SRC_C_LIST      = $(addprefix $(OBJ_DIR)/,$(patsubst $(REL_PATH)/%.c,%.c,$(SRC_C)))
OBJ_LIST        = $(patsubst %.c,%_$(TARGET).o,$(SRC_C_LIST))
DEPENDENCY_LIST = $(patsubst %.c,%_$(TARGET).d,$(SRC_C_LIST))

# Compile options
# -------------------------------------------------------------------
CFLAGS  = 
CFLAGS += \
	-march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=soft -D__thumb2__ -g -gdwarf-3 -Os \
	-D__ARM_ARCH_8M_MAIN__=1 -gdwarf-3 -fstack-usage -fdata-sections -ffunction-sections \
	-fdiagnostics-color=always -Wall -Wpointer-arith -Wstrict-prototypes -Wundef -Wno-write-strings \
	-Wno-maybe-uninitialized --save-temps -c -MMD \
	-DCONFIG_PLATFORM_8710C -DCONFIG_BUILD_RAM=1 \
	-DCONFIG_BUILD_LIB=1 \
	-D_NO_DEFINITIONS_IN_HEADER_FILES \
	-D__LINUX_ERRNO_EXTENSIONS__\
	-imacros $(INC_DIR)/bt_autoconf.h

include toolchain.mk

.PHONY: menuconfig
menuconfig:
	@export ZEPHYR_BASE=$(shell pwd)/$(STACK_DIR)/dependence;  \
	export srctree=$(STACK_DIR);  \
	export KCONFIG_CONFIG=$(PLATFORM_DIR)/amebaz2/.config; \
	export KCONFIG_AUTOHEADER=$(INC_DIR)/bt_autoconf.h; \
	python3 $(KCONFIG_DIR)/menuconfig.py Kconfig

# Compile
# -------------------------------------------------------------------
.PHONY: lib_bt
lib_bt: prerequirement $(OBJ_LIST) $(VER_O)
	$(AR) rvs $(BIN_DIR)/$(TARGET).a $(OBJ_LIST) $(VER_O)
	$(OBJCOPY) --strip-debug $(BIN_DIR)/$(TARGET).a
	cp $(BIN_DIR)/$(TARGET).a $(COMP_DIR)/soc/8710c/misc/bsp/lib/common/GCC/$(TARGET).a

.PHONY: prerequirement
prerequirement:
	@echo const char $(TARGET)_rev[] = \"$(TARGET)_ver_`git rev-parse HEAD`_`date +%Y/%m/%d-%T`\"\; > $(TARGET)_version.c
	@if [ ! -d $(ARM_GCC_TOOLCHAIN) ]; then \
		echo ===========================================================; \
		echo Toolchain not found, \"make toolchain\" first!; \
		echo ===========================================================; \
		exit -1; \
	fi
	@echo ===========================================================
	@echo Build $(TARGET)
	@echo ===========================================================
	mkdir -p $(OBJ_DIR)
	mkdir -p $(BIN_DIR)
	mkdir -p $(INFO_DIR)

$(OBJ_LIST): $(OBJ_DIR)/%_$(TARGET).o:$(REL_PATH)/%.c | prerequirement
	@mkdir -p $(patsubst %/,%,$(dir $(lastword $@)))
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	@mv $(notdir $*.i) $(INFO_DIR)
	@mv $(notdir $*.s) $(INFO_DIR)

$(DEPENDENCY_LIST): $(OBJ_DIR)/%_$(TARGET).d:$(REL_PATH)/%.c | prerequirement
	mkdir -p $(patsubst %/,%,$(dir $(lastword $@)))
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -MM -MT $@ -MF $@

$(VER_O):$(VER_C)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	@mv $(notdir $(patsubst %.c,%.i,$<)) $(INFO_DIR)
	@mv $(notdir $(patsubst %.c,%.s,$<)) $(INFO_DIR)

-include $(DEPENDENCY_LIST)

.PHONY: clean
clean:
	rm -rf $(TARGET)
	rm -f $(VER_C)
