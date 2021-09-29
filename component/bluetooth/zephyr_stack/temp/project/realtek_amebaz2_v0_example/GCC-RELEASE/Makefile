all: lib_all ze

OS := $(shell uname)

include toolchain.mk

.PHONY: lib_all
lib_all: toolchain
	@$(MAKE) -f lib_bt_zephyr.mk all
	@$(MAKE) -f lib_boot.mk all
	@$(MAKE) -f bootloader.mk all
	@$(MAKE) -f lib_wlan.mk all
	@$(MAKE) -f lib_wlan_mp.mk all
	@$(MAKE) -f lib_wlan_ns.mk all
	@$(MAKE) -f lib_wlan_mp_ns.mk all
	@$(MAKE) -f lib_soc_is.mk all
	@$(MAKE) -f lib_soc_s.mk all
	@$(MAKE) -f lib_soc_ns.mk all
	@$(MAKE) -f lib_http.mk all
	@$(MAKE) -f lib_http_ns.mk all
	@$(MAKE) -f lib_dct.mk all
	@$(MAKE) -f lib_dct_ns.mk all
	@$(MAKE) -f lib_websocket.mk all
	@$(MAKE) -f lib_websocket_ns.mk all
	@$(MAKE) -f lib_wps.mk all
	@$(MAKE) -f lib_wps_ns.mk all
	@$(MAKE) -f lib_eap.mk all
	@$(MAKE) -f lib_eap_ns.mk all
	@$(MAKE) -f lib_p2p.mk all
	@$(MAKE) -f lib_p2p_ns.mk all
	@$(MAKE) -f lib_coap.mk all
	@$(MAKE) -f lib_coap_ns.mk all
	
.PHONY: bootloader
bootloader: toolchain
	@$(MAKE) -f bootloader.mk

.PHONY: is
is: toolchain
	@$(MAKE) -f application.is.mk

.PHONY: tz
tz: toolchain
	@$(MAKE) -f application.tz.mk

.PHONY: ze
ze: toolchain
	@$(MAKE) -f application.ze.mk

.PHONY: ze_clean
ze_clean: toolchain
	@$(MAKE) -f application.ze.mk clean

.PHONY: s
s: toolchain | lib_all
	@$(MAKE) -f application.s.mk

.PHONY: ns
ns: toolchain
	@$(MAKE) -f application.ns.mk

.PHONY: mp
mp: toolchain | lib_all
	@$(MAKE) -f application.is.mk mp

.PHONY: lib_bt
lib_bt: toolchain
	@$(MAKE) -f lib_bt_zephyr.mk

.PHONY: lib_bt_menuconfig
lib_bt_menuconfig:
	@$(MAKE) -f lib_bt_zephyr.mk menuconfig

.PHONY: lib_bt_clean
lib_bt_clean:
	@$(MAKE) -f lib_bt_zephyr.mk clean

.PHONY: lib_rom
lib_rom: toolchain
	@$(MAKE) -f lib_rom.mk

.PHONY: lib_boot
lib_boot: toolchain
	@$(MAKE) -f lib_boot.mk	

.PHONY: lib_wlan
lib_wlan: toolchain
	@$(MAKE) -f lib_wlan.mk

.PHONY: lib_wlan_mp
lib_wlan_mp: toolchain
	@$(MAKE) -f lib_wlan_mp.mk

.PHONY: lib_wlan_ns
lib_wlan_ns: toolchain
	@$(MAKE) -f lib_wlan_ns.mk

.PHONY: lib_wlan_mp_ns
lib_wlan_mp_ns: toolchain
	@$(MAKE) -f lib_wlan_mp_ns.mk

.PHONY: lib_dct
lib_dct: toolchain
	@$(MAKE) -f lib_dct.mk

.PHONY: lib_dct_ns
lib_dct_ns: toolchain
	@$(MAKE) -f lib_dct_ns.mk

.PHONY: lib_websocket
lib_websocket: toolchain
	@$(MAKE) -f lib_websocket.mk

.PHONY: lib_websocket_ns
lib_websocket_ns: toolchain
	@$(MAKE) -f lib_websocket_ns.mk

.PHONY: lib_wps
lib_wps: toolchain
	@$(MAKE) -f lib_wps.mk

.PHONY: lib_wps_ns
lib_wps_ns: toolchain
	@$(MAKE) -f lib_wps_ns.mk

.PHONY: lib_http
lib_http: toolchain
	@$(MAKE) -f lib_http.mk

.PHONY: lib_http_ns
lib_http_ns: toolchain
	@$(MAKE) -f lib_http_ns.mk

.PHONY: lib_eap
lib_eap: toolchain
	@$(MAKE) -f lib_eap.mk

.PHONY: lib_eap_ns
lib_eap_ns: toolchain
	@$(MAKE) -f lib_eap_ns.mk

.PHONY: lib_p2p
lib_p2p: toolchain
	@$(MAKE) -f lib_p2p.mk

.PHONY: lib_p2p_ns
lib_p2p_ns: toolchain
	@$(MAKE) -f lib_p2p_ns.mk

.PHONY: lib_soc_is
lib_soc_is: toolchain
	@$(MAKE) -f lib_soc_is.mk

.PHONY: lib_soc_s
lib_soc_s: toolchain
	@$(MAKE) -f lib_soc_s.mk

.PHONY: lib_soc_ns
lib_soc_ns: toolchain
	@$(MAKE) -f lib_soc_ns.mk

.PHONY: lib_coap
lib_coap: toolchain
	@$(MAKE) -f lib_coap.mk

.PHONY: lib_coap_ns
lib_coap_ns: toolchain
	@$(MAKE) -f lib_coap_ns.mk

.PHONY: clean
clean:
	@$(MAKE) -f application.is.mk clean
	@$(MAKE) -f application.tz.mk clean
	@$(MAKE) -f application.ze.mk clean
	@$(MAKE) -f bootloader.mk clean
	@$(MAKE) -f lib_bt_zephyr.mk clean
	@$(MAKE) -f lib_boot.mk clean	
	@$(MAKE) -f lib_eap.mk clean
	@$(MAKE) -f lib_eap_ns.mk clean
	@$(MAKE) -f lib_soc_is.mk clean
	@$(MAKE) -f lib_soc_s.mk clean
	@$(MAKE) -f lib_soc_ns.mk clean
	@$(MAKE) -f lib_wlan.mk clean
	@$(MAKE) -f lib_wlan_mp.mk clean
	@$(MAKE) -f lib_wlan_ns.mk clean
	@$(MAKE) -f lib_wlan_mp_ns.mk clean
	@$(MAKE) -f lib_http.mk clean
	@$(MAKE) -f lib_http_ns.mk clean
	@$(MAKE) -f lib_dct.mk clean
	@$(MAKE) -f lib_dct_ns.mk clean
	@$(MAKE) -f lib_websocket.mk clean
	@$(MAKE) -f lib_websocket_ns.mk clean
	@$(MAKE) -f lib_wps.mk clean
	@$(MAKE) -f lib_wps_ns.mk clean
	@$(MAKE) -f lib_p2p.mk clean
	@$(MAKE) -f lib_p2p_ns.mk clean
	@$(MAKE) -f lib_coap.mk clean
	@$(MAKE) -f lib_coap_ns.mk clean
	
romdebug: toolchain
	@$(MAKE) -f application.is.mk romdebug

debug: toolchain
	@$(MAKE) -f application.is.mk debug

debug_tz: toolchain
	@$(MAKE) -f application.tz.mk debug

debug_ze: toolchain
	@$(MAKE) -f application.ze.mk debug

flash: toolchain
	@$(MAKE) -f application.is.mk flash

flash_tz: toolchain
	@$(MAKE) -f application.tz.mk flash

flash_ze: toolchain
	@$(MAKE) -f application.ze.mk flash
	
setup:
	@$(MAKE) -f application.is.mk $(MAKECMDGOALS)	

setup_tz:
	@$(MAKE) -f application.tz.mk $(MAKECMDGOALS)

setup_ze:
	@$(MAKE) -f application.ze.mk $(MAKECMDGOALS)

dump:
	@$(MAKE) -f application.is.mk dump

dump_tz:
	@$(MAKE) -f application.tz.mk dump

dump_ze:
	@$(MAKE) -f application.ze.mk dump
