cmake_minimum_required(VERSION 3.6)

project(flash_bin)

# root of realtek_amebapro2_v0_example
set (prj_root "${CMAKE_CURRENT_SOURCE_DIR}/..")
# root of SDK
set (sdk_root "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# check system is LINUX 
if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
    message(STATUS "Build LINUX")
endif()

# set elf2bin
if (LINUX)
    set(ELF2BIN ${prj_root}/GCC-RELEASE/elf2bin.linux)
else()
    set(ELF2BIN ${prj_root}/GCC-RELEASE/elf2bin.exe)
endif()

if(NOT DEFINED BUILD_TZ)
	set(BUILD_TZ OFF)
endif()

message(STATUS "Build TZ ${BUILD_TZ}")

#add_subdirectory (bootloader)
if(BUILD_TZ)
	set(target flash_tz)
	set(fw application_tz/firmware_tz.bin)
	set(app_dep application.s application.ns)
	add_subdirectory (application_tz)
else()
	set(target flash_ntz)
	set(fw application_ntz/firmware_ntz.bin)
	set(app_dep application.ntz)
	add_subdirectory (application_ntz)
endif()

add_custom_target(
	flash
	
	COMMAND cp ${prj_root}/GCC-RELEASE/keycfg.json keycfg.json
	COMMAND cp ${prj_root}/GCC-RELEASE/partition.json partition.json
	COMMAND cp ${prj_root}/GCC-RELEASE/amebapro2_partition.json amebapro2_partition.json
	COMMAND ${ELF2BIN} keygen keycfg.json
	
	COMMAND ${ELF2BIN} convert amebapro2_partition.json PARTITIONTABLE 
	COMMAND cp ${prj_root}/GCC-RELEASE/bootloader/boot.bin .
	
	COMMAND cp ${fw} ./firmware.bin
	COMMAND ${ELF2BIN} combine ${target}.bin PTAB=partition.bin,BOOT=boot.bin,FW1=firmware.bin
	
	#DEPENDS bootloader
	DEPENDS ${app_dep}
)
