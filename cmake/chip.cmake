include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../src/chip/rp_common/pioasm ${CMAKE_BINARY_DIR}/chip_pioasm)

set(TARGET_MPU RP2040_M0_0)
set(TARGET_UF2_CODE 0xE48BFF56)
set(TARGET_FLASH_SIZE 2096896)
set(TARGET_RAM_SIZE 262144)
set(TARGET_EEPROM_SIZE 0)
set(TARGET_EXTRA_FLASH_SECTIONS .boot2)

# J-Link Commander lines after `connect` in the flash/reset/connect scripts (Kvasir_SDK cmake/jlink.cmake): core 1 back
# into its boot ROM before anything is reset or written. The commander's `r` restarts the connected core only, and a
# core 1 the old image launched runs on through the erase into the new image's RAM (i2c_testing 2026-09-19). PSM
# FRCE_OFF.PROC1 set, then cleared, through the atomic aliases: PSM_BASE 0x40010000 + FRCE_OFF 0x4, PROC1 = bit 16
# (RP2040 datasheet 2.13.5 "List of Registers", Table 198), +0x2000 set / +0x3000 clear (2.1.2 "Atomic Register
# Access"). uc_log's printer gets the same lines (util.cmake PRE_RESET_COMMANDS -> --pre_reset_command) and writes them
# before its own resets and downloads - it understands only `w4 <address> <value>`, keep them to that.
set(TARGET_JLINK_CONNECT_COMMANDS "w4 0x40012004 0x00010000" "w4 0x40013004 0x00010000")

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld)
# For kvasir_executable(... RAM_ONLY): everything in SRAM, no flash region, no boot2.
set(LINKER_FILE_RAM_ONLY ${CMAKE_CURRENT_LIST_DIR}/../linker/chip_ram_only.ld)

set(CHIP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S PROPERTIES COMPILE_FLAGS "-Wno-c++-keyword")
set(CHIP_LINKER_OPTIONS --wrap=__aeabi_idiv --wrap=__aeabi_idivmod --wrap=__aeabi_ldivmod --wrap=__aeabi_uidiv
                        --wrap=__aeabi_uidivmod --wrap=__aeabi_uldivmod)

svd_convert(peripherals SVD_FILE ${CMAKE_CURRENT_LIST_DIR}/../chip.svd OUTPUT_DIRECTORY peripherals)

# kvasir_devices: chip.hpp includes its drivers unconditionally (rp_common/I2CQueued.hpp -> I2CBusRecovery.hpp ->
# kvasir/Devices/I2C/LineRecovery.hpp, and the USB backend), so every image needs it. Found like CHIP_ROOT
# (KVASIR_DEVICES_ROOT: variable, environment, else next to the SDK); the SDK adds it after project() unless the
# firmware has it already.
kvasir_resolve_root(KVASIR_DEVICES_ROOT kvasir_devices)
kvasir_add_package(${KVASIR_DEVICES_ROOT} kvasir_devices kvasir_devices)
target_link_libraries(peripherals INTERFACE kvasir::devices)
