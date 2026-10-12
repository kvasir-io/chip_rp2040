include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

set(TARGET_MPU RP2040_M0_0)
set(TARGET_UF2_CODE 0xE48BFF56)
set(TARGET_FLASH_SIZE 2096896)
set(TARGET_RAM_SIZE 262144)
set(TARGET_EEPROM_SIZE 0)
# The SRAM4/SRAM5 scratch banks (linker/chip.ld), 4 KiB each (RP2040 datasheet 2.2.2: SRAM4_BASE 0x20040000, SRAM5_BASE
# 0x20041000, SRAM_END 0x20042000). Kvasir_SDK util.cmake: CORE1_STACK_PLACEMENT scratch puts core 1's stack into SRAM4
# (at most this many bytes), the SCRATCH_BANKS flag turns on the KVASIR_COREn_{DATA,BSS,CODE} attributes and the
# copy/zero at boot (src/chip/StartUp.hpp ExtraMemoryInit). Both opt-in.
set(TARGET_CORE1_SCRATCH_SIZE 4096)

# J-Link Commander lines after `connect` in the flash/reset/connect scripts (Kvasir_SDK cmake/jlink.cmake): core 1 back
# into its boot ROM before anything is reset or written. The commander's `r` restarts the connected core only, and a
# core 1 the old image launched runs on through the erase into the new image's RAM. PSM FRCE_OFF.PROC1 set, then
# cleared, through the atomic aliases: PSM_BASE 0x40010000 + FRCE_OFF 0x4, PROC1 = bit 16 (RP2040 datasheet 2.13.5 "List
# of Registers", Table 198), +0x2000 set / +0x3000 clear (2.1.2 "Atomic Register Access"). uc_log's printer gets the
# same lines (util.cmake PRE_RESET_COMMANDS -> --pre_reset_command) and writes them before its own resets and downloads
# - it understands only `w4 <address> <value>`, keep them to that. Then WATCHDOG SCRATCH4 and SCRATCH3 (WATCHDOG_BASE
# 0x40058000 + 0x1c / 0x18, 4.7.6) to 0: disarms Kvasir::CoreResetGuard's watchdog boot vector (2.8.1.1), whose RAM
# routine a download overwrites, and clears its reset-cause word.
set(TARGET_JLINK_CONNECT_COMMANDS "w4 0x40012004 0x00010000" "w4 0x40013004 0x00010000" "w4 0x4005801c 0x00000000"
                                  "w4 0x40058018 0x00000000")

# Chip parked in the boot ROM by the probe's Rescue DP recovery (RP2040 datasheet 2.3.4.2, 2.8.1): CHIP_RESET
# (0x40064008) HAD_PSM_RESTART, bit 20 (Table 191), set until the next chip-level reset (2.12.7), and the core asleep in
# the ROM's `wfi` loop at 0x30. util.cmake PARK_SIGNATURE -> --park_signature: the printer resets such a chip.
set(TARGET_JLINK_PARK_SIGNATURE "0x40064008,0x00100000,0x00100000,0x30")
# The firmware check also reads flash through XIP_NOCACHE_NOALLOC (0x13000000, datasheet 2.2): a reset of core 0 alone
# can leave stale cache lines (2.4.2.9, 2.6.3.2). util.cmake -> --flash_uncached_offset.
set(TARGET_JLINK_FLASH_UNCACHED_OFFSET "0x03000000")
# Boot ROM stuck reading boot2 in plain SPI (2.8.1.2) because core 1 fetched through the SSI after a reset of core 0
# alone (2.4.2.9). Sign: SSI CTRLR0 SPI_FRF 0, DFS_32 7, TMOD 0 (SCPH/SCPOL vary, left out), SR.BUSY 0, RXFLR and TXFLR
# 0 (4.10.13, Table 593). Recovery: watchdog reset of everything but the oscillators (PSM WDSEL bits 16:2, 2.13;
# WATCHDOG CTRL.TRIGGER, 4.7), which also resets core 1 and XIP. util.cmake -> --stall_signature / --stall_recovery.
set(TARGET_JLINK_STALL_SIGNATURE
    "0x18000000,0x007f0300,0x00070000+0x18000028,0x1,0x0+0x18000024,0xffffffff,0x0+0x18000020,0xffffffff,0x0")
set(TARGET_JLINK_STALL_RECOVERY "w4 0x40010008 0x0001fffc" "w4 0x40058000 0x80000000")
# Flash readable over XIP only in boot2's mode: CTRLR0 TMOD (bits 9:8) EEPROM_READ (0x3) and SSIENR (0x18000008) bit 0
# set (RP2040 datasheet 4.10.13, Tables 582/583); an XIP read during the ROM's plain SPI transfer stalls it (2.8.1.2).
# The printer reads flash only while these hold. util.cmake -> --flash_ready_signature.
set(TARGET_JLINK_FLASH_READY_SIGNATURE "0x18000000,0x300,0x300+0x18000008,0x1,0x1")
# The XIP window and its aliases 0x10000000..0x13ffffff (datasheet 2.2): control-socket reads there pass the same gate;
# the SSI registers at 0x18000000 stay outside. util.cmake -> --flash_window.
set(TARGET_JLINK_FLASH_WINDOW "0x10000000,0x04000000")

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld)
# For kvasir_executable(... RAM_ONLY): everything in SRAM, no flash region, no boot2.
set(LINKER_FILE_RAM_ONLY ${CMAKE_CURRENT_LIST_DIR}/../linker/chip_ram_only.ld)

set(CHIP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S PROPERTIES COMPILE_FLAGS "-Wno-c++-keyword")
set(CHIP_LINKER_OPTIONS --wrap=__aeabi_idiv --wrap=__aeabi_idivmod --wrap=__aeabi_ldivmod --wrap=__aeabi_uidiv
                        --wrap=__aeabi_uidivmod --wrap=__aeabi_uldivmod)

# the write-only guard: every write-only field of the SVD is classified (oneToSet, a key, or <!-- Kvasir: write-only
# accepted -->), so a new one stops the build; registers with no readable field are never read
svd_convert(
    peripherals
    SVD_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../chip.svd
    OUTPUT_DIRECTORY
    peripherals
    WRITE_ONLY_GUARD
    error
    WRITE_ONLY_REGISTERS
    derived
    WRITE_ONLY_MASK
    ON)

# kvasir_devices: chip.hpp includes its drivers unconditionally (rp_common/I2CQueued.hpp -> I2CBusRecovery.hpp ->
# kvasir/Devices/I2C/LineRecovery.hpp, and the USB backend), so every image needs it. Found like CHIP_ROOT
# (KVASIR_DEVICES_ROOT: variable, environment, else next to the SDK); the SDK adds it after project() unless the
# firmware has it already.
kvasir_resolve_root(KVASIR_DEVICES_ROOT kvasir_devices)
kvasir_add_package(${KVASIR_DEVICES_ROOT} kvasir_devices kvasir_devices)
target_link_libraries(peripherals INTERFACE kvasir::devices)
