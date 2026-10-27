include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

# The chip's part of the contract (Kvasir_SDK cmake/kvasir_chip.cmake): the kvasir_chip() call at the end, its values
# collected here with their reasons.

# Which chip this is, for every TU of an image (kvasir_chip DEFINITIONS reach them through kvasir::chip):
# src/chip/rp2040.hpp and the shared rp_common headers test it with `#if defined(KVASIR_CHIP_RP2040)` (gcc -Wundef:
# never a bare `#if`), and rp_common/PinConfig.hpp defines Chip::part from it.
set(_chip_definitions KVASIR_CHIP_RP2040=1 KVASIR_CHIP_FAMILY_RP=1)
# The chip's Register::ExecuteSeam (Kvasir_SDK Register/Exec.hpp includes the header named here at its end, so every TU
# that reaches apply() has it): atomic() through the SET/CLR/XOR alias windows, chip_rp_common RegisterAlias.hpp.
list(APPEND _chip_definitions [[KVASIR_CHIP_REGISTER_SEAM="chip/rp_common/RegisterAlias.hpp"]])
# The SRAM4/SRAM5 scratch banks (linker/chip.ld), 4 KiB each (RP2040 datasheet 2.2.2: SRAM4_BASE 0x20040000, SRAM5_BASE
# 0x20041000, SRAM_END 0x20042000). Kvasir_SDK util.cmake: CORE1_STACK_PLACEMENT scratch puts core 1's stack into SRAM4
# (at most this many bytes), the SCRATCH_BANKS flag turns on the KVASIR_COREn_{DATA,BSS,CODE} attributes and the
# copy/zero at boot (src/chip/StartUp.hpp ExtraMemoryInit). Both opt-in. kvasir_chip CORE1_SCRATCH_SIZE below.

# J-Link Commander lines after `connect` in the flash/reset/connect scripts (Kvasir_SDK cmake/jlink.cmake): core 1 back
# into its boot ROM before anything is reset or written. The commander's `r` restarts the connected core only, and a
# core 1 the old image launched runs on through the erase into the new image's RAM. PSM FRCE_OFF.PROC1 set, then
# cleared, through the atomic aliases: PSM_BASE 0x40010000 + FRCE_OFF 0x4, PROC1 = bit 16 (RP2040 datasheet 2.13.5 "List
# of Registers", Table 198), +0x2000 set / +0x3000 clear (2.1.2 "Atomic Register Access"). uc_log's printer gets the
# same lines (util.cmake PRE_RESET_COMMANDS -> --pre_reset_command) and writes them before its own resets and downloads
# - it understands only `w4 <address> <value>`, keep them to that. Then WATCHDOG SCRATCH4 and SCRATCH3 (WATCHDOG_BASE
# 0x40058000 + 0x1c / 0x18, 4.7.6) to 0: disarms Kvasir::CoreResetGuard's watchdog boot vector (2.8.1.1), whose RAM
# routine a download overwrites, and clears its reset-cause word.
set(_chip_jlink_connect "w4 0x40012004 0x00010000" "w4 0x40013004 0x00010000" "w4 0x4005801c 0x00000000"
                        "w4 0x40058018 0x00000000")

# Chip parked in the boot ROM by the probe's Rescue DP recovery (RP2040 datasheet 2.3.4.2, 2.8.1): CHIP_RESET
# (0x40064008) HAD_PSM_RESTART, bit 20 (Table 191), set until the next chip-level reset (2.12.7), and the core asleep in
# the ROM's `wfi` loop at 0x30. util.cmake PARK_SIGNATURE -> --park_signature: the printer resets such a chip.
set(_chip_jlink_park "0x40064008,0x00100000,0x00100000,0x30")
# The firmware check also reads flash through XIP_NOCACHE_NOALLOC (0x13000000, datasheet 2.2): a reset of core 0 alone
# can leave stale cache lines (2.4.2.9, 2.6.3.2). util.cmake -> --flash_uncached_offset.
set(_chip_jlink_uncached "0x03000000")
# Boot ROM stuck reading boot2 in plain SPI (2.8.1.2) because core 1 fetched through the SSI after a reset of core 0
# alone (2.4.2.9). Sign: SSI CTRLR0 SPI_FRF 0, DFS_32 7, TMOD 0 (SCPH/SCPOL vary, left out), SR.BUSY 0, RXFLR and TXFLR
# 0 (4.10.13, Table 593). Recovery: watchdog reset of everything but the oscillators (PSM WDSEL bits 16:2, 2.13;
# WATCHDOG CTRL.TRIGGER, 4.7), which also resets core 1 and XIP - the reboot list, src/chip/WatchdogDomains.hpp
# onReboot. util.cmake -> --stall_signature / --stall_recovery.
set(_chip_jlink_stall
    "0x18000000,0x007f0300,0x00070000+0x18000028,0x1,0x0+0x18000024,0xffffffff,0x0+0x18000020,0xffffffff,0x0")
set(_chip_jlink_stall_recovery "w4 0x40010008 0x0001fffc" "w4 0x40058000 0x80000000")
# Flash readable over XIP only in boot2's mode: CTRLR0 TMOD (bits 9:8) EEPROM_READ (0x3) and SSIENR (0x18000008) bit 0
# set (RP2040 datasheet 4.10.13, Tables 582/583); an XIP read during the ROM's plain SPI transfer stalls it (2.8.1.2).
# The printer reads flash only while these hold. util.cmake -> --flash_ready_signature.
set(_chip_jlink_flash_ready "0x18000000,0x300,0x300+0x18000008,0x1,0x1")
# The XIP window and its aliases 0x10000000..0x13ffffff (datasheet 2.2): control-socket reads there pass the same gate;
# the SSI registers at 0x18000000 stay outside. util.cmake -> --flash_window.
set(_chip_jlink_flash_window "0x10000000,0x04000000")

# the hardware divider (src/chip/divider.S) in place of the runtime's division: each linked as --wrap=<name>, the
# wrapped names are the .S's own `wrapper_func <name>` lines - the list and the code cannot drift
set(_divider_source ${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S)
file(STRINGS ${_divider_source} _divider_wrapped REGEX "^wrapper_func [A-Za-z_0-9]+$")
list(TRANSFORM _divider_wrapped REPLACE "^wrapper_func " "")
if(NOT _divider_wrapped)
    message(FATAL_ERROR "chip_rp2040: no 'wrapper_func <name>' line in ${_divider_source}")
endif()
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS ${_divider_source})
set(_chip_linker_options)
foreach(f IN LISTS _divider_wrapped)
    list(APPEND _chip_linker_options --wrap=${f})
endforeach()

# Kvasir's write semantics (svd_convert's defaults): a new unclassified write-only field stops the build
kvasir_chip_svd(peripherals ${CMAKE_CURRENT_LIST_DIR}/../chip.svd)

# DEVICES: chip/drivers.hpp builds on kvasir_devices, which the SDK then adds after project()
kvasir_chip(
    DEVICES
    MPU
    RP2040_M0_0
    UF2_FAMILY
    0xE48BFF56
    CORE1_SCRATCH_SIZE
    4096
    LINKER_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld
    # for kvasir_executable(... RAM_ONLY): everything in SRAM, no flash region, no boot2
    RAM_ONLY_LINKER_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../linker/chip_ram_only.ld
    INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/../src
    REGISTERS
    peripherals
    DEFINITIONS
    ${_chip_definitions}
    SOURCES
    ${_divider_source}
    LINKER_OPTIONS
    ${_chip_linker_options}
    JLINK_CONNECT_COMMANDS
    ${_chip_jlink_connect}
    JLINK_PARK_SIGNATURE
    ${_chip_jlink_park}
    JLINK_FLASH_UNCACHED_OFFSET
    ${_chip_jlink_uncached}
    JLINK_STALL_SIGNATURE
    ${_chip_jlink_stall}
    JLINK_STALL_RECOVERY
    ${_chip_jlink_stall_recovery}
    JLINK_FLASH_READY_SIGNATURE
    ${_chip_jlink_flash_ready}
    JLINK_FLASH_WINDOW
    ${_chip_jlink_flash_window})

# the files chip.ld and chip_ram_only.ld INCLUDE: an edit relinks every image (the image's LINK_DEPENDS name the SDK's
# scripts and its own LINKER_FILE only)
set_property(
    TARGET kvasir_chip
    APPEND
    PROPERTY INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/../linker/scratch_banks_noload.inc.ld
             ${CMAKE_CURRENT_LIST_DIR}/../linker/scratch_banks_data.inc.ld)
