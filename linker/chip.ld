/* Linker script for RP2040 with 2MB Flash */

/* RAM (RP2040 datasheet 2.2.2 "Address Map, Detail", md lines 1275-1281; 2.6.2 "SRAM", md line 5888):
   SRAM0-3 striped at 0x20000000 (256K, `ram`), then two 4 KiB banks that are "always non-striped":
   SRAM4_BASE 0x20040000 (`scratch_x`, core 1's) and SRAM5_BASE 0x20041000 (`scratch_y`, core 0's), up to
   SRAM_END 0x20042000. "Software may choose to use these for per-core purposes, e.g. stack and
   frequently-executed code" (2.6.2). The boot ROM runs boot2 from SRAM5 (2.8.1, md line 6298), which is over
   before ResetISR. */
MEMORY{
    boot2     ( r ) : ORIGIN = 0x10000000, LENGTH = 256
    flash     (xr ) : ORIGIN = 0x10000100, LENGTH = 2096896
    ram       (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
    scratch_x (xrw) : ORIGIN = 0x20040000, LENGTH = 4K
    scratch_y (xrw) : ORIGIN = 0x20041000, LENGTH = 4K
}

SECTIONS{
    . = ORIGIN(boot2);
    .boot2 : {
        _LINKER_INTERN_boot2_start_ = .;
        KEEP (*(.boot2))
        _LINKER_INTERN_boot2_end_ = .;
    } > boot2
    ASSERT(_LINKER_INTERN_boot2_end_ - _LINKER_INTERN_boot2_start_ == 256,
           "ERROR: Pico second stage bootloader must be 256 bytes in size")
}

INCLUDE common_flash.ld

/* The scratch banks' NOLOAD part, before common_ram.ld (the first-match order: in the file) */
INCLUDE scratch_banks_noload.inc.ld

INCLUDE common_ram.ld

/* The scratch banks' initialised part: its load image follows .data's in flash (scratch_banks_data.inc.ld) */
INCLUDE scratch_banks_data.inc.ld

/* Kvasir::CoreResetGuard's routine (src/chip/CoreResetGuard.hpp; empty without it): the boot ROM's watchdog boot
   vector jumps there before the image runs, so it sits in SRAM5 apart from everything else, below the top 256 bytes
   where the ROM loads boot2 (RP2040 datasheet 2.8.1). Copied from its load image by the guard's runtimeInit. */
SECTIONS {
    .core_reset_guard : AT(LOADADDR(.scratch_y) + SIZEOF(.scratch_y)) {
        . = ALIGN(4);
        _LINKER_INTERN_core_reset_guard_start_ = .;
        *(.core_reset_guard*)
        . = ALIGN(4);
        _LINKER_INTERN_core_reset_guard_end_ = .;
    } > scratch_y
    _LINKER_INTERN_core_reset_guard_load_ = LOADADDR(.core_reset_guard);
    ASSERT(_LINKER_INTERN_core_reset_guard_end_ <= ORIGIN(scratch_y) + LENGTH(scratch_y) - 256,
           "ERROR: the CoreResetGuard routine runs into the boot ROM's boot2 area at the top of SRAM5")
    ASSERT(LOADADDR(.core_reset_guard) + SIZEOF(.core_reset_guard) <= ORIGIN(flash) + LENGTH(flash) - 4K,
           "ERROR: the load image of the scratch banks and the CoreResetGuard routine runs into .eeprom")
}

/* The end of the flash image (Kvasir_SDK common.ld, Kvasir::Flash::Region::outsideImage()): the load images above
   follow .data's in this order, .scratch_x, .scratch_y, .core_reset_guard; empty ones take no room. MAX in case an
   empty section's LOADADDR is not where the chain left it. A load image added to this script moves this line. */
_LINKER_INTERN_image_end_flash_ = MAX(_LINKER_INTERN_data_end_flash_,
                                      LOADADDR(.core_reset_guard) + SIZEOF(.core_reset_guard));

/* The flash's last 4 KiB sector, for a firmware's own `.eeprom` data. NOLOAD keeps it out of the image, so a
 * reflash leaves the stored value alone. Kvasir::Flash::CellStore regions are placed by address instead (their
 * outsideImage() check). */
SECTIONS {
    .eeprom (ORIGIN(flash) + LENGTH(flash) - 4K) (NOLOAD) : {
        . = ALIGN(4096);
        KEEP(*(.eeprom*))
    } > flash
}

INCLUDE common.ld
