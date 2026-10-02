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

/* The scratch banks' NOLOAD part (the same block in chip_ram_only.ld): core 1's stack at the bottom of SRAM4 when the
   project asks for it (CORE1_STACK_PLACEMENT scratch -> cmake_core1_stack_in_scratch, Kvasir_SDK util.cmake; the
   stack runs down out of the bank into the top of striped RAM, never into the bank's objects above it), then the
   KVASIR_COREn_BSS objects, zeroed by the chip's ExtraMemoryInit. Placed BEFORE common_ram.ld: an input section
   goes to the first output section statement in script order whose pattern matches it, and common's .bss takes
   *(.bss*). Without either user both sections are empty and every other section is where it was. */
SECTIONS {
    .scratch_x_bss (NOLOAD) : AT(ADDR(.scratch_x_bss)) {
        . = ALIGN(8);
        _LINKER_INTERN_stack1_scratch_start_ = .;
        . = . + (cmake_core1_stack_in_scratch ? cmake_core1_stack_size : 0);
        . = ALIGN(8);
        _LINKER_INTERN_stack1_scratch_end_ = .;
        _LINKER_INTERN_scratch_x_bss_start_ = .;
        *(SORT_BY_ALIGNMENT(.bss.scratch_x*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_bss_end_ = .;
    } > scratch_x

    .scratch_y_bss (NOLOAD) : AT(ADDR(.scratch_y_bss)) {
        . = ALIGN(4);
        _LINKER_INTERN_scratch_y_bss_start_ = .;
        *(SORT_BY_ALIGNMENT(.bss.scratch_y*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_y_bss_end_ = .;
    } > scratch_y
}

_LINKER_INTERN_scratch_x_start_ = ORIGIN(scratch_x);
_LINKER_INTERN_scratch_x_end_   = ORIGIN(scratch_x) + LENGTH(scratch_x);
_LINKER_INTERN_scratch_y_start_ = ORIGIN(scratch_y);
_LINKER_INTERN_scratch_y_end_   = ORIGIN(scratch_y) + LENGTH(scratch_y);

INCLUDE common_ram.ld

/* The scratch banks' initialised part (KVASIR_COREn_DATA / _CODE, and pico-sdk style .scratch_x.<name>): its load
   image follows .data's in flash and is copied by the chip's ExtraMemoryInit (src/chip/StartUp.hpp). The sections
   are in TARGET_EXTRA_FLASH_SECTIONS (cmake/chip.cmake), or the hex/uf2 would carry no load image for them. */
SECTIONS {
    .scratch_x : AT(_LINKER_INTERN_data_end_flash_) {
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_start_ = .;
        *(SORT_BY_ALIGNMENT(.scratch_x.*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_end_ = .;
    } > scratch_x
    .scratch_y : AT(LOADADDR(.scratch_x) + SIZEOF(.scratch_x)) {
        . = ALIGN(4);
        _LINKER_INTERN_scratch_y_data_start_ = .;
        *(SORT_BY_ALIGNMENT(.scratch_y.*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_y_data_end_ = .;
    } > scratch_y
    _LINKER_INTERN_scratch_x_load_ = LOADADDR(.scratch_x);
    _LINKER_INTERN_scratch_y_load_ = LOADADDR(.scratch_y);
    /* Only an image built with SCRATCH_BANKS (Kvasir_SDK util.cmake) copies and zeroes the banks at boot */
    ASSERT(cmake_scratch_banks || SIZEOF(.scratch_x) + SIZEOF(.scratch_y)
               + (_LINKER_INTERN_scratch_x_bss_end_ - _LINKER_INTERN_scratch_x_bss_start_)
               + (_LINKER_INTERN_scratch_y_bss_end_ - _LINKER_INTERN_scratch_y_bss_start_) == 0,
           "ERROR: objects in the scratch banks (.scratch_x/y.*, .bss.scratch_x/y*) need SCRATCH_BANKS")
    ASSERT(LOADADDR(.scratch_y) + SIZEOF(.scratch_y) <= ORIGIN(flash) + LENGTH(flash) - 4K,
           "ERROR: the scratch banks' load image runs into .eeprom")
}

/* The flash's last 4 KiB sector is Kvasir::Flash::SimpleEeprom's (.eeprom in flash.hpp): the
 * value is read through XIP at this address and the sector rewritten in place. NOLOAD keeps
 * it out of the image, so a reflash leaves the stored value alone. */
SECTIONS {
    .eeprom (ORIGIN(flash) + LENGTH(flash) - 4K) (NOLOAD) : {
        . = ALIGN(4096);
        KEEP(*(.eeprom*))
    } > flash
}

INCLUDE common.ld
