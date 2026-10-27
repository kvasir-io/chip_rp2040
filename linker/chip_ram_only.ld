/* RP2040, RAM only: the whole image in SRAM, nothing in flash and no second-stage bootloader.
   For images loaded by the bootrom's UF2 path or a debugger straight into RAM (flash/nuke).
   The UF2 path takes "0x20000000-0x20042000 Main RAM" (RP2040 datasheet 2.8.4.2, md line 6907), the scratch banks
   included; chip.ld has the address map. */
MEMORY{
    ram       (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
    scratch_x (xrw) : ORIGIN = 0x20040000, LENGTH = 4K
    scratch_y (xrw) : ORIGIN = 0x20041000, LENGTH = 4K
}

/* The scratch banks' NOLOAD part, before common_ram_only.ld (the first-match order: in the file) */
INCLUDE scratch_banks_noload.inc.ld

/* An image that links boot2 has it in ram: named here (where it landed as an orphan before, right after .vectors; no
   KEEP, so an image without it still has none), so the stack (common_stack_body.inc.ld) can leave room for exactly
   its size - GNU ld refuses SIZEOF() of an orphan ("undefined section"). Assigned before the INCLUDE: assigned after
   it, lld takes the value one layout pass late (the stack comes out too big). */
SECTIONS {
    .boot2 : {
        *(.boot2)
    } > ram
} INSERT AFTER .vectors;
_LINKER_INTERN_chip_ram_other_size_ = SIZEOF(.boot2);

INCLUDE common_ram_only.ld

/* The scratch banks' initialised part, loaded where it runs (scratch_banks_data.inc.ld) */
INCLUDE scratch_banks_data.inc.ld

/* Kvasir::CoreResetGuard (src/chip/CoreResetGuard.hpp) needs a flash image: the boot ROM never boots a RAM image */
SECTIONS {
    .core_reset_guard : { *(.core_reset_guard*) } > scratch_y
    ASSERT(SIZEOF(.core_reset_guard) == 0, "ERROR: Kvasir::CoreResetGuard needs a flash image, not RAM_ONLY")
}
_LINKER_INTERN_core_reset_guard_start_ = 0;
_LINKER_INTERN_core_reset_guard_end_   = 0;
_LINKER_INTERN_core_reset_guard_load_  = 0;

INCLUDE common.ld
