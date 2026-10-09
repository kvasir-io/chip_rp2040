/* RP2040, RAM only: the whole image in SRAM, nothing in flash and no second-stage bootloader.
   For images loaded by the bootrom's UF2 path or a debugger straight into RAM (flash/nuke).
   The UF2 path takes "0x20000000-0x20042000 Main RAM" (RP2040 datasheet 2.8.4.2, md line 6907), the scratch banks
   included; chip.ld has the address map. */
MEMORY{
    ram       (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
    scratch_x (xrw) : ORIGIN = 0x20040000, LENGTH = 4K
    scratch_y (xrw) : ORIGIN = 0x20041000, LENGTH = 4K
}

/* The scratch banks' NOLOAD part (the same block as in chip.ld): core 1's stack at the bottom of SRAM4 when the
   project asks for it (CORE1_STACK_PLACEMENT scratch -> cmake_core1_stack_in_scratch, Kvasir_SDK util.cmake; the
   stack runs down out of the bank into the top of striped RAM, never into the bank's objects above it), then the
   KVASIR_COREn_BSS objects, zeroed by the chip's ExtraMemoryInit. Placed BEFORE common_ram_only.ld: an input section
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

/* The scratch banks' initialised part, loaded where it runs (load address = run address: ExtraMemoryInit copies
   nothing). Loaded, so the hex the debugger loads carries it (Kvasir_SDK cmake/tools/elf_image.py). */
SECTIONS {
    .scratch_x : AT(ADDR(.scratch_x)) {
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_start_ = .;
        *(SORT_BY_ALIGNMENT(.scratch_x.*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_end_ = .;
    } > scratch_x
    .scratch_y : AT(ADDR(.scratch_y)) {
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
}

INCLUDE common.ld
