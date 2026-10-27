/* The scratch banks' NOLOAD part, INCLUDEd by chip.ld and chip_ram_only.ld before the SDK's RAM script
   (common_ram.ld / common_ram_only.ld): an input section goes to the first output section statement in script order
   whose pattern matches it, and common's .bss takes *(.bss*). Core 1's stack at the bottom of SRAM4 when the project
   asks for it (CORE1_STACK_PLACEMENT scratch -> cmake_core1_stack_in_scratch, Kvasir_SDK util.cmake; the stack runs
   down out of the bank into the top of striped RAM, never into the bank's objects above it), then the
   KVASIR_COREn_BSS objects, zeroed by the chip's ExtraMemoryInit. Without either user both sections are empty and
   every other section is where it was. The banks themselves: chip.ld's address map. */
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
