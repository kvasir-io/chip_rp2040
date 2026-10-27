/* The scratch banks' initialised part (KVASIR_COREn_DATA / _CODE, and pico-sdk style .scratch_x.<name>), INCLUDEd by
   chip.ld and chip_ram_only.ld after the SDK's RAM script. Loaded, so the hex/uf2 carry it (Kvasir_SDK
   cmake/tools/elf_image.py takes every section the linker loads). Its load image goes where the image keeps .data's,
   by the SDK's own convention for a RAM-only image (common.ld: _LINKER_INTERN_data_end_flash_ is 0 there,
   common_ram_only.ld) - no symbol of the including script:
   - a flash image (chip.ld): in flash right after .data's load image, copied to the banks by the chip's
     ExtraMemoryInit (src/chip/StartUp.hpp);
   - a RAM-only image (chip_ram_only.ld): load address = run address, ExtraMemoryInit copies nothing. */
SECTIONS {
    .scratch_x : AT(_LINKER_INTERN_data_end_flash_ != 0 ? _LINKER_INTERN_data_end_flash_ : ADDR(.scratch_x)) {
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_start_ = .;
        *(SORT_BY_ALIGNMENT(.scratch_x.*))
        . = ALIGN(4);
        _LINKER_INTERN_scratch_x_data_end_ = .;
    } > scratch_x
    .scratch_y : AT(_LINKER_INTERN_data_end_flash_ != 0 ? LOADADDR(.scratch_x) + SIZEOF(.scratch_x)
                                                         : ADDR(.scratch_y)) {
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
