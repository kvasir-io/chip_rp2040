/* Linker script for RP2040 with 2MB Flash */

MEMORY{
    boot2  ( r ) : ORIGIN = 0x10000000, LENGTH = 256
    flash  (xr ) : ORIGIN = 0x10000100, LENGTH = 2096896
    ram    (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
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
/*INCLUDE common_eeprom.ld*/
INCLUDE common_ram.ld

INCLUDE common.ld

