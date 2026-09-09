/* RP2040, RAM only: the whole image in SRAM, nothing in flash and no second-stage bootloader.
   For images loaded by the bootrom's UF2 path or a debugger straight into RAM (flash/nuke). */
MEMORY{
    ram    (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
}

INCLUDE common_ram_only.ld

INCLUDE common.ld
