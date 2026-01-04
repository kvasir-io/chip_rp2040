include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../src/chip/rp_common/pioasm ${CMAKE_BINARY_DIR}/chip_pioasm)

set(TARGET_MPU RP2040_M0_0)
set(TARGET_UF2_CODE 0xE48BFF56)
set(TARGET_FLASH_SIZE 2096896)
set(TARGET_RAM_SIZE 262144)
set(TARGET_EEPROM_SIZE 0)
set(TARGET_EXTRA_FLASH_SECTIONS .boot2)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld)

set(CHIP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/../src/chip/divider.S PROPERTIES COMPILE_FLAGS "-Wno-c++-keyword")
set(CHIP_LINKER_OPTIONS --wrap=__aeabi_idiv --wrap=__aeabi_idivmod --wrap=__aeabi_ldivmod --wrap=__aeabi_uidiv
                        --wrap=__aeabi_uidivmod --wrap=__aeabi_uldivmod)

svd_convert(peripherals SVD_FILE ${CMAKE_CURRENT_LIST_DIR}/../chip.svd OUTPUT_DIRECTORY peripherals)
