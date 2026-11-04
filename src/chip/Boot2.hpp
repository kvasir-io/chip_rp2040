#pragma once
// The RP2040's second-stage bootloader, the 256 bytes of .boot2 (Startup::secondStageBootloader). A header of its own:
// chip/StartUp.hpp places it in every image, and rp_common/XipControl.hpp copies it to put XIP back after a flash
// write - without including the start code, which includes rp_common headers itself.
#include "kvasir/Util/Crc.hpp"
#include "kvasir/Util/attributes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace Kvasir::Startup {
// inline: one copy per image whichever TUs include this header (a second copy overflows the 256-byte .boot2 and
// fails chip.ld's ASSERT); KVASIR_SECTION keeps the section under gcc's LTO (kvasir/Util/attributes.hpp)
[[gnu::used,
  KVASIR_SECTION(".boot2")]] inline constexpr std::array<std::uint32_t, 64> secondStageBootloader{
  0x4b32b500, 0x60582021, 0x21026898, 0x60984388, 0x611860d8, 0x4b2e6158, 0x60992100, 0x61592102,
  0x22f02101, 0x492b5099, 0x21016019, 0x20356099, 0xf844f000, 0x42902202, 0x2106d014, 0xf0006619,
  0x6e19f834, 0x66192101, 0x66182000, 0xf000661a, 0x6e19f82c, 0x6e196e19, 0xf0002005, 0x2101f82f,
  0xd1f94208, 0x60992100, 0x6019491b, 0x60592100, 0x481b491a, 0x21016001, 0x21eb6099, 0x21a06619,
  0xf0006619, 0x2100f812, 0x49166099, 0x60014814, 0x60992101, 0x2800bc01, 0x4700d000, 0x49134812,
  0xc8036008, 0x8808f380, 0xb5034708, 0x20046a99, 0xd0fb4201, 0x42012001, 0xbd03d1f8, 0x6618b502,
  0xf7ff6618, 0x6e18fff2, 0xbd026e18, 0x40020000, 0x18000000, 0x00070000, 0x005f0300, 0x00002221,
  0x180000f4, 0xa0002022, 0x10000100, 0xe000ed08, 0x00000000, 0x00000000, 0x00000000, 0x7a4eb274};
}   // namespace Kvasir::Startup

// Startup::secondStageBootloader is pico-sdk 2.3.1's src/rp2040/boot_stage2/boot2_w25q080.S built with the settings
// of its default board header (boards/pico.h: W25Q080, PICO_FLASH_SPI_CLKDIV 2) and padded by its pad_checksum -s 0xffffffff - the 256 bytes compared equal. The boot ROM
// copies it to SRAM5 and enters it only if the last word is the CRC32 of the first 252 bytes: polynomial 0x04c11db7,
// no reflection, initial 0xffffffff, no final XOR, the checksum little-endian (RP2040 datasheet 2.8.1.3.1, md l.6364)
// - CRC-32/MPEG-2. Words changed without their checksum would drop every image into the USB boot.
namespace Kvasir::Startup::Detail {
consteval bool boot2ChecksumHolds() {
    Kvasir::Crc::Engine<Kvasir::Crc::Presets::crc32Mpeg2, 0> crc{};
    for(std::size_t i = 0; i != 252; ++i) {   // the words' bytes in memory order
        crc.update(static_cast<std::byte>(secondStageBootloader[i / 4] >> (8 * (i % 4))));
    }
    return crc.finish() == secondStageBootloader[63];
}
}   // namespace Kvasir::Startup::Detail

static_assert(
  Kvasir::Startup::Detail::boot2ChecksumHolds(),
  "chip/Boot2.hpp: the last .boot2 word is not the boot ROM's CRC32 of the first 252 bytes");
