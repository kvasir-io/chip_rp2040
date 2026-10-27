#pragma once
#include "BootTraits.hpp"
#include "core/core.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/StartUp/MemoryInit.hpp"
#include "kvasir/Util/attributes.hpp"
#include "rp_common/BootSequence.hpp"
#include "rp_common/Rosc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#ifdef KVASIR_CORE_SCRATCH
// linker/chip.ld, chip_ram_only.ld
extern "C" {
extern std::byte _LINKER_INTERN_scratch_x_load_;
extern std::byte _LINKER_INTERN_scratch_x_data_start_;
extern std::byte _LINKER_INTERN_scratch_x_data_end_;
extern std::byte _LINKER_INTERN_scratch_x_bss_start_;
extern std::byte _LINKER_INTERN_scratch_x_bss_end_;
extern std::byte _LINKER_INTERN_scratch_y_load_;
extern std::byte _LINKER_INTERN_scratch_y_data_start_;
extern std::byte _LINKER_INTERN_scratch_y_data_end_;
extern std::byte _LINKER_INTERN_scratch_y_bss_start_;
extern std::byte _LINKER_INTERN_scratch_y_bss_end_;
}
#endif

namespace Kvasir { namespace Startup {
    // inline: one copy per image whichever TUs include this header (a second copy overflows the 256-byte .boot2 and
    // fails chip.ld's ASSERT); KVASIR_SECTION keeps the section under gcc's LTO (kvasir/Util/attributes.hpp)
    [[gnu::used,
      KVASIR_SECTION(
        ".boot2")]] inline constexpr std::array<std::uint32_t, 64> secondStageBootloader{
      0x4b32b500, 0x60582021, 0x21026898, 0x60984388, 0x611860d8, 0x4b2e6158, 0x60992100,
      0x61592102, 0x22f02101, 0x492b5099, 0x21016019, 0x20356099, 0xf844f000, 0x42902202,
      0x2106d014, 0xf0006619, 0x6e19f834, 0x66192101, 0x66182000, 0xf000661a, 0x6e19f82c,
      0x6e196e19, 0xf0002005, 0x2101f82f, 0xd1f94208, 0x60992100, 0x6019491b, 0x60592100,
      0x481b491a, 0x21016001, 0x21eb6099, 0x21a06619, 0xf0006619, 0x2100f812, 0x49166099,
      0x60014814, 0x60992101, 0x2800bc01, 0x4700d000, 0x49134812, 0xc8036008, 0x8808f380,
      0xb5034708, 0x20046a99, 0xd0fb4201, 0x42012001, 0xbd03d1f8, 0x6618b502, 0xf7ff6618,
      0x6e18fff2, 0xbd026e18, 0x40020000, 0x18000000, 0x00070000, 0x005f0300, 0x00002221,
      0x180000f4, 0xa0002022, 0x10000100, 0xe000ed08, 0x00000000, 0x00000000, 0x00000000,
      0x7a4eb274};

    // RP boot sequence (rp_common/BootSequence.hpp) over the RP2040's steps (chip/BootTraits.hpp)
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() { Rp::firstInit<Rp::BootTraits>(); }
    };

}}   // namespace Kvasir::Startup

#if KVASIR_RP_CORE_RESET_GUARDED
    // Multicore flash images only (armed by SecondaryCoreInit::armCoreResetGuard); included below FirstInitStep so
    // its source lines, which a sanitizer records, do not move
    #include "CoreResetGuard.hpp"
#endif

namespace Kvasir { namespace Startup {

    // The second core (kvasir/StartUp/SecondaryCore.hpp). What core 1 has to do for itself
    // is nothing on the RP2040: no coprocessor to enable, no exclusives to route, so only
    // the bootrom handshake and the PSM reset are here. The atomic shim's cross-core lock
    // for a multicore build is chip/CrossCoreLock.hpp (Port::CrossCoreLock, included below).
    template<typename... Ts>
    struct SecondaryCoreInit<Tag::User, Ts...>
      : Rp::SecondaryCoreInitBase<
#if KVASIR_RP_CORE_RESET_GUARDED
          Kvasir::CoreResetGuard
#else
          Rp::NoGuard
#endif
          > {
        static constexpr std::uint32_t cpacrEnable = Core::SecondaryCoreTraits::cpacrEnable;

        void operator()() {}
    };

#ifdef KVASIR_CORE_SCRATCH
    // SCRATCH_BANKS (Kvasir_SDK util.cmake): the SRAM4/SRAM5 sections of linker/chip.ld - copy
    // the KVASIR_COREn_DATA/_CODE load images out of flash, zero KVASIR_COREn_BSS. Right after
    // initMemory(), before any constructor. Core 1's stack in SRAM4 is not touched here:
    // SecondaryCore fills it before the launch. In a RAM-only image the load image is where it
    // runs and nothing is copied. Without SCRATCH_BANKS this does not exist, and chip.ld
    // refuses scratch-bank objects.
    template<typename... Ts>
    struct ExtraMemoryInit<Tag::User, Ts...> {
        // the SDK's helpers (kvasir/StartUp/MemoryInit.hpp), with start/end symbols
        [[gnu::always_inline]] static void copy(std::byte const* load,
                                                std::byte*       start,
                                                std::byte*       end) {
            copyImage(load, start, static_cast<std::size_t>(end - start), true);
        }

        [[gnu::always_inline]] static void zero(std::byte* start,
                                                std::byte* end) {
            zeroRange(start, static_cast<std::size_t>(end - start));
        }

        [[gnu::always_inline]] void operator()() const {
            copy(&_LINKER_INTERN_scratch_x_load_,
                 &_LINKER_INTERN_scratch_x_data_start_,
                 &_LINKER_INTERN_scratch_x_data_end_);
            copy(&_LINKER_INTERN_scratch_y_load_,
                 &_LINKER_INTERN_scratch_y_data_start_,
                 &_LINKER_INTERN_scratch_y_data_end_);
            zero(&_LINKER_INTERN_scratch_x_bss_start_, &_LINKER_INTERN_scratch_x_bss_end_);
            zero(&_LINKER_INTERN_scratch_y_bss_start_, &_LINKER_INTERN_scratch_y_bss_end_);
        }
    };
#endif

    // The stack guard's per-boot value (Kvasir_SDK StartUp.hpp seedStackGuard): 32 reads of
    // ROSC.RANDOMBIT (RP2040 datasheet 2.17.8 Table 264, md line 10726). Random only while the
    // cores do not run from the ROSC (2.17.5, md line 10685): read after coreClockInit(), which
    // moves clk_sys to pll_sys and leaves the ROSC running (2.17.1, md line 10635). A ROSC that is
    // not stable (STATUS.STABLE, Table 271) gives the old constant. NOT cryptographic (2.17.5,
    // Table 272): enough to make the canary differ between boots; seedStackGuard zeroes the low byte.
    template<typename... Ts>
    struct StackGuardEntropy<Tag::User, Ts...> {
        [[gnu::always_inline]] std::uint32_t operator()() const {
            if(!Rosc::stable()) { return 0xdeadc0deU; }
            std::uint32_t bits{};
            for(unsigned i = 0; i < 32; ++i) { bits = (bits << 1U) | Rosc::randomBit(); }
            return bits;
        }
    };
}}   // namespace Kvasir::Startup

#if defined(KVASIR_MULTICORE) && KVASIR_MULTICORE
    // Port::CrossCoreLock<Tag::User>: the atomic shim's lock, which the main TU's kvasir/StartUp/StartUp.hpp defines its
    // extern "C" entry points on (kvasir/StartUp/Runtime/LibAtomic.hpp; read late, so before or after it)
    #include "CrossCoreLock.hpp"
#endif

// An address no bus slave answers at, for Kvasir::Fault::Inject (kvasir/Runtime/FaultInject.hpp): 0xF0000000 lies
// in none of the address map's segments (RP2040 datasheet 2.2.1 Table 15: ROM 0x0, XIP 0x1, SRAM 0x2, APB 0x4,
// AHB-Lite 0x5, IOPORT 0xd, M0+ internal 0xe), and "unmapped address ranges raise a bus error when accessed" (2.2); the
// M0+ puts every access above 0xEFFFFFFF on its AHB-Lite port (2.4.3.4, Cortex-M0+ TRM DDI0484C 3.4): a HardFault.
template<>
struct Kvasir::Port::UnmappedAddress<Kvasir::Tag::User> {
    static constexpr std::uintptr_t value = 0xF000'0000U;
};

// Startup::second_stage_bootloader (.boot2, at the top of this file) is pico-sdk 2.3.1's
// src/rp2040/boot_stage2/boot2_w25q080.S built with the settings of its default board header (boards/pico.h: W25Q080,
// PICO_FLASH_SPI_CLKDIV 2) and padded by its pad_checksum -s 0xffffffff - the 256 bytes compared equal. The boot ROM
// copies it to SRAM5 and enters it only if the last word is the CRC32 of the first 252 bytes: polynomial 0x04c11db7,
// no reflection, initial 0xffffffff, no final XOR, the checksum little-endian (RP2040 datasheet 2.8.1.3.1, md l.6364)
// - CRC-32/MPEG-2. Words changed without their checksum would drop every image into the USB boot.
#include "kvasir/Util/Crc.hpp"

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
  "chip/StartUp.hpp: the last .boot2 word is not the boot ROM's CRC32 of the first 252 bytes");
