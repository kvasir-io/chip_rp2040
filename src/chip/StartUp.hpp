#pragma once
#include "core/core.hpp"
#include "kvasir/Common/Core.hpp"
#include "peripherals/ROSC.hpp"
#include "rp_common/Multicore.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(KVASIR_CORE_SCRATCH)
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
    [[gnu::used,
      gnu::section(
        ".boot2")]] static constexpr std::array<std::uint32_t, 64> second_stage_bootloader{
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

    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() {
            Core::startup();

            using Reset = Kvasir::Peripheral::RESETS::Registers<>::RESET;
            apply(set(Reset::usbctrl),
                  set(Reset::uart1),
                  set(Reset::uart0),
                  set(Reset::timer),
                  set(Reset::tbman),
                  clear(Reset::sysinfo),
                  clear(Reset::syscfg),
                  set(Reset::spi1),
                  set(Reset::spi0),
                  set(Reset::rtc),
                  set(Reset::pwm),
                  set(Reset::pll_usb),
                  clear(Reset::pll_sys),
                  set(Reset::pio1),
                  set(Reset::pio0),
                  clear(Reset::pads_qspi),
                  set(Reset::pads_bank0),
                  set(Reset::jtag),
                  clear(Reset::io_qspi),
                  set(Reset::io_bank0),
                  set(Reset::i2c1),
                  set(Reset::i2c0),
                  set(Reset::dma),
                  set(Reset::busctrl),
                  set(Reset::adc));

            using PSM_WDSEL = Kvasir::Peripheral::PSM::Registers<>::WDSEL;
            apply(set(PSM_WDSEL::proc1),
                  set(PSM_WDSEL::proc0),
                  set(PSM_WDSEL::sio),
                  set(PSM_WDSEL::vreg_and_chip_reset),
                  set(PSM_WDSEL::xip),
                  set(PSM_WDSEL::sram5),
                  set(PSM_WDSEL::sram4),
                  set(PSM_WDSEL::sram3),
                  set(PSM_WDSEL::sram2),
                  set(PSM_WDSEL::sram1),
                  set(PSM_WDSEL::sram0),
                  set(PSM_WDSEL::rom),
                  set(PSM_WDSEL::busfabric),
                  set(PSM_WDSEL::resets),
                  set(PSM_WDSEL::clocks),
                  set(PSM_WDSEL::xosc),
                  set(PSM_WDSEL::rosc));

            using WDSEL = Kvasir::Peripheral::RESETS::Registers<>::WDSEL;
            apply(set(WDSEL::usbctrl),
                  set(WDSEL::uart1),
                  set(WDSEL::uart0),
                  set(WDSEL::timer),
                  set(WDSEL::tbman),
                  set(WDSEL::sysinfo),
                  set(WDSEL::syscfg),
                  set(WDSEL::spi1),
                  set(WDSEL::spi0),
                  set(WDSEL::rtc),
                  set(WDSEL::pwm),
                  set(WDSEL::pll_usb),
                  set(WDSEL::pll_sys),
                  set(WDSEL::pio1),
                  set(WDSEL::pio0),
                  set(WDSEL::pads_qspi),
                  set(WDSEL::pads_bank0),
                  set(WDSEL::jtag),
                  set(WDSEL::io_qspi),
                  set(WDSEL::io_bank0),
                  set(WDSEL::i2c1),
                  set(WDSEL::i2c0),
                  set(WDSEL::dma),
                  set(WDSEL::busctrl),
                  set(WDSEL::adc));
        }
    };

    // The second core (kvasir/StartUp/SecondaryCore.hpp). What core 1 has to do for itself
    // is nothing on the RP2040: no coprocessor to enable, no exclusives to route, so only
    // the bootrom handshake and the PSM reset are here. The atomic shim's cross-core lock
    // for a multicore build is chip/CrossCoreLock.hpp.
    template<typename... Ts>
    struct SecondaryCoreInit<Tag::User, Ts...> {
        static constexpr std::uint32_t cpacrEnable = 0;

        void operator()() {}

        [[nodiscard]] static bool launch(std::uint32_t entry,
                                         std::uint32_t sp,
                                         std::uint32_t vtor) {
            return Multicore::launchCore1(entry, sp, vtor);
        }

        static void reset() { Multicore::resetCore1(); }
    };

#if defined(KVASIR_CORE_SCRATCH)
    // SCRATCH_BANKS (Kvasir_SDK util.cmake): the SRAM4/SRAM5 sections of linker/chip.ld - copy
    // the KVASIR_COREn_DATA/_CODE load images out of flash, zero KVASIR_COREn_BSS. Right after
    // initMemory(), before any constructor. Core 1's stack in SRAM4 is not touched here:
    // SecondaryCore fills it before the launch. In a RAM-only image the load image is where it
    // runs and nothing is copied. Without SCRATCH_BANKS this does not exist, and chip.ld
    // refuses scratch-bank objects.
    template<typename... Ts>
    struct ExtraMemoryInit<Tag::User, Ts...> {
        [[gnu::always_inline]] static void copy(std::byte const* from,
                                                std::byte*       to,
                                                std::byte*       end) {
            // the linker's symbols are distinct objects to the compiler: hide them from it
            asm("" : "+l"(from), "+l"(to), "+l"(end));
            if(from != to) { std::memcpy(to, from, static_cast<std::size_t>(end - to)); }
        }

        [[gnu::always_inline]] static void zero(std::byte* from,
                                                std::byte* end) {
            asm("" : "+l"(from), "+l"(end));
            std::memset(from, 0, static_cast<std::size_t>(end - from));
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
            using ROSC = Kvasir::Peripheral::ROSC::Registers<>;
            if(!apply(read(ROSC::STATUS::stable))) { return 0xdeadc0deU; }
            std::uint32_t bits{};
            for(unsigned i = 0; i < 32; ++i) {
                bits = (bits << 1U) | apply(read(ROSC::RANDOMBIT::randombit));
            }
            return bits;
        }
    };
}}   // namespace Kvasir::Startup

#include "kvasir/StartUp/StartUp.hpp"
