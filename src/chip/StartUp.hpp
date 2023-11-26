#pragma once
#include "core/core.hpp"
#include "kvasir/Common/Core.hpp"

#include <array>
#include <cstdint>

namespace Kvasir { namespace Startup {
    [[gnu::used, gnu::section(".boot2")]] static constexpr std::array<std::uint32_t, 64>
      second_stage_bootloader{
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
            apply(
              set(Reset::usbctrl),
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
            apply(
              set(PSM_WDSEL::proc1),
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
            apply(
              set(WDSEL::usbctrl),
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
}}   // namespace Kvasir::Startup

#include "kvasir/StartUp/StartUp.hpp"
