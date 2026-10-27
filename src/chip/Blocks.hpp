#pragma once

// The RP2040's gated blocks: every block FirstInitStep holds in reset, with its RESETS field, interrupt lines and
// DMA requests (chip_rp_common Block.hpp explains the row; kvasir/Hal/Block.hpp the check it drives). A row names its
// block's base address as a number from the address map (RP2040 datasheet 2.2.2 "Detail", APB md l.1297-1334, AHB
// md l.1336-1343): APB blocks are 16 KiB apart, AHB blocks 1 MiB. `Rp::Block<Regs>` finds a row by the generated
// `Regs::baseAddr`, so a wrong number is "no block of the chip's table" at the first driver, and tests/conformance
// asserts every row against its register set. The table includes only RESETS, the interrupt list and (through
// Block.hpp) DMA's request enum: a TU that names `Rp::Block<Regs>` includes Regs' own header.

#include "rp_common/Block.hpp"

#include <peripherals/Interrupts.hpp>
#include <peripherals/RESETS.hpp>

namespace Kvasir { namespace Chip { namespace Blocks {
    using Reset = Peripheral::RESETS::Registers<>::RESET;
    using Done  = Peripheral::RESETS::Registers<>::RESET_DONE;
    using D     = ::Kvasir::Rp::Dreq;
    using I     = Peripheral::DeviceInterrupts;
    template<typename... Is>
    using Irqs = brigand::list<Is...>;
    template<std::uint32_t Base,
             std::uint32_t Size,
             auto const&   R,
             auto const&   Dn,
             typename Is = brigand::list<>,
             D Tx        = D::permanent,
             D Rx        = D::permanent>
    using Row = ::Kvasir::Rp::BlockRow<Base, Size, R, Dn, Is, Tx, Rx>;

    using Table
      = ::Kvasir::Hal::BlockTable<Row<0x4004c000,
                                      0x4000,
                                      Reset::adc,
                                      Done::adc,
                                      Irqs<decltype(I::adc_fifo)>,
                                      D::permanent,
                                      D::adc>,
                                  Row<0x40030000, 0x4000, Reset::busctrl, Done::busctrl>,
                                  Row<0x50000000, 0x100000, Reset::dma, Done::dma>,
                                  Row<0x40044000,
                                      0x4000,
                                      Reset::i2c0,
                                      Done::i2c0,
                                      Irqs<decltype(I::i2c0)>,
                                      D::i2c0_tx,
                                      D::i2c0_rx>,
                                  Row<0x40048000,
                                      0x4000,
                                      Reset::i2c1,
                                      Done::i2c1,
                                      Irqs<decltype(I::i2c1)>,
                                      D::i2c1_tx,
                                      D::i2c1_rx>,
                                  Row<0x40014000, 0x4000, Reset::io_bank0, Done::io_bank0>,
                                  Row<0x4001c000, 0x4000, Reset::pads_bank0, Done::pads_bank0>,
                                  Row<0x50200000,
                                      0x100000,
                                      Reset::pio0,
                                      Done::pio0,
                                      Irqs<decltype(I::pio0_0), decltype(I::pio0_1)>,
                                      D::pio0_tx0,
                                      D::pio0_rx0>,
                                  Row<0x50300000,
                                      0x100000,
                                      Reset::pio1,
                                      Done::pio1,
                                      Irqs<decltype(I::pio1_0), decltype(I::pio1_1)>,
                                      D::pio1_tx0,
                                      D::pio1_rx0>,
                                  Row<0x4002c000, 0x4000, Reset::pll_usb, Done::pll_usb>,
                                  Row<0x40050000, 0x4000, Reset::pwm, Done::pwm>,
                                  Row<0x4005c000, 0x4000, Reset::rtc, Done::rtc>,
                                  Row<0x4003c000,
                                      0x4000,
                                      Reset::spi0,
                                      Done::spi0,
                                      Irqs<decltype(I::spi0)>,
                                      D::spi0_tx,
                                      D::spi0_rx>,
                                  Row<0x40040000,
                                      0x4000,
                                      Reset::spi1,
                                      Done::spi1,
                                      Irqs<decltype(I::spi1)>,
                                      D::spi1_tx,
                                      D::spi1_rx>,
                                  Row<0x4006c000, 0x4000, Reset::tbman, Done::tbman>,
                                  Row<0x40054000, 0x4000, Reset::timer, Done::timer>,
                                  Row<0x40034000,
                                      0x4000,
                                      Reset::uart0,
                                      Done::uart0,
                                      Irqs<decltype(I::uart0)>,
                                      D::uart0_tx,
                                      D::uart0_rx>,
                                  Row<0x40038000,
                                      0x4000,
                                      Reset::uart1,
                                      Done::uart1,
                                      Irqs<decltype(I::uart1)>,
                                      D::uart1_tx,
                                      D::uart1_rx>,
                                  Row<0x50100000,
                                      0x100000,
                                      Reset::usbctrl,
                                      Done::usbctrl>   // DPRAM and USB registers (0x50110000)
                                  >;

}}}   // namespace Kvasir::Chip::Blocks

template<>
struct Kvasir::Hal::BlockGates<Kvasir::Tag::User> {
    using type = Kvasir::Chip::Blocks::Table;
};

namespace Kvasir { namespace Chip { namespace Blocks {
    // Every RESETS bit is either a row, kept running by FirstInitStep, or has no driver (jtag).
    inline constexpr std::uint32_t rowMask
      = []<typename... Rows>(::Kvasir::Hal::BlockTable<Rows...>*) {
            return (0U | ... | (1U << Rows::id));
        }(static_cast<Table*>(nullptr));
    static_assert((rowMask | Reset::sysinfo.mask | Reset::syscfg.mask | Reset::pll_sys.mask
                   | Reset::pads_qspi.mask | Reset::io_qspi.mask | Reset::jtag.mask)
                    == 0x01FFFFFFU,
                  "a RESETS bit is neither a block row nor released by FirstInitStep");
    static_assert((rowMask
                   & (Reset::sysinfo.mask | Reset::syscfg.mask | Reset::pll_sys.mask
                      | Reset::pads_qspi.mask | Reset::io_qspi.mask))
                    == 0U,
                  "a block FirstInitStep keeps running is not claimed");
}}}   // namespace Kvasir::Chip::Blocks
