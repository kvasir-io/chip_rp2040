#pragma once
#include "core/CoreInterrupts.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Port/Interrupts.hpp"
#include "peripherals/Interrupts.hpp"
#include "rp2040.hpp"

#include <array>

namespace Kvasir {
// The core's exceptions (core/CoreInterrupts.hpp) and the device lines of chip.svd (svd_converter:
// peripherals/Interrupts.hpp, the SVD's names lower-cased without IRQ: TIMER_IRQ_0 -> timer_0). Lines 26..31 are
// tied to zero but can be forced through NVIC_ISPR (RP2040 datasheet 2.3.2): chip.svd names them SPARE_IRQ_0..5, so
// a forced one has a vector entry and hits the default handler instead of a fetch past the table.
struct Interrupt
  : CoreInterrupts
  , Peripheral::DeviceInterrupts {};

namespace Nvic {
    template<>
    struct InterruptOffsetTraits<Kvasir::Tag::User>
      : Core::ChipInterruptTraits<Core::InterruptRules,
                                  Peripheral::interruptEnd,
                                  Peripheral::interruptGaps> {
        // Lines that exist once per core: "there is one GPIO interrupt per bank, per core. These are completely
        // independent" (RP2040 datasheet 2.3.2, md l.2923), so both cores may install an IO_BANK0/IO_QSPI handler
        // (each enables through its own PROCn_INTE). Every other line, SIO_IRQ_PROC0/1 included, reaches both NVICs
        // and belongs to one core (Startup refuses it in both vector tables).
        static constexpr std::array perCore = {Interrupt::io_bank0.value, Interrupt::io_qspi.value};
    };
}   // namespace Nvic

}   // namespace Kvasir
