#pragma once

// the specialisations name the chip's interrupts
#include "Interrupt.hpp"
#include "rp_common/PinConfig.hpp"

namespace Kvasir { namespace PinConfig {

    template<>
    struct ChipTraits<Chip::Part::rp2040> {
        static constexpr std::size_t pinCount
          = static_cast<std::size_t>(pinCountOf(Chip::Part::rp2040));
    };

    template<>
    struct DmaTraits<Chip::Part::rp2040> {
        static constexpr std::size_t channelCount   = 12;
        static constexpr std::size_t interruptCount = 2;

        static constexpr auto interrupts
          = brigand::list<decltype(Kvasir::Interrupt::dma_0), decltype(Kvasir::Interrupt::dma_1)>{};
    };

    template<>
    struct PwmTraits<Chip::Part::rp2040> {
        static constexpr auto interrupts = brigand::list<decltype(Kvasir::Interrupt::pwm_wrap)>{};
    };

    // the pins' UART/SPI/I2C roles: rp_common/PinConfig.hpp functionSelect (formulas from Table 2)

}}   // namespace Kvasir::PinConfig
