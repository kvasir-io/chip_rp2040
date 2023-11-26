#pragma once
#include <peripherals/PIO.hpp>

namespace Kvasir { namespace Pio {
    template<unsigned Instance>
    static constexpr auto getEnable() {
        if constexpr(Instance == 0) {
            return clear(Peripheral::RESETS::Registers<>::RESET::pio0);
        } else if(Instance == 1) {
            return clear(Peripheral::RESETS::Registers<>::RESET::pio1);
        }
    }

    template<unsigned Instance, typename Pin>
    static constexpr auto getPinConfig(Pin) {
        if constexpr(Instance == 0) {
            return action(Kvasir::Io::Action::PinFunction<6>{}, Pin{});
        } else if(Instance == 1) {
            return action(Kvasir::Io::Action::PinFunction<7>{}, Pin{});
        }
    }

    static constexpr auto getDiv(double div) {
        std::uint16_t div_int  = static_cast<std::uint8_t>(div);
        std::uint8_t  div_frac = static_cast<std::uint8_t>((div - div_int) * 256.0);
        return std::pair<std::uint16_t, std::uint8_t>(div_int, div_frac);
    }

    template<typename SM, typename Div>
    static constexpr auto getDivConfig(Div) {
        constexpr auto divs = getDiv(Div{}());

        return list(
          write(SM::CLKDIV::_int, Kvasir::Register::value<std::get<0>(divs)>()),
          write(SM::CLKDIV::frac, Kvasir::Register::value<std::get<1>(divs)>()));
    }

    template<typename Dma, unsigned PioInstance, unsigned SmInstance>
    static constexpr typename Dma::TriggerSource getTxDmaTrigger() {
        if constexpr(PioInstance == 0) {
            if constexpr(SmInstance == 0) {
                return Dma::TriggerSource::pio0_tx0;
            }
            if constexpr(SmInstance == 1) {
                return Dma::TriggerSource::pio0_tx1;
            }
            if constexpr(SmInstance == 2) {
                return Dma::TriggerSource::pio0_tx2;
            }
            if constexpr(SmInstance == 3) {
                return Dma::TriggerSource::pio0_tx3;
            }
        } else {
            if constexpr(SmInstance == 0) {
                return Dma::TriggerSource::pio1_tx0;
            }
            if constexpr(SmInstance == 1) {
                return Dma::TriggerSource::pio1_tx1;
            }
            if constexpr(SmInstance == 2) {
                return Dma::TriggerSource::pio1_tx2;
            }
            if constexpr(SmInstance == 3) {
                return Dma::TriggerSource::pio1_tx3;
            }
        }
    }
    template<typename Dma, unsigned PioInstance, unsigned SmInstance>
    static constexpr typename Dma::TriggerSource getRxDmaTrigger() {
        if constexpr(PioInstance == 0) {
            if constexpr(SmInstance == 0) {
                return Dma::TriggerSource::pio0_rx0;
            }
            if constexpr(SmInstance == 1) {
                return Dma::TriggerSource::pio0_rx1;
            }
            if constexpr(SmInstance == 2) {
                return Dma::TriggerSource::pio0_rx2;
            }
            if constexpr(SmInstance == 3) {
                return Dma::TriggerSource::pio0_rx3;
            }
        } else {
            if constexpr(SmInstance == 0) {
                return Dma::TriggerSource::pio1_rx0;
            }
            if constexpr(SmInstance == 1) {
                return Dma::TriggerSource::pio1_rx1;
            }
            if constexpr(SmInstance == 2) {
                return Dma::TriggerSource::pio1_rx2;
            }
            if constexpr(SmInstance == 3) {
                return Dma::TriggerSource::pio1_rx3;
            }
        }
    }

}}   // namespace Kvasir::Pio
