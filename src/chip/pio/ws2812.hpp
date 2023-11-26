#pragma once
#include "chip/PIO.hpp"
#include "ws2812Pio/ws2812.hpp"

namespace Kvasir { namespace Pio {
    template<
      typename Clock,
      typename Pin,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      typename Config_>
    struct WS2812 {
        struct Config : Config_ {
            static constexpr auto LedClockSpeed = [] {
                if constexpr(requires { Config_::LedClockSpeed; }) {
                    return Config_::LedClockSpeed;
                } else {
                    return 800000;
                }
            }();
            static constexpr auto ProgrammOffset = [] {
                if constexpr(requires { Config_::ProgrammOffset; }) {
                    return Config_::ProgrammOffset;
                } else {
                    return 0;
                }
            }();
        };

        using Programm = Pio::ws2812Programm;

        static constexpr double DivFactor{
          double{Config::ClockSpeed}
          / (double{Config::LedClockSpeed} * double{Programm::CylcesPerBit})};

        using PioRegs = Kvasir::Peripheral::PIO::Registers<Config::PioInstance>;
        using SmRegs  = typename PioRegs::template SM<Config::SmInstance>;

        static constexpr auto powerClockEnable
          = list(Kvasir::Pio::getEnable<Config::PioInstance>());

        static constexpr auto initStepPinConfig
          = list(Kvasir::Pio::getPinConfig<Config::PioInstance>(Pin{}));

        static constexpr auto initStepPeripheryConfig = list(

          Kvasir::Pio::getDivConfig<SmRegs>([]() { return DivFactor; }),

          SmRegs::EXECCTRL::overrideDefaults(
            write(
              SmRegs::EXECCTRL::wrap_bottom,
              Kvasir::Register::value<Programm::WrapTarget + Config::ProgrammOffset>()),
            write(
              SmRegs::EXECCTRL::wrap_top,
              Kvasir::Register::value<Programm::Wrap + Config::ProgrammOffset>())),

          clear(SmRegs::SHIFTCTRL::fjoin_rx),
          write(SmRegs::SHIFTCTRL::fjoin_tx, Kvasir::Register::value<1>()),
          write(SmRegs::SHIFTCTRL::pull_thresh, Kvasir::Register::value<8>()),
          write(SmRegs::SHIFTCTRL::push_thresh, Kvasir::Register::value<0>()),
          write(SmRegs::SHIFTCTRL::out_shiftdir, Kvasir::Register::value<0>()),
          write(SmRegs::SHIFTCTRL::in_shiftdir, Kvasir::Register::value<1>()),
          write(SmRegs::SHIFTCTRL::autopull, Kvasir::Register::value<1>()),
          write(SmRegs::SHIFTCTRL::autopush, Kvasir::Register::value<0>()));

        static void preEnableRuntimeInit() {
            static_assert(
              32 >= Config::ProgrammOffset + Programm::Instructions.size(),
              "to many Instructions");
            for(std::uint16_t volatile* addr = reinterpret_cast<std::uint16_t volatile*>(
                  PioRegs::template INSTR_MEM<Config::ProgrammOffset>::Addr::value);
                auto v : Programm::Instructions)
            {
                *addr = v;
                ++addr;
                ++addr;
            }
            static constexpr auto PinNumber
              = []<int Port, int PinN>(Kvasir::Register::PinLocation<Port, PinN>) {
                    return PinN;
                }(Pin{});

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Kvasir::Register::value<PinNumber>()),
              write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<1>())));

            apply(write(SmRegs::INSTR::instr, Kvasir::Register::value<0xe000 | (4 << 5) | 0x1f>()));

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::sideset_count, Kvasir::Register::value<1>()),
              write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<0>()),
              write(SmRegs::PINCTRL::sideset_base, Kvasir::Register::value<PinNumber>())));
        }

        static constexpr auto initStepPeripheryEnable = list(
          write(PioRegs::CTRL::sm_restart, Kvasir::Register::value<1 << Config::SmInstance>()),
          write(PioRegs::CTRL::clkdiv_restart, Kvasir::Register::value<1 << Config::SmInstance>()),
          //JUMP to programm
          write(SmRegs::INSTR::instr, Kvasir::Register::value<Config::ProgrammOffset>()),
          //enable
          Kvasir::Register::SequencePoint{},
          write(PioRegs::CTRL::sm_enable, Kvasir::Register::value<1 << Config::SmInstance>())

        );

        static inline bool                       running{false};
        static inline typename Clock::time_point whenRdy{};

        template<typename RGB>
        static void send(std::span<RGB> leds) {
            static_assert(sizeof(RGB) == 3, "only rgb");
            assert(ready());

            Dma::template start<
              DmaChannel,
              DmaPriority,
              Kvasir::Pio::getTxDmaTrigger<Dma, Config::PioInstance, Config::SmInstance>(),
              Dma::TransferSize::_8,
              false,
              true>(
              PioRegs::template FIFO<Config::SmInstance>::TXF::Addr::value,
              reinterpret_cast<std::uint32_t>(leds.data()),
              leds.size() * sizeof(RGB));

            apply(
              write(PioRegs::FDEBUG::txstall, Kvasir::Register::value<1 << Config::SmInstance>()));
        }

        static bool ready() {
            if(running) {
                return false;
            }
            return whenRdy > Clock::now() || whenRdy == typename Clock::time_point{};
        }

        static void handler() {
            bool const stall
              = get<0>(apply(read(PioRegs::FDEBUG::txstall))) & (1 << Config::SmInstance);
            if(stall && running) {
                running = false;
                whenRdy = Clock::now() + 60us;
            }
        }
    };
}}   // namespace Kvasir::Pio
