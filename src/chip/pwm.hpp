#pragma once
#include <cstdint>
#include <optional>
#include <utility>

namespace Kvasir { namespace PWM {
    namespace detail {

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcFrequency(std::uint16_t div16, std::uint16_t top) {
            return ((ClockSpeed * 16) / (div16)) / top;
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcTop(std::uint16_t div16, std::uint32_t frequency) {
            return ((ClockSpeed * 16) / (div16)) / frequency;
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::uint32_t calcDiv16(std::uint16_t top, std::uint32_t frequency) {
            auto const ret = (ClockSpeed * 16) / (top * frequency);
            assert(ret >= 16);
            return ret;
        }

        template<std::uint32_t ClockSpeed>
        static constexpr std::optional<std::pair<std::uint16_t, std::uint16_t>>
        calcDivAndTop(std::uint32_t frequency, std::uint32_t topMin) {
            constexpr std::uint32_t TopMax = 65536 - 1;
            constexpr std::uint32_t DivMax = (256 * 16) - 1;
            constexpr std::uint32_t Div    = 16;

            std::uint32_t div16_top = Div * ClockSpeed / frequency;
            std::uint32_t top       = 1;
            std::uint32_t div_tmp   = DivMax;

            std::uint32_t div_best{};
            std::uint32_t top_best{};
            while(div_tmp > Div) {
                std::uint32_t tmpTop
                  = calcTop<ClockSpeed>(static_cast<std::uint16_t>(div_tmp), frequency);
                if(TopMax >= tmpTop && tmpTop > topMin) {
                    if(
                      tmpTop > top_best
                      && frequency
                           == calcFrequency<ClockSpeed>(
                             static_cast<std::uint16_t>(div_tmp),
                             static_cast<std::uint16_t>(tmpTop)))
                    {
                        top_best = tmpTop;
                        div_best = div_tmp;
                    }
                }
                --div_tmp;
            }
            if(top_best == 0) {
                div16_top = Div * ClockSpeed / frequency;
                top       = 1;
                while(true) {
                    // Try a few small prime factors to get close to the desired frequency.
                    if(div16_top >= Div * 7 && div16_top % 7 == 0 && top * 7 <= TopMax) {
                        div16_top /= 7;
                        top *= 7;
                    } else if(div16_top >= Div * 5 && div16_top % 5 == 0 && top * 5 <= TopMax) {
                        div16_top /= 5;
                        top *= 5;
                    } else if(div16_top >= Div * 3 && div16_top % 3 == 0 && top * 3 <= TopMax) {
                        div16_top /= 3;
                        top *= 3;
                    } else if(div16_top >= Div * 2 && top * 2 <= TopMax) {
                        div16_top /= 2;
                        top *= 2;
                    } else {
                        break;
                    }
                }
            } else {
                div16_top = div_best;
                top       = top_best;
            }

            if(div16_top < Div) {
                UC_LOG_W("freq too large");
                return std::nullopt;
            } else if(div16_top >= DivMax) {
                return std::nullopt;
                UC_LOG_W("freq too small");
            } else if(topMin > top) {
                UC_LOG_W("cant reach min top");
                return std::nullopt;
            }

            return std::make_pair(
              static_cast<std::uint16_t>(div16_top),
              static_cast<std::uint16_t>(top));
        }

        template<int Port, int Pin>
        constexpr std::uint32_t getChannel(Kvasir::Register::PinLocation<Port, Pin>) {
            return static_cast<std::uint32_t>((Pin % 16) / 2);
        }

        template<int Port, int Pin>
        constexpr bool isChannelA(Kvasir::Register::PinLocation<Port, Pin>) {
            return Pin % 2 == 0;
        }

        template<std::uint16_t Duty, std::uint32_t Channel, bool a_or_b>
        constexpr auto setDuty() {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;
            if constexpr(a_or_b) {
                return write(Regs::CC::a, Kvasir::Register::value<Duty>());
            } else {
                return write(Regs::CC::b, Kvasir::Register::value<Duty>());
            }
        }
        template<std::uint16_t Duty, int Port, int Pin>
        constexpr auto setDuty(Kvasir::Register::PinLocation<Port, Pin> x) {
            return setDuty<Duty, getChannel(x), isChannelA(x)>();
        }

        template<bool Invert, int Port, int Pin>
        constexpr auto setInvert(Kvasir::Register::PinLocation<Port, Pin> x) {
            using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<getChannel(x)>;
            if constexpr(isChannelA(x)) {
                return write(Regs::CSR::a_inv, Kvasir::Register::value<Invert>());
            } else {
                return write(Regs::CSR::b_inv, Kvasir::Register::value<Invert>());
            }
        }
    }   // namespace detail

    template<typename Pin, typename Config_>
    struct PWM {
        struct Config : Config_ {
            static constexpr auto invert = [] {
                if constexpr(requires { Config_::invert; }) {
                    return Config_::invert;
                } else {
                    return false;
                }
            }();
            static constexpr auto top = [] {
                if constexpr(requires { Config_::top; }) {
                    return Config_::top;
                } else {
                    return false;
                }
            }();
            static constexpr auto minTop = [] {
                if constexpr(requires { Config_::minTop; }) {
                    return Config_::minTop;
                } else {
                    return false;
                }
            }();
            static constexpr auto initDuty = [] {
                if constexpr(requires { Config_::initDuty; }) {
                    return Config_::initDuty;
                } else {
                    return 0;
                }
            }();
        };

        static constexpr bool hasMinTop
          = std::is_same_v<bool, std::remove_cvref_t<decltype(Config::minTop)>>;
        static constexpr bool hasTop
          = std::is_same_v<bool, std::remove_cvref_t<decltype(Config::top)>>;

        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<detail::getChannel(Pin{})>;

        static constexpr std::uint16_t MinTop = []() {
            if constexpr(hasMinTop) {
                return Config::minTop;
            } else {
                if constexpr(hasTop) {
                    return Config::top;
                } else {
                    return 256;
                }
            }
        }();

        static constexpr std::uint16_t InitialTop = []() {
            if constexpr(hasTop) {
                auto const divTopO
                  = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop);
                assert(divTopO.has_value());
                return divTopO->second;
            } else {
                return Config::top;
            }
        }();

        static constexpr std::uint16_t InitialDiv16 = []() {
            if constexpr(hasTop) {
                auto const divTopO
                  = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop);
                assert(divTopO.has_value());
                return divTopO->first;
            } else {
                return detail::calcDiv16<Config::clockSpeed>(Config::top, Config::frequency);
            }
        }();

        static constexpr std::uint16_t InitialDuty = Config::initDuty;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pwm));

        static constexpr auto initStepPinConfig
          = list(action(Kvasir::Io::Action::PinFunction<4>{}, Pin{}));

        static constexpr auto initStepPeripheryConfig = list(
          write(Regs::DIV::div_16, Kvasir::Register::value<InitialDiv16>()),
          write(Regs::TOP::top, Kvasir::Register::value<InitialTop>()),
          detail::setInvert<Config::invert>(Pin{}),
          detail::setDuty<InitialDuty>(Pin{}));

        static constexpr auto initStepPeripheryEnable = list(set(Regs::CSR::en));

        static void reset() {
            apply(initStepPinConfig, initStepPeripheryConfig, initStepPeripheryEnable);
        }

        static std::uint16_t getTop() {
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::TOP::top))));
        }

        static void setDuty(std::uint16_t duty) {
            if constexpr(detail::isChannelA(Pin{})) {
                apply(write(Regs::CC::a, duty));
            } else {
                apply(write(Regs::CC::b, duty));
            }
        }

        static void setFrequency(std::uint32_t frequency) {
            auto const result = setFrequencyChecked(frequency);
            assert(result);
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            return divTopO.has_value();
        }

        static bool setFrequencyChecked(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            if(!divTopO.has_value()) {
                return false;
            }
            auto const& [div16, top] = *divTopO;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));

            return true;
        }
    };

    template<std::size_t Channel, typename Config, typename Callback>
    struct PWM_Timer {
        using Regs = Kvasir::Peripheral::PWM::Registers<>::CH<Channel>;

        static constexpr auto InterruptIndex = Kvasir::Interrupt::pwm_wrap;

        static constexpr std::uint16_t MinTop = 1;

        static constexpr std::uint16_t InitialTop = []() {
            auto const divTopO
              = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop);
            assert(divTopO.has_value());
            return divTopO->second;
        }();

        static constexpr std::uint16_t InitialDiv16 = []() {
            auto const divTopO
              = detail::calcDivAndTop<Config::clockSpeed>(Config::frequency, MinTop);
            assert(divTopO.has_value());
            return divTopO->first;
        }();

        static constexpr std::uint16_t InitialDuty = InitialTop;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pwm));

        static constexpr auto initStepPeripheryConfig = list(
          write(Regs::DIV::div_16, Kvasir::Register::value<InitialDiv16>()),
          write(Regs::TOP::top, Kvasir::Register::value<InitialTop>()),
          detail::setDuty<InitialDuty, Channel, true>());

        static constexpr auto initStepInterruptConfig = list(
          Nvic::makeSetPriority<Config::isrPriority>(InterruptIndex),
          Nvic::makeClearPending(InterruptIndex));

        static constexpr auto initStepPeripheryEnable = list(
          set(Regs::CSR::en),
          set(Kvasir::Peripheral::PWM::Registers<>::INTE::ch0),
          Nvic::makeEnable(InterruptIndex));

        static void reset() {
            apply(initStepPeripheryConfig, initStepInterruptConfig, initStepPeripheryEnable);
        }

        static void setFrequency(std::uint32_t frequency) {
            auto const result = setFrequencyChecked(frequency);
            assert(result);
        }

        static bool isValidFrequency(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            return divTopO.has_value();
        }

        static bool setFrequencyChecked(std::uint32_t frequency) {
            auto const divTopO = detail::calcDivAndTop<Config::clockSpeed>(frequency, MinTop);
            if(!divTopO.has_value()) {
                return false;
            }
            auto const& [div16, top] = *divTopO;
            apply(write(Regs::DIV::div_16, div16), write(Regs::TOP::top, top));

            return true;
        }

        static void onIsr() {
            auto state = apply(read(Kvasir::Peripheral::PWM::Registers<>::INTS::ch0));
            if(state) {
                Callback{}();
            }
            apply(
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch0),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch1),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch2),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch3),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch4),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch5),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch6),
              set(Kvasir::Peripheral::PWM::Registers<>::INTR::ch7));
        }
        static constexpr Nvic::Isr<std::addressof(onIsr), std::decay_t<decltype(InterruptIndex)>>
          isr{};
    };

}}   // namespace Kvasir::PWM
