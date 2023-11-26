#pragma once

#include "DMA.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"

#include <array>
#include <cassert>

namespace Kvasir { namespace UART {

    enum class DataBits {
        _5,
        _6,
        _7,
        _8,
    };

    enum class Parity {
        none,
        odd,
        even,
    };

    enum class StopBits {
        _1,
        _2,
    };

    namespace Detail {

        enum class PinType { Rx0, Tx0, Rts0, Cts0, Rx1, Tx1, Rts1, Cts1 };

        static constexpr std::array<PinType, 30> UartPins{
          PinType::Tx0,  PinType::Rx0,  PinType::Cts0, PinType::Rts0, PinType::Tx1,  PinType::Rx1,
          PinType::Cts1, PinType::Rts1, PinType::Tx1,  PinType::Rx1,  PinType::Cts1, PinType::Rts1,
          PinType::Tx0,  PinType::Rx0,  PinType::Cts0, PinType::Rts0, PinType::Tx0,  PinType::Rx0,
          PinType::Cts0, PinType::Rts0, PinType::Tx1,  PinType::Rx1,  PinType::Cts1, PinType::Rts1,
          PinType::Tx1,  PinType::Rx1,  PinType::Cts1, PinType::Rts1, PinType::Tx0,  PinType::Rx0};

        static constexpr double
        calcf_Baud(std::uint32_t f_clockSpeed, std::uint32_t divint, std::uint32_t divfrac) {
            return (double(f_clockSpeed) / (16.0 * (double(divint) + double(divfrac) / 64.0)));
        }

        static constexpr std::pair<std::uint16_t, std::uint8_t>
        calcBaudRegs(std::uint32_t f_clockSpeed, std::uint32_t f_baud) {
            std::pair<std::uint16_t, std::uint8_t> ret{};
            double                                 best = std::numeric_limits<double>::max();

            std::uint32_t divint = f_clockSpeed / (16 * f_baud);
            ret.first            = static_cast<std::uint16_t>(divint);

            for(std::uint32_t divfrac = 0; divfrac < 65; ++divfrac) {
                double f_div     = f_baud - calcf_Baud(f_clockSpeed, divint, divfrac);
                double abs_f_div = f_div > 0.0 ? f_div : -f_div;
                if(best > abs_f_div) {
                    best       = abs_f_div;
                    ret.second = static_cast<std::uint8_t>(divfrac);
                    if(abs_f_div == 0.0) {
                        return ret;
                    }
                }
            }
            return ret;
        }

        template<
          std::uint32_t f_clockSpeed,
          std::uint32_t f_baud,
          std::intmax_t Num,
          std::intmax_t Denom>
        static constexpr bool isValidBaudConfig(std::ratio<Num, Denom>) {
            constexpr auto baudRegs     = calcBaudRegs(f_clockSpeed, f_baud);
            constexpr auto divint       = std::get<0>(baudRegs);
            constexpr auto divfrac      = std::get<1>(baudRegs);
            constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, divint, divfrac);
            constexpr auto err          = f_baudCalced - double(f_baud);
            constexpr auto absErr       = err > 0.0 ? err : -err;
            constexpr auto ret = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
            return ret;
        }

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::UART::Registers<Instance>;

            static constexpr bool isValidPinLocationCTS(Io::NotUsed<>) { return true; }
            template<int Port, int Pin>
            static constexpr bool isValidPinLocationCTS(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? UartPins[Pin] == PinType::Cts0
                                     : UartPins[Pin] == PinType::Cts1;
            }

            static constexpr bool isValidPinLocationRTS(Io::NotUsed<>) { return true; }
            template<int Port, int Pin>
            static constexpr bool isValidPinLocationRTS(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? UartPins[Pin] == PinType::Rts0
                                     : UartPins[Pin] == PinType::Rts1;
            }

            static constexpr bool isValidPinLocationRX(Io::NotUsed<>) { return true; }
            template<int Port, int Pin>
            static constexpr bool isValidPinLocationRX(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? UartPins[Pin] == PinType::Rx0
                                     : UartPins[Pin] == PinType::Rx1;
            }

            static constexpr bool isValidPinLocationTX(Io::NotUsed<>) { return true; }
            template<int Port, int Pin>
            static constexpr bool isValidPinLocationTX(Kvasir::Register::PinLocation<Port, Pin>) {
                return Instance == 0 ? UartPins[Pin] == PinType::Tx0
                                     : UartPins[Pin] == PinType::Tx1;
            }

            template<typename CTSPIN>
            struct GetCtsPinConfig;

            template<typename dummy>
            struct GetCtsPinConfig<Io::NotUsed<dummy>> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetCtsPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<2>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename RTSPIN>
            struct GetRtsPinConfig;

            template<typename dummy>
            struct GetRtsPinConfig<Io::NotUsed<dummy>> {
                using pinConfig = brigand::list<>;
            };

            template<int Port, int Pin>
            struct GetRtsPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<2>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<typename RXPIN>
            struct GetRxPinConfig;

            template<typename dummy>
            struct GetRxPinConfig<Io::NotUsed<dummy>> {
                using enable    = decltype(clear(Regs::UARTCR::rxe));
                using pinConfig = brigand::list<>;
                using interrupt = brigand::list<>;
                template<typename InterruptIndex>
                using interruptEnable = brigand::list<>;
            };
            template<int Port, int Pin>
            struct GetRxPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = decltype(set(Regs::UARTCR::rxe));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<2>{},
                  Register::PinLocation<Port, Pin>{}));
                using interrupt = brigand::
                  list<decltype(set(Regs::UARTIMSC::rxim)), decltype(set(Regs::UARTIMSC::oeim))>;
                template<typename InterruptIndex>
                using interruptEnable = decltype(Kvasir::Nvic::makeEnable(InterruptIndex{}));
            };

            template<typename TXPIN>
            struct GetTxPinConfig;

            template<typename dummy>
            struct GetTxPinConfig<Io::NotUsed<dummy>> {
                using enable    = decltype(set(Regs::UARTCR::txe));
                using pinConfig = brigand::list<>;
            };
            template<int Port, int Pin>
            struct GetTxPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using enable    = decltype(set(Regs::UARTCR::txe));
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunction<2>{},
                  Register::PinLocation<Port, Pin>{}));
            };

            template<DataBits dbits>
            struct GetDataBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(dbits == DataBits::_5) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<0>());
                    } else if constexpr(dbits == DataBits::_6) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<1>());
                    } else if constexpr(dbits == DataBits::_7) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<2>());
                    } else if constexpr(dbits == DataBits::_8) {
                        return write(Regs::UARTLCR_H::wlen, Register::value<3>());
                    }
                }();
                using config = decltype(config_);
            };

            template<StopBits sbits>
            struct GetStopBitConfig {
                static constexpr auto config_ = []() {
                    if constexpr(sbits == StopBits::_1) {
                        return clear(Regs::UARTLCR_H::stp2);
                    } else if constexpr(sbits == StopBits::_2) {
                        return set(Regs::UARTLCR_H::stp2);
                    }
                }();
                using config = decltype(config_);
            };

            template<Parity pa>
            struct GetParityConfig {
                static constexpr auto config_ = []() {
                    if constexpr(pa == Parity::none) {
                        return brigand::list<decltype(clear(Regs::UARTLCR_H::pen))>{};
                    } else if constexpr(pa == Parity::odd) {
                        return brigand::list<
                          decltype(set(Regs::UARTLCR_H::pen)),
                          decltype(clear(Regs::UARTLCR_H::eps))>{};
                    } else if constexpr(pa == Parity::even) {
                        return brigand::list<
                          decltype(set(Regs::UARTLCR_H::pen)),
                          decltype(set(Regs::UARTLCR_H::eps))>{};
                    }
                }();
                using config = decltype(config_);
            };

            template<std::uint32_t f_clockSpeed, std::uint32_t f_baud>
            struct GetBaudConfig {
                static constexpr auto config_ = []() {
                    constexpr auto baudRegs = calcBaudRegs(f_clockSpeed, f_baud);
                    return list(
                      write(Regs::UARTIBRD::baud_divint, Register::value<std::get<0>(baudRegs)>()),
                      write(
                        Regs::UARTFBRD::baud_divfrac,
                        Register::value<std::get<1>(baudRegs)>()));
                }();
                using config = decltype(config_);
            };
        };

    }   // namespace Detail

    namespace Traits { namespace UART {
            template<unsigned Instance>
            static constexpr auto getIsrIndexs() {
                if constexpr(Instance == 0) {
                    return brigand::list<decltype(Kvasir::Interrupt::uart0)>{};

                } else if(Instance == 1) {
                    return brigand::list<decltype(Kvasir::Interrupt::uart1)>{};
                }
            }

            template<unsigned Instance>
            static constexpr auto getEnable() {
                if constexpr(Instance == 0) {
                    return clear(Peripheral::RESETS::Registers<>::RESET::uart0);
                } else if(Instance == 1) {
                    return clear(Peripheral::RESETS::Registers<>::RESET::uart1);
                }
            }

            template<unsigned Instance>
            static constexpr auto DmaRX_Trigger() {
                if constexpr(Instance == 0) {
                    return DMA::TriggerSource::uart0_rx;

                } else if(Instance == 1) {
                    return DMA::TriggerSource::uart1_rx;
                }
            }
            template<unsigned Instance>
            static constexpr auto DmaTX_Trigger() {
                if constexpr(Instance == 0) {
                    return DMA::TriggerSource::uart0_tx;

                } else if(Instance == 1) {
                    return DMA::TriggerSource::uart1_tx;
                }
            }

    }}   // namespace Traits::UART

    template<typename UartConfig_>
    struct UartBase {
        struct UartConfig : UartConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { UartConfig_::userConfigOverride; }) {
                    return UartConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto maxBaudRateError = [] {
                if constexpr(requires { UartConfig_::maxBaudRateError; }) {
                    return UartConfig_::maxBaudRateError;
                } else {
                    return std::ratio<5, 1000>{};
                }
            }();
        };

        // needed config
        // clockSpeed
        // baudRate
        // usartInstance
        // txPinLocation
        // rxPinLocation
        // userConfigOverride
        // dataBits
        // parity
        // stopBits
        static constexpr auto Instance = UartConfig::instance;
        using Regs                     = Kvasir::Peripheral::UART::Registers<Instance>;

        using InterruptIndexs = decltype(Traits::UART::getIsrIndexs<Instance>());

        using Config = Detail::Config<Instance>;

        static constexpr auto RxDmaTrigger = Traits::UART::DmaRX_Trigger<Instance>();
        static constexpr auto TxDmaTrigger = Traits::UART::DmaTX_Trigger<Instance>();

        static_assert(
          Detail::isValidBaudConfig<UartConfig::clockSpeed, UartConfig::baudRate>(
            UartConfig::maxBaudRateError),
          "invalid baud configuration baudRate error to big");
        static_assert(Config::isValidPinLocationTX(UartConfig::txPinLocation), "invalid TXPin");
        static_assert(Config::isValidPinLocationRX(UartConfig::rxPinLocation), "invalid RXPin");

        static constexpr auto powerClockEnable = list(Traits::UART::getEnable<Instance>());

        static constexpr auto initStepPinConfig = list(
          typename Config::template GetTxPinConfig<
            std::decay_t<decltype(UartConfig::txPinLocation)>>::pinConfig{},
          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UartConfig::rxPinLocation)>>::pinConfig{});

        static constexpr auto initStepPeripheryConfig = list(
          clear(Regs::UARTCR::uarten),
          Kvasir::Register::SequencePoint{},

          typename Config::template GetBaudConfig<UartConfig::clockSpeed, UartConfig::baudRate>::
            config{},
          Kvasir::Register::SequencePoint{},

          typename Config::template GetTxPinConfig<
            std::decay_t<decltype(UartConfig::txPinLocation)>>::enable{},
          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UartConfig::rxPinLocation)>>::enable{},
          typename Config::template GetDataBitConfig<UartConfig::dataBits>::config{},
          typename Config::template GetStopBitConfig<UartConfig::stopBits>::config{},
          typename Config::template GetParityConfig<UartConfig::parity>::config{},

          // The following parameters are only supported via userConfigOverride
          set(Regs::UARTDMACR::txdmae),
          //set(Regs::UARTLCR_H::fen),

          typename Config::template GetRxPinConfig<
            std::decay_t<decltype(UartConfig::rxPinLocation)>>::interrupt{},
          UartConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig = list(
          Nvic::makeSetPriority<UartConfig::isrPriority>(InterruptIndexs{}),
          Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(
          set(Regs::UARTCR::uarten),
          typename Config::template GetRxPinConfig<std::decay_t<
            decltype(UartConfig::rxPinLocation)>>::template interruptEnable<InterruptIndexs>{});
    };

    template<
      typename UartConfig,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      std::size_t            BufferSize>
    struct UartBehaviorImpl : Kvasir::UART::UartBase<UartConfig> {
        using base                           = Kvasir::UART::UartBase<UartConfig>;
        using Regs                           = typename base::Regs;
        using DmaRegs                        = typename Dma::Regs;
        static constexpr std::size_t DmaChId = std::size_t(DmaChannel);
        static constexpr std::size_t DmaLvl  = std::size_t(DmaPriority);

        static_assert(Dma::numberOfChannels > DmaChId);

        inline static Kvasir::Atomic::
          Queue<std::optional<std::byte>, BufferSize, Kvasir::Atomic::OverFlowPolicyIgnore>
            rxbuffer_{};

        enum class OperationState { succeeded, failed, ongoing };

        inline static bool busy = false;
        // inline static std::atomic<OperationState> operationState_ = OperationState::succeeded;
        static OperationState operationState() {
            if(!busy) {
                return OperationState::succeeded;
            }
            /*            if(!Dma::template wd<DmaChannel>().isValid() && apply(read(Regs::INTFLAG::txc))) {
                b = false;
                return OperationState::succeeded;
            }*/
            // TODO timeout
            return OperationState::ongoing;
        }

        static void send_nocopy(std::span<std::byte const> span) {
            assert(!busy);
            busy = true;
            Dma::template start<
              DmaChannel,
              DmaPriority,
              base::TxDmaTrigger,
              Dma::TransferSize::_8,
              false,
              true>(
              Regs::UARTDR::Addr::value,
              reinterpret_cast<std::uint32_t>(span.data()),
              span.size(),
              []() { busy = false; });
        }
    };

    template<
      typename UartConfig,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      std::size_t            BufferSize,
      bool                   isr>
    struct UartBehaviorSelector;

    template<
      typename UartConfig,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      std::size_t            BufferSize>
    struct UartBehaviorSelector<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize, false>
      : UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, 0> {};

    template<
      typename UartConfig,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      std::size_t            BufferSize>
    struct UartBehaviorSelector<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize, true>
      : UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize> {
        using base = UartBehaviorImpl<UartConfig, Dma, DmaChannel, DmaPriority, BufferSize>;
        using Regs = typename base::Regs;

        static void onIsr() {
            auto const intflag = apply(read(Regs::UARTMIS::rxmis, Regs::UARTMIS::oemis));
            if(intflag.template get<1>()) {
                UC_LOG_E("isr error");
                apply(set(Regs::UARTICR::oeic));
                base::rxbuffer_.push(std::nullopt);
            } else if(intflag.template get<0>()) {
                std::byte data = std::byte(apply(read(Regs::UARTDR::data)).template get<0>());
                base::rxbuffer_.push(data);
            } else {
                assert(false);
            }
        }
        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }
        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };

    template<
      typename UartConfig,
      typename Dma,
      typename Dma::Channel  DmaChannel,
      typename Dma::Priority DmaPriority,
      std::size_t            BufferSize>
    struct UartBehavior
      : UartBehaviorSelector<
          UartConfig,
          Dma,
          DmaChannel,
          DmaPriority,
          BufferSize,
          !std::is_same_v<
            std::remove_cvref_t<decltype(UartConfig::rxPinLocation)>,
            std::remove_cvref_t<decltype(Io::NotUsed<>{})>>> {};

}}   // namespace Kvasir::UART
