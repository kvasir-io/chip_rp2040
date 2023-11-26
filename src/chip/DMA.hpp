#pragma once

#include "Interrupt.hpp"
#include "core/core.hpp"
#include "kvasir/Util/StaticFunction.hpp"
#include "peripherals/DMA.hpp"
#include "peripherals/RESETS.hpp"

#include <optional>

namespace Kvasir { namespace DMA {

    enum class DMAChannel {
        ch0  = 0,
        ch1  = 1,
        ch2  = 2,
        ch3  = 3,
        ch4  = 4,
        ch5  = 5,
        ch6  = 6,
        ch7  = 7,
        ch8  = 8,
        ch9  = 9,
        ch10 = 10,
        ch11 = 11,
    };

    enum class DMAPriority { high = 0, low };

    using TriggerSource = Kvasir::Peripheral::DMA::Registers<>::CH<0>::CTRL_TRIG::TREQ_SELVal;

    enum class DMATransferSize { _8 = 0, _16 = 1, _32 = 2 };

    template<typename DMAConfig_>
    struct DmaBase {
        // needed config
        // numberOfChannels

        struct DMAConfig : DMAConfig_ {
            static constexpr auto callbackFunctionSize = [] {
                if constexpr(requires { DMAConfig_::callbackFunctionSize; }) {
                    return DMAConfig_::callbackFunctionSize;
                } else {
                    return 0;
                }
            }();

            static constexpr auto isrPriority = [] {
                if constexpr(requires { DMAConfig_::isrPriority; }) {
                    return DMAConfig_::isrPriority;
                } else {
                    return 0;
                }
            }();

            static constexpr auto numberOfChannels = DMAConfig_::numberOfChannels;
        };

        static_assert(12 >= DMAConfig::numberOfChannels);

        static constexpr std::size_t numberOfChannels{DMAConfig::numberOfChannels};

        using FunctionArray_t = std::conditional_t<
          DMAConfig::callbackFunctionSize != 0,
          std::array<
            Kvasir::StaticFunction<void(), DMAConfig::callbackFunctionSize>,
            DMAConfig::numberOfChannels>,
          std::array<std::byte, 0>>;

        using Regs = Kvasir::Peripheral::DMA::Registers<>;

        using TriggerSource = ::Kvasir::DMA::TriggerSource;
        using Priority      = ::Kvasir::DMA::DMAPriority;
        using Channel       = ::Kvasir::DMA::DMAChannel;
        using TransferSize  = ::Kvasir::DMA::DMATransferSize;

        using InterruptIndexs
          = brigand::list<decltype(Kvasir::Interrupt::dma_0), decltype(Kvasir::Interrupt::dma_1)>;

        static inline FunctionArray_t callbackFunctions{};

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::dma));

        static constexpr auto initStepPeripheryConfig = list(write(
          Regs::INTE0::inte0,
          Register::value<Register::maskFromRange(DMAConfig::numberOfChannels, 0)>()));

        static constexpr auto initStepInterruptConfig = list(
          Nvic::makeSetPriority<DMAConfig::isrPriority>(InterruptIndexs{}),
          Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(InterruptIndexs{}));

        template<typename T>
        struct Fail {
            static_assert(!std::is_void_v<T>, "DMA callback not configured");
        };

        template<
          DMAChannel      Channel,
          DMAPriority     Priority,
          TriggerSource   Trigger,
          DMATransferSize Size,
          bool            IncDest,
          bool            IncSource,
          typename F>
        static void start(std::uint32_t dest, std::uint32_t source, std::size_t count, F&& f) {
            using CHRegs = Regs::CH<static_cast<int>(Channel)>;

            if constexpr(!std::is_same_v<std::remove_cvref_t<F>, std::nullopt_t>) {
                if constexpr(DMAConfig::callbackFunctionSize > 0) {
                    callbackFunctions[static_cast<std::size_t>(Channel)] = std::forward<F>(f);
                } else {
                    Fail<void>{};
                }
            } else {
                if constexpr(DMAConfig::callbackFunctionSize > 0) {
                    callbackFunctions[static_cast<std::size_t>(Channel)].reset();
                }
            }

            apply(write(CHRegs::READ_ADDR::read_addr, source));
            apply(write(CHRegs::WRITE_ADDR::write_addr, dest));
            apply(write(CHRegs::TRANS_COUNT::trans_count, count));

            using ctrl = typename CHRegs::CTRL_TRIG;

            apply(ctrl::overrideDefaults(
              write(
                ctrl::treq_sel,
                Register::value<
                  typename ctrl::TREQ_SELVal,
                  static_cast<typename ctrl::TREQ_SELVal>(Trigger)>()),
              write(ctrl::chain_to, Register::value<static_cast<int>(Channel)>()),
              set(ctrl::sniff_en),
              write(ctrl::incr_write, Register::value < IncDest ? 1 : 0 > ()),
              write(ctrl::incr_read, Register::value < IncSource ? 1 : 0 > ()),
              write(
                ctrl::data_size,
                Register::value<
                  typename ctrl::DATA_SIZEVal,
                  static_cast<typename ctrl::DATA_SIZEVal>(Size)>()),
              write(
                ctrl::high_priority,
                Register::value < Priority == DMAPriority::high ? 1 : 0 > ()),
              set(ctrl::en)));
        }
        template<
          DMAChannel      Channel,
          DMAPriority     Priority,
          TriggerSource   Trigger,
          DMATransferSize Size,
          bool            IncDest,
          bool            IncSource>
        static void start(std::uint32_t dest, std::uint32_t source, std::size_t count) {
            start<Channel, Priority, Trigger, Size, IncDest, IncSource>(
              dest,
              source,
              count,
              std::nullopt);
        }

        template<DMAChannel Channel>
        static bool ready() {
            using CHRegs = Regs::CH<static_cast<int>(Channel)>;
            return !apply(read(CHRegs::CTRL_TRIG::busy));
        }

        static void onIsr() {
            auto const channels = get<0>(apply(read(Regs::INTS0::ints0)));
            apply(write(Regs::INTS0::ints0, channels));

            if constexpr(DMAConfig::callbackFunctionSize > 0) {
                for(std::size_t i{}; auto& f : callbackFunctions) {
                    if(((channels & (1 << i)) != 0) && f) {
                        f();
                    }
                    ++i;
                }
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }
        using Isr = decltype(makeIsr(InterruptIndexs{}));
    };
}}   // namespace Kvasir::DMA
