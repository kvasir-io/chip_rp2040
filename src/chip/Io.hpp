#pragma once

#include "kvasir/Io/Io.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/IO_BANK0.hpp"
#include "peripherals/PADS_BANK0.hpp"
#include "peripherals/SIO.hpp"

#include <array>
namespace Kvasir { namespace Io {
    template<>
    struct PinLocationTraits<void> {
        static constexpr unsigned baseAddress = Kvasir::Peripheral::IO_BANK0::Registers<>::baseAddr;
        static constexpr int      portBegin   = 0;
        static constexpr int      portEnd     = 1;
        static constexpr int      pinBegin    = 0;
        static constexpr int      pinEnd      = 30;
        static constexpr int      ListEndIndicator = 255;
        static constexpr std::array<std::array<int, pinEnd - pinBegin>, portEnd - portBegin>
          PinsDisabled{
            {
             {{ListEndIndicator}},
             }
        };
    };

    template<int Port, int Pin>
    constexpr bool isValidPinLocation() {
        if(!(Port < PinLocationTraits<void>::portEnd && Port >= PinLocationTraits<void>::portBegin))
        {
            return false;
        }

        if(!(Pin < PinLocationTraits<void>::pinEnd && Pin >= PinLocationTraits<void>::pinBegin)) {
            return false;
        }

        for(auto dp : PinLocationTraits<void>::PinsDisabled[Port]) {
            if(dp == PinLocationTraits<void>::ListEndIndicator) {
                break;
            }
            if(dp == Pin) {
                return false;
            }
        }

        return true;
    }

    using SIO = Kvasir::Peripheral::SIO::Registers<>;

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(write(SIO::GPIO_OUT_CLR::gpio_out_clr, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };
    template<int Port, int Pin>
    struct MakeAction<Action::Set, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(write(SIO::GPIO_OUT_SET::gpio_out_set, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };
    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(write(SIO::GPIO_OUT_XOR::gpio_out_xor, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Read, Register::PinLocation<Port, Pin>>
      : Register::Action<
          Register::RWBitLocT<
            Register::Address<SIO::GPIO_IN::Addr::value, Register::maskFromRange(29, 0)>,
            Pin>,
          Register::ReadAction> {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    namespace detail {
        template<int Pin>
        using Pad = typename Kvasir::Peripheral::PADS_BANK0::Registers<>::GPIO<Pin>::PAD;
        template<int Pin>
        using Ctrl = typename Kvasir::Peripheral::IO_BANK0::Registers<>::GPIO<Pin>::CTRL;

    }   // namespace detail

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
          set(detail::Pad<Pin>::od),
          write(detail::Pad<Pin>::DRIVEValC::_2ma),
          write(
            detail::Pad<Pin>::pde,
            Register::value<PC == PullConfiguration::PullDown ? 1 : 0>()),
          write(detail::Pad<Pin>::pue, Register::value<PC == PullConfiguration::PullUp ? 1 : 0>()),
          clear(detail::Pad<Pin>::slewfast),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          write(detail::Ctrl<Pin>::OEOVERValC::disable),
          write(detail::Ctrl<Pin>::OUTOVERValC::normal),
          write(detail::Ctrl<Pin>::FUNCSELValC::sio),
          write(SIO::GPIO_OE_CLR::gpio_oe_clr, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
          clear(detail::Pad<Pin>::od),
          write(detail::Pad<Pin>::DRIVEValC::_12ma),
          clear(detail::Pad<Pin>::pde),
          clear(detail::Pad<Pin>::pue),
          write(detail::Pad<Pin>::slewfast, Register::value<OS == OutputSpeed::Low ? 0 : 1>()),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          write(detail::Ctrl<Pin>::OEOVERValC::normal),
          write(detail::Ctrl<Pin>::OUTOVERValC::normal),
          write(detail::Ctrl<Pin>::FUNCSELValC::sio),
          write(SIO::GPIO_OE_SET::gpio_oe_set, Register::value<1 << Pin>()),
          write(SIO::GPIO_OUT_CLR::gpio_out_clr, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(OT == Io::OutputType::PushPull, "only push pull supported");
    };

    template<
      Io::OutputType    OT,
      Io::OutputSpeed   OS,
      PullConfiguration PC,
      int               Port,
      int               Pin,
      int               Function>
    struct MakeAction<Action::PinFunction<Function, OT, OS, PC>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
          clear(detail::Pad<Pin>::od),
          write(detail::Pad<Pin>::DRIVEValC::_12ma),
          write(
            detail::Pad<Pin>::pde,
            Register::value<PC == PullConfiguration::PullDown ? 1 : 0>()),
          write(detail::Pad<Pin>::pue, Register::value<PC == PullConfiguration::PullUp ? 1 : 0>()),
          write(detail::Pad<Pin>::slewfast, Register::value<OS == OutputSpeed::Low ? 0 : 1>()),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          write(detail::Ctrl<Pin>::OEOVERValC::normal),
          write(detail::Ctrl<Pin>::OUTOVERValC::normal),
          write(
            detail::Ctrl<Pin>::funcsel,
            Register::value<
              typename detail::Ctrl<Pin>::FUNCSELVal,
              static_cast<typename detail::Ctrl<Pin>::FUNCSELVal>(Function)>()),
          write(SIO::GPIO_OE_SET::gpio_oe_set, Register::value<1 << Pin>()),
          write(SIO::GPIO_OUT_CLR::gpio_out_clr, Register::value<1 << Pin>()))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(OT == Io::OutputType::PushPull, "only push pull supported");
    };

}}   // namespace Kvasir::Io
