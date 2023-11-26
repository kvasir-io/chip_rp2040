#pragma once
#include "kvasir/Common/Interrupt.hpp"

#include <array>

namespace Kvasir {
namespace Interrupt {
    template<int I>
    using Type = ::Kvasir::Nvic::Index<I>;

    static constexpr Type<-14> nonMaskableInt{};
    static constexpr Type<-13> hardFault{};
    static constexpr Type<-5>  sVCall{};
    static constexpr Type<-2>  pendSV{};
    static constexpr Type<-1>  systick{};
    static constexpr Type<0>   timer_0{};
    static constexpr Type<1>   timer_1{};
    static constexpr Type<2>   timer_2{};
    static constexpr Type<3>   timer_3{};
    static constexpr Type<4>   pwm_wrap{};
    static constexpr Type<5>   usbctrl{};
    static constexpr Type<6>   xip{};
    static constexpr Type<7>   pio0_0{};
    static constexpr Type<8>   pio0_1{};
    static constexpr Type<9>   pio1_0{};
    static constexpr Type<10>  pio1_1{};
    static constexpr Type<11>  dma_0{};
    static constexpr Type<12>  dma_1{};
    static constexpr Type<13>  io_bank0{};
    static constexpr Type<14>  io_qspi{};
    static constexpr Type<15>  sio_proc0{};
    static constexpr Type<16>  sio_proc1{};
    static constexpr Type<17>  clocks{};
    static constexpr Type<18>  spi0{};
    static constexpr Type<19>  spi1{};
    static constexpr Type<20>  uart0{};
    static constexpr Type<21>  uart1{};
    static constexpr Type<22>  adc_fifo{};
    static constexpr Type<23>  i2c0{};
    static constexpr Type<24>  i2c1{};
    static constexpr Type<25>  rtc{};
}   // namespace Interrupt

namespace Nvic {
    using namespace Kvasir::Interrupt;
    template<>
    struct InterruptOffsetTraits<void> {
        static constexpr int        begin    = -14;
        static constexpr int        end      = 26;
        static constexpr std::array disabled = {-13, -12, -11, -10, -9, -8, -7, -6, -4, -3};
        static constexpr std::array noEnable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noDisable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noSetPending = {sVCall.index(), hardFault.index()};
        static constexpr std::array noClearPending
          = {nonMaskableInt.index(), sVCall.index(), hardFault.index()};
        static constexpr std::array noSetPriority = {nonMaskableInt.index(), hardFault.index()};

        using FaultInterruptIndexs           = brigand::list<decltype(hardFault)>;
        using FaultInterruptIndexsNeedEnable = brigand::list<>;
    };

}   // namespace Nvic
}   // namespace Kvasir
