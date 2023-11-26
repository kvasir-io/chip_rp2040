#pragma once

#include "peripherals/WATCHDOG.hpp"

#include <chrono>

namespace Kvasir {

template<typename Config>
struct Watchdog {
    using Regs = Kvasir::Peripheral::WATCHDOG::Registers<>;

    //TODO make better
    static constexpr std::uint32_t ReadloadValue{
      std::chrono::duration_cast<std::chrono::microseconds>(Config::overrunTime).count() * 2};

    static constexpr std::uint32_t CyclesPerTick{Config::clockSpeed / 1'000'000};

    static constexpr auto initStepPeripheryConfig = list(
      Regs::TICK::overrideDefaults(clear(Regs::TICK::enable)),
      Regs::CTRL::overrideDefaults(clear(Regs::CTRL::enable)),
      Kvasir::Register::sequencePoint,
      Regs::TICK::overrideDefaults(
        set(Regs::TICK::enable),
        write(Regs::TICK::cycles, Kvasir::Register::value<CyclesPerTick>())),
      write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>()));

    static constexpr auto initStepPeripheryEnable = list(Regs::CTRL::overrideDefaults(
      set(Regs::CTRL::enable),
      clear(Regs::CTRL::pause_dbg0),
      clear(Regs::CTRL::pause_dbg1),
      clear(Regs::CTRL::pause_jtag),
      write(Regs::CTRL::time, Kvasir::Register::value<ReadloadValue>())));

    static void feed() { apply(write(Regs::LOAD::load, Kvasir::Register::value<ReadloadValue>())); }

    static bool causedReset() {
        auto const reasons = apply(read(Regs::REASON::force), read(Regs::REASON::timer));
        return get<0>(reasons) || get<1>(reasons);
    }
};

}   // namespace Kvasir
