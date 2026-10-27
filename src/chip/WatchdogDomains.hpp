#pragma once
// The RP2040's PSM.WDSEL: the power-on state machine stages a watchdog reset runs through (RP2040 datasheet 2.13.5),
// one list per intent (chip_rp_common BootSequence.hpp, BootromReboot.hpp):
// - onTimeout: what a watchdog that runs out resets, set by FirstInitStep - every stage, the oscillators included:
//   a hang gets the fullest reset the watchdog can give.
// - onReboot: Kvasir's own reboot (Rp::watchdogReboot) and the probe's stall recovery (cmake/chip.cmake,
//   `0x0001fffc`) - everything but XOSC and ROSC, as the pico-sdk's watchdog_reboot(): a deliberate reboot keeps the
//   oscillators running. The RESETS stage is in both (every peripheral back into reset), and so are the PROC stages
//   (core 1 back into the boot ROM's holding pen).
#include "kvasir/Register/Register.hpp"
#include "peripherals/PSM.hpp"

namespace Kvasir::Rp {
struct WatchdogDomains {
    using PsmWdsel = Kvasir::Peripheral::PSM::Registers<>::WDSEL;

    // Every stage but the oscillators, which the two intents set differently (`difference`)
    template<typename... Difference>
    [[gnu::always_inline]] static void selectStages(Difference... difference) {
        apply(set(PsmWdsel::proc1),
              set(PsmWdsel::proc0),
              set(PsmWdsel::sio),
              set(PsmWdsel::vreg_and_chip_reset),
              set(PsmWdsel::xip),
              set(PsmWdsel::sram5),
              set(PsmWdsel::sram4),
              set(PsmWdsel::sram3),
              set(PsmWdsel::sram2),
              set(PsmWdsel::sram1),
              set(PsmWdsel::sram0),
              set(PsmWdsel::rom),
              set(PsmWdsel::busfabric),
              set(PsmWdsel::resets),
              set(PsmWdsel::clocks),
              difference...);
    }

    [[gnu::always_inline]] static void onTimeout() {
        selectStages(set(PsmWdsel::xosc), set(PsmWdsel::rosc));
    }

    [[gnu::always_inline]] static void onReboot() {
        selectStages(clear(PsmWdsel::xosc), clear(PsmWdsel::rosc));
    }

    // Right before Kvasir triggers a watchdog reset itself (rp_common WatchdogReset.hpp): nothing on the RP2040, which
    // has no POWMAN and no separate watchdog tick generator.
    [[gnu::always_inline]] static void beforeReset() {}
};
}   // namespace Kvasir::Rp
