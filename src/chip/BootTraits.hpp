#pragma once
// The RP2040's steps of the RP boot sequence (rp_common/BootSequence.hpp firstInit, from FirstInitStep in
// chip/StartUp.hpp): registers only, before .data/.bss, every step always_inline.
#include "Blocks.hpp"
#include "WatchdogDomains.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Wait.hpp"
#include "peripherals/PSM.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/SYSCFG.hpp"
#include "peripherals/WATCHDOG.hpp"
#include "peripherals/XIP_CTRL.hpp"
#include "rp_common/BootSequence.hpp"
#include "rp_common/Rom.hpp"

namespace Kvasir { namespace Startup { namespace detail {
    // _flash_flush_cache ('F','C', RP2040 datasheet 2.8.3.1.3): "Flush and enable the XIP cache" - from the boot ROM,
    // so no XIP fetch is in flight while the tags are cleared (afterCore1Parked below). Rom.hpp is the ROM table alone,
    // so this header compiles by itself.
    inline void flushXipCacheInRom() { Kvasir::RomFunctions::call<'F', 'C', void (*)()>(); }
}}}   // namespace Kvasir::Startup::detail

namespace Kvasir::Rp {
struct BootTraits {
    [[gnu::always_inline]] static void afterCore1Parked() {
        // Flush the XIP cache: after a reset of core 0 alone, core 1 may have filled lines while the boot ROM
        // reconfigured the SSI (2.4.2.9, 2.8.1); only an XIP reset or a FLUSH write clears them (2.6.3.2).
        // The flush runs in the boot ROM (_flash_flush_cache, 'F','C', 2.8.3.1.3), never from code fetched
        // through XIP: a FLUSH written from flash can leave one cache set holding the other way's data (not in
        // the datasheet; measured). First any flush in progress (an XIP reset's) to its end: STAT.FLUSH_READY
        // (Table 157); a flush takes just over 1024 cycles (2.6.3.2), a poll at least 4.
        Kvasir::Register::waitUntil<Kvasir::Register::Bound::Polls<1'000>>(
          Kvasir::Register::isSet(Kvasir::Peripheral::XIP_CTRL::Registers<>::STAT::flush_ready));
        Startup::detail::flushXipCacheInRom();

        // Watchdog off until the image arms it: only rst_n_run resets it (RP2040 datasheet 4.7.1), so a previous
        // image's watchdog reboot leaves CTRL.ENABLE set.
        apply(clear(Kvasir::Peripheral::WATCHDOG::Registers<>::CTRL::enable));
    }

    // Every block of chip/Blocks.hpp into reset, and JTAG (no row: no driver); SYSINFO, SYSCFG, PLL_SYS and the QSPI
    // pads and IO keep running (chip/Blocks.hpp asserts the table and these cover every RESETS bit).
    [[gnu::always_inline]] static void holdBlocksInReset() {
        using Reset = Kvasir::Peripheral::RESETS::Registers<>::RESET;
        Rp::holdBlocksInReset<Kvasir::Chip::Blocks::Table>(set(Reset::jtag),
                                                           clear(Reset::sysinfo),
                                                           clear(Reset::syscfg),
                                                           clear(Reset::pll_sys),
                                                           clear(Reset::pads_qspi),
                                                           clear(Reset::io_qspi));
    }

    [[gnu::always_inline]] static void afterResetsHeld() {
        // Clear PROC0_NMI_MASK (SYSCFG + 0x00, RP2040 datasheet 2.21.2 Table 354): it is Syscfg state, which only a
        // reset of that block clears - a watchdog event leaves the mask set unless "the RESETS block (and hence
        // Syscfg)" is in the watchdog reset list (2.3.2 CAUTION, md l.2962); "the NMI mask could be left set
        // following a processor reset" on the RP2040 (RP2350 datasheet 3.2.1, md l.3797, whose own mask is
        // core-local and reset with the core) - and syscfg stays out of reset above.
        using ResetDone = Kvasir::Peripheral::RESETS::Registers<>::RESET_DONE;
        Kvasir::Register::waitUntil<Kvasir::Chip::ResetDoneBound>(
          Kvasir::Register::isSet(ResetDone::syscfg));
        using Syscfg = Kvasir::Peripheral::SYSCFG::Registers<>;
        apply(write(Syscfg::PROC0_NMI_MASK::FULLREGISTER, Register::value<0U>()));
    }

    [[gnu::always_inline]] static void selectWatchdogDomains() {
        WatchdogDomains::onTimeout();
        // every block (RESETS.WDSEL has RESETS.RESET's bits: RP2040 datasheet Tables 202/203, md l.8341/8393)
        Rp::setEveryField<Kvasir::Peripheral::RESETS::Registers<>::WDSEL>();
    }
};
}   // namespace Kvasir::Rp
