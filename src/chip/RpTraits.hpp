#pragma once
// The RP2040's data constants that chip_rp_common's drivers read (Kvasir::Rp::ChipTraits); each comment there is at
// the user. Generated register headers only.
#include <cstdint>

namespace Kvasir::Rp {
struct ChipTraits {
    // XIP_NOCACHE_NOALLOC_BASE (RP2040 datasheet 2.2.2, md l.1249): uncached, and with no address translation on this
    // chip also the "untranslated" view (rp_common flash.hpp)
    static constexpr std::uint32_t xipUncachedBase     = 0x1300'0000U;
    static constexpr std::uint32_t xipUntranslatedBase = 0x1300'0000U;
    // the fastest clk_sys Kvasir runs the chip at (rp_common clock_config.hpp vregInit, WaitBounds.hpp)
    static constexpr std::uint64_t cpuHzCeiling = 200'000'000;
    // The ring oscillator before its divider, at most: "During boot the ROSC runs at a nominal 6.5MHz and is guaranteed
    // to be in the range 1.8MHz to 12MHz" (RP2040 datasheet 2.17, md l.10643) at the default divisor 16 (2.17.4, md
    // l.10679): 192 MHz undivided, nothing in Kvasir writes FREQA/FREQB/FREQ_RANGE (rp_common clock_config.hpp
    // roscRefDivisor). Its slowest output: 1.8 MHz at that divisor.
    static constexpr std::uint64_t roscBaseHzCeiling = 192'000'000;
    static constexpr std::uint64_t roscHzFloor       = 1'800'000;
    // The fastest clk_ref on the ring oscillator Kvasir lets it run at (rp_common clock_config.hpp onRingOscillator
    // divides it there; WaitBounds.hpp XoscStableBoundFor counts at it, clk_sys = clk_ref then): the boot's own 12 MHz -
    // the datasheet gives clk_ref no maximum, its clock table "6 - 12MHz" nominal (Table 205, md l.8649).
    static constexpr std::uint64_t clkRefOnRoscHzCeiling = 12'000'000;
    // ROSC.DIV: written 0xaa0 + div, div 1..31 divides by div, 0 by 32, any other value by 31 (Table 269, md l.10863)
    static constexpr std::uint32_t roscDivMax     = 31;
    static constexpr std::uint32_t roscDivOfZero  = 32;
    static constexpr std::uint32_t roscDivOfOther = 31;
    // the XIP streaming FIFO through the auxiliary window (rp_common Xip.hpp); the RP2040's SVD has no XIP_AUX block,
    // its window is at 0x50400000
    static constexpr std::uint32_t xipStreamFifoAux = 0x5040'0000U;
};
}   // namespace Kvasir::Rp
