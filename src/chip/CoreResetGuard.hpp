#pragma once
// RP2040: a clean boot after a reset of core 0 alone. Armed by every image that starts core 1 (SecondaryCoreInit::
// armCoreResetGuard; NoCoreResetGuard in SecondaryCore's list opts out, RAM_ONLY images go without); a single-core
// image can list Kvasir::CoreResetGuard itself.
//
// AIRCR.SYSRESETREQ resets only the requesting core; core 1, DMA and XIP run on (RP2040 datasheet 2.4.2.9). The boot
// ROM then reads boot2 over the SSI in plain SPI (2.8.1.2) while core 1 still fetches through it: the ROM can stall for
// good, and cache lines filled meanwhile stay wrong (2.6.3.2).
//
// The ROM's watchdog boot vector (2.8.1.1, SCRATCH4..7) runs before it touches the SSI. The routine here (in SRAM5)
// stops DMA and core 1, flushes the XIP cache and re-enters the ROM's reset handler with SCRATCH4 cleared, so the ROM
// boots flash with nothing else on the SSI.
//
// Reset cause: REASON and CHIP_RESET do not change on a core-0 reset, so the routine records it in SCRATCH3
// (PM::CoreResetCarried) and PM::resetCause() reports `core_reset`. Detected from leftover state: clocks still
// on their run-time sources, or the marker in XIP_CTRL.STREAM_ADDR (a Kvasir watchdog reset resets CLOCKS and XIP).
//
// Not covered: debugger resets/downloads (pre-reset commands clear SCRATCH4/3), Kvasir's own reboots (Bootrom::reboot
// clears SCRATCH4), power-on/brown-out/RUN-pin resets (watchdog block reset, 2.8.1.1). Kvasir::Xip::startStream
// overwrites the marker; the clocks still tell.

#include "kvasir/Register/Memory.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/attributes.hpp"
#include "peripherals/CLOCKS.hpp"
#include "peripherals/DMA.hpp"
#include "peripherals/PSM.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/WATCHDOG.hpp"
#include "peripherals/XIP_CTRL.hpp"
#include "rp_common/RegisterAlias.hpp"
#include "rp_common/WatchdogBoot.hpp"
#include "rp_common/chip_reset.hpp"

#include <cstddef>
#include <cstdint>

extern "C" {
// linker/chip.ld: the routine's section in SRAM5 and its load image in flash
extern std::uint32_t _LINKER_INTERN_core_reset_guard_start_;
extern std::uint32_t _LINKER_INTERN_core_reset_guard_end_;
extern std::uint32_t _LINKER_INTERN_core_reset_guard_load_;
}

namespace Kvasir {
struct CoreResetGuard {
    static constexpr std::uint32_t bootMagic = Rp::WatchdogBoot::magic;   // datasheet 2.8.1.1
    // Marker in XIP_CTRL.STREAM_ADDR (Table 160): only bits 23:2 are kept, so the marker fits in them
    static constexpr std::uint32_t streamMarker = 0x00AB'CDE4U;

    using Xip     = Kvasir::Peripheral::XIP_CTRL::Registers<>;
    using Clocks  = Kvasir::Peripheral::CLOCKS::Registers<>;
    using FrceOff = Kvasir::Peripheral::PSM::Registers<>::FRCE_OFF;

    // The routine: registers only, no stack, literals in its own pool (.ltorg) - it runs from RAM and must not touch
    // flash; KVASIR_RAM_FUNC_MARK() lets check_ram_funcs.py enforce that at link time. Every address and bit is an
    // operand from the generated registers: XIP_CTRL STREAM_ADDR, FLUSH, STAT.FLUSH_READY (datasheet 2.6.3, Tables
    // 156/157/160), CLOCKS CLK_REF_SELECTED (one-hot over CLK_REF_CTRL.SRC: bit 0 = ROSC) and CLK_SYS_SELECTED (bit 0
    // = clk_ref; 2.15.7), WATCHDOG SCRATCH3 (4.7.6), RESETS RESET_DONE.DMA (2.14.3), DMA CHAN_ABORT (reads non-zero
    // until the channels have stopped; 2.5.7), PSM FRCE_OFF.PROC1 (2.13.5) through the atomic set / clear aliases
    // (2.1.2, rp_common/RegisterAlias.hpp); the ROM's vector table at 0 (word 0 its stack pointer, word 1 its reset
    // handler).
    [[gnu::naked,
      gnu::used,
      gnu::noinline,
      gnu::section(".core_reset_guard.text")
#ifndef __clang__
        ,
      gnu::noipa
#endif
    ]] static void entry() {
        KVASIR_RAM_FUNC_MARK();
        asm volatile(
          ".syntax unified\n"
          // r4: the word for SCRATCH3 - 0 unless this was a reset of core 0 alone
          "  movs r4, #0\n"
          "  ldr  r0, =%c[streamAddr]\n"
          "  ldr  r1, [r0]\n"
          "  ldr  r2, =%c[marker]\n"
          "  cmp  r1, r2\n"
          "  beq  11f\n"
          "  ldr  r0, =%c[sysSelected]\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, %[sysOnRef]\n"
          "  bne  11f\n"
          "  ldr  r0, =%c[refSelected]\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, %[refOnRosc]\n"
          "  beq  12f\n"
          "11:\n"
          "  ldr  r4, =%c[carried]\n"
          "12:\n"
          "  ldr  r0, =%c[scratch3]\n"
          "  str  r4, [r0]\n"
          // DMA: every channel aborted, if the block is out of reset (an access to a block held in reset
          // is not answered)
          "  ldr  r0, =%c[resetDone]\n"
          "  ldr  r1, [r0]\n"
          "  movs r2, %[dmaDone]\n"
          "  tst  r1, r2\n"
          "  beq  14f\n"
          "  ldr  r0, =%c[chanAbort]\n"
          "  ldr  r1, =%c[allChannels]\n"
          "  str  r1, [r0]\n"
          "13:\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, #0\n"
          "  bne  13b\n"
          "14:\n"
          // core 1: FRCE_OFF.PROC1 set, read back set, released
          "  ldr  r0, =%c[frceOffSet]\n"
          "  ldr  r1, =%c[proc1]\n"
          "  str  r1, [r0]\n"
          "  ldr  r2, =%c[frceOff]\n"
          "15:\n"
          "  ldr  r3, [r2]\n"
          "  tst  r3, r1\n"
          "  beq  15b\n"
          "  ldr  r0, =%c[frceOffClr]\n"
          "  str  r1, [r0]\n"
          // XIP cache flush (reading FLUSH stalls until done, Table 156). Wait for STAT.FLUSH_READY
          // (Table 157) first: after a watchdog reset XIP runs its own flush (2.6.3.2), and a
          // FLUSH write during it leaves the chip unbootable
          "  ldr  r0, =%c[xipStat]\n"
          "  movs r2, %[flushReady]\n"
          "16:\n"
          "  ldr  r1, [r0]\n"
          "  tst  r1, r2\n"
          "  beq  16b\n"
          "  ldr  r0, =%c[xipFlush]\n"
          "  movs r1, %[flush]\n"
          "  str  r1, [r0]\n"
          "  ldr  r1, [r0]\n"
          // the ROM's reset handler with the ROM's stack pointer
          "  movs r0, #0\n"
          "  ldr  r1, [r0]\n"
          "  msr  msp, r1\n"
          "  ldr  r1, [r0, #4]\n"
          "  bx   r1\n"
          ".ltorg\n" ::[marker] "i"(streamMarker),
          [carried] "i"(PM::coreResetCarried),
          [streamAddr] "i"(Xip::STREAM_ADDR::Addr::value),
          [sysSelected] "i"(Clocks::CLK_SYS_SELECTED::Addr::value),
          [sysOnRef] "i"(1U << static_cast<unsigned>(Clocks::CLK_SYS_CTRL::SRCVal::clk_ref)),
          [refSelected] "i"(Clocks::CLK_REF_SELECTED::Addr::value),
          [refOnRosc] "i"(
            1U << static_cast<unsigned>(Clocks::CLK_REF_CTRL::SRCVal::rosc_clksrc_ph)),
          [scratch3] "i"(Kvasir::Peripheral::WATCHDOG::Registers<>::SCRATCH3::Addr::value),
          [resetDone] "i"(Kvasir::Peripheral::RESETS::Registers<>::RESET_DONE::Addr::value),
          [dmaDone] "i"(Kvasir::Peripheral::RESETS::Registers<>::RESET_DONE::dma.mask),
          [chanAbort] "i"(Kvasir::Peripheral::DMA::Registers<>::CHAN_ABORT::Addr::value),
          [allChannels] "i"(Kvasir::Peripheral::DMA::Registers<>::CHAN_ABORT::chan_abort.mask),
          [frceOff] "i"(FrceOff::Addr::value),
          [frceOffSet] "i"(FrceOff::Addr::value + Rp::aliasSet),
          [frceOffClr] "i"(FrceOff::Addr::value + Rp::aliasClr),
          [proc1] "i"(FrceOff::proc1.mask),
          [xipStat] "i"(Xip::STAT::Addr::value),
          [flushReady] "i"(Xip::STAT::flush_ready.mask),
          [xipFlush] "i"(Xip::FLUSH::Addr::value),
          [flush] "i"(Xip::FLUSH::flush.mask));
    }

    // After initMemory (the Startup list's runtimeInit): the routine copied into its section and compared, then armed
    // (Rp::WatchdogBoot::arm). A copy that does not compare leaves the vector unarmed: the image runs unguarded, and
    // armed() says so.
    static void runtimeInit() {
        // the cause this boot started with, latched before the next pass of the routine could change the word
        (void)PM::isCoreResetCarried();
        auto*       to   = &_LINKER_INTERN_core_reset_guard_start_;
        auto* const end  = &_LINKER_INTERN_core_reset_guard_end_;
        auto const* from = &_LINKER_INTERN_core_reset_guard_load_;
        // the linker's symbols are distinct objects to the compiler: hide them from it
        asm("" : "+r"(to), "+r"(from));
        auto* const first     = to;
        auto const* source    = from;
        auto const  addressOf = [](auto const* p) { return reinterpret_cast<std::uintptr_t>(p); };
        while(to != end) { Kvasir::Memory::store<std::uint32_t>(addressOf(to++), *from++); }
        for(auto const* p = first; p != end; ++p, ++source) {
            if(Kvasir::Memory::load<std::uint32_t>(addressOf(p)) != *source) {
                return;
            }   // not armed
        }
        apply(write(Xip::STREAM_ADDR::FULLREGISTER, Register::value<streamMarker>()));
        auto const at = reinterpret_cast<std::uint32_t>(&entry) | 1U;   // Thumb
        Rp::WatchdogBoot::arm(at, reinterpret_cast<std::uint32_t>(end));
    }

    // The watchdog boot vector is set: a reset of core 0 alone runs the routine.
    [[nodiscard]] static bool armed() { return Rp::WatchdogBoot::armed(); }
};
}   // namespace Kvasir
