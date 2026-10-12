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
// (PM::detail::CoreResetCarried) and PM::reset_cause() reports `core_reset`. Detected from leftover state: clocks still
// on their run-time sources, or the marker in XIP_CTRL.STREAM_ADDR (a Kvasir watchdog reset resets CLOCKS and XIP).
//
// Not covered: debugger resets/downloads (pre-reset commands clear SCRATCH4/3), Kvasir's own reboots (Bootrom::reboot
// clears SCRATCH4), power-on/brown-out/RUN-pin resets (watchdog block reset, 2.8.1.1). Kvasir::Xip::startStream
// overwrites the marker; the clocks still tell.

#include "kvasir/Register/Register.hpp"
#include "peripherals/WATCHDOG.hpp"
#include "peripherals/XIP_CTRL.hpp"
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
    static constexpr std::uint32_t BootMagic = 0xB007'C0D3U;   // datasheet 2.8.1.1
    // Marker in XIP_CTRL.STREAM_ADDR (Table 160): only bits 23:2 are kept, so the marker fits in them
    static constexpr std::uint32_t StreamMarker = 0x00AB'CDE4U;

    // The routine: registers only, no stack, literals in its own pool (.ltorg) - it runs from RAM and must not touch
    // flash; the kvasir_ram_funcs label lets check_ram_funcs.py enforce that at link time. Addresses: XIP_CTRL
    // 0x14000000 (STREAM_ADDR +0x14, FLUSH +0x04; datasheet 2.6.3, Tables 156/157/160), CLOCKS 0x40008000
    // (CLK_REF_SELECTED +0x38: bit 0 = ROSC, CLK_SYS_SELECTED +0x44: bit 0 = clk_ref; 2.15.7), WATCHDOG 0x40058000
    // (SCRATCH3 +0x18; 4.7.6), RESETS 0x4000C000 (RESET_DONE +0x08, bit 2 DMA; 2.14.3), DMA 0x50000000 (CHAN_ABORT
    // +0x444: reads non-zero until the channels have stopped; 2.5.7), PSM 0x40010000 (FRCE_OFF +0x04, bit 16 PROC1;
    // 2.13.5), atomic set / clear aliases +0x2000 / +0x3000 (2.1.2), the ROM's vector table at 0 (word 0 its stack
    // pointer, word 1 its reset handler).
    [[gnu::naked,
      gnu::used,
      gnu::noinline,
      gnu::section(".core_reset_guard.text")
#if !defined(__clang__)
        ,
      gnu::noipa
#endif
    ]] static void entry() {
        asm volatile(
          ".syntax unified\n"
          ".pushsection kvasir_ram_funcs,\"\",%%progbits\n.4byte 10f\n.popsection\n10:\n"
          // r4: the word for SCRATCH3 - 0 unless this was a reset of core 0 alone
          "  movs r4, #0\n"
          "  ldr  r0, =0x14000014\n"
          "  ldr  r1, [r0]\n"
          "  ldr  r2, =%c[marker]\n"
          "  cmp  r1, r2\n"
          "  beq  11f\n"
          "  ldr  r0, =0x40008044\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, #1\n"
          "  bne  11f\n"
          "  ldr  r0, =0x40008038\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, #1\n"
          "  beq  12f\n"
          "11:\n"
          "  ldr  r4, =%c[carried]\n"
          "12:\n"
          "  ldr  r0, =0x40058018\n"
          "  str  r4, [r0]\n"
          // DMA: every channel aborted, if the block is out of reset (an access to a block held in reset
          // is not answered)
          "  ldr  r0, =0x4000C008\n"
          "  ldr  r1, [r0]\n"
          "  movs r2, #4\n"
          "  tst  r1, r2\n"
          "  beq  14f\n"
          "  ldr  r0, =0x50000444\n"
          "  ldr  r1, =0xFFFF\n"
          "  str  r1, [r0]\n"
          "13:\n"
          "  ldr  r1, [r0]\n"
          "  cmp  r1, #0\n"
          "  bne  13b\n"
          "14:\n"
          // core 1: FRCE_OFF.PROC1 set, read back set, released
          "  ldr  r0, =0x40012004\n"
          "  ldr  r1, =0x00010000\n"
          "  str  r1, [r0]\n"
          "  ldr  r2, =0x40010004\n"
          "15:\n"
          "  ldr  r3, [r2]\n"
          "  tst  r3, r1\n"
          "  beq  15b\n"
          "  ldr  r0, =0x40013004\n"
          "  str  r1, [r0]\n"
          // XIP cache flush (reading FLUSH stalls until done, Table 156). Wait for STAT.FLUSH_READY
          // (+0x08, bit 0, Table 157) first: after a watchdog reset XIP runs its own flush (2.6.3.2), and a
          // FLUSH write during it leaves the chip unbootable
          "  ldr  r0, =0x14000008\n"
          "  movs r2, #1\n"
          "16:\n"
          "  ldr  r1, [r0]\n"
          "  tst  r1, r2\n"
          "  beq  16b\n"
          "  ldr  r0, =0x14000004\n"
          "  movs r1, #1\n"
          "  str  r1, [r0]\n"
          "  ldr  r1, [r0]\n"
          // the ROM's reset handler with the ROM's stack pointer
          "  movs r0, #0\n"
          "  ldr  r1, [r0]\n"
          "  msr  msp, r1\n"
          "  ldr  r1, [r0, #4]\n"
          "  bx   r1\n"
          ".ltorg\n" ::[marker] "i"(StreamMarker),
          [carried] "i"(PM::detail::CoreResetCarried));
    }

    // After initMemory (the Startup list's runtimeInit): the routine copied into its section and compared, then armed
    // - SCRATCH4 last, so a reset in between finds no half-written vector (the ROM checks both magic words).
    static void runtimeInit() {
        using WD = Kvasir::Peripheral::WATCHDOG::Registers<>;
        // the cause this boot started with, latched before the next pass of the routine could change the word
        (void)PM::detail::coreResetCarried();
        auto*       to   = &_LINKER_INTERN_core_reset_guard_start_;
        auto* const end  = &_LINKER_INTERN_core_reset_guard_end_;
        auto const* from = &_LINKER_INTERN_core_reset_guard_load_;
        // the linker's symbols are distinct objects to the compiler: hide them from it
        asm("" : "+r"(to), "+r"(from));
        auto* const first  = to;
        auto const* source = from;
        while(to != end) { *reinterpret_cast<std::uint32_t volatile*>(to++) = *from++; }
        for(auto const* p = first; p != end; ++p, ++source) {
            if(*reinterpret_cast<std::uint32_t const volatile*>(p) != *source) {
                return;
            }   // not armed
        }
        apply(write(Kvasir::Peripheral::XIP_CTRL::Registers<>::STREAM_ADDR::FULLREGISTER,
                    Register::value<StreamMarker>()));
        auto const at = reinterpret_cast<std::uint32_t>(&entry) | 1U;   // Thumb
        apply(write(WD::SCRATCH7::FULLREGISTER, at));
        apply(write(WD::SCRATCH6::FULLREGISTER, reinterpret_cast<std::uint32_t>(end)));
        apply(write(WD::SCRATCH5::FULLREGISTER, at ^ (0U - BootMagic)));
        apply(write(WD::SCRATCH4::FULLREGISTER, Register::value<BootMagic>()));
    }
};
}   // namespace Kvasir
