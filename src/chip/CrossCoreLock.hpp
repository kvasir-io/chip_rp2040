#pragma once
#include <cstdint>
#include <peripherals/SIO.hpp>

// The lock under the atomic shim in a multicore RP2040 build (kvasir/Atomic/detail/
// arm_Common_atomic.hpp). The Cortex-M0+ has no exclusive accesses, so every std::atomic
// read-modify-write goes through the shim, and the shim's cross-core lock is one of the
// SIO's hardware spinlocks: a read claims it (nonzero when the claim succeeded, zero when
// another core holds it), a write releases it. SPINLOCK31 is reserved for this; the others
// are the application's. (The RP2350 does not come here: it has exclusives, and erratum
// RP2350-E2 makes its SIO spinlocks unsafe anyway.)
namespace Kvasir::Atomic {

struct CrossCoreLock {
    static constexpr std::uint32_t Address
      = Kvasir::Peripheral::SIO::Registers<0>::SPINLOCK31::Addr::value;

    static void acquire() {
        auto volatile& lock = *reinterpret_cast<std::uint32_t volatile*>(Address);
        while(lock == 0) {}
        asm volatile("dmb" ::: "memory");
    }

    static void release() {
        asm volatile("dmb" ::: "memory");
        *reinterpret_cast<std::uint32_t volatile*>(Address) = 1;
    }
};

}   // namespace Kvasir::Atomic
