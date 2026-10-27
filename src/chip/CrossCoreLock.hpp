#pragma once
#include "core/Barrier.hpp"

#include <kvasir/Common/Tags.hpp>
#include <kvasir/Port/Hooks.hpp>
#include <kvasir/Register/Register.hpp>
#include <peripherals/SIO.hpp>

// The lock under the atomic shim in a multicore RP2040 build (Kvasir_SDK kvasir/StartUp/Runtime/LibAtomic.hpp), the
// Port::CrossCoreLock hook. The Cortex-M0+ has no exclusive accesses, so every std::atomic read-modify-write goes
// through the shim, and the shim's cross-core lock is one of the SIO's hardware spinlocks: a read claims it (nonzero
// when the claim succeeded, zero when another core holds it), a write releases it (RP2040 datasheet 2.3.1.3 "Hardware
// Spinlocks"; SPINLOCK31 at SIO offset 0x17c). The claim read and the release write are whole-word actions, not fields.
// SPINLOCK31 is reserved for this; the others are the application's. (The RP2350 does not come here: it has
// exclusives, and erratum RP2350-E2 makes its SIO spinlocks unsafe anyway.) Included by chip/StartUp.hpp in a
// KVASIR_MULTICORE build, before the SDK's StartUp.hpp and with it the shim.
template<typename... Ts>
struct Kvasir::Port::CrossCoreLock<Kvasir::Tag::User, Ts...> {
    using Lock = Kvasir::Peripheral::SIO::Registers<0>::SPINLOCK31;

    static void acquire() {
        while(Kvasir::Register::readWhole(Lock::FULLREGISTER) == 0) {}
        Kvasir::Core::dmb();
    }

    static void release() {
        Kvasir::Core::dmb();
        Kvasir::Register::writeWhole(Lock::FULLREGISTER, 1U);
    }
};
