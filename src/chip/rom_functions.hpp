#pragma once
#include <cassert>
#include <cstdint>
#include <functional>

namespace Kvasir {
namespace RomFunctions {
    constexpr std::uint32_t lookupCode(char c1, char c2) {
        return static_cast<std::uint32_t>(c1) | (static_cast<std::uint32_t>(c2) << 8);
    }

    template<typename T, std::uint32_t Address>
    T dereferenceAs() {
        return reinterpret_cast<T>(*reinterpret_cast<std::uint16_t const*>(Address));
    }

    template<char C1, char C2, typename F>
    F getRomFunctionPointer() {
        static constexpr std::uint32_t LookupFunctionAddress{0x18};
        static constexpr std::uint32_t FunctionTableAddress{0x14};
        static constexpr auto          FunctionLookupCode = lookupCode(C1, C2);

        using RomTabelLookupFunction = F (*)(std::uint16_t const* table, std::uint32_t code);

        auto const romTableLookupFunction
          = dereferenceAs<RomTabelLookupFunction, LookupFunctionAddress>();

        auto const romFunctionLookupTable
          = dereferenceAs<std::uint16_t const*, FunctionTableAddress>();

        auto const fp = romTableLookupFunction(romFunctionLookupTable, FunctionLookupCode);
        assert(fp != nullptr);

        return fp;
    }

    template<char C1, char C2, typename F, typename... Args>
    std::invoke_result_t<F, Args...> call(Args... args) {
        return std::invoke(getRomFunctionPointer<C1, C2, F>(), args...);
    }
}   // namespace RomFunctions
inline void resetToUsbBoot() {
    using romResetToUsbBoot
      = void (*)(std::uint32_t gpioActivityPinMask, std::uint32_t disableInterfaceMask);

    RomFunctions::call<'U', 'B', romResetToUsbBoot>(0, 0);
    apply(Kvasir::SystemControl::SystemReset{});
}
struct Boot2XipDisabler {
    using RomConnectInternalFlash = void (*)(void);
    using RomFlashExitXip         = void (*)(void);
    using RomFlashFlushCache      = void (*)(void);
    using Boot2Function           = void (*)(void);

    RomFlashFlushCache const            flushCache;
    std::array<std::uint32_t, 64> const secondStageBootloaderRamCopy;
    Boot2Function const                 callBoot2;

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] Boot2XipDisabler()
      : flushCache{Kvasir::RomFunctions::getRomFunctionPointer<'F', 'C', RomFlashFlushCache>()}
      , secondStageBootloaderRamCopy{Kvasir::Startup::second_stage_bootloader}
      , callBoot2{reinterpret_cast<Boot2Function>(const_cast<std::byte*>(
          std::as_bytes(std::span{secondStageBootloaderRamCopy}).data() + 1))} {
        Kvasir::RomFunctions::call<'I', 'F', RomConnectInternalFlash>();
        Kvasir::RomFunctions::call<'E', 'X', RomFlashExitXip>();
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] ~Boot2XipDisabler() {
        flushCache();
        callBoot2();
    }
};

}   // namespace Kvasir
