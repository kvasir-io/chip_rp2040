#pragma once

#include "StartUp.hpp"
#include "peripherals/IO_QSPI.hpp"
#include "peripherals/XIP_SSI.hpp"
#include "rom_functions.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

namespace Kvasir {
namespace detail {

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void
    flash_do_cmd(std::span<std::byte const> txBuffer, std::span<std::byte> rxBuffer) {
        using QSPI_CS  = Kvasir::Peripheral::IO_QSPI::Registers<>::GPIO_QSPI_SS_CTRL::OUTOVERValC;
        using SSI_Regs = Kvasir::Peripheral::XIP_SSI::Registers<>;
        Boot2XipDisabler xipDisabler{};

        apply(write(QSPI_CS::low));

        static constexpr std::size_t maxInFlight = 16 - 2;

        std::size_t inFlight{};
        while(!txBuffer.empty() || !rxBuffer.empty()) {
            auto const flags   = apply(read(SSI_Regs::SR::tfnf), read(SSI_Regs::SR::rfne));
            bool const can_put = get<0>(flags);
            bool const can_get = get<1>(flags);
            if(can_put && !txBuffer.empty() && inFlight < maxInFlight) {
                apply(write(SSI_Regs::DR0::dr, static_cast<std::uint32_t>(txBuffer[0])));
                txBuffer = txBuffer.subspan(1);
                ++inFlight;
            }
            if(can_get && !rxBuffer.empty()) {
                auto const v = get<0>(apply(read(SSI_Regs::DR0::dr)));
                rxBuffer[0]  = static_cast<std::byte>(v);
                rxBuffer     = rxBuffer.subspan(1);
                --inFlight;
            }
        }

        apply(write(QSPI_CS::high));
    }

    static auto flash_get_unique_id() {
        static constexpr std::byte   Cmd{0x4b};
        static constexpr std::size_t DummyBytes = 5;
        static constexpr std::size_t DataBytes  = 8;
        static constexpr std::size_t TotalBytes = DummyBytes + DataBytes;

        std::array<std::byte, TotalBytes> txBuffer{};
        std::array<std::byte, TotalBytes> rxBuffer{};
        txBuffer[0] = Cmd;
        {
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            flash_do_cmd(txBuffer, rxBuffer);
        }
        std::array<std::byte, DataBytes> id;
        std::copy(rxBuffer.begin() + DummyBytes, rxBuffer.end(), id.begin());
        return id;
    }

}   // namespace detail
inline std::array<std::byte, 8> serial_number() {
    static std::array<std::byte, 8> const sn = detail::flash_get_unique_id();
    return sn;
}
}   // namespace Kvasir
