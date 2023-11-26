#pragma once
#include <span>

namespace Kvasir { namespace CRC {

    enum class CRC_Type { crc16, crc32 };
    template<CRC_Type type, typename DMA, typename DMA::Channel channel>
    static inline auto calcCrc(std::span<std::byte const> data) {
        static_assert(type == CRC_Type::crc16);
        apply(write(DMA::Regs::SNIFF_DATA::data, Kvasir::Register::value<0xffff>()));

        apply(
          write(DMA::Regs::SNIFF_CTRL::CALCValC::crc16),
          write(
            DMA::Regs::SNIFF_CTRL::dmach,
            Kvasir::Register::value<static_cast<std::uint32_t>(channel)>()),
          set(DMA::Regs::SNIFF_CTRL::en));

        std::byte         buffer;
        std::atomic<bool> running = true;
        DMA::template start<
          channel,
          DMA::Priority::low,
          DMA::TriggerSource::permanent,
          DMA::TransferSize::_8,
          false,
          true>(
          reinterpret_cast<std::uint32_t>(std::addressof(buffer)),
          reinterpret_cast<std::uint32_t>(data.data()),
          data.size(),
          [&]() { running = false; });

        while(running) {
            asm("nop");
        }

        auto const v   = apply(read(DMA::Regs::SNIFF_DATA::data));
        auto const crc = static_cast<std::uint16_t>(get<0>(v));
        return crc;
    }

}}   // namespace Kvasir::CRC
