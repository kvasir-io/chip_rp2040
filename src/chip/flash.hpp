#include <array>
#include <cassert>
#include <cstdint>

namespace Kvasir {
namespace Flash {
    namespace detail {
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
        eraseAndWrite(std::uint32_t offset, std::uint8_t const* data, std::size_t size) {
            auto erase = RomFunctions::getRomFunctionPointer<
              'R',
              'E',
              void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();
            auto write = RomFunctions::getRomFunctionPointer<
              'R',
              'P',
              void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

            Boot2XipDisabler xipDisabler{};
            erase(offset, 4096, 1 << 16, 0xD8);
            write(offset, data, size);
        }
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
        erase(std::uint32_t offset, std::size_t blocks) {
            auto erase = RomFunctions::getRomFunctionPointer<
              'R',
              'E',
              void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();

            Boot2XipDisabler xipDisabler{};
            erase(offset, blocks * 4096, 1 << 16, 0xD8);
        }
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
        write(std::uint32_t offset, std::uint8_t const* data, std::size_t size) {
            auto write = RomFunctions::getRomFunctionPointer<
              'R',
              'P',
              void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

            Boot2XipDisabler xipDisabler{};
            write(offset, data, size);
        }

    }   // namespace detail
    static inline void eraseAndWrite(std::uint32_t addr, std::span<std::byte const> data) {
        constexpr std::size_t flashBlockSize{4096};
        assert(addr % flashBlockSize == 0);
        //Löschen von n 4096 byte Speicherbereichen
        std::uint32_t offset = addr - 0x10000000;
        {
            std::size_t eraseBlocks
              = data.size() / flashBlockSize + (data.size() % flashBlockSize == 0 ? 0 : 1);
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            detail::erase(offset, eraseBlocks);
        }
        constexpr std::size_t writeBlockSize{256};
        while(!data.empty()) {
            std::array<std::byte, writeBlockSize> buffer{};
            std::copy_n(data.begin(), std::min(data.size(), writeBlockSize), buffer.begin());
            {
                Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
                detail::write(
                  offset,
                  reinterpret_cast<std::uint8_t const*>(buffer.data()),
                  buffer.size());
            }
            offset += writeBlockSize;
            data = data.subspan(std::min(data.size(), writeBlockSize));
        }
    }
}   // namespace Flash
template<typename Clock, typename T, typename Crc>
struct SimpleEeprom {
    static auto calcCrc(T const& v) {
        return Crc::calc(std::as_bytes(std::span{std::addressof(v), 1}));
    }

    struct alignas(std::uint32_t) ValueStruct {
        T                  v{};
        typename Crc::type crc{};
    };

    [[gnu::section(".eeprom")]] static inline ValueStruct flashValue{};

    static inline T    ramCopy{};
    static inline bool valueRead{false};

    static T readFlashValue() {
        if(flashValue.crc != calcCrc(flashValue.v)) {
            return T{};
        }
        return flashValue.v;
    }

    static T& value() {
        if(!valueRead) {
            ramCopy   = readFlashValue();
            valueRead = true;
        }
        return ramCopy;
    }

    static void internalWrite() {
        ValueStruct newV;
        newV.v   = ramCopy;
        newV.crc = calcCrc(ramCopy);

        asm("" : "=m"(newV)::);

        Kvasir::Flash::eraseAndWrite(
          reinterpret_cast<std::uint32_t>(std::addressof(flashValue)),
          std::as_bytes(std::span{std::addressof(newV), 1}));
    }

    static void writeValue() {
        auto const currentFlashValue = readFlashValue();
        if(ramCopy != currentFlashValue) {
            internalWrite();
        }
    }
};
}   // namespace Kvasir
