#include <array>
#include <cassert>
#include <cstdint>

namespace Kvasir {
namespace Flash {
    namespace detail {
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
        eraseAndWrite_int(std::uint32_t offset, std::uint8_t const* data, std::size_t size) {
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

    }   // namespace detail
    static inline void eraseAndWrite(std::uint32_t addr, std::span<std::byte const> data) {
        assert(256 >= data.size());
        std::uint32_t const        offset = addr - 0x10000000;
        std::array<std::byte, 256> buffer{};
        std::copy(data.begin(), data.end(), buffer.begin());
        assert(addr % 4096 == 0);
        {
            Kvasir::Nvic::InterruptGuard<Kvasir::Nvic::Global> guard{};
            detail::eraseAndWrite_int(
              offset,
              reinterpret_cast<std::uint8_t const*>(buffer.data()),
              buffer.size());
        }
    }
}   // namespace Flash
template<typename Clock, typename T, typename Crc>
struct SimpleEeprom {
    static std::uint16_t calcCrc(T const& v) {
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
