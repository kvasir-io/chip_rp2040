#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/VREG_AND_CHIP_RESET.hpp"

#include <cstdint>
#include <string_view>
namespace Kvasir { namespace PM {
    enum class ResetCause : std::uint8_t { psm_restart, run, por };

    inline ResetCause reset_cause() {
        using reg = Kvasir::Peripheral::VREG_AND_CHIP_RESET::Registers<>::CHIP_RESET;
        auto f    = apply(read(reg::had_psm_restart), read(reg::had_run), read(reg::had_por));
        if(f[reg::had_psm_restart]) {
            return ResetCause::psm_restart;
        }
        if(f[reg::had_run]) {
            return ResetCause::run;
        }
        if(f[reg::had_por]) {
            return ResetCause::por;
        }
        return ResetCause::por;
    }
}}   // namespace Kvasir::PM
