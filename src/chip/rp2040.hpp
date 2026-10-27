#pragma once
// The chip this package is: the RP2040. cmake/chip.cmake passes KVASIR_CHIP_RP2040 to every TU of an image (kvasir_chip
// DEFINITIONS); rp_common/PinConfig.hpp defines Chip::Part and Chip::part from it. chip/Interrupt.hpp includes this
// header, so every chip user gets the check below. Build scripts test for the file name to tell the packages apart.
#include "rp_common/PinConfig.hpp"

#include <string_view>

#ifndef KVASIR_CHIP_RP2040
    #error \
      "chip_rp2040's chip/rp2040.hpp without KVASIR_CHIP_RP2040: compiled with another chip package's kvasir_chip DEFINITIONS?"
#endif

namespace Kvasir::Chip {
// which part this image is for, as text (the value is Chip::part)
inline constexpr std::string_view partName = "RP2040";
}   // namespace Kvasir::Chip
