#pragma once
// The chip itself: what Startup, Io, the clocks, reset, reboot and the watchdog need. No driver and nothing from
// kvasir_devices: include a driver by name (chip/rp_common/I2CQueued.hpp, ...) or all of them through
// chip/drivers.hpp. Every driver header includes the peripherals it uses.
// Any number of an image's TUs may include it, and every other chip header: the Startup itself (vector table, reset
// entry, the runtime-library stand-ins) is kvasir/StartUp/StartUp.hpp, which the main TU alone includes.
#include "peripherals/CLOCKS.hpp"
#include "peripherals/IO_BANK0.hpp"
#include "peripherals/IO_QSPI.hpp"
#include "peripherals/PLL.hpp"
#include "peripherals/PSM.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/SIO.hpp"
#include "peripherals/TIMER.hpp"
#include "peripherals/WATCHDOG.hpp"
#include "peripherals/XIP_SSI.hpp"
#include "peripherals/XOSC.hpp"
///
#include "PinConfig.hpp"
///
#include "Interrupt.hpp"
#include "StartUp.hpp"
#include "core/core.hpp"
/// (Register::atomic() on the alias windows, rp_common/RegisterAlias.hpp, comes with the register layer:
/// KVASIR_CHIP_REGISTER_SEAM, cmake/chip.cmake.)
#include "rp_common/DebuggerPresent.hpp"
#include "rp_common/Io.hpp"
#include "rp_common/Timer.hpp"
/// the boot ROM and the flash, one header per concept; AnnouncedResetClock.hpp is the chip's clock of
/// Kvasir::AnnouncedReset (a weak definition every image needs). The OTP reads and the serial number are no part
/// of it: a user includes rp_common/SerialNumber.hpp (or rp_common/Otp.hpp) by name.
#include "rp_common/AnnouncedResetClock.hpp"
#include "rp_common/BootInfo.hpp"
#include "rp_common/BootromReboot.hpp"
#include "rp_common/FlashOps.hpp"
#include "rp_common/Rom.hpp"
#include "rp_common/XipControl.hpp"
///
#include "rp_common/DegradedBoot.hpp"   // Clocks::DegradedBoot: the chip's own entry in every Startup list
#include "rp_common/chip_reset.hpp"
#include "rp_common/clock_config.hpp"
#include "rp_common/watchdog.hpp"
///
#include "rp_common/Reboot.hpp"
