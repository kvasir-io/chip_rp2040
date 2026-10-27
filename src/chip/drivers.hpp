#pragma once
// Every driver of the RP2040, opt-in: chip/chip.hpp is the chip without them. A firmware includes the drivers
// it uses by name (chip/rp_common/<Driver>.hpp) or all of them here.
#include "chip.hpp"
///
#include "rp_common/ADC.hpp"
#include "rp_common/DMA.hpp"
#include "rp_common/DmaCrc.hpp"
#include "rp_common/GpioIrq.hpp"
#include "rp_common/I2C.hpp"
#include "rp_common/I2CQueued.hpp"   // builds on kvasir::bus (kvasir_devices)
#include "rp_common/PIO.hpp"
#include "rp_common/SPI.hpp"
#include "rp_common/UART.hpp"
#include "rp_common/flash.hpp"
#include "rp_common/pwm.hpp"
#include "rp_common/usb.hpp"   // the USB backend, on kvasir::bus as I2CQueued
