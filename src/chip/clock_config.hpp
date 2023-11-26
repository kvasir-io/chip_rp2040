#pragma once
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"

#include <cmath>
#include <cstdint>

namespace Kvasir { namespace DefaultClockSettings {

    namespace detail {
        struct PllSettings {
            std::uint32_t fbdiv;
            std::uint32_t pd1;
            std::uint32_t pd2;
            std::uint32_t refdiv;
        };

        static constexpr PllSettings calcPllSettings(double clockSpeed, double crystalSpeed) {
            constexpr double vco_max = 1'600'000'000;
            constexpr double vco_min = 750'000'000;

            constexpr std::uint32_t fbdiv_max = 320;
            constexpr std::uint32_t fbdiv_min = 16;

            constexpr std::uint32_t pd2_max = 7;
            constexpr std::uint32_t pd2_min = 1;

            constexpr std::uint32_t pd1_max = 7;
            constexpr std::uint32_t pd1_min = 1;

            PllSettings bestSettings{0, 0, 0, 0};
            double      besetMargin = clockSpeed;
            double      bestOut     = 0.0;

            for(std::uint32_t fbdiv{fbdiv_min}; fbdiv <= fbdiv_max; ++fbdiv) {
                double const vco = crystalSpeed * fbdiv;
                if(vco < vco_min || vco > vco_max) {
                    continue;
                }
                for(std::uint32_t pd2{pd2_min}; pd2 <= pd2_max; ++pd2) {
                    for(std::uint32_t pd1{pd1_min}; pd1 <= pd1_max; ++pd1) {
                        double const out        = vco / pd1 / pd2;
                        double const margin     = (out - clockSpeed);
                        double const abs_margin = margin > 0 ? margin : -margin;
                        if(abs_margin < besetMargin) {
                            bestSettings = PllSettings{fbdiv, pd1, pd2, 1};
                            besetMargin  = abs_margin;
                            bestOut      = out;
                        }
                    }
                }
            }
            assert(bestOut == clockSpeed);
            return bestSettings;
        }
    }   // namespace detail

    template<auto ClockSpeed, auto CrystalSpeed>
    void coreClockInit() {
        using Kvasir::Register::value;

        static constexpr auto pllSettings     = detail::calcPllSettings(ClockSpeed, CrystalSpeed);
        static constexpr auto usb_pllSettings = detail::calcPllSettings(48'000'000, CrystalSpeed);

        static_assert(
          ClockSpeed
            == (CrystalSpeed / pllSettings.refdiv) * pllSettings.fbdiv
                 / (pllSettings.pd1 * pllSettings.pd2),
          "bad clock config");

        static_assert(
          48'000'000
            == (CrystalSpeed / usb_pllSettings.refdiv) * usb_pllSettings.fbdiv
                 / (usb_pllSettings.pd1 * usb_pllSettings.pd2),
          "bad clock config");

        using PERI_CLOCK = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_CTRL;
        using SYS_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
        using REF_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_CTRL;
        using USB_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using XOSC       = Kvasir::Peripheral::XOSC::Registers<>;
        using RST        = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL        = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL     = Kvasir::Peripheral::PLL::Registers<1>;

        // disable periphery clocks
        apply(PERI_CLOCK::overrideDefaults(clear(PERI_CLOCK::enable)));

        // set ref clock to default
        apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::rosc_clksrc_ph)));

        // set sysclock to default
        apply(SYS_CLOCK::overrideDefaults(write(SYS_CLOCK::SRCValC::clk_ref)));

        apply(write(XOSC::CTRL::ENABLEValC::en), write(XOSC::CTRL::FREQ_RANGEValC::_1_15mhz));
        // wait for XOSC stable
        while(!apply(read(XOSC::STATUS::stable))) {
        }
        {   //sys pll
            // reset pll
            apply(set(RST::RESET::pll_sys));
            apply(clear(RST::RESET::pll_sys));
            while(!apply(read(RST::RESET_DONE::pll_sys))) {
            }

            apply(PLL::CS::overrideDefaults(
              clear(PLL::CS::bypass),
              write(PLL::CS::refdiv, value<pllSettings.refdiv>())));

            apply(write(PLL::FBDIV_INT::fbdiv_int, value<pllSettings.fbdiv>()));

            apply(PLL::PWR::overrideDefaults(
              clear(PLL::PWR::vcopd),
              clear(PLL::PWR::pd),
              set(PLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(PLL::CS::lock))) {
            }

            apply(PLL::PRIM::overrideDefaults(
              write(PLL::PRIM::postdiv1, value<pllSettings.pd1>()),
              write(PLL::PRIM::postdiv2, value<pllSettings.pd2>())));

            apply(PLL::PWR::overrideDefaults(
              clear(PLL::PWR::vcopd),
              clear(PLL::PWR::pd),
              clear(PLL::PWR::postdivpd)));

            // set sysclock to pll
            apply(SYS_CLOCK::overrideDefaults(
              write(SYS_CLOCK::SRCValC::clksrc_clk_sys_aux),
              write(SYS_CLOCK::AUXSRCValC::clksrc_pll_sys)));

            // set ref clock to xosc
            apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::xosc_clksrc)));

            // enable periphery clock
            apply(PERI_CLOCK::overrideDefaults(set(PERI_CLOCK::enable)));
        }

        {   //usb pll
            // reset pll
            apply(set(RST::RESET::pll_usb));
            apply(clear(RST::RESET::pll_usb));
            while(!apply(read(RST::RESET_DONE::pll_usb))) {
            }

            apply(USBPLL::CS::overrideDefaults(
              clear(USBPLL::CS::bypass),
              write(USBPLL::CS::refdiv, value<usb_pllSettings.refdiv>())));

            apply(write(USBPLL::FBDIV_INT::fbdiv_int, value<usb_pllSettings.fbdiv>()));

            apply(USBPLL::PWR::overrideDefaults(
              clear(USBPLL::PWR::vcopd),
              clear(USBPLL::PWR::pd),
              set(USBPLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(USBPLL::CS::lock))) {
            }

            apply(USBPLL::PRIM::overrideDefaults(
              write(USBPLL::PRIM::postdiv1, value<usb_pllSettings.pd1>()),
              write(USBPLL::PRIM::postdiv2, value<usb_pllSettings.pd2>())));

            apply(USBPLL::PWR::overrideDefaults(
              clear(USBPLL::PWR::vcopd),
              clear(USBPLL::PWR::pd),
              clear(USBPLL::PWR::postdivpd)));

            // enable periphery clock
            apply(USB_CLOCK::overrideDefaults(
              set(USB_CLOCK::enable),
              write(USB_CLOCK::AUXSRCValC::clksrc_pll_usb)));
        }
    }

    template<auto ClockSpeed, auto CrystalSpeed>
    void peripheryClockInit() {}

}}   // namespace Kvasir::DefaultClockSettings
