// Copyright (c) 2015 Michael Smith, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

#pragma once

#include <LPC8xx.h>

#include "pin.h"

class Sysctl
{
public:
    enum Clock_t {
        CLOCK_SYS = 0,      // System clock
        CLOCK_ROM,          // ROM clock
        CLOCK_RAM,          // RAM clock
        CLOCK_FLASHREG,     // FLASH register interface clock
        CLOCK_FLASH,        // FLASH array access clock
        CLOCK_I2C0,         // I2C0 clock
        CLOCK_GPIO,         // GPIO clock
        CLOCK_SWM,          // Switch matrix clock
        CLOCK_SCT,          // State configurable timer clock
        CLOCK_WKT,          // Self wake-up timer clock
        CLOCK_MRT,          // Multi-rate timer clock
        CLOCK_SPI0,         // SPI0 clock
        CLOCK_SPI1,         // SPI01 clock
        CLOCK_CRC,          // CRC clock
        CLOCK_UART0,        // UART0 clock
        CLOCK_UART1,        // UART1 clock
        CLOCK_UART2,        // UART2 clock
        CLOCK_WWDT,         // Watchdog clock
        CLOCK_IOCON,        // IOCON clock
        CLOCK_ACOMP,        // Analog comparator clock

        CLOCK_NONE
    };

    enum Reset_t {
        RESET_SPI0,         // SPI0 reset control
        RESET_SPI1,         // SPI1 reset control
        RESET_UARTFBRG,     // UART fractional baud rate generator reset control
        RESET_UART0,        // UART0 reset control
        RESET_UART1,        // UART1 reset control
        RESET_UART2,        // UART2 reset control
        RESET_I2C0,         // I2C0 reset control
        RESET_MRT,          // MRT reset control
        RESET_SCT,          // SCT reset control
        RESET_WKT,          // Self wake-up timer (WKT) control
        RESET_GPIO,         // GPIO reset control
        RESET_FLASH,        // FLASH reset control
        RESET_ACMP,         // ACMP reset control

        RESET_NONE
    };

    enum PowerDown_t {
        SLPWAKE_IRCOUT_PD = (1 << 0),   // IRC oscillator output wake-up configuration
        SLPWAKE_IRC_PD    = (1 << 1),   // IRC oscillator power-down wake-up configuration
        SLPWAKE_FLASH_PD  = (1 << 2),   // Flash wake-up configuration
        SLPWAKE_BOD_PD    = (1 << 3),   // BOD wake-up configuration
        SLPWAKE_SYSOSC_PD = (1 << 5),   // System oscillator wake-up configuration
        SLPWAKE_WDTOSC_PD = (1 << 6),   // Watchdog oscillator wake-up configuration
        SLPWAKE_SYSPLL_PD = (1 << 7),   // System PLL wake-up configuration
        SLPWAKE_ACMP_PD   = (1 << 15)   // Analog comparator wake-up configuration
    };

    enum PLLClockSource_t {
        PLLCLKSRC_IRC       = 0,        // Internal oscillator
        PLLCLKSRC_SYSOSC    = 1,        // Crystal (system) oscillator
        PLLCLKSRC_EXT_CLKIN = 3         // External clock input
    };

    enum MainClockSource_t {
        MAINCLKSRC_IRC      = 0,        // Internal oscillator
        MAINCLKSRC_PLLIN    = 1,        // System PLL input
        MAINCLKSRC_WDTOSC   = 2,        // Watchdog oscillator rate
        MAINCLKSRC_PLLOUT   = 3         // System PLL output
    };

    enum ClkoutClockSource_t {
        CLKOUTSRC_IRC           = 0,    // Internal oscillator for CLKOUT
        CLKOUTSRC_SYSOSC        = 1,    // System oscillator for CLKOUT
        CLKOUTSRC_WDTOSC        = 2,    // Watchdog oscillator for CLKOUT
        CLKOUTSRC_MAINSYSCLK    = 3,    // Main system clock for CLKOUT
    };

    constexpr Sysctl(uint32_t clock, uint32_t reset) :
        _clock_bit((clock < CLOCK_NONE) ? (1 << clock) : 0),
        _reset_bit((reset < RESET_NONE) ? (1 << reset) : 0)
    {}

    void reset() const __attribute__((always_inline))
    {
        LPC_SYSCON->PRESETCTRL &= ~_reset_bit;
        LPC_SYSCON->PRESETCTRL |= _reset_bit;
    }

    void clock(bool enable) const __attribute__((always_inline))
    {
        if (enable) {
            LPC_SYSCON->SYSAHBCLKCTRL |= _clock_bit;
            reset();
        } else {
            LPC_SYSCON->SYSAHBCLKCTRL &= ~_clock_bit;
        }
    }

    static void     init_12MHz();           // run CPU & fabric @ 12MHz
    static void     init_24MHz();           // run CPU & fabric @ 24MHz

    static void     set_uart_prescale(unsigned idiv, unsigned fmul = 0);
    static void     enable_clkout(const Pin &pin, ClkoutClockSource_t clk, unsigned div);


private:
    const unsigned  _clock_bit;
    const unsigned  _reset_bit;
};


#define SYSCTL_SYS      Sysctl(Sysctl::CLOCK_SYS,       Sysctl::RESET_NONE)
#define SYSCTL_ROM      Sysctl(Sysctl::CLOCK_ROM,       Sysctl::RESET_NONE)
#define SYSCTL_RAM      Sysctl(Sysctl::CLOCK_RAM,       Sysctl::RESET_NONE)
#define SYSCTL_FLASHREG Sysctl(Sysctl::CLOCK_FLASHREG,  Sysctl::RESET_NONE)
#define SYSCTL_FLASH    Sysctl(Sysctl::CLOCK_FLASH,     Sysctl::RESET_FLASH)
#define SYSCTL_I2C0     Sysctl(Sysctl::CLOCK_I2C0,      Sysctl::RESET_I2C0)
#define SYSCTL_GPIO     Sysctl(Sysctl::CLOCK_GPIO,      Sysctl::RESET_GPIO)
#define SYSCTL_SWM      Sysctl(Sysctl::CLOCK_SWM,       Sysctl::RESET_NONE)
#define SYSCTL_SCT      Sysctl(Sysctl::CLOCK_SCT,       Sysctl::RESET_SCT)
#define SYSCTL_WKT      Sysctl(Sysctl::CLOCK_WKT,       Sysctl::RESET_WKT)
#define SYSCTL_MRT      Sysctl(Sysctl::CLOCK_MRT,       Sysctl::RESET_MRT)
#define SYSCTL_SPI0     Sysctl(Sysctl::CLOCK_SPI0,      Sysctl::RESET_SPI0)
#define SYSCTL_SPI1     Sysctl(Sysctl::CLOCK_SPI1,      Sysctl::RESET_SPI1)
#define SYSCTL_UARTFBRG Sysctl(Sysctl::CLOCK_NONE,      Sysctl::RESET_UARTFBRG)
#define SYSCTL_CRC      Sysctl(Sysctl::CLOCK_CRC,       Sysctl::RESET_NONE)
#define SYSCTL_UART0    Sysctl(Sysctl::CLOCK_UART0,     Sysctl::RESET_UART0)
#define SYSCTL_UART1    Sysctl(Sysctl::CLOCK_UART1,     Sysctl::RESET_UART1)
#define SYSCTL_UART2    Sysctl(Sysctl::CLOCK_UART2,     Sysctl::RESET_UART1)
#define SYSCTL_WWDT     Sysctl(Sysctl::CLOCK_WWDT,      Sysctl::RESET_NONE)
#define SYSCTL_IOCON    Sysctl(Sysctl::CLOCK_IOCON,     Sysctl::RESET_NONE)
#define SYSCTL_ACOMP    Sysctl(Sysctl::CLOCK_ACOMP,     Sysctl::RESET_ACMP)
