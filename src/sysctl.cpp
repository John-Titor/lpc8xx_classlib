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

#include "sysctl.h"

namespace {
void
common_init()
{
    LPC_SYSCON->SYSAHBCLKCTRL |= Sysctl::CLOCK_ALL;  // all clocks on

//    LPC_FLASHCTRL->FLASHCFG &= ~(3);            // no flash waitstates required up to 30MHz

    LPC_GPIO_PORT->DIR0 |= (1U << 10) | (1U << 11); // default I2C pin states
    LPC_GPIO_PORT->SET0 |= (1U << 10) | (1U << 11); // ... to prevent them floating

    LPC_SWM->PINENABLE0 |=                      // disable annoying fixed pin functions
        (1U << 2) |                             // ... SWCLK
        (1U << 3) |                             // ... SWDIO
        (1U << 6);                              // ... RST
}
} // namespace

void
Sysctl::init_12MHz()
{
    common_init();

    LPC_SYSCON->SYSAHBCLKDIV = 1;               // run fabric at /1
    LPC_SYSCON->MAINCLKSEL = 0;                 // ... using the IRC
    LPC_SYSCON->MAINCLKUEN = 0;                 // latch the new clock source
    LPC_SYSCON->MAINCLKUEN = 1;

    set_uart_prescale(6, 21);                   // UART base clock = 115200 * 16
}

void
Sysctl::init_24MHz()
{
    common_init();

    LPC_SYSCON->SYSPLLCLKSEL = PLLCLKSRC_IRC;   // select the IRC as the PLL input
    LPC_SYSCON->SYSPLLCLKUEN = 0;               // latch the new PLL source
    LPC_SYSCON->SYSPLLCLKUEN = 1;

    LPC_SYSCON->PDRUNCFG |= SLPWAKE_SYSPLL_PD;  // power down the PLL to configure
    LPC_SYSCON->SYSPLLCTRL =                    // Configure for FCLKIN = 12MHz, FCLKOUT = 24MHz
        (1U << 0) |                             // ... M
        (2U << 5);                              // ... P
    LPC_SYSCON->PDRUNCFG &= ~SLPWAKE_SYSPLL_PD; // power up the PLL
    while (~LPC_SYSCON->SYSPLLSTAT & 1) {}      // ... wait for PLL lock

    LPC_SYSCON->SYSAHBCLKDIV = 1;               // run fabric at /1
    LPC_SYSCON->MAINCLKSEL = MAINCLKSRC_PLLOUT; // ... using the PLL
    LPC_SYSCON->MAINCLKUEN = 0;                 // latch the new clock source
    LPC_SYSCON->MAINCLKUEN = 1;

    set_uart_prescale(13, 0);                   // UART base clock = 115200 * 16
}

void
Sysctl::set_uart_prescale(unsigned idiv, unsigned fmul)
{
    SYSCTL_UARTFBRG.reset();                    // reset the FBRG
    LPC_SYSCON->UARTCLKDIV = 0;                 // UART clock off
    LPC_SYSCON->UARTFRGMULT = fmul;             // configure
    LPC_SYSCON->UARTFRGDIV = 0xff;              // ... always 256
    LPC_SYSCON->UARTCLKDIV = idiv;              // and clock back on
}

void
Sysctl::enable_clkout(const Pin &pin, ClkoutClockSource_t clk, unsigned div)
{
    CLKOUT.claim_pin(pin);
    LPC_SYSCON->CLKOUTSEL = clk;
    LPC_SYSCON->CLKOUTUEN = 0;
    LPC_SYSCON->CLKOUTUEN = 1;
    LPC_SYSCON->CLKOUTDIV = div;
}
