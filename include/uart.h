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
#include "sysctl.h"

class UART
{
public:
    constexpr UART(LPC_USART_TypeDef *base, const Sysctl &sysctl) :
        _base(base),
        _sysctl(sysctl)
    {}

    void configure(unsigned rate) __attribute__((always_inline))
    {
        _sysctl.clock(true);

        _base->CFG = 0;
        _base->CTRL = 0;
        _base->BRG = (115200 / rate) - 1;   // assumes prescaler configured by Sysctl
        _base->STAT = ~0;
        _base->CFG = CFG_DATALEN_8;
        _base->CFG |= CFG_ENABLE;
    }

    void send(uint8_t c) const __attribute__((always_inline))
    {
        while ((_base->STAT & STAT_TXRDY) == 0) {}

        _base->TXDATA = c;
    }

    const UART & operator << (uint8_t c) const __attribute__((always_inline))
    {
        send(c);
        return *this;
    }

    const UART & operator << (char c) const __attribute__((always_inline))
    {
        send(c);
        return *this;
    }

    const UART & operator << (const char *s) const __attribute__((always_inline))
    {
        while (*s != '\0') {
            send(*s++);
        }
        return *this;
    }

    const UART & operator << (unsigned val) const
    {
        for (int shift = 28; shift >= 0; shift -= 4) {
            unsigned n = (val >> shift) & 0xf;
            uint8_t c;
            if (n < 10) {
                c = '0' + n;
            } else {
                c = 'A' + n - 10;
            }
            send(c);
        }
        return *this;
    }

private:
    LPC_USART_TypeDef   * const _base;
    const Sysctl        &_sysctl;

    enum Cfg_t {
        CFG_ENABLE      = (1 << 0),     // enable the UART
        CFG_DATALEN_7   = (0 << 2),     // UART 7 bit length mode
        CFG_DATALEN_8   = (1 << 2),     // UART 8 bit length mode
        CFG_DATALEN_9   = (2 << 2),     // UART 9 bit length mode
        CFG_PARITY_NONE = (0 << 4),     // No parity
        CFG_PARITY_EVEN = (2 << 4),     // Even parity
        CFG_PARITY_ODD  = (3 << 4),     // Odd parity
        CFG_STOPLEN_1   = (0 << 6),     // UART One Stop Bit Select
        CFG_STOPLEN_2   = (1 << 6),     // UART Two Stop Bits Select
        CFG_CTSEN       = (1 << 9),     // CTS enable bit
        CFG_SYNCEN      = (1 << 11),    // Synchronous mode enable bit
        CFG_CLKPOL      = (1 << 12),    // Un_RXD rising edge sample enable bit
        CFG_SYNCMST     = (1 << 14),    // Select master mode (synchronous mode) enable bit
        CFG_LOOP        = (1 << 15)     // Loopback mode enable bit
    };

    enum Ctrl_t {
        CTRL_TXBRKEN    = (1 << 1),     // Continuous break enable bit
        CTRL_ADDRDET    = (1 << 2),     // Address detect mode enable bit
        CTRL_TXDIS      = (1 << 6),     // Transmit disable bit
        CTRL_CC         = (1 << 8),     // Continuous Clock mode enable bit
        CTRL_CLRCC      = (1 << 9)      // Clear Continuous Clock bit
    };

    enum Stat_t {
        STAT_RXRDY      = (1 << 0),     // Receiver ready
        STAT_RXIDLE     = (1 << 1),     // Receiver idle
        STAT_TXRDY      = (1 << 2),     // Transmitter ready for data
        STAT_TXIDLE     = (1 << 3),     // Transmitter idle
        STAT_CTS        = (1 << 4),     // Status of CTS signal
        STAT_DELTACTS   = (1 << 5),     // Change in CTS state
        STAT_TXDISINT   = (1 << 6),     // Transmitter disabled
        STAT_OVERRUNINT = (1 << 8),     // Overrun Error interrupt flag.
        STAT_RXBRK      = (1 << 10),    // Received break
        STAT_DELTARXBRK = (1 << 11),    // Change in receive break detection
        STAT_START      = (1 << 12),    // Start detected
        STAT_FRM_ERRINT = (1 << 13),    // Framing Error interrupt flag
        STAT_PAR_ERRINT = (1 << 14),    // Parity Error interrupt flag
        STAT_RXNOISEINT = (1 << 15)     // Received Noise interrupt flag
    };
};

#define UART0   UART(LPC_USART0, SYSCTL_UART0)
#define UART1   UART(LPC_USART1, SYSCTL_UART1)
#define UART2   UART(LPC_USART2, SYSCTL_UART2)
