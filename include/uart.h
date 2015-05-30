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
#include "_compiler.h"
#include "sysctl.h"

class UART
{
public:
    constexpr UART(LPC_USART_TypeDef *base, const Sysctl &sysctl) :
        _base(base),
        _sysctl(sysctl)
    {}

    UART &configure(unsigned rate) __always_inline {
        _base->CFG = 0;                     // disable
        _base->CTRL = 0;                    // reset
        _base->INTENCLR =                   // disable all interrupts
        INTEN_RXRDY         |
        INTEN_TXRDY         |
        INTEN_DELTACTS      |
        INTEN_TXDIS         |
        INTEN_OVERRUN       |
        INTEN_DELTARXBRK    |
        INTEN_START         |
        INTEN_FRAMERR       |
        INTEN_PARITYERR     |
        INTEN_RXNOISE;
        _base->BRG = (115200 / rate) - 1;   // assumes prescaler configured by Sysctl
        _base->CFG = CFG_DATALEN_8 | CFG_STOPLEN_2 | CFG_ENABLE;

        return *this;
    }

    void send(uint8_t c) const __always_inline
    {
        while (!txidle()) {}

        _base->TXDATA = c;
    }

    bool recv(unsigned &c) const __always_inline
    {
        if (rxready()) {
            c = _base->RXDATA;
            return true;
        }

        return false;
    }

    bool discard() const __always_inline
    {
        unsigned c;
        return recv(c);
    }

    bool rxready() const __always_inline
    {
        return _base->STAT & STAT_RXRDY;
    }

    bool txready() const __always_inline
    {
        return _base->STAT & STAT_TXRDY;
    }

    bool txidle() const __always_inline
    {
        return _base->STAT & STAT_TXIDLE;
    }


    const UART &operator << (uint8_t c) const __always_inline
    {
        send(c);
        return *this;
    }

    const UART &operator << (char c) const __always_inline
    {
        send(c);
        return *this;
    }

    const UART &operator << (const char *s) const __always_inline
    {
        while (*s != '\0') {
            send(*s++);
        }

        return *this;
    }

    const UART &operator << (unsigned val) const
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
    LPC_USART_TypeDef    *const _base;
    const Sysctl        &_sysctl;

    enum Cfg_t {
        CFG_ENABLE      = (1U << 0),    // enable the UART
        CFG_DATALEN_7   = (0U << 2),    // UART 7 bit length mode
        CFG_DATALEN_8   = (1U << 2),    // UART 8 bit length mode
        CFG_DATALEN_9   = (2U << 2),    // UART 9 bit length mode
        CFG_PARITY_NONE = (0U << 4),    // No parity
        CFG_PARITY_EVEN = (2U << 4),    // Even parity
        CFG_PARITY_ODD  = (3U << 4),    // Odd parity
        CFG_STOPLEN_1   = (0U << 6),    // UART One Stop Bit Select
        CFG_STOPLEN_2   = (1U << 6),    // UART Two Stop Bits Select
        CFG_CTSEN       = (1U << 9),    // CTS enable bit
        CFG_SYNCEN      = (1U << 11),   // Synchronous mode enable bit
        CFG_CLKPOL      = (1U << 12),   // Un_RXD rising edge sample enable bit
        CFG_SYNCMST     = (1U << 14),   // Select master mode (synchronous mode) enable bit
        CFG_LOOP        = (1U << 15)    // Loopback mode enable bit
    };

    enum Ctrl_t {
        CTRL_TXBRKEN    = (1U << 1),    // Continuous break enable bit
        CTRL_ADDRDET    = (1U << 2),    // Address detect mode enable bit
        CTRL_TXDIS      = (1U << 6),    // Transmit disable bit
        CTRL_CC         = (1U << 8),    // Continuous Clock mode enable bit
        CTRL_CLRCC      = (1U << 9)     // Clear Continuous Clock bit
    };

    enum Stat_t {
        STAT_RXRDY      = (1U << 0),    // Receiver ready
        STAT_RXIDLE     = (1U << 1),    // Receiver idle
        STAT_TXRDY      = (1U << 2),    // Transmitter ready for data
        STAT_TXIDLE     = (1U << 3),    // Transmitter idle
        STAT_CTS        = (1U << 4),    // Status of CTS signal
        STAT_DELTACTS   = (1U << 5),    // Change in CTS state
        STAT_TXDISINT   = (1U << 6),    // Transmitter disabled
        STAT_OVERRUNINT = (1U << 8),    // Overrun Error interrupt flag.
        STAT_RXBRK      = (1U << 10),   // Received break
        STAT_DELTARXBRK = (1U << 11),   // Change in receive break detection
        STAT_START      = (1U << 12),   // Start detected
        STAT_FRM_ERRINT = (1U << 13),   // Framing Error interrupt flag
        STAT_PAR_ERRINT = (1U << 14),   // Parity Error interrupt flag
        STAT_RXNOISEINT = (1U << 15)    // Received Noise interrupt flag
    };

    enum Inten_t {
        INTEN_RXRDY      = (1U << 0),   // Receive Ready interrupt
        INTEN_TXRDY      = (1U << 2),   // Transmit Ready interrupt
        INTEN_DELTACTS   = (1U << 5),   // Change in CTS state interrupt
        INTEN_TXDIS      = (1U << 6),   // Transmitter disable interrupt
        INTEN_OVERRUN    = (1U << 8),   // Overrun error interrupt
        INTEN_DELTARXBRK = (1U << 11),  // Change in receiver break detection interrupt
        INTEN_START      = (1U << 12),  // Start detect interrupt
        INTEN_FRAMERR    = (1U << 13),  // Frame error interrupt
        INTEN_PARITYERR  = (1U << 14),  // Parity error interrupt
        INTEN_RXNOISE    = (1U << 15)   // Received noise interrupt
    };
};

#define UART0   UART(LPC_USART0, SYSCTL_UART0)
#define UART1   UART(LPC_USART1, SYSCTL_UART1)
#define UART2   UART(LPC_USART2, SYSCTL_UART2)
