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

class Pin
{
public:
    enum Direction_t {
        Input,
        Output
    };

    enum Modifier_t {
        NoPull      = (0 << 3),
        PullDown    = (1 << 3),
        PullUp      = (2 << 3),
        Hysteresis  = (1 << 5),
        Invert      = (1 << 6),
        PushPull    = (0 << 10),
        OpenDrain   = (1 << 10),
        Filter_0    = (0 << 11),
        Filter_1    = (1 << 11),
        Filter_2    = (2 << 11),
        Filter_3    = (3 << 11),
        ClkDiv_0    = (0 << 13),
        ClkDiv_1    = (1 << 13),
        ClkDiv_2    = (2 << 13),
        ClkDiv_3    = (3 << 13),
        ClkDiv_4    = (4 << 13),
        ClkDiv_5    = (5 << 13),
        ClkDiv_6    = (6 << 13),
    };

private:
    const unsigned  _pin_number;
    const unsigned  _pin_mask;
    __IO uint32_t   * const _iocon_reg;

public:
    constexpr Pin(unsigned pin_number, __IO uint32_t *iocon_reg) : 
        _pin_number(pin_number),
        _pin_mask(1 << pin_number),
        _iocon_reg(iocon_reg)
    {}

    unsigned    number() const __always_inline { return _pin_number; }
    bool        get()    const __always_inline { return LPC_GPIO_PORT->PIN0 & _pin_mask; }
    operator    bool()   const __always_inline { return get(); }
    void        clear()  const __always_inline { LPC_GPIO_PORT->CLR0 = _pin_mask; }
    void        set()    const __always_inline { LPC_GPIO_PORT->SET0 = _pin_mask; }
    void        toggle() const __always_inline { LPC_GPIO_PORT->NOT0 = _pin_mask; }

    void set(bool value) const __always_inline
    {
        (value ? LPC_GPIO_PORT->SET0 : LPC_GPIO_PORT->CLR0) = _pin_mask; 
    }

    Pin & configure(Direction_t direction) __attribute__((always_inline))
    {
        if (direction) {
            LPC_GPIO_PORT->DIR0 |= _pin_mask;
        } else {
            LPC_GPIO_PORT->DIR0 &= ~_pin_mask;
        }
        return *this;
    }

    const Pin & operator << (Modifier_t modifier) const __attribute__((always_inline))
    {
        *_iocon_reg |= (unsigned)modifier;
        return *this;
    }

    const Pin & operator << (int i) const __attribute__((always_inline)) 
    {
        set(i);
        return * this;
    }

};

#define P0_0    Pin(0, &LPC_IOCON->PIO0_0)
#define P0_1    Pin(1, &LPC_IOCON->PIO0_1)
#define P0_2    Pin(2, &LPC_IOCON->PIO0_2)
#define P0_3    Pin(3, &LPC_IOCON->PIO0_3)
#define P0_4    Pin(4, &LPC_IOCON->PIO0_4)
#define P0_5    Pin(5, &LPC_IOCON->PIO0_5)
#define P0_6    Pin(6, &LPC_IOCON->PIO0_6)
#define P0_7    Pin(7, &LPC_IOCON->PIO0_7)
#define P0_8    Pin(8, &LPC_IOCON->PIO0_8)
#define P0_9    Pin(9, &LPC_IOCON->PIO0_9)
#define P0_10   Pin(10, &LPC_IOCON->PIO0_10)
#define P0_11   Pin(11, &LPC_IOCON->PIO0_11)
#define P0_12   Pin(12, &LPC_IOCON->PIO0_12)
#define P0_13   Pin(13, &LPC_IOCON->PIO0_13)
#define P0_14   Pin(14, &LPC_IOCON->PIO0_14)
#define P0_15   Pin(15, &LPC_IOCON->PIO0_15)
#define P0_16   Pin(16, &LPC_IOCON->PIO0_16)
#define P0_17   Pin(17, &LPC_IOCON->PIO0_17)

class MovableFunction
{
public:
    constexpr MovableFunction(unsigned function_number) :
        _function_number(function_number)
    {}

    void claim_pin(const Pin &pin) const __always_inline
    {
        __IO uint32_t *reg = &LPC_SWM->PINASSIGN0 + (_function_number >> 4);
        unsigned shift = (_function_number & 0xf) * 8;

        *reg = (*reg & ~(0xff << shift)) | (pin.number() << shift);
    }

    void release_pin() const __always_inline
    {
        __IO uint32_t *reg = &LPC_SWM->PINASSIGN0 + (_function_number >> 4);
        unsigned shift = (_function_number & 0xf) * 8;

        *reg |= 0xff << shift;        
    }

private:
    const unsigned  _function_number;
};

#define U0_TXD          MovableFunction(0x00) // UART0 TXD Output
#define U0_RXD          MovableFunction(0x01) // UART0 RXD Input
#define U0_RTS          MovableFunction(0x02) // UART0 RTS Output
#define U0_CTS          MovableFunction(0x03) // UART0 CTS Input
#define U0_SCLK         MovableFunction(0x10) // UART0 SCLK I/O
#define U1_TXD          MovableFunction(0x11) // UART1 TXD Output
#define U1_RXD          MovableFunction(0x12) // UART1 RXD Input
#define U1_RTS          MovableFunction(0x13) // UART1 RTS Output
#define U1_CTS          MovableFunction(0x20) // UART1 CTS Input
#define U1_SCLK         MovableFunction(0x21) // UART1 SCLK I/O
#define U2_TXD          MovableFunction(0x22) // UART2 TXD Output
#define U2_RXD          MovableFunction(0x23) // UART2 RXD Input
#define U2_RTS          MovableFunction(0x30) // UART2 RTS Output
#define U2_CTS          MovableFunction(0x31) // UART2 CTS Input
#define U2_SCLK         MovableFunction(0x32) // UART2 SCLK I/O
#define SPI0_SCK        MovableFunction(0x33) // SPI0 SCK I/O
#define SPI0_MOSI       MovableFunction(0x40) // SPI0 MOSI I/O
#define SPI0_MISO       MovableFunction(0x41) // SPI0 MISO I/O
#define SPI0_SSEL       MovableFunction(0x42) // SPI0 SSEL I/O
#define SPI1_SCK        MovableFunction(0x43) // SPI1 SCK I/O
#define SPI1_MOSI       MovableFunction(0x50) // SPI1 MOSI I/O
#define SPI1_MISO       MovableFunction(0x51) // SPI1 MISO I/O
#define SPI1_SSEL       MovableFunction(0x52) // SPI1 SSEL I/O
#define CTIN_0          MovableFunction(0x53) // CTIN0 Input
#define CTIN_1          MovableFunction(0x60) // CTIN1 Input
#define CTIN_2          MovableFunction(0x61) // CTIN2 Input
#define CTIN_3          MovableFunction(0x62) // CTIN3 Input
#define CTOUT_0         MovableFunction(0x63) // CTOUT0 Output
#define CTOUT_1         MovableFunction(0x70) // CTOUT1 Output
#define CTOUT_2         MovableFunction(0x71) // CTOUT2 Output
#define CTOUT_3         MovableFunction(0x72) // CTOUT3 Output
#define I2C_SDA         MovableFunction(0x73) // I2C SDA I/O
#define I2C_SCL         MovableFunction(0x80) // I2C SCL I/O
#define ACMP            MovableFunction(0x81) // I2C ACMP Output
#define CLKOUT          MovableFunction(0x82) // I2C CLKOUT Output
#define GPIO_INT_BMAT   MovableFunction(0x83) // I2C GPIO INT BMAT Output

class FixedFunction
{
public:
    constexpr FixedFunction(unsigned function_number) :
        _function_number(function_number)
    {}

    void enable() const __always_inline { LPC_SWM->PINENABLE0 &= ~(1 << _function_number); }
    void disable() const __always_inline { LPC_SWM->PINENABLE0 |= (1 << _function_number); }

private:
    const unsigned  _function_number;
};

#define ACMP_I1     FixedFunction(0)
#define ACMP_I2     FixedFunction(1)
#define SWCLK       FixedFunction(2)
#define SWDIO       FixedFunction(3)
#define XTALIN      FixedFunction(4)
#define XTALOUT     FixedFunction(5)
#define RST         FixedFunction(6)
#define CLKIN       FixedFunction(7)
#define VDDCMP      FixedFunction(8)

