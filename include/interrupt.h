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

class Interrupt
{
public:
    constexpr Interrupt(IRQn_Type vector) :
        _vector(vector)
    {}

    void                enable()  const __always_inline { NVIC_EnableIRQ(_vector); }
    void                disable() const __always_inline { NVIC_DisableIRQ(_vector); }
    void                set_priority(unsigned priority) const __always_inline { NVIC_SetPriority(_vector, priority); }

    static void         enable_all() __always_inline { __enable_irq(); }
    static void         disable_all() __always_inline { __disable_irq(); }
    static void         wait() __always_inline { __WFI(); }

private:
    const IRQn_Type      _vector;
};

#define SPI0_IRQ        Interrupt(SPI0_IRQn)
#define SPI1_IRQ        Interrupt(SPI1_IRQn)
#define UART0_IRQ       Interrupt(UART0_IRQn)
#define UART1_IRQ       Interrupt(UART1_IRQn)
#define UART2_IRQ       Interrupt(UART2_IRQn)
#define I2C_IRQ         Interrupt(I2C_IRQn)
#define SCT_IRQ         Interrupt(SCT_IRQn)
#define MRT_IRQ         Interrupt(MRT_IRQn)
#define CMP_IRQ         Interrupt(CMP_IRQn)
#define WDT_IRQ         Interrupt(WDT_IRQn)
#define BOD_IRQ         Interrupt(BOD_IRQn)
#define WKT_IRQ         Interrupt(WKT_IRQn)
#define PININT0_IRQ     Interrupt(PININT0_IRQn)
#define PININT1_IRQ     Interrupt(PININT1_IRQn)
#define PININT2_IRQ     Interrupt(PININT2_IRQn)
#define PININT3_IRQ     Interrupt(PININT3_IRQn)
#define PININT4_IRQ     Interrupt(PININT4_IRQn)
#define PININT5_IRQ     Interrupt(PININT5_IRQn)
#define PININT6_IRQ     Interrupt(PININT6_IRQn)
#define PININT7_IRQ     Interrupt(PININT7_IRQn)
