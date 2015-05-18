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

extern "C" void MRT_IRQHandler();

class Timer
{
public:
    typedef void        (* Callback)();

    constexpr Timer(unsigned index, __IO uint32_t *intval, __IO uint32_t *ctrl) :
        _index(index),
        _intval(intval),
        _ctrl(ctrl)
    {}

    void configure(Callback callback, unsigned period, bool repeat) __attribute__((always_inline))
    {
        *_intval = 0x80000000;              // stop the timer
        *_ctrl = repeat ? 0x01 : 0x03;      // pick one-shot or repeat, interrupt enabled
        _callbacks[_index] = callback;      // save the callback
        *_intval = period | 0x80000000;     // immediate load of new value & start
    }

    void                cancel() { configure (nullptr, 0, false); }

private:
    const unsigned      _index;
    __IO uint32_t       * const _intval;
    __IO uint32_t       * const _ctrl;

    static Callback     _callbacks[4];

    friend void ::MRT_IRQHandler();
};

#define Timer0  Timer(0, &LPC_MRT->INTVAL0, &LPC_MRT->CTRL0)
#define Timer1  Timer(1, &LPC_MRT->INTVAL1, &LPC_MRT->CTRL1)
#define Timer2  Timer(2, &LPC_MRT->INTVAL2, &LPC_MRT->CTRL2)
#define Timer3  Timer(3, &LPC_MRT->INTVAL3, &LPC_MRT->CTRL3)
