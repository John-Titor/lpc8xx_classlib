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

class SCT
{
private:
    enum Config_t : uint32_t {
        CONFIG_16BIT_COUNTER        = (0U << 0),   // Operate as 2 16-bit counters
        CONFIG_32BIT_COUNTER        = (1U << 0),   // Operate as 1 32-bit counter
        CONFIG_CLKMODE_BUSCLK       = (0U << 1),   // Bus clock
        CONFIG_CLKMODE_SCTCLK       = (1U << 1),   // SCT clock
        CONFIG_CLKMODE_INCLK        = (2U << 1),   // Input clock selected in CLKSEL field
        CONFIG_CLKMODE_INEDGECLK    = (3U << 1),   // Input clock edge selected in CLKSEL field
        CONFIG_NORELOAD_U           = (1U << 7),   // Prevent match register reload
        CONFIG_NORELOAD_L           = (1U << 7),   // Prevent lower match register reload
        CONFIG_NORELOAD_H           = (1U << 8),   // Prevent upper match register reload
        CONFIG_AUTOLIMIT_U          = (1U << 17),  // Limits counter(unified) based on MATCH0
        CONFIG_AUTOLIMIT_L          = (1U << 17),  // Limits counter(L) based on MATCH0
        CONFIG_AUTOLIMIT_H          = (1U << 18)   // Limits counter(L) based on MATCH0
    };

    enum Ctrl_t : uint16_t {
        CTRL_STOP                   = (1U << 1),    // Stop counter
        CTRL_HALT                   = (1U << 2),    // Halt counter
        CTRL_CLRCTR                 = (1U << 3),    // Clear counter
        CTRL_BIDIR                  = (1U << 4)     // Enable bidirectional mode
    };

    enum Conflict_t : unsigned {
        RES_NONE                    = 0,
        RES_SET                     = 1,
        RES_CLEAR                   = 2,
        RES_TOGGLE                  = 3
    };

    enum Event_t : uint32_t {
        EVENT_MATCH_0               = (0U << 0),
        EVENT_MATCH_1               = (1U << 0),
        EVENT_MATCH_2               = (2U << 0),
        EVENT_MATCH_3               = (3U << 0),
        EVENT_MATCH_4               = (4U << 0),
        EVENT_LEVENT                = (0U << 4),
        EVENT_HEVENT                = (1U << 4),
        EVENT_OUTSEL                = (1U << 5),
        EVENT_IOSEL_0               = (0U << 6),
        EVENT_IOSEL_1               = (1U << 6),
        EVENT_IOSEL_2               = (2U << 6),
        EVENT_IOSEL_3               = (3U << 6),
        EVENT_IOCOND_LOW            = (0U << 10),
        EVENT_IOCOND_RISE           = (1U << 10),
        EVENT_IOCOND_FALL           = (2U << 10),
        EVENT_IOCOND_HIGH           = (3U << 10),
        EVENT_COMBMODE_OR           = (0U << 12),
        EVENT_COMBMODE_MATCH        = (1U << 12),
        EVENT_COMBMODE_IO           = (2U << 12),
        EVENT_COMBMODE_AND          = (3U << 12),
        EVENT_STATELD               = (1U << 14),
        EVENT_MATCHMEM              = (1U << 20),
        EVENT_COUNTING_UP           = (1U << 21),
        EVENT_COUNTING_DOWN         = (2U << 21)
    };

public:

    static void configure_fout()
    {
        LPC_SCT->OUTPUT = 0;                        // turn off outputs initially

        LPC_SCT->CONFIG =
            CONFIG_16BIT_COUNTER |                  // two separate counters
            CONFIG_AUTOLIMIT_L |                    // auto-limit L on match 0
            CONFIG_AUTOLIMIT_H;                     // auto-limit H on match 0

        LPC_SCT->CTRL_L = (23 << 5);                // set / 24 prescaler (microseconds)
        LPC_SCT->CTRL_H = (23 << 5);                // set / 24 prescaler (microseconds)

        LPC_SCT->RES |= (RES_TOGGLE << 0) |         // output 0 toggles on conflict
                        (RES_TOGGLE << 2);          // output 1 toggles on conflict

        LPC_SCT->MATCHREL[0].L = 0xffff;            // event 0 reload maximum value (min. freq. output)

        LPC_SCT->EVENT[0].STATE = 1;                // always enabled
        LPC_SCT->EVENT[0].CTRL =
            EVENT_MATCH_0 |                         // match 0
            EVENT_LEVENT |                          // L counter
            EVENT_COMBMODE_MATCH;                   // on match only

        LPC_SCT->OUT[0].SET = (1 << 0);             // event 0 will set output 0
        LPC_SCT->OUT[0].CLR = (1 << 0);             // event 0 will clear output 0

        LPC_SCT->MATCHREL[0].H = 0xffff;            // event 0 reload maximum value (min. freq. output)

        LPC_SCT->EVENT[1].STATE = 1;                // always enabled
        LPC_SCT->EVENT[1].CTRL =
            EVENT_MATCH_0 |                         // match 0
            EVENT_HEVENT |                          // H counter
            EVENT_COMBMODE_MATCH;                   // on match only

        LPC_SCT->OUT[1].SET = (1 << 1);             // event 1 will set output 1
        LPC_SCT->OUT[1].CLR = (1 << 1);             // event 1 will clear output 1

        LPC_SCT->CTRL_L &= ~(CTRL_HALT);            // start counters
        LPC_SCT->CTRL_H &= ~(CTRL_HALT);            // start counters
    }

    static void set_fout_period(unsigned output, unsigned period) __always_inline {
        if (period < 4)
        {
            period = 4;
        } else if (period > 0x1ffff)
        {
            period = 0x1ffff;
        }

        period /= 2;
        period -= 1;

        if (output == 0)
        {
            LPC_SCT->MATCHREL[0].L = period;
        } else {
            LPC_SCT->MATCHREL[0].H = period;
        }
    }
};
