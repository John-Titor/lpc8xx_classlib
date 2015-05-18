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

#include <stdint.h>
#include <CMSIS/LPC8xx.h>

extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _estack;

typedef void (* initfunc_t)();
extern initfunc_t _sinit;
extern initfunc_t _einit;

extern "C" void main();
static void _start();
static void _crt_start();

void
_start()
{
    /* do early init */
    _crt_start();

    /* call main */
    main();

    /* dead loop */
    for (;;);
}

void
_crt_start()
{
    uint32_t *src, *dst;

    /* copy data to RAM */
    src = (uint32_t *)&_etext;
    dst = (uint32_t *)&_data;

    while (dst < (uint32_t *)&_edata) {
        *dst++ = *src++;
    }

    /* zero the BSS */
    dst = (uint32_t *) &_bss;

    while (dst < (uint32_t *)&_ebss) {
        *dst++ = 0;
    }

    /* and fill the rest of memory (stack) */
    while (dst < (uint32_t *)&_estack) {
        *dst++ = 0xffffffff;
    }

    /* run initialiser functions */
    initfunc_t *fp = &_sinit;

    while (fp < &_einit) {
        (*fp++)();
    }
}

// 
// Vectors
// 
static void _start();
extern "C" void _default_handler() { for(;;); }
extern "C" void NMI_Handler()          __attribute__((weak, alias("_default_handler")));
extern "C" void HardFault_Handler()    __attribute__((weak, alias("_default_handler")));
extern "C" void SVC_Handler()          __attribute__((weak, alias("_default_handler")));
extern "C" void PendSV_Handler()       __attribute__((weak, alias("_default_handler")));
extern "C" void SysTick_Handler()      __attribute__((weak, alias("_default_handler")));
extern "C" void IntDefaultHandler()    __attribute__((weak, alias("_default_handler")));
extern "C" void SPI0_IRQHandler()      __attribute__((weak, alias("_default_handler")));
extern "C" void SPI1_IRQHandler()      __attribute__((weak, alias("_default_handler")));
extern "C" void UART0_IRQHandler()     __attribute__((weak, alias("_default_handler")));
extern "C" void UART1_IRQHandler()     __attribute__((weak, alias("_default_handler")));
extern "C" void UART2_IRQHandler()     __attribute__((weak, alias("_default_handler")));
extern "C" void I2C_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void SCT_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void MRT_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void CMP_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void WDT_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void BOD_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void WKT_IRQHandler()       __attribute__((weak, alias("_default_handler")));
extern "C" void PININT0_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT1_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT2_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT3_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT4_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT5_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT6_IRQHandler()   __attribute__((weak, alias("_default_handler")));
extern "C" void PININT7_IRQHandler()   __attribute__((weak, alias("_default_handler")));

typedef void (* handler_t)();

extern "C" handler_t _vectors[];

__attribute__((section(".vectors")))
handler_t _vectors[] = {
    (handler_t)(&_estack),  // -16
    _start,                 // -15
    NMI_Handler,            // -14
    HardFault_Handler,      // -13
    0,                      // -12
    0,                      // -11
    0,                      // -10
    0,                      //  -9
    0,                      //  -8
    0,                      //  -7
    0,                      //  -6
    SVC_Handler,            //  -5
    PendSV_Handler,         //  -2
    SysTick_Handler,        //  -1
    SPI0_IRQHandler,        //   0
    SPI1_IRQHandler,        //   1
    0,                      //   2
    UART0_IRQHandler,       //   3
    UART1_IRQHandler,       //   4
    UART2_IRQHandler,       //   5
    0,                      //   6
    0,                      //   7
    I2C_IRQHandler,         //   8
    SCT_IRQHandler,         //   9
    MRT_IRQHandler,         //  10
    CMP_IRQHandler,         //  11
    WDT_IRQHandler,         //  12
    BOD_IRQHandler,         //  13
    0,                      //  14
    WKT_IRQHandler,         //  15
    0,                      //  16
    0,                      //  17
    0,                      //  18
    0,                      //  19
    0,                      //  20
    0,                      //  21
    0,                      //  22
    0,                      //  23
    PININT0_IRQHandler,     //  24
    PININT1_IRQHandler,     //  25
    PININT2_IRQHandler,     //  26
    PININT3_IRQHandler,     //  27
    PININT4_IRQHandler,     //  28
    PININT5_IRQHandler,     //  29
    PININT6_IRQHandler,     //  30
    PININT7_IRQHandler,     //  31
};
