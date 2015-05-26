
extern "C" void main();

#include <sysctl.h>
#include <pin.h>
#include <uart.h>
#include <timer.h>
#include <interrupt.h>

void
callback(void)
{
}

void
main()
{
    Sysctl::init_24MHz();

    SWDIO.disable();
    SWCLK.disable();
    RST.disable();

    // feed system clock out on P0_3 at 1:10 for checking purposes.
    Sysctl::enable_clkout(P0_3, Sysctl::CLKOUTSRC_MAINSYSCLK, 10);

    // start a timer
    SYSCTL_MRT.clock(true);
    Timer0.configure(callback, 24000, true);

    // mess around with a GPIO
    P0_2.configure(Pin::Output, Pin::PushPull);
    P0_2.clear();
    P0_2.set();
    P0_2 << 1 << 1 << 0;
    P0_2.toggle();
    P0_2.set(0);
    P0_2.clear();
  
    // print some stuff
    UART0_TXD.claim_pin(P0_4);
    UART0_RXD.claim_pin(P0_0);

    UART0.configure(57600);
    UART0 << 'X' << "test string";
    for (;;) {
        UART0.send('A');
        P0_2.toggle();
    }

}
