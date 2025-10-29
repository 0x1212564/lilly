#include <avr/io.h>
#include "system.h"

/**
 * @file system.c
 * @brief Implementatie van klok, knop en timer-debounce.
 */

/**
 * @brief Stelt de systeemklok in op 32MHz m.b.v. de externe oscillator en PLL.
 * @note Eis: fCPU = 32MHz
 */
void CLK_init_32MHz(void) {
    // 1. Externe 8MHz oscillator inschakelen (ervan uitgaande dat HvA board 8MHz kristal heeft)
    OSC.XOSCTRL = OSC_XOSCSEL_8MHZ_gc | OSC_XOSCEN_bm;
    while (!(OSC.STATUS & OSC_XOSCRDY_bm)); // Wacht tot oscillator stabiel is

    // 2. PLL instellen: 32MHz = 8MHz (bron) * 4 (factor)
    OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 4; // Bron = Ext. Osc, Factor = 4
    OSC.CTRL |= OSC_PLLEN_bm;             // PLL inschakelen
    while (!(OSC.STATUS & OSC_PLLRDY_bm)); // Wacht tot PLL stabiel is

    // 3. Systeemklok instellen op PLL
    CCP = CCP_IOREG_gc; // Configuratiebeveiliging uitschakelen
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}

/**
 * @brief Initialiseert de knop op PORTD.0 met een interrupt.
 * @note Feature: Externe Interrupts
 */
void BTN_init(void) {
    PORTD.DIRCLR = PIN0_bm; // Pin 0 als input

    // Interne pull-up weerstand inschakelen en interrupt op 'falling edge'
    PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; 
    
    // Interrupt 0 instellen voor PORTD
    PORTD.INT0MASK = PIN0_bm; // Interrupt 0 op Pin 0
    PORTD.INTCTRL = PORT_INT0LVL_LO_gc; // Low-level interrupt
}

/**
 * @brief Initialiseert Timer TCC0 voor een 50ms debounce interrupt.
 * @note Feature: Timer/Counters
 */
void TMR_init_debounce(void) {
    // Klokbron instellen op 32MHz / 1024 = 31250 Hz
    TCC0.CTRLA = TC_CLKSEL_DIV1024_gc; 
    
    // PER (Period) instellen voor 50ms (0.05s)
    // Counts = 31250 Hz * 0.05s = 1562.5
    // We ronden af naar 1563
    TCC0.PER = 1563; 
    
    // Overflow interrupt instellen op low-level
    TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
    
    // De timer staat nu klaar, maar wordt pas gestart in de knop-ISR.
}