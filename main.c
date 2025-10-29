/**
 * @file main.c
 * @brief Hoofdapplicatie voor de Xmega Kirby Image Cycler.
 * @author [Jouw Naam]
 * @date 29-10-2025
 *
 * Project dat 5 Xmega features gebruikt om te voldoen aan de eindopdracht:
 * 1. Grafisch LCD (ST7735)
 * 2. SPI (Hardware SPIC)
 * 3. Externe Interrupts (PORTD Knoppen)
 * 4. Timer/Counters (TCC0 voor debounce)
 * 5. Flash (PROGMEM voor 4x 40KB afbeeldingen)
 */

#define F_CPU 32000000UL // 32MHz (Eis)

#include <avr/io.h>
#include <avr/interrupt.h>
#include "system.h"
#include "st7735_driver.h"
#include "images.h" // Onze "lijm" header

// Globale 'flags' voor communicatie tussen ISRs en main()
static volatile bool g_update_display = true; // Start met tekenen
static volatile uint8_t g_current_image_index = 0;

/**
 * @brief ISR voor de knop (PORTD, INT0).
 * Start de debounce-timer.
 */
ISR(PORTD_INT0_vect) {
    // Schakel deze interrupt tijdelijk uit om dender te voorkomen
    PORTD.INTCTRL &= ~PORT_INT0LVL_gm; 
    
    // Reset en start de 50ms debounce-timer
    TCC0.CNT = 0;
    TCC0.CTRLA = TC_CLKSEL_DIV1024_gc; // Start timer
}

/**
 * @brief ISR voor de Timer TCC0 (Debounce).
 * Wordt 50ms na de knopdruk uitgevoerd.
 */
ISR(TCC0_OVF_vect) {
    // 1. Stop de timer
    TCC0.CTRLA = TC_CLKSEL_OFF_gc;
    
    // 2. Lees de knop. Is deze *nog steeds* ingedrukt?
    if (!(PORTD.IN & PIN0_bm)) {
        // JA: Knopdruk is geldig
        
        // Verhoog de index, met 'wrap-around'
        g_current_image_index++;
        if (g_current_image_index >= 4) {
            g_current_image_index = 0;
        }
        
        // Sein de main-loop dat het scherm ververst moet worden
        g_update_display = true;
    }
    
    // 3. Her-activeer de knop-interrupt
    PORTD.INTFLAGS = PORT_INT0IF_bm; // Clear de vlag
    PORTD.INTCTRL |= PORT_INT0LVL_LO_gc; // Zet interrupt weer aan
}


int main(void) {
    // 1. Initialisaties
    CLK_init_32MHz();       // Eis: 32MHz klok
    ST7735_init();          // Features: Grafisch LCD, SPI
    BTN_init();             // Feature: Externe Interrupts
    TMR_init_debounce();    // Feature: Timer/Counters
    
    // Globale interrupts inschakelen
    PMIC.CTRL |= PMIC_LOLVLEN_bm; // Low-level interrupts aanzetten
    sei();                        // Global interrupt enable
    
    // 2. Hoofdlus (Superloop)
    while (1) {
        
        // Moet het display bijgewerkt worden?
        if (g_update_display) {
            g_update_display = false; // Reset de vlag

            // Haal de pointer naar de juiste afbeelding op uit PROGMEM
            // pgm_read_word leest een 'word' (16-bit) uit het PROGMEM-geheugen
            const uint16_t* image_ptr = (const uint16_t*)pgm_read_word(
                &g_image_array[g_current_image_index]
            );

            // Stuur de afbeelding naar het display
            ST7735_draw_image_from_progmem(image_ptr);
        }
        
        // De CPU kan hier andere taken doen of in slaapstand gaan
        // (De interrupt breekt hier vanzelf in)
    }
}