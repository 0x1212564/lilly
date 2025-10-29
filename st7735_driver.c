#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "st7735_driver.h"

/**
 * @file st7735_driver.c
 * @brief Hardware-specifieke driver voor het ST7735 TFT display via SPI.
 * @note Feature: SPI, Feature: Grafisch LCD
 */

// Defineer de pinnen voor CS, DC, en RES (volgens het schema)
#define CS_PORT     PORTC
#define DC_PORT     PORTC
#define RES_PORT    PORTC

#define CS_PIN      PIN4_bm
#define DC_PIN      PIN3_bm
#define RES_PIN     PIN2_bm

// Macro's voor pin-controle
#define CS_LOW()    (CS_PORT.OUTCLR = CS_PIN)
#define CS_HIGH()   (CS_PORT.OUTSET = CS_PIN)
#define DC_LOW()    (DC_PORT.OUTCLR = DC_PIN) // Command
#define DC_HIGH()   (DC_PORT.OUTSET = DC_PIN) // Data
#define RES_LOW()   (RES_PORT.OUTCLR = RES_PIN)
#define RES_HIGH()  (RES_PORT.OUTSET = RES_PIN)

// ST7735 Commando's
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_NORON   0x13

// Schermresolutie
#define WIDTH  128
#define HEIGHT 160


/**
 * @brief Initialiseert de SPIC module op 16MHz (fCPU/2).
 */
static void SPI_init(void) {
    // CS, DC, RES als output
    CS_PORT.DIRSET  = CS_PIN;
    DC_PORT.DIRSET  = DC_PIN;
    RES_PORT.DIRSET = RES_PIN;
    
    // SPI pinnen (MOSI, SCK) als output
    PORTC.DIRSET = PIN5_bm | PIN7_bm; 
    
    // SPI instellen: Master, Mode 0, fCPU/2 = 16MHz, enabled
    // 32MHz / 2 = 16MHz kloksnelheid. Snelst mogelijk.
    SPIC.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_CLK2X_bm | SPI_PRESCALER_DIV4_gc; 
}

/**
 * @brief Verstuurt één byte data via SPI.
 * @param data Het te versturen byte.
 */
static void SPI_write(uint8_t data) {
    SPIC.DATA = data;
    while (!(SPIC.STATUS & SPI_IF_bm)); // Wacht tot transfer voltooid is
}

/**
 * @brief Verstuurt een commando naar de ST7735.
 * @param cmd Het commando-byte.
 */
static void ST7735_write_cmd(uint8_t cmd) {
    DC_LOW();  // Command-mode
    CS_LOW();
    SPI_write(cmd);
    CS_HIGH();
}

/**
 * @brief Verstuurt één byte data naar de ST7735.
 * @param data Het data-byte.
 */
static void ST7735_write_data(uint8_t data) {
    DC_HIGH(); // Data-mode
    CS_LOW();
    SPI_write(data);
    CS_HIGH();
}

/**
 * @brief Stelt het 'teken-venster' in op het display.
 * @param x1 Start X-coördinaat
 * @param y1 Start Y-coördinaat
 * @param x2 Eind X-coördinaat
 * @param y2 Eind Y-coördinaat
 */
static void ST7735_set_window(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    ST7735_write_cmd(ST7735_CASET); // Column Address Set
    ST7735_write_data(0x00);
    ST7735_write_data(x1);
    ST7735_write_data(0x00);
    ST7735_write_data(x2);

    ST7735_write_cmd(ST7735_RASET); // Row Address Set
    ST7735_write_data(0x00);
    ST7735_write_data(y1);
    ST7735_write_data(0x00);
    ST7735_write_data(y2);
}

/**
 * @brief Initialiseert het ST7735 display.
 */
void ST7735_init(void) {
    SPI_init(); // Initialiseer de SPI hardware

    // Hardware reset
    RES_LOW();
    _delay_ms(100);
    RES_HIGH();
    _delay_ms(100);

    // Initialisatie-sequentie
    ST7735_write_cmd(ST7735_SWRESET); // 1: Software reset
    _delay_ms(150);
    ST7735_write_cmd(ST7735_SLPOUT);  // 2: Out of sleep mode
    _delay_ms(255);
    
    // 3: Color mode
    ST7735_write_cmd(ST7735_COLMOD);
    ST7735_write_data(0x05); // 16-bit (RGB565)
    _delay_ms(10);
    
    // 4: Memory Access Control (oriëntatie)
    ST7735_write_cmd(ST7735_MADCTL);
    ST7735_write_data(0xC0); // 0xC0 = MX, MY, RGB-order. Pas dit aan als je kleuren/oriëntatie vreemd is!
                             // Probeer 0x00, 0x60, 0xA0 etc.
    
    // 5: Normal display mode
    ST7735_write_cmd(ST7735_NORON);
    _delay_ms(10);
    
    // 6: Display on
    ST7735_write_cmd(ST7735_DISPON);
    _delay_ms(100);
}

/**
 * @brief Tekent een volledige afbeelding (128x160) vanuit PROGMEM.
 * @note Feature: Flash (PROGMEM)
 * @param img_data Pointer naar de afbeelding-array in PROGMEM.
 */
void ST7735_draw_image_from_progmem(const uint16_t* img_data) {
    // 1. Zet venster op volledig scherm
    ST7735_set_window(0, 0, WIDTH - 1, HEIGHT - 1);
    
    // 2. Stuur 'RAM Write' commando
    ST7735_write_cmd(ST7735_RAMWR);
    
    // 3. Loop door alle 20480 pixels
    DC_HIGH(); // Zet in Data-mode voor de hele transfer
    CS_LOW();  // Zet Chip Select laag voor de hele transfer
    
    for (uint32_t i = 0; i < (WIDTH * HEIGHT); i++) {
        // Haal 16-bit pixel op uit Flash-geheugen
        uint16_t pixel = pgm_read_word(&img_data[i]);
        
        // Stuur de 16-bit pixel (in 2 bytes)
        SPI_write(pixel >> 8);   // High Byte
        SPI_write(pixel & 0xFF); // Low Byte
    }
    
    CS_HIGH(); // Voltooi de transfer
}