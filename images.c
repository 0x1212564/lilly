#include "images.h"

/**
 * @file images.c
 * @brief Definieert de PROGMEM array van pointers naar de afbeeldingen.
 */

// Deze array wijst naar de 4 afbeeldingen die in de andere .c bestanden staan
const uint16_t* const g_image_array[4] PROGMEM = {
    kirby_image_1,
    kirby_image_2,
    kirby_image_3,
    kirby_image_4
};