#ifndef SYSTEM_H_
#define SYSTEM_H_

/**
 * @file system.h
 * @brief Declaraties voor systeem-initialisatiefuncties.
 */

void CLK_init_32MHz(void);
void BTN_init(void);
void TMR_init_debounce(void);

#endif // SYSTEM_H_