#ifndef NES_H
#define NES_H

#include <avr/io.h>
#include <util/delay.h>

// Pin definitions
#define NES_CLK   PD4  // Digital 4
#define NES_LATCH PD5  // Digital 5
#define NES1_DATA PD6 // Digital 6
#define NES2_DATA PD7 // Digital 7

void nes_init(void);
uint8_t nes_read(uint8_t data_pin);

#endif
