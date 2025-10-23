#ifndef NES_H
#define NES_H

#include <avr/io.h>
#include <util/delay.h>

// Pin definitions
#define NES_CLK   PD2
#define NES_LATCH PD3
#define NES1_DATA PD4
#define NES2_DATA PD5

void nes_init(void);
uint8_t nes_read(uint8_t data_pin);

#endif
