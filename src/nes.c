#include "nes.h"

void nes_init(void) {
    DDRD |= (1 << NES_CLK) | (1 << NES_LATCH);        // outputs
    DDRD &= ~((1 << NES1_DATA) | (1 << NES2_DATA));   // inputs
    PORTD |= (1 << NES1_DATA) | (1 << NES2_DATA);     // pull-ups
}

uint8_t nes_read(uint8_t data_pin) {
    uint8_t i, state = 0;
    PORTD |= (1 << NES_LATCH);
    _delay_us(12);
    PORTD &= ~(1 << NES_LATCH);
    for (i = 0; i < 8; i++) {
        if (PIND & (1 << data_pin))
            state |= (1 << i);
        PORTD |= (1 << NES_CLK);
        _delay_us(6);
        PORTD &= ~(1 << NES_CLK);
        _delay_us(6);
    }
    return ~state; // active low buttons
}
