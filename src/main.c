#include <avr/io.h>
#include <avr/interrupt.h>
#include "nrf24.h"
#include "nes.h"

volatile uint8_t tx_flag = 0;
volatile uint8_t c1_state = 0, c2_state = 0;

ISR(TIMER0_COMPA_vect) {
    uint8_t n1 = nes_read(NES1_DATA);
    uint8_t n2 = nes_read(NES2_DATA);
    if (n1 != c1_state || n2 != c2_state) {
        c1_state = n1;
        c2_state = n2;
        tx_flag = 1;
    }
}

void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00); // /1024 prescaler
    OCR0A = 156;                        // ≈10 ms @16 MHz
    TIMSK0 = (1 << OCIE0A);
}

int main(void) {
    spi_init();
    nrf_init_tx();
    nes_init();
    timer0_init();
    sei();

    const uint8_t addr1[5] = {'C','A','R','1','A'};
    const uint8_t addr2[5] = {'C','A','R','2','B'};

    while (1) {
        if (tx_flag) {
            tx_flag = 0;
            nrf_set_tx_address(addr1);
            nrf_send_payload(&c1_state, 1);
            nrf_set_tx_address(addr2);
            nrf_send_payload(&c2_state, 1);
        }
    }
}
