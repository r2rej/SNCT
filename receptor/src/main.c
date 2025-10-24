#include <avr/io.h>
#include <avr/interrupt.h>
#include "nrf24.h"

#define CAR_ADDR ((uint8_t[]){'C','A','R','1','A'})

volatile uint8_t rx_flag = 0;
volatile uint8_t ctrl;

// IRQ interrupt handler
ISR(INT0_vect) {
    rx_flag = 1;
}

int main(void) {
    spi_init();
    nrf_init_rx_irq(CAR_ADDR);
    irq_init();
    sei();

    DDRB |= (1 << PB0);

    while (1) {
        if (rx_flag) {
            rx_flag = 0;
            nrf_read_payload((uint8_t *)&ctrl, 1);
            PORTB ^= (1 << PB0); // toggle LED on packet
        }
    }
}
