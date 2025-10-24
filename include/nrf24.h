#ifndef NRF24_H
#define NRF24_H

#include <avr/io.h>
#include <util/delay.h>

// Pins
#define IRQ PD2  // Digital 2
#define CE  PB1  // Digital 9
#define CSN PB2  // Digital 10
#define MOSI PB3 // Digital 11
#define MISO PB4 // Digital 12
#define SCK PB5  // Digital 13

// Macros
#define CE_HIGH()  PORTB |= (1 << CE)
#define CE_LOW()   PORTB &= ~(1 << CE)
#define CSN_HIGH() PORTB |= (1 << CSN)
#define CSN_LOW()  PORTB &= ~(1 << CSN)

// SPI setup
static inline void spi_init(void) {
    DDRB |= (1 << MOSI) | (1 << SCK) | (1 << CSN) | (1 << CE);
    DDRB &= ~(1 << MISO);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

static inline uint8_t spi_tx(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

// IRQ setup
static inline void irq_init(void) {
    DDRD &= ~(1 << IRQ);
    PORTD |= (1 << IRQ);
    EICRA |= (1 << ISC01); // Falling edge
    EIMSK |= (1 << INT0);  // Enable INT0
}

// nRF24 commands
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF

// nRF24 registers
#define CONFIG        0x00
#define EN_AA         0x01
#define RF_CH         0x05
#define RF_SETUP      0x06
#define STATUS        0x07
#define RX_ADDR_P0    0x0A
#define TX_ADDR       0x10
#define RX_PW_P0      0x11

// Register access
static inline void nrf_write_reg(uint8_t reg, uint8_t val) {
    CSN_LOW();
    spi_tx(W_REGISTER | (reg & 0x1F));
    spi_tx(val);
    CSN_HIGH();
}

static inline void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len) {
    CSN_LOW();
    spi_tx(W_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) spi_tx(buf[i]);
    CSN_HIGH();
}

static inline void nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
    CSN_LOW();
    spi_tx(R_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) buf[i] = spi_tx(NOP);
    CSN_HIGH();
}

// FIFO commands
static inline void nrf_flush_tx(void) {
    CSN_LOW(); spi_tx(FLUSH_TX); CSN_HIGH();
}

static inline void nrf_flush_rx(void) {
    CSN_LOW(); spi_tx(FLUSH_RX); CSN_HIGH();
}

// TX / RX payload
static inline void nrf_send_payload(const volatile uint8_t *data, uint8_t len) {
    CE_LOW();
    CSN_LOW();
    spi_tx(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) spi_tx(data[i]);
    CSN_HIGH();
    CE_HIGH();
    _delay_us(15);
    CE_LOW();
}

static inline void nrf_read_payload(uint8_t *data, uint8_t len) {
    CSN_LOW();
    spi_tx(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) data[i] = spi_tx(NOP);
    CSN_HIGH();
    nrf_flush_rx();
}

// TX and RX init

// TX mode init
static inline void nrf_init_tx(void) {
    CE_LOW();
    _delay_ms(5);
    nrf_write_reg(EN_AA, 0x00);
    nrf_write_reg(RF_CH, 76);
    nrf_write_reg(RF_SETUP, 0x06);
    nrf_write_reg(CONFIG, 0x0A);  // PWR_UP + TX mode
    nrf_flush_tx();
    CE_HIGH();
}

// RX mode init IRQ
void nrf_init_rx_irq(const uint8_t *rx_addr) {
    CE_LOW();
    _delay_ms(5);
    nrf_write_buf(RX_ADDR_P0, rx_addr, 5);
    nrf_write_reg(EN_AA, 0x00);
    nrf_write_reg(RF_CH, 76);
    nrf_write_reg(RF_SETUP, 0x06);
    nrf_write_reg(RX_PW_P0, 1);
    nrf_write_reg(CONFIG, 0b00001111); // PWR_UP + PRIM_RX + IRQ enabled
    nrf_flush_rx();
    CE_HIGH();
}

// Set TX address
static inline void nrf_set_tx_address(const uint8_t *addr) {
    CE_LOW();
    nrf_write_buf(TX_ADDR, addr, 5);
    nrf_write_buf(RX_ADDR_P0, addr, 5);
    CE_HIGH();
}

#endif
