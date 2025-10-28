#define F_CPU 16000000UL
#include "funsape/funsapeLibGlobalDefines.hpp"
#include "funsape/peripheral/funsapeLibUsart0.hpp"
#include "include/nrf24.h"
// Definições dos pinos - NRF24L01
#define CE_PIN  PB0
#define CSN_PIN PB1

#define BUTTON_DIR		DDRB
#define BUTTON_OUT		PORTB
#define BUTTON_IN		PINB
#define BUTTON_A		PB2
#define BUTTON_B		PB3
#define BUTTON_C		PB4

#define MOTOR_DIR		DDRC
#define MOTOR_OUT		PORTC
#define MOTOR_IN1		PC0
#define MOTOR_IN2		PC1
#define MOTOR_IN3		PC2
#define MOTOR_IN4		PC3

#define MOTOR_EN_DIR	DDRD
#define MOTOR_EN_OUT	PORD
#define MOTOR_ENA		PD5
#define MOTOR_ENB		PD6


// Botões NES
#define BTN_A      0x01
#define BTN_B      0x02
#define BTN_SELECT 0x04
#define BTN_START  0x08
#define BTN_UP     0x10
#define BTN_DOWN   0x20
#define BTN_LEFT   0x40
#define BTN_RIGHT  0x80


// Variáveis globais
volatile uint8_t cmd = 0;
uint8_t motor_speed = 127; // Velocidade padrão 50%
// Endereço para comunicação
const uint8_t rx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
typedef enum {
    MOV_STOP = 0,
    MOV_FORWARD,           // Ambos motores para frente
    MOV_BACKWARD,          // Ambos motores para trás
    MOV_TURN_LEFT,         // Giro no eixo: esq trás, dir frente
    MOV_TURN_RIGHT,        // Giro no eixo: esq frente, dir trás
    MOV_FORWARD_LEFT,      // Curva suave: dir frente, esq parado
    MOV_FORWARD_RIGHT,     // Curva suave: esq frente, dir parado
    MOV_BACKWARD_LEFT,     // Curva ré: dir trás, esq parado
    MOV_BACKWARD_RIGHT,    // Curva ré: esq trás, dir parado
    MOV_PIVOT_LEFT,        // Pivô: esq trás, dir frente (giro mais fechado)
    MOV_PIVOT_RIGHT,       // Pivô: esq frente, dir trás (giro mais fechado)
    MOV_DRIFT_LEFT,        // Derrapagem: esq trás lento, dir frente rápido
    MOV_DRIFT_RIGHT        // Derrapagem: esq frente rápido, dir trás lento
} movement_t;

typedef union {
    struct {
        bool_t      readData                    : 1;
        bool_t      alarm                       : 1;
        bool_t      menuAlarm                   : 1;
        bool_t      menuAjuste                  : 1;
        bool_t      reciveNumber                : 1;
        uint8_t     unusedFlags                 : 3;
    };
    uint8_t all;
} systemFlags_t;
volatile systemFlags_t flag;

// Controle dos motores (ponte H)
void set_motors(uint8_t left_dir, uint8_t right_dir) {
    // Motor A (esquerdo)
    if(left_dir == 1) { // Frente
        MOTOR_OUT |= (1 << MOTOR_IN1);
        MOTOR_OUT &= ~(1 << MOTOR_IN2);
    }
    else if(left_dir == 2) { // Trás
        MOTOR_OUT &= ~(1 << MOTOR_IN1);
        MOTOR_OUT |= (1 << MOTOR_IN2);
    }
    else { // Parar
        MOTOR_OUT &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2));
        // Desabilita o motor A
    }

    // Motor B (direito)
    if(right_dir == 1) { // Frente
        MOTOR_OUT |= (1 << MOTOR_IN3);
        MOTOR_OUT &= ~(1 << MOTOR_IN4);
    }
    else if(right_dir == 2) { // Trás
        MOTOR_OUT &= ~(1 << MOTOR_IN3);
        MOTOR_OUT |= (1 << MOTOR_IN4);
    }
    else { // Parar
        MOTOR_OUT &= ~((1 << MOTOR_IN3) | (1 << MOTOR_IN4));
    }
}

// typedef struct {
//     uint8_t left : 2;  // 2 bits para motor esquerdo
//     uint8_t right : 2; // 2 bits para motor direito
// } motor_cmd_t;

// void set_motors_compact(motor_cmd_t cmd) {
//     // Combina tudo em uma operação
//     MOTOR_OUT = (MOTOR_OUT & 0xC3) |        // Mantém outros bits
//                ((cmd.left & 1) << MOTOR_IN1) |     // Bit 0 -> IN1
//                ((cmd.left >> 1) << MOTOR_IN2) |    // Bit 1 -> IN2
//                ((cmd.right & 1) << MOTOR_IN3) |    // Bit 0 -> IN3
//                ((cmd.right >> 1) << MOTOR_IN4);    // Bit 1 -> IN4
// }


void movement(movement_t mov) {
    switch(mov) {
        case MOV_FORWARD:
            set_motors(1, 1); // Ambos para frente
            break;
        case MOV_BACKWARD:
            set_motors(2, 2); // Ambos para trás
            break;
        case MOV_TURN_LEFT:
            set_motors(2, 1); // Esquerdo trás, direito frente
            break;
        case MOV_TURN_RIGHT:
            set_motors(1, 2); // Esquerdo frente, direito trás
            break;
        case MOV_FORWARD_LEFT:
            set_motors(0, 1); // Apenas motor direito para frente
            break;
        case MOV_FORWARD_RIGHT:
            set_motors(1, 0); // Apenas motor esquerdo para frente
            break;
        case MOV_BACKWARD_LEFT:
            set_motors(0, 2); // Apenas motor direito para trás
            break;
        case MOV_BACKWARD_RIGHT:
            set_motors(2, 0); // Apenas motor esquerdo para trás
            break;
        case MOV_PIVOT_LEFT:
            set_motors(2, 1); // Mesmo que TURN_LEFT (para consistência)
            break;
        case MOV_PIVOT_RIGHT:
            set_motors(1, 2); // Mesmo que TURN_RIGHT (para consistência)
            break;
        case MOV_DRIFT_LEFT:
            set_motors(2, 1); // Mesmo que TURN_LEFT (velocidade controlada por PWM)
            break;
        case MOV_DRIFT_RIGHT:
            set_motors(1, 2); // Mesmo que TURN_RIGHT (velocidade controlada por PWM)
            break;
        default: // MOV_STOP
            set_motors(0, 0);
            break;
    }
}

void process_command(uint8_t command) {
    // Controle de velocidade (usando ENA/ENB depois)

    // Movimentos prioritários - combinações de botões
    if((command & BTN_UP) && (command & BTN_RIGHT)) {
        set_motors(1, 0); // Curva suave direita + frente
    }
    else if((command & BTN_UP) && (command & BTN_LEFT)) {
        set_motors(0, 1); // Curva suave esquerda + frente
    }
    else if((command & BTN_DOWN) && (command & BTN_RIGHT)) {
        set_motors(2, 0); // Curva suave direita + ré
    }
    else if((command & BTN_DOWN) && (command & BTN_LEFT)) {
        set_motors(0, 2); // Curva suave esquerda + ré
    }
    // Movimentos básicos
    else if(command & BTN_UP) {
        set_motors(1, 1); // Frente reto
    }
    else if(command & BTN_DOWN) {
        set_motors(2, 2); // Ré reto
    }
    else if(command & BTN_LEFT) {
        set_motors(2, 1); // Giro no eixo - esquerda
    }
    else if(command & BTN_RIGHT) {
        set_motors(1, 2); // Giro no eixo - direita
    }
    else {
        set_motors(0, 0); // Parar
    }
}



void test_movements_simple() {
    // Configurar stdout para UART
    printf("Starting motor test sequence..\r\n");

    // Teste 1: Frente por 2 segundos
    printf("MOV_FORWARD - Both motors forward\r\n");
    movement(MOV_FORWARD);
    _delay_ms(2000);

    // Teste 2: Parar brevemente
    printf("MOV_STOP - Stopping\r\n");
    movement(MOV_STOP);
    _delay_ms(500);

    // Teste 3: Ré por 2 segundos
    printf("MOV_BACKWARD - Both motors backward\r\n");
    movement(MOV_BACKWARD);
    _delay_ms(2000);

    // Teste 4: Parar brevemente
    printf("MOV_STOP - Stopping\r\n");
    movement(MOV_STOP);
    _delay_ms(500);

    // Teste 5: Girar esquerda por 1 segundo
    printf("MOV_TURN_LEFT - Left motor backward, right motor forward\r\n");
    movement(MOV_TURN_LEFT);
    _delay_ms(1000);

    // Teste 6: Girar direita por 1 segundo
    printf("MOV_TURN_RIGHT - Left motor forward, right motor backward\r\n");
    movement(MOV_TURN_RIGHT);
    _delay_ms(1000);

    // Teste 7: Parar
    printf("MOV_STOP - Test sequence completed\r\n");
    movement(MOV_STOP);
}

void print_command_description(uint8_t command) {
    if (command == 0) {
        printf("STOP\r\n");
        return;
    }

    // Combinações prioritárias
    if ((command & BTN_UP) && (command & BTN_RIGHT)) {
        printf("FORWARD_RIGHT (soft curve)\r\n");
    } else if ((command & BTN_UP) && (command & BTN_LEFT)) {
        printf("FORWARD_LEFT (soft curve)\r\n");
    } else if ((command & BTN_DOWN) && (command & BTN_RIGHT)) {
        printf("BACKWARD_RIGHT (soft curve)\r\n");
    } else if ((command & BTN_DOWN) && (command & BTN_LEFT)) {
        printf("BACKWARD_LEFT (soft curve)\r\n");
    } else if (command & BTN_UP) {
        printf("FORWARD\r\n");
    } else if (command & BTN_DOWN) {
        printf("BACKWARD\r\n");
    } else if (command & BTN_LEFT) {
        printf("TURN_LEFT\r\n");
    } else if (command & BTN_RIGHT) {
        printf("TURN_RIGHT\r\n");
    } else {
        printf("UNKNOWN COMMAND: 0x%02X\r\n", command);
    }
}

void test_command_sequence() {
    printf("Starting command sequence test...\r\n");

    // Vetor de comandos (simulando pressionamento de botões)
    uint8_t commands[] = {
        BTN_UP,                                 // Frente
        BTN_UP | BTN_RIGHT,                     // Frente + Direita (curva suave direita)
        BTN_UP | BTN_LEFT,                      // Frente + Esquerda (curva suave esquerda)
        BTN_DOWN,                               // Ré
        BTN_DOWN | BTN_RIGHT,                   // Ré + Direita (curva ré direita)
        BTN_DOWN | BTN_LEFT,                    // Ré + Esquerda (curva ré esquerda)
        BTN_LEFT,                               // Giro esquerda
        BTN_RIGHT,                              // Giro direita
        0,                                      // Parar
        BTN_UP,                                 // Frente novamente
        0                                       // Parar no final
    };

    int num_commands = sizeof(commands) / sizeof(commands[0]);

    for (int i = 0; i < num_commands; i++) {
        // Processa o comando
        process_command(commands[i]);

        // Imprime o comando atual
        print_command_description(commands[i]);

        // Aguarda 2 segundos para cada comando (exceto o último que para)
        _delay_ms(2000);
    }

    printf("Command sequence test completed!\r\n");
}

void motor_left_speed(uint8_t speed) {
    OCR0A = speed;  // MOTOR_ENA está no pino PD6 (OC0A)
}

void motor_right_speed(uint8_t speed) {
    OCR0B = speed;  // MOTOR_ENB está no pino PD5 (OC0B)
}
void pwm_init(void) {
    // Configurar Timer0 para PWM nos pinos OC0A (PD6) e OC0B (PD5)

    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS01);
}
int main(void) {
	// // Configura pinos do motor (PD2-PD7) como saída
	// MOTOR_DIR |= 0b11111100;
	// MOTOR_DIR |= (1 << MOTOR_IN1) | (1 << MOTOR_IN2) |
	// 	(1 << MOTOR_IN3) | (1 << MOTOR_IN4) |
	// 	(0 << MOTOR_ENA) | (0 << MOTOR_ENB);
	MOTOR_DIR |= (1 << MOTOR_IN1) | (1 << MOTOR_IN2) |
		(1 << MOTOR_IN3) | (1 << MOTOR_IN4);
	MOTOR_EN_DIR |= (1 << MOTOR_ENA) | (1 << MOTOR_ENB);


	usart0.setBaudRate(Usart0::BaudRate::BAUD_RATE_57600);
	usart0.enableTransmitter();
	usart0.enableReceiver();
	usart0.stdio();
	printf("UART configured!\r\n");

    // Configurar PWM para controle de velocidade
	pwm_init();
	motor_left_speed(motor_speed);
	motor_right_speed(motor_speed);

    // Inicializar NRF24L01
    spi_init();
    irq_init();
    nrf_init_rx_irq(rx_address);

    // Habilitar interrupções globais
    sei();

    printf("=== Car Control System Started ===\r\n");
    printf("NRF24L01 Initialized - RX Mode\r\n");
    printf("Address: 0x%02X%02X%02X%02X%02X\r\n",
           rx_address[0], rx_address[1], rx_address[2],
           rx_address[3], rx_address[4]);
    printf("Waiting for commands...\r\n\n");

	// MOTOR_OUT |= (1 << MOTOR_IN1) | (0 << MOTOR_IN2) |
		// (1 << MOTOR_IN3) | (0 << MOTOR_IN4);
	while (1) {
	if (flag.readData) {
		flag.readData = false;
		process_command(cmd);
	}
	}
	return 0;
}

// =============================================================================
// Interrupt Service Routines
// =============================================================================


ISR(INT0_vect) {
    uint8_t status;
    CSN_LOW();
    status = spi_tx(NOP); // Lê o registrador STATUS
    CSN_HIGH();

    // Verifica se há dados recebidos (RX_DR)
    if (status & 0x40) {
        uint8_t data;
        nrf_read_payload(&data, 1);
        cmd = data;
        flag.readData = true;

        // Limpa a flag RX_DR
        nrf_write_reg(STATUS, 0x40);

        printf("IRQ: Received 0x%02X\r\n", data);
    }

    // Limpa outras flags se necessário
    if (status & 0x10) { // MAX_RT
        nrf_write_reg(STATUS, 0x10);
        nrf_flush_tx();
        printf("IRQ: Max Retries\r\n");
    }
    if (status & 0x20) { // TX_DS
        nrf_write_reg(STATUS, 0x20);
        printf("IRQ: TX Success\r\n");
    }
}
