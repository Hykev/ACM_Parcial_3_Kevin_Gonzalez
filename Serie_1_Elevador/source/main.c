#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32l053xx.h"
#include "core_cm0plus.h" // ya viene con CMSIS

// -x-x-x-x- Definición de pines elevador -x-x-x-x-

// 7-seg: leds = a,b,c,d,e,g → PB0–PB5 AZUL
//        displays = PC0–PC3 NEGRO


// LCD:    D4–D7 = PC4–PC7 AQUA
//         E  = PB10 VERDE
//         RS = PB11 VERDE
//         RW = GND  (solo escritura)
//         VO = potenciómetro 10kΩ entre VCC y GND (ajusta contraste)
//         VSS = GND
//         VDD = 5V o 3.3V (según módulo)
//         A (LED+) = VCC (con resistencia 100–220Ω)
//         K (LED–) = GND

// Buzzer: PA8 CAFE
//         → Conectado mediante transistor NPN (2N2222):
//            - Base: resistor 1kΩ desde PA8
//            - Emisor: a GND
//            - Colector: al pin negativo (–) del buzzer
//            - Pin positivo (+) del buzzer: a +5 V

// Motor elevador: IN1–IN4 = PB12–PB15 BLANCO

// Motor puerta:   IN1–IN4 = PC8–PC11 AMARILLO

// Keypad: columnas (piso 1–3+A) = PA0, PA1, PA4, PA10  → entradas con pull-up ROJO
//         fila (común) = PA7 → salida (controlada por software) NARANJA

// Switch impresora: S = PB9

// Botones para llamar al piso:
//         Piso 1 = PA6 (EXTI6)
//         Piso 2 = PA9 (EXTI9)
//         Piso 3 = PC12 (EXTI12)

// Leds de cada piso
//         Piso 1 = PB6
//         Piso 2 = PB7
//         Piso 3 = PB8

// USART2: TX = PA2, RX = PA3

// -x-x-x-x- Funciones prototipo -x-x-x-x-
// --- Prototipos (ISR lógicas y helpers) ---
void isr_display_digits_time_multiplexing(void);
void isr_keypad_read(void);
void isr_display_animation(void);
void isr_elevator_logic(void);
void isr_lcd_update(void);

void isr_elevator_motor_movement(void);
void isr_door_motor_movement(void);
void isr_buzzer_beep(void);

void push1_button_isr(void);
void push2_button_isr(void);
void push3_button_isr(void);

void door_open(void);
void door_close(void);
void move_queue(void);

// LCD helpers
void lcd_send_nibble(uint8_t nibble);
void lcd_cmd(uint8_t cmd);
void escribir_caracter_lcd(uint8_t caracter);
void lcd_init(void);
void lcd_pulso_E(void);
void escribir_texto_lcd(char *mensaje);

// USART2
void usart2_init(uint32_t baud);
void usart2_putc(char c);
void usart2_write(const char *s);
void usart2_write_line(const char *s);
void System_Init(void);



// -x-x-x-x- Variables Globales -x-x-x-x-
// Logica elevador
volatile uint8_t elevator_floor = 1; // Piso actual del elevador
volatile uint8_t next_floors[3] = {0,0,0};   // Piso objetivo

// buzzer
volatile uint8_t buzzer_flag = 0; // Bandera para activar el buzzer
volatile uint16_t buzzer_beep_counter = 0; // Contador para la duración del beep
volatile uint8_t buzzer_last_state = 0; // Ultimo estado del buzzer

// Motores
volatile uint8_t elevator_moving = 0; // Estado de movimiento del elevador (0: detenido, 1: arriba, 2: abajo)

volatile uint8_t elevator_motor_right_fsm = 1; // FSM motor elevador
volatile uint16_t elevator_motor_right_counter = 0; // contador

volatile uint8_t elevator_motor_left_fsm = 1; // FSM motor elevador
volatile uint16_t elevator_motor_left_counter = 0; // contador

// Puerta
volatile uint8_t door_moving = 0; // Estado de movimiento de la puerta (0: detenido, 1: abriendo, 2: esperando 3: cerrando)

volatile uint8_t door_motor_right_fsm = 1; // FSM motor puerta
volatile uint16_t door_motor_right_counter = 0; // contador

volatile uint8_t door_motor_left_fsm = 1; // FSM motor puerta
volatile uint16_t door_motor_left_counter = 0; // contador

volatile uint16_t door_motor_waiting_counter = 0; // contador

// Displays
volatile uint8_t display_fsm = 0;

// LCD
volatile uint8_t lcd_update_flag = 0;
char status_message[32] = "Esperando...";
char queue_message[32] = "Cola: 0, 0, 0";

volatile uint8_t led_test = 0;

    // -x-x-x-x- Init -x-x-x-x-
void System_Init(void)
{
    /* ---------- SysTick OFF (por si el template lo dejó activo) ---------- */
    SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;

    /* ---------- Reloj: HSI16 como SYSCLK, sin prescalers; MSI/PLL OFF ---------- */
    RCC->CR   |=  (1u << 0);                             // HSI16 ON
    while(!(RCC->CR & (1u << 1)));                       // HSIRDY
    RCC->CFGR &= ~((0x3u<<0) | (0xFu<<4) | (0x7u<<8));   // limpia SW, HPRE, PPRE
    RCC->CFGR |=  (0x1u << 0);                           // SW=01 (HSI16)
    while(((RCC->CFGR >> 2) & 0x3u) != 0x1u);            // espera SWS=01
    RCC->CR   &= ~(1u << 8);                             // MSION=0
    RCC->CR   &= ~(1u << 24);                            // PLLON=0
    FLASH->ACR |=  (1u<<8);                              // PRFTEN
    FLASH->ACR &= ~(1u<<0);                              // LATENCY=0 (16MHz)

    /* ---------- Clocks periféricos base ---------- */
    RCC->IOPENR  |= (1u<<0) | (1u<<1) | (1u<<2);         // GPIOA/B/C
    RCC->APB2ENR |= (1u<<0);                             // SYSCFG

    /* =========================
     *  GPIO (tus pines)
     * ========================= */

    // 7-seg: PB0..PB5 salida
    GPIOB->MODER &= ~0x00000FFFu;
    GPIOB->MODER |=  0x00000555u;

    // 7-seg: PC0..PC3 salida
    GPIOC->MODER &= ~0x000000FFu;
    GPIOC->MODER |=  0x00000055u;

    // LCD: PC4..PC7 salida
    GPIOC->MODER &= ~0x0000FF00u;
    GPIOC->MODER |=  0x00005500u;

    // LCD: PB10 (E) y PB11 (RS) salida
    GPIOB->MODER &= ~0x00F00000u;
    GPIOB->MODER |=  0x00500000u;

    // Motor elevador: PB12..PB15 salida
    GPIOB->MODER &= ~0xFF000000u;
    GPIOB->MODER |=  0x55000000u;

    // Motor puerta: PC8..PC11 salida
    GPIOC->MODER &= ~0x00FF0000u;
    GPIOC->MODER |=  0x00550000u;

    // Buzzer: PA8 salida
    GPIOA->MODER &= ~0x00030000u;
    GPIOA->MODER |=  0x00010000u;

    // LEDs PB6..PB8 salida
    GPIOB->MODER &= ~0x0003F000u;
    GPIOB->MODER |=  0x00015000u;

    // Keypad: PA0,PA1,PA4 entradas pull-up; PA15 salida (fila) = LOW
    GPIOA->MODER &= ~((0x3u<<(0*2)) | (0x3u<<(1*2)) | (0x3u<<(4*2)));
    GPIOA->PUPDR &= ~((0x3u<<(0*2)) | (0x3u<<(1*2)) | (0x3u<<(4*2)));
    GPIOA->PUPDR |=  ((0x1u<<(0*2)) | (0x1u<<(1*2)) | (0x1u<<(4*2))); // pull-up
    GPIOA->MODER &= ~(0x3u << (15*2));
    GPIOA->MODER |=  (0x1u << (15*2)); // PA15 salida
    GPIOA->BSRR   =  (1u << (15 + 16)); // PA15=0

    // Botones externos: PA6,PA7,PC12 entradas pull-up
    GPIOA->MODER &= ~((0x3u<<(6*2)) | (0x3u<<(7*2)));
    GPIOA->PUPDR &= ~((0x3u<<(6*2)) | (0x3u<<(7*2)));
    GPIOA->PUPDR |=  ((0x1u<<(6*2)) | (0x1u<<(7*2)));
    GPIOC->MODER &= ~(0x3u<<(12*2));
    GPIOC->PUPDR &= ~(0x3u<<(12*2));
    GPIOC->PUPDR |=  (0x1u<<(12*2));

    /* =========================
     *  USART2 (logs)
     * ========================= */
    usart2_init(115200);

    /* =========================
     *  EXTI (orden correcto)
     * ========================= */
    // Mapear líneas primero
    RCC->APB2ENR |= (1u<<0); // SYSCFG
    // EXTI0<-PA0, EXTI1<-PA1
    SYSCFG->EXTICR[0] &= ~((0xFu<<0) | (0xFu<<4));
    // EXTI4<-PA4, EXTI6<-PA6, EXTI7<-PA7
    SYSCFG->EXTICR[1] &= ~((0xFu<<0) | (0xFu<<8) | (0xFu<<12));
    // EXTI12<-PC12
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(0xFu<<0)) | (0x2u<<0);
    // EXTI13<-PC13 (botón usuario)
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(0xFu<<4)) | (0x2u<<4);

    // Limpiar pendientes ANTES
    EXTI->PR  = (1u<<0)|(1u<<1)|(1u<<4)|(1u<<6)|(1u<<7)|(1u<<12)|(1u<<13);

    // Unmask + flanco bajada
    EXTI->IMR  |= (1u<<0)|(1u<<1)|(1u<<4)|(1u<<6)|(1u<<7)|(1u<<12)|(1u<<13);
    EXTI->FTSR |= (1u<<0)|(1u<<1)|(1u<<4)|(1u<<6)|(1u<<7)|(1u<<12)|(1u<<13);

    // NVIC para EXTI
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    /* =========================
     *  TIMERS (higiene: UG, limpiar SR, limpiar NVIC pending)
     * ========================= */

    // TIM21 @1 kHz (multiplex + keypad/heartbeat)
    RCC->APB2ENR |= (1u<<2);
    TIM21->CR1  = 0;
    TIM21->PSC  = 16000 - 1;    // 16MHz/16000 = 1kHz
    TIM21->ARR  = 1 - 1;        // update cada 1 tick
    TIM21->EGR  = 1u;           // UG
    TIM21->SR   = 0;            // limpia flags
    TIM21->DIER = 1u;           // UIE
    NVIC_ClearPendingIRQ(TIM21_IRQn);
    NVIC_EnableIRQ(TIM21_IRQn);
    TIM21->CR1 |= 1u;           // CEN

    // TIM22 @100 Hz (animación + lógica + LCD)
    RCC->APB2ENR |= (1u<<5);
    TIM22->CR1  = 0;
    TIM22->PSC  = 16000 - 1;    // 1 kHz base
    TIM22->ARR  = 10 - 1;       // 100 Hz
    TIM22->EGR  = 1u;
    TIM22->SR   = 0;
    TIM22->DIER = 1u;
    NVIC_ClearPendingIRQ(TIM22_IRQn);
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= 1u;

    // TIM2 @1 kHz (motores + buzzer)
    RCC->APB1ENR |= (1u<<0);
    TIM2->CR1  = 0;
    TIM2->PSC  = 16000 - 1;     // 1 kHz
    TIM2->ARR  = 1 - 1;         // update por tick
    TIM2->EGR  = 1u;
    TIM2->SR   = 0;
    TIM2->DIER = 1u;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= 1u;

    /* =========================
     *  Estados iniciales seguros
     * ========================= */
    // 7-seg off
    GPIOB->BSRR = (0x3Fu << 16);
    GPIOC->BSRR = 0x0F;
    // Motores off
    GPIOB->BSRR = (0xFu << (12+16));
    GPIOC->BSRR = (0xFu << (8+16));
    // Buzzer off
    GPIOA->BSRR = (1u << (8+16));
    // LEDs off
    GPIOB->BSRR = ((1u<<6)|(1u<<7)|(1u<<8)) << 16;

    /* ---------- Global IRQ y LCD init (sin delays grandes) ---------- */
    __enable_irq();

    lcd_init();
    lcd_cmd(0x80);                    // pos 0
    escribir_texto_lcd("Hola LCD");   // prueba rápida
}



// -x-x-x-x- Main e Init -x-x-x-x-

int main(void) {
    // Inicializaciones de periféricos
    System_Init();

    while (1) {
    }
}

// -x-x-x-x-x- Funciones Interrupciones -x-x-x-x-
void TIM21_IRQHandler(void)
{
    if (TIM21->SR & 1u) {
        // heartbeat en PB6
        GPIOB->ODR ^= (1u<<6);
        isr_display_digits_time_multiplexing(); // lo tuyo
        TIM21->SR = 0;
    }
}

// TIM22: 100 Hz (animación + lógica + LCD)
void TIM22_IRQHandler(void)
{
    if (TIM22->SR & 1u) {
        isr_display_animation();
        isr_elevator_logic();
        isr_lcd_update();
        TIM22->SR = 0;    // <- limpiar UIF
    }
}

// TIM2: 1 kHz (motores + buzzer)
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 1u) {
        isr_elevator_motor_movement();
        isr_door_motor_movement();
        isr_buzzer_beep();
        TIM2->SR = 0;     // <- limpiar UIF
    }
}

// EXTI: limpia PR y despacha si necesitas colas/banderas
static inline void _debounce_us(volatile int n){ while(n--) __asm__("nop"); }
void EXTI0_1_IRQHandler(void)
{
    if (EXTI->PR & (1u<<0)) {
        GPIOB->ODR ^= (1u<<6);         // TEST: PB6
        _debounce_us(400);
        if ((GPIOA->IDR & (1u<<0)) == 0) {
            if (next_floors[0] == 0) next_floors[0] = 1;
            else if (next_floors[1] == 0 && next_floors[0] != 1) next_floors[1] = 1;
            else if (next_floors[2] == 0 && next_floors[1] != 1) next_floors[2] = 1;
            lcd_update_flag = 1;
        }
        EXTI->PR = (1u<<0);            // limpia PR escribiendo 1
    }

    if (EXTI->PR & (1u<<1)) {
        GPIOB->ODR ^= (1u<<6);         // TEST: PB6
        _debounce_us(400);
        if ((GPIOA->IDR & (1u<<1)) == 0) {
            if (next_floors[0] == 0) next_floors[0] = 2;
            else if (next_floors[1] == 0 && next_floors[0] != 2) next_floors[1] = 2;
            else if (next_floors[2] == 0 && next_floors[1] != 2) next_floors[2] = 2;
            lcd_update_flag = 1;
        }
        EXTI->PR = (1u<<1);
    }
}


void EXTI4_15_IRQHandler(void)
{
    // Botón usuario PC13: parpadea PB7 para ver que entra
    if (EXTI->PR & (1u<<13)) {
        GPIOB->ODR ^= (1u<<7);
        EXTI->PR = (1u<<13); // limpiar escribiendo 1
    }

    // Resto de tu despacho (keypad y pushes) tal como lo tienes:
    if (EXTI->PR & (1u<<4))  { /* ... */ EXTI->PR = (1u<<4); }
    if (EXTI->PR & (1u<<6))  { push1_button_isr(); EXTI->PR = (1u<<6); }
    if (EXTI->PR & (1u<<7))  { push2_button_isr(); EXTI->PR = (1u<<7); }
    if (EXTI->PR & (1u<<12)) { push3_button_isr(); EXTI->PR = (1u<<12); }
}

// -x-x-x-x- Funciones Motor Elevador -x-x-x-x-
// arriba
void first_to_second_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 64;
    elevator_moving = 1;
    //usart2_write_line("Piso: 1 a 2.");
}

void second_to_third_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 64;
    elevator_moving = 1;
    //usart2_write_line("Piso: 2 a 3.");
}

void first_to_third_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 128;
    elevator_moving = 1;
    //usart2_write_line("Piso: 1 a 3.");
}

// abajo
void third_to_second_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 64;
    elevator_moving = 2;
    //usart2_write_line("Piso: 3 a 2.");
}
void second_to_first_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 64;
    elevator_moving = 2;
    //usart2_write_line("Piso: 2 a 1.");
}
void third_to_first_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 128;
    elevator_moving = 2;
    //usart2_write_line("Piso: 3 a 1.");
}

void buzzer_beep(void) {
    // Configurar pin del buzzer como salida
        buzzer_beep_counter = 1000; // Duración del beep en ms
        buzzer_flag = 1; // Beep al llegar al piso
        buzzer_last_state = 0; // Duración del beep en ms
        //usart2_write_line("Buzzer sonando...");
}

void isr_buzzer_beep(void) {
    if (buzzer_flag == 1) {
        buzzer_beep_counter--; // Duración del beep en ms
        if (buzzer_beep_counter == 0) {
            buzzer_flag = 0; // Apagar el buzzer
            GPIOA->BSRR |= (1 << (8 + 16)); // Apagar pin del buzzer
        }

        if (buzzer_last_state == 0) {
            GPIOA->BSRR |= (1 << 8); // Encender pin del buzzer
        } else {
            GPIOA->BSRR |= (1 << (8 + 16)); // Apagar pin del buzzer
        }
        buzzer_last_state ^= 1;
    }
}

void isr_elevator_motor_movement(void) {
    switch (elevator_moving) {
        case 0: // Detenido
            // No hacer nada
            break;
        case 1: // Subiendo
            elevator_motor_right_counter--;
            strcpy(status_message, "Subiendo...");
            lcd_update_flag = 1;
            //usart2_write_line("Subiendo...");

            if (elevator_motor_right_counter == 0) {
                move_queue(); // Mover la cola de pisos
                door_open(); // Abrir puerta al llegar al piso
                buzzer_beep(); // Beep al llegar al piso
            }
            GPIOB->BSRR |= (0x0F << (12+16)); // Apagar PB12–PB15
            switch(elevator_motor_right_fsm) {
                case 1:
                    GPIOB->BSRR |= (0b0001 << 12); // Activar bobina 1
                    elevator_motor_right_fsm = 2;
                    break;
                case 2:
                    GPIOB->BSRR |= (0b0010 << 12); // Activar bobina 2
                    elevator_motor_right_fsm = 3;
                    break;
                case 3:
                    GPIOB->BSRR |= (0b0100 << 12); // Activar bobina 3
                    elevator_motor_right_fsm = 4;
                    break;
                case 4:
                    GPIOB->BSRR |= (0b1000 << 12); // Activar bobina 4
                    elevator_motor_right_fsm = 1;
                    break;
                default:
                    GPIOB->BSRR |= (0b0001 << 12); // Activar bobina 1
                    elevator_motor_right_fsm = 2;
                    break;
                }
            break;
        case 2: // Bajando
            elevator_motor_left_counter--;
            strcpy(status_message, "Bajando...");
            lcd_update_flag = 1;
            //usart2_write_line("Bajando...");

            if (elevator_motor_left_counter == 0) {
                move_queue(); // Mover la cola de pisos
                door_open(); // Abrir puerta al llegar al piso
                buzzer_beep(); // Beep al llegar al piso
            }
            GPIOB->BSRR |= (0x0F << (12+16)); // Apagar PB12–PB15
            switch(elevator_motor_left_fsm) {
                case 1:
                    GPIOB->BSRR |= (0b0001 << 12); // Activar bobina 1
                    elevator_motor_left_fsm = 2;
                    break;
                case 2:
                    GPIOB->BSRR |= (0b1000 << 12); // Activar bobina 4
                    elevator_motor_left_fsm = 3;
                    break;
                case 3:
                    GPIOB->BSRR |= (0b0100 << 12); // Activar bobina 3
                    elevator_motor_left_fsm = 4;
                    break;
                case 4:
                    GPIOB->BSRR |= (0b0010 << 12); // Activar bobina 2
                    elevator_motor_left_fsm = 1;
                    break;
                default:
                    GPIOB->BSRR |= (0b0001 << 12); // Activar bobina 1
                    elevator_motor_left_fsm = 2;
                    break;
            }
            break;
        default:
            elevator_moving = 0; // Actualizar el estado de movimiento del elevador
            break;
    }
}

// -x-x-x-x- Funciones Motor Puerta -x-x-x-x-
// izquierda
void door_open(void) {
    door_motor_right_fsm = 1;
    door_motor_right_counter = 64;
    door_moving = 1; // Cambiar estado a abriendo
    strcpy(status_message, "Abriendo puerta");
    lcd_update_flag = 1;
    //usart2_write_line("Abriendo puerta");
}
// derecha
void door_close(void) {
    door_motor_left_fsm = 1;
    door_motor_left_counter = 64;
    door_moving = 3; // Cambiar estado a cerrando
    strcpy(status_message, "Cerrando puerta");
    lcd_update_flag = 1;
    //usart2_write_line("Cerrando puerta");
}

void isr_door_motor_movement(void) {
        switch (door_moving) {
            case 0: // Detenido
                break;

            case 1: // Abriendo
                door_motor_right_counter--;
                if (door_motor_right_counter == 0) {
                    door_moving = 2; // Cambiar a esperando
                    door_motor_waiting_counter = 1000; // Establecer el contador de espera (1000 ms)
                }
                GPIOC->BSRR |= (0x0F << (8+16)); // Apagar PC8–PC11
                switch(door_motor_right_fsm) {
                    case 1:
                        GPIOC->BSRR |= (0b0001 << 8); // Activar bobina 1
                        door_motor_right_fsm = 2;
                        break;
                    case 2:
                        GPIOC->BSRR |= (0b0010 << 8); // Activar bobina 2
                        door_motor_right_fsm = 3;
                        break;
                    case 3:
                        GPIOC->BSRR |= (0b0100 << 8); // Activar bobina 3
                        door_motor_right_fsm = 4;
                        break;
                    case 4:
                        GPIOC->BSRR |= (0b1000 << 8); // Activar bobina 4
                        door_motor_right_fsm = 1;
                        break;
                    default:
                        GPIOC->BSRR |= (0b0001 << 8); // Activar bobina 1
                        door_motor_right_fsm = 2;
                        break;
                }
                break;

            case 2: // Esperando
                door_motor_waiting_counter--;
                strcpy(status_message, "Esperando...");
                //usart2_write_line("Esperando con puerta abierta.");
                lcd_update_flag = 1;
                if (door_motor_waiting_counter == 0) {
                    door_close(); // Iniciar cierre de puerta
                }
                break;

            case 3: // Cerrando
                door_motor_left_counter--;
                if (door_motor_left_counter == 0) {
                    door_moving = 0; // Cambiar a detenido
                    strcpy(status_message, "Esperando...");
                    //usart2_write_line("Elevador en espera.");
                    lcd_update_flag = 1;
                }
                GPIOC->BSRR = (0x0F << (8+16)); // Apagar PC8–PC11
                switch(door_motor_left_fsm) {
                    case 1:
                        GPIOC->BSRR |= (0b0001 << 8); // Activar bobina 1
                        door_motor_left_fsm = 2;
                        break;
                    case 2:
                        GPIOC->BSRR |= (0b1000 << 8); // Activar bobina 4
                        door_motor_left_fsm = 3;
                        break;
                    case 3:
                        GPIOC->BSRR |= (0b0100 << 8); // Activar bobina 3
                        door_motor_left_fsm = 4;
                        break;
                    case 4:
                        GPIOC->BSRR |= (0b0010 << 8); // Activar bobina 2
                        door_motor_left_fsm = 1;
                        break;
                    default:
                        GPIOC->BSRR |= (0b0001 << 8); // Activar bobina 1
                        door_motor_left_fsm = 2;
                        break;
                }
            default:
                break;
        }
}

// -x-x-x-x- Botones Llamar Piso -x-x-x-x-

void push1_button_isr(void) {
    // Agregar la solicitud del piso 1 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 1;
        //usart2_write_line("Piso 1 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 6); // Encender led piso 1
    } else if (next_floors[1] == 0 && next_floors[0] != 1) {
        next_floors[1] = 1;
        //usart2_write_line("Piso 1 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 6); // Encender led piso 1
    } else if (next_floors[2] == 0 && next_floors[1] != 1) {
        next_floors[2] = 1;
        //usart2_write_line("Piso 1 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 6); // Encender led piso 1
    }
}

void push2_button_isr(void) {
    // Agregar la solicitud del piso 2 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 2;
        //usart2_write_line("Piso 2 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 7); // Encender led piso 2
    } else if (next_floors[1] == 0 && next_floors[0] != 2) {
        next_floors[1] = 2;
        //usart2_write_line("Piso 2 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 7); // Encender led piso 2
    } else if (next_floors[2] == 0 && next_floors[1] != 2) {
        next_floors[2] = 2;
        //usart2_write_line("Piso 2 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 7); // Encender led piso 2
    }
}

void push3_button_isr(void) {
    // Agregar la solicitud del piso 3 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 3;
        //usart2_write_line("Piso 3 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 8); // Encender led piso 3
    } else if (next_floors[1] == 0 && next_floors[0] != 3) {
        next_floors[1] = 3;
        //usart2_write_line("Piso 3 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 8); // Encender led piso 3
    } else if (next_floors[2] == 0 && next_floors[1] != 3) {
        next_floors[2] = 3;
        //usart2_write_line("Piso 3 solicitado desde push button.");
        GPIOB->BSRR |= (1 << 8); // Encender led piso 3
    }
}

// -x-x-x-x- Funciones generales -x-x-x-x-
void isr_elevator_logic(void) {
    // Compare de 1ms si se debe mover el elevador
    if (elevator_moving == 0 && door_moving == 0) {
        // Verificar si hay un piso objetivo en la cola
        if (next_floors[0] != 0) {
            // Iniciar movimiento hacia el piso objetivo
            switch(next_floors[0]) {
                case 1:
                    switch (elevator_floor) {
                        case 1:
                            // Ya está en el piso 1
                            move_queue();
                            break;
                        case 2:
                            second_to_first_floor();
                            break;
                        case 3:
                            third_to_first_floor();
                            break;
                        default:
                            break;
                    }
                    break;
                case 2:
                    switch (elevator_floor) {
                        case 1:
                            first_to_second_floor();
                            break;
                        case 2:
                            // Ya está en el piso 2
                            move_queue();
                            break;
                        case 3:
                            third_to_second_floor();
                            break;
                        default:
                            break;
                    }
                    break;
                case 3:
                    switch (elevator_floor) {
                        case 1:
                            first_to_third_floor();
                            break;
                        case 2:
                            // Ya está en el piso 2
                            second_to_third_floor();
                            break;
                        case 3:
                            move_queue();
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

void move_queue(void) {
        char texto_concatenado[64];
        elevator_floor = next_floors[0];
    sprintf(texto_concatenado, "Llego al piso %u", elevator_floor);
    //usart2_write_line(texto_concatenado);
    // Desplazar la cola de pisos hacia adelante
    next_floors[0] = next_floors[1];
    next_floors[1] = next_floors[2];
    next_floors[2] = 0;
    sprintf(texto_concatenado, "Nueva cola: %u,%u,%u", next_floors[0], next_floors[1], next_floors[2]);
    //usart2_write_line(texto_concatenado);
    // Apagar led del piso al que llegó
    switch (elevator_floor) {
        case 1:
            GPIOB->BSRR |= (1 << (6 + 16)); // Apagar led piso 1
            break;
        case 2:
            GPIOB->BSRR |= (1 << (7 + 16)); // Apagar led piso 2
            break;
        case 3:
            GPIOB->BSRR |= (1 << (8 + 16)); // Apagar led piso 3
            break;
        default:
            GPIOB->BSRR |= (0b111 << (6 + 16)); // Apagar leds
            break;
    }
}

// -x-x-x-x- Funciones Displays -x-x-x-x-

uint8_t display_digits(uint8_t digit) {
    switch(digit) {
        case 1:
            return 0b000110; // b,c
        case 2:
            return 0b111011; // a,b,d,e,g
        case 3:
            return 0b101111; // a,b,c,d,g
        default:
            return 0b00000000; // Apagado
    }
}

volatile uint8_t display_animation_bits = 0b000000;

// 7-seg: leds = a,b,c,d,e,g → PB0–PB5
//        displays = PC0–PC3
void isr_display_digits_time_multiplexing(void) {
    switch (display_fsm) {
        // 7-seg: segmentos PB0..PB5 activos en HIGH (correcto)
        // dígitos PC0..PC3: activo en LOW (común cátodo)

        case 0:
            GPIOB->BSRR = (0x3F << 16);          // limpia segmentos
            GPIOC->BSRR = 0xF;                   // dígitos OFF (HIGH en PC0..PC3)
            GPIOB->BSRR = (display_digits(elevator_floor) << 0);
            GPIOC->BSRR = (1u << (0 + 16));      // dígito 1 ON (LOW)
            display_fsm = 1;
            break;

        case 1:
            GPIOB->BSRR = (0x3F << 16);
            GPIOC->BSRR = 0xF;                   // todos OFF
            GPIOB->BSRR = (display_animation_bits << 0);
            GPIOC->BSRR = (1u << (1 + 16));      // dígito 2 ON
            display_fsm = 2;
            break;

        case 2:
            GPIOB->BSRR = (0x3F << 16);
            GPIOC->BSRR = 0xF;                   // todos OFF
            // aquí quieres apagado:
            // (no escribas segmentos)
            GPIOC->BSRR = (1u << (2 + 16));      // dígito 3 ON (pero sin segmentos → se ve apagado)
            display_fsm = 3;
            break;

        case 3:
            GPIOB->BSRR = (0x3F << 16);
            GPIOC->BSRR = 0xF;                   // todos OFF
            GPIOC->BSRR = (1u << (3 + 16));      // dígito 4 ON (sin segmentos)
            display_fsm = 0;
            break;

        default:
            GPIOB->BSRR = (0x3F << 16);
            GPIOC->BSRR = 0xF;                   // todos OFF
            display_fsm = 0;
            break;

    }
}
volatile uint8_t display_animation_fsm = 0;

void isr_display_animation(void) {
    switch(display_animation_fsm) {
        case 0:
            display_animation_fsm = 1;
                switch (elevator_moving) {
                    case 0: // Detenido
                        display_animation_bits = 0b100000; // g en medio (apagado)
                        break;
                    case 1: // Subiendo
                        display_animation_bits = 0b010000; // e abajo
                        break;
                    case 2: // Bajando
                        display_animation_bits = 0b000010; // b arriba
                        break;
                    default:
                        display_animation_bits = 0b100000; // g en medio (apagado)
                        break;
                }
            break;
        case 1:
            display_animation_fsm = 2;
            display_animation_bits = display_animation_bits | 0b100000; // g en medio
            break;
        case 2:
            display_animation_fsm = 0;
                                switch (elevator_moving) {
                    case 0: // Detenido
                        display_animation_bits = 0b100000; // g en medio (apagado)
                        break;
                    case 1: // Subiendo
                        display_animation_bits = 0b000010; // b arriba
                    break;
                    case 2: // Bajando
                        display_animation_bits = 0b010000; // e abajo
                        break;
                    default:
                        display_animation_bits = 0b100000; // g en medio (apagado)
                        break;
                }
            break;
        default:
            display_animation_fsm = 0;
            break;
    }
}

// -x-x-x-x- Funciones Keypad -x-x-x-x-

// Keypad: columnas (piso 1–3) = PA0, PA1, PA4  → entradas con pull-up
//         fila (común) = PA15 → salida

volatile uint8_t fsm_keypad = 0;
void isr_keypad_read(void) {
    switch(fsm_keypad) {
        case 0:
            GPIOA->ODR |= (0b10011 << 0);  // "reset columnas" (tu estilo)
            GPIOA->ODR &= ~(1 << 0);       // columna 1
            if ((GPIOA->IDR & (1 << 15)) == 0) {
                if (next_floors[0] == 0) {
                    next_floors[0] = 1;
                    //usart2_write_line("Piso 1 solicitado desde keypad.");
                } else if (next_floors[1] == 0 && next_floors[0] != 1) {
                    next_floors[1] = 1;
                    //usart2_write_line("Piso 1 solicitado desde keypad.");
                } else if (next_floors[2] == 0 && next_floors[1] != 1) {
                    next_floors[2] = 1;
                    //usart2_write_line("Piso 1 solicitado desde keypad.");
                }
            }
            fsm_keypad = 1;
            break;

        case 1:
            GPIOA->ODR |= (0b10011 << 0);  // "reset columnas"
            GPIOA->ODR &= ~(1 << 1);       // columna 2
            if ((GPIOA->IDR & (1 << 15)) == 0) {
                if (next_floors[0] == 0) {
                    next_floors[0] = 2;
                    //usart2_write_line("Piso 2 solicitado desde keypad.");
                } else if (next_floors[1] == 0 && next_floors[0] != 2) {
                    next_floors[1] = 2;
                    //usart2_write_line("Piso 2 solicitado desde keypad.");
                } else if (next_floors[2] == 0 && next_floors[1] != 2) {
                    next_floors[2] = 2;
                    //usart2_write_line("Piso 2 solicitado desde keypad.");
                }
            }
            fsm_keypad = 2;
            break;

        case 2:
            GPIOA->ODR |= (0b10011 << 0);  // "reset columnas"
            GPIOA->ODR &= ~(1 << 4);       // columna 3
            if ((GPIOA->IDR & (1 << 15)) == 0) {
                if (next_floors[0] == 0) {
                    next_floors[0] = 3;
                    //usart2_write_line("Piso 3 solicitado desde keypad.");
                } else if (next_floors[1] == 0 && next_floors[0] != 3) {
                    next_floors[1] = 3;
                    //usart2_write_line("Piso 3 solicitado desde keypad.");
                } else if (next_floors[2] == 0 && next_floors[1] != 3) {
                    next_floors[2] = 3;
                    //usart2_write_line("Piso 3 solicitado desde keypad.");
                }
            }
            fsm_keypad = 0;
            break;
    }
}


// -x-x-x-x-x- funciones LCD -x-x-x-x-x-

// Pequeño delay por software para cumplir tiempos del HD44780
void lcd_init(void){
    // RS=0, E=0, D4..D7=0
    GPIOB->BSRR = (1u<<(11+16)) | (1u<<(10+16));
    GPIOC->BSRR = ((0x0Fu<<4) << 16);
    // >15ms tras power-on
    for(int i=0;i<12000;i++) __asm__("nop");

    // wakeup 4-bit: 0x3,0x3,0x3,0x2 (nibble alto)
    lcd_send_nibble(0x3); for(int i=0;i<3000;i++) __asm__("nop");
    lcd_send_nibble(0x3); for(int i=0;i<3000;i++) __asm__("nop");
    lcd_send_nibble(0x3); for(int i=0;i<3000;i++) __asm__("nop");
    lcd_send_nibble(0x2); for(int i=0;i<3000;i++) __asm__("nop");

    lcd_cmd(0x28); // 4-bit, 2L
    lcd_cmd(0x0C); // display on
    lcd_cmd(0x01); // clear (necesita pausa larga)
    for(int i=0;i<12000;i++) __asm__("nop");
    lcd_cmd(0x06); // entry mode
}

void lcd_pulso_E(void){
    GPIOB->BSRR = (1u<<10);          // E=1
    
    GPIOB->BSRR = (1u<<(10+16));     // E=0
    
}

// caracteres
void escribir_texto_lcd(char* mensaje) {
    GPIOB->BSRR = (1u << 11);        // RS=1 (datos) en PB11
    int16_t len = strlen(mensaje);
    for (int16_t i = 0; i < len; i++) {
        escribir_caracter_lcd((uint8_t)mensaje[i]);
    }
}

void escribir_caracter_lcd(uint8_t c){
    GPIOB->BSRR = (1u<<11);          // RS=1
    lcd_send_nibble((c>>4)&0x0F);    
    lcd_send_nibble(c & 0x0F);       
}

void lcd_send_nibble(uint8_t nibble){
    GPIOC->BSRR = ((0x0F << 4) << 16);             // limpia PC4..PC7
    GPIOC->BSRR = (uint32_t)(nibble & 0x0F) << 4;  // pone nibble
    lcd_pulso_E();
}

// comandos
void lcd_cmd(uint8_t cmd){
    GPIOB->BSRR = (1u<<(11+16));     // RS=0
    lcd_send_nibble((cmd>>4)&0x0F);  
    lcd_send_nibble(cmd & 0x0F);     
}

// esta funcion debe ser llamada cada cierto tiempo con una interrupcion de timer
void isr_lcd_update(void) {
    if (lcd_update_flag) {
        lcd_cmd(0x01); // limpiar
        escribir_texto_lcd(status_message);
        lcd_cmd(0xC0); // salto linea
        sprintf(queue_message, "Cola: %u, %u, %u",
                next_floors[0], next_floors[1], next_floors[2]);
        escribir_texto_lcd(queue_message);
        lcd_update_flag = 0;
    }
}

// -x-x-x-x- Funciones USART -x-x-x-x-

void usart2_init(uint32_t baud)
{
    RCC->IOPENR  |= (1u<<0);          // GPIOA
    RCC->APB1ENR |= (1u<<17);         // USART2

    // GPIOA PA2/PA3 como Alternate Function (AF4 = USART2)
    // PA2
    GPIOA->MODER &= ~(0x3u << (2*2));
    GPIOA->MODER |=  (0x2u << (2*2));       // AF
    GPIOA->AFR[0] &= ~(0xFu << (2*4));
    GPIOA->AFR[0] |=  (0x4u << (2*4));      // AF4 (USART2_TX)
    // PA3
    GPIOA->MODER &= ~(0x3u << (3*2));
    GPIOA->MODER |=  (0x2u << (3*2));       // AF
    GPIOA->AFR[0] &= ~(0xFu << (3*4));
    GPIOA->AFR[0] |=  (0x4u << (3*4));      // AF4 (USART2_RX)

    // (opcional) velocidad/media-alta y push-pull sin pull-ups:
    GPIOA->OSPEEDR |= (0x3u<<(2*2)) | (0x3u<<(3*2));
    GPIOA->PUPDR   &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->OTYPER  &= ~((1u<<2) | (1u<<3));

    // --- USART2 core ---
    USART2->CR1 = 0;                         // limpia config
    // Baud = fCK/baud (oversampling x16). fCK = 16 MHz (HSI16)
    USART2->BRR = 16000000u / baud;          // 115200 → ~138 (0x8A)
    // 8N1: M=0, PCE=0, STOP=00 (STOP está en CR2)
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    // Habilita TX y RX
    USART2->CR1 |= (1u<<3) | (1u<<2);        // TE | RE
    USART2->CR1 |= (1u<<0);                  // UE (USART enable)
}

void usart2_putc(char c)
{
    // espera a que TXE=1 (TDR vacío)
    while(!(USART2->ISR & (1u<<7)));         // TXE
    USART2->TDR = (uint8_t)c;
}

void usart2_write(const char *s)
{
    while(*s) usart2_putc(*s++);
}

void usart2_write_line(const char *s)
{
    usart2_write(s);
    usart2_write("\r\n");
}
