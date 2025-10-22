#include <stdint.h>
#include <string.h>
#include "stm32l053xx.h"

// -x-x-x-x- Definición de pines elevador -x-x-x-x-

// 7-seg: leds = a,b,c,d,e,g → PB0–PB5
//        displays = PC0–PC3

// LCD:    D4–D7 = PC4–PC7
//         E = PB10
//         RS = PB11

// Buzzer: PA8

// Motor elevador: IN1–IN4 = PB12–PB15

// Motor puerta:   IN1–IN4 = PC8–PC11

// Keypad: columnas (piso 1–3) = PA0, PA1, PA4  → entradas con pull-up
//         fila (común) = PA15 → salida (controlada por software)

// Botones para llamar al piso:
//         Piso 1 = PA6 (EXTI6)
//         Piso 2 = PA7 (EXTI7)
//         Piso 3 = PC12 (EXTI12)

// -x-x-x-x- Variables Globales -x-x-x-x-
// Logica elevador
volatile uint8_t elevator_floor = 0; // Piso actual del elevador
volatile uint8_t next_floors[3] = {0,0,0};   // Piso objetivo
volatile uint8_t next_floors_temp[3] = {0,0,0};   // Piso objetivo

// buzzer
volatile uint8_t buzzer_flag = 0; // Bandera para activar el buzzer
volatile uint16_t buzzer_beep_counter = 0; // Contador para la duración del beep
volatile uint8_t buzzer_last_state = 0; // Ultimo estado del buzzer

// Motores
volatile uint8_t elevator_moving = 0; // Estado de movimiento del elevador (0: detenido, 1: arriba, 2: abajo)

volatile uint8_t elevator_motor_right_fsm = 1; // FSM motor elevador
volatile uint8_t elevator_motor_right_counter = 0; // contador

volatile uint8_t elevator_motor_left_fsm = 1; // FSM motor elevador
volatile uint8_t elevator_motor_left_counter = 0; // contador

// Puerta
volatile uint8_t door_moving = 0; // Estado de movimiento de la puerta (0: detenido, 1: abriendo, 2: esperando 3: cerrando)

volatile uint8_t door_motor_right_fsm = 1; // FSM motor puerta
volatile uint8_t door_motor_right_counter = 0; // contador

volatile uint8_t door_motor_left_fsm = 1; // FSM motor puerta
volatile uint8_t door_motor_left_counter = 0; // contador

volatile uint8_t door_motor_waiting_counter = 0; // contador

// Displays
display_fsm = 0;

// -x-x-x-x- Funciones Motor Elevador -x-x-x-x-
// arriba
void first_to_second_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 64;
    elevator_moving = 1;
}

void second_to_third_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 64;
    elevator_moving = 1;
}

void first_to_third_floor(void) {
    elevator_motor_right_fsm = 1;
    elevator_motor_right_counter = 128;
    elevator_moving = 1;
}

// abajo
void third_to_second_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 64;
    elevator_moving = 2;
}
void second_to_first_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 64;
    elevator_moving = 2;
}
void third_to_first_floor(void) {
    elevator_motor_left_fsm = 1;
    elevator_motor_left_counter = 128;
    elevator_moving = 2;
}

void buzzer_beep(void) {
    // Configurar pin del buzzer como salida
        buzzer_beep_counter = 1000; // Duración del beep en ms
        buzzer_flag = 1; // Beep al llegar al piso
        buzzer_last_state = 0; // Duración del beep en ms

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
    
    }
}

void isr_elevator_motor_movement() {
    switch (elevator_moving) {
        case 0: // Detenido
            // No hacer nada
            break;
        case 1: // Subiendo
            elevator_motor_right_counter--;
            if (elevator_motor_right_counter == 1) {
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
            if (elevator_motor_left_counter == 1) {
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
}
// derecha
void door_close(void) {
    door_motor_left_fsm = 1;
    door_motor_right_counter = 64;
    door_moving = 3; // Cambiar estado a cerrando
}

void isr_door_motor_movement() {
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
                if (door_motor_waiting_counter == 0) {
                    door_close(); // Iniciar cierre de puerta
                }
                break;

            case 3: // Cerrando
                door_motor_left_counter--;
                if (door_motor_left_counter == 0) {
                    door_moving = 0; // Cambiar a detenido
                }
                GPIOC->BSRR |= (0x0F << (1816)); // Apagar PB12–PB15
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

void push1_button_isr() {
    // Agregar la solicitud del piso 1 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 1;
    } else if (next_floors[1] == 0 && next_floors[0] != 1) {
        next_floors[1] = 1;
    } else if (next_floors[2] == 0 && next_floors[1] != 1) {
        next_floors[2] = 1;
    } else {
        
    }

}

void push2_button_isr() {
    // Agregar la solicitud del piso 2 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 2;
    } else if (next_floors[1] == 0 && next_floors[0] != 2) {
        next_floors[1] = 2;
    } else if (next_floors[2] == 0 && next_floors[1] != 2) {
        next_floors[2] = 2;
    } else {
        
    }

}

void push3_button_isr() {
    // Agregar la solicitud del piso 3 a la cola
    if (next_floors[0] == 0) {
        next_floors[0] = 3;
    } else if (next_floors[1] == 0 && next_floors[0] != 3) {
        next_floors[1] = 3;
    } else if (next_floors[2] == 0 && next_floors[1] != 3) {
        next_floors[2] = 3;
    } else {
        
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
    // Desplazar la cola de pisos hacia adelante
    next_floors[0] = next_floors[1];
    next_floors[1] = next_floors[2];
    next_floors[2] = 0;
}

void add_queue(uint8_t floor) {
    // Desplazar la cola de pisos hacia adelante
    next_floors[0] = next_floors[1];
    next_floors[1] = next_floors[2];
    next_floors[2] = 0;
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
void isr_display_digits_time_multiplexing() {
    switch (display_fsm) {
        case 0:
            GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
            GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
            GPIOB->BSRR = (display_digits(elevator_floor) << 0);
            GPIOC->BSRR = (1 << 0); // Encender display 1
            display_fsm++;
            break;
        case 1:
            GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
            GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
            GPIOB->BSRR = (display_animation_bits << 0);
            GPIOC->BSRR = (1 << 1); // Encender display 2
            display_fsm++;
            break;
        case 2:
            GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
            GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
            GPIOB->BSRR = (0b000000 << 0); // Apagado
            GPIOC->BSRR = (1 << 2); // Encender display 3
            display_fsm++;
            break;
        case 3:
            GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
            GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
            GPIOB->BSRR = (0b000000 << 0); // Apagado
            GPIOC->BSRR = (1 << 3); // Encender display 4
            display_fsm++;
            break;
        default:
        GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
        GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
        display_fsm = 0;
        break;
    }
}
volatile uint8_t display_animation_fsm = 0;

void isr_display_animation() {
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

void isr_keypad_read() {
    switch(fsm_keypad) {
        case 0:
            GPIOC->ODR |= (0b10011 << 0);  // reset columnas
            GPIOC->ODR &= ~(1 << 0); // columna 1
            if ((GPIOC-> IDR & (1 << 15)) == 0) {
                if (next_floors[0] == 0) {
                    next_floors[0] = 1;
                } else if (next_floors[1] == 0 && next_floors[0] != 1) {
                    next_floors[1] = 1;
                } else if (next_floors[2] == 0 && next_floors[1] != 1) {
                    next_floors[2] = 1;
                } else {
                    
                }
            }
            fsm_keypad = 1;
            break;
        case 1:
            GPIOC->ODR |= (0b10011 << 0);  // reset columnas
            GPIOC->ODR &= ~(1 << 1); // columna 2
            if ((GPIOC-> IDR & (1 << 15)) == 0) {
                // Agregar la solicitud del piso 2 a la cola
                if (next_floors[0] == 0) {
                    next_floors[0] = 2;
                } else if (next_floors[1] == 0 && next_floors[0] != 2) {
                    next_floors[1] = 2;
                } else if (next_floors[2] == 0 && next_floors[1] != 2) {
                    next_floors[2] = 2;
                } else {
                    
                }
            }
            fsm_keypad = 2;
            break;
        case 2:
            GPIOC->ODR |= (0b10011 << 0);  // reset columnas
            GPIOC->ODR &= ~(1 << 4); // columna 3
            if ((GPIOC-> IDR & (1 << 15)) == 0) {
                // Agregar la solicitud del piso 3 a la cola
                if (next_floors[0] == 0) {
                    next_floors[0] = 3;
                } else if (next_floors[1] == 0 && next_floors[0] != 3) {
                    next_floors[1] = 3;
                } else if (next_floors[2] == 0 && next_floors[1] != 3) {
                    next_floors[2] = 3;
                } else {
                    
                }
            }
            fsm_keypad = 0;
            break;
    }
}