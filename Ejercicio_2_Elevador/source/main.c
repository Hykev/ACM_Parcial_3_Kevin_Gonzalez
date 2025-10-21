#include <stdint.h>
#include <string.h>
#include "stm32l053xx.h"

// -x-x-x-x- Definición de pines elevador -x-x-x-x-

// 7-seg: leds = a,b,c,d,e,g → PB0–PB5
//        displays = PC0–PC3

// LCD:    D4–D7 = PC4–PC7
//         E = PB10
//         RS = PB11

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

// 7-seg: leds = a,b,c,d,e,g → PB0–PB5
//        displays = PC0–PC3
void isr_display_time_multiplexing() {
    switch (display_fsm) {
        // 7-seg: leds = A,B (PA0-PA1), C,D (PA11-PA12), E,F,G(PA4-PA6), displays = PA7-PA10
        case 0:
        GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
        GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
        GPIOB->BSRR = (display_digits(elevator_floor) << 0);
        GPIOC->BSRR = (1 << 0); // Encender display 1
        display_fsm++;
        break;
        case 1:
        
        case 2:
        
        case 3:
        

        default:
        GPIOB->BSRR = (0x3F << 16); // Reset display pines PB0-PB5
        GPIOC->BSRR = (0xF << 16); // Reset display pines de PC0-PC3
        display_fsm = 0;
        break;
    }
}

void isr_display_animation() {
    if (elevator_moving == 1) {
        // Lógica de parpadeo o animación
    }
}

// -x-x-x-x- Funciones Keypad -x-x-x-x-
int16_t keypad_read() {
    GPIOC->ODR |= (0x0F << 0);  // reset

    GPIOC->ODR &= ~(1 << 0);
    timer_loop(4);
    if        ((GPIOC-> IDR & (1 << 4)) == 0) {
        return 1;
    } else if ((GPIOC-> IDR & (1 << 5)) == 0) {
        return 4;
    } else if ((GPIOC-> IDR & (1 << 6)) == 0) {
        return 7;
    } else if ((GPIOC-> IDR & (1 << 7)) == 0) {
        return '*';
    }


    GPIOC->ODR |= (0x0F << 0);  // reset
    GPIOC->ODR &= ~(1 << 1);
    timer_loop(4);
    if        ((GPIOC-> IDR & (1 << 4)) == 0) {
        return 2;
    } else if ((GPIOC-> IDR & (1 << 5)) == 0) {
        return 5;
    } else if ((GPIOC-> IDR & (1 << 6)) == 0) {
        return 8;
    } else if ((GPIOC-> IDR & (1 << 7)) == 0) {
        return 0;
    }

    GPIOC->ODR |= (0x0F << 0);  // reset
    GPIOC->ODR &= ~(1 << 2);
    timer_loop(4);
    if        ((GPIOC-> IDR & (1 << 4)) == 0) {
        return 3;
    } else if ((GPIOC-> IDR & (1 << 5)) == 0) {
        return 6;
    } else if ((GPIOC-> IDR & (1 << 6)) == 0) {
        return 9;
    } else if ((GPIOC-> IDR & (1 << 7)) == 0) {
        return '#';
    }

    GPIOC->ODR |= (0x0F << 0);  // reset
    GPIOC->ODR &= ~(1 << 3);
    timer_loop(4);
    if        ((GPIOC-> IDR & (1 << 4)) == 0) {
        return 'A';
    } else if ((GPIOC-> IDR & (1 << 5)) == 0) {
        return 'B';
    } else if ((GPIOC-> IDR & (1 << 6)) == 0) {
        return 'C';
    } else if ((GPIOC-> IDR & (1 << 7)) == 0) {
        return 'D';
    }

    return -1; // Si no hay tecla
}