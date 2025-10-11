#include <stdint.h>
#include <string.h>

#define PERIPH_BASE           (0x40000000)
#define AHB_BASE              (PERIPH_BASE + 0x20000)
#define IOPORT_BASE           (PERIPH_BASE + 0x10000000)
#define RCC_BASE              (AHB_BASE + 0x1000)
#define GPIOA_BASE            (IOPORT_BASE + 0x0000)
#define GPIOB_BASE            (IOPORT_BASE + 0x0400)
#define GPIOC_BASE            (IOPORT_BASE + 0x0800)

typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR, ICSCR, CRRCR, CFGR, CIER, CIFR, CICR, IOPRSTR,
                      AHBRSTR, APB2RSTR, APB1RSTR, IOPENR, AHBENR, APB2ENR,
                      APB1ENR, IOPSMENR, AHBSMENR, APB2SMENR, APB1SMENR, CCIPR, CSR;
} RCC_TypeDef;

#define GPIOA ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *) GPIOC_BASE)
#define RCC   ((RCC_TypeDef *) RCC_BASE)

// === Función de delay ===
void timer_loop(int32_t ticks){
    for(int32_t t = 0; t < ticks; t++) {
        for(int32_t i = 0; i < 15; i++) { // 140
            __asm__ __volatile__ ("nop");
        }
    }
}

// -x-x-x-x- Funciones 7-seg -x-x-x-x-

const uint8_t nums[10] = {
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111  // 9
    };

uint16_t display_digits(uint8_t numero){

    // 7-seg: leds = A,B (PA0-PA1), C,D (PA11-PA12), E,F,G(PA4-PA6), displays = PA7-PA10
    uint8_t pattern = nums[numero];
    uint16_t mask = 0b0000000000000; // Inicializa el mask en 0

    if(pattern & (1<<0)) mask |= (1<<0);   // A → PA0
    if(pattern & (1<<1)) mask |= (1<<1);   // B → PA1
    if(pattern & (1<<2)) mask |= (1<<11);  // C → PA11
    if(pattern & (1<<3)) mask |= (1<<12);  // D → PA12
    if(pattern & (1<<4)) mask |= (1<<4);   // E → PA4
    if(pattern & (1<<5)) mask |= (1<<5);   // F → PA5
    if(pattern & (1<<6)) mask |= (1<<6);   // G → PA6

    return mask;
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

int read_key(void){
    int k1 = keypad_read();
    if (k1 == -1) return -1;
    timer_loop(20);     // ~50 ms
    int k2 = keypad_read();
    if(k1 == k2) return k1;
    else return -1;
}

void move_microwave_time(uint8_t *microwave_time_ptr, int8_t key) {
    microwave_time_ptr[0] = microwave_time_ptr[1];
    microwave_time_ptr[1] = microwave_time_ptr[2];
    microwave_time_ptr[2] = microwave_time_ptr[3];
    microwave_time_ptr[3] = key;
}

// -x-x-x-x- Funciones LCD -x-x-x-x-

void lcd_init(void){
    // RS=0, E=0, D4..D7=0
    GPIOB->BSRR = ( (1u<<(5+16)) | (1u<<(4+16)) | (0x0Fu<<(0+16)) );
    timer_loop(8); // >15ms

    // Modo 4-bit wakeup (tres veces 0x3 en nibble alto, luego 0x2)
    lcd_send_nibble(0x3);
    lcd_send_nibble(0x3);
    lcd_send_nibble(0x3);
    lcd_send_nibble(0x2); // entra 4-bit

    // Function set: 4-bit, 2 lines, 5x8
    lcd_cmd(0x28);
    // Display ON, cursor OFF, blink OFF
    lcd_cmd(0x0C);
    // Clear
    lcd_cmd(0x01);
    timer_loop(8);
    // Entry mode: increment, no shift
    lcd_cmd(0x06);
}

void lcd_pulso_E(void){
    GPIOB->BSRR = (1u<<4);         // E=1
    timer_loop(8);
    GPIOB->BSRR = (1u<<(4+16));    // E=0
    timer_loop(8);
}

// caracteres
void escribir_texto_lcd(char* mensaje) {
    GPIOB->BSRR = (1 << 5); // RS Modo datos
    int16_t len = strlen(mensaje);
    for (int16_t i = 0; i < len; i++) {
        escribir_caracter_lcd(mensaje[i]);
    }
}

void escribir_caracter_lcd(uint8_t caracter){
    uint8_t alto, bajo = 0;
    alto = (caracter >> 4) & 0x0F;
    bajo = caracter & 0x0F;

    lcd_send_nibble(alto);
    lcd_send_nibble(bajo);
    timer_loop(8);
}

void lcd_send_nibble(uint8_t nibble){ // nib=0..15 en PB0..PB3 → D4..D7
    // limpia PB0..PB3
    GPIOB->BSRR = (0x0F << 16);
    // coloca nibble
    GPIOB->BSRR = nibble & 0x0F;
    lcd_pulso_E();
}

// comandos
void lcd_cmd(uint8_t cmd){
    GPIOB->BSRR |= (1u<<(5+16)); // RS=0 (comando)
    uint8_t alto, bajo = 0;
    alto = (cmd >> 4) & 0x0F;
    bajo = cmd & 0x0F;

    lcd_send_nibble(alto);
    lcd_send_nibble(bajo);
    timer_loop(8);
}

// Motor pines PB6-9
void activacion_motor_stepper(uint8_t* fsm_motor_ptr) {
    GPIOB->BSRR |= (0x0F << (6+16)); // Apagar PB6–PB9
    switch(*fsm_motor_ptr) {
        case 1:
            GPIOB->BSRR |= (0b0001 << 6); // Activar bobina 1
            *fsm_motor_ptr = 2;
            break;
        case 2:
            GPIOB->BSRR |= (0b0010 << 6); // Activar bobina 2
            *fsm_motor_ptr = 3;
            break;
        case 3:
            GPIOB->BSRR |= (0b0100 << 6); // Activar bobina 3
            *fsm_motor_ptr = 4;
            break;
        case 4:
            GPIOB->BSRR |= (0b1000 << 6); // Activar bobina 4
            *fsm_motor_ptr = 1;
            break;
        default:
            GPIOB->BSRR |= (0b0001 << 6); // Activar bobina 1
            *fsm_motor_ptr = 2;
            break;
    }
}


// -x-x-x-x- Definicion de pines -x-x-x-x-

// 7-seg: leds = A,B (PA0-PA1), C,D (PA11-PA12), E,F,G(PA4-PA6), displays = PA7-PA10
// LCD: D4-D7 = PB0-3, E = PB4, RS = PB5
// Motor: IN1-IN4 = PB6–PB9
// Keypad: PC0-PC3 columnas, PC4-PC7 filas

// -x-x-x-x- Main -x-x-x-x-

int main() {

    // inicializacion de los perifericos y pines
    RCC->IOPENR |= 0b111; // Habilita reloj GPIOA, GPIOB, GPIOC

    // 7-seg: leds = PA0-PA1, PA11-PA12, PA4-PA6, displays = PA7-PA10  (sin punto decimal)
    GPIOA->MODER &= ~(0b1111<<0); // PA0-PA1
    GPIOA->MODER |=  (0b0101<<0); // A, B

    GPIOA->MODER &= ~(0b1111<<22); // PA11-PA12
    GPIOA->MODER |=  (0b0101<<22); // C, D

    GPIOA->MODER &= ~(0b111111<<8); // PA4-PA6
    GPIOA->MODER |=  (0b010101<<8); // E, F, G

    GPIOA->MODER &= ~(0b11111111<<14); // PA7-PA10
    GPIOA->MODER |=  (0b01010101<<14); // displays

    // LCD: D4-D7 = PB0-3, E = PB4, RS = PB5
    GPIOB->MODER &= ~(0b111111111111<<0); // PB0-PB5
    GPIOB->MODER |=  (0b010101010101<<0); // output

    // Motor: IN1-IN4 = PB6–PB9
    GPIOB->MODER &= ~(0b11111111 << 12); // PB6-PB9
    GPIOB->MODER |=  (0b01010101 << 12); // output

    // Keypad: PC0-PC3 columnas (salida high), PC4-PC7 filas (entrada pull-up)
    GPIOC->MODER &= ~(0b1111111111111111<<0);
    // columnas
    GPIOC->MODER |=  (0b01010101<<0); // output (solo columnas)
    GPIOC->BSRR  |=  (0b1111<<0);
    // filas
    GPIOC->PUPDR &= ~(0b11111111<<8);
    GPIOC->PUPDR |=  (0b01010101<<8);

    // Otras variables
    uint8_t fsm_keypad = 1; // Para leer la columna del keypad
    uint8_t fsm_motor = 1; // Para el motor stepper
    uint16_t motor_delay = 0; // para controlar la velocidad del motor
    int16_t key = -1; // tecla presionada
    int16_t last_key = -2; // ultima tecla presionada (para debounce) -2 para que no sea igual a -1
    uint8_t microwave_time[4] = {0, 0, 0, 0}; // tiempo display del microondas MM:SS
    uint8_t clock_time[4] = {0, 0, 0, 0}; // tiempo display del reloj HH:MM
    uint8_t display_time[4] = {0, 0, 0, 0}; // tiempo a mostrar en el display
    uint16_t display_mask = 0; // para controlar los pines de los digitos del display
    uint16_t display_delay = 0; // para controlar la frecuencia de cada digito del display
    uint32_t microwave_counter = 0; // contador para el tiempo del microondas
    uint32_t clock_counter = 0; // contador para el tiempo del reloj
    uint8_t hu_max = 9; // Máximo del digito de unidades de hora
    const uint32_t second_counter = 80; // 1 segundo
    const uint32_t minute_counter = 60 * second_counter; // 1 minuto

    // Variables para mensajes en LCD
    char mensaje_tiempo[6] = "00:00"; // mensaje a mostrar en el display
    char mensaje1[] = "Poporopos";
    char mensaje2[] = "Pizza";
    char mensaje3[] = "Carne";
    char mensaje4[] = "Verduras";
    char mensaje5[] = "Tiempo no valido";
    char mensaje6[] = "Listo!";
    char mensaje7[] = "Calentando...";
    uint8_t display_fsm = 0; // fsm para el display
    uint8_t microwave_typing_state = 0; // ingresando numeros
    uint8_t microwave_running_state = 0; // calentando

    lcd_init();

    while(1) {

        key = read_key(); // boton presionado

        if (key != last_key) {
            last_key = key;
        } else {
            key = -1; // si es igual, no es un nuevo boton
        }

        if (key != -1 && (key >= 0 && key <= 9)) {
        lcd_cmd(0x01); // limpiar
        move_microwave_time(microwave_time, key); // actualizar tiempo del microondas
        microwave_typing_state = 1; // estado de ingresando numeros
        }

        switch (key)
        {
        case 'A':
            // Mostrar en la LCD "Poporopos 2:30"
            lcd_cmd(0x01); // limpiar
            escribir_texto_lcd(mensaje1);
            lcd_cmd(0xC0); // salto linea
            strcpy(mensaje_tiempo, "02:30");
            escribir_texto_lcd(mensaje_tiempo);
            microwave_time[0] = 0;
            microwave_time[1] = 2;
            microwave_time[2] = 3;
            microwave_time[3] = 0;
            break;
        case 'B':
            // Mostrar en la LCD "Pizza 1:30"
            lcd_cmd(0x01); // limpiar
            escribir_texto_lcd(mensaje2);
            lcd_cmd(0xC0); // salto linea
            strcpy(mensaje_tiempo, "01:30");
            escribir_texto_lcd(mensaje_tiempo);
            microwave_time[0] = 0;
            microwave_time[1] = 1;
            microwave_time[2] = 3;
            microwave_time[3] = 0;
            break;
        case 'C':
            // Mostrar en la LCD "Carne 10:00"
            lcd_cmd(0x01); // limpiar
            escribir_texto_lcd(mensaje3);
            lcd_cmd(0xC0); // salto linea
            strcpy(mensaje_tiempo, "10:00");
            escribir_texto_lcd(mensaje_tiempo);
            microwave_time[0] = 1;
            microwave_time[1] = 0;
            microwave_time[2] = 0;
            microwave_time[3] = 0;
            break;
        case 'D':
            // Mostrar en la LCD "Verduras 3:00"
            lcd_cmd(0x01); // limpiar
            escribir_texto_lcd(mensaje4);
            lcd_cmd(0xC0); // salto linea
            strcpy(mensaje_tiempo, "03:00");
            escribir_texto_lcd(mensaje_tiempo);
            microwave_time[0] = 0;
            microwave_time[1] = 3;
            microwave_time[2] = 0;
            microwave_time[3] = 0;
            break;
        case '#':
            // Limpiar LCD y cancelar
            lcd_cmd(0x01); // limpiar
            microwave_time[0] = 0;
            microwave_time[1] = 0;
            microwave_time[2] = 0;
            microwave_time[3] = 0;
            microwave_typing_state = 0;
            microwave_running_state = 0;
            break;
        case '*':
            if (microwave_time[0] == 0 && microwave_time[1] == 0 && microwave_time[2] == 0 && microwave_time[3] == 0) {
                // No iniciar si el tiempo es 0
                break;
            } else if (microwave_time[0] >= 6 || microwave_time[2] >= 6) {
                // No iniciar si las decenas de minutos o segundos son mayores a 5
                lcd_cmd(0x01); // limpiar
                escribir_texto_lcd(mensaje5);
                break;
            }
            // Iniciar microondas
            lcd_cmd(0x01); // limpiar
            escribir_texto_lcd(mensaje7);
            microwave_running_state = 1;
            break;
        default:
            break;
        }

        if (microwave_typing_state == 1 || microwave_running_state == 1) {
            display_time[0] = microwave_time[0];
            display_time[1] = microwave_time[1];
            display_time[2] = microwave_time[2];
            display_time[3] = microwave_time[3];

        } else {
            display_time[0] = clock_time[0];
            display_time[1] = clock_time[1];
            display_time[2] = clock_time[2];
            display_time[3] = clock_time[3];

        }

        display_delay++;
        if(display_delay >= 3){
            display_delay = 0;
            switch (display_fsm)
            {

                // 7-seg: leds = A,B (PA0-PA1), C,D (PA11-PA12), E,F,G(PA4-PA6), displays = PA7-PA10
                case 0:
                GPIOA->BSRR = (0x1FF3 << 16); // Reset display pines de 0-1, 4-10 y 11-12 es 1111111110011
                display_mask = display_digits(display_time[0]);
                GPIOA->BSRR = display_mask; // Mostrar digito 1
                GPIOA->BSRR = (1 << 7); // Encender display 1
                display_fsm++;
                break;
                case 1:
                GPIOA->BSRR = (0x1FF3 << 16); // Reset pines
                display_mask = display_digits(display_time[1]);
                GPIOA->BSRR = display_mask; // Mostrar digito 2
                GPIOA->BSRR = (1 << 8); // Encender display 1
                display_fsm++;
                break;
                case 2:
                GPIOA->BSRR = (0x1FF3 << 16); // Reset pines
                display_mask = display_digits(display_time[2]);
                GPIOA->BSRR = display_mask; // Mostrar digito 3
                GPIOA->BSRR = (1 << 9); // Encender display 1
                display_fsm++;
                break;
                case 3:
                GPIOA->BSRR = (0x1FF3 << 16); // Reset pines
                display_mask = display_digits(display_time[3]);
                GPIOA->BSRR = display_mask; // Mostrar digito 4
                GPIOA->BSRR = (1 << 10); // Encender display 1
                display_fsm = 0;
                break;

                default:
                GPIOA->BSRR = (0x1FF3 << 16); // Reset pines
                display_fsm = 0;
                break;
            }
        }

        if(microwave_running_state == 1) {

            motor_delay++;
            if(motor_delay >= 5){ // Ajusta la velocidad del motor
                motor_delay = 0;
                activacion_motor_stepper(&fsm_motor);
            }

            microwave_counter++;
            if (microwave_counter >= second_counter) { // 1 segundo
                microwave_counter = 0;
                if (microwave_time[3] > 0) {
                    microwave_time[3]--;
                } else {
                    if (microwave_time[2] > 0) {
                        microwave_time[2]--;
                        microwave_time[3] = 9;
                    } else {
                        if (microwave_time[1] > 0) {
                            microwave_time[1]--;
                            microwave_time[2] = 5;
                            microwave_time[3] = 9;
                        } else {
                            if (microwave_time[0] > 0) {
                                microwave_time[0]--;
                                microwave_time[1] = 9;
                                microwave_time[2] = 5;
                                microwave_time[3] = 9;
                            } else {
                                // Tiempo terminado
                                lcd_cmd(0x01); // limpiar
                                escribir_texto_lcd(mensaje6);
                                microwave_running_state = 0;
                                microwave_typing_state = 0;
                            }
                        }
                    }
                }
            }
        }

        clock_counter++;
            if (clock_counter >= minute_counter) { // 1 minuto
                clock_counter = 0;
                if (clock_time[3] < 9) {
                    clock_time[3]++;
                } else {
                    if (clock_time[2] < 5) {
                        clock_time[3] = 0;
                        clock_time[2]++;
                    } else {
                        if (clock_time[1] < hu_max) {
                            clock_time[2] = 0;
                            clock_time[3] = 0;
                            clock_time[1]++;
                        } else {
                            if (clock_time[0] < 2) {
                                clock_time[1] = 0;
                                clock_time[2] = 0;
                                clock_time[3] = 0;
                                clock_time[0]++;
                                if (clock_time[0] == 2) {
                                    hu_max = 3;
                                }
                            } else {
                                clock_time[0] = 0;
                                clock_time[1] = 0;
                                clock_time[2] = 0;
                                clock_time[3] = 0;
                                hu_max = 9;
                            }
                        }
                    }
                }
            }
    }
}
