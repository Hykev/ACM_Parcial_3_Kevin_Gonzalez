// librerias
#include <stdint.h>
#include <string.h>
#include "stm32l053xx.h"

// -x-x-x-x- Definición de pines Dispensadora -x-x-x-x-

// 7-seg: leds = a,b,c,d,e,f,g → PB0–PB6
//        displays (4 dígitos) = PC0–PC3
//        punto decimal = conectado directo a VDD

// LCD:   D4–D7 = PC4–PC7
//        E = PB10
//        RS = PB11

// Keypad: columnas = PA0, PA1, PA4, PA5
//          filas = PA15, PB9   (2 filas usadas)

// Push buttons (debounce físico): 
//          .25Q = PA6 (EXTI6)
//          1Q   = PA7 (EXTI7)

// Motores stepper (2 dispensadores):
//          Motor 1 (Snack A): PB12–PB15
//          Motor 2 (Snack B): PC8–PC11

// Buzzer: PA8 (PWM con timer)

// USART2: TX = PA2, RX = PA3
// Prototipos mínimos usados más abajo

static inline void usart2_init(uint32_t baud);

void SystemInit(void)
{
    /* =========================
     * 1) Reloj de sistema (HSI16 @ 16 MHz)
     * ========================= */
    RCC->CR   |= (1u << 0);                 // HSION
    while(!(RCC->CR & (1u<<1)));            // HSIRDY
    RCC->CFGR &= ~(0x3u << 0);              // SW = HSI16
    RCC->CFGR |=  (0x1u << 0);

    /* =========================
     * 2) Habilitar clocks AHB/APB
     * ========================= */
    RCC->IOPENR  |= (1u<<0) | (1u<<1) | (1u<<2);   // GPIOA/B/C
    RCC->APB2ENR |= (1u<<0) | (1u<<2) | (1u<<5);   // SYSCFG, TIM21, TIM22
    RCC->APB1ENR |= (1u<<0) | (1u<<17);            // TIM2, USART2
    RCC->APB2ENR |= (1u<<11);                      // TIM1 (PWM buzzer)

    /* =========================
     * 3) Configuración GPIO
     * ========================= */

    // --- 7-seg: PB0..PB6 salida (a..g) ---
    // Limpia bits 0..13 y pone 01 en cada par
    GPIOB->MODER &= ~0x00003FFFu;
    GPIOB->MODER |=  0x00001555u;

    // --- 7-seg: PC0..PC3 salida (dígitos) ---
    GPIOC->MODER &= ~0x000000FFu;
    GPIOC->MODER |=  0x00000055u;

    // --- LCD: PC4..PC7 salida (D4..D7) ---
    GPIOC->MODER &= ~0x0000FF00u;
    GPIOC->MODER |=  0x00005500u;

    // --- LCD: PB10 (E) y PB11 (RS) salida ---
    GPIOB->MODER &= ~0x00F00000u;
    GPIOB->MODER |=  0x00500000u;

    // --- Motores: PB12..PB15 salida (Snack A), PC8..PC11 salida (Snack B) ---
    GPIOB->MODER &= ~0xFF000000u;  GPIOB->MODER |= 0x55000000u;   // PB12..PB15
    GPIOC->MODER &= ~0x00FF0000u;  GPIOC->MODER |= 0x00550000u;   // PC8..PC11

    // --- Buzzer: PA8 salida AF2 (TIM1_CH1) ---
    GPIOA->MODER &= ~0x00030000u;  // limpiar
    GPIOA->MODER |=  0x00020000u;  // AF
    GPIOA->AFR[1] &= ~(0xFu << ((8-8)*4));
    GPIOA->AFR[1] |=  (0x2u << ((8-8)*4)); // AF2 = TIM1_CH1

    // --- USART2: PA2/PA3 AF4 ---
    GPIOA->MODER &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->MODER |=  ((0x2u<<(2*2)) | (0x2u<<(3*2)));
    GPIOA->AFR[0] &= ~((0xFu<<(2*4)) | (0xFu<<(3*4)));
    GPIOA->AFR[0] |=  ((0x4u<<(2*4)) | (0x4u<<(3*4))); // AF4 USART2

    // --- Keypad: columnas = PA0, PA1, PA4, PA5 → entradas con pull-up ---
    GPIOA->MODER &= ~((0x3u<<(0*2)) | (0x3u<<(1*2)) | (0x3u<<(4*2)) | (0x3u<<(5*2)));
    GPIOA->PUPDR &= ~((0x3u<<(0*2)) | (0x3u<<(1*2)) | (0x3u<<(4*2)) | (0x3u<<(5*2)));
    GPIOA->PUPDR |=  ((0x1u<<(0*2)) | (0x1u<<(1*2)) | (0x1u<<(4*2)) | (0x1u<<(5*2)));

    // --- Keypad: filas = PA15, PB9 → salidas ---
    GPIOA->MODER &= ~(0x3u << (15*2));  GPIOA->MODER |= (0x1u << (15*2)); // PA15 out
    GPIOB->MODER &= ~(0x3u << (9*2));   GPIOB->MODER |= (0x1u << (9*2));  // PB9  out
    // fila en nivel alto por defecto (desactiva fila)
    GPIOA->BSRR = (1u<<15);
    GPIOB->BSRR = (1u<<9);

    // --- Push buttons: PA6 (.25Q), PA7 (1Q) → entradas con pull-up (EXTI) ---
    GPIOA->MODER &= ~((0x3u<<(6*2)) | (0x3u<<(7*2)));
    GPIOA->PUPDR &= ~((0x3u<<(6*2)) | (0x3u<<(7*2)));
    GPIOA->PUPDR |=  ((0x1u<<(6*2)) | (0x1u<<(7*2)));

    /* =========================
     * 4) USART2 (log @115200 8N1)
     * ========================= */
    usart2_init(115200);

    /* =========================
     * 5) EXTI (push PA6/PA7, flanco bajada)
     * ========================= */
    SYSCFG->EXTICR[1] &= ~((0xFu<<8) | (0xFu<<12)); // EXTI6/7 ← PA
    EXTI->IMR  |= (1u<<6) | (1u<<7);
    EXTI->FTSR |= (1u<<6) | (1u<<7);
    EXTI->PR   |= (1u<<6) | (1u<<7);
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    /* =========================
     * 6) TIMERS del sistema
     * ========================= */

    // TIM21 @ 1 kHz → multiplex 7-seg + scan keypad
    TIM21->PSC  = 16000 - 1;   // 16 MHz / 16000 = 1 kHz
    TIM21->ARR  = 1 - 1;       // update cada 1 ms
    TIM21->DIER = 1u;          // UIE
    NVIC_SetPriority(TIM21_IRQn, 2);
    NVIC_EnableIRQ(TIM21_IRQn);
    TIM21->CR1  = 1u;          // CEN

    // TIM22 @ 100 Hz → LCD + FSM general
    TIM22->PSC  = 16000 - 1;   // 1 kHz base
    TIM22->ARR  = 10 - 1;      // 100 Hz
    TIM22->DIER = 1u;
    NVIC_SetPriority(TIM22_IRQn, 3);
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1  = 1u;

    // TIM2 @ 1 kHz → motores (pasos)
    TIM2->PSC   = 16000 - 1;   // 1 kHz
    TIM2->ARR   = 1 - 1;
    TIM2->DIER  = 1u;
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1   = 1u;

    // TIM1 (PWM buzzer en PA8, CH1) @ 2 kHz, duty 0% (silencio)
    TIM1->PSC   = 16 - 1;      // 1 MHz
    TIM1->ARR   = 500 - 1;     // 2 kHz
    TIM1->CCR1  = 0;           // duty 0% (apagado)
    TIM1->CCMR1 &= ~(0xFFu);
    TIM1->CCMR1 |=  (0x6u << 4);   // PWM mode 1 en CH1
    TIM1->CCER  &= ~(1u<<0);       // CH1 deshabilitado inicialmente
    TIM1->BDTR  |=  (1u<<15);      // MOE
    TIM1->CR1   |=  (1u<<0);       // CEN

    /* =========================
     * 7) Estados iniciales seguros
     * ========================= */
    // 7-seg: apaga segmentos y dígitos
    GPIOB->BSRR = (0x7Fu << 16);   // PB0..PB6 OFF
    GPIOC->BSRR = (0x0Fu << 16);   // PC0..PC3 OFF

    // Motores OFF
    GPIOB->BSRR = (0x0Fu << (12+16)); // PB12..PB15
    GPIOC->BSRR = (0x0Fu << (8+16));  // PC8..PC11

    // Buzzer OFF (CCR1=0 ya está). CH1 queda deshabilitado.
    // Filas keypad en alto ya puesto arriba.

    __enable_irq();

    // Inicializa LCD y muestra estado inicial
    LCD_Init();
    ClearCodeBufferAndLCDLine1();   // pone "Codigo: "
    strcpy(g_lcd_line2_text, "Credito: Q0.00");
    g_lcd_should_refresh = 1;

}

// -x-x-x-x-x- Funciones Push -x-x-x-x-x-
// BOTONES PA6 = Q0.25, PA7 = Q1.00
volatile float g_credit_total_q = 0.0f;           // Total en quetzales
#define CREDIT_MAX_Q  (99.75f)

char g_lcd_line2_text[32] = "Credito: Q00.00";   // Texto para la 2da línea de la LCD
volatile uint8_t g_lcd_should_refresh = 0;       // Bandera para pedir refresco de LCD

// Prototipo del log (debes tenerlo en tu módulo USART)
static inline void usart2_write_line(const char *s);

// Botón .25 Q (PA6 / EXTI6)
static inline void OnPushQuarterPressed(void)
{
    if ((g_credit_total_q + 0.25f) <= CREDIT_MAX_Q) {
        g_credit_total_q += 0.25f;
        usart2_write_line("Agregado Q0.25");
    } else {
        strcpy(g_lcd_line2_text, "ERROR: Max Q99.75");
        usart2_write_line("ERROR: Max Q99.75");
        g_lcd_should_refresh = 1;
        return;
    }
    UpdateLine2WithCreditStatus(); // ahora muestra suf./insuf. si hay selección, o crédito si no la hay
}

// Botón 1 Q (PA7 / EXTI7)
static inline void OnPushOneQuetzalPressed(void)
{
    if ((g_credit_total_q + 1.00f) <= CREDIT_MAX_Q) {
        g_credit_total_q += 1.00f;
        usart2_write_line("Agregado Q1.00");
    } else {
        strcpy(g_lcd_line2_text, "ERROR: Max Q99.75");
        usart2_write_line("ERROR: Max Q99.75");
        g_lcd_should_refresh = 1;
        return;
    }
    UpdateLine2WithCreditStatus();
}

// Handler EXTI para PA6 y PA7
void EXTI4_15_IRQHandler(void)
{
    if (EXTI->PR & (1u<<6)) {      // PA6 → .25Q
        OnPushQuarterPressed();
        EXTI->PR |= (1u<<6);       // limpiar pendiente
    }
    if (EXTI->PR & (1u<<7)) {      // PA7 → 1Q
        OnPushOneQuetzalPressed();
        EXTI->PR |= (1u<<7);       // limpiar pendiente
    }
}
// -x-x-x-x-x- Funciones LCD -x-x-x-x-x-
// Pines: D4..D7 = PC4..PC7, E = PB10, RS = PB11

// Estado LCD
char g_lcd_line1_text[17] = "Dispensadora   "; // 16 chars + '\0'
extern char g_lcd_line2_text[32];              // definida arriba
extern volatile uint8_t g_lcd_should_refresh;

// Pulsar E
static inline void LCD_PulseEnable(void)
{
    GPIOB->BSRR = (1u<<10);        // E=1
    GPIOB->BSRR = (1u<<(10+16));   // E=0
}

// Escribir 4 bits en D4..D7
static inline void LCD_Write4Bits(uint8_t nibble4)
{
    GPIOC->BSRR = ((0x0Fu << 4) << 16);           // limpia PC4..PC7
    GPIOC->BSRR = ((uint32_t)(nibble4 & 0x0F)) << 4;
    LCD_PulseEnable();
}

// Enviar comando
void LCD_SendCommand(uint8_t cmd)
{
    GPIOB->BSRR = (1u<<(11+16));   // RS=0 (comando)
    LCD_Write4Bits((cmd >> 4) & 0x0F);
    LCD_Write4Bits(cmd & 0x0F);
}

// Enviar carácter
void LCD_WriteChar(uint8_t c)
{
    GPIOB->BSRR = (1u<<11);        // RS=1 (datos)
    LCD_Write4Bits((c >> 4) & 0x0F);
    LCD_Write4Bits(c & 0x0F);
}

// Enviar string (hasta max_len chars si se desea limitar)
void LCD_WriteStringMax(const char *s, uint8_t max_len)
{
    uint8_t count = 0;
    while (*s && (count < max_len)) {
        LCD_WriteChar((uint8_t)*s++);
        count++;
    }
}

// Inicializar LCD (llamar una vez tras SystemInit)
void LCD_Init(void)
{
    // RS=0, E=0, D4..D7=0
    GPIOB->BSRR = (1u<<(11+16)) | (1u<<(10+16));
    GPIOC->BSRR = ((0x0Fu<<4) << 16);

    // Secuencia 4-bit wake-up
    LCD_Write4Bits(0x3);
    LCD_Write4Bits(0x3);
    LCD_Write4Bits(0x3);
    LCD_Write4Bits(0x2);           // 4-bit

    LCD_SendCommand(0x28);         // 4-bit, 2 líneas, 5x8
    LCD_SendCommand(0x0C);         // Display ON, cursor OFF
    LCD_SendCommand(0x01);         // Clear
    LCD_SendCommand(0x06);         // Entry mode: inc, no shift
}

// ISR de refresco (llamar desde TIM “lento”, p.ej. TIM22 @100 Hz)
void LCD_RefreshIfRequested(void)
{
    if (!g_lcd_should_refresh) return;

    // Línea 1 (posición 0)
    LCD_SendCommand(0x80);                     // DDRAM line1
    LCD_WriteStringMax(g_lcd_line1_text, 16);

    // Línea 2 (posición 0)
    LCD_SendCommand(0xC0);                     // DDRAM line2
    LCD_WriteStringMax(g_lcd_line2_text, 16);  // truncar suavemente

    g_lcd_should_refresh = 0;
}

// -x-x-x-x-x- Funciones Keypad -x-x-x-x-x-

// ================== KEYPAD (2 filas x 4 columnas) ==================
// Columnas (entradas con pull-up): PA0, PA1, PA4, PA5
// Filas (salidas): PA15 (fila 0), PB9 (fila 1)
// Layout asumido:
//   Fila 0 (PA15 baja): [1] [2] [3] [A]
//   Fila 1 (PB9  baja): [4] [5] [6] [B]

volatile uint8_t g_keypad_col_fsm = 0;   // 0..3 (columna activa)
volatile uint8_t g_keypad_row_fsm = 0;   // 0=fila0 (PA15 baja), 1=fila1 (PB9 baja)

uint8_t g_product_code_digits[3] = {0,0,0};
uint8_t g_product_code_len = 0;          // 0..3

extern char g_lcd_line1_text[17];
extern volatile uint8_t g_lcd_should_refresh;

// Prototipos (se implementarán luego)
static inline void OnKeypadAccept(void);   // tecla A
static inline void OnKeypadCancel(void);   // tecla B
static inline void usart2_write_line(const char *s);

// --- Utilidad: actualizar la línea 1 con el código ingresado ---
static void UpdateLcdLine1WithCode(void)
{
    // Construye "Codigo: xxx" con padding
    // Si hay <3 dígitos, muestra los ingresados.
    char temp[17] = "Codigo: ";
    uint8_t idx = 8; // después de "Codigo: "
    for (uint8_t i = 0; i < g_product_code_len && idx < 16; ++i) {
        temp[idx++] = (char)('0' + g_product_code_digits[i]);
    }
    while (idx < 16) temp[idx++] = ' ';
    temp[16] = '\0';

    // Copia a la línea 1 y pide refresco
    for (uint8_t i = 0; i < 17; ++i) g_lcd_line1_text[i] = temp[i];
    g_lcd_should_refresh = 1;
}

// --- Utilidad: agregar dígito si hay espacio ---
static inline void Keypad_AppendDigit(uint8_t digit)
{
    if (g_product_code_len < 3) {
        g_product_code_digits[g_product_code_len++] = digit;
        UpdateLcdLine1WithCode();
        // Log simple
        char msg[24];
        sprintf(msg, "Key %u", digit);
        usart2_write_line(msg);

        if (g_product_code_len == 3) {
            OnThreeDigitsCompleted_UpdateSelection(); // <-- IMPORTANTE
        }
    }
}

// --- Lectura de columna activa (col_idx = 0..3) ---
static inline uint8_t Keypad_IsColumnLow(uint8_t col_idx)
{
    // Mapea a pines PA0, PA1, PA4, PA5
    static const uint8_t col_pin[4] = {0, 1, 4, 5};
    return ( (GPIOA->IDR & (1u << col_pin[col_idx])) == 0 );
}

// --- Activación de filas: pone una fila en 0 y la otra en 1 ---
static inline void Keypad_SetActiveRow(uint8_t row) // 0=fila0 (PA15), 1=fila1 (PB9)
{
    if (row == 0) {
        // Fila0 activa (PA15=0), Fila1 inactiva (PB9=1)
        GPIOA->BSRR = (1u << (15+16));  // PA15 = 0
        GPIOB->BSRR = (1u << 9);        // PB9  = 1
    } else {
        // Fila1 activa (PB9=0), Fila0 inactiva (PA15=1)
        GPIOB->BSRR = (1u << (9+16));   // PB9  = 0
        GPIOA->BSRR = (1u << 15);       // PA15 = 1
    }
}

// --- FSM de escaneo por columnas/filas (llamar desde TIM21 @1kHz p.ej.) ---
void Keypad_ScanStep(void)
{
    switch (g_keypad_col_fsm) {

        case 0: // Columna 0 (PA0): teclas 1 / 4
        {
            Keypad_SetActiveRow(g_keypad_row_fsm);
            if (Keypad_IsColumnLow(0)) {
                if (g_keypad_row_fsm == 0) {      // fila 0 → '1'
                    Keypad_AppendDigit(1);
                } else {                           // fila 1 → '4'
                    Keypad_AppendDigit(4);
                }
            }
            // Avanza a la otra fila y luego cambia de columna
            if (g_keypad_row_fsm == 0) g_keypad_row_fsm = 1;
            else { g_keypad_row_fsm = 0; g_keypad_col_fsm = 1; }
        } break;

        case 1: // Columna 1 (PA1): teclas 2 / 5
        {
            Keypad_SetActiveRow(g_keypad_row_fsm);
            if (Keypad_IsColumnLow(1)) {
                if (g_keypad_row_fsm == 0) {      // fila 0 → '2'
                    Keypad_AppendDigit(2);
                } else {                           // fila 1 → '5'
                    Keypad_AppendDigit(5);
                }
            }
            if (g_keypad_row_fsm == 0) g_keypad_row_fsm = 1;
            else { g_keypad_row_fsm = 0; g_keypad_col_fsm = 2; }
        } break;

        case 2: // Columna 2 (PA4): teclas 3 / 6
        {
            Keypad_SetActiveRow(g_keypad_row_fsm);
            if (Keypad_IsColumnLow(2)) {
                if (g_keypad_row_fsm == 0) {      // fila 0 → '3'
                    Keypad_AppendDigit(3);
                } else {                           // fila 1 → '6'
                    Keypad_AppendDigit(6);
                }
            }
            if (g_keypad_row_fsm == 0) g_keypad_row_fsm = 1;
            else { g_keypad_row_fsm = 0; g_keypad_col_fsm = 3; }
        } break;

        case 3: // Columna 3 (PA5): teclas A / B
        {
            Keypad_SetActiveRow(g_keypad_row_fsm);
            if (Keypad_IsColumnLow(3)) {
                if (g_keypad_row_fsm == 0) {      // fila 0 → 'A' (Aceptar)
                    usart2_write_line("Key A (Aceptar)");
                    OnKeypadAccept();
                } else {                           // fila 1 → 'B' (Cancelar)
                    usart2_write_line("Key B (Cancelar)");
                    OnKeypadCancel();
                }
            }
            if (g_keypad_row_fsm == 0) g_keypad_row_fsm = 1;
            else { g_keypad_row_fsm = 0; g_keypad_col_fsm = 0; }
        } break;
    }
}



// ================== 7-SEG (4 dígitos, multiplex) ==================
// Segs a..g = PB0..PB6 (1 = encendido)
// Dígitos   = PC0..PC3  (1 = habilitado)
// Punto decimal: cableado a VDD (siempre encendido, no se controla por software)

extern volatile float g_credit_total_q;   // total acumulado (Q)
// Llama SevenSeg_RefreshStep() desde un timer (p.ej. TIM21 @1 kHz)

static inline uint8_t SevenSeg_EncodeDigit(uint8_t d)
{
    // bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g
    switch (d) {
        case 0: return 0x3F; // a b c d e f
        case 1: return 0x06; //   b c
        case 2: return 0x5B; // a b   d e   g
        case 3: return 0x4F; // a b c d     g
        case 4: return 0x66; //   b c     f g
        case 5: return 0x6D; // a   c d   f g
        case 6: return 0x7D; // a   c d e f g
        case 7: return 0x07; // a b c
        case 8: return 0x7F; // a b c d e f g
        case 9: return 0x6F; // a b c d   f g
        default: return 0x00; // apagado
    }
}

volatile uint8_t g_7seg_phase = 0;  // 0..3
static uint8_t s_digits[4] = {0,0,0,0}; // buffer de dígitos a mostrar

static inline void SevenSeg_LoadDigitsFromCredit(void)
{
    // Formato XX.XX → tomamos g_credit_total_q en centavos (redondeo)
    int value_cents = (int)(g_credit_total_q * 100.0f + 0.5f);
    if (value_cents < 0) value_cents = 0;
    if (value_cents > 9999) value_cents = 9999; // seguridad (máx 99.99)

    // pos0 pos1 . pos2 pos3   (XX.XX)
    s_digits[0] = (uint8_t)(value_cents / 1000);             // decenas (entero)
    s_digits[1] = (uint8_t)((value_cents / 100) % 10);       // unidades (entero)
    s_digits[2] = (uint8_t)((value_cents / 10)  % 10);       // décimas
    s_digits[3] = (uint8_t)( value_cents        % 10);       // centésimas
}

void SevenSeg_RefreshStep(void)
{
    // Actualiza el buffer cada vuelta completa (fase 0)
    if (g_7seg_phase == 0) {
        SevenSeg_LoadDigitsFromCredit();
    }

    // Apaga todo: segmentos y dígitos
    GPIOB->BSRR = (0x7Fu << 16);  // PB0..PB6 = 0
    GPIOC->BSRR = (0x0Fu << 16);  // PC0..PC3 = 0

    // Escribe segmentos del dígito actual
    uint8_t segmask = SevenSeg_EncodeDigit(s_digits[g_7seg_phase]);
    GPIOB->BSRR = segmask;        // pone a..g

    // Habilita el dígito correspondiente (activo en 1)
    GPIOC->BSRR = (1u << g_7seg_phase);

    // Avanza fase 0→1→2→3→0
    g_7seg_phase = (g_7seg_phase + 1) & 0x03;
}

// ================== MOTORES DISPENSADORES (Stepper, una sola dirección) ==================
// Snack A → PB12..PB15
// Snack B → PC8..PC11
// Llamar DispenseSnackA_Start(steps) o DispenseSnackB_Start(steps) cuando se apruebe la compra.
// Llamar DispenseMotors_StepISR() desde el timer de pasos (p.ej. TIM2 @1 kHz).

// --- Estado global de motores ---
volatile uint8_t  g_snackA_is_moving = 0;
volatile uint8_t  g_snackA_fsm_step  = 1;     // 1..4
volatile uint16_t g_snackA_steps_rem = 0;

volatile uint8_t  g_snackB_is_moving = 0;
volatile uint8_t  g_snackB_fsm_step  = 1;     // 1..4
volatile uint16_t g_snackB_steps_rem = 0;

// LCD y Log
extern char g_lcd_line2_text[32];
extern volatile uint8_t g_lcd_should_refresh;
static inline void usart2_write_line(const char *s);

// --- API de arranque ---
void DispenseSnackA_Start(uint16_t steps_to_run)
{
    if (steps_to_run == 0) return;
    g_snackA_fsm_step  = 1;
    g_snackA_steps_rem = steps_to_run;
    g_snackA_is_moving = 1;

    strcpy(g_lcd_line2_text, "Dispensando A...");
    g_lcd_should_refresh = 1;
    usart2_write_line("Dispensando Snack A");
}

void DispenseSnackB_Start(uint16_t steps_to_run)
{
    if (steps_to_run == 0) return;
    g_snackB_fsm_step  = 1;
    g_snackB_steps_rem = steps_to_run;
    g_snackB_is_moving = 1;

    strcpy(g_lcd_line2_text, "Dispensando B...");
    g_lcd_should_refresh = 1;
    usart2_write_line("Dispensando Snack B");
}

// --- Paso de motores (ISR de timer) ---
void DispenseMotors_StepISR(void)
{
    // ------- Snack A (PB12..PB15) -------
    if (g_snackA_is_moving) {
        if (g_snackA_steps_rem > 0) g_snackA_steps_rem--;
        if (g_snackA_steps_rem == 0) {
            g_snackA_is_moving = 0;
            // Apaga bobinas
            GPIOB->BSRR = (0x0Fu << (12+16));
            usart2_write_line("Snack A: listo");
            strcpy(g_lcd_line2_text, "Listo (A)");
            g_lcd_should_refresh = 1;
            Buzzer_BeepStart(300);  // p.ej. 300 ms
            
        } else {
            // Apaga bobinas
            GPIOB->BSRR = (0x0Fu << (12+16));
            // Secuencia 1→2→3→4 (una sola dirección)
            switch (g_snackA_fsm_step) {
                case 1: GPIOB->BSRR = (0b0001u << 12); g_snackA_fsm_step = 2; break;
                case 2: GPIOB->BSRR = (0b0010u << 12); g_snackA_fsm_step = 3; break;
                case 3: GPIOB->BSRR = (0b0100u << 12); g_snackA_fsm_step = 4; break;
                case 4: GPIOB->BSRR = (0b1000u << 12); g_snackA_fsm_step = 1; break;
                default: GPIOB->BSRR = (0b0001u << 12); g_snackA_fsm_step = 2; break;
            }
        }
    } else {
        // asegúrate off
        GPIOB->BSRR = (0x0Fu << (12+16));
    }

    // ------- Snack B (PC8..PC11) -------
    if (g_snackB_is_moving) {
        if (g_snackB_steps_rem > 0) g_snackB_steps_rem--;
        if (g_snackB_steps_rem == 0) {
            g_snackB_is_moving = 0;
            // Apaga bobinas
            GPIOC->BSRR = (0x0Fu << (8+16));
            usart2_write_line("Snack B: listo");
            strcpy(g_lcd_line2_text, "Listo (B)");
            g_lcd_should_refresh = 1;
            Buzzer_BeepStart(300);  // p.ej. 300 ms
        } else {
            // Apaga bobinas
            GPIOC->BSRR = (0x0Fu << (8+16));
            // Secuencia 1→2→3→4 (una sola dirección)
            switch (g_snackB_fsm_step) {
                case 1: GPIOC->BSRR = (0b0001u << 8); g_snackB_fsm_step = 2; break;
                case 2: GPIOC->BSRR = (0b0010u << 8); g_snackB_fsm_step = 3; break;
                case 3: GPIOC->BSRR = (0b0100u << 8); g_snackB_fsm_step = 4; break;
                case 4: GPIOC->BSRR = (0b1000u << 8); g_snackB_fsm_step = 1; break;
                default: GPIOC->BSRR = (0b0001u << 8); g_snackB_fsm_step = 2; break;
            }
        }
    } else {
        // asegúrate off
        GPIOC->BSRR = (0x0Fu << (8+16));
    }
}

// funcioens generales
// ================== LÓGICA GENERAL DE SELECCIÓN / ACEPTAR / CANCELAR ==================
//
// Códigos válidos:
//   Snack 1 → 354  (Q5.00)
//   Snack 2 → 612  (Q7.50)
//
// Integra con:
//   - g_product_code_digits[3], g_product_code_len (llenados por Keypad_ScanStep)
//   - g_credit_total_q (crédito actual)
//   - LCD: g_lcd_line1_text (producto y precio), g_lcd_line2_text (estado), g_lcd_should_refresh
//   - Motores: DispenseSnackA_Start(), DispenseSnackB_Start()
//   - Log: usart2_write_line(...)

#define CODE_SNACK1        354
#define CODE_SNACK2        612
#define PRICE_SNACK1_Q     5.00f
#define PRICE_SNACK2_Q     7.50f

// Selección actual
uint8_t g_selected_snack_id = 0;     // 0 = ninguno, 1 = Snack1, 2 = Snack2
float   g_selected_price_q  = 0.0f;

// Buffers y banderas LCD
extern char g_lcd_line1_text[17];
extern char g_lcd_line2_text[32];
extern volatile uint8_t g_lcd_should_refresh;

// Crédito y código ingresado
extern volatile float g_credit_total_q;
extern uint8_t g_product_code_digits[3];
extern uint8_t g_product_code_len;

// Motores y log
static inline void usart2_write_line(const char *s);
void DispenseSnackA_Start(uint16_t steps_to_run);
void DispenseSnackB_Start(uint16_t steps_to_run);


// -------- Utilidades internas --------
static inline int BuildCodeValueFromDigits(void)
{
    // Asume g_product_code_len == 3
    int c = 0;
    c = g_product_code_digits[0]*100
      + g_product_code_digits[1]*10
      + g_product_code_digits[2];
    return c;
}

static void ClearCodeBufferAndLCDLine1(void)
{
    g_product_code_len = 0;
    g_product_code_digits[0] = g_product_code_digits[1] = g_product_code_digits[2] = 0;

    // "Codigo: " + espacios
    const char *base = "Codigo: ";
    uint8_t i = 0;
    for (; base[i] && i < 16; ++i) g_lcd_line1_text[i] = base[i];
    for (; i < 16; ++i) g_lcd_line1_text[i] = ' ';
    g_lcd_line1_text[16] = '\0';
    g_lcd_should_refresh = 1;
}

static void ShowSelectionOnLCDLine1(uint8_t snack_id, int code, float price_q)
{
    // Formato: "Sel: 354 Q5.00  " (16 chars)
    // Construcción simple para mantener claridad
    char line[17] = "Sel: ";         // 5 chars para arrancar
    // Añadir código
    int idx = 5;
    line[idx++] = (char)('0' + (code/100)%10);
    line[idx++] = (char)('0' + (code/10)%10);
    line[idx++] = (char)('0' + (code%10));
    line[idx++] = ' ';
    line[idx++] = 'Q';

    // Precio con 2 decimales fijo
    int price_int   = (int)price_q;
    int price_cents = (int)((price_q - price_int) * 100.0f + 0.5f);
    if (price_cents >= 100) { price_int += 1; price_cents -= 100; }

    // entero
    if (price_int >= 10) {
        line[idx++] = (char)('0' + (price_int/10)%10);
        line[idx++] = (char)('0' + (price_int%10));
    } else {
        line[idx++] = (char)('0' + price_int);
    }
    line[idx++] = '.';
    line[idx++] = (char)('0' + (price_cents/10)%10);
    line[idx++] = (char)('0' + (price_cents%10));

    // Rellenar espacios
    while (idx < 16) line[idx++] = ' ';
    line[16] = '\0';

    // Copiar a la línea 1 real
    for (int i=0;i<17;i++) g_lcd_line1_text[i] = line[i];
    g_lcd_should_refresh = 1;
}


// -------- Llamar cuando se hayan ingresado los 3 dígitos --------
void OnThreeDigitsCompleted_UpdateSelection(void)
{
    if (g_product_code_len != 3) return;

    int code = BuildCodeValueFromDigits();

    if (code == CODE_SNACK1) {
        g_selected_snack_id = 1;
        g_selected_price_q  = PRICE_SNACK1_Q;
        ShowSelectionOnLCDLine1(1, CODE_SNACK1, PRICE_SNACK1_Q);
        usart2_write_line("Seleccion: Snack 1 (354)");
        UpdateLine2WithCreditStatus();
    }
    else if (code == CODE_SNACK2) {
        g_selected_snack_id = 2;
        g_selected_price_q  = PRICE_SNACK2_Q;
        ShowSelectionOnLCDLine1(2, CODE_SNACK2, PRICE_SNACK2_Q);
        usart2_write_line("Seleccion: Snack 2 (612)");
        UpdateLine2WithCreditStatus();
    }
    else {
        g_selected_snack_id = 0;
        g_selected_price_q  = 0.0f;
        strcpy(g_lcd_line2_text, "Codigo invalido");
        usart2_write_line("Error: codigo invalido");
        g_lcd_should_refresh = 1;
    }
}



// -------- ACEPTAR (tecla A) --------
void OnKeypadAccept(void)
{
    if (g_product_code_len != 3 || g_selected_snack_id == 0) {
        strcpy(g_lcd_line2_text, "Codigo invalido");
        usart2_write_line("Aceptar: codigo invalido");
        g_lcd_should_refresh = 1;
        return;
    }

    if (g_credit_total_q + 1e-6f < g_selected_price_q) {
        strcpy(g_lcd_line2_text, "Credito insuficiente");
        usart2_write_line("Aceptar: credito insuficiente");
        g_lcd_should_refresh = 1;
        return;
    }

    // Descuento y arranque
    g_credit_total_q -= g_selected_price_q;
    const uint16_t STEPS_TO_DISPENSE = 320;

    if (g_selected_snack_id == 1) {
        DispenseSnackA_Start(STEPS_TO_DISPENSE);
        usart2_write_line("Dispensando: Snack 1");
    } else {
        DispenseSnackB_Start(STEPS_TO_DISPENSE);
        usart2_write_line("Dispensando: Snack 2");
    }

    strcpy(g_lcd_line2_text, "Dispensando");
    g_lcd_should_refresh = 1;
}



// -------- CANCELAR (tecla B) --------
void OnKeypadCancel(void)
{
    g_credit_total_q    = 0.0f;
    g_selected_snack_id = 0;
    g_selected_price_q  = 0.0f;

    ClearCodeBufferAndLCDLine1();  // Pone "Codigo: "

    strcpy(g_lcd_line2_text, "Cancelado, Q0.00");
    usart2_write_line("Cancelado: credito a 0, sin seleccion");
    g_lcd_should_refresh = 1;
}

// ------ Helpers de formato y estado de crédito ------

static void FormatCreditToText(char *out16, float q) // escribe "Qxx.xx" en 16 chars (padded)
{
    int cents = (int)(q * 100.0f + 0.5f);
    if (cents < 0) cents = 0;
    int qint   = cents / 100;
    int qdec   = cents % 100;

    char tmp[16];
    int n = 0;
    tmp[n++] = 'Q';
    if (qint >= 10) {
        tmp[n++] = (char)('0' + (qint / 10) % 10);
        tmp[n++] = (char)('0' + (qint % 10));
    } else {
        tmp[n++] = (char)('0' + qint);
    }
    tmp[n++] = '.';
    tmp[n++] = (char)('0' + (qdec / 10) % 10);
    tmp[n++] = (char)('0' + (qdec % 10));
    while (n < 16) tmp[n++] = ' ';
    tmp[15] = '\0';

    // copia
    for (int i=0; i<16; ++i) out16[i] = tmp[i];
}

static void UpdateLine2WithCreditStatus(void)
{
    // Si hay selección válida, muestra “Credito suficiente/insuficiente”.
    if (g_selected_snack_id != 0) {
        if (g_credit_total_q + 1e-6f >= g_selected_price_q) {
            strcpy(g_lcd_line2_text, "Credito suficiente");
        } else {
            strcpy(g_lcd_line2_text, "Credito insuficiente");
        }
    } else {
        // Sin selección: muestra el crédito actual “Credito: Qxx.xx”
        char cred[16];
        FormatCreditToText(cred, g_credit_total_q);
        char line[32] = "Credito: ";
        // copiar "Qxx.xx" al final de "Credito: "
        int i = 9, j = 0;
        while (j < 6 && i < 31) line[i++] = cred[j++];
        line[i] = '\0';
        strcpy(g_lcd_line2_text, line);
    }
    g_lcd_should_refresh = 1;
}



// funciones USART

static inline void usart2_init(uint32_t baud)
{
    RCC->IOPENR  |= (1u<<0);          // GPIOA
    RCC->APB1ENR |= (1u<<17);         // USART2

    // PA2 (TX), PA3 (RX) en AF4
    GPIOA->MODER &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->MODER |=  ((0x2u<<(2*2)) | (0x2u<<(3*2)));
    GPIOA->AFR[0] &= ~((0xFu<<(2*4)) | (0xFu<<(3*4)));
    GPIOA->AFR[0] |=  ((0x4u<<(2*4)) | (0x4u<<(3*4)));

    // Opcional: alta velocidad, push-pull, sin pull-ups
    GPIOA->OSPEEDR |= (0x3u<<(2*2)) | (0x3u<<(3*2));
    GPIOA->PUPDR   &= ~((0x3u<<(2*2)) | (0x3u<<(3*2)));
    GPIOA->OTYPER  &= ~((1u<<2) | (1u<<3));

    // Core USART2 @ 16 MHz HSI16
    USART2->CR1 = 0;                 // limpia config
    USART2->BRR = 16000000u / baud;  // oversampling x16
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->CR1 |= (1u<<3) | (1u<<2); // TE | RE (si no usarás RX, quita RE)
    USART2->CR1 |= (1u<<0);           // UE
}

static inline void usart2_putc(char c)
{
    while(!(USART2->ISR & (1u<<7)));  // TXE
    USART2->TDR = (uint8_t)c;
}

static inline void usart2_write(const char *s)
{
    while(*s) usart2_putc(*s++);
}

static inline void usart2_write_line(const char *s)
{
    usart2_write(s);
    usart2_write("\r\n");
}

// ================== BUZZER (PA8, PWM con TIM1_CH1) ==================
// Beep al terminar de expulsar el snack (llámalo cuando finaliza el motor).
// Usa TIM1 ya configurado en SystemInit (PA8 AF2). El “tick” esperado es 1 ms
// desde tu timer de sistema (p.ej., TIM2 o TIM22).

volatile uint8_t  g_buzzer_is_on = 0;
volatile uint16_t g_buzzer_ms_left = 0;

// prototipo de log
static inline void usart2_write_line(const char *s);

// Inicia un beep de 'duration_ms' a ~2 kHz con ~50% duty
static inline void Buzzer_BeepStart(uint16_t duration_ms)
{
    if (duration_ms == 0) return;

    g_buzzer_ms_left = duration_ms;
    g_buzzer_is_on   = 1;

    // TIM1 ya inicializado en SystemInit:
    // ARR = 500-1 (2 kHz), PSC = 16-1 (1 MHz)
    TIM1->CCR1 = (TIM1->ARR + 1) / 2; // ~50% duty
    TIM1->CCER |= 1u;                  // habilita CH1 (PA8 suena)

    usart2_write_line("Buzzer: beep iniciado");
}

// Paso de buzzer (llamar cada 1 ms desde tu ISR de timer “lento” o de motores)
static inline void Buzzer_BeepStep_1ms(void)
{
    if (!g_buzzer_is_on) return;

    if (g_buzzer_ms_left > 0) {
        g_buzzer_ms_left--;
    }

    if (g_buzzer_ms_left == 0) {
        g_buzzer_is_on = 0;
        TIM1->CCER &= ~1u;   // deshabilita CH1 (silencio)
        TIM1->CCR1  = 0;
        usart2_write_line("Buzzer: beep finalizado");
    }
}

// ================== INTERRUPCIONES - DISPENSADORA ==================

// TIM21 @1 kHz: 7-seg multiplex + Keypad scan
void TIM21_IRQHandler(void)
{
    SevenSeg_RefreshStep();   // muestra g_credit_total_q en XX.XX
    Keypad_ScanStep();        // arma g_product_code_digits[] y teclas A/B
    TIM21->SR &= ~1u;         // UIF = 0
}

// TIM22 @100 Hz: LCD (si hay bandera)
void TIM22_IRQHandler(void)
{
    LCD_RefreshIfRequested(); // usa g_lcd_should_refresh, g_lcd_line1/2_text
    TIM22->SR &= ~1u;
}

// TIM2 @1 kHz: motores + buzzer tick 1ms
void TIM2_IRQHandler(void)
{
    DispenseMotors_StepISR(); // stepper A/B, detecta fin y lanza Buzzer_BeepStart(...)
    Buzzer_BeepStep_1ms();    // cuenta atrás del beep (PA8, TIM1_CH1)
    TIM2->SR &= ~1u;
}

// EXTI4_15: PA6 (.25Q) y PA7 (1Q) con debounce físico
void EXTI4_15_IRQHandler(void)
{
    if (EXTI->PR & (1u<<6)) {      // PA6 → .25Q
        OnPushQuarterPressed();
        EXTI->PR |= (1u<<6);
    }
    if (EXTI->PR & (1u<<7)) {      // PA7 → 1Q
        OnPushOneQuetzalPressed();
        EXTI->PR |= (1u<<7);
    }
}


// -x-x-x-x- Main e Init -x-x-x-x-

int main(void) {
    // Inicializaciones de periféricos
    SystemInit();

    while (1) {
    }
}
