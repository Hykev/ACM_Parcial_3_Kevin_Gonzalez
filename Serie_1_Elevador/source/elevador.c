#include <stdint.h>
#include <stdio.h>
#include "stm32l053xx.h"

/* ======================== Estados y variables ======================== */
typedef enum { IDLE=0, MOVING_UP, MOVING_DOWN, HOLD_AT_FLOOR } state_t;
static volatile state_t state = IDLE;

static volatile uint8_t  currentFloor = 1;     // 1..3
static volatile uint8_t  destFloor    = 0;     // destino “activo”
static volatile uint8_t  q[3];
static volatile uint8_t  qn = 0;
static volatile uint8_t  pending_mask = 0;     // bit por piso solicitado

/* -------- Movimiento ascensor piso-a-piso por PASOS -------- */
static volatile int8_t   moving_dir = 0;       // -1 baja, +1 sube
static volatile uint8_t  hop_target = 0;       // piso intermedio al que vamos
static volatile uint32_t hop_steps_remaining = 0;   // pasos restantes del hop
static volatile uint8_t  hop_done_flag = 0;    // lo levanta TIM2 al terminar pasos

/* ================= Door FSM (no bloqueante) ================= */
typedef enum { DOOR_IDLE=0, DOOR_OPENING, DOOR_OPEN_HOLD, DOOR_CLOSING, DOOR_DONE } door_state_t;
static volatile door_state_t door_state = DOOR_IDLE;

/* Calibración puerta */
static volatile uint32_t door_steps_open  = 1600;   // pasos para abrir (AJUSTABLE)
static volatile uint32_t door_steps_close = 1800;   // pasos para cerrar (AJUSTABLE)
static volatile uint16_t door_rate_hz     = 500;   // velocidad pasos/seg (AJUSTABLE)
static volatile uint16_t door_hold_ms_cfg = 3000;  // tiempo abierta (ms) (AJUSTABLE)
static volatile uint16_t door_hold_ms     = 0;     // contador runtime

/* Señal cuándo termina un tramo (open/close) */
static volatile uint8_t  door_step_done_flag = 0;

/* ================= Animación (fase 0..2) =================
   Subiendo: e->g->b;  Bajando: b->g->e. Se actualiza cada 100 ms (TIM21). */
static volatile uint8_t  anim_phase   = 0;
static volatile uint16_t anim_tick_ms = 0;

/* =================== Buzzer por GPIO en PA8 =================== */
static volatile uint16_t buzzer_ms     = 0;    // ms restantes del beep
static volatile uint8_t  buzzer_active = 0;    // 0=OFF,1=ON
static volatile uint16_t buzzer_hz     = 2000; // tono (Hz) configurable

/* ================== Stepper ASCENSOR (PB12..PB15) ==================
   - Full-step de 4 fases
   - Rate por defecto 400 Hz (configurable)
   - “Horario” (subir) = 1,2,3,4; “Antihorario” (bajar) = 4,3,2,1
*/
static volatile uint16_t stepper_rate_hz = 400;   // pasos/segundo
static volatile uint8_t  stepper_phase   = 0;     // 0..3
static volatile uint8_t  stepper_enabled = 0;

/* ================== Stepper PUERTA (PC8..PC11) ==================
   - Full-step 4 fases (mismo patrón)
   - “Horario” = abrir; “Antihorario” = cerrar
*/
static volatile uint8_t  door_stepper_enabled = 0;
static volatile int8_t   door_moving_dir = 0;     // +1 abrir (horario), -1 cerrar (antihorario)
static volatile uint8_t  door_phase = 0;          // 0..3
static volatile uint32_t door_steps_remaining = 0;

/* ============= 7-seg LUT (a,b,c,d,e,g → PB0..PB5) ============= */
static const uint8_t seg_digit_lut[10] = {
    /*0*/ (1u<<0)|(1u<<1)|(1u<<2)|(1u<<3)|(1u<<4),
    /*1*/ (1u<<1)|(1u<<2),
    /*2*/ (1u<<0)|(1u<<1)|(1u<<3)|(1u<<4)|(1u<<5),
    /*3*/ (1u<<0)|(1u<<1)|(1u<<2)|(1u<<3)|(1u<<5),
    /*4*/ (1u<<1)|(1u<<2)|(1u<<5),
    /*5*/ (1u<<0)|(1u<<2)|(1u<<3)|(1u<<5),
    /*6*/ (1u<<0)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5),
    /*7*/ (1u<<0)|(1u<<1)|(1u<<2),
    /*8*/ (1u<<0)|(1u<<1)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5),
    /*9*/ (1u<<0)|(1u<<1)|(1u<<2)|(1u<<3)|(1u<<5)
};

/* ======= Pasos por salto entre pisos (calibrables) ======= */
static volatile uint32_t steps_between[4][4] = {
    {0,    0,    0,    0},
    {0,    0, 2400,    0},   // 1->2
    {0, 2400,    0, 2400},   // 2->1, 2->3
    {0,    0, 2400,    0}    // 3->2
};


/* ===================== LCD 16x2 (HD44780, 4-bit, no bloqueante) ===================== */
/* Pines: PC4..PC7 = D4..D7, PB10 = E, PB11 = RS */

static volatile uint8_t  lcd_inited = 0;
static volatile uint16_t lcd_busy_ms = 0;      // espera entre bytes (≈2ms normal, 3ms clear/home)
static volatile uint8_t  lcd_dirty   = 1;      // hay que refrescar contenido
static volatile uint16_t lcd_poweron_ms = 20;  // espera al encender (>15ms)

/* Cola TX (byte + RS bit en el bit7 de 'rsflag' para simplificar) */
#define LCDQ_SZ 64u
static volatile uint8_t lcd_q[LCDQ_SZ];
static volatile uint8_t lcd_q_rs[LCDQ_SZ];
static volatile uint8_t lcd_ql=0, lcd_qr=0;

static inline uint8_t LCDQ_Empty(void){ return lcd_ql==lcd_qr; }
static inline uint8_t LCDQ_Full(void){ return (uint8_t)((lcd_qr+1u)&(LCDQ_SZ-1u))==lcd_ql; }
static void LCDQ_Push(uint8_t b, uint8_t rs){
    uint8_t n=(uint8_t)((lcd_qr+1u)&(LCDQ_SZ-1u)); if(n==lcd_ql) return;
    lcd_q[lcd_qr]=b; lcd_q_rs[lcd_qr]=rs; lcd_qr=n;
}
static uint8_t LCDQ_Pop(uint8_t *b, uint8_t *rs){
    if (LCDQ_Empty()) return 0u;
    *b = lcd_q[lcd_ql]; *rs = lcd_q_rs[lcd_ql];
    lcd_ql = (uint8_t)((lcd_ql+1u)&(LCDQ_SZ-1u));
    return 1u;
}

/* Low-level: escribir nibble (alto/ bajo) + pulso E */
static inline void LCD_WriteNibble(uint8_t nibble, uint8_t rs){
    /* RS */
    if (rs) GPIOB->BSRR = (1u<<11); else GPIOB->BSRR = (1u<<(11+16));
    /* D4..D7 (PC4..PC7) */
    uint32_t mask = (1u<<4)|(1u<<5)|(1u<<6)|(1u<<7);
    uint32_t bits = ((nibble & 0x01)?(1u<<4):0) |
                    ((nibble & 0x02)?(1u<<5):0) |
                    ((nibble & 0x04)?(1u<<6):0) |
                    ((nibble & 0x08)?(1u<<7):0);
    GPIOC->BSRR = (mask<<16);  /* clear */
    GPIOC->BSRR = bits;        /* set   */
    /* E pulse (PB10) con pequeño gap por software */
    GPIOB->BSRR = (1u<<10);
    for (volatile int i=0;i<12;i++) __NOP();   // ≈ sub-µs, suficiente para ancho de pulso
    GPIOB->BSRR = (1u<<(10+16));
}

/* Enviar byte (alto→bajo) */
static inline void LCD_WriteByte(uint8_t b, uint8_t rs){
    LCD_WriteNibble((uint8_t)((b>>4)&0x0F), rs);
    LCD_WriteNibble((uint8_t)(b & 0x0F), rs);
}

/* API no bloqueante: encola comandos/datos */
static inline void LCD_Cmd(uint8_t cmd){
    LCDQ_Push(cmd, 0u);
}
static inline void LCD_Data(uint8_t d){
    LCDQ_Push(d, 1u);
}

/* Helpers de alto nivel (no bloqueantes) */
static inline void LCD_Clear(void){ LCD_Cmd(0x01); }                     // clear
static inline void LCD_Home(void){  LCD_Cmd(0x02); }                     // return home
static inline void LCD_SetCursor(uint8_t row, uint8_t col){              // row:0..1, col:0..15
    uint8_t addr = (uint8_t)((row?0x40:0x00) + (col & 0x0F));
    LCD_Cmd((uint8_t)(0x80 | addr));
}
static void LCD_Print(const char *s){
    while(*s && !LCDQ_Full()) { LCD_Data((uint8_t)*s++); }
}

/* Lanzar secuencia de init 4-bit (asíncrona) */
static void LCD_InitStart(void){
    lcd_inited = 0;
    lcd_poweron_ms = 20;   // >15ms
    lcd_busy_ms    = 0;
    /* la secuencia se ejecuta en tim21_isr_body() */
}

/* Servicio 1ms: power-on wait, init, y drenado de cola con tiempos seguros */
static void LCD_Service_1ms(void){
    if (!lcd_inited){
        if (lcd_poweron_ms){ lcd_poweron_ms--; return; }

        /* Init 4-bit estándar */
        static uint8_t init_step = 0;
        if (lcd_busy_ms){ lcd_busy_ms--; return; }

        switch(init_step){
        case 0: LCD_WriteNibble(0x03, 0); lcd_busy_ms=5; init_step++; break;   // 0x30
        case 1: LCD_WriteNibble(0x03, 0); lcd_busy_ms=5; init_step++; break;   // 0x30
        case 2: LCD_WriteNibble(0x03, 0); lcd_busy_ms=5; init_step++; break;   // 0x30
        case 3: LCD_WriteNibble(0x02, 0); lcd_busy_ms=5; init_step++; break;   // 0x20 (4-bit)
        case 4: LCD_WriteByte(0x28, 0);   lcd_busy_ms=2; init_step++; break;   // 2 líneas, 5x8
        case 5: LCD_WriteByte(0x0C, 0);   lcd_busy_ms=2; init_step++; break;   // display ON, cursor OFF
        case 6: LCD_WriteByte(0x06, 0);   lcd_busy_ms=2; init_step++; break;   // entry mode
        case 7: LCD_WriteByte(0x01, 0);   lcd_busy_ms=3; init_step++; break;   // clear
        default:
            lcd_inited = 1;
            lcd_dirty  = 1;
            break;
        }
        return;
    }

    /* Si estamos inicializados: servir cola TX respetando tiempos */
    if (lcd_busy_ms){ lcd_busy_ms--; return; }

    uint8_t b, rs;
    if (LCDQ_Pop(&b, &rs)){
        LCD_WriteByte(b, rs);
        /* tiempos: clear/home tardan más */
        if (b==0x01 || b==0x02) lcd_busy_ms = 3;
        else                    lcd_busy_ms = 2;
    }
}

/* ===== Render de líneas ===== */
static void LCD_RequestRefresh(void){ lcd_dirty = 1; }

/* Devuelve texto de estado (16 chars máx) */
static const char* ui_state_text(void){
    /* Prioridad: mostrar estado de puerta cuando estamos en HOLD_AT_FLOOR */
    if (state == HOLD_AT_FLOOR){
        switch(door_state){
            case DOOR_OPENING:    return "Abriendo";
            case DOOR_OPEN_HOLD:  return "Puerta Abierta";
            case DOOR_CLOSING:    return "Cerrando";
            default:              return "Esperando";
        }
    }

    switch(state){
        case MOVING_UP:     return "Subiendo";
        case MOVING_DOWN:   return "Bajando";
        case IDLE:          return "Esperando";
        default:            return "Esperando";
    }
}

/* Envía a la LCD las dos líneas actuales */
static void LCD_RefreshNow(void){
    if (!lcd_inited) return;

    char line1[17]; char line2[17];
    /* Línea 1: estado, padded a 16 */
    const char* s = ui_state_text();
    uint8_t i=0; while(s[i] && i<16){ line1[i]=s[i]; i++; }
    while(i<16){ line1[i++]=' '; }
    line1[16]=0;

    /* Línea 2: "Cola: [dest q0 q1]"  (incluye destino actual al frente si va en movimiento) */
    char tmp[16]; uint8_t p=0;
    uint8_t buf[3]; uint8_t cnt = 0;

    /* Si estamos moviéndonos, anteponer el destino actual */
    if ((state==MOVING_UP || state==MOVING_DOWN) && destFloor>=1 && destFloor<=3){
        buf[cnt++] = destFloor;
    }
    /* Luego, los elementos de la cola restante (máx hasta completar 3) */
    for (uint8_t k=0; k<qn && cnt<3; k++){
        buf[cnt++] = q[k];
    }

    /* Formateo: "Cola: [x y z]" */
    p += (uint8_t)snprintf(&tmp[p], sizeof(tmp)-p, "Cola: [");
    for (uint8_t i=0; i<cnt && p<sizeof(tmp)-3; i++){
        p += (uint8_t)snprintf(&tmp[p], sizeof(tmp)-p, (i? " %u":"%u"), (unsigned)buf[i]);
    }
    if (p<sizeof(tmp)-1){ tmp[p++]=']'; }
    tmp[(p<sizeof(tmp))?p:(sizeof(tmp)-1)] = 0;

    i=0; while(tmp[i] && i<16){ line2[i]=tmp[i]; i++; }
    while(i<16){ line2[i++]=' '; }
    line2[16]=0;

    /* Enviar a LCD (no bloqueante) */
    LCD_SetCursor(0,0); LCD_Print(line1);
    LCD_SetCursor(1,0); LCD_Print(line2);
    lcd_dirty = 0;
}


/* ====== API de configuración rápida ====== */
static inline void set_steps_between(uint8_t from, uint8_t to, uint32_t steps){
    if (from>=1 && from<=3 && to>=1 && to<=3 && from!=to) steps_between[from][to] = steps;
}
static inline void set_stepper_rate(uint16_t rate_hz){ if (rate_hz) stepper_rate_hz = rate_hz; }
static inline void set_buzzer_tone(uint16_t hz){ if (hz) buzzer_hz = hz; }

/* ====== API puerta (calibración rápida) ====== */
static inline void set_door_steps_open (uint32_t s){ door_steps_open  = s; }
static inline void set_door_steps_close(uint32_t s){ door_steps_close = s; }
static inline void set_door_rate_hz(uint16_t hz){ if (hz) door_rate_hz = hz; }
static inline void set_door_hold_ms(uint16_t ms){ door_hold_ms_cfg = ms; }

/* ======================== USART2 TX circular ======================== */
#define TXSZ 256u
static volatile uint8_t  txbuf[TXSZ];
static volatile uint16_t tx_head=0, tx_tail=0;
static inline int tx_empty(void){ return tx_head==tx_tail; }
static inline void tx_kick(void){ USART2->CR1 |= USART_CR1_TXEIE; }
static void uart_putc(uint8_t c){ uint16_t n=(uint16_t)((tx_head+1u)&(TXSZ-1u)); if(n==tx_tail)return; txbuf[tx_head]=c; tx_head=n; tx_kick(); }
static void uart_puts(const char*s){ while(*s) uart_putc((uint8_t)*s++); uart_putc('\r'); uart_putc('\n'); }

/* ============================ Helpers GPIO ============================ */
static inline void seg_write(uint8_t pat){ GPIOB->BSRR=(0x3Fu<<16); GPIOB->BSRR=(uint32_t)(pat & 0x3F); }
static inline void digits_all_off(void){ GPIOC->BSRR=(1u<<(0+16))|(1u<<(1+16))|(1u<<(2+16))|(1u<<(3+16)); }
static inline void digit_on(uint8_t i){ static const uint8_t p[4]={0,1,2,3}; GPIOC->BSRR=(1u<<p[i]); }
static inline void LED_FloorOn (uint8_t f){ if(f==1)GPIOB->BSRR=(1u<<6); else if(f==2)GPIOB->BSRR=(1u<<7); else if(f==3)GPIOB->BSRR=(1u<<8); }
static inline void LED_FloorOff(uint8_t f){ if(f==1)GPIOB->BSRR=(1u<<(6+16)); else if(f==2)GPIOB->BSRR=(1u<<(7+16)); else if(f==3)GPIOB->BSRR=(1u<<(8+16)); }

/* ===== Helpers de cola (declarados ANTES de add_request) ===== */
static inline uint8_t is_pending(uint8_t f){
    if (f<1 || f>3) return 0;
    return (pending_mask & (1u<<(f-1))) ? 1u : 0u;
}
static inline uint8_t queue_contains(uint8_t f){
    for (uint8_t i=0;i<qn;i++) if (q[i]==f) return 1u;
    return 0u;
}

/* =============== Cola de pisos: encola robusto (máx 3, sin duplicados) =============== */
static void add_request(uint8_t f){
    if (f<1 || f>3) return;

    // No encolar si ya estoy en ese piso (en cualquier estado).
    if (f == currentFloor){
        uart_puts("IGNORED: already at this floor");
        return;
    }
    // No encolar si ya está pendiente (en cola o como destino activo).
    if (is_pending(f) || queue_contains(f)){
        uart_puts("IGNORED: floor already pending");
        return;
    }
    // Cola máxima = 3.
    if (qn >= 3){
        uart_puts("IGNORED: queue full (max=3)");
        return;
    }

    // Encolar y marcar pendiente (LED encendido hasta atender).
    q[qn++] = f;
    pending_mask |= (1u<<(f-1));
    LED_FloorOn(f);
    uart_puts("ENQUEUE");
    LCD_RequestRefresh();
}

/* ====================== Reloj: forzar HSI16 SYSCLK ====================== */
static void SystemClock_HSI16(void){
    RCC->CR |= RCC_CR_HSION; while(!(RCC->CR & RCC_CR_HSIRDY)){}
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while(((RCC->CFGR>>RCC_CFGR_SWS_Pos)&0x3u)!=0x01u){}
}

/* =================== USART2 @9600 8N1 (PCLK1=16MHz) =================== */
static void USART2_init_9600(void){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~((3u<<(2*2)) | (3u<<(3*2)));
    GPIOA->MODER |=  ((2u<<(2*2)) | (2u<<(3*2)));
    GPIOA->AFR[0]  &= ~((0xFu<<(2*4)) | (0xFu<<(3*4)));
    GPIOA->AFR[0]  |=  ((4u<<(2*4)) | (4u<<(3*4))); // AF4
    USART2->CR1 = 0; USART2->CR2 = 0; USART2->CR3 = 0;
    USART2->BRR = 0x0683;                               // 9600
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ========================= TIM21: 1 ms init ========================= */
static void TIM21_init_1ms(void){
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
    TIM21->CR1 = 0; TIM21->PSC = 15u; TIM21->ARR = 999u; TIM21->CNT = 0;
    TIM21->EGR = TIM_EGR_UG; TIM21->SR  = 0; TIM21->DIER = TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM21_IRQn); NVIC_EnableIRQ(TIM21_IRQn);
    TIM21->CR1 |= TIM_CR1_CEN;
}

/* ===================== TIM22: ~2 kHz (display) ===================== */
static void TIM22_init_2kHz(void){
    RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
    TIM22->CR1 = 0; TIM22->PSC = 159u; TIM22->ARR = 49u; TIM22->CNT = 0;
    TIM22->EGR = TIM_EGR_UG; TIM22->SR  = 0; TIM22->DIER = TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM22_IRQn); NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= TIM_CR1_CEN;
}

/* ===================== TIM2: 10 kHz (buzzer + stepper x2) ===================== */
static void TIM2_init_10kHz(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;
    TIM2->PSC = 159u;   // 16MHz/160 = 100 kHz
    TIM2->ARR = 9u;     // 100 kHz / (9+1) = 10 kHz tick
    TIM2->CNT = 0; TIM2->EGR = TIM_EGR_UG; TIM2->SR  = 0; TIM2->DIER = TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM2_IRQn); NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* =================== GPIO init (display, leds, keypad, buzzer, motores) =================== */
static void GPIO_init_all(void){
    RCC->IOPENR |= RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN;

    /* PA5 heartbeat */
    GPIOA->MODER &= ~(3u<<(5*2)); GPIOA->MODER |= (1u<<(5*2)); GPIOA->ODR &= ~(1u<<5);

    /* Botones de piso: PA6 (P1), PA9 (P2), PC12 (P3) -> entrada con pull-up */
    /* PA6 */
    GPIOA->MODER &= ~(3u<<(6*2));
    GPIOA->PUPDR &= ~(3u<<(6*2));
    GPIOA->PUPDR |=  (1u<<(6*2));   // pull-up

    /* PA9 */
    GPIOA->MODER &= ~(3u<<(9*2));
    GPIOA->PUPDR &= ~(3u<<(9*2));
    GPIOA->PUPDR |=  (1u<<(9*2));   // pull-up

    /* PC12 */
    GPIOC->MODER &= ~(3u<<(12*2));
    GPIOC->PUPDR &= ~(3u<<(12*2));
    GPIOC->PUPDR |=  (1u<<(12*2));  // pull-up

    /* 7-seg: PB0..PB5 (salida), dígitos: PC0..PC3 (salida) */
    GPIOB->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|(3u<<(4*2))|(3u<<(5*2)));
    GPIOB->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|(1u<<(4*2))|(1u<<(5*2)));
    GPIOC->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
    GPIOC->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
    seg_write(0x00); digits_all_off();

    /* LEDs de piso: PB6..PB8 */
    GPIOB->MODER &= ~((3u<<(6*2))|(3u<<(7*2))|(3u<<(8*2)));
    GPIOB->MODER |=  ((1u<<(6*2))|(1u<<(7*2))|(1u<<(8*2)));
    LED_FloorOff(1); LED_FloorOff(2); LED_FloorOff(3);

    /* Keypad: PA0/PA1/PA4 entradas pull-up; PA7 salida LOW (fila) */
    GPIOA->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(4*2)));
    GPIOA->PUPDR &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(4*2)));
    GPIOA->PUPDR |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(4*2)));
    GPIOA->MODER &= ~(3u<<(7*2)); GPIOA->MODER |= (1u<<(7*2)); GPIOA->ODR &= ~(1u<<7);

    /* Buzzer: PA8 salida push-pull */
    GPIOA->MODER &= ~(3u<<(8*2)); GPIOA->MODER |= (1u<<(8*2));
    GPIOA->OTYPER &= ~(1u<<8); GPIOA->OSPEEDR |= (3u<<(8*2)); GPIOA->PUPDR &= ~(3u<<(8*2));
    GPIOA->BSRR = (1u<<(8+16));

    /* Stepper ASCENSOR: PB12..PB15 (IN1..IN4) */
    GPIOB->MODER &= ~( (3u<<(12*2)) | (3u<<(13*2)) | (3u<<(14*2)) | (3u<<(15*2)) );
    GPIOB->MODER |=  ( (1u<<(12*2)) | (1u<<(13*2)) | (1u<<(14*2)) | (1u<<(15*2)) );
    GPIOB->BSRR = ( (1u<<(12+16)) | (1u<<(13+16)) | (1u<<(14+16)) | (1u<<(15+16)) );

    /* Stepper PUERTA: PC8..PC11 (IN1..IN4) */
    GPIOC->MODER &= ~( (3u<<(8*2)) | (3u<<(9*2)) | (3u<<(10*2)) | (3u<<(11*2)) );
    GPIOC->MODER |=  ( (1u<<(8*2)) | (1u<<(9*2)) | (1u<<(10*2)) | (1u<<(11*2)) );
    GPIOC->BSRR = ( (1u<<(8+16)) | (1u<<(9+16)) | (1u<<(10+16)) | (1u<<(11+16)) );

    /* LCD 16x2 4-bit: D4..D7 = PC4..PC7, RS=PB11, E=PB10 */
    GPIOC->MODER &= ~((3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
    GPIOC->MODER |=  ((1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));
    GPIOB->MODER &= ~((3u<<(10*2))|(3u<<(11*2)));
    GPIOB->MODER |=  ((1u<<(10*2))|(1u<<(11*2)));
    /* líneas en bajo por defecto */
    GPIOC->BSRR = (1u<<(4+16))|(1u<<(5+16))|(1u<<(6+16))|(1u<<(7+16));
    GPIOB->BSRR = (1u<<(10+16))|(1u<<(11+16));

}

static void EXTI_buttons_init(void){
    /* SYSCFG para mapear puertos en EXTI */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* EXTI6 -> PA6 (por defecto ya es A, pero limpiamos por claridad) */
    SYSCFG->EXTICR[1] &= ~(0xFu << 8);   // line 6 -> port A (0000)

    /* EXTI9 -> PA9 */
    SYSCFG->EXTICR[2] &= ~(0xFu << 4);   // line 9 -> port A (0000)

    /* EXTI12 -> PC12 */
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (0x2u << 0);   // 0x2 = port C

    /* Unmask + flanco de bajada */
    EXTI->IMR  |= (1u<<6) | (1u<<9) | (1u<<12);
    EXTI->FTSR |= (1u<<6) | (1u<<9) | (1u<<12);
    EXTI->RTSR &= ~((1u<<6) | (1u<<9) | (1u<<12));  // solo bajada

    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(void){
    uint32_t pending = EXTI->PR & ( (1u<<6) | (1u<<9) | (1u<<12) );

    if (pending & (1u<<6)){                    // PA6 -> Piso 1
        EXTI->PR = (1u<<6);                    // limpiar pending
        if ( (GPIOA->IDR & (1u<<6)) == 0 ){    // aún en 0 (presionado)
            add_request(1);
            uart_puts("EXTI BTN: Piso 1");
        }
    }

    if (pending & (1u<<9)){                    // PA9 -> Piso 2
        EXTI->PR = (1u<<9);
        if ( (GPIOA->IDR & (1u<<9)) == 0 ){
            add_request(2);
            uart_puts("EXTI BTN: Piso 2");
        }
    }

    if (pending & (1u<<12)){                   // PC12 -> Piso 3
        EXTI->PR = (1u<<12);
        if ( (GPIOC->IDR & (1u<<12)) == 0 ){
            add_request(3);
            uart_puts("EXTI BTN: Piso 3");
        }
    }
}


/* ===================== STEPPER ASCENSOR low-level ===================== */
static inline void stepper_coils_off(void){
    GPIOB->BSRR = ( (1u<<(12+16)) | (1u<<(13+16)) | (1u<<(14+16)) | (1u<<(15+16)) );
}
static inline void stepper_apply_phase(uint8_t ph){
    stepper_coils_off();
    switch(ph & 0x03){
        case 0: GPIOB->BSRR = (1u<<12) | (1u<<13); break; // IN1+IN2
        case 1: GPIOB->BSRR = (1u<<13) | (1u<<14); break; // IN2+IN3
        case 2: GPIOB->BSRR = (1u<<14) | (1u<<15); break; // IN3+IN4
        case 3: GPIOB->BSRR = (1u<<15) | (1u<<12); break; // IN4+IN1
    }
}
static inline void stepper_enable(int8_t dir){
    moving_dir = dir; stepper_enabled = 1; stepper_phase = 0; stepper_apply_phase(stepper_phase);
}
static inline void stepper_disable(void){
    stepper_enabled = 0; stepper_coils_off();
}
static inline void stepper_step_once(void){
    if (!stepper_enabled) return;
    if (moving_dir>0) stepper_phase = (uint8_t)((stepper_phase + 1u) & 0x03u);
    else              stepper_phase = (uint8_t)((stepper_phase + 3u) & 0x03u);
    stepper_apply_phase(stepper_phase);
}

/* ===================== STEPPER PUERTA low-level ===================== */
static inline void door_coils_off(void){
    GPIOC->BSRR = ( (1u<<(8+16)) | (1u<<(9+16)) | (1u<<(10+16)) | (1u<<(11+16)) );
}
static inline void door_apply_phase(uint8_t ph){
    door_coils_off();
    switch(ph & 0x03){
        case 0: GPIOC->BSRR = (1u<<8)  | (1u<<9);  break; // IN1+IN2
        case 1: GPIOC->BSRR = (1u<<9)  | (1u<<10); break; // IN2+IN3
        case 2: GPIOC->BSRR = (1u<<10) | (1u<<11); break; // IN3+IN4
        case 3: GPIOC->BSRR = (1u<<11) | (1u<<8);  break; // IN4+IN1
    }
}
static inline void door_stepper_enable(int8_t dir, uint32_t steps){
    door_moving_dir = dir;               // +1 abrir (horario), -1 cerrar (antihorario)
    door_steps_remaining = steps;
    door_stepper_enabled = 1;
    door_phase = 0;
    door_apply_phase(door_phase);
}
static inline void door_stepper_disable(void){
    door_stepper_enabled = 0;
    door_coils_off();
}
static inline void door_step_once(void){
    if (!door_stepper_enabled) return;
    if (door_moving_dir>0) door_phase = (uint8_t)((door_phase + 1u) & 0x03u);
    else                   door_phase = (uint8_t)((door_phase + 3u) & 0x03u);
    door_apply_phase(door_phase);
}

/* -------- Helpers FSM para hops por PASOS (ascensor) -------- */
static inline uint32_t hop_steps(uint8_t from, uint8_t to){
    if (from<1||from>3||to<1||to>3||from==to) return 0;
    return steps_between[from][to];
}

/* ============ Lógica de ASCENSOR (sin paradas intermedias) ============ */
static void start_next_destination(void){
    if (qn==0){ state=IDLE; moving_dir=0; hop_steps_remaining=0; stepper_disable(); return; }

    destFloor = q[0];
    for(uint8_t i=0;i<qn-1;i++) q[i]=q[i+1];
    qn--;

    if (destFloor == currentFloor){
        pending_mask &= ~(1u<<(destFloor-1));
        LED_FloorOff(destFloor);
        buzzer_ms=300; buzzer_active=1;
        uart_puts("ARRIVED (same floor)");
        state = IDLE; stepper_disable();
        LCD_RequestRefresh();
        return;
    }

    moving_dir = (destFloor > currentFloor) ? +1 : -1;
    hop_target = (uint8_t)(currentFloor + moving_dir);
    hop_steps_remaining = hop_steps(currentFloor, hop_target);

    if (hop_steps_remaining == 0u){
        uart_puts("WARN: first hop steps=0; skipping");
        currentFloor = hop_target;
        if (currentFloor == destFloor){
            pending_mask &= ~(1u<<(currentFloor-1));
            LED_FloorOff(currentFloor);
            state = HOLD_AT_FLOOR; /* aquí entra a puerta */
            door_state = DOOR_OPENING;
            door_stepper_enable(+1, door_steps_open);
            buzzer_ms=350; buzzer_active=1;
            stepper_disable();
            uart_puts("ARRIVED (destination, zero-step)");
            LCD_RequestRefresh();
            return;
        } else {
            hop_target = (uint8_t)(currentFloor + moving_dir);
            hop_steps_remaining = hop_steps(currentFloor, hop_target);
            if (hop_steps_remaining == 0u){
                stepper_disable();
                state = IDLE; moving_dir=0;
                uart_puts("ERROR: consecutive zero-step hops; IDLE");
                return;
            }
        }
    }

    state = (moving_dir>0) ? MOVING_UP : MOVING_DOWN;
    anim_phase = 0; anim_tick_ms = 0;

    LCD_RequestRefresh();

    stepper_enable(moving_dir);
    uart_puts((moving_dir>0) ? "STATE:UP (hop start)" : "STATE:DOWN (hop start)");
}

static void continue_or_finish_after_floor_reached(void){
    hop_done_flag = 0;
    currentFloor = hop_target;

    /* ¿Llegamos al DESTINO? -> parar, iniciar puerta */
    if (currentFloor == destFloor){
        /* Limpiar pendiente y LED del piso si estaba solicitado */
        if (pending_mask & (1u<<(currentFloor-1))){
            pending_mask &= ~(1u<<(currentFloor-1));
            LED_FloorOff(currentFloor);
        }
        stepper_disable();

        /* Puerta: abrir -> hold -> cerrar (no bloqueante) */
        state = HOLD_AT_FLOOR;
        door_state = DOOR_OPENING;
        door_stepper_enable(+1, door_steps_open);   // +1 horario = abrir
        buzzer_ms = 350; buzzer_active = 1;
        uart_puts("ARRIVED (destination) -> DOOR_OPENING");

        LCD_RequestRefresh();
        return;
    }

    /* Piso intermedio: NO parar aunque esté solicitado. */
    hop_target = (uint8_t)(currentFloor + moving_dir);
    hop_steps_remaining = hop_steps(currentFloor, hop_target);

    if (hop_steps_remaining == 0u){
        uart_puts("WARN: hop steps = 0; skipping");
        currentFloor = hop_target;
        if (currentFloor == destFloor){
            if (pending_mask & (1u<<(currentFloor-1))){
                pending_mask &= ~(1u<<(currentFloor-1));
                LED_FloorOff(currentFloor);
            }
            stepper_disable();
            state = HOLD_AT_FLOOR;
            door_state = DOOR_OPENING;
            door_stepper_enable(+1, door_steps_open);
            buzzer_ms = 350; buzzer_active=1;
            uart_puts("ARRIVED (destination, zero-step) -> DOOR_OPENING");
            LCD_RequestRefresh();
            return;
        } else {
            hop_target = (uint8_t)(currentFloor + moving_dir);
            hop_steps_remaining = hop_steps(currentFloor, hop_target);
            if (hop_steps_remaining == 0u){
                stepper_disable();
                state = IDLE; moving_dir = 0;
                uart_puts("ERROR: consecutive zero-step hops; IDLE");
                LCD_RequestRefresh();
                return;
            }
        }
    }

    /* Reanudar motor hacia el siguiente hop */
    stepper_enable(moving_dir);
    uart_puts("Passing floor (no stop); next hop");
}

/* =================== ISRs bodies =================== */
static volatile uint16_t hb250=0;

static void tim21_isr_body(void){
    /* Heartbeat PA5 */
    if (++hb250 >= 250u){ hb250=0; GPIOA->ODR ^= (1u<<5); }

    /* Keypad (PA7 fila=0; PA0/1/4 pull-up) */
    uint8_t A = ( (GPIOA->IDR & (1u<<0)) ? 1:0 );
    uint8_t B = ( (GPIOA->IDR & (1u<<1)) ? 1:0 );
    uint8_t C = ( (GPIOA->IDR & (1u<<4)) ? 1:0 );
    static uint8_t prevA=1, prevB=1, prevC=1;
    if (A==0 && prevA==1){ add_request(1); uart_puts("KEYPAD:1"); }
    if (B==0 && prevB==1){ add_request(2); uart_puts("KEYPAD:2"); }
    if (C==0 && prevC==1){ add_request(3); uart_puts("KEYPAD:3"); }
    prevA=A; prevB=B; prevC=C;

    /* Animación cada 100 ms */
    if (++anim_tick_ms >= 100u){
        anim_tick_ms = 0;
        if (state==MOVING_UP || state==MOVING_DOWN) anim_phase = (uint8_t)((anim_phase + 1u) % 3u);
        else anim_phase = 0;
    }

    /* FSM principal del ascensor + puerta */
    switch(state){
    case IDLE:
        if (qn>0) start_next_destination();
        break;

    case MOVING_UP:
    case MOVING_DOWN:
        if (hop_done_flag){ continue_or_finish_after_floor_reached(); }
        break;

    case HOLD_AT_FLOOR:
        /* Avance de FSM de puerta (no bloqueante) */

        /* 1) Fin de tramo OPENING/CLOSING notificado por TIM2 */
        if (door_step_done_flag){
            door_step_done_flag = 0;
            if (door_state == DOOR_OPENING){
                door_state = DOOR_OPEN_HOLD;
                door_hold_ms = door_hold_ms_cfg;   // tiempo abierta
                uart_puts("DOOR_OPENED -> HOLD");
                LCD_RequestRefresh();
            } else if (door_state == DOOR_CLOSING){
                door_state = DOOR_DONE;
                uart_puts("DOOR_CLOSED -> DONE");
            }
        }

        /* 2) HOLD (cuenta regresiva 1 ms) */
        if (door_state == DOOR_OPEN_HOLD){
            if (door_hold_ms>0) door_hold_ms--;
            else {
                door_state = DOOR_CLOSING;
                door_stepper_enable(-1, door_steps_close);  // -1 antihorario = cerrar
                uart_puts("HOLD done -> DOOR_CLOSING");
                LCD_RequestRefresh();
            }
        }

        /* 3) Si puerta terminó (DONE), salir de HOLD_AT_FLOOR */
        if (door_state == DOOR_DONE){
            /* Reset puerta a IDLE para próxima vez */
            door_state = DOOR_IDLE;

            if (currentFloor == destFloor){
                if (qn>0) start_next_destination();
                else { state=IDLE; moving_dir=0; stepper_disable(); uart_puts("IDLE"); }
            } else {
                /* Teóricamente no debería ocurrir porque solo abrimos en destino */
                hop_target = (uint8_t)(currentFloor + moving_dir);
                hop_steps_remaining = hop_steps(currentFloor, hop_target);
                state = (moving_dir>0)?MOVING_UP:MOVING_DOWN;
                stepper_enable(moving_dir);
                uart_puts("RESUME after door (unexpected path)");
            }
            LCD_RequestRefresh();
        }
        break;

    default: break;
    }

    /* Tiempo del buzzer */
    if (buzzer_ms){
        if (--buzzer_ms == 0u){
            buzzer_active = 0u;
            GPIOA->BSRR = (1u<<(8+16));
        }
    }
    /* Servicio LCD 1 ms (init + cola + timings) */
    LCD_Service_1ms();

    /* Si hay que redibujar y la cola está libre, manda las dos líneas */
    if (lcd_inited && lcd_dirty && (lcd_busy_ms==0)){
        LCD_RefreshNow();
    }
}

static void tim22_isr_body(void){
    static uint8_t idx=0;
    digits_all_off();
    if (idx==0){
        seg_write(seg_digit_lut[currentFloor]);
        digit_on(0);
    } else {
        if (state==MOVING_UP || state==MOVING_DOWN){
            uint8_t seg_pat=0;
            if (state==MOVING_UP){
                if (anim_phase==0) seg_pat=(1u<<4); else if (anim_phase==1) seg_pat=(1u<<5); else seg_pat=(1u<<1);
            } else {
                if (anim_phase==0) seg_pat=(1u<<1); else if (anim_phase==1) seg_pat=(1u<<5); else seg_pat=(1u<<4);
            }
            seg_write(seg_pat); digit_on(idx);
        } else {
            seg_write(0x00); digit_on(idx);
        }
    }
    idx = (uint8_t)((idx+1u)&0x03u);
}

/* ===== TIM2: 10 kHz tick ⇒ buzzer + stepper (cabina) + stepper (puerta) ===== */
void TIM2_IRQHandler(void){
    if (TIM2->SR & TIM_SR_UIF){
        TIM2->SR &= ~TIM_SR_UIF;

        /* ---- BUZZER (PA8) ---- */
        static uint16_t buzz_cnt=0;
        const uint32_t tick_hz = 10000u;  // 10 kHz
        uint32_t desired = buzzer_hz ? buzzer_hz : 1u;
        uint32_t buzz_div = tick_hz / (2u*desired); if (buzz_div==0) buzz_div=1;

        if (buzzer_active){
            if (++buzz_cnt >= buzz_div){ buzz_cnt=0; GPIOA->ODR ^= (1u<<8); }
        } else {
            GPIOA->BSRR = (1u<<(8+16));
            buzz_cnt = 0;
        }

        /* ---- STEPPER ASCENSOR (PB12..PB15) ---- */
        static uint16_t step_cnt=0;
        uint32_t srate = stepper_rate_hz ? stepper_rate_hz : 1u;
        uint32_t step_div  = (tick_hz / srate); if (step_div==0) step_div=1;

        if (stepper_enabled && hop_steps_remaining>0){
            if (++step_cnt >= step_div){
                step_cnt=0;
                stepper_step_once();
                hop_steps_remaining--;
                if (hop_steps_remaining==0){
                    hop_done_flag = 1;
                    stepper_disable();
                }
            }
        } else {
            step_cnt=0;
        }

        /* ---- STEPPER PUERTA (PC8..PC11) ---- */
        static uint16_t door_cnt=0;
        uint32_t drate = door_rate_hz ? door_rate_hz : 1u;
        uint32_t door_div = (tick_hz / drate); if (door_div==0) door_div=1;

        if (door_stepper_enabled && door_steps_remaining>0){
            if (++door_cnt >= door_div){
                door_cnt=0;
                door_step_once();
                door_steps_remaining--;
                if (door_steps_remaining==0){
                    door_step_done_flag = 1;
                    door_stepper_disable();
                }
            }
        } else {
            door_cnt=0;
        }
    }
}

/* ========================= Handlers de interrupción ========================= */
void TIM21_IRQHandler(void){ if (TIM21->SR & TIM_SR_UIF){ TIM21->SR &= ~TIM_SR_UIF; tim21_isr_body(); } }
void TIM22_IRQHandler(void){ if (TIM22->SR & TIM_SR_UIF){ TIM22->SR &= ~TIM_SR_UIF; tim22_isr_body(); } }
void TIM21_TIM22_IRQHandler(void){
    if (TIM21->SR & TIM_SR_UIF){ TIM21->SR &= ~TIM_SR_UIF; tim21_isr_body(); }
    if (TIM22->SR & TIM_SR_UIF){ TIM22->SR &= ~TIM_SR_UIF; tim22_isr_body(); }
}

/* ================================ USART2 IRQ ================================ */
void USART2_IRQHandler(void){
    uint32_t isr = USART2->ISR;
    if (isr & USART_ISR_RXNE){
        uint8_t ch = (uint8_t)USART2->RDR;
        if      (ch=='1'){ add_request(1); uart_puts("UART:1"); }
        else if (ch=='2'){ add_request(2); uart_puts("UART:2"); }
        else if (ch=='3'){ add_request(3); uart_puts("UART:3"); }
        else if (ch=='r'){ set_stepper_rate(600); uart_puts("STEP RATE=600Hz"); }
        else if (ch=='t'){ set_buzzer_tone(3000); uart_puts("BUZZ 3kHz"); }
        else if (ch=='o'){ set_door_steps_open(800); uart_puts("DOOR OPEN=800"); }
        else if (ch=='c'){ set_door_steps_close(800); uart_puts("DOOR CLOSE=800"); }
        else if (ch=='h'){ set_door_hold_ms(1500); uart_puts("DOOR HOLD=1500ms"); }
        else if (ch=='d'){ set_door_rate_hz(400); uart_puts("DOOR RATE=400Hz"); }
        else              { uart_puts("UART:?"); }
    }
    if ((isr & USART_ISR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)){
        if (!tx_empty()){ USART2->TDR = txbuf[tx_tail]; tx_tail=(uint16_t)((tx_tail+1u)&(TXSZ-1u)); }
        else USART2->CR1 &= ~USART_CR1_TXEIE;
    }
}

/* ================================== MAIN ================================== */
int main(void){
    SystemClock_HSI16();
    GPIO_init_all();
    EXTI_buttons_init();
    LCD_InitStart();


    USART2_init_9600();
    uart_puts("BOOT: HSI16 SYSCLK, UART2 9600 OK");

    TIM21_init_1ms();      // FSM, keypad, animación, buzzer timeout, door hold
    TIM22_init_2kHz();     // multiplex display
    TIM2_init_10kHz();     // buzzer + stepper ascensor + stepper puerta

    /* ======= calibración rápida (opcional) =======
       set_steps_between(1,2,1300);
       set_steps_between(2,3,1250);
       set_stepper_rate(500);          // cabina (pasos/seg)
       set_buzzer_tone(2500);          // Hz
       set_door_steps_open(700);       // pasos abrir
       set_door_steps_close(700);      // pasos cerrar
       set_door_rate_hz(450);          // pasos/seg puerta
       set_door_hold_ms(1200);         // ms puerta abierta
    */

    __enable_irq();

    while(1){ __NOP(); }
}
