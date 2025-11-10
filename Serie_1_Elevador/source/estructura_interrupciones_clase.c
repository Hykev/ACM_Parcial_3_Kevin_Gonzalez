#include <stdint.h>
#include "stm32l053xx.h"

// ===================== PROTOTYPES =====================
void delay_ms(uint16_t n);
void USART2_write(uint8_t ch);
uint8_t USART2_read(void);
void USART2_Putstring(uint8_t *stringptr);
void USART2_Putstring8(uint8_t *stringptr);
void USART2_init(void);

// ===================== GLOBAL VARIABLES =====================
uint8_t atm_fsm       = 0x00;
uint8_t recieved_Vic  = 0x00;
uint8_t recieved_var  = 0x00;

// ===================== MAIN =====================
int main(void)
{
    // ---- Enable HSI 16MHz ----
    RCC->CR |= (1 << 0);          // HSI ON
    RCC->CFGR |= (1 << 0);        // HSI16 as SYSCLK

    // ---- GPIO CLOCK ENABLE ----
    RCC->IOPENR |= (1 << 0);      // GPIOA
    RCC->IOPENR |= (1 << 2);      // GPIOC

    // ---- GPIO CONFIG ----
    // PA5 -> Output (LED)
    GPIOA->MODER &= ~(1 << 11);
    GPIOA->MODER |=  (1 << 10);

    // PC4 -> Output
    GPIOC->MODER &= ~(1 << 9);
    GPIOC->MODER |=  (1 << 8);

    // PC13 -> Input (User Button)
    GPIOC->MODER &= ~(1 << 26);
    GPIOC->MODER &= ~(1 << 27);

    // ---- ENABLE EXTERNAL INTERRUPTS ----
    RCC->APB2ENR |= (1 << 0);     // Enable SYSCFG clock
    SYSCFG->EXTICR[3] |= (1 << 5);// Select PORTC for EXTI13
    EXTI->IMR  |= (1 << 13);      // Unmask line 13
    EXTI->FTSR |= (1 << 13);      // Falling-edge trigger

    // ---- USART2 INIT ----
    USART2_init();
    USART2->CR1 |= (1 << 5);      // RX interrupt enable

    // ---- TIM21 CONFIG ----
    RCC->APB2ENR |= (1 << 2);     // TIM21 clock enable
    TIM21->PSC = 1600 - 1;
    TIM21->ARR = 10000 - 1;
    TIM21->CR1 |= (1 << 0);
    TIM21->DIER |= (1 << 0);      // UIE interrupt enable

    // ---- TIM22 CONFIG ----
    RCC->APB2ENR |= (1 << 5);     // TIM22 clock enable
    TIM22->PSC = 16000 - 1;
    TIM22->ARR = 10 - 1;
    TIM22->CNT = 0;
    TIM22->CR1 |= (1 << 0);
    TIM22->DIER |= (1 << 0);

    // ---- SYSTICK CONFIG ----
    SysTick->LOAD = 160 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL |= (1 << 2) | (1 << 1) | (1 << 0);

    // ---- NVIC ENABLE INTERRUPTS ----
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(TIM21_IRQn);
    NVIC_EnableIRQ(TIM22_IRQn);

    // ---- Enable Global Interrupts ----
    enable_irq();

    // ---- Start Message ----
    USART2_Putstring((uint8_t*)"Vic & var detector");

    // ---- MAIN LOOP ----
    while(1)
    {
        if(recieved_Vic == 0x01)
        {
            GPIOA->ODR ^= (1 << 5);
            recieved_Vic = 0;
        }
        else if(recieved_var == 0x01)
        {
            GPIOA->ODR ^= (1 << 5);
            recieved_var = 0;
        }
    }
}

// ===================== INTERRUPT HANDLERS =====================

void USART2_init(void)
{
    // (Configuración UART2 no visible en las imágenes)
    // Aquí normalmente se activan los pines TX/RX, baud rate, modo 8N1, etc.
}

void TIM21_IRQHandler(void)
{
    GPIOA->ODR ^= (1 << 5);        // Toggle LED (PA5)
    TIM21->SR &= ~(1 << 0);        // Clear UIF flag
}

void TIM22_IRQHandler(void)
{
    GPIOC->ODR ^= (1 << 4);        // Toggle LED (PC4)
    TIM22->SR &= ~(1 << 0);        // Clear UIF flag
}

void SysTick_Handler(void)
{
    // Something
}

void EXTI4_15_IRQHandler(void)
{
    GPIOA->ODR ^= (1 << 5);        // Toggle LED
    EXTI->PR |= (1 << 13);         // Clear pending flag
}

void USART2_IRQHandler(void)
{
    // (Rutina de recepción USART, no visible en las imágenes)
}
