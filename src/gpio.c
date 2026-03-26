#include "gpio.h"
#include "TM4C123.h"

/* ===== PORT A: Red LED (PA5), Green LED (PA6), Buzzer (PA7) ===== */
void GPIOA_LED_Buzzer_Init(void)
{
    SYSCTL->RCGCGPIO |= GPIO_PORTA_CLOCK_EN;
    for (volatile int i = 0; i < 3; i++);

    GPIOA->DIR |= (1U << 5) | (1U << 6) | (1U << 7);
    GPIOA->DEN |= (1U << 5) | (1U << 6) | (1U << 7);

    GPIOA->DATA |= (1U << 5);                    /* Red LED ON  - power indicator */
    GPIOA->DATA &= ~((1U << 6) | (1U << 7));     /* Green LED and Buzzer OFF      */
}

/* ===== PORT B: I2C0 (PB2=SCL, PB3=SDA) ===== */
void GPIOB_Init(void)
{
    SYSCTL->RCGCGPIO |= GPIO_PORTB_CLOCK_EN;
    for (volatile int i = 0; i < 3; i++);

    GPIOB->DIR   &= ~0xFF;
    GPIOB->DEN   |=  0xFF;

    /* I2C alternate function on PB2/PB3 */
    GPIOB->AFSEL |=  (1U << 2) | (1U << 3);
    GPIOB->ODR   |=  (1U << 3);               /* SDA open-drain */
    GPIOB->PCTL  &= ~0x0000FF00U;
    GPIOB->PCTL  |=  0x00003300U;
}

/* ===== PORT C: Timer CCP inputs (PC4, PC5) ===== */
void GPIOC_Init(void)
{
    SYSCTL->RCGCGPIO |= GPIO_PORTC_CLOCK_EN;
    for (volatile int i = 0; i < 3; i++);

    GPIOC->DIR   &= ~((1U << 4) | (1U << 5));
    GPIOC->DEN   |=   (1U << 4) | (1U << 5);
    GPIOC->AFSEL |=   (1U << 4) | (1U << 5);
    GPIOC->PCTL  &=   0xFF00FFFF;
    GPIOC->PCTL  |=   0x00770000;
}

/* ===== PORT D: Motor PWM outputs (PD0 = Motor1, PD1 = Motor2) ===== */
void GPIOD_Init(void)
{
    SYSCTL->RCGCGPIO |= GPIO_PORTD_CLOCK_EN;
    for (volatile int i = 0; i < 3; i++);

    GPIOD->DIR  |= (1U << 0) | (1U << 1);
    GPIOD->DEN  |= (1U << 0) | (1U << 1);
    GPIOD->DATA &= ~((1U << 0) | (1U << 1));
}
