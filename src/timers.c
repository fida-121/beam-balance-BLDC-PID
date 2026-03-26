#include "timers.h"
#include "system.h"
#include "TM4C123.h"

/* ===== EXPORTED MOTOR STATE ===== */
volatile unsigned long pulse_us_1 = MOTOR_PULSE_MIN_US;
volatile unsigned long pulse_us_2 = MOTOR_PULSE_MIN_US;
volatile uint32_t      rpm_1      = 0;
volatile uint32_t      rpm_2      = 0;


/* ===== TIMER 0A: 50 Hz PERIODIC (PWM FRAME) =====
   Fires every 20 ms. ISR starts both motor one-shot timers simultaneously
   so pulses on PD0 and PD1 begin at the same time each frame.
*/
void Timer0A_50Hz_Init(void)
{
    SYSCTL->RCGCTIMER |= (1U << 0);
    for (volatile int i = 0; i < 3; i++);

    TIMER0->CTL   = 0;
    TIMER0->CFG   = 0;            /* 32-bit mode */
    TIMER0->TAMR  = 0x02;         /* Periodic */
    TIMER0->TAILR = SYSCLK / 50 - 1;
    TIMER0->ICR   = 0x1;
    TIMER0->IMR   = 0x1;
    NVIC->ISER[0] |= NVIC_TIMER0A_EN;
    TIMER0->CTL  |= 0x1;
}

/* ===== TIMER 1A: ONE-SHOT for MOTOR 1 PULSE ===== */
void Timer1A_OneShot_Init(void)
{
    SYSCTL->RCGCTIMER |= (1U << 1);
    for (volatile int i = 0; i < 3; i++);

    TIMER1->CTL  &= ~0x1;
    TIMER1->CFG   = 0x04;         /* 16-bit mode */
    TIMER1->TAMR  = 0x01;         /* One-shot */
    TIMER1->ICR   = 0x1;
    TIMER1->IMR   = 0x1;
    NVIC->ISER[0] |= NVIC_TIMER1A_EN;
}

/* ===== TIMER 3A: ONE-SHOT for MOTOR 2 PULSE ===== */
void Timer3A_OneShot_Init(void)
{
    SYSCTL->RCGCTIMER |= (1U << 3);
    for (volatile int i = 0; i < 3; i++);

    TIMER3->CTL  &= ~0x1;
    TIMER3->CFG   = 0x04;         /* 16-bit mode */
    TIMER3->TAMR  = 0x01;         /* One-shot */
    TIMER3->ICR   = 0x1;
    TIMER3->IMR   = 0x1;
    NVIC->ISER[1] |= NVIC_TIMER3A_EN;
}

/* ===== TIMER 2A: 1 Hz GATE for RPM MEASUREMENT ===== */
void Timer2A_RPM_Measure_Init(void)
{
    SYSCTL->RCGCTIMER |= (1U << 2);
    for (volatile int i = 0; i < 3; i++);

    TIMER2->CTL   = 0;
    TIMER2->CFG   = 0;
    TIMER2->TAMR  = 0x02;         /* Periodic */
    TIMER2->TAILR = SYSCLK - 1;   /* 1 second period */
    TIMER2->ICR   = 0x1;
    TIMER2->IMR   = 0x1;
    NVIC->ISER[0] |= NVIC_TIMER2A_EN;
    TIMER2->CTL  |= 0x1;
}

/* ===== WIDE TIMER 0: EDGE-COUNT INPUT CAPTURE (RPM) =====
   TA counts rising edges on WT0CCP0, TB on WT0CCP1.
   Timer 2A ISR reads counts every second -> RPM.
*/
void WTIMER0A_CCP0_init(void)
{
    SYSCTL->RCGCWTIMER |= (1U << 0);
    for (int j = 0; j < 3; j++);

    WTIMER0->CTL = 0;
    WTIMER0->CFG = 0x04;            /* Split 32-bit timers */

    WTIMER0->TAMR = 0x13;           /* Edge count, capture mode */
    WTIMER0->TAMATCHR = 0xFFFF;
    WTIMER0->TAPMR   = 0xFF;

    WTIMER0->TBMR = 0x13;
    WTIMER0->TBMATCHR = 0xFFFF;
    WTIMER0->TBPMR   = 0xFF;

    WTIMER0->CTL |= 0x0101;         /* Enable both A and B */
}


/* ===== MOTOR PULSE WIDTH SETTER =====
   motor 0 = Motor 1 (PD0), motor 1 = Motor 2 (PD1).
   Clamped to [1000, 2000] µs (standard ESC range).
*/
void set_pulse_width_us(unsigned long us, int motor)
{
    if (us < MOTOR_PULSE_MIN_US) us = MOTOR_PULSE_MIN_US;
    if (us > MOTOR_PULSE_MAX_US) us = MOTOR_PULSE_MAX_US;

    if (motor == 0) pulse_us_1 = us;
    if (motor == 1) pulse_us_2 = us;
}


/* ===== ISRs ===== */

/* Timer 0A: Start PWM frame - raise both signal lines, arm one-shots */
void TIMER0A_Handler(void)
{
    TIMER0->ICR = 0x1;

    GPIOD->DATA |= (1U << 0) | (1U << 1);         /* Both lines HIGH */

    TIMER1->TAILR = (SYSCLK / 1000000UL) * pulse_us_1 - 1;
    TIMER3->TAILR = (SYSCLK / 1000000UL) * pulse_us_2 - 1;

    TIMER1->CTL |= 0x1;    /* Start Motor 1 one-shot */
    TIMER3->CTL |= 0x1;    /* Start Motor 2 one-shot */
}

/* Timer 1A: Motor 1 pulse end - lower signal line */
void TIMER1A_Handler(void)
{
    TIMER1->ICR = 0x1;
    GPIOD->DATA &= ~(1U << 0);
}

/* Timer 3A: Motor 2 pulse end - lower signal line */
void TIMER3A_Handler(void)
{
    TIMER3->ICR = 0x1;
    GPIOD->DATA &= ~(1U << 1);
}

/* Timer 2A: 1 Hz gate - snapshot edge counts, compute RPM, reset counters */
void TIMER2A_Handler(void)
{
    TIMER2->ICR = 0x1;

    uint32_t pulses_1 = WTIMER0->TAR;
    uint32_t pulses_2 = WTIMER0->TBR;

    /* Reset wide timer counters */
    WTIMER0->CTL &= ~0x0101;
    WTIMER0->TAV  = 0;
    WTIMER0->TBV  = 0;
    WTIMER0->CTL |=  0x0101;

    /* 2 pulses per revolution (2-pole motor) */
    rpm_1 = (pulses_1 * 60) / 2;
    rpm_2 = (pulses_2 * 60) / 2;
}
