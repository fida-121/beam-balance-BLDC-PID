#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>

/* ===== NVIC BIT POSITIONS ===== */
#define NVIC_TIMER0A_EN   (1U << 19)
#define NVIC_TIMER1A_EN   (1U << 21)
#define NVIC_TIMER2A_EN   (1U << 23)
#define NVIC_TIMER3A_EN   (1U << 3)

/* ===== MOTOR PULSE LIMITS ===== */
#define MOTOR_PULSE_MIN_US   1000UL
#define MOTOR_PULSE_MAX_US   2000UL
#define MOTOR_PULSE_BASE_US  1125UL   /* Idle / arming speed */

/* ===== EXPORTED MOTOR STATE ===== */
extern volatile unsigned long pulse_us_1;    /* Motor 1 current pulse width */
extern volatile unsigned long pulse_us_2;    /* Motor 2 current pulse width */
extern volatile uint32_t      rpm_1;
extern volatile uint32_t      rpm_2;

/* ===== TIMER INITIALIZATIONS ===== */
void Timer0A_50Hz_Init(void);       /* 50 Hz PWM frame timer               */
void Timer1A_OneShot_Init(void);    /* One-shot for Motor 1 pulse width     */
void Timer3A_OneShot_Init(void);    /* One-shot for Motor 2 pulse width     */
void Timer2A_RPM_Measure_Init(void);/* 1 Hz gate for RPM counting           */
void WTIMER0A_CCP0_init(void);      /* Wide timer edge counter (RPM input)  */

/* ===== MOTOR CONTROL ===== */
void set_pulse_width_us(unsigned long us, int motor);

#endif /* TIMERS_H */
