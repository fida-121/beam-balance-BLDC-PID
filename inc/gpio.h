#ifndef GPIO_H
#define GPIO_H

/* ===== CLOCK ENABLE BITS ===== */
#define GPIO_PORTA_CLOCK_EN  (1U << 0)
#define GPIO_PORTB_CLOCK_EN  (1U << 1)
#define GPIO_PORTC_CLOCK_EN  (1U << 2)
#define GPIO_PORTD_CLOCK_EN  (1U << 3)

void GPIOA_LED_Buzzer_Init(void);
void GPIOB_Init(void);
void GPIOC_Init(void);
void GPIOD_Init(void);

#endif /* GPIO_H */
