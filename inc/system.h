#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

/* ===== SYSTEM CLOCK ===== */
#define SYSCLK      16000000UL

void PLL_Init(void);
void systick_delay_ms(uint32_t ms);

#endif /* SYSTEM_H */
