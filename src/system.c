#include "system.h"
#include "TM4C123.h"

/* ===== PLL INITIALIZATION (80 MHz) ===== */
void PLL_Init(void)
{
    SYSCTL->RCC2 |=  0x80000000;
    SYSCTL->RCC2 |=  0x00000800;
    SYSCTL->RCC   = (SYSCTL->RCC  & ~0x000007C0) + 0x00000540;
    SYSCTL->RCC2 &= ~0x00000070;
    SYSCTL->RCC2 &= ~0x00002000;
    SYSCTL->RCC2 |=  0x40000000;
    SYSCTL->RCC2  = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22);
    while ((SYSCTL->RIS & 0x00000040) == 0) {}
    SYSCTL->RCC2 &= ~0x00000800;
}

/* ===== SYSTICK BLOCKING DELAY ===== */
void systick_delay_ms(uint32_t ms)
{
    SysTick->LOAD = (SYSCLK / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5U;
    for (uint32_t i = 0; i < ms; i++) {
        while ((SysTick->CTRL & 0x10000U) == 0);
    }
    SysTick->CTRL = 0;
}
