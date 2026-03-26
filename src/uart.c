#include "uart.h"
#include "TM4C123.h"

/* ===== UART0: 9600 baud, 8N1, PA0=RX PA1=TX ===== */
void UART0_Init(void)
{
    SYSCTL->RCGCUART |= 1;
    SYSCTL->RCGCGPIO |= 1;
    for (volatile int i = 0; i < 3; i++);

    GPIOA->AFSEL |=  0x03;
    GPIOA->PCTL   = (GPIOA->PCTL & 0xFFFFFF00) | 0x00000011;
    GPIOA->DEN   |=  0x03;

    UART0->CTL  &= ~0x01;       /* Disable UART before config */
    UART0->IBRD  =  104;        /* 16 MHz / (16 * 9600) = 104.167 */
    UART0->FBRD  =  11;
    UART0->LCRH  =  0x60;       /* 8-bit, no parity, 1 stop, no FIFO */
    UART0->CC    =  0x0;        /* System clock source */
    UART0->CTL  |=  0x301;      /* Enable TX, RX, UART */
}

void UART0_SendString(char *str)
{
    while (*str) {
        while ((UART0->FR & 0x20) != 0);
        UART0->DR = *str++;
    }
}

void uart0_tx_char(char c)
{
    while (UART0->FR & (1U << 5));
    UART0->DR = (uint32_t)c;
}

void uart0_tx_string(const char *s)
{
    while (*s) uart0_tx_char(*s++);
}

/* Sends a float as "±ddd.ff\0" (2 decimal places) */
void uart0_tx_float(float f)
{
    if (f < 0.0f) { uart0_tx_char('-'); f = -f; }

    int i    = (int)f;
    int frac = (int)((f - (float)i) * 100.0f);
    if (frac < 0) frac = -frac;

    char buf[16];
    int pos = 0;

    if (i == 0) {
        buf[pos++] = '0';
    } else {
        int tmp = i, digits = 0;
        while (tmp > 0) { tmp /= 10; digits++; }
        for (int d = digits - 1; d >= 0; d--) {
            buf[pos + d] = (char)('0' + (i % 10));
            i /= 10;
        }
        pos += digits;
    }

    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (frac / 10));
    buf[pos++] = (char)('0' + (frac % 10));
    buf[pos]   = 0;

    uart0_tx_string(buf);
}
