#ifndef UART_H
#define UART_H

void UART0_Init(void);
void UART0_SendString(char *str);
void uart0_tx_char(char c);
void uart0_tx_string(const char *s);
void uart0_tx_float(float f);

#endif /* UART_H */
