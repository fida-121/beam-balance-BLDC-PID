#include "adc.h"
#include "TM4C123.h"

/* ===== ADC0 INIT: Sequencer 3, software trigger, PE2/PE3 ===== */
void ADC0_Init(void)
{
    SYSCTL->RCGCADC  |= 1;
    SYSCTL->RCGCGPIO |= (1 << 4);          /* Port E clock */
    while (!(SYSCTL->PRGPIO & (1 << 4)));

    GPIOE->AFSEL |=  (1 << 2) | (1 << 3);
    GPIOE->DEN   &= ~((1 << 2) | (1 << 3));
    GPIOE->AMSEL |=  (1 << 2) | (1 << 3);

    ADC0->ACTSS  &= ~8;          /* Disable SS3 during config    */
    ADC0->EMUX   &= ~0xF000;     /* Software trigger for SS3     */
    ADC0->SSMUX3  =  0;          /* Channel set at read time     */
    ADC0->SSCTL3  =  0x06;       /* IE0 + END0                   */
    ADC0->ACTSS  |=  8;          /* Re-enable SS3                */
}

/* ===== ADC0 BLOCKING READ on given channel ===== */
uint16_t ADC0_Read(uint8_t channel)
{
    ADC0->SSMUX3 = channel;
    ADC0->PSSI   = 8;                          /* Initiate SS3 sample */
    while ((ADC0->RIS & 8) == 0);             /* Wait for conversion */
    uint16_t result = ADC0->SSFIFO3 & 0xFFF;
    ADC0->ISC = 8;                             /* Clear interrupt flag */
    return result;
}
