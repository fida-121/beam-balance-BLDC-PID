#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* ===== ADC CHANNEL ASSIGNMENTS ===== */
#define MOTOR1_ADC_CH   1    /* PE2 -> AIN1 */
#define MOTOR2_ADC_CH   0    /* PE3 -> AIN0 */

/* ===== ADC REFERENCE ===== */
#define ADC_MAX_COUNTS  4095.0f
#define ADC_REF_VOLT    3.3f

void     ADC0_Init(void);
uint16_t ADC0_Read(uint8_t channel);

#endif /* ADC_H */
