#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

/* ===== ACS712 SENSOR PARAMETERS ===== */
#define ACS712_SENSITIVITY      0.066f    /* V/A  (30A module = 0.066) */
#define CURRENT_FILTER_ALPHA    0.90f     /* LPF coefficient (0.85-0.95) */
#define MAX_ALLOWED_CURRENT_A   12.0f     /* Hard overcurrent limit (A)  */
#define MIN_ALLOWED_CURRENT_A    0.0f

/* ===== MOTOR POLARITY ===== */
/* Motor 1 sensor is mounted reversed physically */
#define MOTOR1_POLARITY   -1.0f
#define MOTOR2_POLARITY    1.0f

/* ===== EXPORTED STATE ===== */
extern volatile uint8_t overcurrent_fault;
extern float motor1_current_filt;
extern float motor2_current_filt;

/* ===== FUNCTIONS ===== */
void  Calibrate_Current_Sensors(void);
float Convert_ADC_To_Current(uint16_t adc, float zero_adc, float polarity);
float LowPassFilter(float prev, float input);
float Clamp(float val, float min, float max);
void  Current_Protection_Check(float i1, float i2);

#endif /* CURRENT_SENSE_H */
