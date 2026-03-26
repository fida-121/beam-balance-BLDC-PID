#include "current_sense.h"
#include "adc.h"

/* ===== MODULE-PRIVATE CALIBRATION OFFSETS ===== */
static float motor1_zero_adc = 2048.0f;
static float motor2_zero_adc = 2048.0f;

/* ===== EXPORTED FILTERED CURRENTS ===== */
float motor1_current_filt = 0.0f;
float motor2_current_filt = 0.0f;

/* ===== FAULT FLAG ===== */
volatile uint8_t overcurrent_fault = 0;


/* ===== ZERO-CURRENT CALIBRATION =====
   IMPORTANT: Call ONLY when motors are stopped.
   Averages 500 samples to find the ADC mid-point for each sensor.
*/
void Calibrate_Current_Sensors(void)
{
    float sum1 = 0.0f, sum2 = 0.0f;

    for (int i = 0; i < 500; i++) {
        sum1 += ADC0_Read(MOTOR1_ADC_CH);
        sum2 += ADC0_Read(MOTOR2_ADC_CH);
        for (volatile int d = 0; d < 2000; d++);
    }

    motor1_zero_adc = sum1 / 500.0f;
    motor2_zero_adc = sum2 / 500.0f;
}


/* ===== ADC COUNT -> AMPERES =====
   Converts raw ADC reading to current using:
     voltage = (adc / 4095) * 3.3
     current = (voltage - zero_voltage) / sensitivity * polarity
*/
float Convert_ADC_To_Current(uint16_t adc, float zero_adc, float polarity)
{
    float voltage  = ((float)adc      / ADC_MAX_COUNTS) * ADC_REF_VOLT;
    float zero_v   = (zero_adc        / ADC_MAX_COUNTS) * ADC_REF_VOLT;
    float diff_v   = voltage - zero_v;
    float current  = (diff_v / ACS712_SENSITIVITY) * polarity;
    return current;
}


/* ===== EXPONENTIAL LOW-PASS FILTER ===== */
float LowPassFilter(float prev, float input)
{
    return (CURRENT_FILTER_ALPHA * prev) +
           ((1.0f - CURRENT_FILTER_ALPHA) * input);
}


/* ===== CLAMP UTILITY ===== */
float Clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}


/* ===== OVERCURRENT PROTECTION =====
   Sets overcurrent_fault flag. Caller must take action (e.g. cut PWM).
*/
void Current_Protection_Check(float i1, float i2)
{
    if (i1 > MAX_ALLOWED_CURRENT_A || i2 > MAX_ALLOWED_CURRENT_A) {
        overcurrent_fault = 1;
        /* TODO: Disable motors / set PWM to minimum here */
    }
}
