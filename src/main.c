#include "TM4C123.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "system.h"
#include "gpio.h"
#include "uart.h"
#include "adc.h"
#include "i2c.h"
#include "mpu6050.h"
#include "current_sense.h"
#include "timers.h"
#include "pid.h"

/* ===== GYRO CALIBRATION SAMPLES ===== */
#define GYRO_CAL_SAMPLES  500

/* ===== BASE MOTOR SPEED ===== */
static volatile int base_pulse = MOTOR_PULSE_BASE_US;

/* ===== PID TIMING ===== */
static float pid_time_accumulator = 0.0f;
static uint32_t last_cycles       = 0;


/* ────────────────────────────────────────────────────────────
   GYRO BIAS CALIBRATION
   Keep the beam perfectly still while this runs (~2.5 seconds).
   Fills gyro_bias_x/y/z in the mpu6050 module.
   ──────────────────────────────────────────────────────────── */
static void calibrate_gyro(void)
{
    uart0_tx_string("Calibrating gyro... keep sensor still\r\n");

    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t accel_raw[3], gyro_raw[3];

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        mpu6050_read_raw(accel_raw, gyro_raw);
        sum_gx += gyro_raw[0];
        sum_gy += gyro_raw[1];
        sum_gz += gyro_raw[2];
        systick_delay_ms(5);
    }

    gyro_bias_x = ((float)sum_gx / (float)GYRO_CAL_SAMPLES) / 131.0f;
    gyro_bias_y = ((float)sum_gy / (float)GYRO_CAL_SAMPLES) / 131.0f;
    gyro_bias_z = ((float)sum_gz / (float)GYRO_CAL_SAMPLES) / 131.0f;

    uart0_tx_string("Gyro calibration done.\r\n");
}


/* ────────────────────────────────────────────────────────────
   ESC ARMING SEQUENCE
   Standard arming: 1000 µs for ~1 s, then go to base speed.
   ──────────────────────────────────────────────────────────── */
static void arm_escs(void)
{
    uart0_tx_string("Arming ESCs...\r\n");
    set_pulse_width_us(1000, 0);
    for (volatile int i = 0; i < 120000; i++);
    set_pulse_width_us(1000, 1);
    for (volatile int i = 0; i < 12000000; i++);   /* ~1 s at 16 MHz */

    set_pulse_width_us(base_pulse, 0);
    set_pulse_width_us(base_pulse, 1);
    uart0_tx_string("ESCs armed.\r\n");
}


/* ────────────────────────────────────────────────────────────
   MAIN
   ──────────────────────────────────────────────────────────── */
int main(void)
{
    /* ---- Peripheral init ---- */
    PLL_Init();
    GPIOA_LED_Buzzer_Init();
    GPIOB_Init();
    GPIOC_Init();
    GPIOD_Init();
    UART0_Init();
    ADC0_Init();

    Timer1A_OneShot_Init();
    Timer3A_OneShot_Init();
    Timer0A_50Hz_Init();
    Timer2A_RPM_Measure_Init();
    WTIMER0A_CCP0_init();

    systick_delay_ms(100);
    i2c0_init();
    systick_delay_ms(100);
    mpu6050_init();

    __enable_irq();

    uart0_tx_string("\r\n=== PID Beam Balance Control ===\r\n");
    for (volatile int d = 0; d < 50000; d++);

    /* ---- Current sensor zero calibration (motors must be off) ---- */
    uart0_tx_string("Calibrating current sensors...\r\n");
    Calibrate_Current_Sensors();

    /* ---- MPU6050 identity check ---- */
    uint8_t id = mpu6050_whoami();
    uart0_tx_string("WHO_AM_I = 0x");
    if (id == 0x68) {
        uart0_tx_string("68 OK\r\n");
        GPIOA->DATA &= ~(1U << 7);    /* Buzzer OFF - sensor OK  */
    } else {
        uart0_tx_string("?? ERROR\r\n");
        GPIOA->DATA |=  (1U << 7);    /* Buzzer ON  - sensor fault */
    }

    /* ---- ESC arming ---- */
    arm_escs();

    /* ---- Gyro bias calibration ---- */
    calibrate_gyro();

    /* ---- Init DWT cycle counter ---- */
    dwt_init();
    last_cycles = dwt_get_cycles();

    /* ---- Local loop variables ---- */
    char     msg[128];
    uint32_t uart_counter = 0;

    /* ════════════════════════════════════════════════
       MAIN LOOP
       ════════════════════════════════════════════════ */
    while (1) {

        /* ---- Compute dt ---- */
        uint32_t now_cycles = dwt_get_cycles();
        uint32_t diff       = now_cycles - last_cycles;
        float    dt         = (float)diff / (float)SYSCLK;
        if (dt <= 0.0f) dt  = 0.001f;
        last_cycles         = now_cycles;

        /* ---- Update angles (complementary filter inside) ---- */
        mpu6050_update_angles(dt);

        /* ---- PID at fixed 200 Hz ---- */
        pid_time_accumulator += dt;
        if (pid_time_accumulator >= PID_PERIOD_S) {

            float pid_dt = pid_time_accumulator;
            pid_time_accumulator = 0.0f;

            float pid_output = pid_compute(TARGET_PITCH, pitch, pid_dt);

            /* Clamp adjustment to ±300 µs */
            long adjustment = (long)pid_output;
            if (adjustment >  300) adjustment =  300;
            if (adjustment < -300) adjustment = -300;

            set_pulse_width_us((unsigned long)(base_pulse - adjustment), 0);
            set_pulse_width_us((unsigned long)(base_pulse + 5 + adjustment), 1);
        }

        /* ---- Current sensing ---- */
        uint16_t adc1 = ADC0_Read(MOTOR1_ADC_CH);
        uint16_t adc2 = ADC0_Read(MOTOR2_ADC_CH);

        float i1_raw = Convert_ADC_To_Current(adc1, 2048.0f, MOTOR1_POLARITY);
        float i2_raw = Convert_ADC_To_Current(adc2, 2048.0f, MOTOR2_POLARITY);

        motor1_current_filt = LowPassFilter(motor1_current_filt, i1_raw);
        motor2_current_filt = LowPassFilter(motor2_current_filt, i2_raw);

        motor1_current_filt = Clamp(motor1_current_filt, MIN_ALLOWED_CURRENT_A, MAX_ALLOWED_CURRENT_A);
        motor2_current_filt = Clamp(motor2_current_filt, MIN_ALLOWED_CURRENT_A, MAX_ALLOWED_CURRENT_A);

        Current_Protection_Check(motor1_current_filt, motor2_current_filt);

        /* Green LED: ON when motors are drawing current */
        if (motor1_current_filt > 0.1f || motor2_current_filt > 0.1f) {
            GPIOA->DATA |=  (1U << 6);
        } else {
            GPIOA->DATA &= ~(1U << 6);
        }

        /* ---- UART telemetry every 50 cycles ---- */
        if (uart_counter % 50 == 0) {
            float error = TARGET_PITCH - pitch;

            uart0_tx_string("Pitch=");
            uart0_tx_float(pitch);
            uart0_tx_string(" | Err=");
            uart0_tx_float(error);

            sprintf(msg, " | P1=%lu P2=%lu | RPM1=%lu RPM2=%lu | I1=",
                    pulse_us_1, pulse_us_2, rpm_1, rpm_2);
            UART0_SendString(msg);

            uart0_tx_float(motor1_current_filt);
            uart0_tx_string("A I2=");
            uart0_tx_float(motor2_current_filt);
            uart0_tx_string("A\r\n");
        }

        uart_counter++;

        for (volatile int d = 0; d < 1000; d++);
    }
}
