#include "mpu6050.h"
#include "i2c.h"
#include "system.h"
#include "TM4C123.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ===== EXPORTED ANGLE STATE ===== */
float roll        = 0.0f;
float pitch       = 0.0f;
float yaw         = 0.0f;

/* ===== EXPORTED GYRO BIAS ===== */
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;


/* ===== MPU6050 INITIALIZATION =====
   Wakes device, sets sample rate divider, DLPF, and full-scale ranges.
   ±250 °/s gyro (131 LSB/°/s), ±2 g accel (16384 LSB/g).
*/
void mpu6050_init(void)
{
    systick_delay_ms(50);
    i2c0_write_reg(MPU6050_ADDR, 0x6B, 0x00);  /* Wake up (clear sleep bit) */
    i2c0_write_reg(MPU6050_ADDR, 0x19, 0x07);  /* Sample rate divider = 7 */
    i2c0_write_reg(MPU6050_ADDR, 0x1A, 0x03);  /* DLPF = 44 Hz accel / 42 Hz gyro */
    i2c0_write_reg(MPU6050_ADDR, 0x1B, 0x00);  /* Gyro  ±250 °/s */
    i2c0_write_reg(MPU6050_ADDR, 0x1C, 0x00);  /* Accel ±2 g      */
    systick_delay_ms(50);
}

uint8_t mpu6050_whoami(void)
{
    uint8_t id;
    i2c0_read_multi(MPU6050_ADDR, 0x75, &id, 1);
    return id;
}

/* ===== READ RAW 16-BIT ACCEL AND GYRO VALUES ===== */
void mpu6050_read_raw(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    i2c0_read_multi(MPU6050_ADDR, 0x3B, buf, 14);

    accel[0] = (int16_t)((buf[0]  << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2]  << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, skip */
    gyro[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
}

/* ===== COMPLEMENTARY FILTER ANGLE UPDATE =====
   Call once per control loop after mpu6050_read_raw().
   Updates global roll, pitch, yaw.
   dt: elapsed time in seconds since last call.
*/
void mpu6050_update_angles(float dt)
{
    int16_t accel_raw[3], gyro_raw[3];
    mpu6050_read_raw(accel_raw, gyro_raw);

    /* Convert accel to g */
    float ax = (float)accel_raw[0] / 16384.0f;
    float ay = (float)accel_raw[1] / 16384.0f;
    float az = (float)accel_raw[2] / 16384.0f;

    /* Convert gyro to deg/s and remove bias */
    float gx = ((float)gyro_raw[0] / 131.0f) - gyro_bias_x;
    float gy = ((float)gyro_raw[1] / 131.0f) - gyro_bias_y;
    float gz = ((float)gyro_raw[2] / 131.0f) - gyro_bias_z;

    /* Accel-derived angles */
    float accel_roll  = atan2f(ay, az)                         * 180.0f / M_PI;
    float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az))      * 180.0f / M_PI;

    /* Complementary filter */
    roll  = COMP_FILTER_ALPHA * (roll  + gx * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    pitch = COMP_FILTER_ALPHA * (pitch + gy * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
    yaw  += gz * dt;
}


/* ===== DWT CYCLE COUNTER ===== */
void dwt_init(void)
{
    CoreDebug->DEMCR |= 0x01000000U;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= 1U;
}

uint32_t dwt_get_cycles(void)
{
    return DWT->CYCCNT;
}
