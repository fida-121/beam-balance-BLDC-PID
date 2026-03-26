#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* ===== MPU6050 I2C ADDRESS ===== */
#define MPU6050_ADDR   0x68

/* ===== COMPLEMENTARY FILTER ALPHA ===== */
/* Higher = trust gyro more, lower = trust accel more */
#define COMP_FILTER_ALPHA  0.95f

/* ===== EXPORTED ANGLE STATE ===== */
extern float roll;
extern float pitch;
extern float yaw;

/* ===== GYRO BIAS (set during calibration) ===== */
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

/* ===== FUNCTIONS ===== */
void    mpu6050_init(void);
uint8_t mpu6050_whoami(void);
void    mpu6050_read_raw(int16_t *accel, int16_t *gyro);
void    mpu6050_update_angles(float dt);

/* ===== DWT CYCLE COUNTER ===== */
void     dwt_init(void);
uint32_t dwt_get_cycles(void);

#endif /* MPU6050_H */
