#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c0_init(void);
void i2c0_wait(void);
void i2c0_write_reg(uint8_t dev, uint8_t reg, uint8_t val);
void i2c0_read_multi(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* I2C_H */
