#include "i2c.h"
#include "TM4C123.h"

/* ===== I2C0 INIT: PB2=SCL, PB3=SDA, ~100 kHz @ 16 MHz ===== */
void i2c0_init(void)
{
    SYSCTL->RCGCI2C  |= 1U;
    SYSCTL->RCGCGPIO |= (1U << 1);              /* Port B */
    while ((SYSCTL->PRGPIO & (1U << 1)) == 0);

    for (volatile int i = 0; i < 1000; i++);    /* Settle */

    I2C0->MCR  = (1U << 4);                     /* Master enable */
    I2C0->MTPR = 7;                             /* TPR = clk/(20*SCL)-1 */
}

/* ===== SPIN UNTIL BUS IS IDLE ===== */
void i2c0_wait(void)
{
    while (I2C0->MCS & 1U);
}

/* ===== WRITE ONE BYTE TO REGISTER ===== */
void i2c0_write_reg(uint8_t dev, uint8_t reg, uint8_t val)
{
    i2c0_wait();
    I2C0->MSA = (dev << 1);       /* Slave address, write */
    I2C0->MDR = reg;
    I2C0->MCS = 3U;               /* START + RUN */
    i2c0_wait();

    I2C0->MDR = val;
    I2C0->MCS = 5U;               /* RUN + STOP */
    i2c0_wait();
}

/* ===== READ MULTIPLE BYTES FROM REGISTER ===== */
void i2c0_read_multi(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Send register address */
    i2c0_wait();
    I2C0->MSA = (dev << 1);
    I2C0->MDR = reg;
    I2C0->MCS = 3U;               /* START + RUN */
    i2c0_wait();

    /* Switch to read mode */
    I2C0->MSA = (dev << 1) | 1U;
    if (len == 1) {
        I2C0->MCS = 7U;           /* START + RUN + STOP (single byte) */
    } else {
        I2C0->MCS = 0x0BU;        /* START + RUN + ACK */
    }

    for (uint8_t i = 0; i < len - 1; i++) {
        i2c0_wait();
        buf[i] = I2C0->MDR;
        if (i == len - 2)
            I2C0->MCS = 5U;       /* RUN + STOP (last-1 byte, NACK) */
        else
            I2C0->MCS = 9U;       /* RUN + ACK */
    }

    i2c0_wait();
    buf[len - 1] = I2C0->MDR;
}
