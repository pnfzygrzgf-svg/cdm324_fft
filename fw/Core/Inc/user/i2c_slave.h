#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#include "user/defines.h"

#define I2C_SLAVE_ADDR  0x20  /* 7-bit address 0x10, left-shifted for HAL */

void i2c_slave_init(void);
void i2c_slave_update_speed(uint16_t speed_kmh);

#endif
