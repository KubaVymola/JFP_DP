#ifndef HP203B_H
#define HP203B_H

#include <stdint.h>

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_i2c.h"

uint8_t hp203b_test(I2C_HandleTypeDef *hi2c);
void hp203b_setup(I2C_HandleTypeDef *hi2c);
float hp203b_get_pressure(I2C_HandleTypeDef *hi2c);


#endif
