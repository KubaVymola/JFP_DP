#ifndef HP203B_H
#define HP203B_H

#include <stdint.h>

#include "stm32xxxx_hal.h"
#include "stm32xxxx_hal_i2c.h"

#define HP203B_CSB0_ADDR        0xEE
#define HP203B_CSB1_ADDR        0xEC

struct hp203b_t {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
};

void hp203b_setup(struct hp203b_t *hhp203b, I2C_HandleTypeDef *hi2c, uint8_t csb_pin);
void hp203b_softreset(struct hp203b_t *hhp203b);
uint8_t hp203b_test(struct hp203b_t *hhp203b);
float hp203b_get_pressure_pa(struct hp203b_t *hhp203b);


#endif
