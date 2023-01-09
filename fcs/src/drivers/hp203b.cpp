#include "hp203b.h"

#include "stm32l1xx_ll_i2c.h"

uint8_t hp203b_test(I2C_HandleTypeDef *hi2c) {
    uint8_t device_addr = 0xEE;     // CSB=0
    // uint8_t device_addr = 0xEC;     // CSB=1

    // 0b10nnnnnn
    uint8_t i2c_cmd[1] = { 0x80 | 0x0D }; /* reg addr (INT_SRC) */;
    uint8_t buf[1] = { 0 };

    HAL_I2C_Master_Transmit(hi2c, device_addr, i2c_cmd, sizeof(i2c_cmd), HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, device_addr, buf, sizeof(buf), HAL_MAX_DELAY);

    return buf[0] & 0x40;
}

void hp203b_setup(I2C_HandleTypeDef *hi2c) {
    uint8_t device_addr = 0xEE;     // CSB=0

    uint8_t i2c_cmd[1];

    i2c_cmd[0] = 0x06; /* Softreset */
    HAL_I2C_Master_Transmit(hi2c, device_addr, i2c_cmd, sizeof(i2c_cmd), HAL_MAX_DELAY);
}

float hp203b_get_pressure(I2C_HandleTypeDef *hi2c) {
    uint8_t device_addr = 0xEE;     // CSB=0

    uint8_t i2c_cmd[1];
    uint8_t buf[3] = { 0 };

    i2c_cmd[0] = 0x40; /* Preform ADC conversion. OSR=128 and CHNL=0 (pressure and temperature) */
    HAL_I2C_Master_Transmit(hi2c, device_addr, i2c_cmd, sizeof(i2c_cmd), HAL_MAX_DELAY);

    i2c_cmd[0] = 0x30; /* Read pressure */
    HAL_I2C_Master_Transmit(hi2c, device_addr, i2c_cmd, sizeof(i2c_cmd), HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, device_addr, buf, sizeof(buf), HAL_MAX_DELAY);

    return ((int)(buf[0] << 16 | buf[1] << 8 | buf[2]) & 0xFFFFF) / 100.0f;
}

