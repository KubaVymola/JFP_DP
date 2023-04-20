//==============================================================================
// hp203b.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "hp203b.h"

void hp203b_setup(struct hp203b_t *hhp203b, I2C_HandleTypeDef *hi2c, uint8_t csb_pin) {
    hhp203b->hi2c = hi2c;

    hhp203b->address = csb_pin == 0
        ? HP203B_CSB0_ADDR
        : HP203B_CSB1_ADDR;

    hp203b_softreset(hhp203b);
}

uint8_t hp203b_test(struct hp203b_t *hhp203b) {
    // 0b10nnnnnn
    uint8_t i2c_cmd = 0x80 | 0x0D; /* reg addr (INT_SRC) */;
    uint8_t buf[1];

    HAL_I2C_Master_Transmit(hhp203b->hi2c, hhp203b->address, &i2c_cmd, sizeof(i2c_cmd), 100);
    HAL_I2C_Master_Receive(hhp203b->hi2c, hhp203b->address, buf, sizeof(buf), 100);

    return buf[0] & 0x40;
}

void hp203b_softreset(struct hp203b_t *hhp203b) {
    uint8_t i2c_cmd = 0x06; /* Softreset */
    HAL_I2C_Master_Transmit(hhp203b->hi2c, hhp203b->address, &i2c_cmd, sizeof(i2c_cmd), 100);
}


float hp203b_get_pressure_pa(struct hp203b_t *hhp203b) {
    uint8_t i2c_cmd;
    uint8_t buf[3] = { 0 };

    i2c_cmd = 0x48; /* Preform ADC conversion */
    HAL_I2C_Master_Transmit(hhp203b->hi2c, hhp203b->address, &i2c_cmd, sizeof(i2c_cmd), 100);

    i2c_cmd = 0x30; /* Read pressure */
    HAL_I2C_Master_Transmit(hhp203b->hi2c, hhp203b->address, &i2c_cmd, sizeof(i2c_cmd), 100);
    HAL_I2C_Master_Receive(hhp203b->hi2c, hhp203b->address, buf, sizeof(buf), 100);

    return ((int)(buf[0] << 16 | buf[1] << 8 | buf[2]) & 0xFFFFF);
}

