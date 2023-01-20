#include "bmi160.h"

void bmi160_init(struct bmi160_t *hbmi160,
                 SPI_HandleTypeDef *hspi,
                 uint8_t sd0_pin,
                 uint8_t accel_rate,
                 uint8_t accel_range,
                 uint8_t gyro_rate,
                 uint8_t gyro_range) {
    // TODO add checks on each write or read and return error code

    // Initialize the struct
    hbmi160->address = sd0_pin == 0
        ? BMI160_I2C_ADDRESS
        : BMI160_I2C_ADDRESS_ALT;
    hbmi160->hspi = hspi;

    hbmi160->acc_x = 0.0f;
    hbmi160->acc_y = 0.0f;
    hbmi160->acc_z = 1.0f;

    hbmi160->gyro_x = 0.0f;
    hbmi160->gyro_y = 0.0f;
    hbmi160->gyro_z = 0.0f;

    /**
     * Enable SPI interface by sending rising edge to CS
     */
    bmi160_get_chip_id(hbmi160);
    
    bmi160_softreset(hbmi160);

    bmi160_config_accel(hbmi160, accel_rate, accel_range);
    bmi160_config_gyro(hbmi160, gyro_rate, gyro_range);

    bmi160_start_accel(hbmi160);
    bmi160_start_gyro(hbmi160);
}

void bmi160_softreset(struct bmi160_t *hbmi160) {
    // Perform the softreset
    uint8_t buf[2] = { BMI160_CMD_ADDRESS, BMI160_CMD_SOFTRESET };

    bmi160_spi_write(hbmi160->hspi, buf, 2);

    // Wait for the command to complete
    HAL_Delay(5);
}

void bmi160_start_accel(struct bmi160_t *hbmi160) {
    // Start the accelerometer
    uint8_t buf[2] = { BMI160_CMD_ADDRESS, BMI160_CMD_SET_ACCEL | 0x01 };
    
    bmi160_spi_write(hbmi160->hspi, buf, 2);

    // Wait for the command to complete
    HAL_Delay(5);
}

void bmi160_start_gyro(struct bmi160_t *hbmi160) {
    // Start the gyroscope
    uint8_t buf[2] = { BMI160_CMD_ADDRESS, BMI160_CMD_SET_GYRO | 0x01 };

    bmi160_spi_write(hbmi160->hspi, buf, 2);

    // Wait for the command to complete
    HAL_Delay(5);
}


uint8_t bmi160_get_chip_id(struct bmi160_t *hbmi160) {
    uint8_t buf[1] = { 0 };

    // uint8_t reg_addr = BMI160_WHOAMI_ADDRESS | 0x80;

    bmi160_spi_read(hbmi160->hspi, BMI160_WHOAMI_ADDRESS, buf, 1);

    return buf[0];
}

uint8_t bmi160_test_chip_id(struct bmi160_t *hbmi160) {
    return bmi160_get_chip_id(hbmi160) == BMI160_WHOAMI_VALUE;
}


void bmi160_config_accel(struct bmi160_t *hbmi160, uint8_t accel_rate, uint8_t accel_range) {
    {
        // Config accel output data rate (ODR) and bandwidth parameter (BWP)
        uint8_t buf[2] = { BMI160_ACC_CONF_ADDRESS, (uint8_t)(accel_rate | BMI160_ACC_BWP_NORMAL) };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }

    {
        // Config accel range
        uint8_t buf[2] = { BMI160_ACC_RANGE_ADDRESS, accel_range };

        bmi160_spi_write(hbmi160->hspi, buf, 2);
        
        if (accel_range == BMI160_ACC_RANGE_2G) {
            hbmi160->accel_divide_factor = BMI160_ACC_RANGE_2G_DIVIDE;
        } else if (accel_range == BMI160_ACC_RANGE_4G) {
            hbmi160->accel_divide_factor = BMI160_ACC_RANGE_4G_DIVIDE;
        } else if (accel_range == BMI160_ACC_RANGE_8G) {
            hbmi160->accel_divide_factor = BMI160_ACC_RANGE_8G_DIVIDE;
        } else if (accel_range == BMI160_ACC_RANGE_16G) {
            hbmi160->accel_divide_factor = BMI160_ACC_RANGE_16G_DIVIDE;
        }

        HAL_Delay(5);
    }
}

void bmi160_config_gyro(struct bmi160_t *hbmi160, uint8_t gyro_rate, uint8_t gyro_range) {
    {
        // Config gyro output data rate (ODR) and bandwidth parameter (BWP)
        uint8_t buf[2] = { BMI160_GYRO_CONF_ADDRESS, (uint8_t)(gyro_rate | BMI160_GYRO_BWP_NORMAL) };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }

    {
        // Config gyro range
        uint8_t buf[2] = { BMI160_GYRO_RANGE_ADDRESS, gyro_range };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        if (gyro_range == BMI160_GYRO_RANGE_2000DPS) {
            hbmi160->gyro_divide_factor = BMI160_GYRO_RANGE_2000DPS_DIVIDE;
        } else if (gyro_range == BMI160_GYRO_RANGE_1000DPS) {
            hbmi160->gyro_divide_factor = BMI160_GYRO_RANGE_1000DPS_DIVIDE;
        } else if (gyro_range == BMI160_GYRO_RANGE_500DPS) {
            hbmi160->gyro_divide_factor = BMI160_GYRO_RANGE_500DPS_DIVIDE;
        } else if (gyro_range == BMI160_GYRO_RANGE_250DPS) {
            hbmi160->gyro_divide_factor = BMI160_GYRO_RANGE_250DPS_DIVIDE;
        } else if (gyro_range == BMI160_GYRO_RANGE_125DPS) {
            hbmi160->gyro_divide_factor = BMI160_GYRO_RANGE_125DPS_DIVIDE;
        }

        HAL_Delay(5);
    }
}

uint8_t bmi160_get_data_rdy(struct bmi160_t *hbmi160) {
    uint8_t buf[1];

    bmi160_spi_read(hbmi160->hspi, BMI160_STATUS_ADDRESS, buf, 1);

    return buf[0] | 0xC0;
}

void bmi160_update_acc_gyro_data(struct bmi160_t *hbmi160) {
    uint8_t buf[12];

    bmi160_spi_read(hbmi160->hspi, BMI160_DATA_GYRO_ADDRESS, buf, 12);

    // HAL_I2C_Mem_Read(hbmi160->hi2c, hbmi160->address, BMI160_DATA_GYRO_ADDRESS, I2C_MEMADD_SIZE_8BIT, buf, 12, BMI160_DEFAULT_TIMEOUT);

    hbmi160->gyro_raw[0] = (buf[1] << 8) | buf[0];
    hbmi160->gyro_raw[1] = (buf[3] << 8) | buf[2];
    hbmi160->gyro_raw[2] = (buf[5] << 8) | buf[4];

    hbmi160->acc_raw[0] = (buf[7]  << 8) | buf[6];
    hbmi160->acc_raw[1] = (buf[9]  << 8) | buf[8];
    hbmi160->acc_raw[2] = (buf[11] << 8) | buf[10];

    hbmi160->gyro_x = hbmi160->gyro_raw[0] / hbmi160->gyro_divide_factor;
    hbmi160->gyro_y = hbmi160->gyro_raw[1] / hbmi160->gyro_divide_factor;
    hbmi160->gyro_z = hbmi160->gyro_raw[2] / hbmi160->gyro_divide_factor;

    hbmi160->acc_x = (hbmi160->acc_raw[0]) / hbmi160->accel_divide_factor;
    hbmi160->acc_y = (hbmi160->acc_raw[1]) / hbmi160->accel_divide_factor;
    hbmi160->acc_z = (hbmi160->acc_raw[2]) / hbmi160->accel_divide_factor;
}

float bmi160_get_temp_data(struct bmi160_t *hbmi160) {
    uint8_t buf[2];

    bmi160_spi_read(hbmi160->hspi, BMI160_TEMPERATURE_ADDRESS, buf, 2);

    return 23 + ((int16_t)((buf[1] << 8) | buf[0]) / 512.0f);
}

void bmi160_configure_interrupts(struct bmi160_t *hbmi160) {
    {
        // Enable the data ready interrupt engine
        uint8_t buf[2] = { BMI160_INT_EN_ADDRESS_1, BMI160_INT_EN_DATA_READY };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }

    {
        // Enable output on the INT1 pin and set it to active-high; the pin will be push-pull
        uint8_t buf[2] = { BMI160_INT_OUT_CTRL_ADDRESS, BMI160_INT_OUT_CTRL_INT1_OE | BMI160_INT_OUT_CTRL_INT1_LVL };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }

    {
        // Configure the interrupt to not latch
        uint8_t buf[2] = { BMI160_INT_LATCH_ADDRESS, BMI160_INT_LATCH_NOT_LATCHED };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }

    {
        // Map the data ready interrupt to the INT1 pin
        uint8_t buf[2] = { BMI160_INT_MAP_ADDRESS_1, BMI160_INT_MAP_INT1_DRDY };

        bmi160_spi_write(hbmi160->hspi, buf, 2);

        HAL_Delay(5);
    }
}

void bmi160_spi_write(SPI_HandleTypeDef *hspi, uint8_t *buf, uint8_t len) {
    HAL_GPIO_WritePin(BMI160_CS_GPIO_Port, BMI160_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, buf, len, 1000);
    HAL_GPIO_WritePin(BMI160_CS_GPIO_Port, BMI160_CS_Pin, GPIO_PIN_SET);
}

void bmi160_spi_read(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buf, uint8_t len) {
    uint8_t addr_flag = addr | 0x80;
    
    HAL_GPIO_WritePin(BMI160_CS_GPIO_Port, BMI160_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &addr_flag, 1, 1000);
    HAL_SPI_Receive(hspi, buf, len, 1000);
    HAL_GPIO_WritePin(BMI160_CS_GPIO_Port, BMI160_CS_Pin, GPIO_PIN_SET);
}

