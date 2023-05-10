//==============================================================================
// bmi160.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef BMI160_H
#define BMI160_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"


#define BMI160_I2C_ADDRESS              0xD0
#define BMI160_I2C_ADDRESS_ALT          0xD2

#define BMI160_DEFAULT_TIMEOUT          1000

#define BMI160_WHOAMI_ADDRESS           0x00
#define BMI160_WHOAMI_VALUE             0xD1

#define BMI160_PMU_STATUS_ADDRESS       0x03
#define BMI160_PMU_ACCEL_NORMAL         (0x01 << 4)
#define BMI160_PMU_GYRO_NORMAL          (0x01 << 2)

#define BMI160_DATA_MAG_ADDRESS         0x04
#define BMI160_DATA_GYRO_ADDRESS        0x0C
#define BMI160_DATA_ACCEL_ADDRESS       0x12

#define BMI160_STATUS_ADDRESS           0x1B
#define BMI160_STATUS_DATA_RDY_ACCEL    0x40
#define BMI160_STATUS_DATA_RDY_GYRO     0x20
#define BMI160_STATUS_DATA_RDY_MAG      0x10

#define BMI160_TEMPERATURE_ADDRESS      0x20

#define BMI160_ACC_CONF_ADDRESS         0x40
#define BMI160_ACC_RATE_1HZ             0x01
#define BMI160_ACC_RATE_25HZ            0x06
#define BMI160_ACC_RATE_50HZ            0x07
#define BMI160_ACC_RATE_100HZ           0x08
#define BMI160_ACC_RATE_200HZ           0x09
#define BMI160_ACC_RATE_400HZ           0x0A
#define BMI160_ACC_RATE_800HZ           0x0B
#define BMI160_ACC_RATE_1600HZ          0x0C
#define BMI160_ACC_BWP_NORMAL           (0x02 << 4)
#define BMI160_ACC_BWP_OSR2             (0x01 << 4)
#define BMI160_ACC_BWP_OSR4             (0x00 << 4)

#define BMI160_ACC_RANGE_ADDRESS        0x41
#define BMI160_ACC_RANGE_2G             0x03
#define BMI160_ACC_RANGE_4G             0x05
#define BMI160_ACC_RANGE_8G             0x08
#define BMI160_ACC_RANGE_16G            0x0C
#define BMI160_ACC_RANGE_2G_DIVIDE      (32768.0 / 2)
#define BMI160_ACC_RANGE_4G_DIVIDE      (32768.0 / 4)
#define BMI160_ACC_RANGE_8G_DIVIDE      (32768.0 / 8)
#define BMI160_ACC_RANGE_16G_DIVIDE     (32768.0 / 16)

#define BMI160_GYRO_CONF_ADDRESS        0x42
#define BMI160_GYRO_RATE_25HZ           0x06
#define BMI160_GYRO_RATE_50HZ           0x07
#define BMI160_GYRO_RATE_100HZ          0x08
#define BMI160_GYRO_RATE_200HZ          0x09
#define BMI160_GYRO_RATE_400HZ          0x0A
#define BMI160_GYRO_RATE_800HZ          0x0B
#define BMI160_GYRO_RATE_1600HZ         0x0C
#define BMI160_GYRO_RATE_3200HZ         0x0D
#define BMI160_GYRO_BWP_NORMAL          (0x02 << 4)
#define BMI160_GYRO_BWP_OSR2            (0x01 << 4)
#define BMI160_GYRO_BWP_OSR4            (0x00 << 4)

#define BMI160_GYRO_RANGE_ADDRESS       0x43
#define BMI160_GYRO_RANGE_2000DPS       0x00
#define BMI160_GYRO_RANGE_1000DPS       0x01
#define BMI160_GYRO_RANGE_500DPS        0x02
#define BMI160_GYRO_RANGE_250DPS        0x03
#define BMI160_GYRO_RANGE_125DPS        0x04
#define BMI160_GYRO_RANGE_2000DPS_DIVIDE (32768.0 / 2000)
#define BMI160_GYRO_RANGE_1000DPS_DIVIDE (32768.0 / 1000)
#define BMI160_GYRO_RANGE_500DPS_DIVIDE (32768.0 / 500)
#define BMI160_GYRO_RANGE_250DPS_DIVIDE (32768.0 / 250)
#define BMI160_GYRO_RANGE_125DPS_DIVIDE (32768.0 / 125)

#define BMI160_INT_EN_ADDRESS_0         0x50
#define BMI160_INT_EN_ADDRESS_1         0x51
#define BMI160_INT_EN_ADDRESS_2         0x52
#define BMI160_INT_EN_DATA_READY        (0x01 << 4)

#define BMI160_INT_OUT_CTRL_ADDRESS     0x53
#define BMI160_INT_OUT_CTRL_INT2_OE     0x80
#define BMI160_INT_OUT_CTRL_INT2_OD     0x40
#define BMI160_INT_OUT_CTRL_INT2_LVL    0x20
#define BMI160_INT_OUT_CTRL_INT1_OE     0x08
#define BMI160_INT_OUT_CTRL_INT1_OD     0x04
#define BMI160_INT_OUT_CTRL_INT1_LVL    0x02

#define BMI160_INT_LATCH_ADDRESS        0x54
#define BMI160_INT_LATCH_NOT_LATCHED    0x00
#define BMI160_INT_LATCH_LATCHED        0x0F

#define BMI160_INT_MAP_ADDRESS_0        0x55
#define BMI160_INT_MAP_ADDRESS_1        0x56
#define BMI160_INT_MAP_ADDRESS_2        0x57
#define BMI160_INT_MAP_INT1_DRDY        0x80

#define BMI160_CMD_ADDRESS              0x7E
#define BMI160_CMD_SET_ACCEL            0x10
#define BMI160_CMD_SET_GYRO             0x14
#define BMI160_CMD_RESET                0xB1
#define BMI160_CMD_SOFTRESET            0xB6


struct bmi160_t {
    SPI_HandleTypeDef *hspi;
    // I2C_HandleTypeDef *hi2c;

    uint8_t address;

    int16_t acc_raw[3];
    int16_t gyro_raw[3];

    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_divide_factor;
    float gyro_divide_factor;
};


/**
 * Initialize the BMI160 6DoF sensor.
 * Performs the configuration and start of accelerometer and gyroscope.
 * Does not configure interrupts.
 * @param hbmi160 Handle for the bmi160_t structure - contains handle to I2C and device address
 * @param hi2c Handle for the I2C which will get saved in the hbmi160 struct
 * @param sd0_pin 0 if SD0 is pulled to GND, 1 if SD0 is pulled to 3V3
 */
void bmi160_init(struct bmi160_t *hbmi160,
                 SPI_HandleTypeDef *hspi,
                 uint8_t sd0_pin,
                 uint8_t accel_rate,
                 uint8_t accel_range,
                 uint8_t gyro_rate,
                 uint8_t gyro_range);
                //  I2C_HandleTypeDef *hi2c,

/**
 * Called by the bmi160_init function
 */
void bmi160_softreset(struct bmi160_t *hbmi160);
/**
 * Called by the bmi160_init function
 */
void bmi160_start_accel(struct bmi160_t *hbmi160);
/**
 * Called by the bmi160_init function
 */
void bmi160_start_gyro(struct bmi160_t *hbmi160);

/**
 * @return If everything is OK, then the returned value should be OxD1
 */
uint8_t bmi160_get_chip_id(struct bmi160_t *hbmi160);
/**
 * @return Returns 1 if chip ID is equal to OxD1
 */
uint8_t bmi160_test_chip_id(struct bmi160_t *hbmi160);

/**
 * Sets range and rate to accelerometer and updates the scaling factor.
 * Called by the bmi160_init function.
 */
void bmi160_config_accel(struct bmi160_t *hbmi160, uint8_t acc_rate, uint8_t acc_range);
/**
 * Sets range and rate to gyro and updates the scaling factor.
 * Called by the bmi160_init function.
 */
void bmi160_config_gyro(struct bmi160_t *hbmi160, uint8_t gyro_rate, uint8_t gyro_range);


/**
 * @return 1 if either accel or gyro has new data ready
 */
uint8_t bmi160_get_data_rdy(struct bmi160_t *hbmi160);

/**
 * Updates the raw and scaled values in the bmi160_t struct for both accel and gyro
*/
void bmi160_update_acc_gyro_data(struct bmi160_t *hbmi160);
float bmi160_get_temp_data(struct bmi160_t *hbmi160);

void bmi160_configure_interrupts(struct bmi160_t *hbmi160);

void bmi160_spi_write(SPI_HandleTypeDef *hspi, uint8_t *buf, uint8_t len);
void bmi160_spi_read(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
} // extern "C"
#endif

#endif