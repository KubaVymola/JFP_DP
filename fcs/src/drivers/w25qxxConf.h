//==============================================================================
// w25qxx.c
//==============================================================================
//
// Author: Nima Askari (github: https://github.com/nimaltd)
// Date: 11/10/2021
//
//==============================================================================

#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#include "main.h"

#define _W25QXX_SPI                   hspi1
#define _W25QXX_CS_GPIO               SPI1_CS_GPIO_Port
#define _W25QXX_CS_PIN                SPI1_CS_Pin
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

#endif
