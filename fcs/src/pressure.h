#ifndef PRESSURE_H
#define PRESSURE_H

#include <math.h>
#include "defines.h"

/**
 * @param pressure In Pascals
 * @return Current above-sea-level altitude in an International standard atmosphere (1013.25 hPa, 15 °C)
*/
inline float get_alt_asl_from_pressure(float pressure) {
    return (288.15f / -0.0065f) * (powf(pressure / 101325.0f, (-8.314f * -0.0065f) / (G_TO_MPS * 0.0289640f)) - 1);
}

#endif
