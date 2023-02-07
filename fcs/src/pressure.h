#ifndef PRESSURE_H
#define PRESSURE_H

#include <math.h>
#include "defines.h"

inline float get_alt_asl_from_pressure(float pressure) {
    return (288.15f / -0.0065f) * (powf(pressure / 101325.0f, (-8.314f * -0.0065f) / (G_TO_MPS * 0.0289640f)) - 1);
}

#endif
