#ifndef SITLDATAINTERFACE_H
#define SITLDATAINTERFACE_H

#ifdef SITL

#include <stdint.h>

extern "C" uint8_t data_from_jsbsim(uint8_t *data, int len);
extern "C" uint8_t data_to_jsbsim(uint8_t *buf);

#endif // SITL

#endif