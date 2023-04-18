#ifndef SITLDATAINTERFACE_H
#define SITLDATAINTERFACE_H

#ifdef SITL

#include <stdint.h>

/**
 * Gets called by fdm's SITLInterface object. Transports data in a j-packet from fdm to FCS.
*/
extern "C" uint8_t data_from_jsbsim(uint8_t *data, int len);

/**
 * Copies a single j-packet from a to_jsbsim_queue to a buffer buf provided by fdm's SITLInterface
 * object.
*/
extern "C" uint8_t data_to_jsbsim(uint8_t *buf);

#endif // SITL

#endif