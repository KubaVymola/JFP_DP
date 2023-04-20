//==============================================================================
// j_packet_send_callback.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef JPACKETSENDCALLBACK_H
#define JPACKETSENDCALLBACK_H

#include <stdint.h>

/**
 * Implements platform-dependent transmit function
*/
uint8_t j_packet_send_callback(uint8_t *Buf, uint16_t Len);

#endif
