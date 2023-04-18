#ifndef JPACKETSENDCALLBACK_H
#define JPACKETSENDCALLBACK_H

#include <stdint.h>

/**
 * Implements platform-dependent transmit function
*/
uint8_t j_packet_send_callback(uint8_t *Buf, uint16_t Len);

#endif
