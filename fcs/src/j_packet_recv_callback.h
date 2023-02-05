#ifndef JPACKETRECVCALLBACK_H
#define JPACKETRECVCALLBACK_H

#include <stdint.h>

extern "C" void j_packet_recv_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset);

#endif
