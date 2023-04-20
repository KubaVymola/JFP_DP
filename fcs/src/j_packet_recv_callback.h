//==============================================================================
// j_packet_recv_callback.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef JPACKETRECVCALLBACK_H
#define JPACKETRECVCALLBACK_H

#include <stdint.h>

extern "C" void j_packet_recv_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset);

#endif
