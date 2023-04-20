//==============================================================================
// j_packet_recv_callback.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "j_packet_recv_callback.h"

#include <string.h>
#include "j_packets.h"
#include "global_variables.h"

extern "C" void j_packet_recv_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset) {
    /**
     * Cmd data in
     */
    if (channel_number == 0x00) {
        memcpy(cmd_string + data_offset, current_data + J_PACKET_HEADER_SIZE, data_size);
    }

    /**
     * SITL/HITL data in
     */
    if (channel_number == 0x02) {
        memcpy((uint8_t *)from_jsbsim + data_offset, current_data + J_PACKET_HEADER_SIZE, data_size);
    }

    /**
     * There is no data in on channel 1 (it is for telemetry output) or channel 3 (debug output)
     */
}