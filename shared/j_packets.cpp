#include "j_packets.h"

#include <string.h>

int round_down_to_multiple(int number, int multiple) {
    return number - number % multiple;    
}

/**
 * Send data up to 64 - HEADER_SIZE - FOOTER_SIZE bytes. Currently can send only one packet.
 * 
 * @param channel_number 0: commands, 1: telemetry, 2: SITL/HITL data, 3: debug log
 * @param data pointer to the data
 * @param data_size how many bytes of data to send
 * @param item_size what is the size of unseparable unit (sizeof(<datatype>), e.g. sizeof(float))
*/
void j_packet_send(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size, std::function<uint8_t(uint8_t *Buf, uint16_t Len)> j_packet_send_callback) {
    uint16_t current_offset = 0;
    uint16_t maximum_data_in_packet = round_down_to_multiple(buffer_size - J_PACKET_HEADER_SIZE - J_PACKET_FOOTER_SIZE, item_size);

    uint8_t to_send[buffer_size];
    to_send[0] = channel_number;
    to_send[1] = 0x55;

    while (current_offset < data_size) {
        uint16_t remaining_size = data_size - current_offset;
        uint16_t to_send_size = MIN(remaining_size, maximum_data_in_packet);

        to_send[2] = LOBYTE(to_send_size);
        to_send[3] = HIBYTE(to_send_size);
        to_send[4] = LOBYTE(current_offset);
        to_send[5] = HIBYTE(current_offset);

        memcpy(to_send + J_PACKET_HEADER_SIZE, (uint8_t *)data + current_offset, to_send_size);

        to_send[J_PACKET_HEADER_SIZE + to_send_size] = '\0';

        while (1) {
            uint8_t result = j_packet_send_callback(to_send, J_PACKET_HEADER_SIZE + to_send_size + J_PACKET_FOOTER_SIZE);

            if (result == 0) break;     // equiv. to USBD_OK
            if (result == 1) continue;  // equiv. to USBD_BUSY
            if (result == 3) return;    // equiv. to USBD_FAIL
        }

        current_offset += to_send_size;
    }
}


void j_packet_recv(uint8_t *Buf, int len, std::function<void(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset)> j_packet_recv_callback) {
    uint8_t *current_data = Buf;

    /**
     * There must be at least J_PACKET_HEADER_SIZE + J_PACKET_FOOTER_SIZE left in the buffer
     * in order for the packet to be valid
    */
    while (current_data - Buf + J_PACKET_HEADER_SIZE + J_PACKET_FOOTER_SIZE <= len) {
        uint8_t channel_number  =  current_data[0];
        uint16_t data_size      = ((uint16_t)current_data[3] << 8) | (uint8_t)current_data[2];
        uint16_t data_offset    = ((uint16_t)current_data[5] << 8) | (uint8_t)current_data[4];

        /**
         * Invalid packet header received
         */
        if (channel_number > 0x03
        || current_data[1] != 0x55
        || data_size > J_PACKET_SIZE
        || data_offset > 1024) {
            current_data++;
            continue;
        }
        
        /**
         * Incomplete packet received
         */
        if (current_data - Buf + J_PACKET_HEADER_SIZE + data_size + J_PACKET_FOOTER_SIZE > len) {
            break;
        }


        /**
         * Invalid packet footer received
         */
        if (current_data[J_PACKET_HEADER_SIZE + data_size] != '\0') {
            current_data++;
            continue;
        }

        j_packet_recv_callback(channel_number, current_data, data_size, data_offset);

        current_data += J_PACKET_HEADER_SIZE + data_size + J_PACKET_FOOTER_SIZE;
    }
}
