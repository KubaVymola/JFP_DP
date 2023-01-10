#include "usb-packets.h"

#include "usbd_cdc_if.h"
#include "constants.h"
#include "utils.h"

__weak void packet_from_usb_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset) {
}

/**
 * Send data up to 64 - HEADER_SIZE - FOOTER_SIZE bytes. Currently can send only one packet.
 * 
 * @param channel_number 0: commands, 1: telemetry, 2: HITL data
 * @param data pointer to the data
 * @param data_size how many bytes of data to send
 * @param item_size what is the size of unseparable unit (sizeof(<datatype>), e.g. sizeof(float))
*/
void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size) {
    // HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    
    uint16_t current_offset = 0;
    uint16_t maximum_data_in_packet = round_down_to_multiple(buffer_size - PACKET_HEADER_SIZE - PACKET_FOOTER_SIZE, item_size);

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

        memcpy(to_send + PACKET_HEADER_SIZE, (uint8_t *)data + current_offset, to_send_size);

        to_send[PACKET_HEADER_SIZE + to_send_size] = '\0';

        while (CDC_Transmit_FS(to_send, PACKET_HEADER_SIZE + to_send_size + PACKET_FOOTER_SIZE) == USBD_BUSY);

        current_offset += to_send_size;
    }
}


void process_packet_from_usb(uint8_t* Buf, int len) {
    uint8_t *current_data = Buf;

    /**
     * There must be at least PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE left in the buffer
     * in order for the packet to be valid
    */
    while (current_data - Buf + PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE <= len) {
        uint8_t channel_number  =  current_data[0];
        uint16_t data_size      = (current_data[3] << 8) | current_data[2];
        uint16_t data_offset    = (current_data[5] << 8) | current_data[4];

        /**
         * Incomplete packet received
         */
        if (current_data - Buf + PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE > len) break;

        /**
         * Invalid packet received (no 0 at the end)
         */
        if (channel_number > 0x02
        || current_data[PACKET_HEADER_SIZE + data_size] != '\0'
        || current_data[1] != 0x55
        || data_size > 64
        || data_offset > 512) {
            current_data++;
            continue;
        }
        
        // TODO move these USB packet functions outside of the project to share it with FDM

        packet_from_usb_callback(channel_number, current_data, data_size, data_offset);

        current_data += PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE;
    }
}
