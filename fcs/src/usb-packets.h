#ifndef USBPACKETS_H
#define USBPACKETS_H

#include <stdint.h>

void packet_from_usb_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset);

extern "C" {
    void process_packet_from_usb(uint8_t* Buf, int len);
    void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size);
}

#endif
