#ifndef USBPACKETS_H
#define USBPACKETS_H

#define J_PACKET_HEADER_SIZE            6 /* 1B (channel) 1B (0x55) 2B (size) 2B (offset) */
#define J_PACKET_FOOTER_SIZE            1 /* 1B (0x00) */

#define J_PACKET_SIZE                   64

#ifndef LOBYTE
#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#endif /* LOBYTE */

#ifndef HIBYTE
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
#endif /* HIBYTE */

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#include <stdint.h>
#include <functional>

int round_down_to_multiple(int number, int multiple);
extern "C" void j_packet_send(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size, std::function<uint8_t(uint8_t *Buf, uint16_t Len)> j_packet_send_callback);
extern "C" void j_packet_recv(uint8_t *Buf, int len, std::function<void(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset)> j_packet_recv_callback);

#endif
