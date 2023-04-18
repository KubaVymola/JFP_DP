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




/**
 * Send data up to 64 - HEADER_SIZE - FOOTER_SIZE bytes. Performs marshalling of data buffer into
 * individual packets.
 * 
 * @param channel_number 0: commands, 1: telemetry, 2: SITL/HITL data, 3: debug log
 * @param data Pointer to the data
 * @param data_size How many bytes of data to send
 * @param item_size What is the size of unseparable unit (sizeof(<datatype>), e.g. sizeof(float))
 * @param buffer_size The maximum size of the sending buffer. Is usually called with J_PACKET_SIZE, which is 64 B, but the design supports arbitrary buffer sizes
 * @param j_packet_send_callback A platform-dependent function used to transmit single packet. j_packet_send_callback can be called many times for each call of the j_packet_send function
 * @return -1 on error, 0 on success
*/
extern "C" int j_packet_send(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size, std::function<uint8_t(uint8_t *Buf, uint16_t Len)> j_packet_send_callback);

/**
 * Demashalls a stream of j-packets in Buf, and integrates the actual data to an outer state by calling the j_packet_recv_callback hook.
 * 
 * @param Buf A buffer of an arbitrary size and with arbitrary amount of j-packets
 * @param len The legth of the buffer
 * @param j_packet_recv_callback Callback function that integrates the received data into outer state
 * @return -1 on error, number of processed bytes on success (inlcuding header and footer bytes)
*/
extern "C" int j_packet_recv(uint8_t *Buf, int len, std::function<void(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset)> j_packet_recv_callback);

#endif
