#include "utils.h"

#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "j_packets.h"
#include "j_packet_send_callback.h"

const float get_x_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * cos(-yaw_rad) + y_world * sin(-yaw_rad);
}

const float get_y_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * -sin(-yaw_rad) + y_world * cos(-yaw_rad);
}

const float vector_norm(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

void send_jpacket_info(uint8_t channel, const char *text, int max_msg_len, ...) {
    va_list argp;

    char buf[max_msg_len];

    va_start(argp, max_msg_len);
    vsnprintf(buf, sizeof(buf), text, argp);
    va_end(argp);

    j_packet_send(channel, buf, strlen(buf), 1, J_PACKET_SIZE, j_packet_send_callback);
}
