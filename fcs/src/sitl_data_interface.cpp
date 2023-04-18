#include "sitl_data_interface.h"

#include "global_variables.h"
#include "string.h"
#include "j_packets.h"
#include "j_packet_recv_callback.h"

#ifdef SITL

extern "C" uint8_t data_from_jsbsim(uint8_t *data, int len) {
    j_packet_recv(data, len, j_packet_recv_callback);

    return 0;
}

extern "C" uint8_t data_to_jsbsim(uint8_t *buf) {
    if (to_jsbsim_queue.empty()) {
        buf = nullptr;
        return 0;
    }

    uint8_t *data_ptr = to_jsbsim_queue.front().data;
    uint8_t len = to_jsbsim_queue.front().len;
    
    memcpy(buf, data_ptr, len);
    to_jsbsim_queue.pop();

    return len;
}

#endif // SITL
