//==============================================================================
// j_packet_send_callback.cpp
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "j_packet_send_callback.h"

#include "string.h"
#include "global_variables.h"

#ifdef MCU
#include "usbd_cdc_if.h"

uint8_t j_packet_send_callback(uint8_t *Buf, uint16_t Len) {
    return CDC_Transmit_FS(Buf, Len);
}
#endif // MCU


#ifdef SITL
uint8_t j_packet_send_callback(uint8_t *Buf, uint16_t Len) {
    to_jsbsim_t tmp_struct = { 0 };
    
    memcpy(tmp_struct.data, Buf, Len);
    tmp_struct.len = Len;

    to_jsbsim_queue.push(tmp_struct);

    return 0; // USBD_OK
}
#endif // SITL