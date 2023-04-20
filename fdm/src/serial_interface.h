//==============================================================================
// serial_interface.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef HITLINTERFACE_H
#define HITLINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "tinyxml2.h"

#include "sim_config.h"
#include "i_sim_client.h"
#include "command_interface.h"

using json = nlohmann::json;

/**
 * SerialInterface encapsulates the USB communication between an FCS running on a flight controller.
 * 
 * This communication contains simulated data and commands in HITL simulation, and real-world sensor
 * data and commands in real-time telemetry mode.
 * 
 * There is a telemetry stream present in the data that gets saved to a file.
 * 
 * There's also a command input/output stream that gets relayed between a socket server and the FCS.
 * 
 * There can be a debug stream that gets printed to a terminal with printf
*/
class SerialInterface : public ISimClient {
public:
    SerialInterface() { }
    ~SerialInterface() { close(serial_device); }
    void serial_init(sim_config_t& sim_config,
                     json *sim_data,
                     CommandInterface *command_interface);
                
    void parse_xml_config(sim_config_t& sim_config);

    /**
     * On EVENT_SIM_BEFORE_ITER event, the SerialInterface receives and processes data from the USB
     * connection. This data gets included in the sim_data object, making it accessible to
     * JSBSimInterface and by extension to JSBSim. This data can contain motor commands.
     * 
     * On EVENT_SIM_AFTER_ITER event, the SerialInterface sends new simulation data from JSBSim over
     * the USB to the FCS. This data can contain sensor values.
    */
    virtual void handle_event(const std::string& event_name, json *sim_data);

private:
    void process_new_telem_line(json *sim_data);
    void receive_data_usb(json *sim_data);

    uint8_t j_packet_send_callback(uint8_t* Buf, uint16_t Len);
    void j_packet_recv_callback(json *sim_data, uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset);

    int serial_device;
    CommandInterface *command_interface;

    char new_telem_line[4096];
    int new_telem_line_ptr;

    uint8_t packet_buf[4096];
    int buf_carry = 0;

    bool use_hitl;
    bool use_rt_telem;
    bool use_replay_telem;

    std::ofstream save_telemetry_file;

    std::vector<std::string> from_jsbsim_properties;
    std::vector<std::string> to_jsbsim_properties;
    std::vector<std::string> telemetry_properties;
    
    std::map<int, std::string> telemetry_mapping;
};

#endif
