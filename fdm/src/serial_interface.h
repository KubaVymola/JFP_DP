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

class SerialInterface : public ISimClient {
public:
    SerialInterface() { }
    ~SerialInterface() { close(serial_device); }
    void serial_init(sim_config_t& sim_config,
                     json *sim_data,
                     CommandInterface *command_interface);
                
    void parse_xml_config(sim_config_t& sim_config);

    virtual void handle_event(const std::string& event_name, json *sim_data);

private:
    void process_new_telem_line(json *sim_data);
    int round_down_to_multiple(int number, int multiple);
    void send_data_usb(int serial_port, int channel_number, void *data, uint16_t data_size, int item_size, int buffer_size);
    void receive_data_usb(int serial_port, json *sim_data);

    int serial_device;
    CommandInterface *command_interface;

    char new_telem_line[4096];
    int new_telem_line_ptr;

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
