#ifndef HITLINTERFACE_H
#define HITLINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "tinyxml2.h"

#include "sim_config.h"
#include "i_sim_client.h"

using json = nlohmann::json;

class HITLInterface : public ISimClient {
public:
    HITLInterface() { }
    ~HITLInterface() { close(hitl_device); }
    void hitl_init(sim_config_t& sim_config,
                   json *sim_data);
                
    void parse_xml_config(sim_config_t& sim_config);

    virtual void handle_event(const std::string& event_name, json *sim_data);

private:
    int round_down_to_multiple(int number, int multiple);
    void send_data_usb(int serial_port, int channel_number, void *data, uint16_t data_size, int item_size, int buffer_size);
    void receive_data_usb(int serial_port, json *sim_data);

    int hitl_device;

    std::vector<std::string> from_jsbsim_properties;
    std::vector<std::string> to_jsbsim_properties;
};

#endif
