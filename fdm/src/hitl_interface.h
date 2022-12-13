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
    int hitl_device;

    std::vector<std::string> from_jsbsim_properties;
    std::vector<std::string> to_jsbsim_properties;
};

#endif
