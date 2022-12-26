#ifndef STATELOGGER_H
#define STATELOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "i_sim_client.h"
#include "sim_config.h"
#include "sim_events.h"

class StateLogger : public ISimClient {
public:
    StateLogger() { }
    ~StateLogger() { outstream.close(); }

    void init(sim_config_t& sim_config);
    virtual void handle_event(const std::string& event_name, json* sim_data);
private:
    // FILE *log_output_file;
    std::ofstream outstream;
    std::vector<std::string> logged_properties; // TODO allow default values
};

#endif
