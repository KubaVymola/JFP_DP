#ifndef JSBSIM_INTERFACE_H
#define JSBSIM_INTERFACE_H

#include <string>
#include <condition_variable>
#include <mutex>
#include <vector>

#include "FGFDMExec.h"
#include "nlohmann/json.hpp"

#include "sim_config.h"
#include "sim_events.h"

using json = nlohmann::json;

class JSBSimInterface {
public:
    JSBSimInterface();
    ~JSBSimInterface();

    void init(sim_config_t& sim_config,
            json *sim_data);

    void parse_xml_config(sim_config_t& sim_config,
                          json *sim_data);

    void jsbsim_init(sim_config_t& sim_config,
                    json *sim_data);

    void jsbsim_iter(sim_config_t& sim_config,
                    bool *continue_running,
                    json *sim_data,
                    std::mutex& sim_data_lock,
                    SimEvents& sim_events);
                    
    bool handle_iter(json *sim_data,
                    std::mutex& sim_data_lock,
                    SimEvents& sim_events);

    void update_sim_properties(json *sim_data);
    void update_sim_data(json *sim_data);

private:
    JSBSim::FGFDMExec* FDMExec;

    std::vector<std::string> properties_from_jsbsim;
    std::vector<std::string> properties_to_jsbsim;

    bool was_paused;
    bool result;
    bool play_nice;

    double override_sim_rate_value;
    double new_five_second_value;
    double actual_elapsed_time;
    double sim_lag_time;
    double cycle_duration;
    double frame_duration;
    double initial_seconds;
    double current_seconds;
    double paused_seconds;
    double sleep_period;

    long sleep_nseconds;
};

#endif
