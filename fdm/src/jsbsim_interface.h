#ifndef JSBSIM_INTERFACE_H
#define JSBSIM_INTERFACE_H

#include <string>
#include <condition_variable>
#include <mutex>

#include "nlohmann/json.hpp"

#include "sim_config.h"

using json = nlohmann::json;

void jsbsim_init(sim_config_t& sim_config,
                 bool *continue_running,
                 json *sim_data,
                 std::mutex& sim_data_lock);
                 
void update_data(json *sim_data,
                 std::mutex& sim_data_lock);

#endif
