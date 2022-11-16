#ifndef FCSINTERFACE_H
#define FCSINTERFACE_H

#include <string>
#include <mutex>

#include "nlohmann/json.hpp"

#include "sim_config.h"

using json = nlohmann::json;

typedef void (*init_func)(void);
typedef void (*data_to_fcs_func)(json*, std::mutex&);
typedef void (*loop_func)(void);
typedef void (*data_from_fcs_func)(json*, std::mutex&);

void fcs_init(sim_config_t& sim_config,
              bool *continue_running,
              json *sim_data,
              std::mutex& sim_data_lock);

#endif
