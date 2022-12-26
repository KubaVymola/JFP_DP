#ifndef REALTIMELOOP_H
#define REALTIMELOOP_H

#include <mutex>

#include "nlohmann/json.hpp"

#include "sim_events.h"

using json = nlohmann::json;

class RealtimeLoop {
public:
    void realtime_iter(bool *continue_running,
                       json *sim_data,
                       std::mutex& sim_data_lock,
                       SimEvents& sim_events);
};

#endif
