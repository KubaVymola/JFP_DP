#include "realtime_loop.h"

#include <unistd.h>
#include <iostream>

void RealtimeLoop::realtime_iter(bool *continue_running,
                  json *sim_data,
                  std::mutex& sim_data_lock,
                  SimEvents& sim_events) {

    while (*continue_running) {
        usleep(1000000.0 / 120.0);

        (*sim_data)["simulation/sim-time-sec"] = sim_data->value<double>("simulation/sim-time-sec", 0.0) + (1.0 / 60.0);

        {
            std::lock_guard<std::mutex> guard(sim_data_lock);
            sim_events.notify_all(EVENT_SIM_BEFORE_ITER, sim_data);
            sim_events.notify_all(EVENT_SIM_AFTER_ITER, sim_data);
        }
    }
}