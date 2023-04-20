//==============================================================================
// realtime_loop.cpp
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "realtime_loop.h"

#include <unistd.h>
#include <iostream>

void RealtimeLoop::realtime_iter(bool *continue_running,
                  json *sim_data,
                  std::mutex& sim_data_lock,
                  SimEvents& sim_events) {

    while (*continue_running) {
        usleep(1000000.0 / 360.0);

        (*sim_data)["simulation/sim-time-sec"] = sim_data->value<double>("simulation/sim-time-sec", 0.0) + (1.0 / 60.0);

        {
            std::lock_guard<std::mutex> guard(sim_data_lock);
            sim_events.notify_all(EVENT_SIM_BEFORE_ITER, sim_data);
            sim_events.notify_all(EVENT_SIM_AFTER_ITER, sim_data);
        }
    }
}