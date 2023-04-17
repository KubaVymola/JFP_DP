#ifndef REALTIMELOOP_H
#define REALTIMELOOP_H

#include <mutex>

#include "nlohmann/json.hpp"

#include "sim_events.h"

using json = nlohmann::json;

/**
 * A class than runs only in real-time telemetry mode. Is a substitude of JSBSimInterface.
 * The class fires EVENT_SIM_BEFORE_ITER, EVENT_SIM_AFTER_ITER events in fixed intervals.
 * This can be used to visualize an aircraft in real-time, or to simply download saved telemetry
 * from the flight controller - in both cases there's no simulation needed.
*/
class RealtimeLoop {
public:
    /**
     * Endless function that fires events each iteration and sleeps for a given period of time.
    */
    void realtime_iter(bool *continue_running,
                       json *sim_data,
                       std::mutex& sim_data_lock,
                       SimEvents& sim_events);
};

#endif
