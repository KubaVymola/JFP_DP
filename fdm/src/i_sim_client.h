#ifndef ISIMCLIENT_H
#define ISIMCLIENT_H

#include <string>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

/**
 * An observer interface class
*/
class ISimClient {
public:
    /**
     * The update mathod of the observer
     * 
     * @param event_name Defined in sim_events.h. Those are EVENT_SIM_BEFORE_ITER and EVENT_SIM_AFTER_ITER
     * @param sim_data Allows the observer to update the sim_data JSON (the copy of JSBSim's internal state)
    */
    virtual void handle_event(const std::string& event_name, json* sim_data) = 0;
};

#endif
