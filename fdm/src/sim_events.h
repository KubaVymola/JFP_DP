//==============================================================================
// sim_events.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef SIMEVENTS_H
#define SIMEVENTS_H

#define EVENT_SIM_BEFORE_ITER       "sim:before_iter"
#define EVENT_SIM_AFTER_ITER        "sim:after_iter"

#include <string>
#include <vector>
#include <map>

#include "nlohmann/json.hpp"

#include "i_sim_client.h"

using json = nlohmann::json;

/**
 * This class handles notifications for all SimEvents observers. Observers must implement the
 * ISimClient interface, i.e. the handle_event method.
 * 
 * There are currently two events supported: EVENT_SIM_BEFORE_ITER, EVENT_SIM_AFTER_ITER.
 * Each observer has to register to respective events that they want to react to.
 * 
 * The events are fired by JSBSimInterface and RealtimeLoop.
*/
class SimEvents {
public:
    void register_client(const std::string& event_name, ISimClient *sim_client);
    void notify_all(const std::string& event_name, json *sim_data);
private:
    std::map<std::string, std::vector<ISimClient *>> registered_clients;
};

#endif
