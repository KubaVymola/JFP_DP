//==============================================================================
// sim_events.cpp
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "sim_events.h"

void SimEvents::register_client(const std::string& event_name, ISimClient *sim_client) {
    registered_clients[event_name].push_back(sim_client);
}

void SimEvents::notify_all(const std::string& event_name, json *sim_data) {
    if (registered_clients.count(event_name) == 0) return;
    
    for (auto client : registered_clients[event_name]) {
        client->handle_event(event_name, sim_data);
    }
}
