#ifndef SIMEVENTS_H
#define SIMEVENTS_H

#include <string>
#include <vector>
#include <map>

#include "nlohmann/json.hpp"

#include "i_sim_client.h"

using json = nlohmann::json;

class SimEvents {
public:
    void register_client(const std::string& event_name, ISimClient *sim_client);
    void notify_all(const std::string& event_name, json *sim_data);
private:
    std::map<std::string, std::vector<ISimClient *>> registered_clients;
};

#endif
