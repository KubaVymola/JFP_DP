#ifndef ISIMCLIENT_H
#define ISIMCLIENT_H

#include <string>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

class ISimClient {
public:
    virtual void handle_event(const std::string& event_name, json* sim_data) = 0;
};

#endif
