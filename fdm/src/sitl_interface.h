#ifndef SITLINTERFACE_H
#define SITLINTERFACE_H

#include <string>

#include "nlohmann/json.hpp"

#include "sim_config.h"
#include "i_sim_client.h"

using json = nlohmann::json;

typedef void (*init_func)(json *);
typedef void (*data_to_fcs_func)(json *);
typedef void (*loop_func)(void);
typedef void (*data_from_fcs_func)(json *);


class SITLInterface : public ISimClient {
public:
    void sitl_init(sim_config_t& sim_config,
                  json *sim_data);

    // void sitl_iter(json *sim_data, std::mutex& sim_data_lock);
    virtual void handle_event(const std::string& event_name, json *sim_data);
private:
    unsigned int iter_num;
    unsigned int sitl_div;

    init_func init_fcs;
    data_to_fcs_func data_to_fcs;
    loop_func loop_fcs;
    data_from_fcs_func data_from_fcs;
};

#endif
