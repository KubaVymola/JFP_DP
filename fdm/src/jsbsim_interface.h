//==============================================================================
// jsbsim_interface.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef JSBSIM_INTERFACE_H
#define JSBSIM_INTERFACE_H

#include <string>
#include <condition_variable>
#include <mutex>
#include <vector>

#include "FGFDMExec.h"
#include "nlohmann/json.hpp"

#include "sim_config.h"
#include "sim_events.h"

using json = nlohmann::json;

/**
 * A wrapper class for JSBSim runtime. This class handles execution of SITL and HITL simulations
 * 
 * At each JSBSim iteration, it notifies all observers through the SimEvents objects. This creates
 * a hook for a SITLInterface (HITL or independent FCS), SITLInterface to read from/write to the
 * sim_data object, and StateLogger to log the internal state
*/
class JSBSimInterface {
public:
    JSBSimInterface();
    ~JSBSimInterface();

    void init(sim_config_t& sim_config,
            json *sim_data);

    void parse_xml_config(sim_config_t& sim_config,
                          json *sim_data);

    /**
     * Initialization inspired by JSBSim standalone execution
    */
    void jsbsim_init(sim_config_t& sim_config,
                    json *sim_data);

    /**
     * A function that will not exit until the simulation is complete (sim time is larger than end time).
     * It will fire EVENT_SIM_BEFORE_ITER and EVENT_SIM_AFTER_ITER events before and after each itertaion.
     * 
     * Implementation is inspired by JSBSim
    */
    void jsbsim_iter(sim_config_t& sim_config,
                    bool *continue_running,
                    json *sim_data,
                    std::mutex& sim_data_lock,
                    SimEvents& sim_events);
    
    bool handle_iter(json *sim_data,
                    std::mutex& sim_data_lock,
                    SimEvents& sim_events);

    /**
     * Copy properties defined in to_jsbim tags in configuration XMLs from sim_data to JSBSim.
     * Happens before JSBSim properties
    */
    void update_sim_properties(json *sim_data);

    /**
     * Copy properties defined in from_jsbim tags in configuration XMLs from JSBSim to sim_data.
     * Happens after every JSBSim iteration
    */
    void update_sim_data(json *sim_data);

private:
    JSBSim::FGFDMExec* FDMExec;

    std::vector<std::string> properties_from_jsbsim;
    std::vector<std::string> properties_to_jsbsim;

    bool was_paused;
    bool result;
    bool play_nice;

    double override_sim_rate_value;
    double new_five_second_value;
    double actual_elapsed_time;
    double sim_lag_time;
    double cycle_duration;
    double frame_duration;
    double initial_seconds;
    double current_seconds;
    double paused_seconds;
    double sleep_period;

    long sleep_nseconds;
};

#endif
