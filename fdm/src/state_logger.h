//==============================================================================
// state_logger.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef STATELOGGER_H
#define STATELOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "i_sim_client.h"
#include "sim_config.h"
#include "sim_events.h"

/**
 * Class used to output the JSBSim internal state. Is a substitude of JSBSim's native logging, and
 * it supports extended properties defiend only in the fdm, such as "ext/altitude-m"
*/
class StateLogger : public ISimClient {
public:
    StateLogger() { }
    ~StateLogger() { outstream.close(); }

    void init(sim_config_t& sim_config);

    /**
     * This function is only subsribed to the EVENT_SIM_AFTER_ITER event, and it outputs configured properties to
     * a CSV text file. The properties are configured in a output_definition XML file, that gets
     * passed in the CLI as the second positional argument. This file defines a default output
     * path, which can be overridden by "--output_path_override" CLI option.
    */
    virtual void handle_event(const std::string& event_name, json* sim_data);
private:
    std::ofstream outstream;
    std::vector<std::string> logged_properties; // TODO allow default values
};

#endif
