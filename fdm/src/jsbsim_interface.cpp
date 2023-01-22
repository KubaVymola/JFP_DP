#include "jsbsim_interface.h"

#include <iostream>
#include <sys/time.h>

#include "constants.h"
#include "FGFDMExec.h"
#include "models/FGInertial.h"
#include "models/FGPropagate.h"
#include "models/FGMassBalance.h"
#include "input_output/FGScript.h"
#include "simgear/misc/sg_path.hxx"
#include "nlohmann/json.hpp"
#include "tinyxml2.h"

using json = nlohmann::json;

double getcurrentseconds(void) {
    struct timeval tval;
    struct timezone tz;

    gettimeofday(&tval, &tz);
    return (tval.tv_sec + tval.tv_usec*1e-6);
}

void sim_nsleep(long nanosec) {
    struct timespec ts, ts1;
    ts.tv_sec = 0;
    ts.tv_nsec = nanosec;
    nanosleep(&ts, &ts1);
}

JSBSimInterface::JSBSimInterface() {
    FDMExec = new JSBSim::FGFDMExec();
}
JSBSimInterface::~JSBSimInterface() {
    delete FDMExec;
}

void JSBSimInterface::init(sim_config_t& sim_config,
                           json *sim_data) {

    parse_xml_config(sim_config, sim_data);

    if (!sim_config.rt_telem) {
        jsbsim_init(sim_config, sim_data);
    }
}

void JSBSimInterface::parse_xml_config(sim_config_t& sim_config,
                                       json *sim_data) {

    using namespace tinyxml2;

    XMLDocument craft_doc(true, COLLAPSE_WHITESPACE);
    craft_doc.LoadFile(sim_config.craft_config_path.c_str());
    XMLElement *root_elem = craft_doc.FirstChildElement("fdm_config");
    
    // TODO allow default values for all properties
    
    /**
     * Get properties for the 3d visualizer
     */
    if (sim_config.ws_port != 0) {

        XMLElement *ws_elem = root_elem->FirstChildElement("ws");
        
        XMLElement *ws_from_jsb_elem = ws_elem->FirstChildElement("from_jsbsim");
        XMLElement *ws_from_jsb_prop_elem = ws_from_jsb_elem->FirstChildElement("property");

        for (; 
            ws_from_jsb_prop_elem != nullptr;
            ws_from_jsb_prop_elem = ws_from_jsb_prop_elem->NextSiblingElement("property")) {
            std::string from_jsbsim_property(ws_from_jsb_prop_elem->GetText());

            properties_from_jsbsim.push_back(from_jsbsim_property);
            (*sim_data)[from_jsbsim_property] = 0.0;
        }

        XMLElement *ws_to_jsb_elem = ws_elem->FirstChildElement("to_jsbsim");
        XMLElement *ws_to_jsb_prop_elem = ws_to_jsb_elem->FirstChildElement("property");

        for (;
            ws_to_jsb_prop_elem != nullptr;
            ws_to_jsb_prop_elem = ws_to_jsb_prop_elem->NextSiblingElement("property")) {
            std::string to_jsbsim_property(ws_to_jsb_prop_elem->GetText());

            properties_to_jsbsim.push_back(to_jsbsim_property);
            (*sim_data)[to_jsbsim_property] = 0.0;
        }
    }


    /**
     * Get properties for FCS interface
     */
    if (!sim_config.sitl_path.empty() || sim_config.use_hitl) {

        XMLElement *fcs_elem = root_elem->FirstChildElement("fcs_interface");
        
        XMLElement *fcs_to_jsb_elem      = fcs_elem->FirstChildElement("to_jsbsim");
        XMLElement *fcs_to_jsb_prop_elem = fcs_to_jsb_elem->FirstChildElement("property");

        for (; fcs_to_jsb_prop_elem != nullptr; fcs_to_jsb_prop_elem = fcs_to_jsb_prop_elem->NextSiblingElement("property")) {
            std::string to_jsbsim_property(fcs_to_jsb_prop_elem->GetText());

            properties_to_jsbsim.push_back(to_jsbsim_property);
            (*sim_data)[to_jsbsim_property] = 0.0;

            // ? Default JSBSim data is supplied to FDMExec later (because now FDMExec is now null)
        }

        XMLElement *fcs_from_jsb_elem      = fcs_elem->FirstChildElement("from_jsbsim");
        XMLElement *fcs_from_jsb_prop_elem = fcs_from_jsb_elem->FirstChildElement("property");

        for (; fcs_from_jsb_prop_elem != nullptr; fcs_from_jsb_prop_elem = fcs_from_jsb_prop_elem->NextSiblingElement("property")) {
            std::string from_jsbsim_property(fcs_from_jsb_prop_elem->GetText());

            properties_from_jsbsim.push_back(from_jsbsim_property);
            (*sim_data)[from_jsbsim_property] = 0.0;
        }
    }


    if (!sim_config.log_output_def.empty()) {
        SGPath log_output_def_path = SGPath(sim_config.root_dir)/sim_config.log_output_def;
        
        XMLDocument out_log_doc(true, COLLAPSE_WHITESPACE);
        out_log_doc.LoadFile(log_output_def_path.c_str());

        XMLElement *root_out_elem = out_log_doc.FirstChildElement("output");
        XMLElement *property_out_elem = root_out_elem->FirstChildElement("property");

        for (; property_out_elem != nullptr; property_out_elem = property_out_elem->NextSiblingElement("property")) {
            properties_from_jsbsim.push_back(property_out_elem->GetText());
        }
    }


    std::sort(properties_from_jsbsim.begin(), properties_from_jsbsim.end());
    properties_from_jsbsim.erase(std::unique(properties_from_jsbsim.begin(), properties_from_jsbsim.end()), properties_from_jsbsim.end());

    std::sort(properties_to_jsbsim.begin(), properties_to_jsbsim.end());
    properties_to_jsbsim.erase(std::unique(properties_to_jsbsim.begin(), properties_to_jsbsim.end()), properties_to_jsbsim.end());
    
}

void JSBSimInterface::jsbsim_init(sim_config_t& sim_config,
                                  json *sim_data) {
    was_paused = false;
    result = false;
    play_nice = false;

    override_sim_rate_value = 0.0;
    new_five_second_value = 0.0;
    actual_elapsed_time = 0;
    sim_lag_time = 0;
    cycle_duration = 0.0;
    initial_seconds = 0;
    current_seconds = 0.0;
    paused_seconds = 0.0;
    sleep_period = 0.01;

    sleep_nseconds = 0;

    printf("Script path: %s\n", sim_config.script_path.c_str());
    printf("Root dir: %s\n", sim_config.root_dir.c_str());
    
    FDMExec->SetRootDir(SGPath::fromLocal8Bit(sim_config.root_dir.c_str()));
    FDMExec->SetAircraftPath(SGPath("aircraft"));
    FDMExec->SetEnginePath(SGPath("engine"));
    FDMExec->SetSystemsPath(SGPath("systems"));
    FDMExec->SetOutputPath(SGPath("."));
    FDMExec->GetPropertyManager()->Tie("simulation/frame_start_time", &actual_elapsed_time);
    FDMExec->GetPropertyManager()->Tie("simulation/cycle_duration", &cycle_duration);

    if (sim_config.simulation_rate < 1.0 )
        FDMExec->Setdt(sim_config.simulation_rate);
    else
        FDMExec->Setdt(1.0 / sim_config.simulation_rate);

    if (sim_config.override_sim_rate) override_sim_rate_value = FDMExec->GetDeltaT();

    for (unsigned int i=0; i < sim_config.command_line_properties.size(); i++) {
        if (sim_config.command_line_properties[i].find("simulation") != std::string::npos) {
            if (FDMExec->GetPropertyManager()->GetNode(sim_config.command_line_properties[i])) {
                FDMExec->SetPropertyValue(sim_config.command_line_properties[i], sim_config.command_line_property_values[i]);
            }
        }
    }

    /**
     * Load script that loads all other files
     */
    result = FDMExec->LoadScript(SGPath::fromLocal8Bit(sim_config.script_path.c_str()),
                                 override_sim_rate_value);

    if (!result) {
        printf("Script %s was not successfully loaded\n", sim_config.script_path.c_str());
        //   delete FDMExec;
        exit(-1);
    }

    /**
     * Create all properties that are propagated back to JSBSim (to enable logging)
     */
    for (const std::string& to_jsbsim_property : properties_to_jsbsim) {
        if (FDMExec->GetPropertyManager()->GetNode(to_jsbsim_property) == nullptr) {
            FDMExec->SetPropertyValue(to_jsbsim_property, 0.0);
            printf("Created property %s\n", to_jsbsim_property.c_str());
        }
    }

    /**
     * Default values for extended properties that are not by default in the property catalogue
     */
    FDMExec->SetPropertyValue("ext/altitude-m", 0.0);
    FDMExec->SetPropertyValue("ext/latitude-deg", 0.0);
    FDMExec->SetPropertyValue("ext/latitude-rad", 0.0);
    FDMExec->SetPropertyValue("ext/longitude-deg", 0.0);
    FDMExec->SetPropertyValue("ext/longitude-rad", 0.0);

    FDMExec->SetPropertyValue("ext/euler-yaw", 0.0);
    FDMExec->SetPropertyValue("ext/euler-pitch", 0.0);
    FDMExec->SetPropertyValue("ext/euler-roll", 0.0);
    FDMExec->SetPropertyValue("ext/heading-true-deg", 0.0);

    FDMExec->SetPropertyValue("ext/local-q-1", 0.0);
    FDMExec->SetPropertyValue("ext/local-q-2", 0.0);
    FDMExec->SetPropertyValue("ext/local-q-3", 0.0);
    FDMExec->SetPropertyValue("ext/local-q-4", 0.0);

    FDMExec->SetPropertyValue("ext/ecef-q-1", 0.0);
    FDMExec->SetPropertyValue("ext/ecef-q-2", 0.0);
    FDMExec->SetPropertyValue("ext/ecef-q-3", 0.0);
    FDMExec->SetPropertyValue("ext/ecef-q-4", 0.0);

    FDMExec->SetPropertyValue("ext/cg-x-m", 0.0);
    FDMExec->SetPropertyValue("ext/cg-y-m", 0.0);
    FDMExec->SetPropertyValue("ext/cg-z-m", 0.0);

    /**
     * Load output directives file[s], if given
     */
    for (unsigned int i=0; i < sim_config.jsbsim_outputs.size(); i++) {
        SGPath logOutput = SGPath::fromLocal8Bit(sim_config.jsbsim_outputs[i].c_str());
        
        if (!logOutput.isNull()) {
            if (!FDMExec->SetOutputDirectives(logOutput)) {
                std::cout << "Output directives not properly set in file " << sim_config.jsbsim_outputs[i] << std::endl;
            }
        }
    }

    FDMExec->RunIC();
    
    /**
     * Set simulation options from command line
     */
    for (unsigned int i=0; i < sim_config.command_line_properties.size(); i++) {
        if (FDMExec->GetPropertyManager()->GetNode(sim_config.command_line_properties[i])) {
            printf("Setting %s to %f\n", sim_config.command_line_properties[i].c_str(), sim_config.command_line_property_values[i]);
            FDMExec->SetPropertyValue(sim_config.command_line_properties[i], sim_config.command_line_property_values[i]);
        } else {
            printf("Unknown property to set: %s\n", sim_config.command_line_properties[i].c_str());
        }
    }

    /**
     * Update sim_data from the initial run, but do not fire event
     * ^ don't care about mutexes since the server should not be running at this point
     */
    update_sim_data(sim_data);
    
    std::cout << "sim_data" << sim_data->dump(4) << std::endl;

    FDMExec->PrintSimulationConfiguration();
    FDMExec->GetPropagate()->DumpState();

    if (sim_config.print_all_properties) {
        FDMExec->PrintPropertyCatalog();
    }
}

void JSBSimInterface::jsbsim_iter(sim_config_t& sim_config,
                                  bool *continue_running,
                                  json *sim_data,
                                  std::mutex& sim_data_lock,
                                  SimEvents& sim_events) {
    std::cout << endl << JSBSim::FGFDMExec::fggreen << JSBSim::FGFDMExec::highint
              << "---- JSBSim Execution beginning ... --------------------------------------------"
              << JSBSim::FGFDMExec::reset << std::endl << std::endl;


    char s[100];
    time_t tod;
    time(&tod);
    struct tm local;
    localtime_r(&tod, &local);
    strftime(s, 99, "%A %B %d %Y %X", &local);
    std::cout << "Start: " << s << " (HH:MM:SS)" << std::endl;

    frame_duration = FDMExec->GetDeltaT();
    if (sim_config.realtime) {
        sleep_nseconds = (long)(frame_duration*1e9);
    } else {
        sleep_nseconds = (sleep_period)*1e9; // 0.01 seconds
    }

    tzset();
    current_seconds = initial_seconds = getcurrentseconds();

    while ((sim_config.end_time < 0.0 && *continue_running)
        || (result && *continue_running && (FDMExec->GetSimTime() <= sim_config.end_time || sim_config.end_time == 0.0))) {


        // if (sim_config.end_time > 0.0 && FDMExec->GetSimTime() > sim_config.end_time) {
        //     printf("Sim end\n");
        //     *continue_running = false;
        //     break;
        // }
        // Check if increment then hold is on and take appropriate actions if it is
        // Iterate is not supported in realtime - only in batch and playnice modes
        FDMExec->CheckIncrementalHold();

        // if running realtime, throttle the execution, else just run flat-out fast
        // unless "playing nice", in which case sleep for a while (0.01 seconds) each frame.
        // If suspended, then don't increment cumulative realtime "stopwatch".

        if ( ! FDMExec->Holding()) {
            if ( ! sim_config.realtime ) {
                /**
                 * Batch mode
                 */
                
                result = handle_iter(sim_data, sim_data_lock, sim_events);

                if (play_nice) sim_nsleep(sleep_nseconds);
            } else {
                /**
                 * Realtime
                 */

                // "was_paused" will be true if entering this "run" loop from a paused state.
                if (was_paused) {
                    initial_seconds += paused_seconds;
                    was_paused = false;
                }

                current_seconds = getcurrentseconds();                      // Seconds since 1 Jan 1970
                actual_elapsed_time = current_seconds - initial_seconds;    // Real world elapsed seconds since start
                sim_lag_time = actual_elapsed_time - FDMExec->GetSimTime(); // How far behind sim-time is from actual
                                                                    // elapsed time.
                for (int i = 0; i < (int)(sim_lag_time / frame_duration); i++) {  // catch up sim time to actual elapsed time.
                    result = handle_iter(sim_data, sim_data_lock, sim_events);

                    cycle_duration = getcurrentseconds() - current_seconds;   // Calculate cycle duration
                    current_seconds = getcurrentseconds();                    // Get new current_seconds
                    if (FDMExec->Holding()) break;
                }

                if (play_nice) sim_nsleep(sleep_nseconds);

                if (FDMExec->GetSimTime() >= new_five_second_value) { // Print out elapsed time every five seconds.
                    std::cout << "Simulation elapsed time: " << FDMExec->GetSimTime() << std::endl;
                    new_five_second_value += 5.0;
                }
            }
        } else { // Suspended
            was_paused = true;
            paused_seconds = getcurrentseconds() - current_seconds;
            sim_nsleep(sleep_nseconds);
            result = handle_iter(sim_data, sim_data_lock, sim_events);
        }
    }

    *continue_running = false;

    time(&tod);
    localtime_r(&tod, &local);
    strftime(s, 99, "%A %B %d %Y %X", &local);
    std::cout << "End: " << s << " (HH:MM:SS)" << std::endl;
}

bool JSBSimInterface::handle_iter(json *sim_data,
                                  std::mutex& sim_data_lock,
                                  SimEvents& sim_events) {
    
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);

        /**
         * Update JSBSim internal state from sim_data after the sim:before_iter event
         */
        sim_events.notify_all(EVENT_SIM_BEFORE_ITER, sim_data);
        update_sim_properties(sim_data);
    }
    
    bool l_result = FDMExec->Run();

    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        
        /**
         * Update sim_data from JSBSim internal state before the sim:after_iter event
         */
        update_sim_data(sim_data);
        sim_events.notify_all(EVENT_SIM_AFTER_ITER, sim_data);
    }

    return l_result;
}

void JSBSimInterface::update_sim_properties(json *sim_data) {
    /**
     * Take data from FCS and ws (user ctrl) stored in sim_data JSON and use them to update JSBSim
     */

    for (const std::string& to_jsbsim_prop : properties_to_jsbsim) {
        FDMExec->SetPropertyValue(to_jsbsim_prop, sim_data->value<double>(to_jsbsim_prop, 0.0));
    }
}

void JSBSimInterface::update_sim_data(json *sim_data) {
    /**
     * Default properties defined by the interface
     */
    FDMExec->SetPropertyValue("ext/altitude-m", FDMExec->GetPropagate()->GetAltitudeASL() * FT_TO_M);
    FDMExec->SetPropertyValue("ext/latitude-deg", FDMExec->GetPropagate()->GetLatitudeDeg());
    FDMExec->SetPropertyValue("ext/latitude-rad", FDMExec->GetPropagate()->GetLatitude());
    FDMExec->SetPropertyValue("ext/longitude-deg", FDMExec->GetPropagate()->GetLongitudeDeg());
    FDMExec->SetPropertyValue("ext/longitude-rad", FDMExec->GetPropagate()->GetLongitude());

    FDMExec->SetPropertyValue("ext/euler-yaw", FDMExec->GetPropagate()->GetEuler(3) * RAD_TO_DEG);
    FDMExec->SetPropertyValue("ext/euler-pitch", FDMExec->GetPropagate()->GetEuler(2) * RAD_TO_DEG);
    FDMExec->SetPropertyValue("ext/euler-roll", FDMExec->GetPropagate()->GetEuler(1) * RAD_TO_DEG);
    FDMExec->SetPropertyValue("ext/heading-true-deg", FDMExec->GetPropertyValue("attitude/heading-true-rad") * RAD_TO_DEG);

    JSBSim::FGQuaternion local_quaternion = FDMExec->GetPropagate()->GetQuaternion();
    FDMExec->SetPropertyValue("ext/local-q-1", local_quaternion(1));
    FDMExec->SetPropertyValue("ext/local-q-2", local_quaternion(2));
    FDMExec->SetPropertyValue("ext/local-q-3", local_quaternion(3));
    FDMExec->SetPropertyValue("ext/local-q-4", local_quaternion(4));

    JSBSim::FGQuaternion ecef_quaternion = FDMExec->GetPropagate()->GetQuaternionECEF();
    FDMExec->SetPropertyValue("ext/ecef-q-1", ecef_quaternion(1));
    FDMExec->SetPropertyValue("ext/ecef-q-2", ecef_quaternion(2));
    FDMExec->SetPropertyValue("ext/ecef-q-3", ecef_quaternion(3));
    FDMExec->SetPropertyValue("ext/ecef-q-4", ecef_quaternion(4));

    FDMExec->SetPropertyValue("ext/cg-x-m", FDMExec->GetMassBalance()->GetXYZcg(1) * IN_TO_M);
    FDMExec->SetPropertyValue("ext/cg-y-m", FDMExec->GetMassBalance()->GetXYZcg(2) * IN_TO_M);
    FDMExec->SetPropertyValue("ext/cg-z-m", FDMExec->GetMassBalance()->GetXYZcg(3) * IN_TO_M);

    /**
     * Update properties from jsbsim into the sim_data for 3d visualization 
     */
    for (const std::string& to_sim_data_prop : properties_from_jsbsim) {
        (*sim_data)[to_sim_data_prop] = FDMExec->GetPropertyValue(to_sim_data_prop.c_str());
    }
}