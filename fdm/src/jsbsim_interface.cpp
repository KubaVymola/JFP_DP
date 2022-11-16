#include "jsbsim_interface.h"

#include <iostream>
#include <sys/time.h>

#include "constants.h"
#include "FGFDMExec.h"
#include "models/FGInertial.h"
#include "models/FGPropagate.h"
#include "models/FGMassBalance.h"
#include "simgear/misc/sg_path.hxx"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

JSBSim::FGFDMExec* FDMExec;

// extern bool realtime;
// extern bool simulation_rate;
// extern bool end_time;
// bool override_sim_rate = false;

// double end_time = 1e99;

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

void jsbsim_init(sim_config_t& sim_config,
                 bool *continue_running,
                 json *sim_data,
                 std::mutex& sim_data_lock) {
    bool was_paused = false;
    bool result = false;
    bool play_nice = false;

    double override_sim_rate_value = 0.0;
    double new_five_second_value = 0.0;
    double actual_elapsed_time = 0;
    double sim_lag_time = 0;
    double cycle_duration = 0.0;
    double frame_duration;
    double initial_seconds = 0;
    double current_seconds = 0.0;
    double paused_seconds = 0.0;
    double sleep_period = 0.01;

    long sleep_nseconds = 0;

                    
    FDMExec = new JSBSim::FGFDMExec();

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
      delete FDMExec;
      exit(-1);
    }

    /**
     * Load output directives file[s], if given
     */
    for (unsigned int i=0; i < sim_config.log_outputs.size(); i++) {
        SGPath logOutput = SGPath::fromLocal8Bit(sim_config.log_outputs[i].c_str());
        
        if (!logOutput.isNull()) {
            if (!FDMExec->SetOutputDirectives(logOutput)) {
                cout << "Output directives not properly set in file " << sim_config.log_outputs[i] << endl;
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

    update_data(sim_data, sim_data_lock);

    FDMExec->PrintSimulationConfiguration();

    FDMExec->GetPropagate()->DumpState();

    if (sim_config.print_all_properties) {
        FDMExec->PrintPropertyCatalog();
    }

    cout << endl << JSBSim::FGFDMExec::fggreen << JSBSim::FGFDMExec::highint
         << "---- JSBSim Execution beginning ... --------------------------------------------"
         << JSBSim::FGFDMExec::reset << endl << endl;


    char s[100];
    time_t tod;
    time(&tod);
    struct tm local;
    localtime_r(&tod, &local);
    strftime(s, 99, "%A %B %d %Y %X", &local);
    cout << "Start: " << s << " (HH:MM:SS)" << endl;

    frame_duration = FDMExec->GetDeltaT();
    if (sim_config.realtime) {
        sleep_nseconds = (long)(frame_duration*1e9);
    } else {
        sleep_nseconds = (sleep_period)*1e9; // 0.01 seconds
    }

    tzset();
    current_seconds = initial_seconds = getcurrentseconds();

    while (result
           && *continue_running
           && (FDMExec->GetSimTime() <= sim_config.end_time || sim_config.end_time == 0.0)) {
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
                
                result = FDMExec->Run();
                update_data(sim_data, sim_data_lock);

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
                    result = FDMExec->Run();
                    update_data(sim_data, sim_data_lock);

                    cycle_duration = getcurrentseconds() - current_seconds;   // Calculate cycle duration
                    current_seconds = getcurrentseconds();                    // Get new current_seconds
                    if (FDMExec->Holding()) break;
                }

                if (play_nice) sim_nsleep(sleep_nseconds);

                if (FDMExec->GetSimTime() >= new_five_second_value) { // Print out elapsed time every five seconds.
                    cout << "Simulation elapsed time: " << FDMExec->GetSimTime() << endl;
                    new_five_second_value += 5.0;
                }
            }
        } else { // Suspended
            was_paused = true;
            paused_seconds = getcurrentseconds() - current_seconds;
            sim_nsleep(sleep_nseconds);
            result = FDMExec->Run();
            update_data(sim_data, sim_data_lock);
        }
    }

    time(&tod);
    localtime_r(&tod, &local);
    strftime(s, 99, "%A %B %d %Y %X", &local);
    cout << "End: " << s << " (HH:MM:SS)" << endl;

    delete FDMExec;
}

void update_data(json *sim_data,
                 std::mutex& sim_data_lock) {
    {
        std::lock_guard<std::mutex> guard(sim_data_lock);
        (*sim_data)["time_sec"] = FDMExec->GetPropertyValue("simulation/sim-time-sec");
        (*sim_data)["altitude_m"] = (const double)FDMExec->GetPropagate()->GetAltitudeASL() * FT_TO_M;
        (*sim_data)["latitude_deg"] = FDMExec->GetPropagate()->GetLatitudeDeg();
        (*sim_data)["latitude_rad"] = FDMExec->GetPropagate()->GetLatitude();
        (*sim_data)["longitude_deg"] = FDMExec->GetPropagate()->GetLongitudeDeg();
        (*sim_data)["longitude_rad"] = FDMExec->GetPropagate()->GetLongitude();

        (*sim_data)["euler_yaw"] = FDMExec->GetPropagate()->GetEuler(3) * RAD_TO_DEG;
        (*sim_data)["euler_pitch"] = FDMExec->GetPropagate()->GetEuler(2) * RAD_TO_DEG;
        (*sim_data)["euler_roll"] = FDMExec->GetPropagate()->GetEuler(1) * RAD_TO_DEG;
        (*sim_data)["attitude/heading-true-rad"] = FDMExec->GetPropertyValue("attitude/heading-true-rad");
        (*sim_data)["attitude/heading-true-deg"] = FDMExec->GetPropertyValue("attitude/heading-true-rad") * RAD_TO_DEG;

        JSBSim::FGQuaternion local_quaternion = FDMExec->GetPropagate()->GetQuaternion();
        (*sim_data)["local_q_1"] = local_quaternion(1);
        (*sim_data)["local_q_2"] = local_quaternion(2);
        (*sim_data)["local_q_3"] = local_quaternion(3);
        (*sim_data)["local_q_4"] = local_quaternion(4);

        JSBSim::FGQuaternion ecef_quaternion = FDMExec->GetPropagate()->GetQuaternionECEF();
        (*sim_data)["ecef_q_1"] = ecef_quaternion(1);
        (*sim_data)["ecef_q_2"] = ecef_quaternion(2);
        (*sim_data)["ecef_q_3"] = ecef_quaternion(3);
        (*sim_data)["ecef_q_4"] = ecef_quaternion(4);

        (*sim_data)["cg_x_m"] = FDMExec->GetMassBalance()->GetXYZcg(1) * IN_TO_M;
        (*sim_data)["cg_y_m"] = FDMExec->GetMassBalance()->GetXYZcg(2) * IN_TO_M;
        (*sim_data)["cg_z_m"] = FDMExec->GetMassBalance()->GetXYZcg(3) * IN_TO_M;

        (*sim_data)["propulsion/engine/propeller-rpm"] = FDMExec->GetPropertyValue("propulsion/engine/propeller-rpm");
        (*sim_data)["propulsion/engine[1]/propeller-rpm"] = FDMExec->GetPropertyValue("propulsion/engine[1]/propeller-rpm");
        (*sim_data)["propulsion/engine[2]/propeller-rpm"] = FDMExec->GetPropertyValue("propulsion/engine[2]/propeller-rpm");
        (*sim_data)["propulsion/engine[3]/propeller-rpm"] = FDMExec->GetPropertyValue("propulsion/engine[3]/propeller-rpm");

        (*sim_data)["propulsion/engine/propeller-sense"] = FDMExec->GetPropertyValue("propulsion/engine/propeller-sense");
        (*sim_data)["propulsion/engine[1]/propeller-sense"] = FDMExec->GetPropertyValue("propulsion/engine[1]/propeller-sense");
        (*sim_data)["propulsion/engine[2]/propeller-sense"] = FDMExec->GetPropertyValue("propulsion/engine[2]/propeller-sense");
        (*sim_data)["propulsion/engine[3]/propeller-sense"] = FDMExec->GetPropertyValue("propulsion/engine[3]/propeller-sense");

        (*sim_data)["propulsion/engine/thrust-lbs"] = FDMExec->GetPropertyValue("propulsion/engine/thrust-lbs");
        (*sim_data)["propulsion/engine[1]/thrust-lbs"] = FDMExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");
        (*sim_data)["propulsion/engine[2]/thrust-lbs"] = FDMExec->GetPropertyValue("propulsion/engine[2]/thrust-lbs");
        (*sim_data)["propulsion/engine[3]/thrust-lbs"] = FDMExec->GetPropertyValue("propulsion/engine[3]/thrust-lbs");

        (*sim_data)["propulsion/engine/pitch-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine/pitch-angle-rad");
        (*sim_data)["propulsion/engine[1]/pitch-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
        (*sim_data)["propulsion/engine[2]/pitch-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[2]/pitch-angle-rad");
        (*sim_data)["propulsion/engine[3]/pitch-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[3]/pitch-angle-rad");

        (*sim_data)["propulsion/engine/yaw-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine/yaw-angle-rad");
        (*sim_data)["propulsion/engine[1]/yaw-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
        (*sim_data)["propulsion/engine[2]/yaw-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[2]/yaw-angle-rad");
        (*sim_data)["propulsion/engine[3]/yaw-angle-rad"] = FDMExec->GetPropertyValue("propulsion/engine[3]/yaw-angle-rad");
        

        /**
         * ==== FCS ====
         */
        
        FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[0]", sim_data->value<double>("fcs/throttle-cmd-norm[0]", 0));
        FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[1]", sim_data->value<double>("fcs/throttle-cmd-norm[1]", 0));
        FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[2]", sim_data->value<double>("fcs/throttle-cmd-norm[2]", 0));
        FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[3]", sim_data->value<double>("fcs/throttle-cmd-norm[3]", 0));
    
        /**
         * ==== User control ====
         */

        // FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[0]", 0);
        // FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[1]", 0);
        // FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[2]", 0);
        // FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[3]", 0);

        // auto pressed_keys = (*sim_data)["keys"];

        // if (std::find(pressed_keys.begin(), pressed_keys.end(), "Shift") != pressed_keys.end()) {
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[0]", 0.3);
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[1]", 0.3);
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[2]", 0.3);
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[3]", 0.3);
        // }

        // if (std::find(pressed_keys.begin(), pressed_keys.end(), "w") != pressed_keys.end()) {
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[0]", 0.2);
        // }
        // if (std::find(pressed_keys.begin(), pressed_keys.end(), "s") != pressed_keys.end()) {
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[1]", 0.2);
        // }

        // if (std::find(pressed_keys.begin(), pressed_keys.end(), "a") != pressed_keys.end()) {
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[2]", 0.2);
        // }
        // if (std::find(pressed_keys.begin(), pressed_keys.end(), "d") != pressed_keys.end()) {
        //     FDMExec->SetPropertyValue("fcs/throttle-cmd-norm[3]", 0.2);
        // }
    }
}