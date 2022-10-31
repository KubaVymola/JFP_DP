#include "jsbsim_interface.h"

#include <sys/time.h>

#include "constants.h"
#include "FGFDMExec.h"
#include "models/FGInertial.h"
#include "models/FGPropagate.h"
#include "models/FGMassBalance.h"
#include "simgear/misc/sg_path.hxx"

JSBSim::FGFDMExec* FDMExec;

bool override_sim_rate = false;
bool realtime = true;
bool was_paused = false;
bool result = false;
bool play_nice = false;

long sleep_nseconds = 0;

double simulation_rate = 1./120.;
double override_sim_rate_value = 0.0;
double sleep_period=0.01;
double frame_duration;
double new_five_second_value = 0.0;
double actual_elapsed_time = 0;
double sim_lag_time = 0;
double cycle_duration = 0.0;
double initial_seconds = 0;
double current_seconds = 0.0;
double paused_seconds = 0.0;
double end_time = 1e99;

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

void jsbsim_init(const std::string& root_dir,
                 const std::string& script_path,
                 bool* continue_running,
                 sim_data_t* sim_data,
                 std::mutex& sim_data_lock) {
    FDMExec = new JSBSim::FGFDMExec();

    printf("Script path: %s\n", script_path.c_str());
    printf("Root dir: %s\n", root_dir.c_str());
    
    FDMExec->SetRootDir(SGPath::fromLocal8Bit(root_dir.c_str()));
    FDMExec->SetAircraftPath(SGPath("aircraft"));
    FDMExec->SetEnginePath(SGPath("engine"));
    FDMExec->SetSystemsPath(SGPath("systems"));
    FDMExec->SetOutputPath(SGPath("."));

    if (simulation_rate < 1.0 )
        FDMExec->Setdt(simulation_rate);
    else
        FDMExec->Setdt(1.0/simulation_rate);

    if (override_sim_rate) override_sim_rate_value = FDMExec->GetDeltaT();


    /**
     * Load script that loads all other files
     */
    result = FDMExec->LoadScript(SGPath::fromLocal8Bit(script_path.c_str()),
                                 override_sim_rate_value);

    if (!result) {
      printf("Script %s was not successfully loaded\n", script_path.c_str());
      delete FDMExec;
      exit(-1);
    }

    /**
     * TODO Load output directives file[s], if given
     */
    // for (unsigned int i=0; i<LogDirectiveName.size(); i++) {
    //     if (!LogDirectiveName[i].isNull()) {
    //         if (!FDMExec->SetOutputDirectives(LogDirectiveName[i])) {
    //             cout << "Output directives not properly set in file " << LogDirectiveName[i] << endl;
    //             delete FDMExec;
    //             exit(-1);
    //         }
    //     }
    // }

    FDMExec->RunIC();

    FDMExec->PrintSimulationConfiguration();

    FDMExec->GetPropagate()->DumpState();

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
    if (realtime) {
        sleep_nseconds = (long)(frame_duration*1e9);
    } else {
        sleep_nseconds = (sleep_period)*1e9; // 0.01 seconds
    }

    tzset();
    current_seconds = initial_seconds = getcurrentseconds();

    while (result && *continue_running && FDMExec->GetSimTime() <= end_time) {
        // Check if increment then hold is on and take appropriate actions if it is
        // Iterate is not supported in realtime - only in batch and playnice modes
        FDMExec->CheckIncrementalHold();

        // if running realtime, throttle the execution, else just run flat-out fast
        // unless "playing nice", in which case sleep for a while (0.01 seconds) each frame.
        // If suspended, then don't increment cumulative realtime "stopwatch".

        if ( ! FDMExec->Holding()) {
            if ( ! realtime ) {
                /**
                 * Batch mode
                 */
                result = FDMExec->Run();
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
                for (int i=0; i<(int)(sim_lag_time/frame_duration); i++) {  // catch up sim time to actual elapsed time.
                    result = FDMExec->Run();
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
        }

        {
            std::lock_guard<std::mutex> guard(sim_data_lock);
            sim_data->altitude_m = FDMExec->GetPropagate()->GetAltitudeASL() * FT_TO_M;
            sim_data->latitude_deg = FDMExec->GetPropagate()->GetLatitudeDeg();
            sim_data->latitude_rad = FDMExec->GetPropagate()->GetLatitude();
            sim_data->longitude_deg = FDMExec->GetPropagate()->GetLongitudeDeg();
            sim_data->longitude_rad = FDMExec->GetPropagate()->GetLongitude();

            sim_data->euler_yaw = FDMExec->GetPropagate()->GetEuler(3) * RAD_TO_DEG;
            sim_data->euler_pitch = FDMExec->GetPropagate()->GetEuler(2) * RAD_TO_DEG;
            sim_data->euler_roll = FDMExec->GetPropagate()->GetEuler(1) * RAD_TO_DEG;

            JSBSim::FGQuaternion local_quaternion = FDMExec->GetPropagate()->GetQuaternion();
            sim_data->local_q_1 = local_quaternion(1);
            sim_data->local_q_2 = local_quaternion(2);
            sim_data->local_q_3 = local_quaternion(3);
            sim_data->local_q_4 = local_quaternion(4);

            JSBSim::FGQuaternion ecef_quaternion = FDMExec->GetPropagate()->GetQuaternionECEF();
            sim_data->ecef_q_1 = ecef_quaternion(1);
            sim_data->ecef_q_2 = ecef_quaternion(2);
            sim_data->ecef_q_3 = ecef_quaternion(3);
            sim_data->ecef_q_4 = ecef_quaternion(4);

            sim_data->cg_x_m = FDMExec->GetMassBalance()->GetXYZcg(1) * IN_TO_M;
            sim_data->cg_y_m = FDMExec->GetMassBalance()->GetXYZcg(2) * IN_TO_M;
            sim_data->cg_z_m = FDMExec->GetMassBalance()->GetXYZcg(3) * IN_TO_M;
        }
    }

    time(&tod);
    localtime_r(&tod, &local);
    strftime(s, 99, "%A %B %d %Y %X", &local);
    cout << "End: " << s << " (HH:MM:SS)" << endl;

    delete FDMExec;
}

void jsbsim_run() {
}