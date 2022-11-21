#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <math.h>

#include "nlohmann/json.hpp"
#include "MadgwickAHRS.h"

#include "pid.h"

#define DEG_TO_GEO_M        6378000 * M_PI / 180;

using json = nlohmann::json;

// json local_sim_data = {};

pid_state_t alt_pid;
pid_state_t yaw_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;

double time_sec;
double prev_time_sec;
double x_world_measure_m;
double y_world_measure_m;
double ax, ay, az, gx, gy, gz;
double pressure_pa;

double alt_measure;
double yaw_measure;
double yaw_rad;
double pitch_measure;
double roll_measure;

// double yaw_est;
// double pitch_est;
// double roll_est;

double phi_est, theta_est, psi_est;

double engine_0_cmd_norm;
double engine_1_cmd_norm;
double engine_2_cmd_norm;
double engine_3_cmd_norm;

/**
 * ==== FORWARD DECLARATIONS ====
 */ 
extern "C" void init(json *sim_data);
extern "C" void data_to_fcs(double *data);
extern "C" void data_from_fcs(double *data);
extern "C" void loop(void);
const double engine_mixer(const int engine_id,
                          const double throttle_cmd,
                          const double yaw_cmd,
                          const double pitch_cmd,
                          const double roll_cmd);
const double mixer_to_cmd(const double mixer_output);
const double get_x_body(const double yaw_rad, const double x_world, const double y_world);
const double get_y_body(const double yaw_rad, const double x_world, const double y_world);

/**
 * ==== END FORWARD DECLARATIONS ====
 */ 

extern "C" void init(json *sim_data) {
    printf("Hello from FCS\n");

    pid_init(alt_pid,    0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    pid_init(yaw_pid,    0.022, 0.088, 0.001325, 0.15, -0.2, 0.2);
    pid_init(x_body_pid, 3,     0,     6,        0.15, -5,   5);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 3,     0,     6,        0.15, -5,   5);  // Output is used as pitch setpoint
    pid_init(roll_pid,   0.001, 0,     0.0005,   0.15, -0.1, 0.1);
    pid_init(pitch_pid,  0.001, 0,     0.0005,   0.15, -0.1, 0.1);

    prev_time_sec = -1;

    engine_0_cmd_norm = 0.0;
    engine_1_cmd_norm = 0.0;
    engine_2_cmd_norm = 0.0;
    engine_3_cmd_norm = 0.0;
}

extern "C" void data_to_fcs(double *data) {
    time_sec          = data[0];

    x_world_measure_m = data[1] * DEG_TO_GEO_M;
    y_world_measure_m = data[2] * DEG_TO_GEO_M;

    alt_measure       = data[3];
    yaw_rad           = data[4];
    yaw_measure       = data[5];
    pitch_measure     = data[6];
    roll_measure      = data[7];

    ax                = data[8];
    ay                = data[9];
    az                = data[10];

    gx                = data[11];
    gy                = data[12];
    gz                = data[13];

    if (prev_time_sec < 0) prev_time_sec = time_sec;
}

extern "C" void data_from_fcs(double *data) {
    data[0] = engine_0_cmd_norm;
    data[1] = engine_1_cmd_norm;
    data[2] = engine_2_cmd_norm;
    data[3] = engine_3_cmd_norm;

    data[4] = phi_est;
    data[5] = theta_est;
    data[6] = psi_est;
}

extern "C" void loop(void) {
    const double deltaT = time_sec - prev_time_sec;
    prev_time_sec = time_sec;

    /**
     * ==== Altitude ====
     */
    double alt_sp = 0.0;
    if (time_sec > 3) alt_sp = 1.0;

    /**
     * ==== Yaw ====
     */
    double yaw_sp = 0;
    if (alt_measure > 0.5) yaw_sp = 60;
    if (yaw_sp - yaw_measure >  180) yaw_measure += 360;
    if (yaw_sp - yaw_measure < -180) yaw_measure -= 360;
    
    /**
     * Position: world_x -> east -> longitude
     *           world_y -> north -> latitude
    */
    double x_world_sp_m = 0;
    double y_world_sp_m = 0;

    if (time_sec > 7) {
        x_world_sp_m = 1;
        y_world_sp_m = 2.5;
    }

    const double x_body_measure_m = get_x_body(-yaw_rad, x_world_measure_m, y_world_measure_m);
    const double y_body_measure_m = get_y_body(-yaw_rad, x_world_measure_m, y_world_measure_m);
    const double x_body_sp_m = get_x_body(-yaw_rad, x_world_sp_m, y_world_sp_m);
    const double y_body_sp_m = get_y_body(-yaw_rad, x_world_sp_m, y_world_sp_m);
    
    /**
     * ==== PIDs ====
     */
    const double alt_pid_out = 0.255 + pid_update(alt_pid, alt_sp, alt_measure, deltaT);
    const double yaw_pid_out = pid_update(yaw_pid, yaw_sp, yaw_measure, deltaT);

    const double x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
    const double y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);

    const double roll_pid_out  = pid_update(roll_pid,   x_body_pid_out, roll_measure,  deltaT);
    const double pitch_pid_out = pid_update(pitch_pid, -y_body_pid_out, pitch_measure, deltaT);

    /**
     * ==== Output ====
     */
    engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));

    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, deltaT);

    phi_est   = atan2(q2 * q3 + q0 * q1, 1 / 2 - (q1 * q1 + q2 * q2));
    theta_est = asin(-2 * (q1 * q3 - q0 * q2));
    psi_est   = atan2(q1 * q2 + q0 * q3, 1 / 2 - (q2 * q2 + q3 * q3));

    phi_est   = phi_est   / M_PI * 180.0;
    theta_est = theta_est / M_PI * 180.0;
    psi_est   = psi_est   / M_PI * 180.0;

    // printf("alt_measure: %f\n", alt_measure);
    // printf("ax: %f\n", ax);
}

/**
 * @param engine_id Engine id from 1 to 4
 */
const double engine_mixer(const int engine_id,
                    const double throttle_cmd,
                    const double yaw_cmd,
                    const double pitch_cmd,
                    const double roll_cmd) {
    if (engine_id == 0) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd;
    }

    if (engine_id == 1) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd;
    }

    if (engine_id == 2) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd;
    }

    if (engine_id == 3) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd;
    }

    return 0;
}

const double mixer_to_cmd(const double mixer_output) {
    return std::max(0.0, std::min(1.0, mixer_output));
}

const double get_x_body(const double yaw_rad, const double x_world, const double y_world) {
    return x_world * cos(yaw_rad) + y_world * sin(yaw_rad);
}

const double get_y_body(const double yaw_rad, const double x_world, const double y_world) {
    return x_world * -sin(yaw_rad) + y_world * cos(yaw_rad);
}
