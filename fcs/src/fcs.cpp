#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <math.h>

#include "nlohmann/json.hpp"
#include "MadgwickAHRS.h"

#include "pid.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "Adafruit_AHRS_NXPFusion.h"

#define DEG_TO_GEO_M        6378000 * M_PI / 180
#define DEG_TO_RAD          M_PI / 180
#define RAD_TO_DEG          180 / M_PI

const double rest_time = 5.0;

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
double ax, ay, az, gx, gy, gz, mx, my, mz;
double ch_1, ch_2, ch_3, ch_4;
double pressure_pa;

double alt_real;
double yaw_real;
double pitch_real;
double roll_real;

double yaw_est;
double pitch_est;
double roll_est;

// Adafruit_NXPSensorFusion sensor_fusion;
Adafruit_Madgwick sensor_fusion;

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
    pid_init(yaw_pid,    0.011, 0.02, 0.0013,   0.15, -0.2, 0.2);
    pid_init(x_body_pid, 4,     0.1,   6,        0.15, -10,  10);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 4,     0.1,   6,        0.15, -10,  10);  // Output is used as pitch setpoint
    pid_init(roll_pid,   0.002, 0,     0.0005,   0.15, -0.1, 0.1);
    pid_init(pitch_pid,  0.002, 0,     0.0005,   0.15, -0.1, 0.1);

    prev_time_sec = -1;

    engine_0_cmd_norm = 0.0;
    engine_1_cmd_norm = 0.0;
    engine_2_cmd_norm = 0.0;
    engine_3_cmd_norm = 0.0;

    sensor_fusion.begin(250.0);
}

extern "C" void data_to_fcs(double *data) {
    time_sec            = data[0];                  // simulation/sim-time-sec

    x_world_measure_m   = data[1] * DEG_TO_GEO_M;   // ext/longitude-deg
    y_world_measure_m   = data[2] * DEG_TO_GEO_M;   // ext/latitude-deg

    alt_real            = data[3];                  // ext/altitude-m
    yaw_real            = data[4];                  // attitude/psi-deg
    pitch_real          = data[5];                  // attitude/theta-deg
    roll_real           = data[6];                  // attitude/phi-deg

    ax                  = data[7];                  // sensor/imu/accelX-g
    ay                  = data[8];                  // sensor/imu/accelY-g
    az                  = data[9];                  // sensor/imu/accelZ-g

    gx                  = data[10];                 // sensor/imu/gyroX-rps
    gy                  = data[11];                 // sensor/imu/gyroY-rps
    gz                  = data[12];                 // sensor/imu/gyroZ-rps

    mx                  = data[13];                 // sensor/imu/magX-uT
    my                  = data[14];                 // sensor/imu/magY-uT
    mz                  = data[15];                 // sensor/imu/magZ-uT

    ch_1                = data[18];                 // user-control/channel-1
    ch_2                = data[19];                 // user-control/channel-2
    ch_3                = data[20];                 // user-control/channel-3
    ch_4                = data[21];                 // user-control/channel-4

    if (prev_time_sec < 0) prev_time_sec = time_sec;
}

extern "C" void data_from_fcs(double *data) {
    data[0] = engine_0_cmd_norm;
    data[1] = engine_1_cmd_norm;
    data[2] = engine_2_cmd_norm;
    data[3] = engine_3_cmd_norm;

    data[4] = yaw_est;
    data[5] = pitch_est;
    data[6] = roll_est;
}

extern "C" void loop(void) {
    const double deltaT = time_sec - prev_time_sec;
    prev_time_sec = time_sec;

    /**
     * Estimate attitude
     */
    sensor_fusion.update(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, ax, ay, az, 0, 0, 0);
    roll_est    = sensor_fusion.getRoll();
    pitch_est   = sensor_fusion.getPitch();
    yaw_est     = sensor_fusion.getYaw() - 180.0;

    bool use_est = true;
    double yaw_used     = use_est ? yaw_est     : yaw_real;
    double pitch_used   = use_est ? pitch_est   : pitch_real;
    double roll_used    = use_est ? roll_est    : roll_real;

    /**
     * ==== Altitude ====
     */
    double alt_sp = 0.0;
    if (time_sec > rest_time) alt_sp = 1.0;

    /**
     * ==== Yaw ====
     */
    double yaw_sp = 2;
    if (time_sec > rest_time + 5) yaw_sp = 20;
    if (yaw_sp - yaw_used >  180) yaw_used += 360;
    if (yaw_sp - yaw_used < -180) yaw_used -= 360;
    
    /**
     * Position: world_x -> east -> longitude
     *           world_y -> north -> latitude
    */
    double x_world_sp_m = 0;
    double y_world_sp_m = 0;

    if (time_sec > rest_time + 10) {
        x_world_sp_m = 1;
        y_world_sp_m = 2.5;
    }

    const double x_body_measure_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
    const double y_body_measure_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
    const double x_body_sp_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);
    const double y_body_sp_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);
    
    /**
     * ==== PIDs ====
     */
    const double alt_pid_out = 0.255 + pid_update(alt_pid, alt_sp, alt_real, deltaT);
    const double yaw_pid_out = pid_update(yaw_pid, yaw_sp, yaw_used, deltaT);

    const double x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
    const double y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);

    // const double roll_pid_out  = pid_update(roll_pid,   x_body_pid_out, roll_used,  deltaT);
    // const double pitch_pid_out = pid_update(pitch_pid, -y_body_pid_out, pitch_used, deltaT);
    const double roll_pid_out  = pid_update(roll_pid,   (ch_4 - 0.5) * 20, roll_used,  deltaT);
    const double pitch_pid_out = pid_update(pitch_pid, -(ch_2 - 0.5) * 20, pitch_used, deltaT);

    /**
     * ==== Output ====
     */
    engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));

    // engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    // engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    // engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    // engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));

    // engine_0_cmd_norm = ch_3 - 0.091 + (ch_1 - 0.5);
    // engine_1_cmd_norm = ch_3 - 0.091 + (ch_1 - 0.5);
    // engine_2_cmd_norm = ch_3 - 0.091 - (ch_1 - 0.5);
    // engine_3_cmd_norm = ch_3 - 0.091 - (ch_1 - 0.5);

    /**
     * ==== Attitude estimation ====
     */

    // MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, deltaT);

    // roll_est   = atan2(q2 * q3 + q0 * q1, 1 / 2 - (q1 * q1 + q2 * q2));
    // pitch_est = asin(-2 * (q1 * q3 - q0 * q2));
    // yaw_est   = atan2(q1 * q2 + q0 * q3, 1 / 2 - (q2 * q2 + q3 * q3));

    // roll_est   = roll_est   / M_PI * 180.0;
    // pitch_est = pitch_est / M_PI * 180.0;
    // yaw_est   = yaw_est   / M_PI * 180.0;
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
    return x_world * cos(-yaw_rad) + y_world * sin(-yaw_rad);
}

const double get_y_body(const double yaw_rad, const double x_world, const double y_world) {
    return x_world * -sin(-yaw_rad) + y_world * cos(-yaw_rad);
}
