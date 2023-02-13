#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#include <stdint.h>
#include <queue>
#include "defines.h"
#include "pid.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "j_packets.h"

#ifdef MCU
#include "bmi160.h"
#include "hp203b.h"
#endif

inline char cmd_string[128] = { 0 };

inline uint8_t heading_mode  = HEADING_MODE_RATE;
inline uint8_t lateral_mode  = LATERAL_MODE_ANGLE;
inline uint8_t vertical_mode = VERTICAL_MODE_DIRECT;

inline Adafruit_Madgwick sensor_fusion;

inline pid_state_t alt_sp_pid;   /* VERTICAL_MODE_SP */
inline pid_state_t alt_rate_pid; /* VERTICAL_MODE_RATE */
inline pid_state_t yaw_sp_pid;
inline pid_state_t yaw_rate_pid;
inline pid_state_t x_body_pid;
inline pid_state_t y_body_pid;
inline pid_state_t roll_pid;
inline pid_state_t pitch_pid;
inline pid_state_t roll_rate_pid;
inline pid_state_t pitch_rate_pid;

inline float time_s = 0.0f;
inline float start_time_s = 0.0f;
inline float prev_time_s = -1.0f;
inline float x_world_measure_m = 0.0f;  // world_x -> east -> longitude
inline float y_world_measure_m = 0.0f;  // world_y -> north -> latitude
inline float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
inline float gx_rad = 0.0f, gy_rad = 0.0f, gz_rad = 0.0f;
inline float pressure_pa = 0.0f;
inline float alt_measurement_m = 0.0f;
inline float alt_measurement_m_prev = -1.0f;
inline float temp_c = 0.0f;
inline float real_yaw_deg = 0.0f;

inline float gx_mean_rad = 0.0f, gy_mean_rad = 0.0f, gz_mean_rad = 0.0f;
inline float acc_norm_mean = 1.0f; 
inline float pressure_pa_mean = 0.0f;
inline int calibration_samples = 0;

inline float alt_est_m = -1.0f;
inline float alt_est_m_prev = -1.0f;
inline float initial_alt_m = -1.0f;
inline float alt_rate_est_mps = 0.0f;
inline float yaw_est_deg = 0.0f;
inline float yaw_est_deg_prev = 0.0f;
inline float pitch_est_deg = 0.0f;
inline float pitch_est_deg_prev = 0.0f;
inline float roll_est_deg = 0.0f;
inline float roll_est_deg_prev = 0.0f;
inline float yaw_rate_dps = 0.0f;
inline float roll_rate_dps = 0.0f;
inline float pitch_rate_dps = 0.0f;

inline int current_tunning = -1;

inline float alt_sp = 0.0f;
inline float yaw_sp_deg = 0.0f;
inline float x_world_sp_m = 0.0f;
inline float y_world_sp_m = 0.0f;

inline float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
inline float lin_acc_x_g = 0.0f, lin_acc_y_g = 0.0f, lin_acc_z_g = 0.0f;

// ? FCS channels indexed from 0, RC transmitter indexed from 1
// ? Channels have range of (-1,1) and default value of 0
// 0-roll, 1-pitch, 2-throttle, 3-yaw, 4-arm, 5-mode, 6-mode, 7-mode
inline float ctrl_channels_norm[NUM_CTRL_CHANNELS] = { 0 };

inline float yaw_channel = 0.0f;
inline float pitch_channel = 0.0f;
inline float roll_channel = 0.0f;
inline float throttle_channel = 0.0f;
inline float vert_mode_channel = 0.0f;

inline float throttle_cmd = 0.0f;
inline float yaw_cmd = 0.0f;
inline float roll_cmd = 0.0f;
inline float pitch_cmd = 0.0f;

inline float engine_0_cmd_norm = 0.0f;
inline float engine_1_cmd_norm = 0.0f;
inline float engine_2_cmd_norm = 0.0f;
inline float engine_3_cmd_norm = 0.0f;

inline bool is_airborne = false;
inline bool is_armed = false;
inline bool force_arm = false;
inline bool can_do_logging = false;
inline float arm_channel_norm_prev = 1.0f;
inline float takeoff_event_start_s = -1.0f;
inline float landing_event_start_s = -1.0f;

inline float cpu_usage = 0.0f;

inline float from_jsbsim[20] = { 0 };

#ifdef SITL
struct to_jsbsim_t {
    uint8_t len;
    uint8_t data[J_PACKET_SIZE];
};
inline std::queue<to_jsbsim_t> to_jsbsim_queue;
#endif // SITL

#ifdef MCU
inline uint32_t last_loop_time_ms = 0;

inline int8_t current_channel = -1;
inline int32_t last_captured_value = -1;
inline int32_t ppm_us[NUM_CTRL_CHANNELS] = { 0 };

inline uint32_t flash_page_addr = 0;
inline uint32_t flash_page_offset = 0;

inline bmi160_t hbmi160;
inline hp203b_t hhp203b;
#endif  // MCU


#endif // GLOBALVARIABLES_H