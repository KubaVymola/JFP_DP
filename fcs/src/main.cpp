// either INDEP, HITL, or SITL will be defined
// either MCU or CPU will be defined

// #define MCU
// #define HITL
// #define SITL

/**
 * ==== INCLUDES ====
 */
#include <stdint.h>
#include <algorithm>

#include "global_variables.h"
#include "defines.h"

#include "utils.h"
#include "complementary_filter.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "low_pass_filter.h"
#include "pid.h"
#include "j_packets.h"
#include "quaternion.h"
#include "flight_events.h"
#include "j_packet_send_callback.h"
#include "j_packet_recv_callback.h"
#include "mixer.h"
#include "flash_interface.h"
#include "user_commands.h"
#include "demo_sequence.h"
#include "pressure.h"

#ifdef SITL
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <map>
#include <string>
#include <queue>

#include "sitl_tunning.h"
#include "sitl_data_interface.h"
#endif  // SITL

#ifdef MCU
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "w25qxx.h"
#include "bmi160.h"
#include "hp203b.h"
#endif  // MCU

/**
 * ==== END INCLUDES ====
 */


/**
 * TODO better altitude measurement (higher oversampling and divide the resulting vertical rate over the duration that it took to produce the masurement - to avoid spikes)
 * TODO better loop timing design - merge the two separate timers in HITL
 */

/**
 * ==== FUNCTION DECLARATIONS ====
 */

extern "C" void init();
extern "C" void from_jsbsim_to_glob_state();
extern "C" void control_loop(void);
extern "C" void after_loop(void);

#ifdef MCU
extern "C" void SystemClock_Config(void);
#endif  // MCU

/**
 * ==== END FUNCTION DECLARATIONS ====
 */

extern "C" void init() {

    send_jpacket_info(0x03, "Hello from FCS", 64);

    // ! In HITL/INDEP pid config from flash is used if present (which it most likely is)
    // ! In SITL this config is taken

    pid_init(alt_sp_pid,   0.0469, 0.0019, 0.0727, 0.15, -0.5, 0.5);
    pid_init(alt_rate_pid, 0.1299, 0.031,  0.0332, 0.15, -0.5, 0.5);

    pid_init(yaw_sp_pid,   0.00098,  0.00001,  0.00059, 0.15, -0.2, 0.2);
    pid_init(yaw_rate_pid, 0.000204, 0.000002, 0.000037, 0.15, -0.2, 0.2);

    pid_init(x_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as pitch setpoint

    pid_init(roll_pid,  0.00009, 0.00003, 0.00005, 0.15, -0.2, 0.2);
    pid_init(pitch_pid, 0.00009, 0.00003, 0.00005, 0.15, -0.2, 0.2);

    pid_init(roll_rate_pid,  0.002, 0.0, 0.0012, 0.15, -0.2, 0.2);
    pid_init(pitch_rate_pid, 0.002, 0.0, 0.0012, 0.15, -0.2, 0.2);

    load_pid_flash(false);
    
    low_pass_filter_init(alt_est_lpf, 0.1211f);
    // low_pass_filter_init(throttle_lpf, 3.0f);

    low_pass_filter_init(yaw_rate_lpf, 5.0f);
    
    low_pass_filter_init(acc_x_lpf, 10.0f);
    low_pass_filter_init(acc_y_lpf, 10.0f);
    low_pass_filter_init(acc_z_lpf, 10.0f);


    /**
     * ==== Good tune ====
     */

    // pid_init(alt_sp_pid,   0.0469, 0.0019, 0.0727, 0.15, -0.5, 0.5);
    // pid_init(alt_rate_pid, 0.1299, 0.031,  0.0332, 0.15, -0.5, 0.5);

    // pid_init(yaw_sp_pid,   0.00197,  0.00001,  0.00118,  0.15, -0.2, 0.2);
    // pid_init(yaw_rate_pid, 0.000402, 0.000002, 0.000074, 0.15, -0.2, 0.2);

    // pid_init(x_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as roll  setpoint
    // pid_init(y_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as pitch setpoint

    // pid_init(roll_pid,  0.00017, 0.00002, 0.000099, 0.15, -0.2, 0.2);
    // pid_init(pitch_pid, 0.00017, 0.00002, 0.000099, 0.15, -0.2, 0.2);

    // pid_init(roll_rate_pid,  0.002, 0, 0.0012, 0.15, -0.2, 0.2);
    // pid_init(pitch_rate_pid, 0.002, 0, 0.0012, 0.15, -0.2, 0.2);

    /**
     * ==== END Good tune ====
     */
    
    disable_pid_integrators();

    sensor_fusion.begin(LOOP_FREQUENCY);

}

extern "C" void from_jsbsim_to_glob_state() {
    time_s              = from_jsbsim[0];  // simulation/sim-time-sec

    x_world_measure_m   = from_jsbsim[1] * DEG_TO_GEO_M;  // ext/longitude-deg
    y_world_measure_m   = from_jsbsim[2] * DEG_TO_GEO_M;  // ext/latitude-deg

    /**
     * Accelerometer adjusted from JSBSim local frame to expected sensor frame (BMI160)
     */
    ax_g                = -from_jsbsim[3];  // sensor/imu/accelX-g
    ay_g                =  from_jsbsim[4];  // sensor/imu/accelY-g
    az_g                = -from_jsbsim[5];  // sensor/imu/accelZ-g

    gx_rad              = -from_jsbsim[6];  // sensor/imu/gyroX-rps
    gy_rad              =  from_jsbsim[7];  // sensor/imu/gyroY-rps
    gz_rad              = -from_jsbsim[8];  // sensor/imu/gyroZ-rps

    pressure_pa         = from_jsbsim[9];   // sensor/baro/presStatic-Pa
    temp_c              = from_jsbsim[10];  // sensor/baro/temp-C

    real_yaw_deg        = from_jsbsim[11];  // attitude/psi-deg
 

// #ifdef SITL

    ctrl_channels_norm[0] = from_jsbsim[12];  // user-control/channel-1
    ctrl_channels_norm[1] = from_jsbsim[13];  // user-control/channel-2
    ctrl_channels_norm[2] = from_jsbsim[14];  // user-control/channel-3
    ctrl_channels_norm[3] = from_jsbsim[15];  // user-control/channel-4
    ctrl_channels_norm[4] = from_jsbsim[16];  // user-control/channel-4
    ctrl_channels_norm[5] = from_jsbsim[17];  // user-control/channel-4
    ctrl_channels_norm[6] = from_jsbsim[18];  // user-control/channel-4
    ctrl_channels_norm[7] = from_jsbsim[19];  // user-control/channel-4

// #endif

    if (prev_time_s < 0) prev_time_s = time_s;
}

extern "C" void control_loop(void) {
    const float delta_t_s = time_s - prev_time_s;
    prev_time_s = time_s;

    if (delta_t_s <= 0.0f) return;

    /**
     * Calibration (First 2 sec of execution)
     */
    if (calibration_samples >= 0 && calibration_samples < LOOP_FREQUENCY * 2.0f) {
        if (pressure_pa == 0.0f) return;
        
        float curr_acc_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        acc_norm_mean = (acc_norm_mean * calibration_samples + curr_acc_norm) / (calibration_samples + 1);

        gx_mean_rad = (gx_mean_rad * calibration_samples + gx_rad) / (calibration_samples + 1);
        gy_mean_rad = (gy_mean_rad * calibration_samples + gy_rad) / (calibration_samples + 1);
        gz_mean_rad = (gz_mean_rad * calibration_samples + gz_rad) / (calibration_samples + 1);

        pressure_pa_mean = (pressure_pa_mean * calibration_samples + pressure_pa) / (calibration_samples + 1);

        calibration_samples++;

        return;
    }


    if (calibration_samples > 0) {
        send_jpacket_info(0x03, "acc_norm_mean %f", 64, acc_norm_mean);
        send_jpacket_info(0x03, "gx_mean %f°", 64, gx_mean_rad * RAD_TO_DEG);
        send_jpacket_info(0x03, "gy_mean %f°", 64, gy_mean_rad * RAD_TO_DEG);
        send_jpacket_info(0x03, "gz_mean %f°", 64, gz_mean_rad * RAD_TO_DEG);
        send_jpacket_info(0x03, "mean pressure %f Pa", 64, pressure_pa_mean);
        send_jpacket_info(0x03, "READY...", 64);
        
        calibration_samples = -1;
    }

    if (acc_norm_mean <= 0.1f) acc_norm_mean = 1.0f;

    ax_g = ax_g / acc_norm_mean;
    ay_g = ay_g / acc_norm_mean;
    az_g = az_g / acc_norm_mean;
    gx_rad = gx_rad - gx_mean_rad;
    gy_rad = gy_rad - gy_mean_rad;
    gz_rad = gz_rad - gz_mean_rad;

    ax_g = low_pass_filter_update(acc_x_lpf, ax_g, delta_t_s);
    ay_g = low_pass_filter_update(acc_y_lpf, ay_g, delta_t_s);
    az_g = low_pass_filter_update(acc_z_lpf, az_g, delta_t_s);

    sensor_fusion.updateIMU(-gx_rad * RAD_TO_DEG, gy_rad * RAD_TO_DEG, -gz_rad * RAD_TO_DEG, ax_g, -ay_g, az_g);
    sensor_fusion.getQuaternion(&qw, &qx, &qy, &qz);
    roll_est_deg = sensor_fusion.getRoll();
    pitch_est_deg = sensor_fusion.getPitch();
    yaw_est_deg = sensor_fusion.getYaw() - 180.0;

    Quaterion_t acc =   { 0,   ax_g, -ay_g, az_g };
    Quaterion_t q =     { qw,  qx,    qy,   qz   };
    Quaterion_t q_inv = { qw, -qx,   -qy,  -qz   };
    Quaterion_t acc_rot = multiplyQuat(multiplyQuat(q, acc), q_inv);

    lin_acc_x_g = acc_rot.x;
    lin_acc_y_g = acc_rot.y;
    lin_acc_z_g = acc_rot.z - 1.0f;

    /**
     * ==== Altitude ====
     */
    
    alt_measurement_m = get_alt_asl_from_pressure(pressure_pa);

    if (alt_est_m      == NO_ALTITUDE) alt_est_m      = get_alt_asl_from_pressure(pressure_pa_mean);
    if (alt_est_m_prev == NO_ALTITUDE) alt_est_m_prev = get_alt_asl_from_pressure(pressure_pa_mean);
    if (initial_alt_m  == NO_ALTITUDE) initial_alt_m  = get_alt_asl_from_pressure(pressure_pa_mean);
    
    alt_est_m = low_pass_filter_update(alt_est_lpf, alt_measurement_m, delta_t_s);
    alt_rate_est_mps = complementary_filter_update(0.01f,
                                                   (alt_est_m - alt_est_m_prev) / delta_t_s,
                                                   alt_rate_est_mps + lin_acc_z_g * G_TO_MPS * delta_t_s);
    
    // alt_rate_est_mps = low_pass_filter_update(alt_rate_est_lpf, alt_rate_est_mps, delta_t_s);

    alt_est_m_prev = alt_est_m;


    /**
     * ==== END Altitude ====
     */

#ifdef DEMO_SEQ
    demo_sequence();
#endif

    /**
     * ==== Yaw ====
     */
    #ifdef HITL
    if (lateral_mode == LATERAL_MODE_FIXED_POS) yaw_est_deg = real_yaw_deg;
    if (yaw_est_deg > 180.0f) yaw_est_deg = -360.0f + yaw_est_deg;
    #endif

    #ifdef SITL
    if (lateral_mode == LATERAL_MODE_FIXED_POS) yaw_est_deg = real_yaw_deg;
    if (yaw_est_deg > 180.0f) yaw_est_deg = -360.0f + yaw_est_deg;
    #endif


    float yaw_sp_diff_deg = yaw_sp_deg - yaw_est_deg;
    while (yaw_sp_diff_deg > 180) yaw_sp_diff_deg -= 360;
    while (yaw_sp_diff_deg < -180) yaw_sp_diff_deg += 360;

    // while (yaw_sp_deg - yaw_est_deg > 180) yaw_sp_deg -= 360;
    // while (yaw_sp_deg - yaw_est_deg < -180) yaw_sp_deg += 360;

    float yaw_diff_deg = yaw_est_deg - yaw_est_deg_prev;
    if (yaw_diff_deg < -180) yaw_diff_deg += 360.0f;
    if (yaw_diff_deg > 180) yaw_diff_deg -= 360.0f;

    yaw_rate_dps = yaw_diff_deg / delta_t_s;
    yaw_rate_dps = low_pass_filter_update(yaw_rate_lpf, yaw_rate_dps, delta_t_s);
    roll_rate_dps = (roll_est_deg - roll_est_deg_prev) / delta_t_s;
    pitch_rate_dps = (pitch_est_deg - pitch_est_deg_prev) / delta_t_s;

    yaw_est_deg_prev = yaw_est_deg;
    roll_est_deg_prev = roll_est_deg;
    pitch_est_deg_prev = pitch_est_deg;

    /**
     * ==== Event detection ====
     */
    if (detect_landing()) {
        is_airborne = false;
        disable_pid_integrators();
    }
    if (detect_takeoff()) {
        is_airborne = true;
        enable_pid_integrators();
    }
    if (detect_arm()) {
        is_armed = true;
        configure_logging();
        // enable_pid_integrators();
    }
    if (detect_disarm()) {
        is_armed = false;
        disable_pid_integrators();
    }

    arm_channel_norm_prev = ctrl_channels_norm[ARM_CHANNEL];

    throttle_channel = ctrl_channels_norm[THROTTLE_CHANNEL];
    yaw_channel = ctrl_channels_norm[YAW_CHANNEL];
    pitch_channel = ctrl_channels_norm[PITCH_CHANNEL];
    roll_channel = ctrl_channels_norm[ROLL_CHANNEL];
    vert_mode_channel = ctrl_channels_norm[VERT_MODE_CHANNEL];


    if (current_tunning < 0) {
        if (vert_mode_channel < -0.5f) vertical_mode = VERTICAL_MODE_RATE;
        if (vert_mode_channel >  0.5f) vertical_mode = VERTICAL_MODE_DIRECT;
    }
    

#ifdef SITL
    tunning_config();
#endif


    /**
     * ==== PIDs ===================================================================================
     */

    throttle_cmd = 0.0;
    yaw_cmd = 0.0;
    roll_cmd = 0.0;
    pitch_cmd = 0.0;

    /**
     * ==== Throttle command ====
     */
    if (vertical_mode == VERTICAL_MODE_SP) {
        throttle_cmd = ZERO_RATE_THROTTLE + pid_update(alt_sp_pid, alt_sp, alt_est_m, delta_t_s);

    } else if (vertical_mode == VERTICAL_MODE_RATE) {
        float target_alt_rate = 0.0f;

        if (throttle_channel < -VERTICAL_RATE_THRUST_POS_DEADBAND) {
            target_alt_rate = (throttle_channel + VERTICAL_RATE_THRUST_POS_DEADBAND) * MAX_VERICAL_RATE / (1.0f - VERTICAL_RATE_THRUST_POS_DEADBAND);
        }

        if (throttle_channel > VERTICAL_RATE_THRUST_POS_DEADBAND) {
            target_alt_rate = (throttle_channel - VERTICAL_RATE_THRUST_POS_DEADBAND) * MAX_VERICAL_RATE / (1.0f - VERTICAL_RATE_THRUST_POS_DEADBAND);
        }

        throttle_cmd = ZERO_RATE_THROTTLE + pid_update(alt_rate_pid, target_alt_rate, alt_rate_est_mps, delta_t_s);
        // throttle_cmd = low_pass_filter_update(throttle_lpf, throttle_cmd, delta_t_s);

        /**
         * Idle thrust when on the ground
         */
        if (!is_airborne && throttle_channel < VERTICAL_RATE_TAKEOFF_THRUST) {
            throttle_cmd = 0.0f;
        }

    } else if (vertical_mode == VERTICAL_MODE_DIRECT) {
        throttle_cmd = (throttle_channel + 1.0f) / 2.0f;

    }


    /**
     * ==== END Throttle command ====
     */

    /**
     * ==== Yaw command ====
     */
    if (heading_mode == HEADING_MODE_SP) {
        yaw_cmd = pid_update(yaw_sp_pid, yaw_sp_diff_deg, 0.0f, delta_t_s);

    } else if (heading_mode == HEADING_MODE_RATE) {
        yaw_cmd = pid_update(yaw_rate_pid, yaw_channel * MAX_YAW_RATE, yaw_rate_dps, delta_t_s);

    } else if (heading_mode == HEADING_MODE_DYNAMIC) {
        float target_yaw_rate = 0.0f;

        if (yaw_channel > HEADING_DYNAMIC_DEADBAND) target_yaw_rate = (yaw_channel - HEADING_DYNAMIC_DEADBAND) * MAX_YAW_RATE / (1.0f - HEADING_DYNAMIC_DEADBAND);
        if (yaw_channel < -HEADING_DYNAMIC_DEADBAND) target_yaw_rate = (yaw_channel + HEADING_DYNAMIC_DEADBAND) * MAX_YAW_RATE / (1.0f - HEADING_DYNAMIC_DEADBAND);

        float yaw_sp_pid_out = pid_update(yaw_sp_pid, yaw_sp_diff_deg, 0.0f, delta_t_s);
        float yaw_rate_pid_out = pid_update(yaw_rate_pid, target_yaw_rate, yaw_rate_dps, delta_t_s);

        if (target_yaw_rate == 0.0f) {
            yaw_cmd = yaw_sp_pid_out;
        } else {
            yaw_cmd = yaw_rate_pid_out;
            yaw_sp_deg = yaw_est_deg;
        }
    }
    /**
     * ==== END Yaw command ====
     */

    /**
     * ==== Pitch/roll command ====
     */
    if (lateral_mode == LATERAL_MODE_FIXED_POS) {
        const float x_body_measure_m = get_x_body(yaw_est_deg * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
        const float y_body_measure_m = get_y_body(yaw_est_deg * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
        const float x_body_sp_m = get_x_body(yaw_est_deg * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);
        const float y_body_sp_m = get_y_body(yaw_est_deg * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);

        const float x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, delta_t_s);
        const float y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, delta_t_s);

        roll_cmd  = pid_update(roll_pid,   x_body_pid_out, roll_est_deg,  delta_t_s);
        pitch_cmd = pid_update(pitch_pid, -y_body_pid_out, pitch_est_deg, delta_t_s);

    } else if (lateral_mode == LATERAL_MODE_ANGLE_RATE) {
        roll_cmd = pid_update(roll_rate_pid, roll_channel * MAX_ANGLE_RATE, roll_rate_dps, delta_t_s);
        pitch_cmd = pid_update(pitch_rate_pid, pitch_channel * MAX_ANGLE_RATE, pitch_rate_dps, delta_t_s);

    } else if (lateral_mode == LATERAL_MODE_ANGLE) {
        roll_cmd = pid_update(roll_pid, roll_channel * MAX_ANGLE, roll_est_deg, delta_t_s);
        pitch_cmd = pid_update(pitch_pid, -pitch_channel * MAX_ANGLE, pitch_est_deg, delta_t_s);
    }
    /**
     * ==== END Pitch/roll command ====
     */


    /**
     * ==== OUTPUT =================================================================================
     */

    if (is_armed) {
        float roll_div = cos(roll_est_deg * DEG_TO_RAD);
        if (roll_div < 0.5f) roll_div = 0.5f;

        float pitch_div = cos(pitch_est_deg * DEG_TO_RAD);
        if (pitch_div < 0.5f) pitch_div = 0.5f;
        
        throttle_cmd = throttle_cmd / pitch_div / roll_div;
        
        engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, throttle_cmd, yaw_cmd, pitch_cmd, roll_cmd));
        engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, throttle_cmd, yaw_cmd, pitch_cmd, roll_cmd));
        engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, throttle_cmd, yaw_cmd, pitch_cmd, roll_cmd));
        engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, throttle_cmd, yaw_cmd, pitch_cmd, roll_cmd));

    } else {
        engine_0_cmd_norm = 0.0f;
        engine_1_cmd_norm = 0.0f;
        engine_2_cmd_norm = 0.0f;
        engine_3_cmd_norm = 0.0f;
    }
}

void after_loop(void) {

    float to_jsbsim[] = {
        engine_0_cmd_norm,
        engine_1_cmd_norm,
        engine_2_cmd_norm,
        engine_3_cmd_norm
    };

    
    j_packet_send(0x02, to_jsbsim, sizeof(to_jsbsim), sizeof(float), J_PACKET_SIZE, j_packet_send_callback);
    
    /**
     * Commands
     */
    parse_user_command();

    char buf[512] = { 0 };

    /**
     * Simple debug output
     */
    // snprintf(buf, sizeof(buf), "%f\r\n", alt_est_m);
    // j_packet_send(0x03, (void *)buf, strlen(buf), 1, J_PACKET_SIZE, j_packet_send_callback);

    /**
     * Rotate quaternion that's send via telemetry by -90 deg in pitch to correctly visualize
     * the vehicle in 3D
     */


    if (iteration % (int)(LOOP_FREQUENCY / LOG_FREQUENCY) == 0) {
    // if (true) {

        float ret_x = 0 * qw + -0.7071067811865475 * qz - 0 * qy + 0.7071067811865476 * qx;
        float ret_y = -0 * qz + -0.7071067811865475 * qw + 0 * qx + 0.7071067811865476 * qy;
        float ret_z = 0 * qy - -0.7071067811865475 * qx + 0 * qw + 0.7071067811865476 * qz;
        float ret_w = -0 * qx - -0.7071067811865475 * qy - 0 * qz + 0.7071067811865476 * qw;

        /**
         * Telemetry output
         */
        snprintf(buf,
            sizeof(buf),
            "%f,"
            "%f,%f,%f,%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,%f,%f,"
            "%f,"
            "%f,%f,%f,%f,"
            "%f,%f,%f,%f,"
            "%f,%f,%f,%f,"
            "%f,"
            "%f,"
            "%f\n",

            time_s,

            ctrl_channels_norm[0],
            ctrl_channels_norm[1],
            ctrl_channels_norm[2],
            ctrl_channels_norm[3],
            ctrl_channels_norm[4],
            ctrl_channels_norm[5],
            
            ax_g,
            ay_g,
            az_g,

            lin_acc_x_g,
            lin_acc_y_g,
            lin_acc_z_g,
            
            gx_rad * RAD_TO_DEG,
            gy_rad * RAD_TO_DEG,
            gz_rad * RAD_TO_DEG,
            
            yaw_est_deg,
            pitch_est_deg,
            roll_est_deg,
            
            roll_rate_dps,
            pitch_rate_dps,
            yaw_rate_dps,

            alt_rate_est_mps,

            throttle_cmd,
            yaw_cmd,
            roll_cmd,
            pitch_cmd,

            engine_0_cmd_norm,
            engine_1_cmd_norm,
            engine_2_cmd_norm,
            engine_3_cmd_norm,
            
            ret_w,
            ret_x,
            ret_y,
            ret_z,
            
            pressure_pa,

            alt_est_m - initial_alt_m,
            
            cpu_usage);

        j_packet_send(0x01, (void *)buf, strlen(buf), 1, J_PACKET_SIZE, j_packet_send_callback);

        do_flash_log(buf);

        #ifdef MCU
        cpu_usage = (HAL_GetTick() - last_loop_time_ms) / (1000.0f / LOOP_FREQUENCY) * 100.0f;
        #endif
    }

    iteration++;
}

#ifdef MCU

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_USB_DEVICE_Init();
    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM11_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();

    HAL_Delay(500);

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

    bmi160_init(&hbmi160, &hspi3, 0, BMI160_ACC_RATE_400HZ, BMI160_ACC_RANGE_4G, BMI160_GYRO_RATE_400HZ, BMI160_GYRO_RANGE_1000DPS);
    hp203b_setup(&hhp203b, &hi2c2, 1);

    HAL_TIM_Base_Start(&htim6);
    W25qxx_Init();

    init();

    HAL_TIM_IC_Start_IT(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_Delay(2000);

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

    start_time_s = HAL_GetTick() / 1000.0f;

    while (1) {

        /**
         * Fix the frequency
         */
        if (HAL_GetTick() - last_loop_time_ms < (1000.0f / LOOP_FREQUENCY)) continue;
        
        /**
         * Skip too slow iterations
         */
        // if (HAL_GetTick() - last_loop_time_ms > (1000.0f / LOOP_FREQUENCY) * 2.0f) {
        //     last_loop_time_ms = HAL_GetTick();
        //     HAL_Delay((uint32_t)(500 / LOOP_FREQUENCY));
        //     continue;
        // }

        last_loop_time_ms = HAL_GetTick();


#ifdef HITL
        __disable_irq();
        from_jsbsim_to_glob_state();
        __enable_irq();

        /**
         * ? Fix for slow USB
         * Unfortunatelly, USB is not fast enough for 100 Hz loop, and causes occasional glitches
         * There are two separate timers for HITL, one timer is timing the MCU (last_loop_time_ms),
         * the other is timing the control loop (time_s)
         */
        if (prev_time_s == time_s) continue;

#endif  // HITL

#ifdef INDEP
        if (bmi160_get_data_rdy(&hbmi160)) {
            bmi160_update_acc_gyro_data(&hbmi160);
        }

        ax_g = hbmi160.acc_x;
        ay_g = hbmi160.acc_y;
        az_g = hbmi160.acc_z;
        gx_rad = hbmi160.gyro_x * DEG_TO_RAD;
        gy_rad = hbmi160.gyro_y * DEG_TO_RAD;
        gz_rad = hbmi160.gyro_z * DEG_TO_RAD;

        pressure_pa = hp203b_get_pressure_pa(&hhp203b);

        time_s = HAL_GetTick() / 1000.0f - start_time_s;
#endif

        control_loop();

        htim3.Instance->CCR1 = engine_0_cmd_norm * 12000.0f + 12000.0f;
        htim3.Instance->CCR2 = engine_1_cmd_norm * 12000.0f + 12000.0f;
        htim4.Instance->CCR1 = engine_2_cmd_norm * 12000.0f + 12000.0f;
        htim4.Instance->CCR2 = engine_3_cmd_norm * 12000.0f + 12000.0f;

        after_loop();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    /**
     * Input capture PPM (pin PB9)
     */
    if (htim == &htim11) {
        int32_t current_captured_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (last_captured_value == -1) {
            last_captured_value = current_captured_value;
            return;
        }

        int32_t difference = current_captured_value > last_captured_value
                                 ? current_captured_value - last_captured_value
                                 : 65535 - last_captured_value + current_captured_value;

        last_captured_value = current_captured_value;

        if (difference > 5000) {
            current_channel = 0;
            return;
        }

        if (current_channel < 0) return;
        if (current_channel >= NUM_CTRL_CHANNELS) return;

        ppm_us[current_channel] = difference;
        ctrl_channels_norm[current_channel] = (difference - 1500) / 500.0f;
        current_channel++;
    }
}

#endif  // MCU
