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

#include "Adafruit_AHRS_Madgwick.h"
#include "Adafruit_AHRS_NXPFusion.h"
#include "complementary_filter.h"
#include "constants.h"
#include "low_pass_filter.h"
#include "pid.h"
#include "quaternion.h"
#include "utils.h"

#ifdef SITL
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <map>
#include <string>
#endif  // SITL

#ifdef MCU
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
// #include "usart.h"
// #include "dma.h"

#include "bmi160.h"
#include "hp203b.h"
#include "usb-packets.h"
#include "w25qxx.h"
#endif  // MCU

/**
 * ==== END INCLUDES ====
 */

/**
 * ==== DEFINES ====
 */

// #define DEMO_SEQ                        1

#ifdef AUTOARM
#define AUTO_ARM                         true
#else
#define AUTO_ARM                         false
#endif

#define LOOP_FREQUENCY                  50.0f

#define HEADING_MODE_SP                  1 /* Heading is held at a fixed value */
#define HEADING_MODE_RATE                2 /* Heading is controlled with rate of change */
#define HEADING_MODE_NONE                3 /* Heading mode used for tunning */

#define VERTICAL_MODE_SP                 1 /* Altitude is held at a fixed value */
#define VERTICAL_MODE_RATE               2 /* Altitude is controlled with rate of change */
#define VERTICAL_MODE_DIRECT             3 /* Direct mapping of throttle pos to the engines */

#define LATERAL_MODE_FIXED_POS           1 /* Position is held at a fixed value */
#define LATERAL_MODE_ANGLE_RATE          2 /* Pitch and roll controls the angular velocity of the craft (freestyle mode) */
#define LATERAL_MODE_ANGLE               3 /* Pitch and roll controls the angle of the craft */

#define CALIBRATION_TIME_S               2.0f

#define MAX_VERICAL_RATE                 10.0f  /* M/S */
#define MAX_YAW_RATE                     100.0f /* DEG/S */
#define MAX_ANGLE_RATE                   20.0f  /* DEG/S */
#define MAX_ANGLE                        20.0f  /* DEG */

#define VERTICAL_RATE_THRUST_POS_DEADBAND 0.3f

#define ZERO_RATE_THROTTLE               0.4f

#define IDLE_ARM_THRUST                  0.1f
#define IDLE_THRUST_POS_THRESHLD        -0.9f   /* Stick position required to register idle thrust */

#define TAKEOFF_THRUST_POS_THRESHLD     -0.5f   /* Stick position required to register take-off event */
#define TAKEOFF_LIN_ACC_THRESHLD         0.1f
#define TAKEOFF_EVENT_DURATION           0.05f  /* Amount of seconds required to hold the conditions until detection */

#define LANDING_LIN_ACC_THRESHLD         0.1f
#define LANDING_RATES_THRESHLD           1.0f
#define LANDING_VERTICAL_RATE_THRESHLD   2.0f
#define LANDING_THRUST_POS_THRESHLD      0.2f
#define LANDING_EVENT_DURATION           2.0f   /* Amount of seconds required to hold the conditions until detection */

#define EVENT_TIME_INVALID               (-1.0f)

#define ROLL_CHANNEL                     0
#define PITCH_CHANNEL                    1
#define THROTTLE_CHANNEL                 2
#define YAW_CHANNEL                      3
#define ARM_CHANNEL                      4
#define VERT_MODE_CHANNEL                5

#define TUNE_ANGLE_SP                    0
#define TUNE_POS_SP                      1
#define TUNE_ALT_SP                      2
#define TUNE_YAW_SP                      3
#define TUNE_YAW_RATE                    4
#define TUNE_ALT_RATE                    5  // Towards pilot - direct, away from pilot - vert. rate

/**
 * ==== END DEFINES ====
 */

/**
 * ==== VARIABLES ====
 */

char cmd_string[128];

uint8_t heading_mode  = HEADING_MODE_RATE;
uint8_t lateral_mode  = LATERAL_MODE_ANGLE;
uint8_t vertical_mode = VERTICAL_MODE_DIRECT;

Adafruit_Madgwick sensor_fusion;

pid_state_t alt_sp_pid;   /* VERTICAL_MODE_SP */
pid_state_t alt_rate_pid; /* VERTICAL_MODE_RATE */
pid_state_t yaw_sp_pid;
pid_state_t yaw_rate_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;
pid_state_t roll_rate_pid;
pid_state_t pitch_rate_pid;

float time_s = 0.0f;
float start_time_s = 0.0f;
float prev_time_s = -1.0f;
float x_world_measure_m = 0.0f;  // world_x -> east -> longitude
float y_world_measure_m = 0.0f;  // world_y -> north -> latitude
float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
float gx_rad = 0.0f, gy_rad = 0.0f, gz_rad = 0.0f;
float pressure_pa = 0.0f;
float alt_measurement_m = 0.0f;
float alt_measurement_m_prev = 0.0f;
float temp_c = 0.0f;
float real_yaw_deg = 0.0f;

float gx_mean_rad = 0.0f, gy_mean_rad = 0.0f, gz_mean_rad = 0.0f;
float acc_norm_mean = 1.0f; 
int calibration_samples = 0;

float alt_est_m = 0.0f;
float alt_est_m_prev = 0.0f;
float alt_rate_est_mps = 0.0f;
float yaw_est_deg = 0.0f;
float yaw_est_deg_prev = 0.0f;
float pitch_est_deg = 0.0f;
float pitch_est_deg_prev = 0.0f;
float roll_est_deg = 0.0f;
float roll_est_deg_prev = 0.0f;
float yaw_rate_dps = 0.0f;
float roll_rate_dps = 0.0f;
float pitch_rate_dps = 0.0f;

int current_tunning = -1;

float alt_sp = 0.0f;
float yaw_sp_deg = 0.0f;
float x_world_sp_m = 0.0f;
float y_world_sp_m = 0.0f;

float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float lin_acc_x_g = 0.0f, lin_acc_y_g = 0.0f, lin_acc_z_g = 0.0f;

// ? FCS channels indexed from 0, RC transmitter indexed from 1
// ? Channels have range of (-1,1)
// 0-roll, 1-pitch, 2-throttle, 3-yaw, 4-arm, 5-mode, 6-mode, 7-mode
float ctrl_channels_norm[NUM_CTRL_CHANNELS] = {0};

float yaw_channel = 0.0f;
float pitch_channel = 0.0f;
float roll_channel = 0.0f;
float throttle_channel = 0.0f;
float vert_mode_channel = 0.0f;

float throttle_cmd = 0.0;
float yaw_cmd = 0.0;
float roll_cmd = 0.0;
float pitch_cmd = 0.0;

float engine_0_cmd_norm = 0.0f;
float engine_1_cmd_norm = 0.0f;
float engine_2_cmd_norm = 0.0f;
float engine_3_cmd_norm = 0.0f;

bool is_airborne = false;
bool is_armed = false;
bool force_arm = false;
bool can_do_logging = false;
float arm_channel_norm_prev = 1.0f;
float takeoff_event_start_s = -1.0f;
float landing_event_start_s = -1.0f;

#ifdef HITL
float from_jsbsim[20] = {0};
float to_jsbsim[15] = {0};
#endif  // HITL

#ifdef MCU
uint32_t last_loop_time_ms = 0;
float cpu_usage = 0.0f;

int8_t current_channel = -1;
int32_t last_captured_value = -1;
int32_t ppm_us[NUM_CTRL_CHANNELS] = {0};

uint32_t flash_page_addr = 0;
uint32_t flash_page_offset = 0;

bmi160_t hbmi160;
hp203b_t hhp203b;

#endif  // MCU

/**
 * ==== END VARIABLES ====
 */

/**
 * ==== FUNCTION DECLARATIONS ====
 */

extern "C" void init();
extern "C" void data_from_jsbsim(float *data);
extern "C" void data_to_jsbsim(float *data);
extern "C" void loop(void);
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd);
const float mixer_to_cmd(const float mixer_output);
bool detect_takeoff();
bool detect_landing();
bool detect_arm();
bool detect_disarm();
void disable_pid_integrators();
void enable_pid_integrators();
void demo_sequence();
void configure_logging();

#ifdef SITL
extern "C" void init_override(std::map<std::string, float> &config);
void tunning_config();
#endif

#ifdef MCU
extern "C" void SystemClock_Config(void);
void parse_user_command();
void write_flash_bytes(uint8_t *buf, uint32_t num_bytes);
void flash_erase();
#endif  // MCU

/**
 * ==== END FUNCTION DECLARATIONS ====
 */

extern "C" void init() {

#ifdef SITL
    printf("Hello from FCS\n");
#endif  // SITL

    /**
     * ==== Good tune ====
     */

    pid_init(alt_sp_pid,   0.0469, 0.0019, 0.0727, 0.15, -0.5, 0.5);
    pid_init(alt_rate_pid, 0.1299, 0.031,  0.0332, 0.15, -0.5, 0.5);

    pid_init(yaw_sp_pid,   0.00197,  0.00001,  0.00118, 0.15, -0.2, 0.2);
    pid_init(yaw_rate_pid, 0.000804, 0.000004, 0.000148, 0.15, -0.2, 0.2);

    pid_init(x_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 4.47, 0.1, 6.82, 0.15, -10, 10);  // Output is used as pitch setpoint

    pid_init(roll_pid,  0.00034, 0.00004, 0.000198, 0.15, -0.2, 0.2);
    pid_init(pitch_pid, 0.00034, 0.00004, 0.000198, 0.15, -0.2, 0.2);

    pid_init(roll_rate_pid,  0.002, 0, 0.0012, 0.15, -0.2, 0.2);
    pid_init(pitch_rate_pid, 0.002, 0, 0.0012, 0.15, -0.2, 0.2);

    /**
     * ==== END Good tune ====
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

    disable_pid_integrators();

    memset(cmd_string, '\0', sizeof(cmd_string));
    sensor_fusion.begin(LOOP_FREQUENCY);


}

#ifdef SITL
extern "C" void init_override(std::map<std::string, float> &config) {
    
    /**
     * alt_sp_pid_*
     * alt_rate_pid_*
     * yaw_sp_pid_*
     * yaw_rate_pid_*
     * pos_pid_*
     * angle_pid_*
     * angle_rate_pid_*
     */

    /**
     * 00
     */
    if (config.count("angle_sp_tune")) {
        current_tunning = TUNE_ANGLE_SP;

        vertical_mode = VERTICAL_MODE_DIRECT;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_ANGLE;
    }

    /**
     * 01
     */
    if (config.count("pos_sp_tune")) {
        current_tunning = TUNE_POS_SP;
        vertical_mode = VERTICAL_MODE_DIRECT;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 02
     */
    if (config.count("alt_sp_tune")) {
        current_tunning = TUNE_ALT_SP;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_NONE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 03
     */
    if (config.count("yaw_sp_tune")) {
        current_tunning = TUNE_YAW_SP;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_SP;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 04
     */
    if (config.count("yaw_rate_tune")) {
        current_tunning = TUNE_YAW_RATE;

        vertical_mode = VERTICAL_MODE_SP;
        heading_mode = HEADING_MODE_RATE;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }

    /**
     * 05
     */
    if (config.count("alt_rate_tune")) {
        current_tunning = TUNE_ALT_RATE;

        vertical_mode = VERTICAL_MODE_RATE;
        heading_mode = HEADING_MODE_SP;
        lateral_mode = LATERAL_MODE_FIXED_POS;
    }


    if (config.count("alt_sp_pid_p") > 0) alt_sp_pid.k_p = config["alt_sp_pid_p"];
    if (config.count("alt_sp_pid_i") > 0) alt_sp_pid.k_i = config["alt_sp_pid_i"];
    if (config.count("alt_sp_pid_d") > 0) alt_sp_pid.k_d = config["alt_sp_pid_d"];

    if (config.count("alt_rate_pid_p") > 0) alt_rate_pid.k_p = config["alt_rate_pid_p"];
    if (config.count("alt_rate_pid_i") > 0) alt_rate_pid.k_i = config["alt_rate_pid_i"];
    if (config.count("alt_rate_pid_d") > 0) alt_rate_pid.k_d = config["alt_rate_pid_d"];

    if (config.count("yaw_sp_pid_p") > 0) yaw_sp_pid.k_p = config["yaw_sp_pid_p"];
    if (config.count("yaw_sp_pid_i") > 0) yaw_sp_pid.k_i = config["yaw_sp_pid_i"];
    if (config.count("yaw_sp_pid_d") > 0) yaw_sp_pid.k_d = config["yaw_sp_pid_d"];

    if (config.count("yaw_rate_pid_p") > 0) yaw_rate_pid.k_p = config["yaw_rate_pid_p"];
    if (config.count("yaw_rate_pid_i") > 0) yaw_rate_pid.k_i = config["yaw_rate_pid_i"];
    if (config.count("yaw_rate_pid_d") > 0) yaw_rate_pid.k_d = config["yaw_rate_pid_d"];

    if (config.count("pos_pid_p") > 0) x_body_pid.k_p = config["pos_pid_p"];
    if (config.count("pos_pid_p") > 0) y_body_pid.k_p = config["pos_pid_p"];
    if (config.count("pos_pid_i") > 0) x_body_pid.k_i = config["pos_pid_i"];
    if (config.count("pos_pid_i") > 0) y_body_pid.k_i = config["pos_pid_i"];
    if (config.count("pos_pid_d") > 0) x_body_pid.k_d = config["pos_pid_d"];
    if (config.count("pos_pid_d") > 0) y_body_pid.k_d = config["pos_pid_d"];

    if (config.count("angle_pid_p") > 0) roll_pid.k_p = config["angle_pid_p"];
    if (config.count("angle_pid_p") > 0) pitch_pid.k_p = config["angle_pid_p"];
    if (config.count("angle_pid_i") > 0) roll_pid.k_i = config["angle_pid_i"];
    if (config.count("angle_pid_i") > 0) pitch_pid.k_i = config["angle_pid_i"];
    if (config.count("angle_pid_d") > 0) roll_pid.k_d = config["angle_pid_d"];
    if (config.count("angle_pid_d") > 0) pitch_pid.k_d = config["angle_pid_d"];

    if (config.count("angle_rate_pid_p") > 0) roll_rate_pid.k_p = config["angle_rate_pid_p"];
    if (config.count("angle_rate_pid_p") > 0) pitch_rate_pid.k_p = config["angle_rate_pid_p"];
    if (config.count("angle_rate_pid_i") > 0) roll_rate_pid.k_i = config["angle_rate_pid_i"];
    if (config.count("angle_rate_pid_i") > 0) pitch_rate_pid.k_i = config["angle_rate_pid_i"];
    if (config.count("angle_rate_pid_d") > 0) roll_rate_pid.k_d = config["angle_rate_pid_d"];
    if (config.count("angle_rate_pid_d") > 0) pitch_rate_pid.k_d = config["angle_rate_pid_d"];
}

void tunning_config() {
    if (current_tunning < 0) return;

    throttle_channel = -1.0f;

    if (current_tunning == TUNE_ANGLE_SP) {
        if (time_s >  5.0f) throttle_channel =  0.1f;
        if (time_s > 15.0f) pitch_channel    = -1.0f;
    }

    if (current_tunning == TUNE_POS_SP) {
        if (time_s >  5.0f) throttle_channel = 0.55f;
        if (time_s > 15.0f) x_world_sp_m = 0.00005f * DEG_TO_GEO_M;
    }

    if (current_tunning == TUNE_ALT_SP) {
        if (time_s >  5.0f) alt_sp = 5.0f;
    }

    if (current_tunning == TUNE_YAW_SP) {
        if (time_s >  5.0f) alt_sp = 5.0f;
        if (time_s > 15.0f) yaw_sp_deg = 25.0f;
    }

    if (current_tunning == TUNE_YAW_RATE) {
        if (time_s >  5.0f) alt_sp = 5.0f;
        if (time_s > 15.0f) yaw_channel = 1.0f;
    }

    if (current_tunning == TUNE_ALT_RATE) {
        if (time_s >  5.0f) throttle_channel = 1.0f;
        if (time_s > 20.0f) throttle_channel = 0.7f; /* Ajdusted for deadband to result in 5 m/s rate */
    }
}

#endif


void demo_sequence() {
    alt_sp = 0.0f;
    x_world_sp_m = 0.0f;
    y_world_sp_m = 0.0f;
    yaw_sp_deg = 0.0f;
    
    if (time_s > 5.0) {
        alt_sp = 2.0f;
    }

    if (time_s < 10.0f) return;

    int time_s_i = (int)(time_s - 10.0f);
    const int demo_period = 40;

    if ((time_s_i % demo_period) < ((float)demo_period * 1.0f / 4.0f)) {
        x_world_sp_m = 5.0f;
        y_world_sp_m = 0.0f;

    } else if ((time_s_i % demo_period) < ((float)demo_period * 2.0f / 4.0f)) {
        x_world_sp_m = 5.0f;
        y_world_sp_m = 5.0f;
        yaw_sp_deg = 30.0f;

    } else if ((time_s_i % demo_period) < ((float)demo_period * 3.0f / 4.0f)) {
        x_world_sp_m = 0.0f;
        y_world_sp_m = 5.0f;
        yaw_sp_deg = 30.f;
        alt_sp = 5.0f;

    } else {
        x_world_sp_m = 0.0f;
        y_world_sp_m = 0.0f;
        alt_sp = 5.0f;
        
    }
}



extern "C" void data_from_jsbsim(float *data) {
    time_s = data[0];  // simulation/sim-time-sec

    x_world_measure_m = data[1] * DEG_TO_GEO_M;  // ext/longitude-deg
    y_world_measure_m = data[2] * DEG_TO_GEO_M;  // ext/latitude-deg

    /**
     * Accelerometer adjusted from JSBSim local frame to expected sensor frame (BMI160)
     */
    ax_g        = -data[3];  // sensor/imu/accelX-g
    ay_g        =  data[4];  // sensor/imu/accelY-g
    az_g        = -data[5];  // sensor/imu/accelZ-g

    gx_rad      = -data[6];  // sensor/imu/gyroX-rps
    gy_rad      =  data[7];  // sensor/imu/gyroY-rps
    gz_rad      = -data[8];  // sensor/imu/gyroZ-rps

    pressure_pa = data[9];   // sensor/baro/presStatic-Pa
    temp_c      = data[10];  // sensor/baro/temp-C

    real_yaw_deg = data[11];  // attitude/psi-deg
 

#ifdef SITL

    ctrl_channels_norm[0] = data[12];  // user-control/channel-1
    ctrl_channels_norm[1] = data[13];  // user-control/channel-2
    ctrl_channels_norm[2] = data[14];  // user-control/channel-3
    ctrl_channels_norm[3] = data[15];  // user-control/channel-4
    ctrl_channels_norm[4] = data[16];  // user-control/channel-4
    ctrl_channels_norm[5] = data[17];  // user-control/channel-4
    ctrl_channels_norm[6] = data[18];  // user-control/channel-4
    ctrl_channels_norm[7] = data[19];  // user-control/channel-4

#endif

    if (prev_time_s < 0) prev_time_s = time_s;
}

extern "C" void data_to_jsbsim(float *data) {
    data[0] = engine_0_cmd_norm;
    data[1] = engine_1_cmd_norm;
    data[2] = engine_2_cmd_norm;
    data[3] = engine_3_cmd_norm;

    data[4] = alt_est_m;
    data[5] = alt_rate_est_mps;

    data[6] = yaw_est_deg;
    data[7] = pitch_est_deg;
    data[8] = roll_est_deg;

    data[9] = lin_acc_x_g;
    data[10] = lin_acc_y_g;
    data[11] = lin_acc_z_g;

    data[12] = yaw_rate_dps;
    data[13] = pitch_rate_dps;
    data[14] = roll_rate_dps;
}

extern "C" void loop(void) {
    const float deltaT = time_s - prev_time_s;
    prev_time_s = time_s;

    if (deltaT <= 0.0f) return;

    /**
     * Calibration
     */
    if (time_s < CALIBRATION_TIME_S) {
        float curr_acc_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        acc_norm_mean = (acc_norm_mean * calibration_samples + curr_acc_norm) / (calibration_samples + 1);

        gx_mean_rad = (gx_mean_rad * calibration_samples + gx_rad) / (calibration_samples + 1);
        gy_mean_rad = (gy_mean_rad * calibration_samples + gy_rad) / (calibration_samples + 1);
        gz_mean_rad = (gz_mean_rad * calibration_samples + gz_rad) / (calibration_samples + 1);

        calibration_samples++;
        return;
    }

#ifdef SITL
    if (time_s >= CALIBRATION_TIME_S && calibration_samples > 0) {
        printf("acc_norm_mean %f\n", acc_norm_mean);

        printf("gx_mean %f°\n", gx_mean_rad * RAD_TO_DEG);
        printf("gy_mean %f°\n", gy_mean_rad * RAD_TO_DEG);
        printf("gz_mean %f°\n", gz_mean_rad * RAD_TO_DEG);

        printf("READY...\n");

        calibration_samples = 0;
    }
#endif

    if (acc_norm_mean <= 0.1f) acc_norm_mean = 1.0f;

    ax_g = ax_g / acc_norm_mean;
    ay_g = ay_g / acc_norm_mean;
    az_g = az_g / acc_norm_mean;
    gx_rad = gx_rad - gx_mean_rad;
    gy_rad = gy_rad - gy_mean_rad;
    gz_rad = gz_rad - gz_mean_rad;

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
    alt_measurement_m = (288.15f / -0.0065f) * (powf(pressure_pa / 101325.0f, (-8.314f * -0.0065f) / (G_TO_MPS * 0.0289640f)) - 1);
    // alt_measurement_m = (275.15f / -0.0065f) * (powf(pressure_pa / 100600.0f, (-8.314f * -0.0065f) / (G_TO_MPS * 0.0289640f)) - 1);
    if (alt_measurement_m_prev == 0.0f) alt_measurement_m_prev = alt_measurement_m;
    if (alt_est_m == 0.0f) alt_est_m = alt_measurement_m;
    if (alt_est_m_prev == 0.0f) alt_est_m_prev = alt_est_m;

    alt_est_m = lowpass_filter_update(0.015f, alt_est_m, alt_measurement_m_prev, alt_measurement_m, deltaT);
    alt_rate_est_mps = complementary_filter_update(alt_rate_est_mps,
                                                    0.05,
                                                    (alt_est_m - alt_est_m_prev) / deltaT,  // barometer based altitude rate estimate
                                                    lin_acc_z_g * G_TO_MPS * deltaT);       // accelerometer based altitude rate estimate

    alt_measurement_m_prev = alt_measurement_m;
    alt_est_m_prev = alt_est_m;

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


    if (yaw_sp_deg - yaw_est_deg > 180) yaw_sp_deg -= 360;
    if (yaw_sp_deg - yaw_est_deg < -180) yaw_sp_deg += 360;

    float yaw_diff_deg = yaw_est_deg - yaw_est_deg_prev;
    if (yaw_diff_deg < -180) yaw_diff_deg += 360.0f;
    if (yaw_diff_deg > 180) yaw_diff_deg -= 360.0f;

    yaw_rate_dps = yaw_diff_deg / deltaT;
    roll_rate_dps = (roll_est_deg - roll_est_deg_prev) / deltaT;
    pitch_rate_dps = (pitch_est_deg - pitch_est_deg_prev) / deltaT;

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

    if (vert_mode_channel < -0.5f) vertical_mode = VERTICAL_MODE_RATE;
    if (vert_mode_channel >  0.5f) vertical_mode = VERTICAL_MODE_DIRECT;
    

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
        throttle_cmd = ZERO_RATE_THROTTLE + pid_update(alt_sp_pid, alt_sp, alt_est_m, deltaT);

    } else if (vertical_mode == VERTICAL_MODE_RATE) {
        float target_alt_rate = 0.0f;

        if (throttle_channel < -VERTICAL_RATE_THRUST_POS_DEADBAND) {
            target_alt_rate = (throttle_channel + VERTICAL_RATE_THRUST_POS_DEADBAND) * MAX_VERICAL_RATE / (1.0 - VERTICAL_RATE_THRUST_POS_DEADBAND);
        }

        if (throttle_channel > VERTICAL_RATE_THRUST_POS_DEADBAND) {
            target_alt_rate = (throttle_channel - VERTICAL_RATE_THRUST_POS_DEADBAND) * MAX_VERICAL_RATE / (1.0 - VERTICAL_RATE_THRUST_POS_DEADBAND);
        }

        throttle_cmd = ZERO_RATE_THROTTLE + pid_update(alt_rate_pid, target_alt_rate, alt_rate_est_mps, deltaT);

        /**
         * Idle thrust when on the ground
         */
        if (!is_airborne && throttle_channel < IDLE_THRUST_POS_THRESHLD) {
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
        yaw_cmd = pid_update(yaw_sp_pid, yaw_sp_deg, yaw_est_deg, deltaT);

    } else if (heading_mode == HEADING_MODE_RATE) {
        yaw_cmd = pid_update(yaw_rate_pid, yaw_channel * MAX_YAW_RATE, yaw_rate_dps, deltaT);
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

        const float x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
        const float y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);

        roll_cmd  = pid_update(roll_pid,   x_body_pid_out, roll_est_deg,  deltaT);
        pitch_cmd = pid_update(pitch_pid, -y_body_pid_out, pitch_est_deg, deltaT);

    } else if (lateral_mode == LATERAL_MODE_ANGLE_RATE) {
        roll_cmd = pid_update(roll_rate_pid, roll_channel * MAX_ANGLE_RATE, roll_rate_dps, deltaT);
        pitch_cmd = pid_update(pitch_rate_pid, pitch_channel * MAX_ANGLE_RATE, pitch_rate_dps, deltaT);

    } else if (lateral_mode == LATERAL_MODE_ANGLE) {
        roll_cmd = pid_update(roll_pid, roll_channel * MAX_ANGLE, roll_est_deg, deltaT);
        pitch_cmd = pid_update(pitch_pid, -pitch_channel * MAX_ANGLE, pitch_est_deg, deltaT);
    }
    /**
     * ==== END Pitch/roll command ====
     */


    /**
     * ==== OUTPUT =================================================================================
     */

    if (is_armed) {
        // throttle_cmd = throttle_cmd / cos(roll_est_deg * DEG_TO_RAD) / cos(pitch_est_deg * DEG_TO_RAD);
        
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

#ifdef MCU

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_USB_DEVICE_Init();
    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM11_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

    bmi160_init(&hbmi160, &hspi3, 0, BMI160_ACC_RATE_50HZ, BMI160_ACC_RANGE_4G, BMI160_GYRO_RATE_50HZ, BMI160_GYRO_RANGE_1000DPS);
    hp203b_setup(&hhp203b, &hi2c2, 1);

    W25qxx_Init();

    init();

    HAL_TIM_IC_Start_IT(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    HAL_Delay(2000);

    start_time_s = HAL_GetTick() / 1000.0f;

    while (1) {
        
        // ==== SITL NOTES ====
        // Telemetry ?
        // Commands ?
        // Config ?
        // ==== END SITL NOTES ====

        /**
         * Fix the frequency to 50 Hz (20 ms)
         */
        if (HAL_GetTick() - last_loop_time_ms < (1000.0f / LOOP_FREQUENCY)) continue;
        last_loop_time_ms = HAL_GetTick();

#ifdef HITL
        __disable_irq();
        data_from_jsbsim(from_jsbsim);
        __enable_irq();
#endif  // HITL

#ifdef INDEP
        if (bmi160_get_data_rdy(&hbmi160)) {
            bmi160_update_acc_gyro_data(&hbmi160);
        }

        time_s = HAL_GetTick() / 1000.0f - start_time_s;
        ax_g = hbmi160.acc_x;
        ay_g = hbmi160.acc_y;
        az_g = hbmi160.acc_z;
        gx_rad = hbmi160.gyro_x * DEG_TO_RAD;
        gy_rad = hbmi160.gyro_y * DEG_TO_RAD;
        gz_rad = hbmi160.gyro_z * DEG_TO_RAD;

        pressure_pa = hp203b_get_pressure_pa(&hhp203b);

// TODO pressure
#endif

        loop();

        htim3.Instance->CCR1 = engine_0_cmd_norm * 3000.0f + 3000.0f;
        htim3.Instance->CCR2 = engine_1_cmd_norm * 3000.0f + 3000.0f;
        htim4.Instance->CCR1 = engine_2_cmd_norm * 3000.0f + 3000.0f;
        htim4.Instance->CCR2 = engine_3_cmd_norm * 3000.0f + 3000.0f;


#ifdef HITL
        data_to_jsbsim(to_jsbsim);
        send_data_over_usb_packets(0x02, to_jsbsim, sizeof(to_jsbsim), sizeof(float), 64);
#endif  // HITL

        /**
         * Commands
         */
        parse_user_command();

        char buf[512];
        /**
         * Simple debug output
         */
        // sprintf(buf, "%f\r\n", alt_est_m);
        // send_data_over_usb_packets(0x03, (void *)buf, strlen(buf), 1, 64);

        /**
         * Rotate quaternion that's send via telemetry by -90 deg in pitch to correctly visualize
         * the vehicle in 3D
         */
        float ret_x = 0 * qw + -0.7071067811865475 * qz - 0 * qy + 0.7071067811865476 * qx;
        float ret_y = -0 * qz + -0.7071067811865475 * qw + 0 * qx + 0.7071067811865476 * qy;
        float ret_z = 0 * qy - -0.7071067811865475 * qx + 0 * qw + 0.7071067811865476 * qz;
        float ret_w = -0 * qx - -0.7071067811865475 * qy - 0 * qz + 0.7071067811865476 * qw;

        /**
         * Telemetry output
         */
        sprintf(buf,
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
                "%f,"
                "%f\n",

                time_s,

                ctrl_channels_norm[0],
                ctrl_channels_norm[1],
                ctrl_channels_norm[2],
                ctrl_channels_norm[3],
                ctrl_channels_norm[4],
                ctrl_channels_norm[5],
                
                hbmi160.acc_x,
                hbmi160.acc_y,
                hbmi160.acc_z,

                lin_acc_x_g,
                lin_acc_y_g,
                lin_acc_z_g,
                
                hbmi160.gyro_x,
                hbmi160.gyro_y,
                hbmi160.gyro_z,
                
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

                0.3,
                
                pressure_pa,

                alt_est_m,
                
                cpu_usage);
        send_data_over_usb_packets(0x01, (void *)buf, strlen(buf), 1, 64);

        if (can_do_logging && is_armed) {
            write_flash_bytes((uint8_t *)buf, strlen(buf));
            HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        }


        cpu_usage = (HAL_GetTick() - last_loop_time_ms) / (1000.0f / LOOP_FREQUENCY) * 100.0f;
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

void parse_user_command() {
    if (cmd_string[strlen(cmd_string) - 1] != '\n') return;

    cmd_string[strlen(cmd_string) - 1] = '\0';  // trim string
    char *token = strtok(cmd_string, " ");

    char res_buf[256];


    if (strcmp("help", token) == 0) {
        sprintf(res_buf, "sayhi\r\nalt_sp <param>\r\narm\r\ndisarm\r\ndumplog\r\ndellog\r\n");
        send_data_over_usb_packets(0x00, (void *)res_buf, strlen(res_buf), 1, 64);
    }
    if (strcmp("sayhi", token) == 0) {
        sprintf(res_buf, "I wont say hi\r\n");
        send_data_over_usb_packets(0x00, (void *)res_buf, strlen(res_buf), 1, 64);
    }
    if (strcmp("alt_sp", token) == 0) {
        char *val = strtok(NULL, " ");
        alt_sp = atof(val);
    }
    if (strcmp("arm", token) == 0) {
        is_armed = true;
        force_arm = true;
        configure_logging();
        
    }
    if (strcmp("disarm", token) == 0) {
        is_armed = false;
        force_arm = false;

    }
    if (strcmp("dumplog", token) == 0) {
        int page = 0;
        while (!W25qxx_IsEmptyPage(page, 0, 256)) {
            W25qxx_ReadPage((uint8_t *)res_buf, page, 0, 256);
            send_data_over_usb_packets(0x00, (void *)res_buf, 256, 1, 64);

            HAL_Delay(10);

            page += 1;
        }
    }
    if (strcmp("dellog", token) == 0) {
        sprintf(res_buf, "Erasing...\r\n");
        send_data_over_usb_packets(0x00, (void *)res_buf, strlen(res_buf), 1, 64);
        flash_erase();
        sprintf(res_buf, "Done erasing\r\n");
        send_data_over_usb_packets(0x00, (void *)res_buf, strlen(res_buf), 1, 64);

    }

    memset(cmd_string, '\0', sizeof(cmd_string));
}

void packet_from_usb_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset) {
    /**
     * Cmd data in
     */
    if (channel_number == 0x00) {
        memcpy(cmd_string + data_offset, current_data + PACKET_HEADER_SIZE, data_size);
    }

    /**
     * There is no data in on channel 1 (it is for telemetry output)
     */

#ifdef HITL
    /**
     * HITL data in
     */
    if (channel_number == 0x02) {
        memcpy((uint8_t *)from_jsbsim + data_offset, current_data + PACKET_HEADER_SIZE, data_size);
    }
#endif
}


void write_flash_bytes(uint8_t *buf, uint32_t num_bytes) {

    const uint32_t page_size = 256;
    uint32_t remaining_bytes = num_bytes;

    while (remaining_bytes > 0) {
        uint32_t bytes_to_write = MIN(remaining_bytes, page_size - flash_page_offset);
        
        W25qxx_WritePage(buf, flash_page_addr, flash_page_offset, bytes_to_write);

        flash_page_offset += bytes_to_write;
        if (flash_page_offset >= page_size) {
            flash_page_offset = 0;
            flash_page_addr += 1;
        }

        buf += bytes_to_write;
        remaining_bytes -= bytes_to_write;
    }
}

void flash_erase() {

    W25qxx_EraseChip();

    flash_page_offset = 0;
    flash_page_addr = 0;
}

#endif  // MCU

/**
 * @param engine_id Engine id from 0 to 3
 */
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd) {
    float min_output = throttle_cmd - abs(yaw_cmd) - abs(pitch_cmd) - abs(roll_cmd);

    float thrust_offset = 0.0f;
    if (min_output < IDLE_ARM_THRUST) thrust_offset = IDLE_ARM_THRUST - min_output;

    if (engine_id == 0) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 1) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd + thrust_offset;
    }

    if (engine_id == 2) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd + thrust_offset;
    }

    if (engine_id == 3) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd + thrust_offset;
    }

    return 0;
}

const float mixer_to_cmd(const float mixer_output) {
    return std::max(IDLE_ARM_THRUST, std::min(1.0f, mixer_output));
}

/**
 * Needs maximum sensitivity. Cannot miss take-off detection but can incorrectly detect take-off
 * @return Returns true only once per take-off
 */
bool detect_takeoff() {
    // TODO maybe check rates
    // TODO maybe check vectical_rate_est

    if (is_airborne
    || vector_norm(lin_acc_x_g, lin_acc_y_g, lin_acc_z_g) < TAKEOFF_LIN_ACC_THRESHLD
    || (vertical_mode != VERTICAL_MODE_SP && ctrl_channels_norm[THROTTLE_CHANNEL] < TAKEOFF_THRUST_POS_THRESHLD)) {
        takeoff_event_start_s = EVENT_TIME_INVALID;
        return false;
    }

    /**
     * All checks OK
     */

    if (takeoff_event_start_s == EVENT_TIME_INVALID) {
        takeoff_event_start_s = time_s;
        return false;
    }

    if (time_s - takeoff_event_start_s < TAKEOFF_EVENT_DURATION) {
        return false;
    }

#ifdef SITL
    printf("Take-off detected\n");
#endif
    takeoff_event_start_s = EVENT_TIME_INVALID;
    return true;
}

/**
 * Needs maximum specificity. Cannot detect landing whilst in the air, but can miss actual landing
 * @return Returns true only once per landing
 */
bool detect_landing() {
    if (!is_airborne
    || vector_norm(lin_acc_x_g, lin_acc_y_g, lin_acc_z_g) > LANDING_LIN_ACC_THRESHLD
    || roll_rate_dps > LANDING_RATES_THRESHLD
    || pitch_rate_dps > LANDING_RATES_THRESHLD
    || (vertical_mode != VERTICAL_MODE_SP && ctrl_channels_norm[THROTTLE_CHANNEL] > LANDING_THRUST_POS_THRESHLD)) {
        landing_event_start_s = EVENT_TIME_INVALID;
        return false;
    }

    /**
     * All checks OK
     */

    if (landing_event_start_s == EVENT_TIME_INVALID) {
        landing_event_start_s = time_s;
        return false;
    }

    if (time_s - landing_event_start_s < LANDING_EVENT_DURATION) {
        return false;
    }

#ifdef SITL
    printf("Landing detected\n");
#endif
    landing_event_start_s = EVENT_TIME_INVALID;
    return true;
}

bool detect_arm() {
    if (!is_armed && AUTO_ARM) return true;

    if (is_armed) return false;
    if (ctrl_channels_norm[THROTTLE_CHANNEL] > IDLE_THRUST_POS_THRESHLD) return false;
    if (is_airborne) return false;
    if (arm_channel_norm_prev > -0.5f) return false;
    if (ctrl_channels_norm[ARM_CHANNEL] < 0.5f) return false;

#ifdef SITL
    printf("Arm detected!\n");
#endif

    return true;
}

bool detect_disarm() {
    if (AUTO_ARM) return false;
    if (!is_armed) return false;
    if (force_arm) return false;
    if (ctrl_channels_norm[ARM_CHANNEL] > 0.5f) return false;

#ifdef SITL
    printf("Disarm detected\n");
#endif

    return true;
}

void disable_pid_integrators() {
    pid_integrator_disable(alt_sp_pid);
    pid_integrator_disable(alt_rate_pid);
    pid_integrator_disable(yaw_sp_pid);
    pid_integrator_disable(yaw_rate_pid);
    pid_integrator_disable(x_body_pid);
    pid_integrator_disable(y_body_pid);
    pid_integrator_disable(roll_pid);
    pid_integrator_disable(pitch_pid);
    pid_integrator_disable(roll_rate_pid);
    pid_integrator_disable(pitch_rate_pid);
}

void enable_pid_integrators() {
    pid_integrator_enable(alt_sp_pid);
    pid_integrator_enable(alt_rate_pid);
    pid_integrator_enable(yaw_sp_pid);
    pid_integrator_enable(yaw_rate_pid);
    pid_integrator_enable(x_body_pid);
    pid_integrator_enable(y_body_pid);
    pid_integrator_enable(roll_pid);
    pid_integrator_enable(pitch_pid);
    pid_integrator_enable(roll_rate_pid);
    pid_integrator_enable(pitch_rate_pid);
}

void configure_logging() {
#ifndef MCU
    can_do_logging = false;
    return;
#else

    if (flash_page_addr == 0) {
        can_do_logging = W25qxx_IsEmptyPage(0, 0, 256);

    } else {
        can_do_logging = true;
        
    }

#endif

}
