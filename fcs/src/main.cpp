// either INDEP, HITL, or SITL will be defined
// either MCU or CPU will be defined

// #define MCU
// #define HITL
// #define SITL

/**
 * ==== INCLUDES ====
 */
#include <algorithm>
#include <stdint.h>

#include "pid.h"
#include "quaternion.h"
#include "complementary_filter.h"
#include "low_pass_filter.h"
#include "Adafruit_AHRS_NXPFusion.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "utils.h"
#include "constants.h"

#ifdef SITL
#include <stdio.h>
#include <unistd.h>
#endif // SITL

#ifdef MCU
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

#include "usb-packets.h"
#include "bmi160.h"
#include "hp203b.h"
#endif // MCU

/**
 * ==== END INCLUDES ====
 */


/**
 * ==== DEFINES ====
 */

// #define AUTO_ARM                 true
#define AUTO_ARM                 false


#define HEADING_MODE_SP         1  /* Heading is held at a fixed value */
#define HEADING_MODE_RATE       2  /* Heading is controlled with rate of change */

#define VERTICAL_MODE_SP        1  /* Altitude is held at a fixed value */
#define VERTICAL_MODE_RATE      2  /* Altitude is controlled with rate of change */
#define VERTICAL_MODE_DIRECT    3  /* Direct mapping of throttle pos to the engines */

#define LATERAL_MODE_FIXED_POS  1  /* Position is held at a fixed value */
#define LATERAL_MODE_ANGLE_RATE 2  /* Pitch and roll controls the angular velocity of the craft (freestyle mode) */
#define LATERAL_MODE_ANGLE      3  /* Pitch and roll controls the angle of the craft */

#define CALIBRATION_TIME_S      2.0f

#define MAX_VERICAL_RATE        30.0f  /* M/S */
#define MAX_YAW_RATE            100.0f /* DEG/S */
#define MAX_ANGLE_RATE          20.0f  /* DEG/S */
#define MAX_ANGLE               40.0f  /* DEG */

#define VERTICAL_RATE_THRUST_POS_DEADBAND 0.1f

#define IDLE_ARM_THRUST          0.05f
#define IDLE_THRUST_POS_THRESHLD 0.05f /* Stick position required to register idle thrust */

#define TAKEOFF_THRUST_POS_THRESHLD 0.2f /* Stick position required to register take-off event */
#define TAKEOFF_LIN_ACC_THRESHLD 0.1f
#define TAKEOFF_EVENT_DURATION   0.05f /* Amount of seconds required to hold the conditions until detection */

#define LANDING_LIN_ACC_THRESHLD 0.1f
#define LANDING_RATES_THRESHLD   1.0f
#define LANDING_THRUST_POS_THRESHLD 0.2f
#define LANDING_EVENT_DURATION   1.0f /* Amount of seconds required to hold the conditions until detection */

#define EVENT_TIME_INVALID      (-1.0f)

#define ROLL_CHANNEL            0
#define PITCH_CHANNEL           1
#define THROTTLE_CHANNEL        2
#define YAW_CHANNEL             3
#define ARM_CHANNEL             4

/**
 * ==== END DEFINES ====
 */


/**
 * ==== VARIABLES ====
 */

char cmd_string[256];

const float rest_time_s = 5.0;
const uint8_t heading_mode = HEADING_MODE_RATE;
const uint8_t vertical_mode = VERTICAL_MODE_RATE;
const uint8_t lateral_mode = LATERAL_MODE_ANGLE;

Adafruit_Madgwick sensor_fusion;

pid_state_t alt_sp_pid;         /* VERTICAL_MODE_SP */
pid_state_t alt_rate_pid;       /* VERTICAL_MODE_RATE */
pid_state_t yaw_sp_pid;
pid_state_t yaw_rate_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;
pid_state_t roll_rate_pid;
pid_state_t pitch_rate_pid;

float time_s;
float prev_time_s;
float x_world_measure_m;
float y_world_measure_m;
float ax_g, ay_g, az_g, gx_rad, gy_rad, gz_rad;
float pressure_pa;
float alt_measurement_m;
float alt_measurement_m_prev;
float temp_c;

float gx_mean_rad, gy_mean_rad, gz_mean_rad, acc_norm_mean;
int calibration_samples;

float alt_est_m;
float alt_est_m_prev;
float alt_rate_est_mps;
float yaw_est_deg;
float yaw_est_deg_prev;
float pitch_est_deg;
float pitch_est_deg_prev;
float roll_est_deg;
float roll_est_deg_prev;
float yaw_rate_dps;
float roll_rate_dps;
float pitch_rate_dps;

float qw, qx, qy, qz;
float lin_acc_x_g, lin_acc_y_g, lin_acc_z_g;

// ? FCS channels indexed from 0, RC transmitter indexed from 1
// 0-roll, 1-pitch, 2-throttle, 3-yaw, 4-arm, 5-mode, 6-mode, 7-mode
float ctrl_channels_norm[NUM_CTRL_CHANNELS] = { 0 };

float engine_0_cmd_norm;
float engine_1_cmd_norm;
float engine_2_cmd_norm;
float engine_3_cmd_norm;

bool is_airborne;
bool is_armed;
float arm_channel_norm_prev;
float takeoff_event_start_s;
float landing_event_start_s;

#ifdef HITL
float from_jsbsim[22] = { 0 };
float to_jsbsim[12] = { 0 };
#endif // HITL

#ifdef MCU
bmi160_t himu;

int8_t current_channel;
int32_t last_captured_value;
uint32_t ppm_us[NUM_CTRL_CHANNELS] = { 0 };

#endif // MCU

/**
 * ==== END VARIABLES ====
 */


/**
 * ==== FUNCTION DECLARATIONS ====
 */

extern "C" void init(/* json *sim_data */);
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

#ifdef MCU
extern "C" void SystemClock_Config(void);
#endif // MCU

/**
 * ==== END FUNCTION DECLARATIONS ====
 */

// #ifdef SITL
// extern "C" void pre_init(json *sim_data) {}
// #endif


extern "C" void init() {

#ifdef SITL
    printf("Hello from FCS\n");
#endif // SITL

    pid_init(alt_sp_pid,    0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    pid_init(alt_rate_pid,  0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    
    pid_init(yaw_sp_pid,    0.012, 0.02,  0.005,    0.15, -0.2, 0.2);
    pid_init(yaw_rate_pid,  0.008, 0.0,   0.0005,   0.15, -0.2, 0.2);
    
    pid_init(x_body_pid,    4,     0.1,   4,        0.15, -10,  10);  // Output is used as roll  setpoint
    pid_init(y_body_pid,    4,     0.1,   4,        0.15, -10,  10);  // Output is used as pitch setpoint

    pid_init(roll_pid,      0.002, 0,     0.0012,   0.15, -0.2, 0.2);
    pid_init(pitch_pid,     0.002, 0,     0.0012,   0.15, -0.2, 0.2);

    pid_init(roll_rate_pid,  0.002, 0,     0.0012,   0.15, -0.2, 0.2);
    pid_init(pitch_rate_pid, 0.002, 0,     0.0012,   0.15, -0.2, 0.2);

    pid_integrator_disable(roll_pid);
    pid_integrator_disable(pitch_pid);

    time_s = 0;
    prev_time_s = -1;

    x_world_measure_m = 0;
    y_world_measure_m = 0;

    ax_g = 0.0f;
    ay_g = 0.0f;
    az_g = 0.0f;
    gx_rad = 0.0f;
    gy_rad = 0.0f;
    gz_rad = 0.0f;
    pressure_pa = 0.0f;
    alt_measurement_m = 0.0f;
    alt_measurement_m_prev = 0.0f;
    temp_c = 0.0f;

    gx_mean_rad = 0.0f;
    gy_mean_rad = 0.0f;
    gz_mean_rad = 0.0f;
    acc_norm_mean = 0.0f;
    calibration_samples = 0.0f;

    alt_est_m = 0.0f;
    alt_est_m_prev = 0.0f;
    alt_rate_est_mps = 0.0f;
    yaw_est_deg = 0.0f;
    yaw_est_deg_prev = 0.0f;
    pitch_est_deg = 0.0f;
    pitch_est_deg_prev = 0.0f;
    roll_est_deg = 0.0f;
    roll_est_deg_prev = 0.0f;
    yaw_rate_dps = 0.0f;
    roll_rate_dps = 0.0f;
    pitch_rate_dps = 0.0f;

    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;

    lin_acc_x_g = 0.0f;
    lin_acc_y_g = 0.0f;
    lin_acc_z_g = 0.0f;

    engine_0_cmd_norm = 0.0;
    engine_1_cmd_norm = 0.0;
    engine_2_cmd_norm = 0.0;
    engine_3_cmd_norm = 0.0;
    
    is_airborne = false;
    is_armed = false;
    arm_channel_norm_prev = 1.0f; /* Conservative for safety */
    takeoff_event_start_s = -1.0f;
    landing_event_start_s = -1.0f;

#ifdef MCU
    current_channel = -1;
    last_captured_value = -1;
#endif // MCU

    sensor_fusion.begin(50.0);
}


extern "C" void data_from_jsbsim(float *data) {
    time_s              = data[0];                  // simulation/sim-time-sec

    x_world_measure_m   = data[1] * DEG_TO_GEO_M;   // ext/longitude-deg
    y_world_measure_m   = data[2] * DEG_TO_GEO_M;   // ext/latitude-deg

    /**
     * Accelerometer adjusted from JSBSim local frame to expected sensor frame (BMI160)
     */
    ax_g                = -data[3];                 // sensor/imu/accelX-g
    ay_g                =  data[4];                 // sensor/imu/accelY-g
    az_g                = -data[5];                 // sensor/imu/accelZ-g

    gx_rad              = -data[6];                 // sensor/imu/gyroX-rps
    gy_rad              =  data[7];                 // sensor/imu/gyroY-rps
    gz_rad              = -data[8];                 // sensor/imu/gyroZ-rps

    pressure_pa         = data[9];                  // sensor/baro/presStatic-Pa
    temp_c              = data[10];                 // sensor/baro/temp-C

    #ifdef SITL
    
    ctrl_channels_norm[0]    = data[11];                 // user-control/channel-1
    ctrl_channels_norm[1]    = data[12];                 // user-control/channel-2
    ctrl_channels_norm[2]    = data[13];                 // user-control/channel-3
    ctrl_channels_norm[3]    = data[14];                 // user-control/channel-4
    ctrl_channels_norm[4]    = data[15];                 // user-control/channel-4
    ctrl_channels_norm[5]    = data[16];                 // user-control/channel-4
    ctrl_channels_norm[6]    = data[17];                 // user-control/channel-4
    ctrl_channels_norm[7]    = data[18];                 // user-control/channel-4
    
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

    data[9]  = lin_acc_x_g;
    data[10] = lin_acc_y_g;
    data[11] = lin_acc_z_g;
}

extern "C" void loop(void) {
    const float deltaT = time_s - prev_time_s;
    prev_time_s = time_s;

    if (deltaT == 0.0f) return;

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

    ax_g = ax_g / acc_norm_mean;
    ay_g = ay_g / acc_norm_mean;
    az_g = az_g / acc_norm_mean;
    gx_rad = gx_rad - gx_mean_rad;
    gy_rad = gy_rad - gy_mean_rad;
    gz_rad = gz_rad - gz_mean_rad;

    
    sensor_fusion.updateIMU(-gx_rad * RAD_TO_DEG, gy_rad * RAD_TO_DEG, -gz_rad * RAD_TO_DEG, ax_g, -ay_g, az_g);
    sensor_fusion.getQuaternion(&qw, &qx, &qy, &qz);
    roll_est_deg    = sensor_fusion.getRoll();
    pitch_est_deg   = sensor_fusion.getPitch();
    yaw_est_deg     = sensor_fusion.getYaw() - 180.0;

    Quaterion_t acc   = { 0, ax_g, ay_g, az_g };
    Quaterion_t q     = { qw, qx, qy, qz };
    Quaterion_t q_inv = { qw, -qx, -qy, -qz };
    Quaterion_t acc_rot = multiplyQuat(multiplyQuat(q, acc), q_inv);

    lin_acc_x_g = acc_rot.x;
    lin_acc_y_g = acc_rot.y;
    lin_acc_z_g = acc_rot.z - 1.0f;

    /**
     * ==== Altitude ====
     */
    alt_measurement_m = (288.15f / -0.0065f) * (powf(pressure_pa / 101325.0f, (-8.314f * -0.0065f) / (G_TO_MPS * 0.0289640f)) - 1);    
    if (alt_measurement_m_prev == 0.0f) alt_measurement_m_prev = alt_measurement_m;
    if (alt_est_m_prev == 0.0f) alt_est_m_prev = alt_est_m;

    alt_est_m = lowpass_filter_update(0.1f, alt_est_m, alt_measurement_m_prev, alt_measurement_m, deltaT);
    alt_rate_est_mps = complementary_filter_update(alt_rate_est_mps,
                                                   0.05,
                                                   (alt_est_m - alt_est_m_prev) / deltaT,  // barometer based altitude rate estimate
                                                   lin_acc_z_g * G_TO_MPS * deltaT);       // accelerometer based altitude rate estimate

    alt_measurement_m_prev = alt_measurement_m;
    alt_est_m_prev = alt_est_m;

    float alt_sp = 0.0;
    if (time_s > rest_time_s) alt_sp = 1.0;

    /**
     * ==== Yaw ====
     */
    float yaw_sp_deg = 2;
    if (time_s > rest_time_s + 5) yaw_sp_deg = 80;
    if (yaw_sp_deg - yaw_est_deg >  180) yaw_sp_deg -= 360;
    if (yaw_sp_deg - yaw_est_deg < -180) yaw_sp_deg += 360;


    float yaw_diff_deg = yaw_est_deg - yaw_est_deg_prev;
    if (yaw_diff_deg < -180) yaw_diff_deg += 360.0f;
    if (yaw_diff_deg > 180)  yaw_diff_deg -= 360.0f;

    yaw_rate_dps   = yaw_diff_deg / deltaT;
    roll_rate_dps  = (roll_est_deg  - roll_est_deg_prev)  / deltaT;
    pitch_rate_dps = (pitch_est_deg - pitch_est_deg_prev) / deltaT;

    yaw_est_deg_prev   = yaw_est_deg;
    roll_est_deg_prev  = roll_est_deg;
    pitch_est_deg_prev = pitch_est_deg;

    /**
     * Position: world_x -> east -> longitude
     *           world_y -> north -> latitude
    */
    float x_world_sp_m = 0;
    float y_world_sp_m = 0;

    if (time_s > rest_time_s + 10) {
        x_world_sp_m = 1;
        y_world_sp_m = 2.5;
    }

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
    }
    if (detect_disarm()) {
        is_armed = false;
        // is_airborne = false;
        disable_pid_integrators();
    }
    
    arm_channel_norm_prev = ctrl_channels_norm[ARM_CHANNEL];

    /**
     * ==== PIDs ===================================================================================
     */

    float throttle_cmd = 0.0;
    float yaw_cmd = 0.0;
    float roll_cmd = 0.0;
    float pitch_cmd = 0.0;

    /**
     * ==== Throttle command ====
     */
    if (vertical_mode == VERTICAL_MODE_SP) {
        throttle_cmd = 0.5 + pid_update(alt_sp_pid, alt_sp, alt_est_m, deltaT);

    } else if (vertical_mode == VERTICAL_MODE_RATE) {
        float target_alt_rate = 0.0f;

        float verical_rate_deadband_low = 0.5f - VERTICAL_RATE_THRUST_POS_DEADBAND;
        float verical_rate_deadband_high = 0.5f + VERTICAL_RATE_THRUST_POS_DEADBAND;

        if (ctrl_channels_norm[2] < verical_rate_deadband_low) {
            target_alt_rate = (ctrl_channels_norm[THROTTLE_CHANNEL] - verical_rate_deadband_low) * MAX_VERICAL_RATE;
        }

        if (ctrl_channels_norm[2] > verical_rate_deadband_high) {
            target_alt_rate = (ctrl_channels_norm[THROTTLE_CHANNEL] - verical_rate_deadband_high) * MAX_VERICAL_RATE;
        }
        
        // TODO what will be the zero rate throttle?
        throttle_cmd = 0.5 + pid_update(alt_rate_pid, target_alt_rate, alt_rate_est_mps, deltaT);
        
        /**
         * Idle thrust when on the ground
         */
        if (!is_airborne && ctrl_channels_norm[THROTTLE_CHANNEL] < IDLE_THRUST_POS_THRESHLD) {
            throttle_cmd = 0.0f;
        }
    } else if (vertical_mode == VERTICAL_MODE_DIRECT) {
        throttle_cmd = ctrl_channels_norm[2];
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
        yaw_cmd = pid_update(yaw_rate_pid, (ctrl_channels_norm[YAW_CHANNEL] - 0.5f) * MAX_YAW_RATE, yaw_rate_dps, deltaT);
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
        pitch_cmd = pid_update(pitch_rate_pid, (ctrl_channels_norm[PITCH_CHANNEL] - 0.5f) * MAX_ANGLE_RATE, pitch_rate_dps, deltaT);
        roll_cmd  = pid_update(roll_rate_pid,  (ctrl_channels_norm[ROLL_CHANNEL] - 0.5f) * MAX_ANGLE_RATE, roll_rate_dps,  deltaT);

    } else if (lateral_mode == LATERAL_MODE_ANGLE) {
        roll_cmd  = pid_update(roll_pid,   (ctrl_channels_norm[ROLL_CHANNEL] - 0.5) * MAX_ANGLE, roll_est_deg,  deltaT);
        pitch_cmd = pid_update(pitch_pid, -(ctrl_channels_norm[PITCH_CHANNEL] - 0.5) * MAX_ANGLE, pitch_est_deg, deltaT);
    }
    /**
     * ==== END Pitch/roll command ====
     */


    /**
     * ==== OUTPUT =================================================================================
     */


    if (is_armed) {
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

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();
    MX_TIM4_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    // MX_DMA_Init();
    // MX_USART1_UART_Init();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

    bmi160_init(&himu, &hi2c1, 0, BMI160_ACC_RATE_50HZ, BMI160_ACC_RANGE_4G, BMI160_GYRO_RATE_50HZ, BMI160_GYRO_RANGE_1000DPS);

    hp203b_setup(&hi2c1);

    init();

    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    htim10.Instance->CCR1 = 1000;
    htim9.Instance->CCR1 = 1000;
    htim9.Instance->CCR2 = 1000;
    htim11.Instance->CCR1 = 1000;

    HAL_Delay(3000);

    while (1) {
        
        // {
        //     // uint8_t hp203_status = hp203b_test(&hi2c1);
        //     float pressure = hp203b_get_pressure(&hi2c1);

        //     char buf[128];
        //     sprintf(buf, "pressure: %f\r\n", pressure);
        //     CDC_Transmit_FS((uint8_t *)buf, strlen(buf));

        //     HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        //     HAL_Delay(1000);

        //     continue;
        // }


        
        // ==== MCU CODE ====
        // Time the main loop (100 Hz / 500 Hz / 1000 Hz)
        // Read sensors (possibly using DMA)
        // Run the main logic
        // Write output to motors
        // Send telemetry (via radio or USB)
        // Log telemetry to flash
        // R/W commands via USB
        // ==== END MCU CODE ====


        // ==== HITL CODE ====
        // Time the main loop (100 Hz / 500 Hz / 1000 Hz)
        // Read data from the sim (via USB, non-DMA)
        // Run the main logic
        // Write output to sim (via USB)
        // Send telemetry (via USB)
        // Log telemetry to flash
        // R/W commands via USB
        // ==== END HITL CODE ====


        // ==== SITL CODE ====
        // Main loop is timed externally by sim
        // Receive data via function call (controlled by sim)
        // Run the main logic (controlled by sim)
        // Send the data via function call (controlled by sim)
        // Telemetry ?
        // Commands ?
        // Config file ?
        // ==== END SITL CODE ====

        
        #ifdef HITL
        __disable_irq();
        data_from_jsbsim(from_jsbsim);
        __enable_irq();
        #endif // HITL

        #ifdef INDEP
        if (bmi160_get_data_rdy(&himu)) {
            bmi160_update_acc_gyro_data(&himu);
        }
        
        time_s = HAL_GetTick() / 1000.0f;
        ax_g = himu.acc_x;
        ay_g = himu.acc_y;
        az_g = himu.acc_z;
        gx_rad = himu.gyro_x * DEG_TO_RAD;
        gy_rad = himu.gyro_y * DEG_TO_RAD;
        gz_rad = himu.gyro_z * DEG_TO_RAD;

        // TODO pressure
        #endif

        loop();

        htim10.Instance->CCR1 = engine_0_cmd_norm * 1000.0f + 1000.0f;
        htim9.Instance->CCR1 = engine_1_cmd_norm * 1000.0f + 1000.0f;
        htim9.Instance->CCR2 = engine_2_cmd_norm * 1000.0f + 1000.0f;
        htim11.Instance->CCR1 = engine_3_cmd_norm * 1000.0f + 1000.0f;

        #ifdef HITL
        data_to_jsbsim(to_jsbsim);
        send_data_over_usb_packets(0x02, to_jsbsim, sizeof(to_jsbsim), sizeof(float), 64);
        #endif // HITL

        /**
         * Simple debug output
         */
        // char buf[256];
        // sprintf(buf, "%f\r\n", alt_rate_est_mps);
        // send_data_over_usb_packets(0x00, (void *)buf, strlen(buf), 1, 64);


        /**
         * Rotate quaternion that's send via telemetry by -90 deg in pitch to correctly visualize
         * the vehicle in 3D
        */
        // float ret_x =  0 * qw + -0.7071067811865475 * qz - 0 * qy + 0.7071067811865476 * qx;
        // float ret_y = -0 * qz + -0.7071067811865475 * qw + 0 * qx + 0.7071067811865476 * qy;
        // float ret_z =  0 * qy - -0.7071067811865475 * qx + 0 * qw + 0.7071067811865476 * qz;
        // float ret_w = -0 * qx - -0.7071067811865475 * qy - 0 * qz + 0.7071067811865476 * qw;


        /**
         * Telemetry output
         */
        // sprintf(buf, "%f," "%f,%f,%f,%f," "%f,%f,%f," "%f,%f,%f," "%f,%f,%f,%f," "%f\n",
        //     time_s,
        //     ctrl_channels_norm[0],
        //     ctrl_channels_norm[1],
        //     ctrl_channels_norm[2],
        //     ctrl_channels_norm[3],
        //     himu.acc_x,
        //     himu.acc_y,
        //     himu.acc_z,
        //     yaw_est,
        //     pitch_est,
        //     roll_est,
        //     ret_w,
        //     ret_x,
        //     ret_y,
        //     ret_z,
        //     0.3);
        // send_data_over_usb_packets(0x01, (void *)buf, strlen(buf), 1, 64);

        HAL_Delay(20);

        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    
    /**
     * Input capture PPM (pin PB8)
     */
    if (htim != &htim4) return;

    int32_t current_captured_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

    if (last_captured_value == -1) {
        last_captured_value = current_captured_value;
        return;
    }

    int32_t difference = current_captured_value > last_captured_value
                       ? current_captured_value - last_captured_value
                       : 0xFFFF - last_captured_value + current_captured_value;

    last_captured_value = current_captured_value;

    if (difference > 5000) {
        current_channel = 0;
        return;
    }

    if (current_channel < 0) return;
    if (current_channel >= NUM_CTRL_CHANNELS) return;

    ppm_us[current_channel] = difference;
    ctrl_channels_norm[current_channel] = (difference - 1000) / 1000.0f;
    current_channel++;
}


void packet_from_usb_callback(uint8_t channel_number, uint8_t *current_data, uint16_t data_size, uint16_t data_offset) {
    /**
     * Cmd data in
     */
    if (channel_number == 0x00) {

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


#endif // MCU




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
        || (vertical_mode != VERTICAL_MODE_SP
            && ctrl_channels_norm[THROTTLE_CHANNEL] < TAKEOFF_THRUST_POS_THRESHLD)) {
        
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
        || (vertical_mode != VERTICAL_MODE_SP 
            && ctrl_channels_norm[THROTTLE_CHANNEL] > LANDING_THRUST_POS_THRESHLD)) {

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
    if (arm_channel_norm_prev > 0.5f) return false;
    if (ctrl_channels_norm[ARM_CHANNEL] < 0.5f) return false;

#ifdef SITL
    printf("Arm detected!\n");
#endif

    return true;
}

bool detect_disarm() {
    if (AUTO_ARM) return false;
    if (!is_armed) return false;
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
