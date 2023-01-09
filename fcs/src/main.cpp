// either INDEP, HITL, or SITL will be defined
// either MCU or CPU will be defined

// // #define MCU
// // #define HITL
// // #define SITL

/**
 * ==== INCLUDES ====
 */

#include "pid.h"
#include "quaternion.h"
#include "complementary_filter.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "Adafruit_AHRS_NXPFusion.h"

#ifdef SITL
#include <stdio.h>
#include <unistd.h>
#include <math.h>
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
#include "bmi160.h"
#include "hp203b.h"
#endif // MCU

/**
 * ==== END INCLUDES ====
 */


/**
 * ==== DEFINES ====
 */

#ifndef M_PI
#define M_PI		        3.14159265358979323846
#endif // M_PI

#define DEG_TO_GEO_M        6378000 * M_PI / 180
#define DEG_TO_RAD          M_PI / 180
#define RAD_TO_DEG          180 / M_PI

#define HEADING_MODE_SP         1  /* Heading is held at a fixed value */
#define HEADING_MODE_RATE       2  /* Heading is controlled with rate of change */

#define VERTICAL_MODE_SP        1  /* Altitude is held at a fixed value */
#define VERTICAL_MODE_RATE      2  /* Altitude is controlled with rate of change */
#define VERTICAL_MODE_DIRECT    3  /* Direct mapping of throttle pos to the engines */

#define LATERAL_MODE_FIXED_POS  1  /* Position is held at a fixed value */
#define LATERAL_MODE_RATE       2  /* Pitch and roll controls the angular velocity of the craft (freestyle mode) */
#define LATERAL_MODE_ANGLE      3  /* Pitch and roll controls the angle of the craft */

#define NUM_CTRL_CHANNELS       8

#ifndef MAX
#define MAX(x,y)            ((x)>(y)?(x):(y))
#endif // MAX

#ifndef MIN
#define MIN(x,y)            ((x)<(y)?(x):(y))
#endif // MIN

#ifdef MCU
#define PACKET_HEADER_SIZE  6 /* 1B (channel) 1B (0x55) 2B (size) 2B (offset) */
#define PACKET_FOOTER_SIZE  1 /* 1B (0x00) */
#endif // MCU

/**
 * ==== END DEFINES ====
 */


/**
 * ==== VARIABLES ====
 */

char cmd_string[256];

const float rest_time = 5.0;

Adafruit_Madgwick sensor_fusion;

pid_state_t throttle_pid;
pid_state_t yaw_pid;
pid_state_t yaw_rate_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;

uint8_t heading_mode = HEADING_MODE_RATE;
uint8_t vertical_mode = VERTICAL_MODE_RATE;
uint8_t lateral_mode = LATERAL_MODE_ANGLE;

float time_sec;
float prev_time_sec;
float x_world_measure_m;
float y_world_measure_m;
float ax, ay, az, gx, gy, gz, mx, my, mz;

float pressure_pa;
float alt_est_m;
float alt_rate_est;

// ? FCS channels indexed from 0, RC transmitter indexed from 1
// 0-roll, 1-pitch, 2-throttle, 3-yaw, 4-arm, 5-mode, 6-mode, 7-mode
float ctrl_channels[NUM_CTRL_CHANNELS] = { 0 };

float alt_real;
float yaw_real;
float pitch_real;
float roll_real;

float qw, qx, qy, qz;
float lin_acc_x, lin_acc_y, lin_acc_z;

float yaw_est;
float pitch_est;
float roll_est;
float yaw_used_prev;
float pitch_used_prev;
float roll_used_prev;
float yaw_rate;
float roll_rate;
float pitch_rate;

float engine_0_cmd_norm;
float engine_1_cmd_norm;
float engine_2_cmd_norm;
float engine_3_cmd_norm;

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
const float get_x_body(const float yaw_rad, const float x_world, const float y_world);
const float get_y_body(const float yaw_rad, const float x_world, const float y_world);
int round_down_to_multiple(int number, int multiple);

#ifdef MCU
extern "C" void process_packet_from_usb(uint8_t* Buf, int len);
extern "C" void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size);
extern "C" void SystemClock_Config(void);
#endif // MCU

/**
 * ==== END FUNCTION DECLARATIONS ====
 */


#ifdef MCU

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
    ctrl_channels[current_channel] = (difference - 1000) / 1000.0f;
    current_channel++;
}

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

    current_channel = -1;
    last_captured_value = -1;

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
        
        time_sec = HAL_GetTick() / 1000.0f;
        ax = himu.acc_x;
        ay = himu.acc_y;
        az = himu.acc_z;
        gx = himu.gyro_x * DEG_TO_RAD;
        gy = himu.gyro_y * DEG_TO_RAD;
        gz = himu.gyro_z * DEG_TO_RAD;
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
        char buf[256];
        sprintf(buf, "%f\r\n", alt_rate_est);
        send_data_over_usb_packets(0x00, (void *)buf, strlen(buf), 1, 64);



        /**
         * Rotate quaternion that's send via telemetry by -90 deg in pitch to correctly visualize
         * the vehicle in 3D
        */
        float ret_x =  0 * qw + -0.7071067811865475 * qz - 0 * qy + 0.7071067811865476 * qx;
        float ret_y = -0 * qz + -0.7071067811865475 * qw + 0 * qx + 0.7071067811865476 * qy;
        float ret_z =  0 * qy - -0.7071067811865475 * qx + 0 * qw + 0.7071067811865476 * qz;
        float ret_w = -0 * qx - -0.7071067811865475 * qy - 0 * qz + 0.7071067811865476 * qw;


        /**
         * Telemetry output
         */
        sprintf(buf, "%f," "%f,%f,%f,%f," "%f,%f,%f," "%f,%f,%f," "%f,%f,%f,%f," "%f\n",
            time_sec,
            ctrl_channels[0],
            ctrl_channels[1],
            ctrl_channels[2],
            ctrl_channels[3],
            himu.acc_x,
            himu.acc_y,
            himu.acc_z,
            yaw_est,
            pitch_est,
            roll_est,
            ret_w,
            ret_x,
            ret_y,
            ret_z,
            0.3);
        send_data_over_usb_packets(0x01, (void *)buf, strlen(buf), 1, 64);

        HAL_Delay(20);

        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }
}


/**
 * Send data up to 64 - HEADER_SIZE - FOOTER_SIZE bytes. Currently can send only one packet.
 * 
 * @param channel_number 0: commands, 1: telemetry, 2: HITL data
 * @param data pointer to the data
 * @param data_size how many bytes of data to send
 * @param item_size what is the size of unseparable unit (sizeof(<datatype>), e.g. sizeof(float))
*/
void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size, uint16_t buffer_size) {
    // HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    
    uint16_t current_offset = 0;
    uint16_t maximum_data_in_packet = round_down_to_multiple(buffer_size - PACKET_HEADER_SIZE - PACKET_FOOTER_SIZE, item_size);

    uint8_t to_send[buffer_size];
    to_send[0] = channel_number;
    to_send[1] = 0x55;

    while (current_offset < data_size) {
        uint16_t remaining_size = data_size - current_offset;
        uint16_t to_send_size = MIN(remaining_size, maximum_data_in_packet);

        to_send[2] = LOBYTE(to_send_size);
        to_send[3] = HIBYTE(to_send_size);
        to_send[4] = LOBYTE(current_offset);
        to_send[5] = HIBYTE(current_offset);

        memcpy(to_send + PACKET_HEADER_SIZE, (uint8_t *)data + current_offset, to_send_size);

        to_send[PACKET_HEADER_SIZE + to_send_size] = '\0';

        while (CDC_Transmit_FS(to_send, PACKET_HEADER_SIZE + to_send_size + PACKET_FOOTER_SIZE) == USBD_BUSY);

        current_offset += to_send_size;
    }
}


void process_packet_from_usb(uint8_t* Buf, int len) {
    uint8_t *current_data = Buf;

    /**
     * There must be at least PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE left in the buffer
     * in order for the packet to be valid
    */
    while (current_data - Buf + PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE <= len) {
        uint8_t channel_number  =  current_data[0];
        uint16_t data_size      = (current_data[3] << 8) | current_data[2];
        uint16_t data_offset    = (current_data[5] << 8) | current_data[4];

        /**
         * Incomplete packet received
         */
        if (current_data - Buf + PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE > len) break;

        /**
         * Invalid packet received (no 0 at the end)
         */
        if (channel_number > 0x02
        || current_data[PACKET_HEADER_SIZE + data_size] != '\0'
        || current_data[1] != 0x55
        || data_size > 64
        || data_offset > 512) {
            current_data++;
            continue;
        }
        


        // TODO add callback function to process data along with the channel
        // TODO move these USB packet functions outside of the project to share it with FDM

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

        current_data += PACKET_HEADER_SIZE + data_size + PACKET_FOOTER_SIZE;
    }
}

#endif // MCU


extern "C" void init(/* json *sim_data */) {

#ifdef SITL
    printf("Hello from FCS\n");
#endif // SITL

    pid_init(throttle_pid,    0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    
    pid_init(yaw_pid,         0.011, 0.02,  0.0013,   0.15, -0.2, 0.2);
    pid_init(yaw_rate_pid,    0.008, 0.0,   0.0005,   0.15, -0.2, 0.2);
    
    pid_init(x_body_pid,      4,     0.1,   6,        0.15, -10,  10);  // Output is used as roll  setpoint
    pid_init(y_body_pid,      4,     0.1,   6,        0.15, -10,  10);  // Output is used as pitch setpoint

    pid_init(roll_pid,        0.002, 0,     0.0005,   0.15, -0.2, 0.2);
    pid_init(pitch_pid,       0.002, 0,     0.0005,   0.15, -0.2, 0.2);

    prev_time_sec = -1;

    alt_est_m = 0.0f;
    alt_rate_est = 0.0f;

    engine_0_cmd_norm = 0.0;
    engine_1_cmd_norm = 0.0;
    engine_2_cmd_norm = 0.0;
    engine_3_cmd_norm = 0.0;

    sensor_fusion.begin(50.0);
}


extern "C" void data_from_jsbsim(float *data) {
    time_sec            = data[0];                  // simulation/sim-time-sec

    x_world_measure_m   = data[1] * DEG_TO_GEO_M;   // ext/longitude-deg
    y_world_measure_m   = data[2] * DEG_TO_GEO_M;   // ext/latitude-deg

    alt_real            = data[3];                  // ext/altitude-m
    yaw_real            = data[4];                  // attitude/psi-deg
    pitch_real          = data[5];                  // attitude/theta-deg
    roll_real           = data[6];                  // attitude/phi-deg

    /**
     * Accelerometer adjusted from JSBSim local frame to expected sensor frame
     */
    ax                  = -data[7];                 // sensor/imu/accelX-g
    ay                  =  data[8];                 // sensor/imu/accelY-g
    az                  = -data[9];                 // sensor/imu/accelZ-g

    gx                  = -data[10];                // sensor/imu/gyroX-rps
    gy                  =  data[11];                // sensor/imu/gyroY-rps
    gz                  = -data[12];                // sensor/imu/gyroZ-rps

    mx                  = data[13];                 // sensor/imu/magX-uT
    my                  = data[14];                 // sensor/imu/magY-uT
    mz                  = data[15];                 // sensor/imu/magZ-uT

    pressure_pa         = data[16];                 // sensor/baro/presStatic-Pa

    #ifdef SITL
    
    ctrl_channels[0]    = data[18];                 // user-control/channel-1
    ctrl_channels[1]    = data[19];                 // user-control/channel-2
    ctrl_channels[2]    = data[20];                 // user-control/channel-3
    ctrl_channels[3]    = data[21];                 // user-control/channel-4
    ctrl_channels[4]    = data[22];                 // user-control/channel-4
    ctrl_channels[5]    = data[23];                 // user-control/channel-4
    ctrl_channels[6]    = data[24];                 // user-control/channel-4
    ctrl_channels[7]    = data[25];                 // user-control/channel-4
    
    #endif

    if (prev_time_sec < 0) prev_time_sec = time_sec;
}

extern "C" void data_to_jsbsim(float *data) {
    data[0] = engine_0_cmd_norm;
    data[1] = engine_1_cmd_norm;
    data[2] = engine_2_cmd_norm;
    data[3] = engine_3_cmd_norm;

    data[4] = alt_est_m;
    data[5] = alt_rate_est;

    data[6] = yaw_est;
    data[7] = pitch_est;
    data[8] = roll_est;

    data[9]  = lin_acc_x;
    data[10] = lin_acc_y;
    data[11] = lin_acc_z;
}

extern "C" void loop(void) {
    const float deltaT = time_sec - prev_time_sec;
    prev_time_sec = time_sec;

    if (deltaT == 0.0f) return;

    /**
     * Estimate attitude
     */
    
    sensor_fusion.updateIMU(-gx * RAD_TO_DEG, gy * RAD_TO_DEG, -gz * RAD_TO_DEG, ax, -ay, az);
    sensor_fusion.getQuaternion(&qw, &qx, &qy, &qz);
    roll_est    = sensor_fusion.getRoll();
    pitch_est   = sensor_fusion.getPitch();
    yaw_est     = sensor_fusion.getYaw() - 180.0;

    Quaterion_t acc   = { 0, ax, ay, az };
    Quaterion_t q     = { qw, qx, qy, qz };
    Quaterion_t q_inv = { qw, -qx, -qy, -qz };
    Quaterion_t acc_rot = multiplyQuat(multiplyQuat(q, acc), q_inv);

    lin_acc_x = acc_rot.x;
    lin_acc_y = acc_rot.y;
    lin_acc_z = acc_rot.z - 1.0f;

    bool use_est = true;
    float yaw_used     = use_est ? yaw_est     : yaw_real;
    float pitch_used   = use_est ? pitch_est   : pitch_real;
    float roll_used    = use_est ? roll_est    : roll_real;


    float new_alt_est_m = (288.15f / -0.0065f) * (powf(pressure_pa / 101325.0f, (-8.314f * -0.0065f) / (9.81f * 0.0289640f)) - 1);    
    if (alt_est_m == 0.0f) alt_est_m = new_alt_est_m;

    alt_rate_est = complementaryFilterUpdates(alt_rate_est,
                                              0.05,
                                              (new_alt_est_m - alt_est_m) / deltaT,     // barometer based altitude rate estimate
                                              lin_acc_z * 9.81f * deltaT);              // accelerometer based altitude rate estimate
    alt_est_m = new_alt_est_m;

    /**
     * ==== Yaw ====
     */
    float yaw_sp = 2;
    // if (time_sec > rest_time + 5) yaw_sp = 80;
    // if (yaw_sp - yaw_used >  180) yaw_used += 360;
    // if (yaw_sp - yaw_used < -180) yaw_used -= 360;


    float yaw_diff = yaw_used - yaw_used_prev;
    if (yaw_diff < -180) yaw_diff += 360.0f;
    if (yaw_diff > 180)  yaw_diff -= 360.0f;

    yaw_rate   = yaw_diff / deltaT;
    roll_rate  = (roll_used  - roll_used_prev)  / deltaT;
    pitch_rate = (pitch_used - pitch_used_prev) / deltaT;

    yaw_used_prev = yaw_used;
    roll_used_prev = roll_used;
    pitch_used_prev = pitch_used;

    /**
     * ==== Altitude ====
     */
    float alt_sp = 0.0;
    if (time_sec > rest_time) alt_sp = 1.0;

    /**
     * Position: world_x -> east -> longitude
     *           world_y -> north -> latitude
    */
    float x_world_sp_m = 0;
    float y_world_sp_m = 0;

    if (time_sec > rest_time + 10) {
        x_world_sp_m = 1;
        y_world_sp_m = 2.5;
    }

    /**
     * ==== PIDs ====
     */
    
    float throttle_pid_out = 0.0;

    if (vertical_mode == VERTICAL_MODE_SP) {
        throttle_pid_out = 0.255 + pid_update(throttle_pid, alt_sp, alt_real, deltaT);
    } else if (vertical_mode == VERTICAL_MODE_RATE) {
        float target_alt_rate = 0.0f;
        if (ctrl_channels[2] < 0.4f) target_alt_rate = (ctrl_channels[2] - 0.4f) * 20.0f;
        if (ctrl_channels[2] > 0.6f) target_alt_rate = (ctrl_channels[2] - 0.6f) * 20.0f;
        
        throttle_pid_out = 0.255 + pid_update(throttle_pid, target_alt_rate, alt_rate_est, deltaT);
    } else if (vertical_mode == VERTICAL_MODE_DIRECT) {
        throttle_pid_out = ctrl_channels[2];
    }

    
    float yaw_pid_out = 0.0;

    if (heading_mode == HEADING_MODE_SP) {
        yaw_pid_out = pid_update(yaw_pid, yaw_sp, yaw_used, deltaT);
    } else if (heading_mode == HEADING_MODE_RATE) {
        yaw_pid_out = pid_update(yaw_rate_pid, (ctrl_channels[3] - 0.5f) * 100.0f, yaw_rate, deltaT);
    }


    float roll_pid_out = 0.0;
    float pitch_pid_out = 0.0;

    if (lateral_mode == LATERAL_MODE_FIXED_POS) {
        const float x_body_measure_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
        const float y_body_measure_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
        const float x_body_sp_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);
        const float y_body_sp_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);    
        
        const float x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
        const float y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);
    
        roll_pid_out  = pid_update(roll_pid,   x_body_pid_out, roll_used,  deltaT);
        pitch_pid_out = pid_update(pitch_pid, -y_body_pid_out, pitch_used, deltaT);
    } else if (lateral_mode == LATERAL_MODE_RATE) {
        // TODO but not yet needed
    } else if (lateral_mode == LATERAL_MODE_ANGLE) {
        roll_pid_out  = pid_update(roll_pid,   (ctrl_channels[0] - 0.5) * 40.0f, roll_used,  deltaT);
        pitch_pid_out = pid_update(pitch_pid, -(ctrl_channels[1] - 0.5) * 40.0f, pitch_used, deltaT);
    }


    /**
     * ==== Output ====
     */

    engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, throttle_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, throttle_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, throttle_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, throttle_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));

}

/**
 * @param engine_id Engine id from 0 to 3
 */
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd) {
    if (engine_id == 0) {
        return throttle_cmd - yaw_cmd - pitch_cmd - roll_cmd;
    }

    if (engine_id == 1) {
        return throttle_cmd + yaw_cmd + pitch_cmd - roll_cmd;
    }

    if (engine_id == 2) {
        return throttle_cmd + yaw_cmd - pitch_cmd + roll_cmd;
    }

    if (engine_id == 3) {
        return throttle_cmd - yaw_cmd + pitch_cmd + roll_cmd;
    }

    return 0;
}

const float mixer_to_cmd(const float mixer_output) {
    return MAX(0.0f, MIN(1.0f, mixer_output));
}

const float get_x_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * cos(-yaw_rad) + y_world * sin(-yaw_rad);
}

const float get_y_body(const float yaw_rad, const float x_world, const float y_world) {
    return x_world * -sin(-yaw_rad) + y_world * cos(-yaw_rad);
}

int round_down_to_multiple(int number, int multiple) {
    return number - number % multiple;
}
