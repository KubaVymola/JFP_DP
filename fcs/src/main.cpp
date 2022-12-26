// either INDEP, HITL, or SITL will be defined
// either MCU or CPU will be defined

// // #define MCU
// // #define HITL
// // #define SITL

/**
 * ==== INCLUDES ====
 */

#include "pid.h"
#include "Adafruit_AHRS_Madgwick.h"
#include "Adafruit_AHRS_NXPFusion.h"

#ifdef SITL
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#endif // SITL

#ifdef MCU
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include "bmi160.h"
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

pid_state_t alt_pid;
pid_state_t yaw_pid;
pid_state_t x_body_pid;
pid_state_t y_body_pid;
pid_state_t roll_pid;
pid_state_t pitch_pid;

float time_sec;
float prev_time_sec;
float x_world_measure_m;
float y_world_measure_m;
float ax, ay, az, gx, gy, gz, mx, my, mz;
float ch_1, ch_2, ch_3, ch_4;
float pressure_pa;

float alt_real;
float yaw_real;
float pitch_real;
float roll_real;

float qw, qx, qy, qz;
float yaw_est;
float pitch_est;
float roll_est;

float engine_0_cmd_norm;
float engine_1_cmd_norm;
float engine_2_cmd_norm;
float engine_3_cmd_norm;

#ifdef HITL
float from_jsbsim[22] = { 0 };
float to_jsbsim[7] = { 0 };
#endif // HITL

#ifdef MCU
bmi160 himu;
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

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_USART1_UART_Init();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

    init();

    bmi160_init(&himu, &hi2c1, 0, BMI160_ACC_RATE_50HZ, BMI160_ACC_RANGE_4G, BMI160_GYRO_RATE_50HZ, BMI160_GYRO_RANGE_1000DPS);

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
        
        #ifdef INDEP
        if (bmi160_get_data_rdy(&himu)) {
            bmi160_update_acc_gyro_data(&himu);
        }
        #endif

        //     char bmi_buf[128];

        //     sprintf(bmi_buf, "acc: %d %d %d gyro: %d %d %d\r\n",
        //         (int16_t)himu.acc_raw[0],
        //         (int16_t)himu.acc_raw[1],
        //         (int16_t)himu.acc_raw[2],
        //         (int16_t)himu.gyro_raw[0],
        //         (int16_t)himu.gyro_raw[1],
        //         (int16_t)himu.gyro_raw[2]
        //     );

        //     CDC_Transmit_FS((uint8_t *)bmi_buf, strlen(bmi_buf));

        //     // send_data_over_usb_packets(1, bmi_buf, sizeof(bmi_buf), 1);



        // HAL_Delay(1000);
        // HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        
        // continue;

        /**
         * RC RX test (with SBUS)
         */
        
        // uint8_t receiver_data[25] = { 0 };
        // HAL_UART_Receive(&huart1, receiver_data, 25, 1000);

        // char usb_data[128];
        // sprintf(usb_data, "header: %x %x %x %x %x\r\n", receiver_data[0], receiver_data[1], receiver_data[2], receiver_data[3], receiver_data[4]);
        // CDC_Transmit_FS((uint8_t *)usb_data, strlen(usb_data));

        // continue;

        /**
         * Motor control
         */

        // htim9.Instance->CCR1 = 1100;
        // htim9.Instance->CCR2 = 1100;
        // htim10.Instance->CCR1 = 1100;
        // htim11.Instance->CCR1 = 1100;
        
        // HAL_Delay(2000);

        // htim9.Instance->CCR1 = 1200;
        // htim9.Instance->CCR2 = 1200;
        // htim10.Instance->CCR1 = 1200;
        // htim11.Instance->CCR1 = 1200;

        // HAL_Delay(2000);

        // continue;
        
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
        time_sec = HAL_GetTick() / 1000.0f;
        ax = himu.acc_x;
        ay = himu.acc_y;
        az = himu.acc_z;
        gx = himu.gyro_x * DEG_TO_RAD;
        gy = himu.gyro_y * DEG_TO_RAD;
        gz = himu.gyro_z * DEG_TO_RAD;
        #endif

        htim10.Instance->CCR1 = engine_0_cmd_norm * 1000.0f + 1000.0f;
        htim9.Instance->CCR1 = engine_1_cmd_norm * 1000.0f + 1000.0f;
        htim9.Instance->CCR2 = engine_2_cmd_norm * 1000.0f + 1000.0f;
        htim11.Instance->CCR1 = engine_3_cmd_norm * 1000.0f + 1000.0f;

        loop();

        #ifdef HITL
        data_to_jsbsim(to_jsbsim);
        send_data_over_usb_packets(0x02, to_jsbsim, sizeof(to_jsbsim), sizeof(float), 64);
        #endif // HITL

        /**
         * Simple debug output
         */
        char buf[128];
        // sprintf(buf, "ax %d, ay %d, az %d, gx %d, gy %d, gz %d\n",
        //     (int)(ax * 1000),
        //     (int)(ay * 1000),
        //     (int)(az * 1000),
        //     (int)(gx * RAD_TO_DEG * 1000),
        //     (int)(gy * RAD_TO_DEG * 1000),
        //     (int)(gz * RAD_TO_DEG * 1000));
        // send_data_over_usb_packets(0x00, (void *)buf, strlen(buf), 1, 64);



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
        sprintf(buf, "%f," "%f,%f,%f," "%f,%f,%f," "%f,%f,%f,%f," "%f\n",
            time_sec,
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

        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
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

    pid_init(alt_pid,    0.10,  0.05,  0.08,     0.15, -0.5, 0.5);
    pid_init(yaw_pid,    0.011, 0.02,  0.0013,   0.15, -0.2, 0.2);
    pid_init(x_body_pid, 4,     0.1,   6,        0.15, -10,  10);  // Output is used as roll  setpoint
    pid_init(y_body_pid, 4,     0.1,   6,        0.15, -10,  10);  // Output is used as pitch setpoint
    pid_init(roll_pid,   0.002, 0,     0.0005,   0.15, -0.1, 0.1);
    pid_init(pitch_pid,  0.002, 0,     0.0005,   0.15, -0.1, 0.1);

    prev_time_sec = -1;

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
    ay                  =  data[7];                 // sensor/imu/accelX-g
    ax                  =  data[8];                 // sensor/imu/accelY-g
    az                  = -data[9];                 // sensor/imu/accelZ-g

    gy                  =  data[10];                // sensor/imu/gyroX-rps
    gx                  =  data[11];                // sensor/imu/gyroY-rps
    gz                  = -data[12];                // sensor/imu/gyroZ-rps

    mx                  = data[13];                 // sensor/imu/magX-uT
    my                  = data[14];                 // sensor/imu/magY-uT
    mz                  = data[15];                 // sensor/imu/magZ-uT

    ch_1                = data[18];                 // user-control/channel-1
    ch_2                = data[19];                 // user-control/channel-2
    ch_3                = data[20];                 // user-control/channel-3
    ch_4                = data[21];                 // user-control/channel-4

    if (prev_time_sec < 0) prev_time_sec = time_sec;
}

extern "C" void data_to_jsbsim(float *data) {
    data[0] = engine_0_cmd_norm;
    data[1] = engine_1_cmd_norm;
    data[2] = engine_2_cmd_norm;
    data[3] = engine_3_cmd_norm;

    data[4] = yaw_est;
    data[5] = pitch_est;
    data[6] = roll_est;
}

extern "C" void loop(void) {
    const float deltaT = time_sec - prev_time_sec;
    prev_time_sec = time_sec;

    /**
     * Estimate attitude
     */
    // sensor_fusion.updateIMU(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, ax, ay, az);
    
    sensor_fusion.update(gy * RAD_TO_DEG, gx * RAD_TO_DEG, -gz * RAD_TO_DEG, -ay, -ax, az, 0, 0, 0);
    sensor_fusion.getQuaternion(&qw, &qx, &qy, &qz);
    roll_est    = sensor_fusion.getRoll();
    pitch_est   = sensor_fusion.getPitch();
    yaw_est     = sensor_fusion.getYaw() - 180.0;

    bool use_est = true;
    float yaw_used     = use_est ? yaw_est     : yaw_real;
    float pitch_used   = use_est ? pitch_est   : pitch_real;
    float roll_used    = use_est ? roll_est    : roll_real;

    /**
     * ==== Altitude ====
     */
    float alt_sp = 0.0;
    if (time_sec > rest_time) alt_sp = 1.0;

    /**
     * ==== Yaw ====
     */
    float yaw_sp = 2;
    if (time_sec > rest_time + 5) yaw_sp = 80;
    if (yaw_sp - yaw_used >  180) yaw_used += 360;
    if (yaw_sp - yaw_used < -180) yaw_used -= 360;

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

    const float x_body_measure_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
    const float y_body_measure_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_measure_m, y_world_measure_m);
    const float x_body_sp_m = get_x_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);
    const float y_body_sp_m = get_y_body(yaw_used * DEG_TO_RAD, x_world_sp_m, y_world_sp_m);

    /**
     * ==== PIDs ====
     */
    const float alt_pid_out = 0.255 + pid_update(alt_pid, alt_sp, alt_real, deltaT);
    const float yaw_pid_out = pid_update(yaw_pid, yaw_sp, yaw_used, deltaT);

    const float x_body_pid_out = pid_update(x_body_pid, x_body_sp_m, x_body_measure_m, deltaT);
    const float y_body_pid_out = pid_update(y_body_pid, y_body_sp_m, y_body_measure_m, deltaT);

    const float roll_pid_out  = pid_update(roll_pid,   x_body_pid_out, roll_used,  deltaT);
    const float pitch_pid_out = pid_update(pitch_pid, -y_body_pid_out, pitch_used, deltaT);
    // const float roll_pid_out  = pid_update(roll_pid,   (ch_4 - 0.5) * 20, roll_used,  deltaT);
    // const float pitch_pid_out = pid_update(pitch_pid, -(ch_2 - 0.5) * 20, pitch_used, deltaT);

    /**
     * ==== Output ====
     */
    // engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    // engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    // engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));
    // engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, ch_3 - 0.092, ch_1 - 0.5, pitch_pid_out, roll_pid_out));

    engine_0_cmd_norm = mixer_to_cmd(engine_mixer(0, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_1_cmd_norm = mixer_to_cmd(engine_mixer(1, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_2_cmd_norm = mixer_to_cmd(engine_mixer(2, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));
    engine_3_cmd_norm = mixer_to_cmd(engine_mixer(3, alt_pid_out, yaw_pid_out, pitch_pid_out, roll_pid_out));

    // engine_0_cmd_norm = ch_3 - 0.091 + (ch_1 - 0.5);
    // engine_1_cmd_norm = ch_3 - 0.091 + (ch_1 - 0.5);
    // engine_2_cmd_norm = ch_3 - 0.091 - (ch_1 - 0.5);
    // engine_3_cmd_norm = ch_3 - 0.091 - (ch_1 - 0.5);

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
    return MAX(0.0, MIN(1.0, (const double)mixer_output));
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
