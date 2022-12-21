// either MCU, MCU + HITL, or SITL will be defined
// #define MCU
// #define HITL
// #define SITL

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
#include "gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif // MCU

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


char cmd_string[256];

#ifdef HITL
float from_jsbsim[22] = { 0 };
float to_jsbsim[7] = { 0 };
#endif // HITL

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

float yaw_est;
float pitch_est;
float roll_est;

float engine_0_cmd_norm;
float engine_1_cmd_norm;
float engine_2_cmd_norm;
float engine_3_cmd_norm;




/**
 * Function declarations
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
extern "C" void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size);
#endif // MCU

#ifdef MCU

int main(void) {
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USB_DEVICE_Init();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

    init();

    while (1) {
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

        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        
    #ifdef HITL
        __disable_irq();
        data_from_jsbsim(from_jsbsim);
        __enable_irq();
    #endif // HITL

        loop();

        data_to_jsbsim(to_jsbsim);
        send_data_over_usb_packets(0x02, to_jsbsim, sizeof(to_jsbsim), sizeof(float));

        char buf[128];
        sprintf(buf, "ax %d, ay %d, az %d, gx %d, gy %d, gz %d\n",
            (int)(ax * 1000),
            (int)(ay * 1000),
            (int)(az * 1000),
            (int)(gx * RAD_TO_DEG * 1000),
            (int)(gy * RAD_TO_DEG * 1000),
            (int)(gz * RAD_TO_DEG * 1000));
        send_data_over_usb_packets(0x00, (void *)buf, strlen(buf), 1);

        HAL_Delay(20);
    }
}


/**
 * Send data up to 64 - HEADER_SIZE - FOOTER_SIZE bytes. Currently can send only one packet.
 * TODO allow longer data to be sent via splitting the data into n-byte packets (n will be parameter)
 * 
 * @param channel_number 0: commands, 1: telemetry, 2: HITL data
 * @param data pointer to the data
 * @param data_size how many bytes of data to send
 * @param item_size what is the size of unseparable unit (sizeof(<datatype>), e.g. sizeof(float))
*/
void send_data_over_usb_packets(uint8_t channel_number, void *data, uint16_t data_size, uint8_t item_size) {
    // HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    
    uint16_t current_offset = 0;
    uint16_t maximum_data_in_packet = round_down_to_multiple(64 - PACKET_HEADER_SIZE - PACKET_FOOTER_SIZE, item_size);

    uint8_t to_send[64];
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
#endif

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
    if (time_sec > rest_time + 5) yaw_sp = 20;
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
 * @param engine_id Engine id from 1 to 4
 */
const float engine_mixer(const int engine_id,
                         const float throttle_cmd,
                         const float yaw_cmd,
                         const float pitch_cmd,
                         const float roll_cmd) {
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
