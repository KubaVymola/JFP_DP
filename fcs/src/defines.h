#ifndef DEFINES_H
#define DEFINES_H

// #define DEMO_SEQ                        1

#ifdef AUTOARM
#define AUTO_ARM                         true
#else
#define AUTO_ARM                         false
#endif

#define LOOP_FREQUENCY                   400.0f
#define LOG_FREQUENCY                    50.0f

#define HEADING_MODE_SP                  1 /* Heading is held at a fixed value */
#define HEADING_MODE_RATE                2 /* Heading rate is held at a fixed value */
#define HEADING_MODE_DYNAMIC             3 /* Combination of SP (in deadband) and rate */
#define HEADING_MODE_NONE                4 /* Heading mode used for tunning */

#define VERTICAL_MODE_SP                 1 /* Altitude is held at a fixed value */
#define VERTICAL_MODE_RATE               2 /* Altitude is controlled with rate of change */
#define VERTICAL_MODE_DIRECT             3 /* Direct mapping of throttle pos to the engines */

#define LATERAL_MODE_FIXED_POS           1 /* Position is held at a fixed value */
#define LATERAL_MODE_ANGLE_RATE          2 /* Pitch and roll controls the angular velocity of the craft (freestyle mode) */
#define LATERAL_MODE_ANGLE               3 /* Pitch and roll controls the angle of the craft */

#define CALIBRATION_TIME_S               2.0f

#define MAX_VERICAL_RATE                 2.0f  /* M/S */
#define MAX_YAW_RATE                     100.0f /* DEG/S */
#define MAX_ANGLE_RATE                   20.0f  /* DEG/S */
#define MAX_ANGLE                        30.0f  /* DEG */

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

#define HEADING_DYNAMIC_DEADBAND         0.1f

#define NUM_CTRL_CHANNELS                8

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

#ifndef M_PI
#define M_PI		                     3.14159265358979323846f
#endif // M_PI

#define DEG_TO_GEO_M                     (6378000.0f * M_PI / 180.0f)
#define GEO_TO_DEG_M                     (180.0f / (6378000.0f * M_PI))
#define DEG_TO_RAD                       (M_PI / 180.0f)
#define RAD_TO_DEG                       (180.0f / M_PI)
#define G_TO_MPS                         9.81f
#define MPS_TO_G                         (1.0f / 9.81f)

#define NO_ALTITUDE                      -1000000

#endif // DEFINES_H