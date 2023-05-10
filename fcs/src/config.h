#ifndef CONFIG_H
#define CONFIG_H

#ifdef CONFIG_HOME_QUAD

#define ZERO_RATE_THROTTLE_CONFIG               0.4f
#define MAX_VERICAL_RATE_CONFIG                 3.0f
#define ROLL_CHANNEL_CONFIG                     0
#define PITCH_CHANNEL_CONFIG                    1
#define THROTTLE_CHANNEL_CONFIG                 2
#define YAW_CHANNEL_CONFIG                      3
#define ARM_CHANNEL_CONFIG                      4
#define VERT_MODE_CHANNEL_CONFIG                5


#endif // CONFIG_HOME_QUAD


#ifdef CONFIG_SCHOOL_QUAD

#define ZERO_RATE_THROTTLE_CONFIG               0.4f
#define MAX_VERICAL_RATE_CONFIG                 3.0f
#define YAW_CHANNEL_CONFIG                      0
#define PITCH_CHANNEL_CONFIG                    1
#define THROTTLE_CHANNEL_CONFIG                 2
#define ROLL_CHANNEL_CONFIG                     3
#define ARM_CHANNEL_CONFIG                      4
#define VERT_MODE_CHANNEL_CONFIG                5

#endif // CONFIG_SCHOOL_QUAD

#endif
