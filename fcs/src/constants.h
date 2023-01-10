#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PACKET_HEADER_SIZE      6 /* 1B (channel) 1B (0x55) 2B (size) 2B (offset) */
#define PACKET_FOOTER_SIZE      1 /* 1B (0x00) */

#define NUM_CTRL_CHANNELS       8

#ifndef M_PI
#define M_PI		            3.14159265358979323846f
#endif // M_PI

#define DEG_TO_GEO_M            (6378000.0f * M_PI / 180.0f)
#define DEG_TO_RAD              (M_PI / 180.0f)
#define RAD_TO_DEG              (180.0f / M_PI)
#define G_TO_MPS                9.81f
#define MPS_TO_G                (1.0f / 9.81f)

#endif
