#include "demo_sequence.h"

#include "global_variables.h"

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

