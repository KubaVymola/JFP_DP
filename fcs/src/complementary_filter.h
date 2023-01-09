#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

float complementaryFilterUpdates(float prev_est, float alpha, float low_pass_mesurement, float high_pass_messurement) {
    return           
                  alpha  * low_pass_mesurement
        + (1.0f - alpha) * (prev_est + high_pass_messurement);
}

#endif
