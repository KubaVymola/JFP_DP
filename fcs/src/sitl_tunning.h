#ifndef SITLTUNNING_H
#define SITLTUNNING_H

#ifdef SITL

#include <map>
#include <string>

/**
 * Function used by fdm's SITLInterface to pass used config from --sitl_config CLI option to FCS,
 * Is used to pass PID coefficients during automatic tuning.
*/
extern "C" void init_override(std::map<std::string, float> &config);

/**
 * Function that gets called each iteration of control_loop and it sets a target setpoints for
 * altitude, yaw, pitch, roll, etc. at given times to perform the output function used by the
 * autotune script
*/
void tunning_config();

#endif // SITL

#endif
