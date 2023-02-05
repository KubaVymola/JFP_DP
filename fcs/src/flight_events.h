#ifndef FLIGHTEVENTS_H
#define FLIGHTEVENTS_H

bool detect_takeoff();
bool detect_landing();
bool detect_arm();
bool detect_disarm();
void disable_pid_integrators();
void enable_pid_integrators();


#endif
