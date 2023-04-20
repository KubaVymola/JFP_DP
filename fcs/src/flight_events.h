//==============================================================================
// flight_events.h
//==============================================================================
//
// Source code of the Flight controller software developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef FLIGHTEVENTS_H
#define FLIGHTEVENTS_H

bool detect_takeoff();
bool detect_landing();
bool detect_arm();
bool detect_disarm();
void disable_pid_integrators();
void enable_pid_integrators();


#endif
