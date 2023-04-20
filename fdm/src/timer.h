//==============================================================================
// timer.h
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#ifndef TIMER_H
#define TIMER_H

#include <chrono>

using namespace std::chrono;

/**
 * A simple class to be allocated on a stack in a given block. Upon exiting of the block, when the
 * class gets deallocated from the stack, the time interval between allocation and deallocation
 * is printed in micoseconds. Used to benchmark things, especially the USB communication.
*/
class Timer {
public:
    Timer();
    ~Timer();
private:
    steady_clock::time_point begin;
};

#endif
