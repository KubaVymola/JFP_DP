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
