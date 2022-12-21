#ifndef TIMER_H
#define TIMER_H

#include <chrono>

using namespace std::chrono;

class Timer {
public:
    Timer();
    ~Timer();
private:
    steady_clock::time_point begin;
};

#endif
