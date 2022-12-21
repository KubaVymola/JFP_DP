#include "timer.h"

#include <iostream>

Timer::Timer() {
    begin = steady_clock::now();
}

Timer::~Timer() {
    steady_clock::time_point end = steady_clock::now();

    std::cout << "Elapsed: " << duration_cast<microseconds>(end - begin).count() << "[us]" << std::endl;
}
