//==============================================================================
// timer.cpp
//==============================================================================
//
// Source code of the fdm program (JSBSim wrapper) developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

#include "timer.h"

#include <iostream>

Timer::Timer() {
    begin = steady_clock::now();
}

Timer::~Timer() {
    steady_clock::time_point end = steady_clock::now();

    std::cout << "Elapsed: " << duration_cast<microseconds>(end - begin).count() << "[us]" << std::endl;
}
