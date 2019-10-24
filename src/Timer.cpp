//
// Created by Lukas Bos on 24/09/2019.
//

#include "Timer.h"
#include <thread>
#include <Timer.h>

namespace roboteam_utils {

Timer::Timer() = default;

/// Starts a loop in the current thread at a given rate (Hz) for a given function
void Timer::loop(std::function<void(void)> func, int rate) {
    auto timeStep = std::chrono::microseconds(static_cast<int>((1000.0 / rate) * 1000));
    auto lastTickedLoopTime = getCurrentTime();

    while (running) {
        auto now = getCurrentTime();
        auto timeDiff = now - lastTickedLoopTime;
        if (timeDiff >= timeStep) {
            lastTickedLoopTime = now;

            lastTickedTimeIteration = 0; // this is used for limiting functions

            func();
        } else {
            std::this_thread::sleep_for((timeStep - timeDiff));
        }
    }
}

/// Returns the current time in milliseconds
std::chrono::milliseconds Timer::getCurrentTime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
    );
}

/// Limit a function to be called at a maximum specified rate
/// it is not exact, since it is dependent on the loop rate when this function is called
void Timer::limit(std::function<void(void)> func, int rate) {
    auto timeStep = std::chrono::microseconds(static_cast<int>((1000.0 / rate) * 1000));
    auto now = getCurrentTime();
    auto timeDiff = now - lastTickedTime[lastTickedTimeIteration];
    if (timeDiff >= timeStep) {
        lastTickedTime[lastTickedTimeIteration] = now;
        func();
    }
    lastTickedTimeIteration++;
}

/// Execute the function and return it's duration
std::chrono::milliseconds Timer::measure(std::function<void(void)> func) {
    auto timeBefore = getCurrentTime();
    func();
    return getCurrentTime() - timeBefore;
}

/// stop the timer
void Timer::stop() {
    running = false;
}


}