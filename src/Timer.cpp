//
// Created by Lukas Bos on 24/09/2019.
//

#include "Timer.h"
#include <thread>
#include <Timer.h>

namespace roboteam_utils {

Timer::Timer() {
  lastTickedTime = getCurrentTime();
}

/// Starts a loop in the current thread at a given rate (Hz) for a given function
void Timer::loop(std::function<void(void)> func, int rate) {
  auto timeStep = std::chrono::microseconds(static_cast<int>((1000.0/rate)*1000));
  lastTickedTime = getCurrentTime();

  while (running) {
    auto now = getCurrentTime();
    auto timeDiff = now - lastTickedTime;
    if (timeDiff >= timeStep) {
      lastTickedTime = now;
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


void Timer::limit(std::function<void(void)> func, int rate) {
  auto timeStep = std::chrono::microseconds(static_cast<int>((1000.0/rate)*1000));
  auto now = getCurrentTime();
  auto timeDiff = now - lastTickedTime;
  if (timeDiff >= timeStep) {
    lastTickedTime = now;
    func();
  }
}

/// stop the timer
void Timer::stop() {
  running = false;
}


}