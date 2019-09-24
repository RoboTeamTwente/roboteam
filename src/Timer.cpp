//
// Created by Lukas Bos on 24/09/2019.
//

#include "Timer.h"
namespace roboteam_utils {

/// Starts a loop in the current thread at a given rate (Hz) for a given function
void Timer::loop(std::function<void(void)> func, int rate) {
  auto timeStep = std::chrono::microseconds(static_cast<int>((1000.0/rate)*1000));
  auto lastTickedTime = getCurrentTime();

  while (true) {
    auto timeDiff = getCurrentTime() - lastTickedTime;
    if (timeDiff >= timeStep) {
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

}