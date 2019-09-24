//
// Created by Lukas Bos on 24/09/2019.
//

#ifndef RTT_TIMER_H
#define RTT_TIMER_H

#include <chrono>
#include <functional>

namespace roboteam_utils {

class Timer {
 private:
  bool running = true;
  std::chrono::milliseconds lastTickedTime;
 public:
  explicit Timer();

  /*
  * Starts a loop in the current thread at a given rate (Hz) for a given function
  */
  void loop(std::function<void(void)> func, int rate);

  /*
   * Limit a function to be called at a maximum specified
   */
  void limit(std::function<void(void)> func, int rate);

  /*
  * Get the current time in milliseconds
  */
  std::chrono::milliseconds getCurrentTime();

  /*
   * Stop the timer and break out of the loop
   */
  void stop();
};

} // roboteam_utils

#endif //RTT_TIMER_H
