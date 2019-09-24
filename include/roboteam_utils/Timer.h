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
  std::chrono::milliseconds lastTickedTime[10000] = {std::chrono::milliseconds(0)};
  int lastTickedTimeIteration = 0;
 public:
  explicit Timer();

  /*
  * Starts a loop in the current thread at a given rate (Hz) for a given function
  */
  void loop(std::function<void(void)> func, int rate);

  /*
   * Limit a function to be called at a maximum specified rate
   */
  void limit(std::function<void(void)> func, int rate);

  /*
   * Execute the function and return it's duration
   */
  static std::chrono::milliseconds measure(std::function<void(void)> func);

  /*
  * Get the current time in milliseconds
  */
  static std::chrono::milliseconds getCurrentTime();

  /*
   * Stop the timer and break out of the loop
   */
  void stop();
};

} // roboteam_utils

#endif //RTT_TIMER_H
