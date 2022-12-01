//
// Created by rolf on 23-06-21.
//

#include "filters/vision/CameraObjectFilter.h"
#include <cassert>
#include <algorithm>
#include <cmath>

Time CameraObjectFilter::lastSeen() const {
  return lastSeenTime;
}
std::size_t CameraObjectFilter::numObservations() const {
  return framesTotal;
}
double CameraObjectFilter::getHealth() const {
  return health;
}
bool CameraObjectFilter::isHealthy() const {
  return health >= HEALTHY_LIMIT;
}
std::size_t CameraObjectFilter::consecutiveFramesNotSeen() const {
  return framesNotSeenFor;
}
void CameraObjectFilter::objectSeen(const Time &time) {
  assert(time >= lastUpdateTime); //we can only update objects into the future
  double newHealth = health + INCREMENT - (time-lastUpdateTime).asSeconds() * DECREMENT_SLOPE;
  health = std::clamp(newHealth,0.0,MAXIMUM);
  framesTotal++;
  lastSeenTime = time;
  lastUpdateTime = time;
  framesNotSeenFor = 0;

}
void CameraObjectFilter::objectInvisible(const Time &time) {
  assert(time > lastSeenTime); // we cannot both mark an object invisible and invisible at the same time
  assert(time >= lastUpdateTime);
  health = fmax(health - (time - lastUpdateTime).asSeconds() * DECREMENT_SLOPE,0);
  lastUpdateTime = time;
  framesNotSeenFor++;
}
CameraObjectFilter::CameraObjectFilter(double fullHealthToUnhealthyTime, double tickRate, double fullHealthTicks, double isHealthyAfter,
Time time) :
    framesTotal{1},
    framesNotSeenFor{0},
    lastSeenTime{time},
    lastUpdateTime{time}
{
  MAXIMUM = 100.0;
  HEALTHY_LIMIT = MAXIMUM * isHealthyAfter / fullHealthTicks;
  DECREMENT_SLOPE = MAXIMUM * (1-isHealthyAfter/ fullHealthTicks) / fullHealthToUnhealthyTime;
  INCREMENT = MAXIMUM/fullHealthTicks + tickRate * (MAXIMUM*(1- isHealthyAfter/fullHealthTicks)) /fullHealthToUnhealthyTime;
  health = INCREMENT;

}
