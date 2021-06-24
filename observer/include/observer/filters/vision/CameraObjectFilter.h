//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_

#include <roboteam_utils/Time.h>

/**
 * @author
 * @class Base class for a single object which is tracked on a single camera.
 * It keeps track of some basic important statistics, and defines the concept of 'health' which indicates how reliable a
 * tracked object. Classes deriving from this class should call the objectSeen() and objectInvisible() methods to update
 * this information appropriately.
 */
class CameraObjectFilter {
 public:
  /**
   * Initializes the camera This can only be done if an object is actually seen
   * This automatically calculates the slope so that the object goes from full health to unhealthy in fullHealthyToUnhealthyTime,
   * given the tickRate number of ticks it should take to reach full health and the number of ticks after which the robot is declared healthy
   * @param time the time the object was seen at
   */
  explicit CameraObjectFilter(double fullHealthToUnhealthyTime, double frame_interval, double fullHealthTicks, double isHealthyAfter,Time time);
  [[nodiscard]] Time lastSeen() const;
  [[nodiscard]] std::size_t numObservations() const;
  [[nodiscard]] double getHealth() const;
  [[nodiscard]] bool isHealthy() const;
  [[nodiscard]] std::size_t consecutiveFramesNotSeen() const;
 protected:
  void objectSeen(const Time& time);
  void objectInvisible(const Time& time);
 private:
  std::size_t framesTotal;
  std::size_t framesNotSeenFor;
  Time lastSeenTime;
  Time lastUpdateTime;
  double health;

  //TODO below 4 could be moved out of class using templates perhaps; effectively constant, but different for ball and robots
  //see calculation (in constructor, now)
  double INCREMENT;
  double DECREMENT_SLOPE;
  double MAXIMUM;
  double HEALTHY_LIMIT;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_
