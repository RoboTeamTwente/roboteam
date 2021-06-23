//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_

/**
 * @author
 * @class Base class for a single object which is tracked on a single camera.
 * It keeps track of some basic important statistics, and defines the concept of 'health' which indicates how reliable a
 * tracked object. Classes deriving from this class should call the objectSeen() and objectInvisible() methods to update
 * this information appropriately.
 */
class CameraObjectFilter {
 public:
  [[nodiscard]] Time lastSeen() const;
  [[nodiscard]] std::size_t numObservations() const;
  [[nodiscard]] double getHealth() const;
  bool isHealthy() const;
  std::size_t consecutiveFramesNotSeen() const;
 protected:
  void objectSeen(const Time& time);
  void objectInvisible(const Time& time);
 private:
  std::size_t framesTotal;
  std::size_t framesNotSeenFor;
  Time lastSeenTime;
  Time lastUpdateTime;
  double health;

  //below 4 could be moved out of class using templates perhaps; effectively constant
  double INCREMENT;
  double DECREMENT_SLOPE;
  double MAXIMUM;
  double HEALTHY_LIMIT;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAOBJECTFILTER_H_
