//
// Created by rolf on 05-11-19.
//

#ifndef RTT_ROBOTFILTER_H
#define RTT_ROBOTFILTER_H

#include "CameraRobotFilter.h"
#include "observer/filters/vision/robot/RobotObservation.h"
class RobotFilter  {
 public:
  explicit RobotFilter(const RobotObservation &observation);
  /**
   * Optionally processes the given detection, returns true if the detection is accepted by one of the camera filters
   * @param observation
   * @return true if the observation is accepted
   */
  bool processDetection(const RobotObservation& observation);
  /**
   * Predicts the robot on the given cameraID (if a filter for this camera exists)
   * @param cameraID the id of the camera
   * @param untilTime time to predict to
   */
  void predictCam(const int& cameraID, const Time& untilTime);

  /**
   * Checks if a camera has not seen, and if so, processes the fact that an object was not seen by a camera frame taken at time
   * @param cameraID
   * @param untilTime
   * @return true if this filter can be removed (e.g. is empty), false otherwise
   */
  bool processNotSeen(int cameraID, const Time& time);

  [[nodiscard]] double getHealth() const;
  /**
   * @brief Merges robot estimates from different cameras into a single robot estimate
   * @param time to estimate the robot position at
   * @return the estimated robot position
   */
  [[nodiscard]] FilteredRobot mergeRobots(const Time& time) const;
  /**
   * @param cameraID
   * @param time
   * @return the estimate of the robot on the given camera at time t, if a filter which tracks the robot on that camera exists
   */
  [[nodiscard]] std::optional<FilteredRobot> getRobot(int cameraID, Time time) const;
  //std::optional<RobotTrajectorySegment> getLastFrameTrajectory(int cameraID, const RobotParameters& parameters) const;
 private:
  TeamRobotID id;
  std::map<int,CameraRobotFilter> cameraFilters;
};

#endif  // RTT_ROBOTFILTER_H
