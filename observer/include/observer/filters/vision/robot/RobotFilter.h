//
// Created by rolf on 05-11-19.
//

#ifndef RTT_ROBOTFILTER_H
#define RTT_ROBOTFILTER_H

#include "CameraRobotFilter.h"
#include "filters/vision/robot/RobotObservation.h"
class RobotFilter  {
 public:
  explicit RobotFilter(const RobotObservation &observation);
  bool processDetection(const RobotObservation&observation);
  void predictCam(const int& cameraID, const Time& untilTime);

  /**
   * Checks if a camera has not seen, and if so, processes the fact that an object was not seen by a camera frame taken at time
   * @param cameraID
   * @param untilTime
   * @return true if this filter can be removed (e.g. is empty), false otherwise
   */
  bool processNotSeen(int cameraID, const Time& time);

  [[nodiscard]] double getHealth() const;
  [[nodiscard]] FilteredRobot mergeRobots(const Time& time) const;
  [[nodiscard]] std::optional<FilteredRobot> getRobot(int cameraID, Time time) const;
  //std::optional<RobotTrajectorySegment> getLastFrameTrajectory(int cameraID, const RobotParameters& parameters) const;
 private:
  TeamRobotID id;
  std::map<int,CameraRobotFilter> cameraFilters;
};

#endif  // RTT_ROBOTFILTER_H
