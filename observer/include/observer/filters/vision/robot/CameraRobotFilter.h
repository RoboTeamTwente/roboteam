//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_

#include "filters/vision/CameraObjectFilter.h"
#include "filters/vision/PosVelFilter2D.h"
#include "RobotOrientationFilter.h"
class CameraRobotFilter : public CameraObjectFilter{
 public:
  CameraRobotFilter(const RobotObservation& observation, RobotVel velocityEstimate);


  void predict(Time time);

  void update(const RobotObservation& observation);

  bool updateRobotNotSeen(const Time& time);

  FilteredRobot estimate(const Time& time) const;

  RobotVel velocityEstimate(const Time& time) const;
  RobotTrajectory getLastFrameTrajectory() const;

 private:
  PosVelFilter2D positionFilter;
  RobotOrientationFilter angleFilter;
  bool justUpdated; //keeps track if the filter was just updated e.g. a observation was given to it
                    // so that we can efficiently find filters which were not updated
  TeamRobotID robot;
  int cameraID;

  RobotPos previousPosition;
  Time previousTime;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_
