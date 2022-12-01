//
// Created by rolf on 23-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_

#include "observer/filters/vision/CameraObjectFilter.h"
#include "observer/filters/vision/PosVelFilter2D.h"
#include "RobotOrientationFilter.h"
#include "RobotObservation.h"
#include "RobotPos.h"
#include "FilteredRobot.h"

class CameraRobotFilter : public CameraObjectFilter{
 public:
  CameraRobotFilter(const RobotObservation& observation, RobotVel velocityEstimate = RobotVel());


  void predict(Time time);

  void update(const RobotObservation& observation);

  bool updateRobotNotSeen(const Time& time);

  [[nodiscard]] FilteredRobot estimate(const Time& time) const;

  [[nodiscard]] RobotVel velocityEstimate(const Time& time) const;

  [[nodiscard]] bool acceptObservation(const RobotObservation& observation) const;

  [[nodiscard]] bool justUpdated() const;

  //RobotTrajectory getLastFrameTrajectory() const; //TODO fix

 private:

  void updatePreviousInfo();
  PosVelFilter2D positionFilter;
  RobotOrientationFilter angleFilter;
  bool just_updated; //keeps track if the filter was just updated e.g. a observation was given to it
                    // so that we can efficiently find filters which were not updated
  TeamRobotID robot;
  int cameraID;

  RobotPos previousPosition;
  Time previousTime;
};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_CAMERAROBOTFILTER_H_
