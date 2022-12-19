//
// Created by rolf on 24-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTOBSERVATION_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTOBSERVATION_H_

#include <proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_utils/Time.h>
#include <Eigen/Dense>
#include "RobotPos.h"
struct RobotObservation {
  RobotObservation(int cameraID, Time timeCaptured, Time timeSent, TeamColor color, const proto::SSL_DetectionRobot& detection) :
  cameraID{cameraID},
  timeCaptured{timeCaptured},
  timeSent{timeSent},
  robot{TeamRobotID(detection.robot_id(),color)},
  position(detection.x()/1000.0,detection.y()/1000.0),
  pixelPosition(detection.pixel_x(),detection.pixel_y()),
  orientation{detection.orientation()},
  confidence{detection.confidence()},
  height{detection.height()}{

  }
  int cameraID;
  Time timeCaptured;
  Time timeSent;
  TeamRobotID robot;
  Eigen::Vector2d position;
  Eigen::Vector2d pixelPosition;
  double orientation;
  double confidence;
  double height;

};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOT_ROBOTOBSERVATION_H_
