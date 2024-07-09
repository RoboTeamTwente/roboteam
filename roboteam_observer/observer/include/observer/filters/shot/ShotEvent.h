#ifndef SHOTEVENT_H
#define SHOTEVENT_H

#include "observer/filters/vision/CameraMap.h"
#include "observer/filters/vision/DetectionFrame.h"
#include "observer/filters/vision/GeometryFilter.h"
#include "observer/filters/vision/RobotFeedbackFilter.h"
#include "observer/filters/vision/ball/BallFilter.h"
#include "observer/filters/vision/robot/RobotFilter.h"
#include "observer/parameters/RobotParameterDatabase.h"

struct ShotEvent {
    TeamRobotID shootingBot;
    RobotPos shottingBotPos;
    Eigen::Vector2d ballPosition;
    Time time;
    std::vector<FilteredBall> ballsSinceShot;
};
struct ShotState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

#endif  // SHOTEVENT_H