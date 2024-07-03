#ifndef KICKEVENT_H
#define KICKEVENT_H

#include "observer/filters/vision/CameraMap.h"
#include "observer/filters/vision/DetectionFrame.h"
#include "observer/filters/vision/GeometryFilter.h"
#include "observer/filters/vision/RobotFeedbackFilter.h"
#include "observer/filters/vision/ball/BallFilter.h"
#include "observer/filters/vision/robot/RobotFilter.h"
#include "observer/parameters/RobotParameterDatabase.h"

struct KickEvent {
    TeamRobotID kickingBot;
    RobotPos kickingBotPos;
    Eigen::Vector2d ballPosition;
    Time time;
    std::vector<FilteredBall> ballsSinceKick;
};

#endif  // KICKEVENT_H