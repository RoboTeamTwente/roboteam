//
// Created by rolf on 29-06-21.
//

#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_DETECTIONFRAME_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_DETECTIONFRAME_H_
#include <proto/messages_robocup_ssl_detection.pb.h>

#include "ball/BallObservation.h"
#include "robot/RobotObservation.h"
struct DetectionFrame {
    explicit DetectionFrame(const proto::SSL_DetectionFrame& protoFrame);
    int cameraID;
    Time timeCaptured;
    Time timeSent;
    double dt = 0.0;  // needs to be set by the world filter to determine the time difference.
    std::vector<BallObservation> balls;
    std::vector<RobotObservation> blue;
    std::vector<RobotObservation> yellow;
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_DETECTIONFRAME_H_
