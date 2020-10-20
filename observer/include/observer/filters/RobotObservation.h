//
// Created by rolf on 21-01-20.
//

#ifndef RTT_ROBOTOBSERVATION_H
#define RTT_ROBOTOBSERVATION_H
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
/**
 * A struct to keep robotData and time as one observation. We can pass this around to various classes easily.
 */
struct RobotObservation {
    explicit RobotObservation(int cameraID, double time, proto::SSL_DetectionRobot detectionRobot) : cameraID(cameraID), time(time), bot(std::move(detectionRobot)) {}
    int cameraID;
    double time;
    proto::SSL_DetectionRobot bot;
};
#endif  // RTT_ROBOTOBSERVATION_H
