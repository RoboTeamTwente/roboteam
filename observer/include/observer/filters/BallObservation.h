//
// Created by rolf on 21-01-20.
//

#ifndef RTT_BALLOBSERVATION_H
#define RTT_BALLOBSERVATION_H
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
/**
 * A struct to keep Ball Data and time as one observation.
 */
struct BallObservation {
    explicit BallObservation(int cameraID, double time, proto::SSL_DetectionBall detectionBall) : cameraID(cameraID), time(time), ball(std::move(detectionBall)) {}
    int cameraID;
    double time;
    proto::SSL_DetectionBall ball;
};
#endif  // RTT_BALLOBSERVATION_H
