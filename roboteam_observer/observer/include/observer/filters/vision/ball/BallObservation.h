#ifndef RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_
#define RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_

#include <proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_utils/Time.h>

#include <Eigen/Dense>

struct BallObservation {
    explicit BallObservation(int cameraID, Time timeCaptured, Time timeSent, const proto::SSL_DetectionBall& detectionBall);
    explicit BallObservation(int cameraID, Time timeCaptured, Time timeSent, Eigen::Vector2d position, Eigen::Vector2d pixelPosition, double confidence, uint32_t totalArea,
                             double height);
    explicit BallObservation()
        : cameraID(-1), timeCaptured(Time()), timeSent(Time()), position(Eigen::Vector2d::Zero()), pixelPosition(Eigen::Vector2d::Zero()), area(0), confidence(0.0), height(0.0) {}

    int cameraID;
    Time timeCaptured;
    Time timeSent;
    Eigen::Vector2d position;
    Eigen::Vector2d pixelPosition;
    uint32_t area;
    double confidence;  // practically useless?
    double height;      // practically useless, is always the same but we want to keep this
};

#endif  // RTT_ROBOTEAM_OBSERVER_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_
