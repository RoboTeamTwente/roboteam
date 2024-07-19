#include "filters/vision/DetectionFrame.h"
DetectionFrame::DetectionFrame(const proto::SSL_DetectionFrame& protoFrame)
    : cameraID(protoFrame.camera_id()), timeCaptured{protoFrame.t_capture()}, timeSent{protoFrame.t_sent()} {
    for (const auto& ball : protoFrame.balls()) {
        // If you only want to  use one side, uncomment this
        // Might be nice to move to inputs
        // if (ball.x() < 0) {
        //     continue;
        // }
        balls.emplace_back(BallObservation(cameraID, timeCaptured, timeSent, ball));
    }
    for (const auto& blueBot : protoFrame.robots_blue()) {
        // if (blueBot.x() < 0) {
        //     continue;
        // }
        blue.emplace_back(RobotObservation(cameraID, timeCaptured, timeSent, TeamColor::BLUE, blueBot));
    }
    for (const auto& yellowBot : protoFrame.robots_yellow()) {
        // if (yellowBot.x() < 0) {
        //     continue;
        // }
        yellow.emplace_back(RobotObservation(cameraID, timeCaptured, timeSent, TeamColor::YELLOW, yellowBot));
    }
}