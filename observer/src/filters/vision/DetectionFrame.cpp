//
// Created by rolf on 29-06-21.
//

#include "filters/vision/DetectionFrame.h"
DetectionFrame::DetectionFrame(const proto::SSL_DetectionFrame &protoFrame) :
    cameraID(protoFrame.camera_id()),
    timeCaptured{protoFrame.t_capture()},
    timeSent{protoFrame.t_sent()}
{
  for(const auto& ball : protoFrame.balls()){
    balls.emplace_back(BallObservation(cameraID,timeCaptured,timeSent,ball));
  }
  for(const auto& blueBot : protoFrame.robots_blue()){
    blue.emplace_back(RobotObservation(cameraID,timeCaptured,timeSent,TeamColor::BLUE,blueBot));
  }
  for(const auto& yellowBot : protoFrame.robots_yellow()){
    yellow.emplace_back(RobotObservation(cameraID,timeCaptured,timeSent,TeamColor::YELLOW,yellowBot));
  }
}