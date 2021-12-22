//
// Created by rolf on 29-06-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_

#include <roboteam_utils/Time.h>
#include <Eigen/Dense>
#include <proto/messages_robocup_ssl_detection.pb.h>

struct BallObservation {
  explicit BallObservation(int cameraID, Time timeCaptured, Time timeSent, const proto::SSL_DetectionBall& detectionBall);
  explicit  BallObservation(int cameraID,Time timeCaptured,
                  Time timeSent,Eigen::Vector2d position,
                  Eigen::Vector2d pixelPosition,
                  double confidence,uint32_t totalArea,double height);

  int cameraID;
  Time timeCaptured;
  Time timeSent;
  Eigen::Vector2d position;
  Eigen::Vector2d pixelPosition;
  uint32_t area;
  double confidence; //practically useless?
  double height; //practically useless, is always the same but we want to keep this

};

#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLOBSERVATION_H_
