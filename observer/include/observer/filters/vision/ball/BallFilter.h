//
// Created by rolf on 17-11-19.
//

#ifndef RTT_BALLFILTER_H
#define RTT_BALLFILTER_H

#include <roboteam_proto/WorldBall.pb.h>
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include "BallObservation.h"
#include "CameraGroundBallFilter.h"
#include "FilteredBall.h"

struct GroundBallPrediction{
  CameraGroundBallPrediction prediction;
  bool hadRequestedCamera;
};
class BallFilter{
 public:
  explicit BallFilter(const BallObservation& observation);

  [[nodiscard]] GroundBallPrediction predictCam(int cameraID, Time until) const;

  bool processDetections(const CameraGroundBallPredictionObservationPair& detections, int cameraID);

  [[nodiscard]] FilteredBall mergeBalls(Time time) const;

  [[nodiscard]] double getHealth() const;
 private:
  std::map<int,CameraGroundBallFilter> groundFilters;
};

#endif  // RTT_BALLFILTER_H
