//
// Created by rolf on 17-11-19.
//

#ifndef RTT_BALLFILTER_H
#define RTT_BALLFILTER_H

#include <proto/WorldBall.pb.h>
#include <proto/messages_robocup_ssl_detection.pb.h>
#include "BallObservation.h"
#include "CameraGroundBallFilter.h"
#include "FilteredBall.h"

struct GroundBallPrediction{
  GroundBallPrediction(CameraGroundBallPrediction prediction,bool hadRequestedCamera);
  CameraGroundBallPrediction prediction;
  bool hadRequestedCamera;
};
class BallFilter{
 public:
  /**
   * Constructs a ball filter with the given observation.
   * @param observation
   */
  explicit BallFilter(const BallObservation& observation);

  /**
   * Predicts the filter until Time to get a ground prediction for camera ID.
   * If the camera is not tracked by the current filter, information from other camera's is used.
   * @param cameraID to predict the ball for
   * @param until Time to predict the ball at
   * @return The prediction
   */
  [[nodiscard]] GroundBallPrediction predictCam(int cameraID, Time until) const;

  /**
   * Updates the ground filter with a given camera ID with Prediction Observation pairs.
   * If not yet tracked, this camera is added to the list of tracked camera's.
   * @return true if the filter can be removed e.g. it's components are empty for a while.
   */
  bool processDetections(const CameraGroundBallPredictionObservationPair& detections, int cameraID);

  /**
   * Predicts and merges the ball filter at given time
   * @param time
   * @return The Predicted/Filtered ball
   */
  [[nodiscard]] FilteredBall mergeBalls(Time time) const;

  /**
   * @return Gets the health of the ball filter
   */
  [[nodiscard]] double getHealth() const;
 private:
  std::map<int,CameraGroundBallFilter> groundFilters;
};

#endif  // RTT_BALLFILTER_H
