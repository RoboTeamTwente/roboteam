//
// Created by rolf on 08-07-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
#include "BallFilter.h"

struct BallAssignmentResult {
  explicit BallAssignmentResult(const std::vector<CameraGroundBallPrediction>& predictions);
  std::vector<CameraGroundBallPredictionObservationPair> op_pairs;
  std::vector<BallObservation> unpairedObservations;
};


namespace BallAssigner{
BallAssignmentResult assign_balls(const std::vector<CameraGroundBallPrediction>& predictions,
                                  const std::vector<BallObservation>& observations);
BallObservation mergeObservationsByArea(const std::vector<BallObservation>& observations);
}


#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
