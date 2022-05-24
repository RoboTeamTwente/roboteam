//
// Created by rolf on 08-07-21.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
#include "BallFilter.h"

/**
 * Struct which holds the result of the assignment; a list of observation-prediction pairs for each object filter,
 * and a list of unmatched ball observations
 */
struct BallAssignmentResult {
  explicit BallAssignmentResult(const std::vector<CameraGroundBallPrediction>& predictions);
  std::vector<CameraGroundBallPredictionObservationPair> op_pairs;
  std::vector<BallObservation> unpairedObservations;
};



namespace BallAssigner{
/**
 * Assigns balls to observations, and list all unmatched balls. Preserves the order of the predictions, currently
 * @param predictions given predictions
 * @param observations given observations
 * @return the matched prediction-observation pairs, and all unpaired observations which require new filters.
 */
BallAssignmentResult assign_balls(const std::vector<CameraGroundBallPrediction>& predictions,
                                  const std::vector<BallObservation>& observations);

/**
 * Merges a set of ball observations, weighted by area. This is useful for when the ball is detected as two balls, which sometimes happens
 * @return The merged observation
 */
BallObservation mergeObservationsByArea(const std::vector<BallObservation>& observations);
}


#endif //RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_BALL_BALLASSIGNER_H_
