//
// Created by rolf on 08-07-21.
//

#include "observer/filters/vision/ball/BallAssigner.h"
#include <algorithm>
namespace BallAssigner{
BallObservation mergeObservationsByArea(const std::vector<BallObservation>& observations){
  //It's the user's responsibility to ensure all observations are from the same detectionFrame;
  //otherwise, merging makes very little sense
  assert(!observations.empty());
#ifndef NDEBUG
  for(const auto& observation : observations){
    assert(observation.cameraID == observations[0].cameraID);
    assert(observation.timeCaptured == observations[0].timeCaptured);
    assert(observation.timeSent == observations[0].timeSent);
  }
#endif
  unsigned int totalArea = 0;
  for(const auto& observation : observations){
    totalArea+=observation.area;
  }
  bool zero_area = totalArea <= 0;
  Eigen::Vector2d position(0,0);
  Eigen::Vector2d pixelPosition(0,0);
  double confidence = 0.0;
  double height = 0.0;
  for(const auto& observation : observations){
    double weight = zero_area ? 1.0/static_cast<double>(observations.size()) :
        observation.area / static_cast<double>(totalArea);
    position += weight * observation.position;
    pixelPosition += weight * observation.pixelPosition;
    confidence = fmax(confidence,observation.confidence);
    height += weight * observation.height;
  }
  return BallObservation(observations[0].cameraID,observations[0].timeCaptured,observations[0].timeSent,
                         position,pixelPosition,confidence,totalArea,height);
}


BallAssignmentResult assign_balls(const std::vector<CameraGroundBallPrediction>& predictions,
                                  const std::vector<BallObservation>& observations) {
  BallAssignmentResult assignment(predictions);
  std::vector<std::vector<BallObservation>> prediction_observations(predictions.size());

  static constexpr double MAX_ACCEPT_DIST = 0.3;
  //first pass: greedily assign observations to closest predictions
  //this will probably become more complicated in the future as it will work on a per-object basis
  for(const auto& observation : observations){
      auto closest_prediction = std::min_element(assignment.op_pairs.begin(),assignment.op_pairs.end(),
                                                 [&](const CameraGroundBallPredictionObservationPair& first,
                                                     const CameraGroundBallPredictionObservationPair& second){
                                                   return (observation.position - first.prediction.position).squaredNorm() <
                                                       (observation.position - second.prediction.position).squaredNorm();
                                                 });
      //if there's no close ball, we will use it for new filters.
      if(!assignment.op_pairs.empty() ||
          (closest_prediction->prediction.position-observation.position).norm() > MAX_ACCEPT_DIST){
        assignment.unpairedObservations.push_back(observation);
      }else{
        //add the observation to the list of observations for a given prediction;
        auto prediction_pos =std::distance(assignment.op_pairs.begin(),closest_prediction);
        prediction_observations[prediction_pos].push_back(observation);
      }
  }

  //we only want to assign a single ball per ground ball filter, so we pick the best one for each prediction:
  for (std::size_t i = 0; i < predictions.size(); ++i) {
    const auto& prediction = predictions[i];
    const auto& assigned_observations =prediction_observations[i];

    //simple base cases which are most often hit
    if(assigned_observations.empty()){
      assignment.op_pairs[i].observation = std::nullopt;
    }else if(assigned_observations.size() == 1){
      assignment.op_pairs[i].observation = assigned_observations.front();
    }else{
      //here, we merge two balls based on area if they are really close; sometimes it happens the ball is detected as two balls
      //other balls are discarded/new filters are created for them.
      auto best_observation = std::min_element(assigned_observations.begin(),assigned_observations.end(),
                                               [prediction](const BallObservation& a, const BallObservation& b){
        return (a.position - prediction.position).squaredNorm() < (b.position-prediction.position).squaredNorm();
      });

      std::vector<BallObservation> mergeableObservations;

      static constexpr double BALL_MERGE_DIST = 0.02;// [m]
      for(const auto& observation : assigned_observations){
        if((observation.position - best_observation->position).norm() < BALL_MERGE_DIST){
          mergeableObservations.push_back(observation);
        }else{
          //observation is discarded e.g., a new filter is created
          assignment.unpairedObservations.push_back(observation);
        }
      }

      assignment.op_pairs[i].observation = mergeableObservations.size() > 1 ? mergeObservationsByArea(mergeableObservations)
          : mergeableObservations.front();
    }

  }


  return assignment;
}
}
BallAssignmentResult::BallAssignmentResult(const std::vector<CameraGroundBallPrediction> &predictions) : op_pairs(predictions.size()){
  //initialize result with predictions
  for (std::size_t i = 0; i < predictions.size(); ++i) {
    op_pairs[i].prediction = predictions[i];
  }

}
