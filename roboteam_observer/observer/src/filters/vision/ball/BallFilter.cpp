#include "filters/vision/ball/BallFilter.h"

#include <utility>

BallFilter::BallFilter(const BallObservation &observation) : groundFilters{std::make_pair(observation.cameraID, CameraGroundBallFilter(observation))} {}
GroundBallPrediction BallFilter::predictCam(int cameraID, Time until, std::vector<FilteredRobot> yellowRobots, std::vector<FilteredRobot> blueRobots) {
    auto camera_filter = groundFilters.find(cameraID);
    if (camera_filter != groundFilters.end()) {
        GroundBallPrediction prediction(camera_filter->second.predict(until, yellowRobots, blueRobots), true);
        return prediction;
    }
    // no information for this camera available; we merge data from available camera's
    FilteredBall estimate = mergeBalls(until);
    GroundBallPrediction prediction(CameraGroundBallPrediction(estimate.position, estimate.velocity, until), false);
    return prediction;
}
bool BallFilter::processDetections(const CameraGroundBallPredictionObservationPair &detections, int cameraID) {
    auto cameraFilter = groundFilters.find(cameraID);
    if (cameraFilter == groundFilters.end()) {
        // create a new filter based on the given observation
        if (detections.observation.has_value()) {
            groundFilters.insert(std::make_pair(cameraID, CameraGroundBallFilter(detections.observation.value(), detections.prediction.velocity)));
        }
        return false;  // we cannot remove a filter based on a new camera image
    }
    bool removeFilter = cameraFilter->second.processDetections(detections);
    if (removeFilter) {
        groundFilters.erase(cameraFilter);
    }
    return groundFilters.empty();
}
FilteredBall BallFilter::mergeBalls(Time time) {
    FilteredBall ball;
    ball.position = Eigen::Vector2d(0, 0);
    ball.velocity = Eigen::Vector2d(0, 0);
    ball.positionCamera = Eigen::Vector2d(0, 0);
    ball.time = time;
    auto posUncertainty = 0.0;
    auto velocityUncertainty = 0.0;
    constexpr double mergeFactor = 1.5;
    for (auto &filter : groundFilters) {
        auto estimate = filter.second.getEstimate(time);
        double weight = 100.0 / estimate.health;
        double posWeight = std::pow(estimate.posUncertainty * weight, -mergeFactor);
        double velWeight = std::pow(estimate.velocityUncertainty * weight, -mergeFactor);
        ball.position += estimate.position * posWeight;
        ball.velocity += estimate.velocity * velWeight;
        ball.positionCamera += filter.second.getLastObservation().position * posWeight;
        posUncertainty += posWeight;
        velocityUncertainty += velWeight;
        if (filter.second.getLastObservationAvailableAndReset()) {
            ball.currentObservation = filter.second.getLastObservation();
        }
    }
    constexpr double limit = 1e-10;
    if (posUncertainty >= limit) {
        ball.position /= posUncertainty;
        ball.positionCamera /= posUncertainty;
    }
    if (velocityUncertainty >= limit) {
        ball.velocity /= velocityUncertainty;
    }

    return ball;
}
double BallFilter::getHealth() const {
    double maxHealth = 0.0;
    for (const auto &filter : groundFilters) {
        maxHealth = fmax(filter.second.getHealth(), maxHealth);
    }
    return maxHealth;
}
double BallFilter::getNumObservations() const {
    double maxTotalObservations = 0.0;
    for (const auto &filter : groundFilters) {
        maxTotalObservations = fmax(filter.second.numObservations(), maxTotalObservations);
    }
    return maxTotalObservations;
}
GroundBallPrediction::GroundBallPrediction(CameraGroundBallPrediction prediction, bool hadRequestedCamera)
    : prediction{std::move(prediction)}, hadRequestedCamera{hadRequestedCamera} {}
