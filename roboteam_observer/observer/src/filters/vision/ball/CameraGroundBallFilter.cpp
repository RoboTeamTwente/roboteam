#include <filters/vision/ball/CameraGroundBallFilter.h>
#include <roboteam_utils/RobotShape.h>

#include <utility>

void CameraGroundBallFilter::update(const BallObservation& observation) {
    ekf.update(observation.position);
    objectSeen(observation.timeCaptured);
}
bool CameraGroundBallFilter::updateNotSeen(Time time) {
    objectInvisible(time);
    return getHealth() <= 0.0 && consecutiveFramesNotSeen() >= 3;
}
bool CameraGroundBallFilter::processDetections(const CameraGroundBallPredictionObservationPair& prediction_observation_pair) {
    predictFilter(prediction_observation_pair.prediction);
    bool removeFilter = false;
    if (prediction_observation_pair.observation.has_value()) {
        update(prediction_observation_pair.observation.value());
    } else {
        removeFilter = updateNotSeen(prediction_observation_pair.prediction.time);
    }
    return removeFilter;
}
void CameraGroundBallFilter::predictFilter(const CameraGroundBallPrediction& prediction) {
    // simple function for now but may become complicated with collisions
    ekf.predict(prediction.time);
}
Eigen::Vector2d CameraGroundBallFilter::getVelocityEstimate(Time time) const { return ekf.getVelocityEstimate(time); }
CameraGroundBallFilter::CameraGroundBallFilter(const BallObservation& observation, const Eigen::Vector2d& velocity_estimate)
    : CameraObjectFilter(0.2, 1 / 60.0, 15, 3, observation.timeCaptured) {
    Eigen::Vector4d startState = {observation.position.x(), observation.position.y(), velocity_estimate.x(), velocity_estimate.y()};
    Eigen::Matrix4d startCovariance = Eigen::Matrix4d::Zero();
    constexpr double BALL_POSITION_INITIAL_COV = 0.05;  //[m] uncertainty in initial ball position
    constexpr double BALL_VELOCITY_INITIAL_COV = 4.0;   //[m/s]

    startCovariance(0, 0) = BALL_POSITION_INITIAL_COV;
    startCovariance(1, 1) = BALL_POSITION_INITIAL_COV;
    startCovariance(2, 2) = BALL_VELOCITY_INITIAL_COV;
    startCovariance(3, 3) = BALL_VELOCITY_INITIAL_COV;

    constexpr double BALL_MODEL_ERROR = 1.0;
    constexpr double BALL_MEASUREMENT_ERROR = 0.002;  //[m] estimated average position uncertainty in ball detections
    ekf = GroundBallExtendedKalmanFilter(startState, startCovariance, BALL_MODEL_ERROR, BALL_MEASUREMENT_ERROR, observation.timeCaptured);
}
CameraGroundBallPrediction CameraGroundBallFilter::predict(Time time, std::vector<FilteredRobot> yellowRobots, std::vector<FilteredRobot> blueRobots) {
    auto positionEstimate = ekf.getPositionEstimate(time);
    auto velocityEstimate = ekf.getVelocityEstimate(time);

    // Checkrobots also alters the ekf if there is a collision
    if (!checkRobots(yellowRobots, positionEstimate, velocityEstimate)) {
        checkRobots(blueRobots, positionEstimate, velocityEstimate);
    }
    const auto& estimate = ekf.getStateEstimate(time);
    CameraGroundBallPrediction prediction(estimate.head<2>(), estimate.tail<2>(), time);
    return prediction;
}
bool CameraGroundBallFilter::checkRobots(const std::vector<FilteredRobot>& robots, const Eigen::Vector2d& positionEstimate, const Eigen::Vector2d& velocityEstimate) {
    for (const auto& robot : robots) {
        if (checkRobotCollision(robot, positionEstimate, velocityEstimate)) {
            return true;
        }
    }
    return false;
}

bool CameraGroundBallFilter::checkRobotCollision(const FilteredRobot& robot, const Eigen::Vector2d& positionEstimate, const Eigen::Vector2d& velocityEstimate) {
    // assumed robot radius of 0.08m and center to front of 0.06m. That's on the low side, but should make sure we are safe
    rtt::Vector2 robotPos = rtt::Vector2(robot.position.position.x(), robot.position.position.y());
    rtt::Angle robotYaw = rtt::Angle(robot.position.yaw);
    rtt::Vector2 ballPos = rtt::Vector2(positionEstimate.x(), positionEstimate.y());
    rtt::Vector2 ballVel = rtt::Vector2(velocityEstimate.x(), velocityEstimate.y());
    rtt::Vector2 robotVel = rtt::Vector2(robot.velocity.velocity.x(), robot.velocity.velocity.y());

    if (!rtt::RobotShape(robotPos, 0.05, 0.07, robotYaw).contains(ballPos)) {
        return false;
    }

    auto ballVelUsed = ballVel - robotVel;
    auto robotShape = rtt::RobotShape(robotPos, 0.05 + 0.0213, 0.07 + 0.0213, robotYaw);
    auto frontLine = robotShape.kicker();
    auto ballVelLine = rtt::LineSegment(ballPos, ballPos - ballVelUsed.stretchToLength(0.09 * 5));

    if (frontLine.intersects(ballVelLine)) {
        auto collisionAngle = (robotYaw - rtt::Angle(ballVelUsed.scale(-1)));

        if (std::abs(collisionAngle) > 0.5 * M_PI) {
            return false;
        }

        auto outVelAbs = ballVelUsed.length() - (ballVelUsed.length() * std::cos(collisionAngle) * 1);
        auto outVel = rtt::Vector2(robotYaw).scale(-1).rotate(collisionAngle).scale(-1).stretchToLength(outVelAbs);
        outVel += robotVel;
        Eigen::Vector2d outVelEigen(outVel.x, outVel.y);
        ekf.setVelocity(outVelEigen);
        return true;
    }

    auto botCircle = rtt::Circle(robotPos, 0.08 + 0.0213);
    auto intersects = botCircle.intersects(ballVelLine);

    if (!intersects.empty()) {
        auto intersect = intersects[0];
        auto collisionAngle = (rtt::Vector2(robotPos - intersect).toAngle() - ballVelUsed.toAngle());

        if (std::abs(collisionAngle) > 0.5 * M_PI) {
            return false;
        }

        auto outVelAbs = ballVelUsed.length() - (ballVelUsed.length() * std::cos(collisionAngle) * 0.6);
        auto outVel = rtt::Vector2(robotPos - intersect).rotate(collisionAngle).scale(-1).stretchToLength(outVelAbs);
        outVel += robotVel;
        Eigen::Vector2d outVelEigen(outVel.x, outVel.y);
        ekf.setVelocity(outVelEigen);
        return true;
    }

    return false;
}
FilteredBall CameraGroundBallFilter::getEstimate(Time time) const {
    FilteredBall ball(ekf.getPositionEstimate(time), ekf.getVelocityEstimate(time), getHealth(), ekf.getPositionUncertainty().norm(), ekf.getVelocityUncertainty().norm());
    return ball;
}

CameraGroundBallPrediction::CameraGroundBallPrediction(Eigen::Vector2d pos, Eigen::Vector2d vel, Time time) : position{std::move(pos)}, velocity{std::move(vel)}, time{time} {}
