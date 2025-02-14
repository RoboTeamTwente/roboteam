#include "world/Robot.hpp"

#include "roboteam_utils/Print.h"
#include "utilities/Constants.h"
#include "utilities/GameSettings.h"
#include "world/World.hpp"

namespace rtt::world::robot {
Robot::Robot(const proto::WorldRobot &copy, rtt::world::Team team, std::optional<view::BallView> ball)
    : id{static_cast<int>(copy.id())},
      team{team},
      pos{copy.pos().x(), copy.pos().y()},
      vel{copy.vel().x(), copy.vel().y()},
      yaw{copy.yaw()},
      distanceToBall{-1.0},
      angularVelocity{copy.w()} {
    if (id < 16) {
        workingDribbler = ai::constants::ROBOT_HAS_WORKING_DRIBBLER(id);
        workingBallSensor = ai::constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
    }

    if (ball.has_value()) {
        setDistanceToBall(pos.dist((*ball)->position));
        auto angleRobotToBall = ((*ball)->position - pos).angle();
        setAngleDiffToBall(yaw.shortestAngleDiff(Angle(angleRobotToBall)));
    }

    if (team == Team::us) {
        if (copy.has_feedbackinfo()) {
            updateFromFeedback(copy.feedbackinfo());
        }
        updateHasBallMap(ball);
    } else {
        auto hasBallAccordingToVision = distanceToBall < ai::constants::HAS_BALL_DISTANCE() && angleDiffToBall < ai::constants::HAS_BALL_ANGLE;
        setHasBall(hasBallAccordingToVision);
    }
}

int Robot::getId() const noexcept { return id; }

Team Robot::getTeam() const noexcept { return team; }

const Vector2 &Robot::getPos() const noexcept { return pos; }

const Vector2 &Robot::getVel() const noexcept { return vel; }

const Angle &Robot::getYaw() const noexcept { return yaw; }

void Robot::setYaw(const Angle &_yaw) noexcept { Robot::yaw = _yaw; }

double Robot::getAngularVelocity() const noexcept { return angularVelocity; }

bool Robot::isBatteryLow() const noexcept { return batteryLow; }

void Robot::setBatteryLow(bool _batteryLow) noexcept { Robot::batteryLow = _batteryLow; }

bool Robot::isWorkingDribbler() const noexcept { return workingDribbler; }

bool Robot::isWorkingBallSensor() const noexcept { return workingBallSensor; }

void Robot::setWorkingBallSensor(bool _workingBallSensor) noexcept { Robot::workingBallSensor = _workingBallSensor; }

void Robot::setBallSensorSeesBall(bool _seesBall) noexcept { ballSensorSeesBall = _seesBall; }

void Robot::setDribblerSeesBall(bool _seesBall) noexcept {
    // if (_seesBall) RTT_INFO("Robot " + std::to_string(id) + " has ball")
    // dribblerSeesBall = _seesBall;
}

void Robot::setHasBall(bool _hasBall) noexcept { Robot::robotHasBall = _hasBall; }

bool Robot::hasBall() const noexcept { return robotHasBall; }

double Robot::getDistanceToBall() const noexcept { return distanceToBall; }

void Robot::setDistanceToBall(double _distanceToBall) noexcept { Robot::distanceToBall = _distanceToBall; }

double Robot::getAngleDiffToBall() const noexcept { return angleDiffToBall; }

void Robot::setAngleDiffToBall(double _angleDiffToBall) noexcept { Robot::angleDiffToBall = _angleDiffToBall; }

void Robot::updateFromFeedback(const proto::RobotProcessedFeedback &feedback) noexcept {
    // TODO: add processing of more of the fields of feedback
    if (ai::constants::FEEDBACK_ENABLED) {
        setWorkingBallSensor(feedback.ball_sensor_is_working());
        setBatteryLow(feedback.battery_level() < 22);  // TODO: Figure out with electronics which value should be considered low
        setBallSensorSeesBall(feedback.ball_sensor_sees_ball());
        setDribblerSeesBall(feedback.dribbler_sees_ball());
    }
}

void Robot::updateHasBallMap(std::optional<view::BallView> &ball) {
    if (!ball) return;

    auto hasBallAccordingToVision = distanceToBall < ai::constants::HAS_BALL_DISTANCE() && angleDiffToBall < ai::constants::HAS_BALL_ANGLE;
    // auto hasBallAccordingToDribblerOrBallSensor = (GameSettings::getRobotHubMode() == net::RobotHubMode::BASESTATION) ? dribblerSeesBall : ballSensorSeesBall;
    auto hasBallAccordingToDribblerOrBallSensor = ballSensorSeesBall;
    if (hasBallAccordingToDribblerOrBallSensor && hasBallAccordingToVision) {
        hasBallUpdateMap[id].score = 25;
    } else {
        hasBallUpdateMap[id].score -= 2;
    }
    // Make sure the value does not get too large/small
    hasBallUpdateMap[id].score = std::clamp(hasBallUpdateMap[id].score, 0, 25);
    setHasBall(hasBallUpdateMap[id].score > 0);
}
}  // namespace rtt::world::robot
