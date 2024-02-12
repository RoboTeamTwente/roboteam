//
// Created by john on 12/18/19.
//

#include "world/Ball.hpp"

#include "gui/Out.h"
#include "utilities/Constants.h"
#include "utilities/GameSettings.h"
#include "world/World.hpp"

namespace rtt::world::ball {

/**
 * The movement friction during simulation and real life are different, because the simulation does not model
 * everything. So the movement friction has to be adjusted to compensate for this difference.
 *
 * The expected movement friction of the ball during simulation
 */
constexpr static float SIMULATION_FRICTION = 0.69;

/**
 * The expected movement friction of the ball on the field
 */
constexpr static float REAL_FRICTION = 0.5;

Ball::Ball(const proto::WorldBall& copy, const World* data) : position{copy.pos().x(), copy.pos().y()}, velocity{copy.vel().x(), copy.vel().y()}, visible{copy.visible()} {
    if (!visible || position == Vector2()) {
        initBallAtExpectedPosition(data);
    }
    updateBallAtRobotPosition(data);
    updateExpectedBallEndPosition(data);
}

void Ball::initBallAtExpectedPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld || !previousWorld->getBall()) {
        return;
    }
    position = previousWorld->getBall().value()->position;
}

void Ball::updateExpectedBallEndPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld || !previousWorld->getBall()) {
        return;
    }

    auto ball = previousWorld->getBall().value();

    double ballVelSquared = ball->velocity.length2();
    const double frictionCoefficient = GameSettings::getRobotHubMode() == net::RobotHubMode::SIMULATOR ? SIMULATION_FRICTION : REAL_FRICTION;

    expectedEndPosition = ball->position + ball->velocity.stretchToLength(ballVelSquared / frictionCoefficient);
    std::array<rtt::Vector2, 2> arr = {expectedEndPosition, ball->position};
    std::span<rtt::Vector2> span(arr);
    // maybe change to a cross at the end instead of a line?
    rtt::ai::gui::Out::draw(
        {
            .label = "expected_end_position_ball",
            .color = proto::Drawing::MAGENTA,
            .method = proto::Drawing::LINES_CONNECTED,
            .thickness = 3,
        },
        span);
}

void Ball::updateBallAtRobotPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> world = data->getWorld();
    if (!world.has_value()) return;

    std::optional<rtt::world::view::RobotView> robotWithBall = world->whichRobotHasBall();
    if (!robotWithBall.has_value()) {
        return;
    }
    if ((robotWithBall->get()->getVel() - velocity).length() > 0.1) {
        return;
    }
    // Place the ball where we would expect it to be given that this robot has the ball
    double distanceInFrontOfRobot = ai::stp::control_constants::CENTER_TO_FRONT + ai::Constants::BALL_RADIUS();
    position = robotWithBall->get()->getPos() + robotWithBall->get()->getAngle().toVector2(distanceInFrontOfRobot);
}

}  // namespace rtt::world::ball
