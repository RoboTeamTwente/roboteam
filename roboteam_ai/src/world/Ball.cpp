#include "world/Ball.hpp"

#include "gui/Out.h"
#include "utilities/Constants.h"
#include "utilities/GameSettings.h"
#include "world/World.hpp"

namespace rtt::world::ball {
// Uncomment the following lines to calculate the friction coefficient
// static Vector2 maxVelocity;
// static Vector2 posAtMaxVelocity;
// static std::chrono::steady_clock::time_point timeOfLastReset;

Ball::Ball(const proto::WorldBall& copy, const World* data) : position{copy.pos().x(), copy.pos().y()}, velocity{copy.vel().x(), copy.vel().y()}, visible{copy.visible()} {
    if (!visible || position == Vector2()) {
        initBallAtExpectedPosition(data);
    }
    updateBallAtRobotPosition(data);
    updateExpectedBallEndPosition();

    if (position != Vector2()) {
        std::array<rtt::Vector2, 1> point = {position};
        rtt::ai::gui::Out::draw(
            {
                .label = "position_ball_AI",
                .color = proto::Drawing::CYAN,
                .method = proto::Drawing::CIRCLES,
                .size = 4,
            },
            point);
    }
}

void Ball::initBallAtExpectedPosition(const world::World* data) noexcept {
    auto previousWorld = data->getHistoryWorld(1);
    if (!previousWorld || !previousWorld->getBall()) {
        return;
    }
    position = previousWorld->getBall().value()->position;
}

void Ball::updateExpectedBallEndPosition() noexcept {
    const double ballVelSquared = velocity.length2();
    const double frictionCoefficient =
        GameSettings::getRobotHubMode() == net::RobotHubMode::SIMULATOR ? ai::stp::control_constants::SIMULATION_FRICTION : ai::stp::control_constants::REAL_FRICTION;
    expectedEndPosition = position + velocity.stretchToLength(ballVelSquared / frictionCoefficient);

    // Uncomment the following lines to calculate the friction coefficient
    // auto currentTime = std::chrono::steady_clock::now();
    // if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - timeOfLastReset).count() >= 30) {
    //     maxVelocity = Vector2(0, 0);
    //     posAtMaxVelocity = Vector2(0, 0);
    //     timeOfLastReset = currentTime;
    // }
    // if (velocity.length() > maxVelocity.length()) {
    //     maxVelocity = velocity;
    //     posAtMaxVelocity = position;
    // }
    // std::cout << "Expected friction coefficient: " << maxVelocity.length2() / (posAtMaxVelocity - ball->position).length() << std::endl;
    // std::cout << "Time till next reset: " << 30.0 - std::chrono::duration<double>(currentTime - timeOfLastReset).count() << std::endl;

    std::array<rtt::Vector2, 2> ballPoints = {expectedEndPosition, position};
    rtt::ai::gui::Out::draw(
        {
            .label = "expected_end_position_ball",
            .color = proto::Drawing::MAGENTA,
            .method = proto::Drawing::LINES_CONNECTED,
            .thickness = 3,
        },
        ballPoints);
}

void Ball::updateBallAtRobotPosition(const world::World* data) noexcept {
    auto world = data->getWorld();
    if (!world.has_value()) {
        return;
    }

    auto robotWithBall = world->whichRobotHasBall();
    if (!robotWithBall.has_value()) {
        return;
    }

    if ((robotWithBall->get()->getVel() - velocity).length() > 1.6) {
        return;
    }

    auto robotClostestToPoint = world->getRobotClosestToPoint(position, Team::both);
    if (robotClostestToPoint.has_value() && (robotClostestToPoint->get()->getPos() - position).length() > 0.15) {
        return;
    }

    double distanceInFrontOfRobot = ai::stp::control_constants::CENTER_TO_FRONT + ai::Constants::BALL_RADIUS();
    position = robotWithBall->get()->getPos() + robotWithBall->get()->getAngle().toVector2(distanceInFrontOfRobot);
    velocity = robotWithBall->get()->getVel();
}

}  // namespace rtt::world::ball
