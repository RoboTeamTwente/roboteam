#include "stp/tactics/KeeperBlockBall.h"

#include <control/positionControl/OvershootComputations.h>
#include <roboteam_utils/HalfLine.h>
#include <roboteam_utils/Mathematics.h>
#include <stp/computations/InterceptionComputations.h>

#include "control/ControlUtils.h"
#include "roboteam_utils/LineSegment.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::tactic {

// We do not want the keeper to stand completely inside the goal, but a tiny bit outside.
const double KEEPER_DISTANCE_TO_GOAL_LINE = Constants::ROBOT_RADIUS() * std::sin(toRadians(80.0));
// And by standing a tiny bit inside, we cannot move completely to a goal side. This is by how much less that is.
const double KEEPER_GOAL_DECREASE_AT_ONE_SIDE = Constants::ROBOT_RADIUS() * std::cos(toRadians(80.0)) + 0.01;  // Plus a small margin to prevent keeper from crashing into goal
// The maximum distance from the goal for when we say the ball is heading towards our goal
constexpr double MAX_DISTANCE_HEADING_TOWARDS_GOAL = 2;
// For determining where the keeper should stand to stand between the ball and the goal, we draw a line from the ball to a bit behind the goal
constexpr double PROJECT_BALL_DISTANCE_TO_GOAL = 0.5;  // Small means keeper will me more in center, big means keeper will be more to the side of the goal
// We stop deciding where the keeper should be if the ball is too far behind our own goal
constexpr double MAX_DISTANCE_BALL_BEHIND_GOAL = 0.3;

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(const StpInfo &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !skillStpInfo.getCurrentWorld()) {
        return std::nullopt;
    }

    const auto targetPosition = calculateTargetPosition(info);
    skillStpInfo.setPositionToMoveTo(targetPosition.first);
    skillStpInfo.setShouldAvoidGoalPosts(targetPosition.second);
    skillStpInfo.setShouldAvoidOutOfField(targetPosition.second);
    skillStpInfo.setShouldAvoidOurRobots(targetPosition.second);
    skillStpInfo.setShouldAvoidTheirRobots(targetPosition.second);

    const auto targetAngle = calculateTargetAngle(info.getBall().value(), targetPosition.first);
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool KeeperBlockBall::isEndTactic() noexcept { return true; }

bool KeeperBlockBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool KeeperBlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    const double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN * M_PI;
    const auto distanceToTarget = (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length();
    return distanceToTarget > errorMargin;
}

const char *KeeperBlockBall::getName() { return "Keeper Block Ball"; }

LineSegment KeeperBlockBall::getKeepersLineSegment(const Field &field) {
    const auto keepersLineSegmentLeft = field.leftGoalArea.topRight() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, -KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    const auto keepersLineSegmentRight = field.leftGoalArea.bottomRight() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    return LineSegment(keepersLineSegmentLeft, keepersLineSegmentRight);
}

std::optional<HalfLine> KeeperBlockBall::estimateBallTrajectory(const world::view::BallView &ball, const std::optional<world::view::RobotView> &enemyRobot) {
    // If the ball is already moving, the trajectory of the ball is clear
    if (ball->velocity.length2() > control_constants::BALL_STILL_VEL2) {
        const auto start = ball->position;
        const auto direction = ball->position + ball->velocity;
        return HalfLine(start, direction);
    }

    const bool hasEnemy = enemyRobot.has_value() && enemyRobot.value().get() != nullptr;

    // If the enemy robot already has the ball, it will probably kick in the direction it is facing
    if (hasEnemy && enemyRobot->get()->hasBall()) {
        const auto start = enemyRobot->get()->getPos();
        const auto direction = enemyRobot->get()->getPos() + enemyRobot->get()->getAngle().toVector2();
        return HalfLine(start, direction);
    }

    // If the enemy is only a bit close, we look at the direction from the robot to the ball
    if (enemyRobot && enemyRobot->get()->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        const auto start = enemyRobot->get()->getPos();
        const auto direction = ball->position;
        return HalfLine(start, direction);
    }

    // Otherwise, the ball probably will not be moved by enemies soon :P
    return std::nullopt;
}

bool KeeperBlockBall::isBallHeadingTowardsOurGoal(const HalfLine &ballTrajectory, const Field &field) {
    const auto goalLineSegment = LineSegment(field.leftGoalArea.bottomRight(), field.leftGoalArea.topRight());
    const auto intersectionWithGoalLine = ballTrajectory.intersect(Line(goalLineSegment));
    // Ball is considered heading towards goal if the intersection exists and is near our goal
    return intersectionWithGoalLine.has_value() && goalLineSegment.distanceToLine(intersectionWithGoalLine.value()) < MAX_DISTANCE_HEADING_TOWARDS_GOAL;
}

std::pair<Vector2, bool> KeeperBlockBall::calculateTargetPosition(const StpInfo info) noexcept {
    bool shouldAvoidGoalPosts = true;
    const auto &field = info.getField().value();
    const auto &ball = info.getBall().value();
    const auto &world = info.getCurrentWorld();
    const auto &robot = info.getRobot().value();
    const auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);

    const auto keepersLineSegment = getKeepersLineSegment(field);
    const auto ballTrajectory = estimateBallTrajectory(ball, enemyRobot);
    const bool ballHeadsTowardsOurGoal = ballTrajectory.has_value() && isBallHeadingTowardsOurGoal(ballTrajectory.value(), field);

    if (ballHeadsTowardsOurGoal) {
        const auto targetPosition = keepersLineSegment.getClosestPointToLine(ballTrajectory->toLine());
        if (targetPosition.has_value()) {
            shouldAvoidGoalPosts = false;
            const auto targetTime = FieldComputations::getBallTimeAtPosition(*ball.get(), targetPosition.value());
            const auto startPosition = robot->getPos();
            const auto startVelocity = robot->getVel();
            const auto maxVelocity = info.getMaxRobotVelocity();
            const auto maxAcceleration = Constants::MAX_ACC();
            const auto newTarget =
                control::OvershootComputations::overshootingDestination(startPosition, targetPosition.value(), startVelocity, maxVelocity, maxAcceleration, targetTime);
            return {newTarget, shouldAvoidGoalPosts};
        }
    }

    // const KeeperInterceptionInfo keeperInterceptionInfo = InterceptionComputations::calculateKeeperInterceptionInfo(world, robot);
    // if (keeperInterceptionInfo.canIntercept) {
    //     return keeperInterceptionInfo.interceptLocation;
    // }

    if (ball->position.x >= field.leftGoalArea.rightLine().center().x - MAX_DISTANCE_BALL_BEHIND_GOAL) {
        const auto ballGoalLine = Line(ball->position, field.leftGoalArea.rightLine().center() - Vector2(PROJECT_BALL_DISTANCE_TO_GOAL, 0));
        const auto targetPosition = keepersLineSegment.getClosestPointToLine(ballGoalLine);
        if (targetPosition.has_value()) {
            return {targetPosition.value(), shouldAvoidGoalPosts};
        }
    }

    // Default case: go to the center of the goal
    return {Vector2(keepersLineSegment.start.x, 0), shouldAvoidGoalPosts};
}

Angle KeeperBlockBall::calculateTargetAngle(const world::view::BallView &ball, const Vector2 &targetKeeperPosition) {
    // Look towards ball to ensure ball hits the front assembly to reduce odds of ball reflecting in goal
    const auto keeperToBall = (ball->position - targetKeeperPosition) / 1.6;
    return keeperToBall.angle();
}

}  // namespace rtt::ai::stp::tactic
