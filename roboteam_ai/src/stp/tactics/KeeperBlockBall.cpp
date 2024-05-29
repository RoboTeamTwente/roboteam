#include "stp/tactics/KeeperBlockBall.h"

#include <control/positionControl/OvershootComputations.h>
#include <roboteam_utils/HalfLine.h>
#include <roboteam_utils/Mathematics.h>
#include <stp/computations/InterceptionComputations.h>

#include "control/ControlUtils.h"
#include "gui/Out.h"
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
constexpr double MAX_DISTANCE_HEADING_TOWARDS_GOAL = 0.2;
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
    // const auto &world = info.getCurrentWorld();
    const auto &robot = info.getRobot().value();
    // const auto enemyRobot = world->getWorld()->getRobotClosestToBall(world::them);

    const auto keepersLineSegment = getKeepersLineSegment(field);
    // const auto ballTrajectory = estimateBallTrajectory(ball, enemyRobot);
    // const bool ballHeadsTowardsOurGoal = ballTrajectory.has_value() && isBallHeadingTowardsOurGoal(ballTrajectory.value(), field);
    LineSegment ballTrajectory(ball->position, ball->expectedEndPosition);
    bool ballHeadsTowardsOurGoal = ballTrajectory.intersects(keepersLineSegment).has_value();

    if (ballHeadsTowardsOurGoal) {
        shouldAvoidGoalPosts = false;
        const auto robotPosition = robot->getPos();
        const auto robotVelocity = robot->getVel();
        const auto maxRobotVelocity = info.getMaxRobotVelocity();
        const auto maxRobotAcceleration = Constants::MAX_ACC();
        const auto closestPointToGoal = Line(ballTrajectory).intersect(Line(keepersLineSegment));

        // If possible, we intercept the ball at the line
        if (closestPointToGoal.has_value() && keepersLineSegment.distanceToLine(closestPointToGoal.value()) < MAX_DISTANCE_HEADING_TOWARDS_GOAL) {
            const auto ballTimeAtClosestPoint = FieldComputations::getBallTimeAtPosition(*ball.get(), closestPointToGoal.value());
            auto [targetPosition, timeToTarget] = control::OvershootComputations::overshootingDestination(robotPosition, closestPointToGoal.value(), robotVelocity,
                                                                                                          maxRobotVelocity, maxRobotAcceleration, ballTimeAtClosestPoint);
            if (timeToTarget <= ballTimeAtClosestPoint) {
                return {targetPosition, shouldAvoidGoalPosts};
            }
        }

        double maxTimeLeftWhenArrived = std::numeric_limits<double>::lowest();
        Vector2 optimalTarget = Vector2();
        // double optimalTargetTime;
        // double optimalTimeToTarget;
        // if ball is already extremely close the robot, and the robot is on the line of the velocity of the ball, robot should not move
        // ??
        if (ball->position.dist(robotPosition) < 0.11) {
            return {ball->position, shouldAvoidGoalPosts};
        }

        for (double timeStep = 0.01; timeStep <= 3; timeStep += 0.01) {
            auto predictedBallPosition = FieldComputations::getBallPositionAtTime(*ball.get(), timeStep);

            if (!field.leftDefenseArea.contains(predictedBallPosition)) {
                continue;
            }

            auto [currentTarget, currentTimeToTarget] =
                control::OvershootComputations::overshootingDestination(robotPosition, predictedBallPosition, robotVelocity, maxRobotVelocity, maxRobotAcceleration, timeStep);

            double timeLeftWhenArrived = timeStep - currentTimeToTarget;

            if (timeLeftWhenArrived > maxTimeLeftWhenArrived) {
                maxTimeLeftWhenArrived = timeLeftWhenArrived;
                optimalTarget = currentTarget;
                // optimalTargetTime = timeStep;
                // optimalTimeToTarget = currentTimeToTarget;
            }
        }
        return {optimalTarget, shouldAvoidGoalPosts};
    }

    // This lets the keeper intercept the ball in the defense area when it's not heading towards our goal. This did not work well at the Schubert open, since we always just barely
    // missed the ball. Might be tweaking control constants const KeeperInterceptionInfo keeperInterceptionInfo = InterceptionComputations::calculateKeeperInterceptionInfo(world,
    // robot); if (keeperInterceptionInfo.canIntercept) {
    //     return keeperInterceptionInfo.interceptLocation;
    // }

    if (ball->position.x >= field.leftGoalArea.rightLine().center().x - MAX_DISTANCE_BALL_BEHIND_GOAL) {
        Vector2 leftGoalPost = field.leftGoalArea.topRight();
        Vector2 rightGoalPost = field.leftGoalArea.bottomRight();

        std::array<rtt::Vector2, 2> rightGoalPostArr = {ball->position, rightGoalPost};
        std::array<rtt::Vector2, 2> leftGoalPostArr = {ball->position, leftGoalPost};
        gui::Out::draw(
            {
                .label = "ballToLeftGoalPost",
                .color = proto::Drawing::MAGENTA,
                .method = proto::Drawing::LINES_CONNECTED,
                .category = proto::Drawing::DEBUG,
                .forRobotId = robot->getId(),
                .thickness = 1,
            },
            rightGoalPostArr);
        gui::Out::draw(
            {
                .label = "ballToRightGoalPost",
                .color = proto::Drawing::MAGENTA,
                .method = proto::Drawing::LINES_CONNECTED,
                .category = proto::Drawing::DEBUG,
                .forRobotId = robot->getId(),
                .thickness = 1,
            },
            leftGoalPostArr);

        // Calculate the angle bisector of the line from the ball to the goal posts
        Vector2 ballToLeftGoalPost = leftGoalPost - ball->position;
        Vector2 ballToRightGoalPost = rightGoalPost - ball->position;
        // Add the unit vectors together and normalize the result to get the bisector unit vector
        Vector2 bisectorUnitVector = (ballToLeftGoalPost.normalize() + ballToRightGoalPost.normalize()).normalize();
        // Scale the bisector unit vector such that it will be long enough to reach the goal line
        auto scalingFactor = (field.leftGoalArea.rightLine().center().x - ball->position.x) / bisectorUnitVector.x;
        Vector2 bisectorPoint = ball->position + bisectorUnitVector * scalingFactor;

        std::array<rtt::Vector2, 2> bisectorArr = {ball->position, bisectorPoint};
        gui::Out::draw(
            {
                .label = "bisectorLine",
                .color = proto::Drawing::RED,
                .method = proto::Drawing::LINES_CONNECTED,
                .category = proto::Drawing::DEBUG,
                .forRobotId = robot->getId(),
                .thickness = 2,
            },
            bisectorArr);

        // Old target position for comparison
        std::array<rtt::Vector2, 2> ballGoalLineArr = {ball->position, field.leftGoalArea.rightLine().center() - Vector2(PROJECT_BALL_DISTANCE_TO_GOAL, 0)};
        gui::Out::draw(
            {
                .label = "targetPositionOld",
                .color = proto::Drawing::BLUE,
                .method = proto::Drawing::LINES_CONNECTED,
                .category = proto::Drawing::DEBUG,
                .forRobotId = robot->getId(),
                .thickness = 1,
            },
            {ballGoalLineArr});

        // project the leftGoalPost on the ball->pos bisectorPoint line if the ball has positive x, else project the rightGoalPost
        Vector2 targetGoalPost = ball->position.y >= 0 ? leftGoalPost : rightGoalPost;
        const auto targetPositionNew = LineSegment(ball->position, bisectorPoint).project(targetGoalPost);
        std::array<rtt::Vector2, 2> goalPostArr = {targetGoalPost, targetPositionNew};
        gui::Out::draw(
            {
                .label = "targetPositionNew",
                .color = proto::Drawing::GREEN,
                .method = proto::Drawing::LINES_CONNECTED,
                .category = proto::Drawing::DEBUG,
                .forRobotId = robot->getId(),
                .thickness = 1,
            },
            goalPostArr);
        return {targetPositionNew, shouldAvoidGoalPosts};
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
