#include "stp/tactics/KeeperBlockBall.h"

#include <control/positionControl/OvershootComputations.h>
#include <roboteam_utils/HalfLine.h>
#include <roboteam_utils/Mathematics.h>
#include <stp/computations/InterceptionComputations.h>

#include <span>

#include "control/ControlUtils.h"
#include "gui/Out.h"
#include "roboteam_utils/LineSegment.h"
#include "stp/computations/PositionComputations.h"
#include "stp/computations/PositionScoring.h"
#include "stp/skills/GoToPos.h"
#include "utilities/Constants.h"

namespace rtt::ai::stp::tactic {

// We do not want the keeper to stand completely inside the goal, but a tiny bit outside.
const double KEEPER_DISTANCE_TO_GOAL_LINE = constants::ROBOT_RADIUS * std::sin(toRadians(80.0));
// And by standing a tiny bit inside, we cannot move completely to a goal side. This is by how much less that is.
const double KEEPER_GOAL_DECREASE_AT_ONE_SIDE = constants::ROBOT_RADIUS * std::cos(toRadians(80.0)) + 0.01;  // Plus a small margin to prevent keeper from crashing into goal
// The maximum distance from the goal for when we say the ball is heading towards our goal
constexpr double MAX_DISTANCE_HEADING_TOWARDS_GOAL = 0.2;
// We stop deciding where the keeper should be if the ball is too far behind our own goal
constexpr double MAX_DISTANCE_BALL_BEHIND_GOAL = 0.1;

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(const StpInfo &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot() || !skillStpInfo.getCurrentWorld()) {
        return std::nullopt;
    }

    const auto targetPosition = calculateTargetPosition(info);
    skillStpInfo.setPositionToMoveTo(std::get<0>(targetPosition));
    skillStpInfo.setShouldAvoidGoalPosts(std::get<1>(targetPosition));
    skillStpInfo.setShouldAvoidOutOfField(std::get<1>(targetPosition));
    skillStpInfo.setShouldAvoidOurRobots(std::get<1>(targetPosition));
    skillStpInfo.setShouldAvoidTheirRobots(std::get<1>(targetPosition));
    skillStpInfo.setKickOrChip(KickType::CHIP);
    skillStpInfo.setShotPower(ShotPower::MAX);
    skillStpInfo.setShootOnFirstTouch(true);
    skillStpInfo.setPositionToShootAt(Vector2(6, 0));

    const auto yaw = calculateYaw(info.getBall().value(), std::get<0>(targetPosition));

    skillStpInfo.setYaw(yaw);
    skillStpInfo.setMaxJerk(std::get<2>(targetPosition));

    return skillStpInfo;
}

bool KeeperBlockBall::isEndTactic() noexcept { return true; }

bool KeeperBlockBall::isTacticFailing(const StpInfo &) noexcept { return false; }

bool KeeperBlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    return false;
}

const char *KeeperBlockBall::getName() { return "Keeper Block Ball"; }

LineSegment KeeperBlockBall::getKeepersLineSegment(const Field &field) {
    const auto keepersLineSegmentLeft = field.leftGoalArea.topRight() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, -KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    const auto keepersLineSegmentRight = field.leftGoalArea.bottomRight() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    return LineSegment(keepersLineSegmentLeft, keepersLineSegmentRight);
}

bool KeeperBlockBall::isBallHeadingTowardsOurGoal(const HalfLine &ballTrajectory, const Field &field) {
    const auto goalLineSegment = LineSegment(field.leftGoalArea.bottomRight(), field.leftGoalArea.topRight());
    const auto intersectionWithGoalLine = ballTrajectory.intersect(Line(goalLineSegment));
    // Ball is considered heading towards goal if the intersection exists and is near our goal
    return intersectionWithGoalLine.has_value() && goalLineSegment.distanceToLine(intersectionWithGoalLine.value()) < MAX_DISTANCE_HEADING_TOWARDS_GOAL;
}

std::tuple<Vector2, bool, double> KeeperBlockBall::calculateTargetPosition(const StpInfo info) noexcept {
    const auto &field = info.getField().value();
    const auto &ball = info.getBall().value();

    const auto keepersLineSegment = getKeepersLineSegment(field);
    const LineSegment ballTrajectory(ball->position, ball->expectedEndPosition);
    bool ballHeadsTowardsOurGoal = ballTrajectory.intersects(keepersLineSegment).has_value();

    if (ballHeadsTowardsOurGoal) {
        auto shouldAvoidGoalPosts = false;
        auto [targetPosition, maxJerk] = calculateTargetPositionBallShot(info, keepersLineSegment, ballTrajectory);
        return std::make_tuple(targetPosition, shouldAvoidGoalPosts, maxJerk);
    }

    // This lets the keeper intercept the ball in the defense area when it's not heading towards our goal. This did not work well at the Schubert open, since we always just barely
    // missed the ball. Might be tweaking control constants const KeeperInterceptionInfo keeperInterceptionInfo = InterceptionComputations::calculateKeeperInterceptionInfo(world,
    // robot); if (keeperInterceptionInfo.canIntercept) {
    // }
    // const KeeperInterceptionInfo keeperInterceptionInfo = InterceptionComputations::calculateKeeperInterceptionInfo(world, robot);
    // if (keeperInterceptionInfo.canIntercept) {
    //     return keeperInterceptionInfo.interceptLocation;
    // }

    if (ball->position.x >= field.leftGoalArea.rightLine().center().x - MAX_DISTANCE_BALL_BEHIND_GOAL) {
        auto shouldAvoidGoalPosts = true;
        std::optional<Vector2> predictedBallPositionTheirRobot = InterceptionComputations::calculateTheirBallInterception(info.getCurrentWorld(), ballTrajectory);
        return {calculateTargetPositionBallNotShot(info, predictedBallPositionTheirRobot), shouldAvoidGoalPosts, ai::constants::MAX_JERK_DEFAULT};
    }

    // Default case: go to the center of the goal
    auto shouldAvoidGoalPosts = true;
    return {Vector2(keepersLineSegment.start.x, 0), shouldAvoidGoalPosts, ai::constants::MAX_JERK_DEFAULT};
}

std::pair<Vector2, double> KeeperBlockBall::calculateTargetPositionBallShot(const StpInfo info, rtt::LineSegment keepersLineSegment, rtt::LineSegment ballTrajectory) noexcept {
    const auto &field = info.getField().value();
    const auto &ball = info.getBall().value();
    const auto &robot = info.getRobot().value();
    const auto robotPosition = robot->getPos();
    const auto robotVelocity = robot->getVel();
    const auto maxRobotVelocity = info.getMaxRobotVelocity();
    const auto maxRobotAcceleration = rtt::ai::constants::MAX_ACC;
    const auto closestPointToGoal = Line(ballTrajectory).intersect(Line(keepersLineSegment));

    // If possible, we intercept the ball at the line
    // if (closestPointToGoal.has_value()) {
    //     const auto ballTimeAtClosestPoint = FieldComputations::getBallTimeAtPosition(*ball.get(), closestPointToGoal.value());
    //     auto [targetPosition, timeToTarget] = control::OvershootComputations::overshootingDestination(robotPosition, closestPointToGoal.value(), robotVelocity, maxRobotVelocity,
    //                                                                                                   maxRobotAcceleration, ballTimeAtClosestPoint);
    //     if (timeToTarget <= ballTimeAtClosestPoint) {
    //         // TODO ROBOCUP 2024: TWEAK THIS VALUE
    //         auto jerk = (1 - std::min((ballTimeAtClosestPoint - timeToTarget), 0.2) / 0.2) * 160 + ai::constants::MAX_JERK_DEFAULT;
    //         return {targetPosition, jerk};
    //     }
    // }
    // if we are at most 0.03m away, set target to the closest point
    if (closestPointToGoal.has_value() && (closestPointToGoal.value() - robotPosition).length() <= 0.03) {
        return {closestPointToGoal.value(), ai::constants::MAX_JERK_DEFAULT};
    }

    double maxTimeLeftWhenArrived = std::numeric_limits<double>::lowest();
    Vector2 optimalTarget = Vector2();
    double jerk = ai::constants::MAX_JERK_DEFAULT;
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
            jerk = (1 - std::min(std::max(timeLeftWhenArrived, 0.0), 0.2) / 0.2) * 160 + ai::constants::MAX_JERK_DEFAULT;
        }
    }
    return {optimalTarget, jerk};
}

Vector2 KeeperBlockBall::calculateTargetPositionBallNotShot(const StpInfo &info, std::optional<Vector2> predictedBallPositionTheirRobot) noexcept {
    const auto &field = info.getField().value();
    const auto &ball = info.getBall().value();
    const auto &robot = info.getRobot().value();
    Vector2 leftGoalPost = field.leftGoalArea.topRight();
    Vector2 rightGoalPost = field.leftGoalArea.bottomRight();
    Vector2 targetFromPos = predictedBallPositionTheirRobot ? predictedBallPositionTheirRobot.value() : ball->position;
    std::array<rtt::Vector2, 2> rightGoalPostArr = {targetFromPos, rightGoalPost};
    std::array<rtt::Vector2, 2> leftGoalPostArr = {targetFromPos, leftGoalPost};
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

    Vector2 ballToLeftGoalPost = leftGoalPost - targetFromPos;
    Vector2 ballToRightGoalPost = rightGoalPost - targetFromPos;
    Vector2 bisectorUnitVector = (ballToLeftGoalPost.normalize() + ballToRightGoalPost.normalize()).normalize();
    auto scalingFactor = (field.leftGoalArea.rightLine().center().x - targetFromPos.x) / bisectorUnitVector.x;
    Vector2 bisectorPoint = targetFromPos + bisectorUnitVector * scalingFactor;

    std::array<rtt::Vector2, 2> bisectorArr = {targetFromPos, bisectorPoint};
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

    // project the leftGoalPost on the ball->pos bisectorPoint line if the ball has positive x, else project the rightGoalPost
    Vector2 targetGoalPost = targetFromPos.y >= 0 ? leftGoalPost : rightGoalPost;
    auto targetPositionNew = LineSegment(targetFromPos, bisectorPoint).project(targetGoalPost);
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
    return targetPositionNew;
}

Angle KeeperBlockBall::calculateYaw(const world::view::BallView &ball, const Vector2 &targetKeeperPosition) {
    // Look towards ball to ensure ball hits the front assembly to reduce odds of ball reflecting in goal
    const auto keeperToBall = (ball->position - targetKeeperPosition);
    return keeperToBall.angle();
}

}  // namespace rtt::ai::stp::tactic
