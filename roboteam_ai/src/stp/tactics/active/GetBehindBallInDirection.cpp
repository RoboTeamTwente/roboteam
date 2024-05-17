#include "stp/tactics/active/GetBehindBallInDirection.h"

#include <roboteam_utils/LineSegment.h>

#include "roboteam_utils/Circle.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Orbit.h"

namespace rtt::ai::stp::tactic {

constexpr double DISTANCE_THRESHOLD = 0.5;
constexpr double BALL_AVOID_DISTANCE = 4 * control_constants::ROBOT_RADIUS;

GetBehindBallInDirection::GetBehindBallInDirection() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Orbit()}; }

std::optional<StpInfo> GetBehindBallInDirection::calculateInfoForSkill(const StpInfo &info) noexcept {
    if (!info.getRobot() || !info.getBall() || !info.getPositionToShootAt()) return std::nullopt;

    StpInfo skillStpInfo = info;
    Vector2 robotPosition = info.getRobot()->get()->getPos();
    Vector2 ballPosition = info.getBall()->get()->position;
    Vector2 positionToShootAt = info.getPositionToShootAt().value();

    skillStpInfo.setAngle((positionToShootAt - robotPosition).angle());

    if ((ballPosition - robotPosition).length() > DISTANCE_THRESHOLD || info.getObjectsToAvoid().shouldAvoidBall) {
        Vector2 targetPos = calculateTargetPosition(ballPosition, robotPosition, positionToShootAt);
        skillStpInfo.setPositionToMoveTo(targetPos);
    } else if (skills.current_num() == 0) {
        skills.skip_to(1);
    }

    return skillStpInfo;
}

Vector2 GetBehindBallInDirection::calculateTargetPosition(Vector2 ballPosition, Vector2 robotPosition, Vector2 positionToShootAt) {
    Vector2 ballToTarget = positionToShootAt - ballPosition;
    Vector2 targetPos = ballPosition - ballToTarget.stretchToLength(BALL_AVOID_DISTANCE);

    Vector2 robotToTarget = targetPos - robotPosition;
    if (!Circle(ballPosition, BALL_AVOID_DISTANCE).intersects(LineSegment(robotPosition, targetPos)).empty()) {
        double direction = ballToTarget.toAngle().rotateDirection(robotToTarget) ? 1.0 : -1.0;
        Vector2 finalPos = ballPosition + robotToTarget.rotate(M_PI_2).stretchToLength(BALL_AVOID_DISTANCE) * direction;
        return finalPos;
    }

    return targetPos;
}

bool GetBehindBallInDirection::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt(); }

bool GetBehindBallInDirection::shouldTacticReset(const StpInfo &info) noexcept {
    return ((info.getBall()->get()->position - info.getRobot()->get()->getPos()).length() > DISTANCE_THRESHOLD) && (skills.current_num() == 1);
}

bool GetBehindBallInDirection::isEndTactic() noexcept { return false; }

const char *GetBehindBallInDirection::getName() { return "Get Ball In Direction"; }

}  // namespace rtt::ai::stp::tactic