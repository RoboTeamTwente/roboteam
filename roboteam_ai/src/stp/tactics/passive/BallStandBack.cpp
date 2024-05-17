#include "stp/tactics/passive/BallStandBack.h"

#include "stp/skills/GoToPos.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::tactic {

const int STAND_STILL_THRESHOLD = 60;

BallStandBack::BallStandBack() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BallStandBack::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo().has_value() || !skillStpInfo.getBall().has_value() || !skillStpInfo.getRobot().has_value()) return std::nullopt;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    Vector2 targetPosition;
    Vector2 ballTarget = info.getBall()->get()->position;
    auto robot = info.getRobot()->get();
    if (standStillCounter > STAND_STILL_THRESHOLD) {
        auto moveVector = robot->getPos() - ballTarget;
        double stretchLength =
            (currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT) ? control_constants::AVOID_BALL_DISTANCE_BEFORE_FREE_KICK : control_constants::AVOID_BALL_DISTANCE;
        targetPosition = ballTarget + moveVector.stretchToLength(stretchLength);
        bool isCloseToTarget = (robot->getPos() - targetPosition).length() < control_constants::GO_TO_POS_ERROR_MARGIN;
        if ((isCloseToTarget || standBack) && currentGameState != RefCommand::BALL_PLACEMENT_US_DIRECT) {
            targetPosition = ballTarget + (skillStpInfo.getField().value().leftGoalArea.leftLine().center() - ballTarget).stretchToLength(control_constants::AVOID_BALL_DISTANCE);
            standBack = true;
            skillStpInfo.setShouldAvoidBall(true);
        }
    } else {
        standBack = false;
        standStillCounter++;
        targetPosition = robot->getPos();
        skillStpInfo.setShouldAvoidBall(false);
    }

    double angle = (info.getBall()->get()->position - targetPosition).angle();
    skillStpInfo.setPositionToMoveTo(targetPosition);
    skillStpInfo.setAngle(angle);
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool BallStandBack::isTacticFailing(const StpInfo &info) noexcept {
    if (!info.getPositionToMoveTo().has_value() ||
        (info.getBall()->get()->position - GameStateManager::getRefereeDesignatedPosition()).length() > control_constants::BALL_PLACEMENT_MARGIN) {
        standStillCounter = 0;
        return true;
    }
    return false;
}

bool BallStandBack::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool BallStandBack::isEndTactic() noexcept { return true; }

const char *BallStandBack::getName() { return "Ball Stand Back"; }

}  // namespace rtt::ai::stp::tactic