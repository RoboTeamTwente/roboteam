#include "stp/tactics/passive/BallStandBack.h"

#include "stp/skills/GoToPos.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::tactic {

const int STAND_STILL_THRESHOLD = 60;
const int MOVE_AFTER_DRIBBLER_OFF = 20;

BallStandBack::BallStandBack() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> BallStandBack::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    Vector2 targetPosition;
    Vector2 ballTarget = info.getBall()->get()->position;
    auto robot = info.getRobot()->get();
    if (standStillCounter > STAND_STILL_THRESHOLD) {
        skillStpInfo.setDribblerOn(false);
        if (standStillCounter > STAND_STILL_THRESHOLD + MOVE_AFTER_DRIBBLER_OFF) {
            auto moveVector = robot->getPos() - ballTarget;
            double stretchLength = (currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT) ? constants::AVOID_BALL_DISTANCE_BEFORE_FREE_KICK : constants::AVOID_BALL_DISTANCE;
            targetPosition = ballTarget + moveVector.stretchToLength(stretchLength);
            bool isCloseToTarget = (robot->getPos() - targetPosition).length() < constants::GO_TO_POS_ERROR_MARGIN;
            if ((isCloseToTarget || standBack) && currentGameState != RefCommand::BALL_PLACEMENT_US_DIRECT) {
                targetPosition = ballTarget + (skillStpInfo.getField().value().leftGoalArea.rightLine().center() - ballTarget)
                                                  .stretchToLength(constants::AVOID_BALL_DISTANCE + constants::GO_TO_POS_ERROR_MARGIN);
                standBack = true;
                skillStpInfo.setShouldAvoidBall(true);
            }
        } else {
            standStillCounter++;
            targetPosition = robot->getPos();
            skillStpInfo.setShouldAvoidBall(false);
        }
    } else {
        skillStpInfo.setDribblerOn(true);
        standBack = false;
        standStillCounter++;
        targetPosition = robot->getPos();
        skillStpInfo.setShouldAvoidBall(false);
    }

    double yaw = (info.getBall()->get()->position - targetPosition).angle();
    skillStpInfo.setPositionToMoveTo(targetPosition);
    skillStpInfo.setYaw(yaw);

    return skillStpInfo;
}

bool BallStandBack::isTacticFailing(const StpInfo &info) noexcept {
    if (!info.getPositionToMoveTo() || (info.getBall()->get()->position - GameStateManager::getRefereeDesignatedPosition()).length() > constants::BALL_PLACEMENT_MARGIN) {
        standStillCounter = 0;
        return true;
    }
    return false;
}

bool BallStandBack::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool BallStandBack::isEndTactic() noexcept { return true; }

const char *BallStandBack::getName() { return "Ball Stand Back"; }

}  // namespace rtt::ai::stp::tactic