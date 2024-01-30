//
// Created by agata on 29/06/2022.
//

#include "stp/tactics/passive/BallStandBack.h"

#include "stp/skills/GoToPos.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::tactic {

BallStandBack::BallStandBack() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> BallStandBack::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!info.getPositionToMoveTo() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;
    RefCommand currentGameState = GameStateManager::getCurrentGameState().getCommandId();
    Vector2 targetPos;
    if (standStillCounter > 60) {
        auto moveVector = info.getRobot()->get()->getPos() - info.getBall()->get()->position;
        double stretchLength =
            (currentGameState == RefCommand::BALL_PLACEMENT_US_DIRECT) ? control_constants::AVOID_BALL_DISTANCE_BEFORE_FREE_KICK : control_constants::AVOID_BALL_DISTANCE;
        targetPos = info.getBall()->get()->position + moveVector.stretchToLength(stretchLength);
    } else {
        standStillCounter++;
        targetPos = info.getRobot()->get()->getPos();
    }
    skillStpInfo.setShouldAvoidBall(false);
    double angle = (info.getBall()->get()->position - targetPos).angle();
    skillStpInfo.setPositionToMoveTo(targetPos);
    skillStpInfo.setAngle(angle);
    // Be 100% sure the dribbler is off during the BallStandBack
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

bool BallStandBack::isTacticFailing(const StpInfo &info) noexcept {
    // BallStandBack tactic fails if there is no location to move to or if the ball is not close enough to the designated position
    if (!info.getPositionToMoveTo() || (info.getBall()->get()->position - GameStateManager::getRefereeDesignatedPosition()).length() > control_constants::BALL_PLACEMENT_MARGIN) {
        standStillCounter = 0;
        return true;
    }
    return false;
}

bool BallStandBack::shouldTacticReset(const StpInfo &) noexcept { return false; }

bool BallStandBack::isEndTactic() noexcept {
    // BallStandBack tactic is an end tactic
    return true;
}

const char *BallStandBack::getName() { return "Ball Stand Back"; }

}  // namespace rtt::ai::stp::tactic
