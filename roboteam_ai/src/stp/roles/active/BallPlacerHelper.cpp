#include "stp/roles/active/BallPlacerHelper.h"

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/passive/BallStandBack.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

BallPlacerHelper::BallPlacerHelper(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::BallStandBack()};
}

Status BallPlacerHelper::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info for ", roleName)
        return Status::Failure;
    }

    auto robot = info.getRobot()->get();
    auto ball = info.getBall()->get();

    // Stop Get Ball tactic when we have the ball
    if (robotTactics.current_num() == 0 && robot->hasBall()) {
        std::shared_ptr<world::view::RobotView> placer = nullptr;
        for (const auto &bot : ballPlacerIt->second) {
            if (bot->getId() != robot->getId() && bot->hasBall()) {
                placer = bot;
                break;
            }
        }if (placer != nullptr){
            const auto &helperPos = robot->getPos();
            const auto &placerPos = placer->getPos();
            const auto &ballPos = ball->position;
            const auto distToPlacer = (helperPos - placerPos).length();
            const auto distToBall = (helperPos - ballPos).length();
            const auto distBallToPlacer = (placerPos - ballPos).length();
            bool isNearPlacer = distToPlacer < constants::ROBOT_RADIUS + 0.05;  
            bool isBallBetween = std::abs(distToPlacer - (distToBall + distBallToPlacer)) < 0.05;
            bool isFacingPlacer = std::abs((placerPos - helperPos).angle() - robot->getAngle()) < 0.2;
            if (isNearPlacer && isBallBetween && isFacingPlacer){
                forceNextTactic();
            }
        }
    }
    if (robotTactics.current_num() < 2 && (info.getBall()->get()->position - GameStateManager::getRefereeDesignatedPosition()).length() < constants::BALL_PLACER_MARGIN) {
        forceLastTactic();
    }

    return Role::update(info);
}
}  // namespace rtt::ai::stp::role
