#include "stp/plays/referee_specific/KickOffUs.h"

#include "stp/roles/Keeper.h"
#include "stp/roles/active/FreeKickTaker.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/passive/Halt.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::play {

KickOffUs::KickOffUs() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyDoNotHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::KickOffUsOrNormalGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"), std::make_unique<role::FreeKickTaker>("kick_off_taker"), std::make_unique<role::PassReceiver>("receiver"),
        std::make_unique<role::Halt>("halt_0"), std::make_unique<role::Halt>("halt_1"), std::make_unique<role::Halt>("halt_2"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_3"), std::make_unique<role::Halt>("halt_4"), std::make_unique<role::Halt>("halt_5"), std::make_unique<role::Halt>("halt_6"),
        std::make_unique<role::Halt>("halt_7")};
}

uint8_t KickOffUs::score(const rtt::Field &) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap KickOffUs::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    Dealer::DealerFlag kickerFlag(DealerFlagTitle::CAN_KICK_BALL);
    Dealer::DealerFlag detectionFlag(DealerFlagTitle::CAN_DETECT_BALL);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"kick_off_taker", {DealerFlagPriority::REQUIRED, {kickerFlag, detectionFlag}}});
    flagMap.insert({"receiver", {DealerFlagPriority::HIGH_PRIORITY, {detectionFlag}}});
    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void KickOffUs::calculateInfoForRoles() noexcept {
    // Kicker
    Vector2 receiverLocation = Vector2(-field.playArea.width() / 5, 0);
    stpInfos["kick_off_taker"].setPositionToShootAt(receiverLocation);
    stpInfos["kick_off_taker"].setShotType(ShotType::PASS);
    stpInfos["kick_off_taker"].setKickOrChip(KickOrChip::KICK);
    if (!ballKicked()) {
        stpInfos["receiver"].setPositionToMoveTo(receiverLocation);
    } else {
        stpInfos["kick_off_taker"].setPositionToMoveTo(PositionComputations::getPosition(std::nullopt, field.middleRightGrid, gen::AttackingPass, field, world));
        auto ball = world->getWorld()->getBall()->get();
        auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(field.playArea.width()));
        stpInfos["receiver"].setPositionToMoveTo(FieldComputations::projectPointToValidPositionOnLine(field, receiverLocation, ballTrajectory.start, ballTrajectory.end));
        stpInfos["receiver"].setPidType(ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT ? PIDType::RECEIVE : PIDType::DEFAULT);
    }
}

bool KickOffUs::shouldEndPlay() noexcept {
    if (stpInfos["receiver"].getRobot() && stpInfos["receiver"].getRobot().value()->hasBall()) return true;
    if (ballKicked() && world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT) return true;
    if (stpInfos["receiver"].getRobot() && world->getWorld()->getBall()->get()->position.x < stpInfos["receiver"].getRobot()->get()->getPos().x) return true;

    return false;
}

bool KickOffUs::ballKicked() {
    return std::any_of(roles.begin(), roles.end(), [](const std::unique_ptr<Role> &role) {
        return role != nullptr && role->getName() == "kick_off_taker" && strcmp(role->getCurrentTactic()->getName(), "Formation") == 0;
    });
}

const char *KickOffUs::getName() const { return "Kick Off Us"; }
}  // namespace rtt::ai::stp::play
