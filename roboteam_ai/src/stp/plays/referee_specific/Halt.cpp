//
// Created by jessevw on 24.03.20.
//

#include "stp/plays/referee_specific/Halt.h"

#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {

Halt::Halt() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::HaltGameState);

    // Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Halt>("halt_0"), std::make_unique<role::Halt>("halt_1"), std::make_unique<role::Halt>("halt_2"), std::make_unique<role::Halt>("halt_3"),
        std::make_unique<role::Halt>("halt_4"), std::make_unique<role::Halt>("halt_5"),
        // Additional roles if we play 11v11
        std::make_unique<role::Halt>("halt_6"), std::make_unique<role::Halt>("halt_7"), std::make_unique<role::Halt>("halt_8"), std::make_unique<role::Halt>("halt_9"),
        std::make_unique<role::Halt>("halt_10")};
}

uint8_t Halt::score(const rtt::Field &) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap Halt::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"halt_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_8", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_9", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"halt_10", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void Halt::calculateInfoForRoles() noexcept {}

const char *Halt::getName() const { return "Halt"; }
}  // namespace rtt::ai::stp::play
