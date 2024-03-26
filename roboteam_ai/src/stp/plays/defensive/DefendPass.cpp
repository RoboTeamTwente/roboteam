#include "stp/plays/defensive/DefendPass.h"

#include <world/views/RobotView.hpp>

#include "roboteam_utils/Hungarian.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/Harasser.h"
#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    // Evaluations that have to be true in order for this play to be considered valid.
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    startPlayEvaluation.emplace_back(GlobalEvaluation::WeDoNotHaveBall);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallOnTheirSide);
    startPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(GlobalEvaluation::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::TheyHaveBall);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallOnTheirSide);
    keepPlayEvaluation.emplace_back(GlobalEvaluation::BallNotInOurDefenseAreaAndStill);

    // Role creation, the names should be unique. The names are used in the stpInfos-map
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        // Roles is we play 6v6
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Harasser>("harasser"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Formation>("waller_0"),
        std::make_unique<role::Defender>("defender_2"),
        // Additional roles if we play 11v11
        std::make_unique<role::Defender>("defender_3"),
        std::make_unique<role::Formation>("attacker_0"),
        std::make_unique<role::Formation>("waller_1"),
        std::make_unique<role::Defender>("defender_4"),
        std::make_unique<role::Defender>("defender_5"),
    };
}

uint8_t DefendPass::score(const rtt::Field&) noexcept {
    // If this play is valid we always want to execute this play
    return control_constants::FUZZY_TRUE;
}

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {keeperFlag}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {}, harasserInfo.interceptId}});
    flagMap.insert({"defender_0", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"defender_5", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"waller_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"waller_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"attacker_0", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    harasserInfo = PositionComputations::calculateHarasserId(world, field);
    PositionComputations::calculateInfoForHarasser(stpInfos, &roles, field, world, harasserInfo.interceptLocation);
    PositionComputations::calculateInfoForDefendersAndWallers(stpInfos, roles, field, world, false);
    PositionComputations::calculateInfoForAttackers(stpInfos, roles, field, world);
}

const char* DefendPass::getName() const { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play