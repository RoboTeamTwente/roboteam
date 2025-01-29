#include "control/ControlUtils.h"

#include <roboteam_utils/Grid.h>

#include <roboteam_utils/Field.hpp>

#include "utilities/Constants.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/StpInfoEnums.h"
#include "world/World.hpp"

namespace rtt::ai::control {

double ControlUtils::getMaxVelocity(bool hasBall) {
    double maxVel = rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().getMaxRobotVel();
    if (hasBall) maxVel = std::min(constants::MAX_VEL_WHEN_HAS_BALL, maxVel);
    return maxVel;
}

/// Calculate the kick force
double ControlUtils::determineKickForce(const double distance, stp::ShotPower shotPower) noexcept {
    if (shotPower == stp::ShotPower::MAX) return constants::MAX_KICK_POWER;

    double kickForce;
    if (shotPower == stp::ShotPower::PASS) {
        kickForce = 1.8;
    } else if (shotPower == stp::ShotPower::TARGET) {
        kickForce = 1.2;
    } else if (shotPower == stp::ShotPower::KICKOFF) {
        kickForce = 0.8;
    } else {
        RTT_WARNING("No shotPower set! Setting kickForce to 0")
        kickForce = 0;
    }
    // Calculate the velocity based on this function with the previously set limitingFactor
    auto velocity = distance * kickForce;

    // Make sure velocity is always between MIN_KICK_POWER and MAX_KICK_POWER
    return std::clamp(velocity, constants::MIN_KICK_POWER, constants::MAX_KICK_POWER);
}
/// Calculates the chip force
double ControlUtils::determineChipForce(const double distance) noexcept {
    // Factor that determines the chipping force
    double chipFactor = 3.0;
    // Calculate the velocity based on this function with the previously set limitingFactor
    auto velocity = distance * chipFactor;
    // Make sure velocity is always between MIN_CHIP_POWER and MAX_CHIP_POWER
    return std::clamp(velocity, constants::MIN_CHIP_POWER, constants::MAX_CHIP_POWER);
}
}  // namespace rtt::ai::control
