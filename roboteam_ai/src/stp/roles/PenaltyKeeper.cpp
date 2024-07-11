#include "stp/roles/PenaltyKeeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

// The PenaltyKeeper is a specialized version of the Keeper class, inheriting all its functions. The key difference lies in the initial movement restrictions during a penalty. The
// PenaltyKeeper must remain on the goal line until the ball has moved at least 0.05 meters. After this, the PenaltyKeeper can move freely. To avoid computational delays, we allow
// the PenaltyKeeper to move as soon as the ball starts moving. Once this condition is met, the PenaltyKeeper behaves like a regular Keeper.

PenaltyKeeper::PenaltyKeeper(std::string name) : Keeper(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation(), tactic::KeeperBlockBall(), tactic::Formation()};
}

Status PenaltyKeeper::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField() || !info.getCurrentWorld()) {
        RTT_WARNING("Required information missing in the tactic info for ", roleName)
        return Status::Failure;
    }

    // Stop Formation tactic when ball is moving, start blocking, getting the ball and pass (normal keeper behavior)
    if (robotTactics.current_num() == 0 && info.getBall().value()->velocity.length() > constants::BALL_STILL_VEL) {
        forceNextTactic();
    }
    if (info.getField().value().leftDefenseArea.rightLine().distanceToLine(info.getBall().value()->position) < 0.8 &&
        info.getField().value().leftDefenseArea.rightLine().center().x < info.getBall().value()->position.x &&
        info.getBall().value()->velocity.length() < constants::BALL_GOT_SHOT_LIMIT) {
        // If the ball is getting close to our goal, get out of the goal to increase our chances of saving it
        forceLastTactic();
    } else if (robotTactics.current_num() == 2)
        reset();

    return Role::update(info);
}
}  // namespace rtt::ai::stp::role
