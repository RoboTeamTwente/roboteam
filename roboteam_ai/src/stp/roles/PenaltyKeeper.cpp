#include "stp/roles/PenaltyKeeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

//
// PenaltyKeeper is a subclass of Keeper and therefore inherit all its functions
// The difference between a normal Keeper and PenaltyKeeper is that the PenaltyKeeper
// has to stay on the goal line until the ball has moved 0.05 meters, afterwards it
// can move freely.
// Since calculating exactly when the ball has moved 0.05 meters takes too long, we
// let the PenaltyKeeper move after the ball has moved. If this requirement is fullfilled
// the keeper will behave as a normal keeper
//

PenaltyKeeper::PenaltyKeeper(std::string name) : Keeper(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation(), tactic::KeeperBlockBall()};
}

Status PenaltyKeeper::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info for ", roleName)
        return Status::Failure;
    }

    // Stop Formation tactic when ball is moving, start blocking, getting the ball and pass (normal keeper behavior)
    if (robotTactics.current_num() == 0 && info.getBall().value()->velocity.length() > control_constants::BALL_STILL_VEL) {
        forceNextTactic();
    }

    return Role::update(info);
}
}  // namespace rtt::ai::stp::role
