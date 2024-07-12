#include "stp/roles/active/BallPlacer.h"

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/passive/BallStandBack.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

BallPlacer::BallPlacer(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::BallStandBack()};
}

Status BallPlacer::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info for ", roleName)
        return Status::Failure;
    }

    // Stop Get Ball tactic when we have the ball
    if (robotTactics.current_num() == 0 && info.getRobot()->get()->hasBall()) {
        RTT_INFO("FORCING NEXT TACTIC CAUSAE ROBOT HAS THE BALL WOOP WOOP!!!!!!!!!")
        forceNextTactic();
    }

    return Role::update(info);
}
}  // namespace rtt::ai::stp::role
