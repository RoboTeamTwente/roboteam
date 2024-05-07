#include "stp/roles/active/FreeKickTaker.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/GetBehindBallInDirection.h"
#include "stp/tactics/active/InstantKick.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

FreeKickTaker::FreeKickTaker(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBehindBallInDirection(), tactic::GetBall(), tactic::InstantKick(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
