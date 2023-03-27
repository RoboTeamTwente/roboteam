#include "stp/roles/active/PenaltyTaker.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/OrbitKick.h"

namespace rtt::ai::stp::role {

PenaltyTaker::PenaltyTaker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::OrbitKick()};
}
}  // namespace rtt::ai::stp::role
