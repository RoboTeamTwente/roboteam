#include "stp/roles/active/Striker.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/OrbitKick.h"

namespace rtt::ai::stp::role {

Striker::Striker(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::OrbitKick()}; }
}  // namespace rtt::ai::stp::role
