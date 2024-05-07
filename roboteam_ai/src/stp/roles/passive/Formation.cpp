#include "stp/roles/passive/Formation.h"

#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Formation::Formation(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation()}; }
}  // namespace rtt::ai::stp::role
