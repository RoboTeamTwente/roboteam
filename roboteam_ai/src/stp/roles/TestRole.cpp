#include "stp/roles/TestRole.h"

#include "stp/tactics/TestTactic.h"

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{TestTactic()}; }
}  // namespace rtt::ai::stp
