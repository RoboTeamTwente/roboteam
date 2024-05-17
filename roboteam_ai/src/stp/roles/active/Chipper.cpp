#include "stp/roles/active/Chipper.h"

#include "stp/tactics/active/ChipAtPos.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/passive/Formation.h"
namespace rtt::ai::stp::role {

Chipper::Chipper(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::ChipAtPos(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role