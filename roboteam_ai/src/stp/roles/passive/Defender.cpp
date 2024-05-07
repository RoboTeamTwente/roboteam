#include "stp/roles/passive/Defender.h"

#include "stp/tactics/passive/BlockBall.h"

namespace rtt::ai::stp::role {

Defender::Defender(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockBall()}; }
}  // namespace rtt::ai::stp::role
