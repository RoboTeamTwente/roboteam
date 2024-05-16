#include "stp/roles/Keeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/KeeperBlockBall.h"

namespace rtt::ai::stp::role {

Keeper::Keeper(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::KeeperBlockBall()}; }
}  // namespace rtt::ai::stp::role
