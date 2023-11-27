//
// Created by tijmen on 14-07-22.
//

#include "stp/roles/active/KeeperPasser.h"

#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/KickAtPos.h"

namespace rtt::ai::stp::role {

KeeperPasser::KeeperPasser(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
//    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos(), tactic::KeeperBlockBall()};
}
}  // namespace rtt::ai::stp::role
