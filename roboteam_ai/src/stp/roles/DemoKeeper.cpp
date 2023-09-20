//
// Created by Luuk and Jorn on 14/09/2023.
//

#include "stp/roles/DemoKeeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/active/DemoKeeperBlockBall.h"
#include "stp/tactics/active/DemoKeeperMove.h"
namespace rtt::ai::stp::role {

DemoKeeper::DemoKeeper(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::DemoKeeperBlockBall(), tactic::DemoKeeperMove()};
}
}  // namespace rtt::ai::stp::role
