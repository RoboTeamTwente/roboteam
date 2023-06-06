//
// Created by jibbe on 6-6-23.
//

#include "stp/roles/active/Assistant.hpp"
#include "stp/tactics/active/Receive.h"
#include "stp/tactics/active/KickAtPos.h"

namespace rtt::ai::stp::role {

Assistant::Assistant(std::string name) : Role(name) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive(), tactic::KickAtPos()};
}

} //namespace rtt::ai::stp::role