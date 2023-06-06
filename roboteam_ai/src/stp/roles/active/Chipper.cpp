//
// Created by doormat on 22-11-22.
//

#include "stp/roles/active/Chipper.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/ChipAtPos.h"
#include "stp/tactics/passive/Formation.h"
namespace rtt::ai::stp::role{

Chipper::Chipper(std::string name) : Role(std::move(name)) {
    //create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic,Status,StpInfo>{tactic::GetBall(),tactic::ChipAtPos(),tactic::Formation()};
}
}// namespace rtt::ai::stp::role