#include "stp/roles/active/Passer.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/OrbitKick.h"
#include "stp/tactics/active/SmoothPass.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Passer::Passer(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{
        tactic::GetBall(), 
        tactic::SmoothPass(), 
        tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
