#include "stp/roles/active/PenaltyTaker.h"

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/OrbitKick.h"

namespace rtt::ai::stp::role {

PenaltyTaker::PenaltyTaker(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::OrbitKick()}; }

[[nodiscard]] Status PenaltyTaker::update(StpInfo const& info) noexcept {
    StpInfo infoCopy = info;
    if (!info.getRobot()->get()->hasBall()) {
        reset();
    }
    return Role::update(infoCopy);
}
}  // namespace rtt::ai::stp::role