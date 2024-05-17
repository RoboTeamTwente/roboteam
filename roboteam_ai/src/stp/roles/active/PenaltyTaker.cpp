#include "stp/roles/active/PenaltyTaker.h"

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/OrbitKick.h"

namespace rtt::ai::stp::role {

PenaltyTaker::PenaltyTaker(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::OrbitKick()};
}

[[nodiscard]] Status PenaltyTaker::update(StpInfo const& info) noexcept {
    StpInfo infoCopy = info;
    if (info.getRobot().has_value()) {
        static double distanceDriven = 0.0;
        static Vector2 lastPosition;
        auto currentPos = info.getRobot()->get()->getPos();
        auto currentVel = info.getRobot()->get()->getVel();
        distanceDriven += (currentPos - lastPosition).length();
        lastPosition = currentPos;
        if (distanceDriven >= 0.5 && (info.getPositionToMoveTo().value() - currentPos).length() > 0.4) {
            // set position to move to the current position
            infoCopy.setPositionToMoveTo(currentPos - Vector2(0.1, 0));
            if (currentVel.x < 0.1) {
                infoCopy.setDribblerSpeed(0);
            }
        }
        // if we don't have ball, reset the tactic
        if (!info.getRobot()->get()->hasBall()) {
            distanceDriven = 0.0;
            reset();
        }
    }

    return Role::update(infoCopy);
}
}  // namespace rtt::ai::stp::role