#include "stp/roles/active/KeeperPasser.h"

#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/GetBehindBallInDirection.h"
#include "stp/tactics/active/InstantKick.h"

namespace rtt::ai::stp::role {

KeeperPasser::KeeperPasser(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBehindBallInDirection(), tactic::GetBall(), tactic::DriveWithBall(), tactic::InstantKick(),
                                                                       tactic::KeeperBlockBall()};
}

Status KeeperPasser::update(StpInfo const& info) noexcept {
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info for ", roleName)
        return Status::Failure;
    }

    StpInfo skillStpInfo = info;
    auto shootArea = FieldComputations::getDefenseArea(*info.getField(), true, -0.4, 0.0);
    // Stop Formation tactic when ball is moving, start blocking, getting the ball and pass (normal keeper behavior)
    if (robotTactics.current_num() == 2 && shootArea.contains(info.getBall().value()->position)) {
        forceNextTactic();
    } else if (robotTactics.current_num() == 2) {
        auto intersections = shootArea.intersections(LineSegment(info.getField().value().leftGoalArea.leftLine().center(), info.getRobot().value()->getPos()));

        if (intersections.size() == 2) {
            Vector2 intersection_one = intersections[0];

            skillStpInfo.setPositionToMoveTo(intersection_one);
        }
    }

    return Role::update(skillStpInfo);
}
}  // namespace rtt::ai::stp::role
