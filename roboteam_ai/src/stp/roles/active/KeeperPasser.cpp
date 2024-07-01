#include "stp/roles/active/KeeperPasser.h"

#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/GetBehindBallInDirection.h"
#include "stp/tactics/active/InstantKick.h"
#include "stp/tactics/active/DriveWithBall.h"
#include "gui/Out.h"

namespace rtt::ai::stp::role {

KeeperPasser::KeeperPasser(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBehindBallInDirection(), tactic::GetBall(), tactic::DriveWithBall(), tactic::InstantKick(), tactic::KeeperBlockBall()};
}

Status KeeperPasser::update(StpInfo const& info) noexcept {
    StpInfo skillStpInfo = info;
    if (info.getRobot() && info.getField() && info.getBall()) {
        auto ourDefenseArea = FieldComputations::getDefenseArea(*info.getField(), true, -0.4, 0.0);
        // Stop Formation tactic when ball is moving, start blocking, getting the ball and pass (normal keeper behavior)
        if (robotTactics.current_num() == 2 && ourDefenseArea.contains(info.getBall().value()->position)) {
            forceNextTactic();
        }
        else if (robotTactics.current_num() == 2) {
            auto intersections = ourDefenseArea.intersections(LineSegment(info.getField().value().leftGoalArea.leftLine().center(), info.getRobot().value()->getPos()));
            std::array<rtt::Vector2, 4> intersectionsTotal = {ourDefenseArea[0], ourDefenseArea[1], ourDefenseArea[2], ourDefenseArea[3]};
                rtt::ai::gui::Out::draw(
                {
                    .label = "Penalty op2",
                    .color = proto::Drawing::BLACK,
                    .method = proto::Drawing::LINES_CONNECTED,
                    .category = proto::Drawing::MARGINS,
                    .size = 12,
                    .thickness = 4,
                },
                intersectionsTotal);


            if (intersections.size() == 2) {
                Vector2 intersection_one = intersections[0];
                std::array<rtt::Vector2, 2> intersectionOne = {intersections[0], intersections[-1]};
                rtt::ai::gui::Out::draw(
                {
                    .label = "Penalty op",
                    .color = proto::Drawing::BLACK,
                    .method = proto::Drawing::CROSSES,
                    .category = proto::Drawing::MARGINS,
                    .size = 12,
                    .thickness = 4,
                },
                intersectionOne);
                skillStpInfo.setPositionToMoveTo(intersection_one);
            }
        }
    }

    return Role::update(skillStpInfo);
}
}  // namespace rtt::ai::stp::role
