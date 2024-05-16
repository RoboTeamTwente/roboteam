#include "stp/roles/active/BallPlacer.h"

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/passive/BallStandBack.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

BallPlacer::BallPlacer(std::string name) : Role(std::move(name)) {
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::BallStandBack()};
}
}  // namespace rtt::ai::stp::role
