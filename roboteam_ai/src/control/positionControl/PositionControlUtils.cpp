//
// Created by ratoone on 30-01-20.
//

#include "control/positionControl/PositionControlUtils.h"

#include "utilities/Constants.h"

namespace rtt::ai::control {

bool PositionControlUtils::positionWithinTolerance(const Vector2 &pos1, const Vector2 &pos2) { return pos1.dist(pos2) < stp::control_constants::GO_TO_POS_ERROR_MARGIN; }

bool PositionControlUtils::isMoving(const Vector2 &velocity) { return velocity.length() > MAX_STALE_VELOCITY; }

pidVals PositionControlUtils::getPIDValue(const stp::PIDType &pidType) {
    switch (pidType) {
        case stp::PIDType::DEFAULT:
            // TODO: Rename numTreePID
            return interface::Output::getNumTreePid();
        case stp::PIDType::RECEIVE:
            return interface::Output::getReceivePid();
        case stp::PIDType::INTERCEPT:
            return interface::Output::getInterceptPid();
        case stp::PIDType::KEEPER:
            return interface::Output::getKeeperPid();
        case stp::PIDType::KEEPER_INTERCEPT:
            return interface::Output::getKeeperInterceptPid();
    }
}
double PositionControlUtils::convertStepToTime(int step) { return step * TIME_STEP; }

int PositionControlUtils::convertTimeToStep(double time) { return static_cast<int>(std::round(time / TIME_STEP)); }

}  // namespace rtt::ai::control