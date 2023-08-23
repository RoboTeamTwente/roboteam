#include "AIData.hpp"

namespace rtt {

bool RobotPath::operator==(const RobotPath &other) const { return this->robotId == other.robotId && this->points == other.points; }

bool RobotSTP::operator==(const RobotSTP &other) const {
    return this->robotId == other.robotId && this->role == other.role && this->roleStatus == other.roleStatus && this->tactic == other.tactic &&
           this->tacticStatus == other.tacticStatus && this->skill == other.skill && this->skillStatus == other.skillStatus;
}

bool AIData::operator==(const AIData &other) const { return this->robotStps == other.robotStps && this->robotPaths == other.robotPaths; }

}  // namespace rtt