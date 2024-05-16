#include "control/positionControl/pathPlanning/PathPlanningAlgorithm.h"

namespace rtt::ai::control {
PathPlanningAlgorithm::PathPlanningAlgorithm(CollisionDetector &collisionDetector) : collisionDetector{collisionDetector} {}
}  // namespace rtt::ai::control