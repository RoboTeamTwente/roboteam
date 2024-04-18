#ifndef RTT_COLLISIONCALCULATIONS_H
#define RTT_COLLISIONCALCULATIONS_H

#include <roboteam_utils/RobotCommands.hpp>

#include "control/positionControl/BBTrajectories/Trajectory2D.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/StpInfoEnums.h"
#include "world/FieldComputations.h"

namespace rtt::ai::control {

/**
 * @brief This class is responsible for performing collision calculations.
 */
class CollisionCalculations {
   private:
   public:
    static bool isTrajectoryAcceptable(const Trajectory2D &Trajectory);
    static bool isCollidingWithMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId, int &completedTimeSteps);
    static double getFirstCollisionTimeMotionlessObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId,
                                                        int &completedTimeSteps);
    static bool isCollidingWithMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId, int &completedTimeSteps);
    static double getFirstCollisionTimeMovingObject(const Trajectory2D &Trajectory, stp::AvoidObjects avoidObjects, const rtt::Field &field, int &robotId, int &completedTimeSteps);
};
}  // namespace rtt::ai::control

#endif  // RTT_COLLISIONCALCULATIONS_H
