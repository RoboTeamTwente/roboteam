//
// Created by tijmen on 27-10-21.
//

#ifndef RTT_BBTPATHTRACKING_H
#define RTT_BBTPATHTRACKING_H

#include <span>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/pid.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

enum UpdatePath {
    DONT_UPDATE,
    UPDATE_TARGET_CHANGED,
    UPDATE_TARGET_REACHED,

    UPDATE_POSITION_CHANGED,

    UPDATE_COLLISION_DETECTED,
};

inline std::ostream& operator<<(std::ostream& os, const UpdatePath updatePath) {
    switch (updatePath) {
        case UpdatePath::DONT_UPDATE:
            os << "DONT_UPDATE";
            break;
        case UpdatePath::UPDATE_TARGET_CHANGED:
            os << "UPDATE_TARGET_CHANGED";
            break;
        case UpdatePath::UPDATE_TARGET_REACHED:
            os << "UPDATE_TARGET_REACHED";
            break;
        case UpdatePath::UPDATE_COLLISION_DETECTED:
            os << "UPDATE_COLLISION_DETECTED";
            break;
    }
    return os;
}

/**
 * Path tracker for single robot.
 */
class PathTracking {
   private:
    static constexpr size_t STEPS_AHEAD = 1;
    const CollisionDetector& collisionDetector;

   public:
    explicit PathTracking(const CollisionDetector& collisionDetector);

    /**
     * \brief Determine if the path needs to be recomputed (e.g. when target has changed)
     * @param currentPos Current position of the robot
     * @param targetPos Target position of the robot from STP
     * @param avoidObjects Objects deemed to be obstacles for collision detection
     *
     * @returns UpdatePath::DONT_UPDATE (i.e. 0) if the path is up to date else ret_val > 0
     */
    [[nodiscard]] UpdatePath shouldUpdatePath(const PositionControlInput& input, std::span<StateVector> remainingPath) const;

    /**
     * \brief Computes and returns next position and angle to be send to the robot
     * @param currentPos Current position of the robot
     * @param currentVel Current velocity of the robot
     * @param pidType PID type to be used (TODO: What is pid type?)
     * @returns Next position and velocity to be send to the robot
     */
    [[nodiscard]] std::pair<std::span<StateVector>, Position> trackPath(const StateVector& currentState, std::span<StateVector> remainingPath, std::pair<PID, PID> pidControllers, stp::PIDType pidType) const;
};

}  // namespace rtt::ai::control

#endif  // RTT_BBTPATHTRACKING_H
