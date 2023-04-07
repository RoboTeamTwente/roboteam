//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include <queue>
#include <roboteam_utils/RobotCommands.hpp>

#include "control/positionControl/CollisionDetector.h"
#include "control/positionControl/PathPlanning.h"
#include "control/positionControl/PathTracking.h"
#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

/**
 * This struct represents a computed path for a robot.
 * The full path is stored as a vector of StateVector objects, while the remaining path is stored as a span
 * (a non-owning view) of the full vector.
 */
struct ComputedPath {
    // The entire path from the origin (where the robot was when the path was computed) to the target
    std::vector<StateVector> full;

    // The remaining part of the path that the robot has to travel over
    std::span<StateVector> remaining;
};

/**
 * The main position control class. Use this for your robot position control
 * requirements.
 */
class PositionControl {
   private:
    using WorldDataView = const rtt::world::view::WorldDataView;
    using FieldView = const rtt::Field;
    using GameStateView = const rtt::ai::GameState;

    CollisionDetector collisionDetector;
    PathTracking pathTracking;
    PathPlanning pathPlanning;

    std::unordered_map<int, ComputedPath> paths;
    std::unordered_map<int, std::pair<PID, PID>> pidControllers;

   public:
    explicit PositionControl();

    /**
     * @brief This function computes the next control command for a robot based on the given input and PID type.
     * @param input PositionControlInput object containing information about the robot's current/desired position and other path constraints (such as the avoidance of moving
     * objects)
     * @param pidType The PID type to use for the path tracking
     * @return Returns next position or nullopt if target position is not reachable (e.g. is occupied)
     */
    [[nodiscard]] std::optional<Position> computeNextPosition(const PositionControlInput& input, stp::PIDType pidType) noexcept;

    /**
     * @brief This function updates the position control with the latest world data and field data. **It is called once at the start of each tick**.
     * @param world The latest world data
     * @param field The latest field data
     * @param gameState The latest game state
     */
    void updatePositionControl(std::optional<WorldDataView> world, std::optional<FieldView> field, const GameStateView& gameState);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
