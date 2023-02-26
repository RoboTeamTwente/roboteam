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

struct PositionControlCommand {
    RobotCommand robotCommand;
    bool isOccupied = false;
};

struct ComputedPath {
    std::vector<StateVector> full;
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

    std::vector<Vector2> drawingBuffer;

   public:
    explicit PositionControl();

    /**
     * @brief Generates a collision-free trajectory and tracks it.
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity that the robot is allowed to have
     * @param pidType The desired PID type (intercept, regular, keeper etc.)
     * @param avoidObjects The objects that the robot should avoid
     * @return A PositionControlCommand containing the robot command and path flags
     */
    PositionControlCommand computeNextControlCommand(const PositionControlInput& input, stp::PIDType pidType);

    void updatePositionControl(std::optional<WorldDataView> world, std::optional<FieldView> field, const GameStateView& gameState);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
