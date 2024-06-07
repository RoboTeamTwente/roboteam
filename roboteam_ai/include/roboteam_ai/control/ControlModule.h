#ifndef RTT_CONTROLMODULE_H
#define RTT_CONTROLMODULE_H

#include <mutex>
#include <roboteam_utils/RobotCommands.hpp>

#include "control/YawPID.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * @author Jaro Kuiken
 * @brief Control Module as seen in the system architecture.
 * Its job: Receive RobotCommands from Skills, check these and limit and/or change things wherever necessary.
 * For now, it limits the velocity and acceleration that a Skill sends so the robot doesn't do a backflip.
 * And it checks if the command that was sent makes sense (e.g., no kicking and chipping at the same time.
 */
class ControlModule {
   protected:
    static inline std::vector<rtt::RobotCommand> robotCommands;      /**< Vector of all robot commands */
    static inline std::map<unsigned int, YawPID> simulatorYawPIDmap; /**< Yaw controller for each robot */

    /**
     * @brief Applies constraints to the internal robot command
     * @param command Robot command that needs to be checked
     * @param robot Info about the robot
     */
    static void limitRobotCommand(rtt::RobotCommand& command, rtt::world::view::RobotView robot);

    /**
     * @brief Limits the velocity with a control_constants value
     * @param command Robot command that needs to be checked
     */
    static void limitVel(rtt::RobotCommand& command);

    /**
     * @brief Limits the angular velocity with a control_constants value
     * @param command Robot command that needs to be checked
     * @param robot Info about the robot
     */
    static void limitAngularVel(rtt::RobotCommand& command, rtt::world::view::RobotView robot);

    /**
     * @brief Rotates the robot command to the other side of the field
     * @param command Robot command that needs to be checked
     */
    static void rotateRobotCommand(rtt::RobotCommand& command);

   public:
    /**
     * @brief Limits the current robot command and adds it to the list of commands to be sent
     * @param command Robot command that needs to be added
     * @param robot Info about the robot
     */
    static void addRobotCommand(std::optional<rtt::world::view::RobotView> robot, rtt::RobotCommand command) noexcept;
    /**
     * @brief Sends all commands to robothub
     */
    static void sendAllCommands();

    /**
     * @brief Adjust the angular velocity to the simulator
     * @param robot Info about the robot
     * @param robot_command Robot command that needs to be adjusted
     */
    static void simulator_angular_control(const std::optional<::rtt::world::view::RobotView>& robot, rtt::RobotCommand& robot_command);
};
}  // namespace rtt::ai::control

#endif  // RTT_CONTROLMODULE_H
