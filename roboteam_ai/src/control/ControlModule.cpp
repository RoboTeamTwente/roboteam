//
// Created by jaro on 15-10-20.
//

#include "control/ControlModule.h"

#include "control/ControlUtils.h"
#include "iostream"
#include "utilities/Constants.h"
#include "utilities/GameSettings.h"
#include "utilities/IOManager.h"
#include "world/World.hpp"

namespace rtt::ai::control {

void ControlModule::rotateRobotCommand(rtt::RobotCommand& command) {
    command.velocity.x = -command.velocity.x;
    command.velocity.y = -command.velocity.y;
    command.targetAngle += M_PI;
}

void ControlModule::limitRobotCommand(rtt::RobotCommand& command, rtt::world::view::RobotView robot) {
    limitVel(command);
    limitAngularVel(command, robot);
}

void ControlModule::limitVel(rtt::RobotCommand& command) {
    command.velocity = command.velocity.stretchToLength(std::clamp(command.velocity.length(), 0.0, Constants::MAX_VEL_CMD()));
}

void ControlModule::limitAngularVel(rtt::RobotCommand& command, rtt::world::view::RobotView robot) {
    // Limit the angular velocity when the robot has the ball by setting the target angle in small steps
    if (robot->hasBall() && !command.useAngularVelocity) {
        auto targetAngle = command.targetAngle;
        auto robotAngle = robot->getAngle();

        // Adjust robot angle if game is not on the left
        if (!GameSettings::isLeft()) {
            robotAngle += M_PI;
        }

        // If the angle error is larger than the desired angle rate, adjust the angle command
        if (robotAngle.shortestAngleDiff(targetAngle) > stp::control_constants::ANGLE_RATE) {
            // Determine direction of rotation (shortest distance)
            int direction = Angle(robotAngle).rotateDirection(targetAngle) ? 1 : -1;

            // Set the angle command to the current robot angle + the angle rate
            command.targetAngle = robotAngle + Angle(direction * stp::control_constants::ANGLE_RATE);
        }
    }
    // TODO: Well, then also limit the target angular velocity just like target angle!
}

void ControlModule::addRobotCommand(std::optional<::rtt::world::view::RobotView> robot, rtt::RobotCommand command) noexcept {
    // If we are not left, commands should be rotated (because we play as right)
    if (!GameSettings::isLeft()) {
        rotateRobotCommand(command);
    }

    if (robot) limitRobotCommand(command, *robot);

    // if we are in simulation; adjust w() to be angular velocity)
    if (GameSettings::getRobotHubMode() == net::RobotHubMode::SIMULATOR && !command.useAngularVelocity) {
        simulator_angular_control(robot, command);
    }

    // Only add commands with a robotID that is not in the vector yet
    if (command.id >= 0 && command.id < 16) {
        robotCommands.emplace_back(command);
    }
}

void ControlModule::simulator_angular_control(const std::optional<::rtt::world::view::RobotView>& robot, rtt::RobotCommand& robot_command) {
    double ang_velocity_out = 0.0;  // in case there is no robot visible, we just adjust the command to not have any angular velocity
    if (robot) {
        Angle current_angle = robot->get()->getAngle();
        if (!GameSettings::isLeft()) {
            current_angle += M_PI;
        }
        Angle target_angle(robot_command.targetAngle);
        // get relevant PID controller
        if (simulatorAnglePIDmap.contains(robot->get()->getId())) {
            ang_velocity_out = simulatorAnglePIDmap.at(robot->get()->getId()).getOutput(target_angle, current_angle);
        } else {
            // initialize PID controller for robot
            // below tuning only works ish for erforce, is completely useless in grsim
            double P = 4.0;
            double I = 0.0;
            double D = 0.01;
            double max_ang_vel = 5.0;  // rad/s
            double dt = 1. / double(Constants::STP_TICK_RATE());

            AnglePID pid(P, I, D, max_ang_vel, dt);
            ang_velocity_out = pid.getOutput(target_angle, current_angle);
            simulatorAnglePIDmap.insert({robot->get()->getId(), pid});
        }
    }
    robot_command.useAngularVelocity = true;
    ang_velocity_out = std::clamp(ang_velocity_out, -8.0 * M_PI, 8.0 * M_PI);
    robot_command.targetAngularVelocity = static_cast<float>(ang_velocity_out);
}

void ControlModule::sendAllCommands() {
    // Remove duplicate commands
    std::sort(robotCommands.begin(), robotCommands.end(), [](const rtt::RobotCommand& a, const rtt::RobotCommand& b) { return a.id < b.id; });
    auto last = std::unique(robotCommands.begin(), robotCommands.end(), [](const rtt::RobotCommand& a, const rtt::RobotCommand& b) { return a.id == b.id; });
    robotCommands.erase(last, robotCommands.end());

    // When vector has all commands, send in one go
    io::io.publishAllRobotCommands(robotCommands);
    robotCommands.clear();
}
}  // namespace rtt::ai::control