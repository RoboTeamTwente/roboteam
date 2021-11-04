#include <simulation/ConfigurationCommand.hpp>

namespace rtt::robothub::simulation {
void ConfigurationCommand::setBallLocation(float x, float y, float z, float xVelocity, float yVelocity, float zVelocity, bool velocityInRolling, bool teleportSafely,
                                           bool byForce) {
    proto::sim::TeleportBall* tpBallCommand = this->configurationCommand.mutable_control()->mutable_teleport_ball();
    tpBallCommand->set_x(x);
    tpBallCommand->set_y(y);
    tpBallCommand->set_z(z);
    tpBallCommand->set_vx(xVelocity);
    tpBallCommand->set_vy(yVelocity);
    tpBallCommand->set_vz(zVelocity);
    tpBallCommand->set_roll(velocityInRolling);
    tpBallCommand->set_teleport_safely(teleportSafely);
    tpBallCommand->set_by_force(byForce);
}
void ConfigurationCommand::addRobotLocation(int id, bool isFromTeamYellow, float x, float y, float xVelocity, float yVelocity, float angularVelocity, float orientation,
                                            bool shouldBePresentOnField, bool byForce) {
    proto::sim::TeleportRobot* tpRobotCommand = this->configurationCommand.mutable_control()->add_teleport_robot();
    tpRobotCommand->mutable_id()->set_id(id);
    tpRobotCommand->mutable_id()->set_team(isFromTeamYellow ? proto::sim::Team::YELLOW : proto::sim::Team::BLUE);
    tpRobotCommand->set_x(x);
    tpRobotCommand->set_y(y);
    tpRobotCommand->set_v_x(xVelocity);
    tpRobotCommand->set_v_y(yVelocity);
    tpRobotCommand->set_v_angular(angularVelocity);
    tpRobotCommand->set_orientation(orientation);
    tpRobotCommand->set_present(shouldBePresentOnField);
    tpRobotCommand->set_by_force(byForce);
}
void ConfigurationCommand::setSimulationSpeed(float speed) { this->configurationCommand.mutable_control()->set_simulation_speed(speed); }
void ConfigurationCommand::addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties) {
    proto::sim::RobotSpecs* specs = this->configurationCommand.mutable_config()->add_robot_specs();

    specs->mutable_id()->set_id(id);
    specs->mutable_id()->set_team(isFromTeamYellow ? proto::sim::Team::YELLOW : proto::sim::Team::BLUE);
    specs->set_radius(robotProperties.radius);
    specs->set_height(robotProperties.height);
    specs->set_mass(robotProperties.mass);
    specs->set_max_linear_kick_speed(robotProperties.maxKickSpeed);
    specs->set_max_chip_kick_speed(robotProperties.maxChipSpeed);
    specs->set_center_to_dribbler(robotProperties.centerToDribblerDistance);
    // Set limits
    specs->mutable_limits()->set_acc_speedup_absolute_max(robotProperties.maxAcceleration);
    specs->mutable_limits()->set_acc_speedup_angular_max(robotProperties.maxAngularAcceleration);
    specs->mutable_limits()->set_acc_brake_absolute_max(robotProperties.maxDeceleration);
    specs->mutable_limits()->set_acc_brake_angular_max(robotProperties.maxAngularDeceleration);
    specs->mutable_limits()->set_vel_absolute_max(robotProperties.maxVelocity);
    specs->mutable_limits()->set_vel_angular_max(robotProperties.maxAngularVelocity);
    // Set wheel angles
    specs->mutable_wheel_angles()->set_front_right(robotProperties.frontRightWheelAngle);
    specs->mutable_wheel_angles()->set_back_right(robotProperties.backRightWheelAngle);
    specs->mutable_wheel_angles()->set_back_left(robotProperties.backLeftWheelAngle);
    specs->mutable_wheel_angles()->set_front_left(robotProperties.frontLeftWheelAngle);
}
void ConfigurationCommand::setVisionPort(int port) { this->configurationCommand.mutable_config()->set_vision_port(port); }
proto::sim::SimulatorCommand& ConfigurationCommand::getPacket() { return this->configurationCommand; }
}  // namespace rtt::robothub::simulation