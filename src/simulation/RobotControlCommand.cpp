#include <simulation/RobotControlCommand.hpp>

namespace rtt::robothub::simulation {
void RobotControlCommand::addRobotControlWithWheelSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float frontRightWheelVelocity,
                                                         float backRightWheelVelocity, float backLeftWheelVelocity, float frontLeftWheelVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveWheelVelocity* velocity = command->mutable_move_command()->mutable_wheel_velocity();
    velocity->set_front_right(frontRightWheelVelocity);
    velocity->set_back_right(backRightWheelVelocity);
    velocity->set_back_left(backLeftWheelVelocity);
    velocity->set_front_left(frontLeftWheelVelocity);
}
void RobotControlCommand::addRobotControlWithLocalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float forwardVelocity, float leftVelocity,
                                                         float angularVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveLocalVelocity* velocity = command->mutable_move_command()->mutable_local_velocity();
    velocity->set_forward(forwardVelocity);
    velocity->set_left(leftVelocity);
    velocity->set_angular(angularVelocity);
}
void RobotControlCommand::addRobotControlWithGlobalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float xVelocity, float yVelocity,
                                                          float angularVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveGlobalVelocity* velocity = command->mutable_move_command()->mutable_global_velocity();
    velocity->set_x(xVelocity);
    velocity->set_y(yVelocity);
    velocity->set_angular(angularVelocity);
}
proto::sim::RobotControl& RobotControlCommand::getPacket() { return this->controlCommand; }
}  // namespace rtt::robothub::simulation