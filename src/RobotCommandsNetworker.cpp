#include <proto/RobotCommands.pb.h>

#include <RobotCommandsNetworker.hpp>

namespace rtt::net {

proto::RobotCommands robotCommandsToProto(const rtt::RobotCommands& commands) {
    proto::RobotCommands protoCommands;

    for (const auto& command : commands) {
        auto protoCommand = protoCommands.add_robot_commands();

        protoCommand->set_id(command.id);
        protoCommand->set_velocity_x((float)command.velocity.x);
        protoCommand->set_velocity_x((float)command.velocity.y);
        protoCommand->set_angle((float)command.targetAngle.getValue());
        protoCommand->set_angular_velocity((float)command.targetAngularVelocity);
        protoCommand->set_use_angular_velocity(command.useAngularVelocity);

        protoCommand->set_camera_angle_of_robot((float)command.cameraAngleOfRobot.getValue());
        protoCommand->set_camera_angle_of_robot_is_set(command.cameraAngleOfRobotIsSet);

        protoCommand->set_kick_speed((float)command.kickSpeed);
        protoCommand->set_wait_for_ball(command.waitForBall);
        protoCommand->set_chip_instead_of_kick(command.kickType == KickType::CHIP);

        protoCommand->set_dribbler_speed((float)command.dribblerSpeed);
        protoCommand->set_ignore_packet(command.ignorePacket);
    }

    return protoCommands;
}

rtt::RobotCommands protoToRobotCommands(const proto::RobotCommands& protoCommands) {
    rtt::RobotCommands robotCommands;

    for (const auto& protoCommand : protoCommands.robot_commands()) {
        rtt::RobotCommand robotCommand = {.id = protoCommand.id(),
                                          .velocity = Vector2(protoCommand.velocity_x(), protoCommand.velocity_y()),
                                          .targetAngle = protoCommand.angle(),
                                          .targetAngularVelocity = protoCommand.angular_velocity(),
                                          .useAngularVelocity = protoCommand.use_angular_velocity(),

                                          .cameraAngleOfRobot = protoCommand.camera_angle_of_robot(),
                                          .cameraAngleOfRobotIsSet = protoCommand.camera_angle_of_robot_is_set(),

                                          .kickSpeed = protoCommand.kick_speed(),
                                          .waitForBall = protoCommand.wait_for_ball(),
                                          .kickType = protoCommand.chip_instead_of_kick() ? KickType::CHIP : KickType::NORMAL,

                                          .dribblerSpeed = protoCommand.dribbler_speed(),
                                          .ignorePacket = protoCommand.ignore_packet()};
        robotCommands.push_back(robotCommand);
    }

    return robotCommands;
}

// Blue commands publisher
RobotCommandsBluePublisher::RobotCommandsBluePublisher() : utils::Publisher(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL) {}

bool RobotCommandsBluePublisher::publish(const rtt::RobotCommands& commands) {
    auto protoRobotCommands = robotCommandsToProto(commands);
    return this->send(protoRobotCommands.SerializeAsString());
}

// Blue commands subscriber
RobotCommandsBlueSubscriber::RobotCommandsBlueSubscriber(const std::function<void(const rtt::RobotCommands&)>& callback)
    : utils::Subscriber(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void RobotCommandsBlueSubscriber::onPublishedMessage(const std::string& message) {
    proto::RobotCommands protoCommands;
    protoCommands.ParseFromString(message);
    this->callback(protoToRobotCommands(protoCommands));
}

// Yellow commands publisher
RobotCommandsYellowPublisher::RobotCommandsYellowPublisher() : utils::Publisher(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL) {}

bool RobotCommandsYellowPublisher::publish(const rtt::RobotCommands& commands) {
    auto protoRobotCommands = robotCommandsToProto(commands);
    return this->send(protoRobotCommands.SerializeAsString());
}

RobotCommandsYellowSubscriber::RobotCommandsYellowSubscriber(const std::function<void(const rtt::RobotCommands&)>& callback)
    : utils::Subscriber(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void RobotCommandsYellowSubscriber::onPublishedMessage(const std::string& message) {
    proto::RobotCommands protoCommands;
    protoCommands.ParseFromString(message);
    this->callback(protoToRobotCommands(protoCommands));
}

}  // namespace rtt::net