#include <proto/RobotCommands.pb.h>

#include <RobotCommandsNetworker.hpp>

namespace rtt::net {

proto::RobotCommand::KickType kickTypeToProto(rtt::KickType kickType) {
    switch (kickType) {
        case rtt::KickType::NO_KICK:
            return proto::RobotCommand_KickType_NO_KICK;
        case rtt::KickType::KICK:
            return proto::RobotCommand_KickType_KICK;
        case rtt::KickType::CHIP:
            return proto::RobotCommand_KickType_CHIP;
        default:
            return proto::RobotCommand_KickType_NO_KICK;
    }
}

rtt::KickType protoToKickType(proto::RobotCommand::KickType kickType) {
    switch (kickType) {
        case proto::RobotCommand_KickType_NO_KICK:
            return rtt::KickType::NO_KICK;
        case proto::RobotCommand_KickType_KICK:
            return rtt::KickType::KICK;
        case proto::RobotCommand_KickType_CHIP:
            return rtt::KickType::CHIP;
        default:
            return rtt::KickType::NO_KICK;
    }
}

proto::RobotCommands robotCommandsToProto(const rtt::RobotCommands& commands) {
    proto::RobotCommands protoCommands;

    for (const auto& command : commands) {
        auto protoCommand = protoCommands.add_robot_commands();

        protoCommand->set_id(command.id);
        protoCommand->set_velocity_x(command.velocity.x);
        protoCommand->set_velocity_y(command.velocity.y);
        protoCommand->set_yaw(command.yaw.getValue());
        protoCommand->set_angular_velocity(command.targetAngularVelocity);
        protoCommand->set_use_angular_velocity(command.useAngularVelocity);

        protoCommand->set_camera_yaw_of_robot(command.cameraYawOfRobot.getValue());
        protoCommand->set_camera_yaw_of_robot_is_set(command.cameraYawOfRobotIsSet);

        protoCommand->set_kick_speed(command.kickSpeed);
        protoCommand->set_wait_for_ball(command.waitForBall);
        protoCommand->set_kick_type(kickTypeToProto(command.kickType));
        protoCommand->set_kick_at_yaw(command.kickAtYaw);

        protoCommand->set_dribbler_on(command.dribblerOn);
        protoCommand->set_wheels_off(command.wheelsOff);
        protoCommand->set_acceleration_x(command.acceleration.x);
        protoCommand->set_acceleration_y(command.acceleration.y);
    }

    return protoCommands;
}

rtt::RobotCommands protoToRobotCommands(const proto::RobotCommands& protoCommands) {
    rtt::RobotCommands robotCommands;

    for (const auto& protoCommand : protoCommands.robot_commands()) {
        rtt::RobotCommand robotCommand = {.id = protoCommand.id(),
                                          .velocity = Vector2(protoCommand.velocity_x(), protoCommand.velocity_y()),
                                          .acceleration = Vector2(protoCommand.acceleration_x(), protoCommand.acceleration_y()),
                                          .yaw = protoCommand.yaw(),
                                          .targetAngularVelocity = protoCommand.angular_velocity(),
                                          .useAngularVelocity = protoCommand.use_angular_velocity(),

                                          .cameraYawOfRobot = protoCommand.camera_yaw_of_robot(),
                                          .cameraYawOfRobotIsSet = protoCommand.camera_yaw_of_robot_is_set(),

                                          .kickSpeed = protoCommand.kick_speed(),
                                          .waitForBall = protoCommand.wait_for_ball(),
                                          .kickType = protoToKickType(protoCommand.kick_type()),
                                          .kickAtYaw = protoCommand.kick_at_yaw(),

                                          .dribblerOn = protoCommand.dribbler_on(),
                                          .wheelsOff = protoCommand.wheels_off()};
        robotCommands.push_back(robotCommand);
    }

    return robotCommands;
}

// Blue commands publisher
RobotCommandsBluePublisher::RobotCommandsBluePublisher() : utils::Publisher(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL) {}

std::size_t RobotCommandsBluePublisher::publish(const rtt::RobotCommands& commands) {
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

std::size_t RobotCommandsYellowPublisher::publish(const rtt::RobotCommands& commands) {
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