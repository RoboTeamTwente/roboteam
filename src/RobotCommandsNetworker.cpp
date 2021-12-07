#include <RobotCommandsNetworker.hpp>

namespace rtt::net {

RobotCommandsBluePublisher::RobotCommandsBluePublisher() : utils::Publisher<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL) {}

void RobotCommandsBluePublisher::publish(const proto::RobotCommands& commands) { this->send(commands); }

RobotCommandsBlueSubscriber::RobotCommandsBlueSubscriber(const std::function<void(const proto::RobotCommands&)>& callback)
    : utils::Subscriber<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL, callback) {}

RobotCommandsYellowPublisher::RobotCommandsYellowPublisher() : utils::Publisher<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL) {}

void RobotCommandsYellowPublisher::publish(const proto::RobotCommands& commands) { this->send(commands); }

RobotCommandsYellowSubscriber::RobotCommandsYellowSubscriber(const std::function<void(const proto::RobotCommands&)>& callback)
    : utils::Subscriber<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL, callback) {}

}  // namespace rtt::net