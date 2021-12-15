#include <RobotCommandsNetworker.hpp>

namespace rtt::net {

RobotCommandsBluePublisher::RobotCommandsBluePublisher() : utils::Publisher<proto::AICommand>(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL) {}

bool RobotCommandsBluePublisher::publish(const proto::AICommand& commands) { return this->send(commands); }

RobotCommandsBlueSubscriber::RobotCommandsBlueSubscriber(const std::function<void(const proto::AICommand&)>& callback)
    : utils::Subscriber<proto::AICommand>(utils::ChannelType::ROBOT_COMMANDS_BLUE_CHANNEL, callback) {}

RobotCommandsYellowPublisher::RobotCommandsYellowPublisher() : utils::Publisher<proto::AICommand>(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL) {}

bool RobotCommandsYellowPublisher::publish(const proto::AICommand& commands) { return this->send(commands); }

RobotCommandsYellowSubscriber::RobotCommandsYellowSubscriber(const std::function<void(const proto::AICommand&)>& callback)
    : utils::Subscriber<proto::AICommand>(utils::ChannelType::ROBOT_COMMANDS_YELLOW_CHANNEL, callback) {}

}  // namespace rtt::net