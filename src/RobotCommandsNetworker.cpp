#include <RobotCommandsNetworker.hpp>

namespace rtt::net {

RobotCommandsPublisher::RobotCommandsPublisher():
    utils::Publisher<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_PRIMARY_CHANNEL) {}

void RobotCommandsPublisher::publish(const proto::RobotCommands& commands) {
    this->send(commands);
}

RobotCommandsSubscriber::RobotCommandsSubscriber(const std::function<void(const proto::RobotCommands&)>& callback) :
    utils::Subscriber<proto::RobotCommands>(utils::ChannelType::ROBOT_COMMANDS_PRIMARY_CHANNEL, callback) {}

} // namespace rtt::net