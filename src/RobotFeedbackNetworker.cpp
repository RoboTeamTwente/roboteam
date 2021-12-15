#include <RobotFeedbackNetworker.hpp>

namespace rtt::net {

RobotFeedbackPublisher::RobotFeedbackPublisher() : utils::Publisher<proto::RobotData>(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL) {}

bool RobotFeedbackPublisher::publish(const proto::RobotData& feedback) { return this->send(feedback); }

RobotFeedbackSubscriber::RobotFeedbackSubscriber(const std::function<void(const proto::RobotData&)>& callback)
    : utils::Subscriber<proto::RobotData>(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL, callback) {}

}  // namespace rtt::net