#include <RobotFeedbackNetworker.hpp>

namespace rtt::net {

RobotFeedbackPublisher::RobotFeedbackPublisher() : utils::Publisher<proto::RobotFeedback>(utils::ChannelType::ROBOT_FEEDBACK_PRIMARY_CHANNEL) {}

void RobotFeedbackPublisher::publish(const proto::RobotFeedback& feedback) { this->send(feedback); }

RobotFeedbackSubscriber::RobotFeedbackSubscriber(const std::function<void(const proto::RobotFeedback&)>& callback)
    : utils::Subscriber<proto::RobotFeedback>(utils::ChannelType::ROBOT_FEEDBACK_PRIMARY_CHANNEL, callback) {}

}  // namespace rtt::net