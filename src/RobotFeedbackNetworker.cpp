#include <RobotFeedbackNetworker.hpp>

namespace rtt::net {

RobotFeedbackPublisher::RobotFeedbackPublisher() : utils::Publisher(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL) {}

bool RobotFeedbackPublisher::publish(const proto::RobotData& feedback) {
    return this->send(feedback.SerializeAsString());
}

RobotFeedbackSubscriber::RobotFeedbackSubscriber(const std::function<void(const proto::RobotData&)>& callback)
    : utils::Subscriber(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); })
    , callback(callback)
{
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void RobotFeedbackSubscriber::onPublishedMessage(const std::string& message) {
    proto::RobotData feedback;
    feedback.ParseFromString(message);
    this->callback(feedback);
}

}  // namespace rtt::net