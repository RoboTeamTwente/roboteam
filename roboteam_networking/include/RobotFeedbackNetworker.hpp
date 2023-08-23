#pragma once

#include <proto/RobotFeedback.pb.h>

#include <functional>
#include <roboteam_utils/RobotFeedback.hpp>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

proto::RobotsFeedback feedbackToProto(const rtt::RobotsFeedback& robotsFeedback);
rtt::RobotsFeedback protoFeedbackToRobotsFeedback(const proto::RobotsFeedback& protoFeedbacks);

class RobotFeedbackPublisher : private utils::Publisher {
   public:
    RobotFeedbackPublisher();

    // Publishes the given robot feedback. Returns bytes sent, 0 on failure
    std::size_t publish(const rtt::RobotsFeedback& feedback);
};

class RobotFeedbackSubscriber : private utils::Subscriber {
   public:
    RobotFeedbackSubscriber(const std::function<void(const rtt::RobotsFeedback&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const rtt::RobotsFeedback&)> callback;
};

}  // namespace rtt::net