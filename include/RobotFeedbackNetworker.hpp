#pragma once
#include <proto/RobotData.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotFeedbackPublisher : private utils::Publisher {
   public:
    RobotFeedbackPublisher();

    // Publishes the given robot feedback. Returns success
    bool publish(const proto::RobotData& feedback);
};

class RobotFeedbackSubscriber : private utils::Subscriber {
   public:
    RobotFeedbackSubscriber(const std::function<void(const proto::RobotData&)>& callback);

private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const proto::RobotData&)> callback;
};

}  // namespace rtt::net