#pragma once
#include <proto/RobotData.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotFeedbackPublisher : private utils::Publisher<proto::RobotData> {
   public:
    RobotFeedbackPublisher();

    bool publish(const proto::RobotData& feedback);
};

class RobotFeedbackSubscriber : private utils::Subscriber<proto::RobotData> {
   public:
    RobotFeedbackSubscriber(const std::function<void(const proto::RobotData&)>& callback);
};

}  // namespace rtt::net