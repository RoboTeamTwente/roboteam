#include <proto/RobotFeedback.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotFeedbackPublisher : private utils::Publisher<proto::RobotFeedback> {
   public:
    RobotFeedbackPublisher();

    void publish(const proto::RobotFeedback& feedback);
};

class RobotFeedbackSubscriber : private utils::Subscriber<proto::RobotFeedback> {
   public:
    RobotFeedbackSubscriber(const std::function<void(const proto::RobotFeedback&)>& callback);
};

}  // namespace rtt::net