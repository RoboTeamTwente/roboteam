#pragma once
#include <proto/RobotCommands.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotCommandsPublisher : private utils::Publisher<proto::RobotCommands> {
   public:
    RobotCommandsPublisher();

    void publish(const proto::RobotCommands& commands);
};

class RobotCommandsSubscriber : private utils::Subscriber<proto::RobotCommands> {
   public:
    RobotCommandsSubscriber(const std::function<void(const proto::RobotCommands&)>& callback);
};

}  // namespace rtt::net