#pragma once
#include <proto/RobotCommands.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotCommandsBluePublisher : private utils::Publisher<proto::RobotCommands> {
   public:
    RobotCommandsBluePublisher();

    void publish(const proto::RobotCommands& commands);
};

class RobotCommandsBlueSubscriber : private utils::Subscriber<proto::RobotCommands> {
   public:
    RobotCommandsBlueSubscriber(const std::function<void(const proto::RobotCommands&)>& callback);
};

class RobotCommandsYellowPublisher : private utils::Publisher<proto::RobotCommands> {
   public:
    RobotCommandsYellowPublisher();

    void publish(const proto::RobotCommands& commands);
};

class RobotCommandsYellowSubscriber : private utils::Subscriber<proto::RobotCommands> {
   public:
    RobotCommandsYellowSubscriber(const std::function<void(const proto::RobotCommands&)>& callback);
};

}  // namespace rtt::net