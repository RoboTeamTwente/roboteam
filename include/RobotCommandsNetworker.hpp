#pragma once
#include <proto/AICommand.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotCommandsBluePublisher : private utils::Publisher<proto::AICommand> {
   public:
    RobotCommandsBluePublisher();

    bool publish(const proto::AICommand& commands);
};

class RobotCommandsBlueSubscriber : private utils::Subscriber<proto::AICommand> {
   public:
    RobotCommandsBlueSubscriber(const std::function<void(const proto::AICommand&)>& callback);
};

class RobotCommandsYellowPublisher : private utils::Publisher<proto::AICommand> {
   public:
    RobotCommandsYellowPublisher();

    bool publish(const proto::AICommand& commands);
};

class RobotCommandsYellowSubscriber : private utils::Subscriber<proto::AICommand> {
   public:
    RobotCommandsYellowSubscriber(const std::function<void(const proto::AICommand&)>& callback);
};

}  // namespace rtt::net