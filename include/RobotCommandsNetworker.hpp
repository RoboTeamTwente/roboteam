#pragma once

#include <functional>
#include <roboteam_utils/RobotCommands.hpp>  // The general data format of robot commands in rtt_utils
#include <string>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class RobotCommandsBluePublisher : private utils::Publisher {
   public:
    RobotCommandsBluePublisher();

    // Publishes the given robot commands on the blue channel. Returns bytes sent, -1 on failure
    std::size_t publish(const rtt::RobotCommands& commands);
};

class RobotCommandsBlueSubscriber : private utils::Subscriber {
   public:
    RobotCommandsBlueSubscriber(const std::function<void(const rtt::RobotCommands&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const rtt::RobotCommands&)> callback;
};

class RobotCommandsYellowPublisher : private utils::Publisher {
   public:
    RobotCommandsYellowPublisher();

    // Publishes the given robot commands on the yellow channel. Returns bytes sent, -1 on failure
    std::size_t publish(const rtt::RobotCommands& commands);
};

class RobotCommandsYellowSubscriber : private utils::Subscriber {
   public:
    RobotCommandsYellowSubscriber(const std::function<void(const rtt::RobotCommands&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const rtt::RobotCommands&)> callback;
};

}  // namespace rtt::net