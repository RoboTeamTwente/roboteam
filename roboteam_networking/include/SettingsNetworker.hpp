#pragma once
#include <proto/GameSettings.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include "proto/GameSettings.pb.h"

namespace rtt::net {

class SettingsPublisher : private utils::Publisher {
   public:
    SettingsPublisher();

    // Publishes the given settings. Returns bytes sent, 0 on failure
    std::size_t publish(const proto::GameSettings& settings);
};

class SettingsSubscriber : private utils::Subscriber {
   public:
    SettingsSubscriber(const std::function<void(const proto::GameSettings&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const proto::GameSettings&)> callback;
};

}  // namespace rtt::net