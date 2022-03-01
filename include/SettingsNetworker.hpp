#pragma once
#include <proto/Setting.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class SettingsPublisher : private utils::Publisher {
   public:
    SettingsPublisher();

    // Publishes the given settings. Returns success
    bool publish(const proto::Setting& settings);
};

class SettingsSubscriber : private utils::Subscriber {
   public:
    SettingsSubscriber(const std::function<void(const proto::Setting&)>& callback);

private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const proto::Setting&)> callback;
};

}  // namespace rtt::net