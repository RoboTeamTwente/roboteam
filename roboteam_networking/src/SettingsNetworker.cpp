#include <SettingsNetworker.hpp>

namespace rtt::net {

    SettingsPublisher::SettingsPublisher()
        : utils::Publisher(utils::ChannelType::SETTINGS_CHANNEL) {
    }

    std::size_t SettingsPublisher::publish(const proto::GameSettings& settings) {
        return this->send(settings.SerializeAsString());
    }

    SettingsSubscriber::SettingsSubscriber(const std::function<void(const proto::GameSettings&)>& callback)
        : utils::Subscriber(utils::ChannelType::SETTINGS_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
        if (callback == nullptr) {
            throw utils::InvalidCallbackException("Callback was nullptr");
        }
    }

    void SettingsSubscriber::onPublishedMessage(const std::string& message) {
        proto::GameSettings settings;
        settings.ParseFromString(message);
        this->callback(settings);
    }

}  // namespace rtt::net