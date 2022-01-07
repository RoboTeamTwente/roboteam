#pragma once

#include <libusb-1.0/libusb.h>

#include <exception>
#include <string>
#include <thread>
#include <functional>

namespace rtt::robothub::basestation {

typedef struct BasestationMessage {
    int payload_size = 0;
    uint8_t* payload = nullptr;
} BasestationMessage;

enum class WirelessChannel {
    BLUE_CHANNEL, YELLOW_CHANNEL, UNKNOWN
};

class Basestation {
public:
    explicit Basestation(libusb_device* device);
    ~Basestation();

    bool operator == (libusb_device* device) const;

    bool sendMessageToBasestation(const BasestationMessage& message) const;
    void setIncomingMessageCallback(std::function<void(const BasestationMessage&)> callback);

    WirelessChannel getCurrentUsedChannel() const;
    bool requestChannelChange(WirelessChannel newChannel);
    bool isWaitingForChannelChange() const;

    static bool isDeviceABasestation(libusb_device* device);
    static bool wirelessChannelToREMChannel(WirelessChannel channel);
    static WirelessChannel remChannelToWirelessChannel(bool remChannel);
private:
    libusb_device* device;
    uint8_t address;
    libusb_device_handle* device_handle;

    WirelessChannel wantedChannel; // Settable by everyone else
    WirelessChannel currentlyUsedChannel; // Only settable by basestation

    bool shouldListenForIncomingMessages;
    std::thread incomingMessageListenerThread;
    void listenForIncomingMessages();
    void handleIncomingMessage(const BasestationMessage& message);
    std::function<void(const BasestationMessage&)> incomingMessageCallback;

    void updateCurrentlyUsedChannel(WirelessChannel newChannel);
    bool sendConfigurationCommand(WirelessChannel channel);
};

class FailedToOpenDeviceException : public std::exception {
public:
    FailedToOpenDeviceException(const std::string& message);
    const char* what() const noexcept;
private:
    const std::string message;
};

} // namespace rtt::robothub::basestation