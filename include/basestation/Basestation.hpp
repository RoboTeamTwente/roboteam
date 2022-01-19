#pragma once

#include <libusb-1.0/libusb.h>

#include <exception>
#include <functional>
#include <string>
#include <thread>

namespace rtt::robothub::basestation {

constexpr int BASESTATION_MESSAGE_BUFFER_SIZE = 4096;
typedef struct BasestationMessage {
    int payloadSize = 0;  // Size of the message inside the big set-size payloadBuffer
    uint8_t payloadBuffer[BASESTATION_MESSAGE_BUFFER_SIZE];
} BasestationMessage;

enum class WirelessChannel : unsigned int { YELLOW_CHANNEL = 0, BLUE_CHANNEL = 1, UNKNOWN = 2 };

class Basestation {
   public:
    explicit Basestation(libusb_device* device);
    ~Basestation();

    // Compares the underlying device of this basestation with the given device
    bool operator==(libusb_device* device) const;

    bool sendMessageToBasestation(BasestationMessage& message) const;
    void setIncomingMessageCallback(std::function<void(const BasestationMessage&)> callback);

    WirelessChannel getChannel() const;

    static bool isDeviceABasestation(libusb_device* device);
    // Converts a WirelessChannel to a value that can be used in REM
    static bool wirelessChannelToREMChannel(WirelessChannel channel);
    // Converts a wireless channel value from REM to a WirelessChannel
    static WirelessChannel remChannelToWirelessChannel(bool remChannel);
    static std::string wirelessChannelToString(WirelessChannel channel);

   private:
    libusb_device* device;               // Corresponds to the basestation itself
    uint8_t address;                     // USB address the basestation is connected to
    libusb_device_handle* deviceHandle;  // Handle on which IO can be performed
    WirelessChannel channel;             // Channel at which the basestation sends messages to robots

    bool shouldListenForIncomingMessages;
    std::thread incomingMessageListenerThread;
    void listenForIncomingMessages();
    void handleIncomingMessage(const BasestationMessage& message);
    std::function<void(const BasestationMessage&)> incomingMessageCallback;

    // Sends a message to the basestation that asks what its channel is
    bool requestChannelOfBasestation();

    // Reads a message directly from the basestation and stores it in the given message
    bool readBasestationMessage(BasestationMessage& message) const;
    // Writes the given message directly to the basestation
    bool writeBasestationMessage(BasestationMessage& message) const;
};

class FailedToOpenDeviceException : public std::exception {
   public:
    FailedToOpenDeviceException(const std::string& message);
    const char* what() const noexcept;

   private:
    const std::string message;
};

}  // namespace rtt::robothub::basestation