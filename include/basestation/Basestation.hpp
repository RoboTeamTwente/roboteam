#pragma once

#include <libusb-1.0/libusb.h>

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace rtt::robothub::basestation {

constexpr int BASESTATION_MESSAGE_BUFFER_SIZE = 4096;
typedef struct BasestationMessage {
    int payloadSize = 0;  // Size of the message inside the big set-size payloadBuffer
    uint8_t payloadBuffer[BASESTATION_MESSAGE_BUFFER_SIZE];
} BasestationMessage;

typedef struct BasestationIdentifier {
    uint8_t usbAddress;
    uint8_t serialIdentifier;

    bool operator==(const BasestationIdentifier& other) const;
    bool operator<(const BasestationIdentifier& other) const;
    const std::string toString() const;
} BasestationIdentifier;

class Basestation {
   public:
    explicit Basestation(libusb_device* const device);
    ~Basestation();

    // Compares the underlying device of this basestation with the given device
    bool operator==(libusb_device* const device) const;
    bool operator==(const BasestationIdentifier& otherBasestationID) const;
    bool operator==(const std::shared_ptr<Basestation>& otherBasestation) const;

    // Sends message to basestation. Returns bytes sent, -1 for error
    int sendMessageToBasestation(BasestationMessage& message) const;
    void setIncomingMessageCallback(const std::function<void(const BasestationMessage&, const BasestationIdentifier&)>& callback);

    const BasestationIdentifier& getIdentifier() const;

    static bool isDeviceABasestation(libusb_device* const device);

   private:
    libusb_device* const device;             // Corresponds to the basestation itself
    libusb_device_handle* deviceHandle;      // Handle on which IO can be performed
    const BasestationIdentifier identifier;  // An identifier object that uniquely represents this basestation

    bool shouldListenForIncomingMessages;
    std::thread incomingMessageListenerThread;
    void listenForIncomingMessages();
    std::function<void(const BasestationMessage&, const BasestationIdentifier&)> incomingMessageCallback;

    // Reads a message directly from the basestation and stores it in the given message. Returns success
    bool readBasestationMessage(BasestationMessage& message) const;
    // Writes the given message directly to the basestation. Returns bytes sent
    int writeBasestationMessage(BasestationMessage& message) const;

    static const BasestationIdentifier getIdentifierOfDevice(libusb_device* const device);
};

class FailedToOpenDeviceException : public std::exception {
   public:
    FailedToOpenDeviceException(const std::string& message);
    const char* what() const noexcept;

   private:
    const std::string message;
};

}  // namespace rtt::robothub::basestation