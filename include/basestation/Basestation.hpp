#pragma once

#include <libusb-1.0/libusb.h>

#include <exception>
#include <functional>
#include <string>
#include <thread>
#include <memory>

namespace rtt::robothub::basestation {

constexpr int BASESTATION_MESSAGE_BUFFER_SIZE = 4096;
typedef struct BasestationMessage {
    int payloadSize = 0;  // Size of the message inside the big set-size payloadBuffer
    uint8_t payloadBuffer[BASESTATION_MESSAGE_BUFFER_SIZE];
} BasestationMessage;

class Basestation {
   public:
    explicit Basestation(libusb_device* device);
    ~Basestation();

    // Compares the underlying device of this basestation with the given device
    bool operator==(libusb_device* device) const;
    bool operator==(uint8_t otherBasestationSerialID) const; // Assumes serial id is that of a basestation device
    bool operator==(std::shared_ptr<Basestation> otherBasestation) const;

    bool sendMessageToBasestation(BasestationMessage& message) const;
    void setIncomingMessageCallback(std::function<void(const BasestationMessage&, uint8_t basestationID)> callback);

    uint8_t getSerialID() const;

    static bool isDeviceABasestation(libusb_device* device);

   private:
    libusb_device* device;              // Corresponds to the basestation itself
    libusb_device_handle* deviceHandle; // Handle on which IO can be performed
    uint8_t serialID;                   // Unique identifier of basestations, used for comparing

    bool shouldListenForIncomingMessages;
    std::thread incomingMessageListenerThread;
    void listenForIncomingMessages();
    std::function<void(const BasestationMessage&, uint8_t basestationIdentifier)> incomingMessageCallback;

    // Reads a message directly from the basestation and stores it in the given message
    bool readBasestationMessage(BasestationMessage& message) const;
    // Writes the given message directly to the basestation
    bool writeBasestationMessage(BasestationMessage& message) const;

    static uint8_t getSerialIdentifierOfDevice(libusb_device* device);
};

class FailedToOpenDeviceException : public std::exception {
   public:
    FailedToOpenDeviceException(const std::string& message);
    const char* what() const noexcept;

   private:
    const std::string message;
};

}  // namespace rtt::robothub::basestation