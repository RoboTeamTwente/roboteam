#pragma once

#include <libusb-1.0/libusb.h>

#include <RobotCommand.h> // REM robot command

#include <thread>
#include <exception>
#include <string>

namespace rtt::robothub::basestation {

typedef struct BasestationMessage {
    uint8_t* payload = nullptr;
    int payload_size = 0;
} BasestationMessage;

class Basestation {
public:
    explicit Basestation(libusb_device* device);
    ~Basestation();

    bool sendMessageToBasestation(const BasestationMessage& message) const;
    BasestationMessage readIncomingMessage() const;

    bool operator == (libusb_device* device) const;

    static bool isDeviceABasestation(libusb_device* device);
    
private:
    libusb_device* device;
    uint8_t address;
    libusb_device_handle* device_handle;
};

class FailedToOpenDeviceException : public std::exception {
public:
    FailedToOpenDeviceException(const std::string& message);
    const char* what() const noexcept;
private:
    const std::string message;
};

} // namespace rtt::robothub::basestation