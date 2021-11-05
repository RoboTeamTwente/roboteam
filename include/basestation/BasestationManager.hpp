#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>
#include <thread>
#include "RobotFeedback.h"
#include "RobotCommand.h"

namespace rtt::robothub::basestation {

int hotplug_callback_attach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data);
int hotplug_callback_detach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data);

class BasestationManager {
public:
    BasestationManager();
    ~BasestationManager();

    void sendSerialCommand(RobotCommandPayload payload);

    void setFeedbackCallback(std::function<void(RobotFeedback&)> callback);

private:
    bool shouldStopRunning;
    std::thread runThread;
    bool shouldStopListening;
    std::thread listenThread;

    std::function<void(RobotFeedback&)> feedbackCallbackFunction;
    
    libusb_context *ctx;
    libusb_device *basestation_device = nullptr;
    libusb_device_handle *basestation_handle = nullptr;
    libusb_hotplug_callback_handle callback_handle_attach;
    libusb_hotplug_callback_handle callback_handle_detach;
public:
    void handleBasestationAttach(libusb_device *device);
    void handleBasestationDetach(libusb_device *device);
private:
    bool setupUsbEventListeners();

    void runManager();
    void listenToBasestation();

    // unused but kept for future reference
    bool openBasestation(libusb_context *ctx, libusb_device_handle **basestation_handle);
};

class FailedToSetupUsbEventListenerException : public std::exception {
public:
    FailedToSetupUsbEventListenerException(const std::string message);
    const char* what() const noexcept override;
private:
    std::string message;
};
}