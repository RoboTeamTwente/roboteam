#pragma once

#include <RobotCommand.h>
#include <RobotFeedback.h>
#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <thread>

namespace rtt::robothub::basestation {

int hotplug_callback_attach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data);
int hotplug_callback_detach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data);

class BasestationManager {
   public:
    BasestationManager();
    ~BasestationManager();

    bool sendSerialCommand(RobotCommandPayload &payload) const;

    void setFeedbackCallback(const std::function<void(const RobotFeedback &)> &callback);

    void handleBasestationAttach(libusb_device *device);
    void handleBasestationDetach(libusb_device *device);

   private:
    bool shouldStopRunning;
    std::thread runThread;
    bool shouldStopListening;
    std::thread listenThread;

    std::function<void(const RobotFeedback &)> feedbackCallbackFunction;

    libusb_context *ctx;
    libusb_device *basestation_device = nullptr;
    libusb_device_handle *basestation_handle = nullptr;
    libusb_hotplug_callback_handle callback_handle_attach;
    libusb_hotplug_callback_handle callback_handle_detach;

    bool setupUsbEventListeners();

    void runManager() const;
    void listenToBasestation() const;
    void callFeedbackCallback(const RobotFeedback &feedback) const;
};

class FailedToSetupUsbEventListenerException : public std::exception {
   public:
    FailedToSetupUsbEventListenerException(const std::string message);
    const char *what() const noexcept override;

   private:
    std::string message;
};
}  // namespace rtt::robothub::basestation