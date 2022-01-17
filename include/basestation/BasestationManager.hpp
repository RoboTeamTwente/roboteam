#pragma once

#include <RobotBuzzer.h>    // REM command
#include <RobotCommand.h>   // REM command
#include <RobotFeedback.h>  // REM command
#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <basestation/BasestationCollection.hpp>
#include <functional>
#include <memory>
#include <thread>

namespace rtt::robothub::basestation {

class BasestationManager {
   public:
    BasestationManager();
    ~BasestationManager();

    bool sendRobotCommand(const RobotCommand &command, utils::TeamColor color) const;
    bool sendRobotBuzzerCommand(const RobotBuzzer &command, utils::TeamColor color) const;
    bool sendBasestationStatisticsRequest(utils::TeamColor color) const;

    void setFeedbackCallback(const std::function<void(const RobotFeedback &, utils::TeamColor color)> &callback);

    void printStatus() const;

   private:
    libusb_context *usbContext;

    bool shouldListenForBasestationPlugs;
    std::thread basestationPlugsListener;
    void listenForBasestationPlugs();

    std::unique_ptr<BasestationCollection> basestationCollection;

    void handleIncomingMessage(const BasestationMessage &message, utils::TeamColor basestationColor) const;

    std::function<void(const RobotFeedback &, utils::TeamColor)> feedbackCallbackFunction;
    void callFeedbackCallback(const RobotFeedback &feedback, utils::TeamColor color) const;

    static std::vector<libusb_device *> filterBasestationDevices(libusb_device **devices, int device_count);
};

class FailedToInitializeLibUsb : public std::exception {
   public:
    FailedToInitializeLibUsb(const std::string message);
    const char *what() const noexcept override;

   private:
    const std::string message;
};
}  // namespace rtt::robothub::basestation