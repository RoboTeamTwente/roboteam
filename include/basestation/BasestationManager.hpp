#pragma once

#include <RobotCommand.h>
#include <RobotFeedback.h>
#include <libusb-1.0/libusb.h>
#include <basestation/Basestation.hpp>

#include <functional>
#include <memory>
#include <thread>

namespace rtt::robothub::basestation {

class BasestationManager {
   public:
    BasestationManager();
    ~BasestationManager();

    bool sendRobotCommand(const RobotCommand& command, bool toTeamYellow);

    void setFeedbackCallback(const std::function<void(const RobotFeedback &)> &callback);

   private:
    libusb_context* usb_context;

    bool shouldListenForBasestationPlugs;
    std::thread basestationPlugsListener;
    void listenForBasestationPlugs();

    std::vector<std::shared_ptr<Basestation>> basestations;
    void updateBasestationsList(const std::vector<libusb_device*>& pluggedBasestationDevices);

    std::function<void(const RobotFeedback &)> feedbackCallbackFunction;
    void callFeedbackCallback(const RobotFeedback &feedback) const;

    static bool basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices);
    static bool deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations);
    static std::vector<libusb_device*> filterBasestationDevices(libusb_device** devices, int device_count);
};

class FailedToInitializeLibUsb : public std::exception {
   public:
    FailedToInitializeLibUsb(const std::string message);
    const char *what() const noexcept override;

   private:
    const std::string message;
};
}  // namespace rtt::robothub::basestation