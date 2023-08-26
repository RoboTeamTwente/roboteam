#pragma once

#include <REM_RobotBuzzer.h>
#include <REM_RobotCommand.h>
#include <REM_RobotFeedback.h>
#include <REM_RobotStateInfo.h>
#include <libusb-1.0/libusb.h>

#include <basestation/BasestationCollection.hpp>
#include <functional>
#include <memory>
#include <roboteam_utils/Teams.hpp>
#include <thread>

namespace rtt::robothub::basestation {

// Contains the status of the basestation manager. Currently only has the status of its collection
typedef struct BasestationManagerStatus {
    BasestationCollectionStatus basestationCollection;
} BasestationManagerStatus;

class BasestationManager {
   public:
    BasestationManager();
    ~BasestationManager();

    int sendRobotCommand(const REM_RobotCommand &command, rtt::Team color) const;
    int sendRobotBuzzerCommand(const REM_RobotBuzzer &command, rtt::Team color) const;

    void setFeedbackCallback(const std::function<void(const REM_RobotFeedback &, rtt::Team color)> &callback);
    void setRobotStateInfoCallback(const std::function<void(const REM_RobotStateInfo &, rtt::Team color)> &callback);
    void setBasestationLogCallback(const std::function<void(const std::string &, rtt::Team color)> &callback);

    [[nodiscard]] BasestationManagerStatus getStatus() const;

   private:
    libusb_context *usbContext;

    bool shouldListenForBasestationPlugs;
    std::thread basestationPlugsListener;
    void listenForBasestationPlugs();

    std::unique_ptr<BasestationCollection> basestationCollection;

    void handleIncomingMessage(const BasestationMessage &message, rtt::Team basestationColor) const;

    std::function<void(const REM_RobotFeedback &, rtt::Team)> feedbackCallbackFunction;
    void callFeedbackCallback(const REM_RobotFeedback &feedback, rtt::Team color) const;
    std::function<void(const REM_RobotStateInfo &, rtt::Team)> robotStateInfoCallbackFunction;
    void callRobotStateInfoCallback(const REM_RobotStateInfo &stateInfo, rtt::Team color) const;
    std::function<void(const std::string &, rtt::Team)> basestationLogCallback;
    void callBasestationLogCallback(const std::string &basestationLog, rtt::Team color) const;

    static std::vector<libusb_device *> filterBasestationDevices(libusb_device *const *const devices, int device_count);
};

class FailedToInitializeLibUsb : public std::exception {
   public:
    explicit FailedToInitializeLibUsb(const std::string &message);
    [[nodiscard]] const char *what() const noexcept override;

   private:
    const std::string message;
};
}  // namespace rtt::robothub::basestation