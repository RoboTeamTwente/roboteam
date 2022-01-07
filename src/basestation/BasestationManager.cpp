#include <basestation/BasestationManager.hpp>

#include <BasestationGetStatistics.h> // REM command

#include <constants.h>
#include <iostream>

namespace rtt::robothub::basestation {

BasestationManager::BasestationManager() {
    int error;
    error = libusb_init(&this->usb_context);
    if (error) {
        throw FailedToInitializeLibUsb("Failed to initialize libusb"); // TODO: Handle error code
    }

    this->basestationCollection = std::make_unique<BasestationCollection>();

    this->shouldListenForBasestationPlugs = true;
    this->basestationPlugsListener = std::thread(&BasestationManager::listenForBasestationPlugs, this);
}

BasestationManager::~BasestationManager() {
    // Stop threads
    this->shouldListenForBasestationPlugs = false;
    if (this->basestationPlugsListener.joinable()) {
        this->basestationPlugsListener.join();
    }

    // In destructor of basestations, the device is closed. This needs to be done
    // before libusb_exit() is called, so delete all basestation objects.
    this->basestationCollection = nullptr; // TODO: Make nice

    libusb_exit(this->usb_context);
}

bool BasestationManager::sendRobotCommand(const RobotCommand& command, utils::TeamColor color) const {
    RobotCommand copy = command; // TODO: Make encodeRobotCommand use const so copy is unecessary
    
    BasestationMessage message;
    RobotCommandPayload payload;
    encodeRobotCommand(&payload, &copy);

    message.payload = payload.payload;
    message.payload_size = PACKET_SIZE_ROBOT_COMMAND;

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

bool BasestationManager::sendRobotBuzzerCommand(const RobotBuzzer& command, utils::TeamColor color) const {
    RobotBuzzer copy = command;

    BasestationMessage message;
    RobotBuzzerPayload payload;
    encodeRobotBuzzer(&payload, &copy);

    message.payload = payload.payload;
    message.payload_size = PACKET_SIZE_ROBOT_BUZZER;

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

bool BasestationManager::sendBasestationStatisticsRequest(utils::TeamColor color) const {
    BasestationGetStatistics command;

    BasestationMessage message;
    BasestationGetStatisticsPayload payload;
    encodeBasestationGetStatistics(&payload, &command);

    message.payload = payload.payload;
    message.payload_size = PACKET_SIZE_BASESTATION_GET_STATISTICS;

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

void BasestationManager::setFeedbackCallback(const std::function<void(const RobotFeedback &)>& callback) {
    this->feedbackCallbackFunction = callback;
}

void BasestationManager::listenForBasestationPlugs() {
    while (this->shouldListenForBasestationPlugs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Get a list of devices
        libusb_device** device_list;
        ssize_t device_count = libusb_get_device_list(this->usb_context, &device_list);

        std::vector<libusb_device*> basestationDevices = filterBasestationDevices(device_list, device_count);
        
        this->basestationCollection->updateBasestationList(basestationDevices);
        
        // Free the list of devices
        libusb_free_device_list(device_list, true);
    }
}

std::vector<libusb_device*> BasestationManager::filterBasestationDevices(libusb_device** devices, int device_count) {
    std::vector<libusb_device*> basestations;
    for (int i = 0; i < device_count; ++i) {
        libusb_device* device = devices[i];

        if (Basestation::isDeviceABasestation(device)) {
            basestations.push_back(device);
        }
    }
    return basestations;
}

void BasestationManager::callFeedbackCallback(const RobotFeedback &feedback) const {
    if (this->feedbackCallbackFunction != nullptr) this->feedbackCallbackFunction(feedback);
}

FailedToInitializeLibUsb::FailedToInitializeLibUsb(const std::string message) : message(message) {}
const char* FailedToInitializeLibUsb::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation
