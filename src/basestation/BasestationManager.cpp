#include <BasestationGetStatistics.h>  // REM command

#include <basestation/BasestationManager.hpp>
#include <cstring>

namespace rtt::robothub::basestation {

BasestationManager::BasestationManager() {
    int error;
    error = libusb_init(&this->usbContext);
    if (error) {
        throw FailedToInitializeLibUsb("Failed to initialize libusb");
    }

    this->basestationCollection = std::make_unique<BasestationCollection>();
    this->basestationCollection->setIncomingMessageCallback([&](const BasestationMessage& message, utils::TeamColor color) { this->handleIncomingMessage(message, color); });

    this->shouldListenForBasestationPlugs = true;
    this->basestationPlugsListener = std::thread(&BasestationManager::listenForBasestationPlugs, this);
}

BasestationManager::~BasestationManager() {
    // Stop threads
    this->shouldListenForBasestationPlugs = false;
    if (this->basestationPlugsListener.joinable()) {
        this->basestationPlugsListener.join();
    }

    // In destructor of basestation objects, the usb device is closed. This needs to be done
    // before libusb_exit() is called, so delete all basestation objects now
    this->basestationCollection = nullptr;

    libusb_exit(this->usbContext);
}

bool BasestationManager::sendRobotCommand(const RobotCommand& command, utils::TeamColor color) const {
    RobotCommand copy = command;  // TODO: Make REM encodeRobotCommand use const so copy is unecessary

    RobotCommandPayload payload;
    encodeRobotCommand(&payload, &copy);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_ROBOT_COMMAND;
    std::memcpy(&message.payloadBuffer, payload.payload, message.payloadSize);

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

bool BasestationManager::sendRobotBuzzerCommand(const RobotBuzzer& command, utils::TeamColor color) const {
    RobotBuzzer copy = command;

    RobotBuzzerPayload payload;
    encodeRobotBuzzer(&payload, &copy);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_ROBOT_BUZZER;
    std::memcpy(message.payloadBuffer, payload.payload, message.payloadSize);

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

bool BasestationManager::sendBasestationStatisticsRequest(utils::TeamColor color) const {
    BasestationGetStatistics command;

    BasestationGetStatisticsPayload payload;
    encodeBasestationGetStatistics(&payload, &command);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_BASESTATION_GET_STATISTICS;
    std::memcpy(message.payloadBuffer, &payload.payload, message.payloadSize);

    return this->basestationCollection->sendMessageToBasestation(message, color);
}

void BasestationManager::setFeedbackCallback(const std::function<void(const RobotFeedback&, utils::TeamColor)>& callback) { this->feedbackCallbackFunction = callback; }

void BasestationManager::listenForBasestationPlugs() {
    while (this->shouldListenForBasestationPlugs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Get a list of devices
        libusb_device** device_list;
        ssize_t device_count = libusb_get_device_list(this->usbContext, &device_list);

        std::vector<libusb_device*> basestationDevices = filterBasestationDevices(device_list, device_count);

        this->basestationCollection->updateBasestationList(basestationDevices);

        // Free the list of devices
        libusb_free_device_list(device_list, true);
    }
}

void BasestationManager::printStatus() const { this->basestationCollection->printCollection(); }

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

void BasestationManager::handleIncomingMessage(const BasestationMessage& message, utils::TeamColor color) const {
    switch (message.payloadBuffer[0]) {
        case PACKET_TYPE_ROBOT_FEEDBACK: {
            RobotFeedbackPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, message.payloadSize);

            RobotFeedback feedback;
            decodeRobotFeedback(&feedback, &payload);

            this->callFeedbackCallback(feedback, color);
            break;
        }  // TODO: Other packets can be handled as well, like basestation statistics
    }
}

void BasestationManager::callFeedbackCallback(const RobotFeedback& feedback, utils::TeamColor color) const {
    if (this->feedbackCallbackFunction != nullptr) this->feedbackCallbackFunction(feedback, color);
}

FailedToInitializeLibUsb::FailedToInitializeLibUsb(const std::string message) : message(message) {}
const char* FailedToInitializeLibUsb::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation
