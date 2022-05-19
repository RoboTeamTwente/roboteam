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
    this->basestationCollection->setIncomingMessageCallback([&](const BasestationMessage& message, rtt::Team color) { this->handleIncomingMessage(message, color); });

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

int BasestationManager::sendRobotCommand(const REM_RobotCommand& command, rtt::Team color) const {
    REM_RobotCommand copy = command;  // TODO: Make REM encodeRobotCommand use const so copy is unecessary

    REM_RobotCommandPayload payload;
    encodeREM_RobotCommand(&payload, &copy);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_REM_ROBOT_COMMAND;
    std::memcpy(&message.payloadBuffer, payload.payload, message.payloadSize);

    int bytesSent = this->basestationCollection->sendMessageToBasestation(message, color);
    return bytesSent;
}

int BasestationManager::sendRobotBuzzerCommand(const REM_RobotBuzzer& command, rtt::Team color) const {
    REM_RobotBuzzer copy = command;

    REM_RobotBuzzerPayload payload;
    encodeREM_RobotBuzzer(&payload, &copy);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_REM_ROBOT_BUZZER;
    std::memcpy(message.payloadBuffer, payload.payload, message.payloadSize);

    int bytesSent = this->basestationCollection->sendMessageToBasestation(message, color);
    return bytesSent;
}

void BasestationManager::setFeedbackCallback(const std::function<void(const REM_RobotFeedback&, rtt::Team)>& callback) { this->feedbackCallbackFunction = callback; }

void BasestationManager::setRobotStateInfoCallback(const std::function<void(const REM_RobotStateInfo&, rtt::Team)>& callback) { this->robotStateInfoCallbackFunction = callback; }

void BasestationManager::listenForBasestationPlugs() {
    while (this->shouldListenForBasestationPlugs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Get a list of devices
        libusb_device** device_list;
        ssize_t device_count = libusb_get_device_list(this->usbContext, &device_list);

        std::vector<libusb_device*> basestationDevices = filterBasestationDevices(device_list, device_count);

        this->basestationCollection->updateBasestationCollection(basestationDevices);

        // Free the list of devices
        libusb_free_device_list(device_list, true);
    }
}

const BasestationManagerStatus BasestationManager::getStatus() const {
    const BasestationManagerStatus status = {.basestationCollection = this->basestationCollection->getStatus()};

    return status;
}

std::vector<libusb_device*> BasestationManager::filterBasestationDevices(libusb_device* const* const devices, int device_count) {
    std::vector<libusb_device*> basestations;
    for (int i = 0; i < device_count; ++i) {
        libusb_device* const device = devices[i];

        if (Basestation::isDeviceABasestation(device)) {
            basestations.push_back(device);
        }
    }
    return basestations;
}

void BasestationManager::handleIncomingMessage(const BasestationMessage& message, rtt::Team color) const {
    switch (message.payloadBuffer[0]) {
        case PACKET_TYPE_REM_ROBOT_FEEDBACK: {
            REM_RobotFeedbackPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, message.payloadSize);

            REM_RobotFeedback feedback;
            decodeREM_RobotFeedback(&feedback, &payload);

            this->callFeedbackCallback(feedback, color);
            break;
        }
        case PACKET_TYPE_REM_ROBOT_STATE_INFO: {
            REM_RobotStateInfoPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, message.payloadSize);

            REM_RobotStateInfo stateInfo;
            decodeREM_RobotStateInfo(&stateInfo, &payload);

            this->callRobotStateInfoCallback(stateInfo, color);

            break;
        }
        // TODO: Other packets can be handled as well, like basestation statistics
    }
}

void BasestationManager::callFeedbackCallback(const REM_RobotFeedback& feedback, rtt::Team color) const {
    if (this->feedbackCallbackFunction != nullptr) this->feedbackCallbackFunction(feedback, color);
}

void BasestationManager::callRobotStateInfoCallback(const REM_RobotStateInfo& robotStateInfo, rtt::Team color) const {
    if (this->robotStateInfoCallbackFunction != nullptr) this->robotStateInfoCallbackFunction(robotStateInfo, color);
}

FailedToInitializeLibUsb::FailedToInitializeLibUsb(const std::string message) : message(message) {}
const char* FailedToInitializeLibUsb::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation
