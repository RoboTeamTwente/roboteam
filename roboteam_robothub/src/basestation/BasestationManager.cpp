#include <REM_BaseTypes.h>
#include <REM_Log.h>
#include <REM_Packet.h>
#include <roboteam_utils/Print.h>

#include <basestation/BasestationManager.hpp>
#include <cstring>
#include <sstream>

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
    message.payloadSize = REM_PACKET_SIZE_REM_ROBOT_COMMAND;
    std::memcpy(&message.payloadBuffer, payload.payload, message.payloadSize);

    int bytesSent = this->basestationCollection->sendMessageToBasestation(message, color);
    return bytesSent;
}

int BasestationManager::sendRobotBuzzerCommand(const REM_RobotBuzzer& command, rtt::Team color) const {
    REM_RobotBuzzer copy = command;

    REM_RobotBuzzerPayload payload;
    encodeREM_RobotBuzzer(&payload, &copy);

    BasestationMessage message;
    message.payloadSize = REM_PACKET_SIZE_REM_ROBOT_BUZZER;
    std::memcpy(message.payloadBuffer, payload.payload, message.payloadSize);

    int bytesSent = this->basestationCollection->sendMessageToBasestation(message, color);
    return bytesSent;
}

void BasestationManager::setFeedbackCallback(const std::function<void(const REM_RobotFeedback&, rtt::Team)>& callback) { this->feedbackCallbackFunction = callback; }

void BasestationManager::setRobotStateInfoCallback(const std::function<void(const REM_RobotStateInfo&, rtt::Team)>& callback) { this->robotStateInfoCallbackFunction = callback; }

void BasestationManager::setBasestationLogCallback(const std::function<void(const std::string&, rtt::Team)>& callback) { this->basestationLogCallback = callback; }

void BasestationManager::listenForBasestationPlugs() {
    while (this->shouldListenForBasestationPlugs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Get a list of devices
        libusb_device** device_list;
        auto device_count = libusb_get_device_list(this->usbContext, &device_list);

        std::vector<libusb_device*> basestationDevices = filterBasestationDevices(device_list, static_cast<int>(device_count));

        this->basestationCollection->updateBasestationCollection(basestationDevices);

        // Free the list of devices
        libusb_free_device_list(device_list, true);
    }
}

BasestationManagerStatus BasestationManager::getStatus() const {
    BasestationManagerStatus status = {.basestationCollection = this->basestationCollection->getStatus()};

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
    REM_PacketPayload* packetPayload = (REM_PacketPayload*)message.payloadBuffer;
    uint32_t payloadSize = REM_Packet_get_payloadSize(packetPayload);
    uint8_t packetType = REM_Packet_get_header(packetPayload);

    if (message.payloadSize != payloadSize) {
        RTT_ERROR("Payload size of message does not match the size specified in the packet header. Received size: ", message.payloadSize, ", indicated size: ", payloadSize);
        return;
    }

    switch (packetType) {
        case REM_PACKET_TYPE_REM_ROBOT_FEEDBACK: {
            REM_RobotFeedbackPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, message.payloadSize);

            REM_RobotFeedback feedback;
            decodeREM_RobotFeedback(&feedback, &payload);

            this->callFeedbackCallback(feedback, color);
            break;
        }
        case REM_PACKET_TYPE_REM_ROBOT_STATE_INFO: {
            REM_RobotStateInfoPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, message.payloadSize);

            REM_RobotStateInfo stateInfo;
            decodeREM_RobotStateInfo(&stateInfo, &payload);

            this->callRobotStateInfoCallback(stateInfo, color);
            break;
        }
        case REM_PACKET_TYPE_REM_LOG: {
            REM_LogPayload payload;
            std::memcpy(payload.payload, message.payloadBuffer, REM_PACKET_SIZE_REM_LOG);

            REM_Log log;
            decodeREM_Log(&log, &payload);

            uint32_t logLength = message.payloadSize - REM_PACKET_SIZE_REM_LOG;

            static constexpr int charsToSkip = 1;  // We skip the last character of log messages, as its always a '\n'

            // Convert the received log bytes into a string
            std::ostringstream oss;
            for (int i = REM_PACKET_SIZE_REM_LOG; i < message.payloadSize - charsToSkip; i++) {
                oss << static_cast<char>(message.payloadBuffer[i]);  // Convert uint32_t byte to char and store in stream
            }
            std::string logMessage = oss.str();

            if (logMessage.length() == logLength - charsToSkip) {
                // We correctly retrieved the log message
                this->callBasestationLogCallback(logMessage, color);
            } else if (logLength == 0) {
                // We received an intended empty log message... But why?
                RTT_WARNING("Received empty basestation log message")
            } else {
                // We ended up with a message shorter (or longer?) than what the length should have been
                RTT_ERROR("BasestationLogMessage turned out wrongly sized (", logMessage.length(), " instead of ", logLength - charsToSkip, ")")
            }

            break;
        }
        case REM_PACKET_TYPE_REM_ROBOT_PIDGAINS: {
            break;
        }
        default: {
            RTT_WARNING("Unhandled basestation message: ", packetType)
            break;
        }
            // TODO: Other packets can be handled as well
    }
}

void BasestationManager::callFeedbackCallback(const REM_RobotFeedback& feedback, rtt::Team color) const {
    if (this->feedbackCallbackFunction != nullptr) this->feedbackCallbackFunction(feedback, color);
}

void BasestationManager::callRobotStateInfoCallback(const REM_RobotStateInfo& robotStateInfo, rtt::Team color) const {
    if (this->robotStateInfoCallbackFunction != nullptr) this->robotStateInfoCallbackFunction(robotStateInfo, color);
}

void BasestationManager::callBasestationLogCallback(const std::string& basestationLog, rtt::Team color) const {
    if (this->basestationLogCallback != nullptr) this->basestationLogCallback(basestationLog, color);
}

FailedToInitializeLibUsb::FailedToInitializeLibUsb(const std::string& message) : message(message) {}
const char* FailedToInitializeLibUsb::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation
