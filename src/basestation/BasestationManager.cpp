#include <RobotStateInfo.h>
#include <basestation/LibusbUtilities.h>
#include <constants.h>

#include <basestation/BasestationManager.hpp>
#include <iostream>

namespace rtt::robothub::basestation {

BasestationManager::BasestationManager() {
    int error;
    error = libusb_init(&this->usb_context);
    if (error) {
        throw FailedToInitializeLibUsb("Failed to initialize libusb"); // TODO: Handle error code
    }

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
    this->basestations.clear();

    libusb_exit(this->usb_context);
}

bool BasestationManager::sendRobotCommand(const RobotCommand& command, bool toTeamYellow) {
    // For now, just pick the first basestation.
    // TODO: Make sure the right basestation is picked from the list
    auto basestation = this->basestations.size() > 0 ? this->basestations[0] : nullptr;
    if (basestation != nullptr) {
        RobotCommand copy = command;
        
        BasestationMessage message;
        RobotCommandPayload payload;
        encodeRobotCommand(&payload, &copy);

        message.payload = payload.payload;
        message.payload_size = PACKET_SIZE_ROBOT_COMMAND;

        return basestation->sendMessageToBasestation(message);
    }

    return false;
}

void BasestationManager::setFeedbackCallback(const std::function<void(const RobotFeedback &)>& callback) {
    this->feedbackCallbackFunction = callback;
}

void BasestationManager::updateBasestationsList(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    // Remove basestations that are not plugged in anymore
    auto iterator = this->basestations.begin();
    while (iterator != this->basestations.end()) {
        auto basestation = *iterator;
        if (!basestationIsInDeviceList(basestation, pluggedBasestationDevices)) {
            // This basestation is not plugged in anymore -> remove it
            iterator = this->basestations.erase(iterator);
        } else {
            ++iterator;
        }
    }

    // Add plugged in basestations that are not in the list yet
    for (libusb_device* pluggedBasestationDevices : pluggedBasestationDevices) {
        if (!deviceIsInBasestationList(pluggedBasestationDevices, this->basestations)) {
            // This basestation is plugged in but not in the list -> add it
            try {
                auto newBasestation = std::make_shared<Basestation>(pluggedBasestationDevices);
                this->basestations.push_back(newBasestation);
            } catch (FailedToOpenDeviceException e) {
                std::cout << "Error: " << e.what() << std::endl;
                std::cout << "Did you edit your user permissions?" << std::endl;
            }
        }
    }
}

void BasestationManager::listenForBasestationPlugs() {
    while (this->shouldListenForBasestationPlugs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Get a list of devices
        libusb_device** device_list;
        ssize_t device_count = libusb_get_device_list(this->usb_context, &device_list);

        std::vector<libusb_device*> basestationDevices = filterBasestationDevices(device_list, device_count);
        this->updateBasestationsList(basestationDevices);

        // Free the list of devices
        libusb_free_device_list(device_list, true);
    }
}

bool BasestationManager::deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations) {
    bool deviceIsInList = false;

    for (auto basestation : basestations) {
        if (*basestation == device) {
            deviceIsInList = true;
            break;
        }
    }

    return deviceIsInList;
}

bool BasestationManager::basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices) {
    bool basestationIsInList = false;

    for (auto device : devices) {
        if (*basestation == device) {
            basestationIsInList = true;
            break;
        }
    }

    return basestationIsInList;
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
