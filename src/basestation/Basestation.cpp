#include <BasestationConfiguration.h>  // REM packet
#include <RobotCommand.h>              //REM packet
#include <basestation/LibusbUtilities.h>

#include <basestation/Basestation.hpp>
#include <chrono>
#include <iostream>

namespace rtt::robothub::basestation {

constexpr int BASESTATION_VENDOR_ID = 1155;
constexpr int BASESTATION_PRODUCT_ID = 22336;
constexpr int USB_INTERFACE_NUMBER = 1;
constexpr unsigned char TRANSFER_IN_BUFFER_ENDPOINT = 0x01;
constexpr unsigned char TRANSFER_OUT_BUFFER_ENDPOINT = 0x81;
constexpr int TRANSFER_IN_BUFFER_SIZE = 4096;
constexpr unsigned int TRANSFER_TIMEOUT_MS = 500;
constexpr unsigned int INCOMING_MESSAGE_TIMEOUT_MS = 100;
constexpr unsigned int INCOMING_MESSAGE_ERROR_PAUSE_MS = 100;

Basestation::Basestation(libusb_device* device) : device(device) {
    int error;

    error = libusb_open(this->device, &this->device_handle);
    if (error) throw FailedToOpenDeviceException("Failed to open libusb device");

    error = libusb_set_auto_detach_kernel_driver(this->device_handle, true);
    if (error) {
        libusb_close(this->device_handle);
        throw FailedToOpenDeviceException("Failed to set auto detach kernel driver");
    }
    error = libusb_claim_interface(this->device_handle, USB_INTERFACE_NUMBER);
    if (error) {
        libusb_close(this->device_handle);
        throw FailedToOpenDeviceException("Failed to claim interface");
    }

    // Set address, because this can be used to compare basestations
    this->address = libusb_get_device_address(this->device);
    std::cout << "Opened basestation on address: " << (int)this->address << std::endl;

    this->currentlyUsedChannel = WirelessChannel::UNKNOWN;

    // Start listen thread for incoming messages
    this->shouldListenForIncomingMessages = true;
    this->incomingMessageListenerThread = std::thread(&Basestation::listenForIncomingMessages, this);
}

Basestation::~Basestation() {
    this->shouldListenForIncomingMessages = false;
    if (this->incomingMessageListenerThread.joinable()) {
        this->incomingMessageListenerThread.join();
    }

    libusb_close(this->device_handle);
}

bool Basestation::operator==(libusb_device* otherDevice) const {
    uint8_t otherAddress = libusb_get_device_address(otherDevice);
    return this->address == otherAddress;
}

bool Basestation::sendMessageToBasestation(const BasestationMessage& message) const {
    int bytesSent;
    int error = libusb_bulk_transfer(this->device_handle, TRANSFER_OUT_BUFFER_ENDPOINT, message.payload, message.payload_size, &bytesSent, TRANSFER_TIMEOUT_MS);

    if (error) {
        std::cout << "Failed to send message to basestation" << std::endl;
        return false;
    }
    return true;
}

void Basestation::setIncomingMessageCallback(std::function<void(const BasestationMessage&)> callback) { this->incomingMessageCallback = callback; }

WirelessChannel Basestation::getCurrentUsedChannel() const { return this->currentlyUsedChannel; }

bool Basestation::requestChannelChange(WirelessChannel newChannel) {
    if (newChannel == WirelessChannel::UNKNOWN) {
        return false;  // We cannot ask for an unknown channel
    }
    if (this->isWaitingForChannelChange()) {
        return false;  // We were already waiting for a channel change, so do not request again for a change
    }
    if (newChannel == this->currentlyUsedChannel) {
        return false;  // Why send a channel change request if the frequency is already used
    }

    this->wantedChannel = newChannel;
    return this->sendConfigurationCommand(newChannel);
}

bool Basestation::isWaitingForChannelChange() const { return this->wantedChannel != this->currentlyUsedChannel; }

bool Basestation::isDeviceABasestation(libusb_device* device) {
    libusb_device_descriptor descriptor;
    int r = libusb_get_device_descriptor(device, &descriptor);
    if (r < 0) {
        std::cout << "Error: Failed to get device descriptor" << std::endl;
        return false;
    }

    return descriptor.idVendor == BASESTATION_VENDOR_ID && descriptor.idProduct == BASESTATION_PRODUCT_ID;
}

bool Basestation::wirelessChannelToREMChannel(WirelessChannel channel) {
    bool remChannel;
    switch (channel) {
        case WirelessChannel::BLUE_CHANNEL:
            remChannel = true;
            break;
        case WirelessChannel::YELLOW_CHANNEL:
            remChannel = false;
            break;
        default:
            remChannel = false;
            break;
    }
    return remChannel;
}

WirelessChannel Basestation::remChannelToWirelessChannel(bool channel) { return channel ? WirelessChannel::BLUE_CHANNEL : WirelessChannel::YELLOW_CHANNEL; }

void Basestation::listenForIncomingMessages() {
    while (this->shouldListenForIncomingMessages) {
        BasestationMessage message = {.payload_size = 0, .payload = nullptr};

        int error =
            libusb_bulk_transfer(this->device_handle, TRANSFER_IN_BUFFER_ENDPOINT, message.payload, TRANSFER_IN_BUFFER_SIZE, &message.payload_size, INCOMING_MESSAGE_TIMEOUT_MS);

        if (error != LIBUSB_SUCCESS && error != LIBUSB_ERROR_TIMEOUT) {
            std::cout << "Error while reading message: " << usbutils_errorToString(error) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(INCOMING_MESSAGE_ERROR_PAUSE_MS));
            continue;
        }
        if (message.payload_size > 0) {
            this->handleIncomingMessage(message);
        }
    }
}

// Assumes a payload size bigger than 0
void Basestation::handleIncomingMessage(const BasestationMessage& message) {
    // If a used frequency is received, update the basestations currentlyUsedChannel
    if (message.payload[0] == PACKET_TYPE_BASESTATION_CONFIGURATION) {
        BasestationConfigurationPayload* configPayload = (BasestationConfigurationPayload*)message.payload;
        BasestationConfiguration configuration;
        decodeBasestationConfiguration(&configuration, configPayload);

        WirelessChannel usedChannel = configuration.channel ? WirelessChannel::YELLOW_CHANNEL : WirelessChannel::BLUE_CHANNEL;
        this->updateCurrentlyUsedChannel(usedChannel);
    }

    if (this->incomingMessageCallback != nullptr) {
        this->incomingMessageCallback(message);
    }
}

void Basestation::updateCurrentlyUsedChannel(WirelessChannel newChannel) {
    if (this->wantedChannel == WirelessChannel::UNKNOWN) {
        // This is the first time we know what frequency the basestation is, so set the wanted frequency to the same thing
        this->wantedChannel = newChannel;
    }
    this->currentlyUsedChannel = newChannel;
}

bool Basestation::sendConfigurationCommand(WirelessChannel newChannel) {
    BasestationConfiguration configurationCommand;
    configurationCommand.header = PACKET_TYPE_BASESTATION_CONFIGURATION;
    configurationCommand.remVersion = LOCAL_REM_VERSION;
    configurationCommand.channel = Basestation::wirelessChannelToREMChannel(newChannel);

    BasestationConfigurationPayload commandPayload;
    encodeBasestationConfiguration(&commandPayload, &configurationCommand);

    BasestationMessage message = {.payload_size = PACKET_SIZE_BASESTATION_CONFIGURATION, .payload = commandPayload.payload};

    return this->sendMessageToBasestation(message);
}

FailedToOpenDeviceException::FailedToOpenDeviceException(const std::string& message) : message(message) {}

const char* FailedToOpenDeviceException::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation