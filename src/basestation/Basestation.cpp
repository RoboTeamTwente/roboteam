#include <BasestationConfiguration.h>     // REM packet
#include <BasestationGetConfiguration.h>  // REM packet
#include <RobotCommand.h>                 //REM packet
#include <basestation/LibusbUtilities.h>

#include <basestation/Basestation.hpp>
#include <chrono>
#include <cstring>
#include <iostream>

namespace rtt::robothub::basestation {

constexpr uint16_t BASESTATION_VENDOR_ID = 1155;             // Vendor id all basestations share
constexpr uint16_t BASESTATION_PRODUCT_ID = 22336;           // Product id all basestations share
constexpr int BASESTATION_USB_INTERFACE_NUMBER = 1;          // USB interface we use for connecting with a basestation
constexpr uint8_t TRANSFER_IN_BUFFER_ENDPOINT = 129;         // Endpoint used for reading messages
constexpr uint8_t TRANSFER_OUT_BUFFER_ENDPOINT = 1;          // Endpoint used for writing messages
constexpr unsigned int TRANSFER_IN_TIMEOUT_MS = 100;         // Timeout for reading messages
constexpr unsigned int TRANSFER_OUT_TIMEOUT_MS = 500;        // Timeout for writing messages
constexpr unsigned int PAUSE_ON_TRANSFER_IN_ERROR_MS = 100;  // Pause every time a read fails

Basestation::Basestation(libusb_device* device) : device(device) {
    int error;

    // Open the basestation device, creating the handle we can perform IO on
    error = libusb_open(this->device, &this->deviceHandle);
    if (error) throw FailedToOpenDeviceException("Failed to open libusb device");

    error = libusb_set_auto_detach_kernel_driver(this->deviceHandle, true);
    if (error) {
        libusb_close(this->deviceHandle);
        throw FailedToOpenDeviceException("Failed to set auto detach kernel driver");
    }
    // error = libusb_detach_kernel_driver(this->device_handle, BASESTATION_USB_INTERFACE_NUMBER);
    // if (error) {
    //     std::cout << "Could not detach kernel driver!" << std::endl;
    // } // TODO: Check if this is necessary

    // Claim the interface, so no other programs are connecting with the basestation
    error = libusb_claim_interface(this->deviceHandle, BASESTATION_USB_INTERFACE_NUMBER);
    if (error) {
        libusb_close(this->deviceHandle);
        throw FailedToOpenDeviceException("Failed to claim interface");
    }

    // TODO: Use serial id instead of address to identify basestations
    // Set address, because this can be used to compare basestations
    this->address = libusb_get_device_address(this->device);
    std::cout << "Opened basestation on address: " << (int)this->address << std::endl;

    this->channel = WirelessChannel::UNKNOWN;

    // Start listen thread for incoming messages
    this->shouldListenForIncomingMessages = true;
    this->incomingMessageListenerThread = std::thread(&Basestation::listenForIncomingMessages, this);

    // Current used channel is UNKNOWN, so already send a message that requests this channel
    if (!this->requestChannelOfBasestation()) {
        std::cout << "Could not send request for channel of basestation. You might want to replug this device" << std::endl;
    }
}

Basestation::~Basestation() {
    // Stop the listen thread that reads messages of the basestation
    this->shouldListenForIncomingMessages = false;
    if (this->incomingMessageListenerThread.joinable()) {
        this->incomingMessageListenerThread.join();
    }

    // Close the usb device
    libusb_close(this->deviceHandle);
    std::cout << "Closed basestation!" << std::endl;
}

bool Basestation::operator==(libusb_device* otherDevice) const {
    // Compare the address of the other device with this device, as it *should* uniquely identify them
    // TODO: Compare the devices serial number instead
    uint8_t otherAddress = libusb_get_device_address(otherDevice);
    return this->address == otherAddress;
}

bool Basestation::sendMessageToBasestation(BasestationMessage& message) const { return this->writeBasestationMessage(message); }

void Basestation::setIncomingMessageCallback(std::function<void(const BasestationMessage&)> callback) { this->incomingMessageCallback = callback; }

WirelessChannel Basestation::getChannel() const { return this->channel; }

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

std::string Basestation::wirelessChannelToString(WirelessChannel channel) {
    switch (channel) {
        case WirelessChannel::BLUE_CHANNEL:
            return "blue channel";
        case WirelessChannel::YELLOW_CHANNEL:
            return "yellow channel";
        default:
            return "unknown channel";
    }
}

void Basestation::listenForIncomingMessages() {
    // The incoming data will be written to this message object
    BasestationMessage incomingMessage;

    while (this->shouldListenForIncomingMessages) {
        bool hasReadMessage = this->readBasestationMessage(incomingMessage);
        if (hasReadMessage) {
            this->handleIncomingMessage(incomingMessage);
        } else {
            // If an error occured, try waiting a while before reading again
            std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_ON_TRANSFER_IN_ERROR_MS));
        }
    }
}

// Assumes a payload size bigger than 0
void Basestation::handleIncomingMessage(const BasestationMessage& message) {
    // If a basestation configuration is received, update channel of this basestation class
    if (message.payloadBuffer[0] == PACKET_TYPE_BASESTATION_CONFIGURATION) {
        BasestationConfigurationPayload configurationPayload;
        std::memcpy(&configurationPayload.payload, message.payloadBuffer, PACKET_SIZE_BASESTATION_CONFIGURATION);

        BasestationConfiguration basestationConfiguration;
        decodeBasestationConfiguration(&basestationConfiguration, &configurationPayload);

        WirelessChannel usedChannel = Basestation::remChannelToWirelessChannel(basestationConfiguration.channel);
        this->channel = usedChannel;
    }

    // And in any case, just forward the message to the callback
    if (this->incomingMessageCallback != nullptr) {
        this->incomingMessageCallback(message);
    }
}

bool Basestation::requestChannelOfBasestation() {
    BasestationGetConfiguration getConfigurationCommand;
    getConfigurationCommand.header = PACKET_TYPE_BASESTATION_GET_CONFIGURATION;

    BasestationGetConfigurationPayload getConfigurationPayload;
    encodeBasestationGetConfiguration(&getConfigurationPayload, &getConfigurationCommand);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_BASESTATION_GET_CONFIGURATION;
    std::memcpy(message.payloadBuffer, getConfigurationPayload.payload, message.payloadSize);

    bool sentMessage = this->writeBasestationMessage(message);
    return sentMessage;
}

bool Basestation::readBasestationMessage(BasestationMessage& message) const {
    int error =
        libusb_bulk_transfer(this->deviceHandle, TRANSFER_IN_BUFFER_ENDPOINT, message.payloadBuffer, BASESTATION_MESSAGE_BUFFER_SIZE, &message.payloadSize, TRANSFER_IN_TIMEOUT_MS);

    switch (error) {
        case LIBUSB_SUCCESS: case LIBUSB_ERROR_TIMEOUT:
            // Either received a message correctly, or received nothing at all, so do not send any debug message
            break;
        case LIBUSB_ERROR_NO_DEVICE: {
            std::cout << "WARNING: No device found. Did you unplug the device? " << std::endl;
            break;
        }
        default: {
            std::cout << "ERROR: Failed to read message: " << usbutils_errorToString(error) << std::endl;
        }
    }

    return error == LIBUSB_SUCCESS;
}
bool Basestation::writeBasestationMessage(BasestationMessage& message) const {
    int bytesSent = 0;

    int error = libusb_bulk_transfer(this->deviceHandle, TRANSFER_OUT_BUFFER_ENDPOINT, message.payloadBuffer, message.payloadSize, &bytesSent, TRANSFER_OUT_TIMEOUT_MS);
    if (error) {
        std::cout << "ERROR: Failed to send message: " << usbutils_errorToString(error) << std::endl;
    }

    return error == LIBUSB_SUCCESS;
}

FailedToOpenDeviceException::FailedToOpenDeviceException(const std::string& message) : message(message) {}

const char* FailedToOpenDeviceException::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation