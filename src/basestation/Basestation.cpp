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
    if (!Basestation::isDeviceABasestation(device)) {
        throw FailedToOpenDeviceException("Device is not a basestation");
    }

    int error;

    // Open the basestation device, creating the handle we can perform IO on
    error = libusb_open(this->device, &this->deviceHandle);
    if (error) throw FailedToOpenDeviceException("Failed to open libusb device");

    error = libusb_set_auto_detach_kernel_driver(this->deviceHandle, true);
    if (error) {
        libusb_close(this->deviceHandle);
        throw FailedToOpenDeviceException("Failed to set auto detach kernel driver");
    }

    // Claim the interface, so no other programs are connecting with the basestation
    error = libusb_claim_interface(this->deviceHandle, BASESTATION_USB_INTERFACE_NUMBER);
    if (error) {
        libusb_close(this->deviceHandle);
        throw FailedToOpenDeviceException("Failed to claim interface");
    }

    this->serialID = Basestation::getSerialIdentifierOfDevice(this->device);

    // Start listen thread for incoming messages
    this->shouldListenForIncomingMessages = true;
    this->incomingMessageListenerThread = std::thread(&Basestation::listenForIncomingMessages, this);
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
    // Compare the serial identifier of the other device with this device
    uint8_t otherSerialID = Basestation::getSerialIdentifierOfDevice(otherDevice);
    return this->serialID == otherSerialID;
}
bool Basestation::operator==(uint8_t otherBasestationSerialID) const {
    return otherBasestationSerialID == this->serialID;
}
bool Basestation::operator==(std::shared_ptr<Basestation> otherBasestation) const {
    return otherBasestation != nullptr
        && this->serialID == otherBasestation->serialID;
}

bool Basestation::sendMessageToBasestation(BasestationMessage& message) const { return this->writeBasestationMessage(message); }

void Basestation::setIncomingMessageCallback(std::function<void(const BasestationMessage&, uint8_t serialID)> callback) { this->incomingMessageCallback = callback; }

uint8_t Basestation::getSerialID() const {
    return this->serialID;
}

bool Basestation::isDeviceABasestation(libusb_device* device) {
    libusb_device_descriptor descriptor;
    int r = libusb_get_device_descriptor(device, &descriptor);
    if (r < 0) {
        std::cout << "Error: Failed to get device descriptor" << std::endl;
        return false;
    }

    return descriptor.idVendor == BASESTATION_VENDOR_ID && descriptor.idProduct == BASESTATION_PRODUCT_ID;
}

void Basestation::listenForIncomingMessages() {
    // The incoming data will keep being written to this message object over and over
    BasestationMessage incomingMessage;

    while (this->shouldListenForIncomingMessages) {
        bool hasReadMessage = this->readBasestationMessage(incomingMessage);
        if (hasReadMessage) {
            // TODO: Protect callback with mutex. Other threads are theoretically able to set the callback to nullptr,
            // so at this point, calling the callback could result in error.
            if (this->incomingMessageCallback != nullptr) {
                this->incomingMessageCallback(incomingMessage, this->serialID);
            }
        } else {
            // If an error occured, try waiting a while before reading again
            std::this_thread::sleep_for(std::chrono::milliseconds(PAUSE_ON_TRANSFER_IN_ERROR_MS));
        }
    }
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

uint8_t Basestation::getSerialIdentifierOfDevice(libusb_device* device) {
    libusb_device_descriptor deviceDescriptor = {};
    libusb_get_device_descriptor(device, &deviceDescriptor);
    return deviceDescriptor.iSerialNumber;
}

FailedToOpenDeviceException::FailedToOpenDeviceException(const std::string& message) : message(message) {}

const char* FailedToOpenDeviceException::what() const noexcept { return this->message.c_str(); }

}  // namespace rtt::robothub::basestation