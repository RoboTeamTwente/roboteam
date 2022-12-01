// Created by emiel on 21-05-21.
#pragma once

#include <libusb-1.0/libusb.h>

#include <string>

namespace rtt::robothub::basestation {

libusb_endpoint_direction usbutils_getEndpointDirection(uint8_t bEndpointAddress);
std::string usbutils_endpointDirectionToString(libusb_endpoint_direction direction);

std::string usbutils_bmAttributes_TransferTypeToString(uint8_t bmAttributes);
std::string usbutils_bmAttributes_SyncTypeToString(uint8_t bmAttributes);
std::string usbutils_bmAttributes_usageTypeToString(uint8_t bmAttributes);

std::string usbutils_bEndpointAddress_EndpointNumberToString(uint8_t bEndpointAddress);

std::string usbutils_descriptorTypeToString(int bDescriptorType);
std::string usbutils_classToString(int bDeviceClass);
std::string usbutils_errorToString(int error);
std::string usbutils_speedToString(int speed);

void usbutils_enumerate();

}  // namespace rtt::robothub::basestation