//
// Created by emiel on 21-05-21.
//

#ifndef RTT_LIBUSBUTILITIES_H
#define RTT_LIBUSBUTILITIES_H

#include <cstdio>
#include <iostream>

namespace rtt {
namespace robothub {

std::string usbutils_bmAttributes_TransferTypeToString();
std::string usbutils_bmAttributes_SyncTypeToString(uint8_t bmAttributes);
std::string usbutils_bmAttributes_usageTypeToString(uint8_t bmAttributes);

std::string usbutils_bEndpointAddress_EndpointNumberToString(uint8_t bEndpointAddress);
std::string usbutils_bEndpointAddress_EndpointDirectionToString(uint8_t bEndpointAddress);

std::string usbutils_descriptorTypeToString(int bDescriptorType);
std::string usbutils_classToString(int bDeviceClass);
std::string usbutils_errorToString(int error);
std::string usbutils_speedToString(int speed);

void usbutils_enumerate();





} // namespace robothub
} // namespace rtt

#endif //RTT_LIBUSBUTILITIES_H
