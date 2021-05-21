//
// Created by emiel on 22-05-21.
//

#include "BasestationReader.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>

#include "Packing.h"
#include "LibusbUtilities.h"

namespace rtt {
namespace robothub {

BasestationReader::BasestationReader() {
    std::cout << "[BasestationReader] New BasestationReader" << std::endl;
}

void BasestationReader::setBasestationHandle(libusb_device_handle *_basestation_handle) {
    basestation_handle = _basestation_handle;
}

void BasestationReader::run() {
    std::cout << "[BR][run] Running" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t buffer[4906];
    int actual_length = 0;

    while(running){

        int error = libusb_bulk_transfer(basestation_handle, 0x81, buffer, 4096, &actual_length, 100);
        if (actual_length == 0) continue;
        if (error) std::cout << "ERROR receiving : " << usbutils_errorToString(error) << std::endl;

        if (buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("LOG: ");
            for (int i = 1; i < actual_length; i++)
                printf("%c", buffer[i]);
        }

        if (buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS) {
//            printStatistics(buffer);
            std::cout << "[BR][run] Statistics" << std::endl;
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_FEEDBACK) {
            std::cout << "[BR][run] Feedback!" << std::endl;
            totalBytesReceived += actual_length;
        }
    }

    std::cout << "[readThread] Terminating" << std::endl;
}

} // namespace robothub
} // namespace rtt