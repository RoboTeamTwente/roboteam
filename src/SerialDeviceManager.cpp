//
// Created by mrlukasbos on 7-3-19.
//

#include "SerialDeviceManager.h"
#include "utilities.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>

namespace rtt {
namespace robothub {

SerialDeviceManager::SerialDeviceManager(const std::string& deviceName) : deviceName(deviceName) { }

/*
 * Open a device it is was not open yet
 * Returns true if the device opens
 * Return false if it fails.
 */
bool SerialDeviceManager::EnsureDeviceOpen() {
    if (!f.is_open()) {
        f.open(deviceName, std::fstream::binary | std::fstream::app | std::fstream::in | std::fstream::out);
        if (f.fail()) {
            ROS_ERROR("[EnsureDeviceOpen] Failed to open Serial Device: %s", deviceName.c_str());
            return false;
        }
    }
    return true;
}

/*
 * Read the data from the serial device and return it
 */
bool SerialDeviceManager::readDevice() {
    if (this->EnsureDeviceOpen()) {

        // calculate the size of the file
        // https://stackoverflow.com/a/2409527
        std::streampos beginOfData, endOfData, size;
        beginOfData = f.tellg();
        f.seekg(0, std::ios::end);
        endOfData = f.tellg();
        size = endOfData - beginOfData;

        // create a data * to store the data
        char * data = new char[size];
        f.read(data, size);
        const char * data2 = data;

        std::string dataStr(data2);
        return f.good();
    }
    return false;
}

/*
 * Write an array to the serial device
 */
bool SerialDeviceManager::writeToDevice(const packed_protocol_message packet) {
    if (!iswriting) {

        if (this->EnsureDeviceOpen()) {
            iswriting = true;

            f.clear();
            f.write((const char*) packet.data(), packet.size());
            f.flush();
            usleep(1/(60*16)*1000000);
            iswriting = false;
        }
    }
    return 1;

    ROS_ERROR("[writeToDevice] the device is closed!");

    return false;
}


} // robothub
} // rtt
