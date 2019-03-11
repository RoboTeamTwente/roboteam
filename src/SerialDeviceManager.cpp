//
// Created by mrlukasbos on 7-3-19.
//

#include "SerialDeviceManager.h"
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
    ROS_INFO("[EnsureDeviceOpen] Opening serial device %s",  deviceName.c_str());

    if (!f.is_open()) {
        f.open(deviceName, std::fstream::binary | std::fstream::in | std::fstream::out);
        if (f.fail()) {
            ROS_ERROR("[EnsureDeviceOpen] Failed to open Serial Device: %s", deviceName.c_str());
            return false;
        }
    }
    ROS_INFO("[EnsureDeviceOpen] Serial device opened: %s", deviceName.c_str());
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
        char * data = new char();
        f.read(data, size);
        return f.good();
    }
    return false;
}

/*
 * Write an array to the serial device
 */
bool SerialDeviceManager::writeToDevice(packed_protocol_message packet) {
    if (this->EnsureDeviceOpen()) {
        f.write(reinterpret_cast<char*>(packet.data()), packet.size());
        return f.good();
    }
    return false;
}

/*
 * Close the serial device
 */
bool SerialDeviceManager::closeDevice() {
    f.close();
    return !f.is_open();
}

/*
 * Close the serial device upon destruction
 */
SerialDeviceManager::~SerialDeviceManager() {
    this->closeDevice();
}


} // robothub
} // rtt
