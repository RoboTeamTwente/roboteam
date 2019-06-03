//
// Created by mrlukasbos on 7-3-19.
//

#include "SerialDeviceManager.h"
#include "utilities.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <fcntl.h>

namespace rtt {
namespace robothub {

SerialDeviceManager::SerialDeviceManager(const std::string& deviceName) : deviceName(deviceName) {

}

bool SerialDeviceManager::ensureDeviceOpen() {
    return fileID != 0;
}

/*
 * Read the data from the serial device and return it
// */
//bool SerialDeviceManager::readDevice() {
//    if (this->ensureDeviceOpen()) {
//
//        // calculate the size of the file
//        // https://stackoverflow.com/a/2409527
//        std::streampos beginOfData, endOfData, size;
//
//        beginOfData = f.tellg();
//        f.seekg(0, std::ios::end);
//        endOfData = f.tellg();
//        size = endOfData - beginOfData;
//
//        // create a data * to store the data
//        char * data = new char[size];
//        f.read(data, size);
//        const char * data2 = data;
//
//        std::string dataStr(data2);
//        return f.good();
//    }
//    return false;
//}

/*
 * Write an array to the serial device
 */
bool SerialDeviceManager::writeToDevice(const packed_protocol_message packet) {
    if (!iswriting) {

        if (this->ensureDeviceOpen()) {
            iswriting = true;


            auto result = write(fileID, packet.data(), packet.size());
            if (result == -1) {
                std::cerr << "write to device failing" << std::endl;
            }
            iswriting = false;
        }
    }
    return 1;
}
void SerialDeviceManager::openDevice() {

    fileID = open(deviceName.c_str(), O_RDWR);

    struct termios tty{};
    memset(&tty, 0, sizeof tty);

    if(tcgetattr(fileID, &tty) != 0) {
        std::cerr <<"Open Device Error"<<std::endl;
    }
    // This is important for 0x0A and 0x09 bytes otherwise they are dropped since ascii
    tty.c_oflag &= ~OPOST;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(fileID, TCSANOW, &tty) != 0) {
        std::cerr <<"Open Device Error"<<std::endl;
    }
}

} // robothub

} // rtt
