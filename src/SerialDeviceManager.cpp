//
// Created by mrlukasbos on 7-3-19.
//

#include "SerialDeviceManager.h"
#include "utilities.h"
#include <iostream>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h> // macOS wants this one

namespace rtt {
namespace robothub {

SerialDeviceManager::SerialDeviceManager(const std::string& deviceName) : deviceName(deviceName) {

}

bool SerialDeviceManager::ensureDeviceOpen() {
    return fileID != 0;
}

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
            auto feedbackPacket = readDevice();
            if(feedbackPacket != nullptr) {
                mostRecentFeedback = std::move(feedbackPacket);
            }
            iswriting = false;
        }
    }
    return true;
}
void SerialDeviceManager::openDevice() {

    fileID = open(deviceName.c_str(), O_RDWR);

    struct termios tty{};
    memset(&tty, 0, sizeof tty);

    if(tcgetattr(fileID, &tty) != 0) {
        std::cerr <<"Open Device Error "<< __FILE__ << "  " <<__LINE__ <<std::endl;
    }
    // This is important for 0x0A and 0x09 bytes otherwise they are dropped since ascii
    tty.c_oflag &= ~OPOST;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(fileID, TCSANOW, &tty) != 0) {
        std::cerr <<"Open Device Error " << __FILE__ << "  " <<__LINE__ <<std::endl;
    }
}


std::shared_ptr<packed_robot_feedback> SerialDeviceManager::readDevice() {
    if (this->ensureDeviceOpen()) {
        packed_robot_feedback packet;
        auto result = read(fileID, packet.data(), packet.size());

        if (result == -1){//|| result != 8) {
            std::cerr << "read from device failing" << std::endl;
        }
        else if(result == 8){
            return std::make_shared<packed_robot_feedback>(packet);
        }

    }
    return nullptr;
}
std::shared_ptr<packed_robot_feedback> SerialDeviceManager::getMostRecentFeedback() const {
    return mostRecentFeedback;
}

void SerialDeviceManager::removeMostRecentFeedback() {
    mostRecentFeedback = nullptr;
}

} // robothub

} // rtt
