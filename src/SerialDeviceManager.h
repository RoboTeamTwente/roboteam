//
// Created by mrlukasbos on 7-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
#define ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H

#include <fstream>
#include "packing.h"

namespace rtt {
namespace robothub {

class SerialDeviceManager {
public:
    explicit SerialDeviceManager() = default;
    explicit SerialDeviceManager(const std::string &deviceName);
    bool ensureDeviceOpen();
//    bool readDevice();  TODO remake
    bool writeToDevice(packed_protocol_message packet);
    void openDevice();


private:
    int fileID = 0;
    std::string deviceName;
    bool iswriting = false;
};

}
}

#endif //ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
