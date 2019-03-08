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
    explicit SerialDeviceManager(const std::string &deviceName);
    bool EnsureDeviceOpen();
    bool readDevice();
    bool closeDevice();
    bool writeToDevice(rtt::packed_protocol_message packet);
    ~SerialDeviceManager();

private:
    bool serialPortOpen = false;

    std::fstream f;
    std::string deviceName;
};

}
}

#endif //ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
