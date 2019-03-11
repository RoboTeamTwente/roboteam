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
    bool EnsureDeviceOpen();
    bool readDevice();
    bool closeDevice();
    bool writeToDevice(packed_protocol_message packet);
    ~SerialDeviceManager();

private:
    std::fstream f;
    std::string deviceName;

    std::string StandardDeviceNames[3] = {
            "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_00000000001A-if00",
            "/dev/serial/by-id/usb-STMicroelectronics_Basestation_078-if00",
            "/dev/serial/by-id/usb-STMicroelectronics_Basestation_080-if00",
    };

    std::string serial_file_path_param = "none";
    std::string serial_file_path = "No basestation selected!";
};

}
}

#endif //ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
