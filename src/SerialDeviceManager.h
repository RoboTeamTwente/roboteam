//
// Created by mrlukasbos on 7-3-19.
//

#ifndef ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
#define ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H

namespace rtt {
namespace robothub {

class SerialDeviceManager {
public:
    SerialDeviceManager() = default;
    bool openDevice();
    bool isOpen();
};

}
}

#endif //ROBOTEAM_ROBOTHUB_SERIALDEVICEMANAGER_H
