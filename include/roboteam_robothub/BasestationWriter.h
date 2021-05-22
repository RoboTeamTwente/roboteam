//
// Created by emiel on 21-05-21.
//

#ifndef RTT_BASESTATIONWRITER_H
#define RTT_BASESTATIONWRITER_H

#include <libusb-1.0/libusb.h>
#include <roboteam_proto/AICommand.pb.h>
#include "roboteam_proto/Setting.pb.h"

namespace rtt {
namespace robothub {

class BasestationWriter {

public:
    libusb_device_handle* basestation_handle;
    bool running = true;
    int packetsWritten = 0;
    int totalBytesWritten = 0;
    proto::Setting settings;

    BasestationWriter();

    void setBasestationHandle(libusb_device_handle* _basestation_handle);
    void setSettings(proto::Setting &settings);

    void processAIcommand(proto::AICommand &AIcmd);
    void sendCommand(const proto::RobotCommand &cmd);
    void sendSimulatorCommand(const proto::RobotCommand &cmd);

    void run();
};

} // namespace robothub
} // namespace rtt

#endif //RTT_BASESTATIONWRITER_H
