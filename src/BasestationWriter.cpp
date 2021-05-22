//
// Created by emiel on 21-05-21.
//

#include "BasestationWriter.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>

#include "Packing.h"
#include "LibusbUtilities.h"
#include "RobotCommand.h"

namespace rtt {
namespace robothub {

BasestationWriter::BasestationWriter() {
    std::cout << "[BasestationWriter] New BasestationWriter" << std::endl;

    std::cout << (settings.isleft() ? "Left " : "Right ")
              << (settings.isyellow() ? "Yellow " : "Blue ")
              << (settings.serialmode() ? "SERIAL " : "GRSIM ")
              << std::endl;
}

void BasestationWriter::setBasestationHandle(libusb_device_handle *_basestation_handle) {
    basestation_handle = _basestation_handle;
}

void BasestationWriter::setSettings(proto::Setting &_settings) {
    settings = _settings;
}

void BasestationWriter::processAIcommand(proto::AICommand &AIcmd) {
    if(AIcmd.commands_size() == 0) return;

    for(const proto::RobotCommand &cmd : AIcmd.commands()){
        packetsWritten++;
        sendCommand(cmd);
    }
}

void BasestationWriter::sendCommand(const proto::RobotCommand &cmd) {
    proto::World world;
    RobotCommandPayload payload = createEmbeddedCommand(cmd, world, false);
    int bytesSent;
    int error = libusb_bulk_transfer(basestation_handle, 0x01, payload.payload, PACKET_SIZE_ROBOT_COMMAND, &bytesSent, 500);
    if(error)
        std::cout << "[BW::sSC] Error " << usbutils_errorToString(error) << std::endl;

    totalBytesWritten += bytesSent;
}

void BasestationWriter::run() {
    while(packetsWritten != packetsWritten + 1) {
        std::cout << "[BW::run] Tick " << packetsWritten << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

} // namespace robothub
} // namespace rtt