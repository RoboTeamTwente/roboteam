//
// Created by emiel on 22-05-21.
//

#include "RobotHub.h"

#include <cstdio>
#include <iostream>
#include "LibusbUtilities.h"

namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    writer = std::make_unique<BasestationWriter>();
    reader = std::make_unique<BasestationReader>();

    robotCommandSubscriber = std::make_unique<proto::Subscriber<proto::AICommand>>(
        robotCommandChannel, &BasestationWriter::processAIcommand, writer.get()
    );

    worldStateSubscriber = std::make_unique<proto::Subscriber<proto::World>>(
        proto::WORLD_CHANNEL, &RobotHub::processWorldState, this
    );

    settingsSubscriber = std::make_unique<proto::Subscriber<proto::Setting>>(
        settingsChannel, &RobotHub::processSettings, this
    );

//    feedbackPublisher = new proto::Publisher<proto::RobotFeedback>(feedbackChannel);

}

bool RobotHub::startBasestation(){
    // =========================== ESTABLISH CONNECTION =========================== //
    int error;
    // Initialize USB context
    libusb_context *ctx;
    error = libusb_init(&ctx);
    if(error) std::cout << "error : " << usbutils_errorToString(error) << std::endl;
    // Set logging level
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if(error) std::cout << "error : " << usbutils_errorToString(error) << std::endl;
    // Open connection
    basestation_handle = nullptr;
    if(!openBasestation(ctx, &basestation_handle)){
        std::cout << "[main] Basestation has not been found.. aborting" << std::endl;
        return -1;
    }
    // =========================== CONNECTION ESTABLISHED =========================== //
}

bool RobotHub::openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle){

    libusb_device* basestation_dev = nullptr;
    libusb_device **list;
    int num_devices = libusb_get_device_list(ctx, &list);

    int error;
    for (int i = 0; i < num_devices; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor desc{};

        error = libusb_get_device_descriptor(device, &desc);
        if (error) std::cout << "[findBasestation] Error : " << usbutils_errorToString(error) << std::endl;
        if (desc.idVendor == 0x0483 && desc.idProduct == 0x5740) {
            basestation_dev = device;
            int deviceBus = libusb_get_bus_number(device);
            int deviceAddress = libusb_get_device_address(device);
            int deviceSpeed = libusb_get_device_speed(device);
            printf("[findBasestation] Basestation found. %04x:%04x (bus %d, device %d) %s\n",
                   desc.idVendor, desc.idProduct, deviceBus, deviceAddress, usbutils_speedToString(deviceSpeed).c_str());
        }
    }

    if(basestation_dev == nullptr) {
        std::cout << "[findBasestation] Basestation not found" << std::endl;
        libusb_free_device_list(list, true);
        return false;
    }

    error = libusb_open(basestation_dev, basestation_handle);
    if(error){
        std::cout << "Error while trying to open handle : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /** libusb_set_auto_detach_kernel_driver() Enable/disable libusb's automatic kernel driver detachment. When this is
     * enabled libusb will automatically detach the kernel driver on an interface when claiming the interface, and
     * attach it when releasing the interface.
     */
    error = libusb_set_auto_detach_kernel_driver(*basestation_handle, 1);
    if(error){
        std::cout << "Error while enabling auto detach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }
    /** libusb_claim_interface() Claim an interface on a given device handle. You must claim the interface you wish to
     * use before you can perform I/O on any of its endpoints.
     */
    error = libusb_claim_interface(*basestation_handle, 1);
    if(error){
        std::cout << "Error while claiming interface : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    return true;
}

void RobotHub::processWorldState(proto::World &world) {
    std::lock_guard<std::mutex> lock(worldLock);
    LastWorld = world;
}

void RobotHub::processSettings(proto::Setting &setting) {

//    grsimCommander->setGrsim_ip(setting.robothubsendip());
//    grsimCommander->setGrsim_port(setting.robothubsendport());
    isLeft = setting.isleft();
//    grsimCommander->setColor(setting.isyellow());
    isYellow = setting.isyellow();

    if (setting.serialmode()) {
        mode = utils::Mode::SERIAL;
    } else {
        mode = utils::Mode::GRSIM;
    }

    std::cout << "[RobotHub::processSettings] "
    << (isLeft ? "Left " : "Right ")
    << (isYellow ? "Yellow " : "Blue ")
    << (mode == utils::Mode::SERIAL ? "SERIAL " : "GRSIM ")
    << std::endl;
}

}  // namespace robothub
}  // namespace rtt

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub();
}