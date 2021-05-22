//
// Created by emiel on 22-05-21.
//

#include "RobotHub.h"

#include <cstdio>
#include <iostream>

#include "LibusbUtilities.h"
#include "RobotCommand.h"
#include "Packing.h"


int hotplug_callback_attach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data){
    std::cout << "[hotplug_callback_attach] Event triggered!" << std::endl;
    rtt::robothub::RobotHub* hub = (rtt::robothub::RobotHub*)(user_data);
    hub->handleBasestationDetach(device);
    //(rtt::robothub::RobotHub*)(user_data)->handleBasestationDetach();
    return 0;
}

int hotplug_callback_detach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data){
    std::cout << "[hotplug_callback_detach] Event triggered!" << std::endl;
    return 0;
}


namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    grsimCommander = std::make_shared<GRSimCommander>();
    reader = std::make_unique<BasestationReader>();

    startBasestation();
    basestation_handle = nullptr;
    reader->setBasestationHandle(basestation_handle);
    readerThread = std::thread([&]() { reader->run(); });
}

void RobotHub::subscribe(){
    robotCommandSubscriber = std::make_unique<proto::Subscriber<proto::AICommand>>(
            proto::ROBOT_COMMANDS_PRIMARY_CHANNEL, &RobotHub::processAIcommand, this
    );

    worldStateSubscriber = std::make_unique<proto::Subscriber<proto::World>>(
            proto::WORLD_CHANNEL, &RobotHub::processWorldState, this
    );

    settingsSubscriber = std::make_unique<proto::Subscriber<proto::Setting>>(
            proto::SETTINGS_PRIMARY_CHANNEL, &RobotHub::processSettings, this
    );
}

void RobotHub::run(){
    std::cout << "[RobotHub::run] Initialize " << std::endl;

    /* Subscribe to relevant topics */
    subscribe();

    /* Start basestation reading thread */
    std::thread readThread();

    timeval usb_event_timeout{.tv_usec=100000}; // 100ms timeout

    std::cout << "[RobotHub::run] Loop " << std::endl;

    while(running){
        std::cout << "[RobotHub::run] Tick " << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        /* Initialize libusb, and subscribe to relevant usb events */
        if(!usb_initialized && settings.serialmode()) {
            if (!startBasestation()) {
                std::cout << "[RobotHub::run] Error while initializing basestation. Aborting..." << std::endl;
                return;
            }
            usb_initialized = true;
        }

        if(usb_initialized)
            libusb_handle_events_timeout(ctx, &usb_event_timeout);

    }
}

void RobotHub::handleBasestationAttach(libusb_device* device){
    std::cout << "[RobotHub::handleBasestationAttach] " << std::endl;
    basestation_device = device;

    int error;
    error = libusb_open(basestation_device, &basestation_handle);
    if(error){
        std::cout << "[RobotHub::handleBasestationAttach] Error while trying to open handle : " << usbutils_errorToString(error) << std::endl;
        return;
    }

    /** libusb_set_auto_detach_kernel_driver() Enable/disable libusb's automatic kernel driver detachment. When this is
     * enabled libusb will automatically detach the kernel driver on an interface when claiming the interface, and
     * attach it when releasing the interface.
     */
    error = libusb_set_auto_detach_kernel_driver(basestation_handle, 1);
    if(error){
        std::cout << "[RobotHub::handleBasestationAttach] Error while enabling auto detach : " << usbutils_errorToString(error) << std::endl;
        return;
    }
    /** libusb_claim_interface() Claim an interface on a given device handle. You must claim the interface you wish to
     * use before you can perform I/O on any of its endpoints.
     */
    error = libusb_claim_interface(basestation_handle, 1);
    if(error){
        std::cout << "[RobotHub::handleBasestationAttach] Error while claiming interface : " << usbutils_errorToString(error) << std::endl;
        return;
    }

    reader->setBasestationHandle(basestation_handle);
}

void RobotHub::handleBasestationDetach(libusb_device* device){
    libusb_release_interface(basestation_handle, 1);
    libusb_close(basestation_handle);
    basestation_handle = nullptr;
    reader->setBasestationHandle(basestation_handle);
    std::cout << "[RobotHub::handleBasestationDetach] " << std::endl;
}

bool RobotHub::startBasestation(){
    int error;

    // Initialize USB context
    error = libusb_init(&ctx);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_init : " << usbutils_errorToString(error) << std::endl;
        return false;
    }
    // Set logging level
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_set_option : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to detach event */
    error = libusb_hotplug_register_callback(
            ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS,
            0x0483, 0x5740, LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback_detach, (void*)this, &callback_handle_detach);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback detach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to attach event */
    error = libusb_hotplug_register_callback(
            ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE,
            0x0483, 0x5740, LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback_attach, (void*) this, &callback_handle_attach);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback attach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    return true;
    // Open connection
    basestation_handle = nullptr;
    if(!openBasestation(ctx, &basestation_handle)){
        std::cout << "[main] Basestation has not been found.. aborting" << std::endl;
        return false;
    }
    return true;
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

void RobotHub::sendSerialCommand(const proto::RobotCommand &cmd) {
    if(basestation_handle == nullptr){
        std::cout << "[RobotHub::sendSerialCommand] Basestation not present!" << std::endl;
        return;
    }

    RobotCommandPayload payload = createEmbeddedCommand(cmd, world, false);
    int bytesSent;
    int error = libusb_bulk_transfer(basestation_handle, 0x01, payload.payload, PACKET_SIZE_ROBOT_COMMAND, &bytesSent, 500);
    if(error){
        std::cout << "[RobotHub::sendSerialCommand] Error while sending to basestation. Resetting connection .." << std::endl;
        std::cout << "[RobotHub::sendSerialCommand] Error : " << usbutils_errorToString(error) << std::endl;
    }

}

void RobotHub::sendGrSimCommand(const proto::RobotCommand &cmd) {
    this->grsimCommander->queueGRSimCommand(cmd);
}

void RobotHub::readBasestation(){
    std::cout << "[RobotHub::readBasestation] Running" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t buffer[4906];
    int actual_length = 0;

    while(running){
        if(basestation_handle == nullptr){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        int error = libusb_bulk_transfer(basestation_handle, 0x81, buffer, 4096, &actual_length, 100);
        if(error != LIBUSB_SUCCESS and error != LIBUSB_ERROR_TIMEOUT){
            std::cout << "[BasestationReader::run] Error receiving : " << usbutils_errorToString(error) << std::endl;
            basestation_handle = nullptr;
        }
        if (actual_length == 0) continue;

        if (buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("LOG: ");
            for (int i = 1; i < actual_length; i++)
                printf("%c", buffer[i]);
        }

        if (buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS) {
//            printStatistics(buffer);
            std::cout << "[RobotHub::readBasestation] Statistics" << std::endl;
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_FEEDBACK) {
            std::cout << "[RobotHub::readBasestation] Feedback!" << std::endl;
        }
    }

    std::cout << "[readThread] Terminating" << std::endl;
}

/* Process functions */
void RobotHub::processAIcommand(proto::AICommand &AIcmd) {
    for(const proto::RobotCommand &cmd : AIcmd.commands()){
        if(settings.serialmode())
            if(usb_initialized)
                sendSerialCommand(cmd);
            else
                std::cout << "[RobotHub::processAIcommand] Warning. Trying to send serial command without initializing USB" << std::endl;
        else
            sendGrSimCommand(cmd);
    }
}

void RobotHub::processWorldState(proto::World &_world) {
    std::lock_guard<std::mutex> lock(worldLock);
    world = _world;
}

void RobotHub::processSettings(proto::Setting &_settings) {
    settings = _settings;

    grsimCommander->setGrsim_ip(settings.robothubsendip());
    grsimCommander->setGrsim_port(settings.robothubsendport());
    grsimCommander->setColor(settings.isyellow());
}

}  // namespace robothub
}  // namespace rtt




int main(int argc, char *argv[]) {

    rtt::robothub::RobotHub app;
    app.run();

//    testUsbStuff();
}