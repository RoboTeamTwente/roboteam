//
// Created by emiel on 22-05-21.
//

#include "RobotHub.h"

#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>

#include "LibusbUtilities.h"
#include "BaseTypes.h"
#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "RobotStateInfo.h"
#include "Packing.h"

using namespace std::chrono;

/** @brief Callback funnction triggered when a basestation is connected */
int hotplug_callback_attach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data){
    std::cout << "[hotplug_callback_attach] Event triggered!" << std::endl;
    auto* robothub = (rtt::robothub::RobotHub*)(user_data);
    robothub->handleBasestationAttach(device);
    return 0;
}
/** @brief Callback funnction triggered when a basestation is disconnected */
int hotplug_callback_detach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data){
    std::cout << "[hotplug_callback_detach] Event triggered!" << std::endl;
    auto* robothub = (rtt::robothub::RobotHub*)(user_data);
    robothub->handleBasestationDetach(device);
    return 0;
}


namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;
    grsimCommander = std::make_shared<GRSimCommander>();
}

RobotHub::~RobotHub() {
    std::cout << "[RobotHub::~RobotHub] Destructor" << std::endl;
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

    /* set up the event listeners that respond to the (dis)connecting of a basestation */
    if (!startBasestation()) {
        std::cout << "[RobotHub::run] Error while initializing basestation. Aborting..." << std::endl;
        return;
    }

    /* Subscribe to relevant topics */
    subscribe();

    /* Start basestation reading thread */
    std::thread readThread(&RobotHub::readBasestation, this);

    timeval usb_event_timeout{.tv_usec=100000}; // 100ms timeout for USB events
    steady_clock::time_point tNextTick = steady_clock::now();

    std::cout << "[RobotHub::run] Loop " << std::endl;
    while(running){
        std::this_thread::sleep_for(milliseconds(10));
        // Handle any possible USB events such as the attaching / detaching of a basestation
        libusb_handle_events_timeout(ctx, &usb_event_timeout);
        // Print statistics every second
        if(tNextTick + seconds(1) < steady_clock::now()){
            printStatistics();
            tNextTick += seconds(1);
        }
    }
}

void RobotHub::printStatistics() {
    std::stringstream ss;

    const int amountOfColumns = 4;
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i += amountOfColumns) {
        for (int j = 0; j < amountOfColumns; j++) {
            const int robotId = i + j;
            if (robotId < MAX_AMOUNT_OF_ROBOTS) {
                int nSent = commands_sent[robotId];
                int nReceived = feedback_received[robotId];
                commands_sent[robotId] = 0;
                feedback_received[robotId] = 0;

                if(robotId < 10) ss << " "; ss << robotId;
                ss << "(";
                if(nSent     < 100) ss << " "; if(nSent     < 10) ss << " "; ss << nSent;
                ss << ":";
                if(nReceived < 100) ss << " "; if(nReceived < 10) ss << " "; ss << nReceived;
                ss << ") | ";
            }
        }
        ss << std::endl;
    }

    std::cout << ss.str();
}



/* =========================================== USB FUNCTIONS =========================================== */
/** @brief Callback function that is triggered when a basestation is connected to the PC
 *
 * When a basestation is connected to the PC, this function claims that basestation. It first detaches the kernel
 * drivers, and then claims the correct interface. It stores the handle to the basestation in the basestation_handle
 * variable. The reading-thread automatically starts reading from the basestation.
 */
void RobotHub::handleBasestationAttach(libusb_device* device){
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
        basestation_handle = nullptr;
        return;
    }

    std::cout << "[RobotHub::handleBasestationAttach] Basestation opened" << std::endl;
}



/** @brief Callback function that is triggered when a basestation is disconnected from the PC
 *
 * When a basestation is disconnected from the PC, the corresponding handle is reset. This causes the reading-thread to
 * sleep instead of hogging the CPU with nonstop device errors.
 */
void RobotHub::handleBasestationDetach(libusb_device* device){
    std::cout << "[RobotHub::handleBasestationDetach] " << std::endl;
    libusb_release_interface(basestation_handle, 1);
    libusb_close(basestation_handle);
    basestation_handle = nullptr;
}



/** @brief sets up the event listeners that respond to the (dis)connecting of a basestation.
 *
 * This function sets up event listeners that respond to the (dis)connecting of a basestation. These listeners trigger
 * functions that automatically claim and release any basestation. Event handlers are preferred over manually
 * enumerating all USB devices to search for a basestation, since manual enumeration does not work nicely when
 * attaching / detaching a basestation. The event listener responsible for listening to the attachment of a basestation
 * immediately triggers once as initialization if a basestation is present. This means that the order of connecting a
 * basestation and running RobotHub is not important. The listeners know that a basestation (dis)connected based on
 * its vendor-id (0x0483) and product-id (0x5740).
 *
 * @return true if everything went smoothly, false otherwise
 */
bool RobotHub::startBasestation(){
    int error;

    /* Initialize USB context */
    error = libusb_init(&ctx);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_init : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Set logging level */
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_set_option : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to detach event */
    error = libusb_hotplug_register_callback(
            ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS,
            BASESTATION_VENDOR_ID, BASESTATION_PRODUCT_ID, LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback_detach, (void*)this, &callback_handle_detach);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback detach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to attach event */
    /* pass LIBUSB_HOTPLUG_ENUMERATE to immediately attach a basestation if already plugged into the PC */
    error = libusb_hotplug_register_callback(
            ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE,
            BASESTATION_VENDOR_ID, BASESTATION_PRODUCT_ID, LIBUSB_HOTPLUG_MATCH_ANY,
            hotplug_callback_attach, (void*) this, &callback_handle_attach);
    if(error){
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback attach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    return true;
}


/** @brief Finds and opens a basestation device by looping over all connected USB devices. Unused.
 * DEPRECATED in favor of event based methods
 *
 * This function loops over all connected USB devices and searches for a basestation by looking for the correct
 * vendor-id (0x0483) and product-id (0x5740). If a basestation is found, it is opened and the handle to it is stored
 * in the 'basestation_handle' parameter. The function first detaches the kernel drivers from the basestation, then
 * claims the basestation for itself. Currently this function is not used, in favor of the event handling method, where
 * a callback is triggered when a USB device is (dis)connected from the PC. See RobotHub::startBasestation. This
 * function is kept as reference, or as backup in case the event handling method doesn't work as intended.
 *
 * @param ctx Libusb_context
 * @param basestation_handle Variable in which the basestation handle can be placed, if a basestation is found
 * @return true if a basestation is opened, false otherwise
 */
bool RobotHub::openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle){

    libusb_device* basestation_device = nullptr;
    libusb_device **device_list;
    int num_devices = libusb_get_device_list(ctx, &device_list);

    int error;
    for (int i = 0; i < num_devices; i++) {
        libusb_device *device = device_list[i];
        libusb_device_descriptor descriptor{};

        error = libusb_get_device_descriptor(device, &descriptor);
        if (error) std::cout << "[openBasestation] Error : " << usbutils_errorToString(error) << std::endl;
        if (descriptor.idVendor == 0x0483 && descriptor.idProduct == 0x5740) {
            basestation_device = device;

            printf("[openBasestation] Basestation found. %04x:%04x (bus %d, device %d) %s\n",
                   descriptor.idVendor, descriptor.idProduct,
                   libusb_get_bus_number(device), libusb_get_device_address(device),
                   usbutils_speedToString(libusb_get_device_speed(device)).c_str());
        }
    }

    libusb_free_device_list(device_list, true);

    if(basestation_device == nullptr) {
        std::cout << "[findBasestation] Basestation not found" << std::endl;
        return false;
    }

    error = libusb_open(basestation_device, basestation_handle);
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




/* =========================================== READING/WRITING FUNCTIONS =========================================== */
/** @brief Sends a proto::RobotCommand to the basestation
 *
 * This function takes a proto::RobotCommand that needs to be sent to the basestation. The command is first converted
 * into a RobotCommandPayload (a packet command, using the minimum amoumt of bytes), and additional information is
 * (possibly) added to it, such as the angle of the robot in the world. If there is no connection to a basestation, the
 * packet is simply dropped.
 *
 * @param cmd Reference to the proto::RobotCommand that needs to be sent to the basestation
 */
void RobotHub::sendSerialCommand(const proto::RobotCommand &cmd) {
    // Check if a connection to a basestation exists
    if(basestation_handle == nullptr){
        std::cout << "[RobotHub::sendSerialCommand] Basestation not present!" << std::endl;
        std::this_thread::sleep_for(milliseconds (300));
        return;
    }
    // Convert the proto::RobotCommand to a RobotCommandPayload
    RobotCommandPayload payload = createEmbeddedCommand(cmd, world, false);
    int bytesSent; // Holds the value of actual bytes sent to the basestation after transfer is complete
    int error = libusb_bulk_transfer(basestation_handle, 0x01, payload.payload, PACKET_SIZE_ROBOT_COMMAND, &bytesSent, 500);

    if(error){
        std::cout << "[RobotHub::sendSerialCommand] Error while sending to basestation. Resetting connection .." << std::endl;
        std::cout << "[RobotHub::sendSerialCommand] Error : " << usbutils_errorToString(error) << std::endl;
    }

}


void RobotHub::sendGrSimCommand(const proto::RobotCommand &cmd) {
    this->grsimCommander->queueGRSimCommand(cmd);
}



/** @brief Receives and handles any packets coming from a basestation
 *
 * This function should be run in a separate thread, since it uses blocking USB reads in a while-loop. Any log messages
 * are printed to the terminal. Any robot feedback is forwarded to the rest of the system via the feedback channel.
 */
void RobotHub::readBasestation(){
    std::cout << "[RobotHub::readBasestation] Running" << std::endl;
    std::this_thread::sleep_for(milliseconds(1000));
    uint8_t buffer[4906];
    int bytes_received = 0;

    while(read_running){

        /* Check if the basestation is connected. Sleep if not */
        if(basestation_handle == nullptr){
            std::this_thread::sleep_for(milliseconds(1000));
            continue;
        }

        /* Read bytes from the basestation. At most 4096 bytes */
        int error = libusb_bulk_transfer(basestation_handle, 0x81, buffer, 4096, &bytes_received, 100);
        if(error != LIBUSB_SUCCESS and error != LIBUSB_ERROR_TIMEOUT){
            std::cout << "[BasestationReader::run] Error receiving : " << usbutils_errorToString(error) << std::endl;
            std::this_thread::sleep_for(milliseconds(1000));
            continue;
        }
        // If a timeout occurs, the number of bytes received is 0. In that case, continue.
        if (bytes_received == 0) continue;

        if (buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("[Basestation] ");
            for (int i = 1; i < bytes_received; i++)
                printf("%c", buffer[i]);
        }

        if (buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS) {
            std::cout << "[RobotHub::readBasestation] Statistics" << std::endl;
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_FEEDBACK) {
            feedback_received[RobotFeedback_get_id((RobotFeedbackPayload*)buffer)]++;
        }

        if(buffer[0] == PACKET_TYPE_ROBOT_STATE_INFO) {
            RobotStateInfo rsi;
            decodeRobotStateInfo(&rsi, (RobotStateInfoPayload*)buffer);

        }
    }

    std::cout << "[RobotHub::readBasestation] Terminating" << std::endl;
}



/* =========================================== PACKET CALLBACK FUNCTIONS =========================================== */
void RobotHub::processAIcommand(proto::AICommand &AIcmd) {
    for(const proto::RobotCommand &cmd : AIcmd.commands()){
        if(settings.serialmode())
            sendSerialCommand(cmd);
        else
            sendGrSimCommand(cmd);
        commands_sent[cmd.id()]++;
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

}