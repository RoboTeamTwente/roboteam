#include <basestation/BasestationManager.hpp>
#include <basestation/LibusbUtilities.h>
#include <constants.h>
#include <iostream>
#include <RobotStateInfo.h>

namespace rtt::robothub::basestation {

BasestationManager::BasestationManager() {
    std::cout << "Hello basestation!" << std::endl;

    /* set up the event listeners that respond to the (dis)connecting of a basestation */
    if (!this->setupUsbEventListeners())
        throw FailedToSetupUsbEventListenerException("Failed to set up libusb event listeners");
    
    /* Start basestation reading thread */
    this->listenThread = std::thread(&BasestationManager::listenToBasestation, this);
    this->runThread = std::thread(&BasestationManager::runManager, this);
}

BasestationManager::~BasestationManager() {
    // Stop threads
    this->shouldStopListening = true;
    this->shouldStopRunning = true;

    if (this->listenThread.joinable())
        this->listenThread.join();
    
    if (this->runThread.joinable())
        this->runThread.join();
}

/** @brief Sends a proto::RobotCommand to the basestation
 *
 * This function takes a proto::RobotCommand that needs to be sent to the basestation. The command is first converted
 * into a RobotCommandPayload (a packet command, using the minimum amoumt of bytes), and additional information is
 * (possibly) added to it, such as the angle of the robot in the world. If there is no connection to a basestation, the
 * packet is simply dropped.
 * @param payload Reference to the proto::RobotCommand that needs to be sent to the basestation
 * @return whether or not the command successfully was sent to the basestation
 */
bool BasestationManager::sendSerialCommand(RobotCommandPayload &payload) const {
    // Check if a connection to a basestation exists
    if (basestation_handle == nullptr) {
        std::cout << "[RobotHub::sendSerialCommand] Basestation not present!" << std::endl;
        // TODO check if sleeping here is a good idea. Will it block ZMQ? Will this fill up some queue somewhere when packets keep coming in?
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_NO_BASESTATION_MS));
        return false;
    }
    payload.payload;
    int bytesSent;  // Holds the value of actual bytes sent to the basestation after transfer is complete
    int error = libusb_bulk_transfer(basestation_handle, 0x01, payload.payload, PACKET_SIZE_ROBOT_COMMAND, &bytesSent, 500);

    if (error) {
        // TODO: Where is the resetting of a connection?
        std::cout << "[RobotHub::sendSerialCommand] Error while sending to basestation. Resetting connection .." << std::endl;
        std::cout << "[RobotHub::sendSerialCommand] Error : " << usbutils_errorToString(error) << std::endl;
    }
    return error;
}
/** @brief Sets the callback function
 *  The given function is the function that will be called everytime this basestation manager
 *  receives feedback. If the callback function is not set, no feedback will be sent to AI.
 *  @param callback The function that will be called whenever feedback is received
*/
void BasestationManager::setFeedbackCallback(const std::function<void(RobotFeedback&)> &callback) {
    this->feedbackCallbackFunction = callback;
}

/** @brief Runs the manager
 *  This function will keep checking for new libusb events and handle them.
 */
void BasestationManager::runManager() const {
    timeval usb_event_timeout{.tv_usec = 100000};  // 100ms timeout for USB events
    
    // TODO: Should make use of libusb_wait_for_event() instead of sleep_for
    while (!this->shouldStopRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Handle any possible USB events such as the attaching / detaching of a basestation
        libusb_handle_events_timeout(ctx, &usb_event_timeout);
    }
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
bool BasestationManager::setupUsbEventListeners() {
    int error;

    /* Initialize USB context */
    error = libusb_init(&ctx);
    if (error) {
        std::cout << "[RobotHub::startBasestation] Error on libusb_init : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Set logging level */
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if (error) {
        std::cout << "[RobotHub::startBasestation] Error on libusb_set_option : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to detach event */
    error = libusb_hotplug_register_callback(ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS, BASESTATION_VENDOR_ID, BASESTATION_PRODUCT_ID,
                                             LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback_detach, (void *)this, &callback_handle_detach);
    if (error) {
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback detach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    /* Subscribe to attach event */
    /* pass LIBUSB_HOTPLUG_ENUMERATE to immediately attach a basestation if already plugged into the PC */
    error = libusb_hotplug_register_callback(ctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED, LIBUSB_HOTPLUG_ENUMERATE, BASESTATION_VENDOR_ID, BASESTATION_PRODUCT_ID,
                                             LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback_attach, (void *)this, &callback_handle_attach);
    if (error) {
        std::cout << "[RobotHub::startBasestation] Error on libusb_hotplug_register_callback attach : " << usbutils_errorToString(error) << std::endl;
        return false;
    }

    return true;
}

/** @brief Callback function that is triggered when a basestation is connected to the PC
 *
 * When a basestation is connected to the PC, this function claims that basestation. It first detaches the kernel
 * drivers, and then claims the correct interface. It stores the handle to the basestation in the basestation_handle
 * variable. The reading-thread automatically starts reading from the basestation.
 */
void BasestationManager::handleBasestationAttach(libusb_device *device) {
    basestation_device = device;

    int error;
    error = libusb_open(basestation_device, &basestation_handle);
    if (error) {
        std::cout << "[RobotHub::handleBasestationAttach] Error while trying to open handle : " << usbutils_errorToString(error) << std::endl;
        return;
    }

    /** libusb_set_auto_detach_kernel_driver() Enable/disable libusb's automatic kernel driver detachment. When this is
     * enabled libusb will automatically detach the kernel driver on an interface when claiming the interface, and
     * attach it when releasing the interface.
     */
    error = libusb_set_auto_detach_kernel_driver(basestation_handle, 1);
    if (error) {
        std::cout << "[RobotHub::handleBasestationAttach] Error while enabling auto detach : " << usbutils_errorToString(error) << std::endl;
        return;
    }
    /** libusb_claim_interface() Claim an interface on a given device handle. You must claim the interface you wish to
     * use before you can perform I/O on any of its endpoints.
     */
    error = libusb_claim_interface(basestation_handle, 1);
    if (error) {
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
void BasestationManager::handleBasestationDetach(libusb_device *device) {
    std::cout << "[RobotHub::handleBasestationDetach] " << std::endl;
    libusb_release_interface(basestation_handle, 1);
    libusb_close(basestation_handle);
    basestation_handle = nullptr;
}

/** @brief Receives and handles any packets coming from a basestation
 *
 * This function should be run in a separate thread, since it uses blocking USB reads in a while-loop. Any log messages
 * are printed to the terminal. Any robot feedback is forwarded to the rest of the system via the feedback channel.
 */
void BasestationManager::listenToBasestation() const {
    std::cout << "[RobotHub::readBasestation] Running" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_NO_BASESTATION_MS));
    uint8_t buffer[USB_BUFFER_SIZE_RECEIVE];
    int bytes_received = 0;

    while (!this->shouldStopListening) {
        /* Check if the basestation is connected. Sleep if not */
        if (basestation_handle == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_NO_BASESTATION_MS));
            continue;
        }

        /* Read bytes from the basestation. At most USB_BUFFER_SIZE_RECEIVE bytes */
        int error = libusb_bulk_transfer(basestation_handle, 0x81, buffer, USB_BUFFER_SIZE_RECEIVE, &bytes_received, THREAD_READ_TIMEOUT_MS);
        if (error != LIBUSB_SUCCESS and error != LIBUSB_ERROR_TIMEOUT) {
            std::cout << "[BasestationReader::run] Error receiving : " << usbutils_errorToString(error) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_NO_BASESTATION_MS));
            continue;
        }
        // If a timeout occurs, the number of bytes received is 0. In that case, continue.
        if (bytes_received == 0) continue;

        if (buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("[Basestation] ");
            for (int i = 1; i < bytes_received; i++) printf("%c", buffer[i]);
        }

        if (buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS) {
            std::cout << "[RobotHub::readBasestation] Statistics" << std::endl;
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_FEEDBACK) {
            RobotFeedback feedback;
            decodeRobotFeedback(&feedback, (RobotFeedbackPayload *)buffer);

            // TODO Make autogenerated
            //            proto::RobotFeedback protoFeedback;
            //            protoFeedback.set_id(feedback.id);
            //            protoFeedback.set_hasball(feedback.hasBall);
            //            protoFeedback.set_x_vel(feedback.rho);
            //
            //            feedbackPublisher->send(protoFeedback);
            //            std::cout << "feedback sent!" << std::endl;
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_STATE_INFO) {
            RobotStateInfo rsi;
            decodeRobotStateInfo(&rsi, (RobotStateInfoPayload *)buffer);
        }
    }

    std::cout << "[RobotHub::readBasestation] Terminating" << std::endl;
}

/** @brief Callback funnction triggered when a basestation is connected */
int hotplug_callback_attach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
    std::cout << "[hotplug_callback_attach] Event triggered!" << std::endl;
    auto *basestationManager = (rtt::robothub::basestation::BasestationManager *)(user_data);
    basestationManager->handleBasestationAttach(device);
    return 0;
}
/** @brief Callback funnction triggered when a basestation is disconnected */
int hotplug_callback_detach(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
    std::cout << "[hotplug_callback_detach] Event triggered!" << std::endl;
    auto *basestationManager = (rtt::robothub::basestation::BasestationManager *)(user_data);
    basestationManager->handleBasestationDetach(device);
    return 0;
}

FailedToSetupUsbEventListenerException::FailedToSetupUsbEventListenerException(const std::string message) {
    this->message = message;
}
const char* FailedToSetupUsbEventListenerException::what() const noexcept {
    return this->message.c_str();
}

} // namespace rtt::robothub::basestation
