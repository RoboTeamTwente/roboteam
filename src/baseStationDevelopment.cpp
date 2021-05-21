//
// Created by emiel on 17-10-20.
// Libusb documentation : http://libusb.sourceforge.net/api-1.0/modules.html
// https://stackoverflow.com/questions/39926448/what-is-the-effective-maximum-payload-throughput-for-usb-in-full-speed
//



#include "baseStationDevelopment.h"

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <bitset>
#include <chrono>
#include <thread>

#include "RobotCommand.h"
#include "BaseTypes.h"
#include "roboteam_utils/Print.h"

#include "BasestationWriter.h"
#include "BasestationReader.h"

#include <Publisher.h>
#include <Subscriber.h>
#include "roboteam_proto/AICommand.pb.h"
#include "LibusbUtilities.h"

namespace rtt {
namespace robothub {

volatile int totalBytesSent;
volatile int totalBytesReceived;
volatile bool resetRequired = false;


void printStatistics(const uint8_t* statistics){
    for(int i = 0; i < 16; i ++){
        uint8_t sent = statistics[i*2+1];
        uint8_t rcvd = statistics[i*2+2];
        if(i != 0 && i%4 == 0) printf("\n");

        if(i < 10) printf(" ");
        printf("%d ", i);

        if(sent < 10) printf(" ");
        printf("%d ", sent);

        if(rcvd < 10) printf(" ");
        printf("%d | ", rcvd);
    }
    printf("\n\n");
}

void handleRobotCommand(proto::RobotCommand &cmd){
    std::cout << "[handleRobotCommand] Robot Command handled" << std::endl;
}

void writeThread(libusb_device_handle *handle){
    std::cout << "[writeThread]" << std::endl;

    uint8_t getStatistics = PACKET_TYPE_BASESTATION_GET_STATISTICS;

    RobotCommandPayload cmd;
    RobotCommand_set_header(&cmd, PACKET_TYPE_ROBOT_COMMAND);

    int actual_length = 0;
    int counter = 0;

    auto tsNow = std::chrono::high_resolution_clock::now();

    while(true){
        if(resetRequired) break;

        tsNow = std::chrono::high_resolution_clock::now();

        for (int id = 0; id < 16; id++) {
            RobotCommand_set_id(&cmd, id);
            int error = libusb_bulk_transfer(handle, 0x01, cmd.payload, PACKET_SIZE_ROBOT_COMMAND, &actual_length,500);
            if(error) {
                std::cout << "ERROR sending : " << usbutils_errorToString(error) << std::endl;
                resetRequired = true;
                break;
            }
            totalBytesSent += actual_length;
        }

        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - tsNow;
        int msToSleep = 17 - 1000 * elapsed.count();
        std::this_thread::sleep_for(std::chrono::milliseconds(msToSleep));

        if(++counter % 60 == 0) {
            libusb_bulk_transfer(handle, 0x01, &getStatistics, 1, &actual_length, 500);
            counter = 0;
        }
    }

    std::cout << "[writeThread] Terminating" << std::endl;
}

void readThread(libusb_device_handle *handle){
    std::cout << "[readThread]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t buffer[4906];
    int actual_length = 0;

    while(true){
        if(resetRequired) break;

        int error = libusb_bulk_transfer(handle, 0x81, buffer, 4096, &actual_length, 100);
        if (actual_length == 0) continue;
        if (error) std::cout << "ERROR receiving : " << usbutils_errorToString(error) << std::endl;

        if (buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("LOG: ");
            for (int i = 1; i < actual_length; i++)
                printf("%c", buffer[i]);
        }

        if (buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS) {
            printStatistics(buffer);
        }

        if (buffer[0] == PACKET_TYPE_ROBOT_FEEDBACK) {
            totalBytesReceived += actual_length;
        }
    }

    std::cout << "[readThread] Terminating" << std::endl;
}

bool openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle){

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
















int start() {
    std::cout << "Hello basestation!" << std::endl;

//    usbutils_enumerate();

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
    libusb_device_handle *basestation_handle = nullptr;
    if(!openBasestation(ctx, &basestation_handle)){
        std::cout << "[main] Basestation has not been found.. aborting" << std::endl;
        return -1;
    }
    // =========================== CONNECTION ESTABLISHED =========================== //


    BasestationWriter writer(basestation_handle);
    BasestationReader reader(basestation_handle);

    proto::Subscriber<proto::AICommand> *robotCommandSubscriber;
    robotCommandSubscriber = new proto::Subscriber<proto::AICommand>(
            proto::ChannelType::ROBOT_COMMANDS_PRIMARY_CHANNEL,
            &BasestationWriter::processAIcommand, &writer);

    std::thread readerThread([&]() { reader.run(); });
    std::thread writerThread([&]() { writer.run(); });

    writerThread.join();
    readerThread.join();

    exit(0);



















    // =========================== ESTABLISH CONNECTION =========================== //
//    int error;
//    // Initialize USB context
//    libusb_context *ctx;
//    error = libusb_init(&ctx);
//    if(error) std::cout << "error : " << usbutils_errorToString(error) << std::endl;
//    // Set logging level
//    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
//    if(error) std::cout << "error : " << usbutils_errorToString(error) << std::endl;
//    // Open connection
//    libusb_device_handle *basestation_handle = nullptr;
//    if(!openBasestation(ctx, &basestation_handle)){
//        std::cout << "[main] Basestation has not been found.. aborting" << std::endl;
//        return -1;
//    }
    // =========================== CONNECTION ESTABLISHED =========================== //

    std::thread tRead(readThread, basestation_handle);
    std::this_thread::sleep_for(std::chrono::seconds (1));
    std::thread tWrite(writeThread, basestation_handle);

    auto tsStart = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1000; i++){
        if(resetRequired) break;
        std::this_thread::sleep_for(std::chrono::seconds (1));
        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - tsStart;
        double seconds = elapsed.count();

        double kbpsSent = 8 * (totalBytesSent / seconds) / 1000.;
        double kbpsRcvd = 8 * (totalBytesReceived / seconds) / 1000.;
        double kbpsSentExpected = (16*60*8*PACKET_SIZE_ROBOT_COMMAND )/1000.;
        double kbpsRcvdExpected = ( 5*60*8*PACKET_SIZE_ROBOT_FEEDBACK)/1000.;

        std::cout << "    totalBytesSent " << totalBytesSent << std::endl;
        std::cout << "totalBytesReceived " << totalBytesReceived << std::endl;;
        printf("          duration %0.2f seconds\n", seconds);
        std::cout << "expected kbps sent " << kbpsSentExpected << std::endl;
        std::cout << "  actual kbps sent " << kbpsSent << std::endl;
        std::cout << "expected kbps rcvd " << kbpsRcvdExpected << std::endl;
        std::cout << "  actual kbps rcvd " << kbpsRcvd << std::endl;
        std::cout << std::endl;
    }

    tWrite.join();
    tRead.join();
    std::cout << "[main] Threads joined" << std::endl;

    libusb_release_interface(basestation_handle, 1);
    libusb_close(basestation_handle);
    libusb_exit(ctx);

    return 0;
}

} // namespace robothub
} // namespace rtt

int main(int argc, char *argv[]) {
    rtt::robothub::start();
}