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

volatile int totalBytesSent;
volatile int totalBytesReceived;

std::string bmAttributes_TransferTypeToString(uint8_t bmAttributes){
    uint8_t transferType = bmAttributes & 0b00000011;
    switch(transferType){
        case 0: return "Control";
        case 1: return "Isochronous";
        case 2: return "Bulk";
        case 3: return "Interrupt";
        default: return "Unknown transfer type " + std::to_string(transferType);
    }
}
std::string bmAttributes_SyncTypeToString(uint8_t bmAttributes){
    if((bmAttributes & 0b11) != 1)  return "Undefined when mode is not Isochronous";

    uint8_t syncType = (bmAttributes >> 2) & 0b11;
    switch(syncType){
        case 0: return "No Synchonisation";
        case 1: return "Asynchronous";
        case 3: return "Adaptive";
        case 4: return "Synchronous";
        default: return "Unknown synchronization type " + std::to_string(syncType);
    }
}
std::string bmAttributes_usageTypeToString(uint8_t bmAttributes){
    if((bmAttributes & 0b11) != 1)  return "Undefined when mode is not Isochronous";

    uint8_t usageType = (bmAttributes >> 4) & 0b11;
    switch(usageType){
        case 0: return "Data Endpoint";
        case 1: return "Feedback Endpoint";
        case 3: return "Explicit Feedback Data Endpoint";
        case 4: return "Reserved";
        default: return "Unknown usage type " + std::to_string(usageType);
    }
}

std::string bEndpointAddress_EndpointNumberToString(uint8_t bEndpointAddress){
    return std::to_string(bEndpointAddress & 0b111);
}
std::string bEndpointAddress_EndpointDirectionToString(uint8_t bEndpointAddress){
    uint8_t direction = bEndpointAddress >> 7;
    switch (direction) {
        case 0 : return "LIBUSB_ENDPOINT_IN (device-to-host)";
        case 1 : return "LIBUSB_ENDPOINT_OUT (host-to-device)";
        default: return "Unknown direction " + std::to_string(direction);
    }
}

/* Enums to strings */
std::string descriptorTypeToString(int bDescriptorType){
    switch(bDescriptorType){
        case 1: return "LIBUSB_DT_DEVICE";
        case 2: return "LIBUSB_DT_CONFIG";
        case 3: return "LIBUSB_DT_STRING";
        case 4: return "LIBUSB_DT_INTERFACE";
        case 5: return "LIBUSB_DT_ENDPOINT";
        case 15: return "LIBUSB_DT_BOS";
        case 16: return "LIBUSB_DT_DEVICE_CAPABILITY";
        case 33: return "LIBUSB_DT_HID";
        case 34: return "LIBUSB_DT_REPORT";
        case 35: return "LIBUSB_DT_PHYSICAL";
        case 41: return "LIBUSB_DT_HUB";
        case 42: return "LIBUSB_DT_SUPERSPEED_HUB";
        case 48: return "LIBUSB_DT_SS_ENDPOINT_COMPANION";
        default: return "Unknown bDescriptorType " + std::to_string(bDescriptorType);
    }
}
std::string classToString(int bDeviceClass){
    switch(bDeviceClass){
        case 0: return "LIBUSB_CLASS_PER_INTERFACE";
        case 1: return "LIBUSB_CLASS_AUDIO";
        case 2: return "LIBUSB_CLASS_COMM";
        case 3: return "LIBUSB_CLASS_HID";
        case 5: return "LIBUSB_CLASS_PHYSICAL";
        case 7: return "LIBUSB_CLASS_PRINTER";
        case 6: return "LIBUSB_CLASS_PTP";
        case 8: return "LIBUSB_CLASS_MASS_STORAGE";
        case 9: return "LIBUSB_CLASS_HUB";
        case 10: return "LIBUSB_CLASS_DATA";
        case 11: return "LIBUSB_CLASS_SMART_CARD";
        case 13: return "LIBUSB_CLASS_CONTENT_SECURITY";
        case 14: return "LIBUSB_CLASS_VIDEO";
        case 15: return "LIBUSB_CLASS_PERSONAL_HEALTHCARE";
        case 220: return "LIBUSB_CLASS_DIAGNOSTIC_DEVICE";
        case 224: return "LIBUSB_CLASS_WIRELESS";
        case 254: return "LIBUSB_CLASS_APPLICATION";
        case 255: return "LIBUSB_CLASS_VENDOR_SPEC";
        default: return "Unknown class " + std::to_string(bDeviceClass);
    }
}
std::string errorToString(int error){
    switch(error){
        case 0:   return "LIBUSB_SUCCESS";
        case -1:  return "LIBUSB_ERROR_IO";
        case -2:  return "LIBUSB_ERROR_INVALID_PARAM";
        case -3:  return "LIBUSB_ERROR_ACCESS";
        case -4:  return "LIBUSB_ERROR_NO_DEVICE";
        case -5:  return "LIBUSB_ERROR_NOT_FOUND";
        case -6:  return "LIBUSB_ERROR_BUSY";
        case -7:  return "LIBUSB_ERROR_TIMEOUT";
        case -8:  return "LIBUSB_ERROR_OVERFLOW";
        case -9:  return "LIBUSB_ERROR_PIPE";
        case -10: return "LIBUSB_ERROR_INTERRUPTED";
        case -11: return "LIBUSB_ERROR_NO_MEM";
        case -12: return "LIBUSB_ERROR_NOT_SUPPORTED";
        case -99: return "LIBUSB_ERROR_OTHER";
        default: return "Unknown error code " + std::to_string(error);
    }
}
std::string speedToString(int speed){
    switch(speed) {
        case 0: return "LIBUSB_SPEED_UNKNOWN";
        case 1: return "LIBUSB_SPEED_LOW";
        case 2: return "LIBUSB_SPEED_FULL";
        case 3: return "LIBUSB_SPEED_HIGH";
        case 4: return "LIBUSB_SPEED_SUPER";
        case 5: return "LIBUSB_SPEED_SUPER_PLUS";
        default: return "Unknown speed " + std::to_string(speed);
    }
}

void enumerate(){
    libusb_context *ctx;
    int error;

    error = libusb_init(&ctx);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;

    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;

    libusb_device **list;
    int count = libusb_get_device_list(ctx, &list);
    std::cout << "Number of devices found : " << count << std::endl;

    unsigned char str[256];
    for (int i = 0; i < count; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor desc{};

        error = libusb_get_device_descriptor(device, &desc);
        if(error) std::cout << "error : " << errorToString(error) << std::endl;
            if (desc.idVendor != 0x0483 || desc.idProduct != 0x5740)
            continue;

        int deviceBus = libusb_get_bus_number(device);
        int deviceAddress = libusb_get_device_address(device);
        int deviceSpeed = libusb_get_device_speed(device);

        printf("\n%04x:%04x (bus %d, device %d) %s\n", desc.idVendor, desc.idProduct, deviceBus, deviceAddress, speedToString(deviceSpeed).c_str());
        std::cout
        << "Class: " << classToString(desc.bDeviceClass) << " (" << std::to_string(desc.bDeviceClass) << "), "
        << "SubClass: " << std::to_string(desc.bDeviceSubClass) << ", "
        << "Protocol: " << std::to_string(desc.bDeviceProtocol) << std::endl;

        libusb_device_handle* handle;
        error = libusb_open(device, &handle);
        if(error){
            std::cout << "Error while trying to open handle : " << errorToString(error) << std::endl;
            continue;
        }

        error = libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, str, sizeof(str));
        std::string manufacturer((char*)str);
        error = libusb_get_string_descriptor_ascii(handle, desc.iProduct, str, sizeof(str));
        std::string product((char*)str);
        error = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, str, sizeof(str));
        std::string serialNumber((char*)str);
        std::cout << "Device : " << manufacturer << " " << product << " " << serialNumber << std::endl;

        /** http://libusb.sourceforge.net/api-1.0/structlibusb__config__descriptor.html */
        for(int iConfig = 0; iConfig < desc.bNumConfigurations; iConfig++){
            libusb_config_descriptor* configDesc;
            error = libusb_get_config_descriptor(device, iConfig, &configDesc);
            if(error) std::cout << "Error while retrieving config " << iConfig << " : " << errorToString(error) << std::endl;
            if(error) continue;

            std::cout << "CONFIGURATION " << iConfig << std::endl;
            // bNumInterfaces : Number of interfaces supported by this configuration.
            std::cout << "│   bNumInterfaces : " << std::to_string(configDesc->bNumInterfaces) << std::endl;
            // bConfigurationValue : Identifier value for this configuration.
            std::cout << "│   bConfigurationValue : " << std::to_string(configDesc->bConfigurationValue) << std::endl;
            error = libusb_get_string_descriptor_ascii(handle, configDesc->iConfiguration, str, sizeof(str));
            // iConfiguration : Index of string descriptor describing this configuration.
            if(error < 0) std::cout << "│   Error on iConfiguration : " << errorToString(error) << std::endl;
            else std::cout << "│   Configuration : " << str << std::endl;
            // 	MaxPower : Maximum power consumption of the USB device from this bus in this configuration when the device is fully operation.
            //  Expressed in units of 2 mA when the device is operating in high-speed mode and in units of 8 mA when the device is operating in super-speed mode.
            std::bitset<8> bmAttributes(configDesc->bmAttributes);
            std::cout << "|   bmAttributes : " << bmAttributes << std::endl;
            std::cout << "│   MaxPower : " << std::to_string(configDesc->MaxPower) << std::endl;
            // Length of the extra descriptors, in bytes.
            std::cout << "│   extra_length : " << std::to_string(configDesc->extra_length) << std::endl;

            for(int iInterface = 0; iInterface < configDesc->bNumInterfaces; iInterface++){
                libusb_interface interface = configDesc->interface[iInterface];
                std::cout << "├───Interface " << iInterface << " with " << interface.num_altsetting << " alternative settings " << std::endl;

                /** http://libusb.sourceforge.net/api-1.0/structlibusb__interface__descriptor.html */
                for(int iInterfaceDesc = 0; iInterfaceDesc < interface.num_altsetting; iInterfaceDesc++){
                    libusb_interface_descriptor interfaceDesc = interface.altsetting[iInterfaceDesc];
                    std::cout << "│   ├──INTERFACE SETTING " << std::endl;
                    // bInterfaceNumber : Number of this interface.
                    std::cout << "│   │   bInterfaceNumber : " << std::to_string(interfaceDesc.bInterfaceNumber) << std::endl;
                    // bAlternateSetting : 	Value used to select this alternate setting for this interface.
                    std::cout << "│   │   bAlternateSetting : " << std::to_string(interfaceDesc.bAlternateSetting) << std::endl;
                    // bNumEndpoints : Number of endpoints used by this interface (excluding the control endpoint).
                    std::cout << "│   │   bNumEndpoints : " << std::to_string(interfaceDesc.bNumEndpoints) << std::endl;
                    // bInterfaceClass : USB-IF class code for this interface.
                    std::cout << "│   │   bInterfaceClass : " << classToString(interfaceDesc.bInterfaceClass) << std::endl;
                    std::cout << "│   │   bInterfaceSubClass : " << std::to_string(interfaceDesc.bInterfaceSubClass) << std::endl;
                    std::cout << "│   │   bInterfaceProtocol : " << std::to_string(interfaceDesc.bInterfaceProtocol) << std::endl;
                    std::cout << "│   │   extra_length: " << std::to_string(interfaceDesc.extra_length) << std::endl;

                    /** http://libusb.sourceforge.net/api-1.0/structlibusb__endpoint__descriptor.html */
                    for(int iEndpoint = 0; iEndpoint < interfaceDesc.bNumEndpoints; iEndpoint++){
                        libusb_endpoint_descriptor endpointDesc = interfaceDesc.endpoint[iEndpoint];
                        std::cout << "│   │   ├──ENDPOINT " << std::to_string(iEndpoint) << std::endl;
                        // bEndPointAddress : The address of the endpoint described by this descriptor.
                        std::bitset<8> bmAttributes(endpointDesc.bEndpointAddress);
                        std::cout << "|   │   │   bEndpointAddress : " << bmAttributes << std::endl;
                        std::cout << "│   │   │   endpoint number : " << bEndpointAddress_EndpointNumberToString(endpointDesc.bEndpointAddress).c_str() << std::endl;
                        std::cout << "│   │   │   endpoint direction : " << bEndpointAddress_EndpointDirectionToString(endpointDesc.bEndpointAddress).c_str() << std::endl;
                        // bmAttributes : Attributes which apply to the endpoint when it is configured using the bConfigurationValue.
                        std::cout << "│   │   │   transfer type : " << bmAttributes_TransferTypeToString(endpointDesc.bmAttributes).c_str() << std::endl;
                        std::cout << "│   │   │   synchronization type : " << bmAttributes_SyncTypeToString(endpointDesc.bmAttributes).c_str() << std::endl;
                        std::cout << "│   │   │   usage type : " << bmAttributes_usageTypeToString(endpointDesc.bmAttributes).c_str() << std::endl;
                        // wMaxPacketSize : Maximum packet size this endpoint is capable of sending/receiving.
                        std::cout << "│   │   │   wMaxPacketSize : " << std::to_string(endpointDesc.wMaxPacketSize) << std::endl;
                        // bInterval : Interval for polling endpoint for data transfers.
                        std::cout << "│   │   │   bInterval : " << std::to_string(endpointDesc.bInterval) << std::endl;
                        // bRefresh : For audio devices only: the rate at which synchronization feedback is provided.
                        std::cout << "│   │   │   bRefresh: " << std::to_string(endpointDesc.bRefresh) << std::endl;
                        // bSynchAddress : For audio devices only: the address if the synch endpoint.
                        std::cout << "│   │   │   bSynchAddress : " << std::to_string(endpointDesc.bSynchAddress) << std::endl;
                        // 	extra_length : Length of the extra descriptors, in bytes.
                        std::cout << "│   │   │   extra_length : " << std::to_string(endpointDesc.extra_length) << std::endl;
                    }
                }
            }
        }


        // todo claim interface

        libusb_close(handle);
    }

    libusb_free_device_list(list, true);
    libusb_exit(ctx);
}

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

void writeThread(libusb_device_handle *handle){
    std::cout << "[writeThread]" << std::endl;

    uint8_t getStatistics = PACKET_TYPE_BASESTATION_GET_STATISTICS;

    RobotCommandPayload cmd;
    RobotCommand_set_header(&cmd, PACKET_TYPE_ROBOT_COMMAND);

    int actual_length = 0;

    auto tsNow = std::chrono::high_resolution_clock::now();

    while(true){
        for(int counter = 0; counter < 60; counter++) {

           tsNow = std::chrono::high_resolution_clock::now();

            for (int id = 0; id < 16; id++) {
                RobotCommand_set_id(&cmd, id);
                int error = libusb_bulk_transfer(handle, 0x01, cmd.payload, PACKET_SIZE_ROBOT_COMMAND, &actual_length,500);
                if (error) std::cout << "ERROR sending : " << errorToString(error) << std::endl;
                totalBytesSent += actual_length;
            }

            std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - tsNow;
            int msToSleep = 17 - 1000 * elapsed.count();
            std::this_thread::sleep_for(std::chrono::milliseconds(msToSleep));

        }

        libusb_bulk_transfer(handle, 0x01, &getStatistics, 1, &actual_length, 500);
    }
}

void readThread(libusb_device_handle *handle){
    std::cout << "[readThread]" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t buffer[4906];
    int actual_length = 0;

    while(true){
        int error = libusb_bulk_transfer(handle, 0x81, buffer, 4096, &actual_length, 100);
        if (actual_length == 0) continue;
        if (error) std::cout << "ERROR receiving : " << errorToString(error) << std::endl;

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

    std::cout << "[readThread] total bytes received : " << totalBytesReceived << std::endl;
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
        if (error) std::cout << "[findBasestation] Error : " << errorToString(error) << std::endl;
        if (desc.idVendor == 0x0483 && desc.idProduct == 0x5740) {
            basestation_dev = device;
            int deviceBus = libusb_get_bus_number(device);
            int deviceAddress = libusb_get_device_address(device);
            int deviceSpeed = libusb_get_device_speed(device);
            printf("[findBasestation] Basestation found. %04x:%04x (bus %d, device %d) %s\n",
                    desc.idVendor, desc.idProduct, deviceBus, deviceAddress, speedToString(deviceSpeed).c_str());
        }
    }

    if(basestation_dev == nullptr) {
        std::cout << "[findBasestation] Basestation not found" << std::endl;
        libusb_free_device_list(list, true);
        return false;
    }

    error = libusb_open(basestation_dev, basestation_handle);
    if(error){
        std::cout << "Error while trying to open handle : " << errorToString(error) << std::endl;
        return false;
    }

    /** libusb_set_auto_detach_kernel_driver() Enable/disable libusb's automatic kernel driver detachment. When this is
     * enabled libusb will automatically detach the kernel driver on an interface when claiming the interface, and
     * attach it when releasing the interface.
     */
    error = libusb_set_auto_detach_kernel_driver(*basestation_handle, 1);
    if(error){
        std::cout << "Error while enabling auto detach : " << errorToString(error) << std::endl;
        return false;
    }
    /** libusb_claim_interface() Claim an interface on a given device handle. You must claim the interface you wish to
     * use before you can perform I/O on any of its endpoints.
     */
    error = libusb_claim_interface(*basestation_handle, 1);
    if(error){
        std::cout << "Error while claiming interface : " << errorToString(error) << std::endl;
        return false;
    }

    return true;
}

int main(int argc, char *argv[]) {
    std::cout << "Hello basestation!" << std::endl;


    // =========================== ESTABLISH CONNECTION =========================== //

    int error;
    // Initialize USB context
    libusb_context *ctx;
    error = libusb_init(&ctx);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;
    // Set logging level
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;
    // Open connection
    libusb_device_handle *basestation_handle = nullptr;
    if(!openBasestation(ctx, &basestation_handle)){
        std::cout << "[main] Basestation has not been found.. aborting" << std::endl;
        return -1;
    }

    // =========================== CONNECTION ESTABLISHED =========================== //

    std::thread tRead(readThread, basestation_handle);
    std::this_thread::sleep_for(std::chrono::seconds (1));
    std::thread tWrite(writeThread, basestation_handle);

    auto tsStart = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1000; i++){
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
