//
// Created by emiel on 17-10-20.
// Libusb documentation : http://libusb.sourceforge.net/api-1.0/modules.html
//

#include "baseStationDevelopment.h"

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <bitset>
#include <chrono>
#include <thread>

#include "RobotCommand.h"
#include "BaseTypes.h"

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

int main(int argc, char *argv[]) {
    std::cout << "Hello basestation!" << std::endl;

    enumerate();

    int error;
    // Initialize USB context
    libusb_context *ctx;
    error = libusb_init(&ctx);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;
    // Set logging level
    error = libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    if(error) std::cout << "error : " << errorToString(error) << std::endl;
    // Retrieve a list of all USB devices
    libusb_device **list;
    int count = libusb_get_device_list(ctx, &list);
    std::cout << "Number of devices found : " << count << std::endl;

    libusb_device* device_basestation = NULL;
    for (int i = 0; i < count; i++) {
        libusb_device *device = list[i];
        libusb_device_descriptor desc{};

        error = libusb_get_device_descriptor(device, &desc);
        if (error) std::cout << "error : " << errorToString(error) << std::endl;
        if (desc.idVendor != 0x0483 || desc.idProduct != 0x5740)
            continue;

        device_basestation = device;
        int deviceBus = libusb_get_bus_number(device);
        int deviceAddress = libusb_get_device_address(device);
        int deviceSpeed = libusb_get_device_speed(device);
        printf("\n%04x:%04x (bus %d, device %d) %s\n", desc.idVendor, desc.idProduct, deviceBus, deviceAddress, speedToString(deviceSpeed).c_str());
    }

    if(device_basestation == NULL){
        std::cout << "Basestation not found!";
        return -1;
    }

    std::cout << "Basestation found!" << std::endl;

    libusb_device_handle* handle;
    error = libusb_open(device_basestation, &handle);
    if(error){
        std::cout << "Error while trying to open handle : " << errorToString(error) << std::endl;
        return -1;
    }

    /** libusb_set_auto_detach_kernel_driver() Enable/disable libusb's automatic kernel driver detachment. When this is
     * enabled libusb will automatically detach the kernel driver on an interface when claiming the interface, and
     * attach it when releasing the interface.
     */
    error = libusb_set_auto_detach_kernel_driver(handle, 1);
    if(error){
        std::cout << "Error while enabling auto detach : " << errorToString(error) << std::endl;
        return -1;
    }

    /** libusb_claim_interface() Claim an interface on a given device handle. You must claim the interface you wish to
     * use before you can perform I/O on any of its endpoints.
     */
    error = libusb_claim_interface(handle, 1);
    if(error){
        std::cout << "Error while claiming interface : " << errorToString(error) << std::endl;
        return -1;
    }

    int actual_length = 0;
    unsigned char buffer[4096];
    for(int i = 0; i < 100; i++) buffer[i] = (i%100)+1;

    int totalBytesSent = 0;
    int totalBytesReceived = 0;

    auto start = std::chrono::high_resolution_clock::now();
    for(int counter = 1; counter < 1000; counter++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Send
        if(counter % 10 == 0) {
            buffer[0] = PACKET_TYPE_BASESTATION_GET_STATISTICS;
            error = libusb_bulk_transfer(handle, 0x01, buffer, 1, &actual_length, 500);
            if(error) std::cout << "ERROR sending : " << errorToString(error) << std::endl;
        }

        // Receive
        error = libusb_bulk_transfer(handle, 0x81, buffer, 4096, &actual_length, 0);
        if(0 < actual_length)
            std::cout << "Received " << actual_length << " bytes" << std::endl;

        if(buffer[0] == PACKET_TYPE_BASESTATION_LOG) {
            printf("LOG: ");
            for (int i = 1; i < actual_length; i++)
                printf("%c", buffer[i]);
        }

        if(buffer[0] == PACKET_TYPE_BASESTATION_STATISTICS){
            std::cout << "PACKET_TYPE_BASESTATION_STATISTICS" << std::endl;
        }
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    double seconds = elapsed.count();
    double kbpsSent = (totalBytesSent / seconds) / 1000.;
    double kbpsRcvd = (totalBytesReceived / seconds) / 1000.;

    std::cout << "totalBytesSent     " << totalBytesSent << std::endl;
    std::cout << "totalBytesReceived " << totalBytesReceived << std::endl;
    std::cout << "duration  " << seconds << std::endl;
    std::cout << "kBps sent " << kbpsSent << std::endl;
    std::cout << "kBps rcvd " << kbpsRcvd << std::endl;

    RobotCommandPayload rcp;
    rcp.payload[1] = 4;

    libusb_release_interface(handle, 1);
    libusb_close(handle);
    libusb_free_device_list(list, true);
    libusb_exit(ctx);

    return 0;
}
