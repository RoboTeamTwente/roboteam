//
// Created by emiel on 22-05-21.
//

#ifndef RTT_BASESTATIONREADER_H
#define RTT_BASESTATIONREADER_H

#include <libusb-1.0/libusb.h>

namespace rtt {
namespace robothub {

class BasestationReader {
public:
    libusb_device_handle* basestation_handle;
    bool running = true;
    int totalBytesReceived = 0;

    BasestationReader();

    void setBasestationHandle(libusb_device_handle* _basestation_handle);

    void run();

};

} // namespace robothub
} // namespace rtt

#endif //RTT_BASESTATIONREADER_H
