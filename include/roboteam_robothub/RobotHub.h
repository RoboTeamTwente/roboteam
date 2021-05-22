//
// Created by emiel on 22-05-21.
//

#ifndef RTT_ROBOTHUB_H
#define RTT_ROBOTHUB_H

#include <Publisher.h>
#include <Subscriber.h>
#include "roboteam_proto/Setting.pb.h"

#include "GRSim.h"
#include "BasestationWriter.h"
#include "BasestationReader.h"
#include "utilities.h"

namespace rtt {
namespace robothub {

class RobotHub {
public:
    RobotHub();
    void subscribe();
    void run();

    void handleBasestationAttach(libusb_device* device);
    void handleBasestationDetach(libusb_device* device);
private:
    bool startBasestation();
    bool openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle);
    bool running = true;
    int whatever = 123;

    proto::Setting settings;

    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber;
    std::unique_ptr<proto::Subscriber<proto::World>> worldStateSubscriber;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber;
    std::unique_ptr<proto::Publisher<proto::RobotFeedback>> feedbackPublisher;

    libusb_context *ctx;
    libusb_device* basestation_device;
    libusb_device_handle* basestation_handle;
    bool usb_initialized = false;
    libusb_hotplug_callback_handle callback_handle_attach;
    libusb_hotplug_callback_handle callback_handle_detach;



    std::shared_ptr<GRSimCommander> grsimCommander;
    std::unique_ptr<BasestationReader> reader;

    std::thread readerThread;

    std::mutex worldLock;
    proto::World world;
    void sendSerialCommand(const proto::RobotCommand &robotCommand);
    void sendGrSimCommand(const proto::RobotCommand &robotCommand);
    void readBasestation();

    void processAIcommand(proto::AICommand &AIcmd);
    void processWorldState(proto::World &world);
    void processSettings(proto::Setting &setting);
};

}  // namespace robothub
}  // namespace rtt

#endif //RTT_ROBOTHUB_H
