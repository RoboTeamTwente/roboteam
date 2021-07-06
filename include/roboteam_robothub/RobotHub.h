//
// Created by emiel on 22-05-21.
//

#ifndef RTT_ROBOTHUB_H
#define RTT_ROBOTHUB_H

#include <libusb-1.0/libusb.h>

#include <Publisher.h>
#include <Subscriber.h>
#include <roboteam_proto/AICommand.pb.h>
#include <roboteam_proto/Setting.pb.h>
#include <roboteam_proto/RobotFeedback.pb.h>

#include "GRSim.h"
#include "utilities.h"
#include "constants.h"

namespace rtt {
namespace robothub {

class RobotHub {
public:
    RobotHub();
    ~RobotHub();
    void run();

    void handleBasestationAttach(libusb_device* device);
    void handleBasestationDetach(libusb_device* device);
private:

    void subscribe();

    // unused but kept for future reference
    bool openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle);

    bool running = true;
    bool read_running = true;

    void printStatistics();
    int commands_sent[MAX_AMOUNT_OF_ROBOTS] = {};
    int feedback_received[MAX_AMOUNT_OF_ROBOTS] = {};

    proto::Setting settings;

    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber;
    std::unique_ptr<proto::Subscriber<proto::World>> worldStateSubscriber;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber;
    std::unique_ptr<proto::Publisher<proto::RobotFeedback>> feedbackPublisher;

    /* libusb */
    bool startBasestation();

    libusb_context *ctx;
    libusb_device* basestation_device = nullptr;
    libusb_device_handle* basestation_handle = nullptr;
    libusb_hotplug_callback_handle callback_handle_attach;
    libusb_hotplug_callback_handle callback_handle_detach;

    std::shared_ptr<GRSimCommander> grsimCommander;

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
