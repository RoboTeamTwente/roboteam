//
// Created by emiel on 22-05-21.
//

#ifndef RTT_ROBOTHUB_H
#define RTT_ROBOTHUB_H

#include <Publisher.h>
#include <Subscriber.h>

#include "roboteam_proto/Setting.pb.h"

#include "BasestationWriter.h"
#include "BasestationReader.h"
#include "utilities.h"

namespace rtt {
namespace robothub {

class RobotHub {
public:
    RobotHub();

private:
    bool startBasestation();
    bool openBasestation(libusb_context* ctx, libusb_device_handle **basestation_handle);

    utils::Mode mode = utils::Mode::GRSIM;
    bool isLeft = true;
    bool isYellow = true;

    proto::ChannelType robotCommandChannel;
    proto::ChannelType settingsChannel;

    std::unique_ptr<proto::Subscriber<proto::AICommand>> robotCommandSubscriber;
    std::unique_ptr<proto::Subscriber<proto::World>> worldStateSubscriber;
    std::unique_ptr<proto::Subscriber<proto::Setting>> settingsSubscriber;
    std::unique_ptr<proto::Publisher<proto::RobotFeedback>> feedbackPublisher;

    libusb_context *ctx;
    libusb_device_handle* basestation_handle;

    std::unique_ptr<BasestationWriter> writer;
    std::unique_ptr<BasestationReader> reader;

    std::mutex worldLock;
    proto::World LastWorld;
    void processWorldState(proto::World &world);
    void processSettings(proto::Setting &setting);
};

}  // namespace robothub
}  // namespace rtt

#endif //RTT_ROBOTHUB_H
