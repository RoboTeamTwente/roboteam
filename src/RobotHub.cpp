
#include <roboteam_utils/Timer.h>
#include "roboteam_proto/Setting.pb.h"
#include "roboteam_utils/normalize.h"

#include "GRSim.h"
#include "RobotHub.h"
#include "SerialDeviceManager.h"
#include "packing.h"

namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    grsimCommander = std::make_shared<GRSimCommander>();

#ifdef __APPLE__
    device = std::make_shared<SerialDeviceManager>("/dev/cu.usbmodem00000000001A1");
#else
    device = std::make_shared<SerialDeviceManager>("/dev/serial/by-id/usb-RTT_BaseStation_00000000001A-if00");
#endif
}

/// subscribe to topics
void RobotHub::subscribeToTopics() {
    /**
     * Memory leaks right here, make these unique_ptr's
     */
    robotCommandSubscriber = new proto::Subscriber<proto::RobotCommand>(robotCommandChannel, &RobotHub::processRobotCommand, this);

    worldStateSubscriber = new proto::Subscriber<proto::World>(proto::WORLD_CHANNEL, &RobotHub::processWorldState, this);

    settingsSubscriber = new proto::Subscriber<proto::Setting>(settingsChannel, &RobotHub::processSettings, this);

    feedbackPublisher = new proto::Publisher<proto::RobotFeedback>(feedbackChannel);
}

void RobotHub::start() {
    int currIteration = 0;
    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            std::cout << "==========| " << currIteration++ << "   " << utils::modeToString(mode) << " |==========" << std::endl;
            printStatistics();
        },
        1);
}

/// print robot ticks in a nicely formatted way
void RobotHub::printStatistics() {
    const int amountOfColumns = 4;
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i += amountOfColumns) {
        for (int j = 0; j < amountOfColumns; j++) {
            const int robotId = i + j;
            if (robotId < MAX_AMOUNT_OF_ROBOTS) {
                std::cout << robotId << ": " << robotTicks[robotId] << "\t";
                robotTicks[robotId] = 0;
            }
        }
        std::cout << std::endl;
    }
}

void RobotHub::processWorldState(proto::World &world) {
    std::lock_guard<std::mutex> lock(worldLock);
    LastWorld = world;
}

void RobotHub::processRobotCommand(proto::RobotCommand &cmd) {
    std::lock_guard<std::mutex> lock(worldLock);
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmd, LastWorld, isYellow);

    // check if the command is valid, otherwise don't send anything
    if (!validateRobotPacket(llrc)) {
        std::cout << "[processRobotCommand] LowLevelRobotCommand is not valid "
                     "for our robots, no command is being sent!"
                  << std::endl;
        printLowLevelRobotCommand(llrc);
        return;
    }

    robotTicks[cmd.id()]++;
    if (mode == utils::Mode::SERIAL) {
        sendSerialCommand(llrc);
    } else {
        sendGrSimCommand(cmd);
    }
}

/// send a serial command from a given robotcommand
void RobotHub::sendSerialCommand(LowLevelRobotCommand llrc) {
    // convert the LLRC to a bytestream which we can send
    std::shared_ptr<packed_protocol_message> bytestream = createRobotPacket(llrc);

    if (!device->ensureDeviceOpen()) {
        device->openDevice();
    }

    // Check if the message was created successfully
    if (!bytestream) {
        std::cout << "[sendSerialCommand] The message was not created succesfully!" << std::endl;
        return;
    }
    packed_protocol_message packet = *bytestream;
    device->writeToDevice(packet);
    if (device->getMostRecentFeedback()) {
        publishRobotFeedback(createRobotFeedback(*device->getMostRecentFeedback()));
        device->removeMostRecentFeedback();
    }
}

/// send a GRSim command from a given robotcommand
void RobotHub::sendGrSimCommand(const proto::RobotCommand &robotCommand) { this->grsimCommander->queueGRSimCommand(robotCommand); }

void RobotHub::publishRobotFeedback(LowLevelRobotFeedback llrf) {
    if (llrf.id >= 0 && llrf.id < 16) {
        feedbackPublisher->send(toRobotFeedback(llrf));
    }
}

void RobotHub::processSettings(proto::Setting &setting) {
    grsimCommander->setGrsim_ip(setting.robothubsendip());
    grsimCommander->setGrsim_port(setting.robothubsendport());
    isLeft = setting.isleft();
    grsimCommander->setColor(setting.isyellow());
    isYellow = setting.isyellow();

    if (setting.serialmode()) {
        mode = utils::Mode::SERIAL;
    } else {
        mode = utils::Mode::GRSIM;
    }
}
void RobotHub::set_robot_command_channel(const proto::ChannelType &robot_command_channel) { robotCommandChannel = robot_command_channel; }
void RobotHub::set_feedback_channel(const proto::ChannelType &feedback_channel) { feedbackChannel = feedback_channel; }
void RobotHub::set_settings_channel(const proto::ChannelType &settings_channel) { settingsChannel = settings_channel; }

}  // namespace robothub
}  // namespace rtt
