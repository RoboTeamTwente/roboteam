
#include "roboteam_proto/Setting.pb.h"
#include "roboteam_utils/normalize.h"

#include "RobotHub.h"
#include "SerialDeviceManager.h"
#include "GRSim.h"
#include "packing.h"
#include "constants.h"

namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    grsimCommander = std::make_shared<GRSimCommander>();
    device = std::make_shared<SerialDeviceManager>("/dev/serial/by-id/usb-RTT_BaseStation_00000000001A-if00");
}

/// subscribe to topics
void RobotHub::subscribeToTopics(){
    robotCommandSubscriber = new roboteam_proto::Subscriber(ai_publisher, TOPIC_COMMANDS, &RobotHub::processRobotCommand, this);
    worldStateSubscriber = new roboteam_proto::Subscriber(ROBOTEAM_WORLD_TCP_PUBLISHER, TOPIC_WORLD_STATE, &RobotHub::processWorldState, this);
    settingsSubscriber = new roboteam_proto::Subscriber(ai_publisher, TOPIC_SETTINGS, &RobotHub::processSettings, this);
    publisher = new roboteam_proto::Publisher(robothub_publisher);
}


void RobotHub::loop(){
    std::chrono::high_resolution_clock::time_point lastStatistics = std::chrono::high_resolution_clock::now();

    grsimCommander->setBatch(true);

    int currIteration = 0;
    while (true) {
        auto timeNow = std::chrono::high_resolution_clock::now();
        auto timeDiff = timeNow-lastStatistics;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(timeDiff).count()>1000) {
            lastStatistics = timeNow;
            std::cout << "==========| " << currIteration++ << "   " << utils::modeToString(mode) << " |==========" << std::endl;
            printStatistics();

        }
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

/// print robot ticks in a nicely formatted way
void RobotHub::printStatistics() {
    const int amountOfColumns = 4;
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i+=amountOfColumns) {
        for (int j = 0; j < amountOfColumns; j++) {
            const int robotId = i+j;
            if (robotId < MAX_AMOUNT_OF_ROBOTS) {
                std::cout << robotId << ": " << robotTicks[robotId] << "\t";
                robotTicks[robotId] = 0;
            }
        }
        std::cout << std::endl;
    }
}

void RobotHub::processWorldState(roboteam_proto::World & world){
    std::lock_guard<std::mutex> lock(worldLock);
    if(!isLeft) roboteam_utils::rotate(&world);
    LastWorld = world;
}

void RobotHub::processRobotCommand(roboteam_proto::RobotCommand & cmd) {
    std::lock_guard<std::mutex> lock(worldLock);
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(cmd, LastWorld);

    // check if the command is valid, otherwise don't send anything
    if(!validateRobotPacket(llrc)) {
        std::cout << "[processRobotCommand] LowLevelRobotCommand is not valid for our robots, no command is being sent!" << std::endl;
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
    if(!bytestream){
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
void RobotHub::sendGrSimCommand(const roboteam_proto::RobotCommand& robotCommand) {
    this->grsimCommander->queueGRSimCommand(robotCommand);
}

void RobotHub::publishRobotFeedback(LowLevelRobotFeedback llrf) {
    if (llrf.id >= 0 && llrf.id < 16) {
        publisher->send(rtt::TOPIC_FEEDBACK, toRobotFeedback(llrf).SerializeAsString());
    }
}

void RobotHub::processSettings(roboteam_proto::Setting &setting) {
    grsimCommander->setColor(setting.isyellow());
    grsimCommander->setGrsim_ip(setting.robothubsendip());
    grsimCommander->setGrsim_port(setting.robothubsendport());
    isLeft = setting.isleft();

    if (setting.serialmode()) {
        mode = utils::Mode::SERIAL;
    } else {
        mode = utils::Mode::GRSIM;
    }
}

void RobotHub::setAiPublisher(const string &aiPublisher) {
    ai_publisher = aiPublisher;
}

void RobotHub::setRobothubPublisher(const string &robothubPublisher) {
    robothub_publisher = robothubPublisher;
}

} // robothub
} // rtt
