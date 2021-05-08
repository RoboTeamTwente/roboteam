
#include <roboteam_utils/Timer.h>
#include "roboteam_proto/Setting.pb.h"

#include "GRSim.h"
#include "RobotHub.h"
#include "SerialDeviceManager.h"
#include "packing.h"

namespace rtt::robothub {

RobotHub::RobotHub() {
    grsimCommander = std::make_shared<GRSimCommander>();

#ifdef __APPLE__
    device = std::make_shared<SerialDeviceManager>("/dev/cu.usbmodem00000000001A1");
#else
    device = std::make_shared<SerialDeviceManager>("/dev/serial/by-id/usb-RTT_BaseStation_00000000001A-if00");
#endif

    simulator_connection = std::make_shared<SSLSimulator>();
}

/// subscribe to topics
void RobotHub::subscribeToTopics() {
    robotCommandSubscriber = new proto::Subscriber<proto::AICommand>(robotCommandChannel, &RobotHub::processAIBatch, this);

    settingsSubscriber = new proto::Subscriber<proto::Setting>(settingsChannel, &RobotHub::processSettings, this);

    feedbackPublisher = new proto::Publisher<proto::RobotData>(feedbackChannel);
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

void RobotHub::processAIBatch(proto::AICommand &cmd) {
  //TODO split up sending here, not in the lower functions
  if( mode == utils::Mode::SSL_SIMULATOR){
      sendSimulatorBatch(cmd);
      return;
  }
  proto::RobotData sentCommands;
  sentCommands.set_isyellow(isYellow);
  for(const auto& command : cmd.commands()){
    bool wasSent =processCommand(command,cmd.extrapolatedworld());
    if(wasSent){
      proto::RobotCommand * sent = sentCommands.mutable_sentcommands()->Add();
      sent->CopyFrom(command);
    }
  }
  //TODO: add times command was sent
  feedbackPublisher->send(sentCommands);

}
bool RobotHub::processCommand(const proto::RobotCommand &robotCommand,const proto::World &world) {
    LowLevelRobotCommand llrc = createLowLevelRobotCommand(robotCommand, world, isYellow);

    // check if the command is valid, otherwise don't send anything
    if (!validateRobotPacket(llrc)) {
        std::cout << "[processRobotCommand] LowLevelRobotCommand is not valid "
                     "for our robots, no command is being sent!"
                  << std::endl;
        printLowLevelRobotCommand(llrc);
        return false;
    }

    robotTicks[robotCommand.id()]++;
    if (mode == utils::Mode::SERIAL) {
        return sendSerialCommand(llrc);
    } else {
        return sendGrSimCommand(robotCommand);
    }
}
/// send a serial command from a given robotcommand
bool RobotHub::sendSerialCommand(LowLevelRobotCommand llrc) {
    // convert the LLRC to a bytestream which we can send
    std::shared_ptr<packed_protocol_message> bytestream = createRobotPacket(llrc);

    if (!device->ensureDeviceOpen()) {
        device->openDevice();
    }

    // Check if the message was created successfully
    if (!bytestream) {
        std::cout << "[sendSerialCommand] The message was not created succesfully!" << std::endl;
        return false;
    }
    packed_protocol_message packet = *bytestream;
    device->writeToDevice(packet);
    if (device->getMostRecentFeedback()) {
        publishRobotFeedback(createRobotFeedback(*device->getMostRecentFeedback()));
        device->removeMostRecentFeedback();
    }
    return true;
}

/// send a GRSim command from a given robotcommand
bool RobotHub::sendGrSimCommand(const proto::RobotCommand &robotCommand) {
  this->grsimCommander->queueGRSimCommand(robotCommand);
  return true;
}

void RobotHub::publishRobotFeedback(LowLevelRobotFeedback llrf) {
    if (llrf.id >= 0 && llrf.id < 16) {
        proto::RobotData data;
        proto::RobotFeedback * feedback = data.mutable_receivedfeedback()->Add();
        feedback->CopyFrom(toRobotFeedback(llrf));
        data.set_isyellow(isYellow);
        feedbackPublisher->send(data);
    }
}

void RobotHub::processSettings(proto::Setting &setting) {
    grsimCommander->setGrsim_ip(setting.robothubsendip());
    grsimCommander->setGrsim_port(setting.robothubsendport());
    isLeft = setting.isleft();
    grsimCommander->setColor(setting.isyellow());
    isYellow = setting.isyellow();

    simulator_connection->set_ip(setting.robothubsendip());
    simulator_connection->set_color(setting.isyellow());
    if (setting.serialmode()) {
        mode = utils::Mode::SERIAL;
    } else {
        //mode = utils::Mode::GRSIM; //TODO; quick hack, make sure these can coexist, but need to fix this in AI and other places as well (bad idea for now)
        mode = utils::Mode::SSL_SIMULATOR;
    }
}
void RobotHub::set_robot_command_channel(const proto::ChannelType &robot_command_channel) { robotCommandChannel = robot_command_channel; }
void RobotHub::set_feedback_channel(const proto::ChannelType &feedback_channel) { feedbackChannel = feedback_channel; }
void RobotHub::set_settings_channel(const proto::ChannelType &settings_channel) { settingsChannel = settings_channel; }

void RobotHub::sendSimulatorBatch(proto::AICommand &cmd) {
    proto::RobotData sentCommands;
    sentCommands.set_isyellow(isYellow);
    for(const auto& command : cmd.commands()){
        LowLevelRobotCommand llrc = createLowLevelRobotCommand(command, cmd.extrapolatedworld(), isYellow);

        // check if the command is valid, otherwise don't send anything
        if (!validateRobotPacket(llrc)) {
            std::cout << "[processRobotCommand] LowLevelRobotCommand is not valid "
                         "for our robots, no command is being sent!"
                      << std::endl;
            printLowLevelRobotCommand(llrc);

        }else{
            simulator_connection->add_robot_command(command);
            robotTicks[command.id()]++;
            proto::RobotCommand * sent = sentCommands.mutable_sentcommands()->Add();
            sent->CopyFrom(command);
        }
    }
    simulator_connection->send_commands();
    feedbackPublisher->send(sentCommands);
}

}  // namespace rtt
