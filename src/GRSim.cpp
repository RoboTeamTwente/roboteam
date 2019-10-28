#include <numeric>
#include "GRSim.h"
#include "roboteam_utils/Vector2.h"
#include "packing.h"

#define VERIFY_COMMANDS

namespace rtt {
namespace robothub {

#ifdef VERIFY_COMMANDS

bool verifyCommandIntegrity(const proto::RobotCommand& cmd, std::string mode)
{
    if (cmd.id()<0) {
        std::cout << "RobotHub (" << mode.c_str() << "): Invalid ID number: " << cmd.id() <<  " (should be positive)" << std::endl;
        return false;
    }
    if (fabs(cmd.vel().x())>10000) {
        std::cout << "RobotHub (" << mode.c_str() << "): X velocity sanity check for " << cmd.id()<< " failed: " << cmd.vel().x() << std::endl;
        return false;
    }
    if (fabs(cmd.vel().x())>10000) {
        std::cout << "RobotHub (" << mode.c_str() << "): Y velocity sanity check for " <<  cmd.id() << " failed: " << cmd.vel().x() << std::endl;
        return false;
    }
    if (fabs(cmd.w())>10000) {
        std::cout << "RobotHub (" << mode.c_str() << "): Rotation velocity sanity check for " << cmd.id() << " failed:" << cmd.w() << std::endl;
        return false;
    }
    if (cmd.vel().x()!=cmd.vel().x()) {
        std::cout << "RobotHub (" << mode.c_str() << "): X velocity for " <<  cmd.id() << " is NAN." << std::endl;
        return false;
    }
    if (cmd.vel().y()!=cmd.vel().y()) {
        std::cout << "RobotHub (" << mode.c_str() << "): Y velocity for " <<  cmd.id() << " is NAN." << std::endl;
        return false;
    }
    if (cmd.w()!=cmd.w()) {
        std::cout << "RobotHub ("<< mode.c_str() <<"): Rotation velocity for "<<  cmd.id() << " is NAN." << std::endl;
        return false;
    }
    if (cmd.geneva_state()<0 || cmd.geneva_state()>5) {
        std::cout << "RobotHub ("<< mode.c_str() <<"): Geneva Drive state of "<< cmd.id() << " is out of bounds. " << cmd.geneva_state() << std::endl;
        return false;
    }
    return true;
}

#endif

GRSimCommander::GRSimCommander(bool batch) :
        batch{batch},
//        colorParam("our_color"),
//        grsim_ip("grsim/ip", "127.0.0.1"),
//        grsim_port("grsim/port", 20011),
        msgReceivedBits{},
        robotsSeenBits{},
        threshold{},
        drops{},
        lastBatchTime{Clock::now()},
        efficiencyIndex{},
        numForcedFlushes{}
{

}

void GRSimCommander::queueGRSimCommand(const proto::RobotCommand& msg)
{
    using namespace std::chrono;

    if (msg.id()>=16) {
        std::cerr << "ID in RobotCommand is >= 16, not allowed!" << std::endl;
        return;
    }

    if (batch) {
        // If we already received this ID, increase drop count
        if (hasMsgForID(msg.id())) {
            drops++;
        }

        if (TRACE) std::cout << "Got message for: " << msg.id() << "\n";

        // Store the message and set appropriate flags
        robotCommands[msg.id()] = std::make_shared<proto::RobotCommand>(msg);
        msgReceivedBits = msgReceivedBits | (1 << msg.id());
        robotsSeenBits = robotsSeenBits | (1 << msg.id());

        // If MAX_DROPS was reached...
        if (drops>=MAX_DROPS) {
            if (TRACE) std::cout << "drops (" << drops << ") >= MAX_DROPS " << MAX_DROPS << "\n";

            // Record a forced flush
            numForcedFlushes++;
            if (TRACE) std::cout << "Incremented numForcedFlushes!\n";

            // Forcibly update the threshold
            updateThreshold();
            // And flush the buffer
            flush();
        }
        else if (getMsgsQueued()>=threshold) {
            // Else if we've reached the buffer threshold...
            if (TRACE) std::cout << "Messages queued: " << getMsgsQueued() << ", threshold: " << threshold << "\n";

            // Check if the threshold has to be updated. If so, do.
            auto const now = Clock::now();
            if (duration_cast<milliseconds>(now-lastBatchTime).count()>=250) {
                updateThreshold();
                lastBatchTime = now;
            }

            // Flush the buffer
            flush();
        }
    }
    else {
        // If not batching, just send it right away
        if (TRACE) std::cout << "Got message for: " << msg.id() << ". Sending NOW\n";
        sendGRSimCommand(msg);
    }
}

void GRSimCommander::updateThreshold()
{
    // Count the 1's in robotsSeenBits. That's the amount of robots seen and
    // thus the new threshold.
    // TODO: @Portability. needs a define for gcc/msvc
    threshold = __builtin_popcount(robotsSeenBits);
    robotsSeenBits = 0;

    if (TRACE) std::cout << "Updating threshold! New threshold: " << threshold << "\n";
}

void GRSimCommander::flush()
{
    // Send the buffered commands if available.
    if (getMsgsQueued()>0) {
        sendMultipleGRSimCommands(robotCommands);
    }

    // Record efficiency
    efficiency[efficiencyIndex] = getMsgsQueued()/(double) threshold;
    efficiencyIndex = (efficiencyIndex+1)%efficiency.size();

    // Reset tracking vars
    robotCommands.fill(nullptr);
    drops = 0;
    msgReceivedBits = 0;

    if (TRACE) std::cout << "Flushing.\n";
}

void GRSimCommander::sendGRSimPacket(proto::grSim_Packet const& packet)
{
    // Use our preallocated buffer if it's big enough. If not, do the slow approach
    if (packet.ByteSize()<1024) {
        packet.SerializeToArray(packetBuffer.data(), packetBuffer.size());
        udpsocket.writeDatagram(
                packetBuffer.data(),
                packet.ByteSize(),
                QHostAddress(QString::fromStdString(grsim_ip)),
                grsim_port
        );
    } else {
        QByteArray dgram;
        dgram.resize(packet.ByteSize());
        packet.SerializeToArray(dgram.data(), dgram.size());
        udpsocket.writeDatagram(
                dgram,
                QHostAddress(QString::fromStdString(grsim_ip)),
                grsim_port
        );
    }
}

void GRSimCommander::sendGRSimCommand(const proto::RobotCommand& _msg)
{
#ifdef VERIFY_COMMANDS
    if (!verifyCommandIntegrity(_msg, "grsim")) {
        return;
    }
#endif

    proto::grSim_Packet packet;

    time_t timeInSec;
    time(&timeInSec);

    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp((double) timeInSec);

    addRobotCommandToPacket(packet, _msg);

    sendGRSimPacket(packet);
}

void GRSimCommander::sendMultipleGRSimCommands(const std::vector<proto::RobotCommand>& msgs)
{
#ifdef VERIFY_COMMANDS
    for (auto const& msg : msgs) {
        if (!verifyCommandIntegrity(msg, "grsim")) {
            return;
        }
    }
#endif

    proto::grSim_Packet packet;


    time_t timeInSec;
    time(&timeInSec);

    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp((double) timeInSec);

    for (auto const& msg : msgs) {
        addRobotCommandToPacket(packet, msg);
    }

    sendGRSimPacket(packet);
}

void GRSimCommander::sendMultipleGRSimCommands(const RobotCommandBuffer& msgs)
{
#ifdef VERIFY_COMMANDS
    for (auto const& msg : msgs) {
        if (msg && !verifyCommandIntegrity(*msg, "grsim")) {
            return;
        }
    }
#endif

    proto::grSim_Packet packet;

    time_t timeInSec;
    time(&timeInSec);

    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp(timeInSec);

    for (auto const& msg : msgs) {
        if (msg) {
            addRobotCommandToPacket(packet, *msg);
        }
    }

    sendGRSimPacket(packet);
}

bool GRSimCommander::hasMsgForID(int const id)
{
    if (!batch) {
        // If not batching we don't have any messages
        return false;
    }
    else if (id>=16) {
        // If the id is big we don't have a message for it
        return false;
    }
    else {
        // Otherwise return the id'th bit
        return (1 << id) & msgReceivedBits;
    }
}

int GRSimCommander::getMsgsQueued()
{
    // Popcount counts the number of 1's in an integer in one instruction
    // TODO: @Portability. needs a define for gcc/msvc
    return __builtin_popcount(msgReceivedBits);
}

GRSimCommander::Stats GRSimCommander::consumeStatistics()
{
    Stats stats;
    stats.averageEfficiency = std::accumulate(efficiency.begin(), efficiency.end(), 0.0)/(double) HISTORY_LEN;;
    stats.numForcedFlushes = numForcedFlushes;
    // This is for general info and doesn't have to be reset
    stats.threshold = threshold;

    // Reset statistics
    numForcedFlushes = 0;

    if (TRACE) std::cout << "numForcedFlushes after consumption: " << numForcedFlushes << "\n";

    return stats;
}

bool GRSimCommander::isBatch()
{
    return batch;
}

void GRSimCommander::setBatch(bool _batch)
{
    this->batch = _batch;
}

        void GRSimCommander::setColor(bool yellow) {
isYellow = yellow;
        }

void GRSimCommander::setGrsim_ip(const std::string &grsim_ip) {
    GRSimCommander::grsim_ip = grsim_ip;
}
void GRSimCommander::setGrsim_port(quint16 grsim_port) {
    GRSimCommander::grsim_port = grsim_port;
}

void addRobotCommandToPacket(proto::grSim_Packet& packet, proto::RobotCommand const& msg)
{

    proto::grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();


//    commands.robot_commands[0].kickspeedx,
//    commands.robot_commands[0].kickspeedz,
//    commands.robot_commands[0].spinner,
//    commands.robot_commands[0].wheelsspeed

    command->set_id(msg.id());
    command->set_wheelsspeed(false);
    command->set_veltangent(msg.vel().x());
    command->set_velnormal(msg.vel().y());
    command->set_velangular(msg.w());

    if (msg.kicker()) {
        command->set_kickspeedx(msg.chip_kick_vel());
    }
    else {
        command->set_kickspeedx(0);
    }

    if (msg.chipper()) {
        rtt::Vector2 vel = rtt::Vector2(msg.chip_kick_vel(), 0);
        vel = vel.rotate(M_PI/4); // 45 degrees up.

        command->set_kickspeedx(vel.x);
        command->set_kickspeedz(vel.y);
    }
    else {
        command->set_kickspeedz(0);
    }

    command->set_spinner((msg.dribbler() > 0));
    command->set_use_angle(msg.use_angle());

    // if no genevastate was given we set it to 3;
    int genevaState = 3;
    if (msg.geneva_state() != 0) {
        genevaState = msg.geneva_state()-1;
    }
    // angles in degrees

    float angles[] = {20.0, 10.0, 0.0, -10.0, -20.0};

    // geneva_angle in radians
    float geneva_angle = 2.0*M_PI*angles[genevaState]/360.0;

    command->set_geneva_angle(geneva_angle);

}

} // robothub
} // rtt