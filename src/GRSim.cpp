#include <roboteam_msgs/RobotCommand.h>

#include <numeric>
#include "GRSim.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/RobotFeedback.h"
#include "packing.h"

#define VERIFY_COMMANDS

namespace rtt {
namespace robothub {

#ifdef VERIFY_COMMANDS

bool verifyCommandIntegrity(const roboteam_msgs::RobotCommand& cmd, std::string mode)
{
    if (cmd.id<0) {
        ROS_ERROR("RobotHub (%s): Invalid ID number: %d (should be positive)", mode.c_str(), cmd.id);
        return false;
    }
    if (fabs(cmd.x_vel)>10000) {
        ROS_ERROR("RobotHub (%s): X velocity sanity check for %d failed: %f", mode.c_str(), cmd.id, cmd.x_vel);
        return false;
    }
    if (fabs(cmd.y_vel)>10000) {
        ROS_ERROR("RobotHub (%s): Y velocity sanity check for %d failed: %f", mode.c_str(), cmd.id, cmd.y_vel);
        return false;
    }
    if (fabs(cmd.w)>10000) {
        ROS_ERROR("RobotHub (%s): Rotation velocity sanity check for %d failed: %f", mode.c_str(), cmd.id, cmd.w);
        return false;
    }
    if (cmd.x_vel!=cmd.x_vel) {
        ROS_ERROR("RobotHub (%s): X velocity for %d is NAN.", mode.c_str(), cmd.id);
        return false;
    }
    if (cmd.y_vel!=cmd.y_vel) {
        ROS_ERROR("RobotHub (%s): Y velocity for %d is NAN.", mode.c_str(), cmd.id);
        return false;
    }
    if (cmd.w!=cmd.w) {
        ROS_ERROR("RobotHub (%s): Rotation velocity for %d is NAN.", mode.c_str(), cmd.id);
        return false;
    }
    if (cmd.geneva_state<0 || cmd.geneva_state>5) {
        ROS_ERROR("RoboHub (%s): Geneva Drive state of %d is out of bounds.", mode.c_str(), cmd.geneva_state);
        return false;
    }
    return true;
}

#endif

GRSimCommander::GRSimCommander(bool batch)
        :
        batch{batch},
        colorParam("our_color"),
        grsim_ip("grsim/ip", "127.0.0.1"),
        grsim_port("grsim/port", 20011),
        msgReceivedBits{},
        robotsSeenBits{},
        threshold{},
        drops{},
        lastBatchTime{Clock::now()},
        efficiencyIndex{},
        numForcedFlushes{}
{

}

void GRSimCommander::queueGRSimCommand(const roboteam_msgs::RobotCommand& msg)
{
    using namespace std::chrono;

    if (msg.id>=16) {
        ROS_ERROR("ID in RobotCommand is >= 16, not allowed!");
        return;
    }

    if (batch) {
        // If we already received this ID, increase drop count
        if (hasMsgForID(msg.id)) {
            drops++;
        }

        if (TRACE) std::cout << "Got message for: " << msg.id << "\n";

        // Store the message and set appropriate flags
        robotCommands[msg.id] = std::make_shared<roboteam_msgs::RobotCommand>(msg);
        msgReceivedBits = msgReceivedBits | (1 << msg.id);
        robotsSeenBits = robotsSeenBits | (1 << msg.id);

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
        if (TRACE) std::cout << "Got message for: " << msg.id << ". Sending NOW\n";
        sendGRSimCommand(msg);
    }

    printRobotCommand(msg);
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

void GRSimCommander::sendGRSimPacket(grSim_Packet const& packet)
{
    // Use our preallocated buffer if it's big enough. If not, do the slow approach
    if (packet.ByteSize()<1024) {
        packet.SerializeToArray(packetBuffer.data(), packetBuffer.size());
        udpsocket.writeDatagram(
                packetBuffer.data(),
                packet.ByteSize(),
                QHostAddress(QString::fromStdString(grsim_ip())), grsim_port()
        );
    }
    else {
        QByteArray dgram;
        dgram.resize(packet.ByteSize());
        packet.SerializeToArray(dgram.data(), dgram.size());
        udpsocket.writeDatagram(
                dgram,
                QHostAddress(QString::fromStdString(grsim_ip())), grsim_port()
        );
    }
}

void GRSimCommander::sendGRSimCommand(const roboteam_msgs::RobotCommand& _msg)
{
#ifdef VERIFY_COMMANDS
    if (!verifyCommandIntegrity(_msg, "grsim")) {
        return;
    }
#endif

    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(colorParam()=="yellow");
    packet.mutable_commands()->set_timestamp(ros::Time::now().toSec());

    addRobotCommandToPacket(packet, _msg);

    sendGRSimPacket(packet);
}

void GRSimCommander::sendMultipleGRSimCommands(const std::vector<roboteam_msgs::RobotCommand>& msgs)
{
#ifdef VERIFY_COMMANDS
    for (auto const& msg : msgs) {
        if (!verifyCommandIntegrity(msg, "grsim")) {
            return;
        }
    }
#endif

    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(colorParam()=="yellow");
    packet.mutable_commands()->set_timestamp(ros::Time::now().toSec());

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

    grSim_Packet packet;

    packet.mutable_commands()->set_isteamyellow(colorParam()=="yellow");
    packet.mutable_commands()->set_timestamp(ros::Time::now().toSec());

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

void GRSimCommander::setBatch(bool batch)
{
    this->batch = batch;
}

void addRobotCommandToPacket(grSim_Packet& packet, roboteam_msgs::RobotCommand const& msg)
{
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    command->set_id(msg.id);
    command->set_wheelsspeed(false);
    command->set_veltangent(msg.x_vel);
    command->set_velnormal(msg.y_vel);
    command->set_velangular(msg.w);

    if (msg.kicker || msg.kicker_forced) {
        command->set_kickspeedx(msg.kicker_vel);
    }
    else {
        command->set_kickspeedx(0);
    }

    if (msg.chipper || msg.chipper_forced) {
        rtt::Vector2 vel = rtt::Vector2(msg.chipper_vel, 0);
        vel = vel.rotate(M_PI/4); // 45 degrees up.

        command->set_kickspeedx(vel.x);
        command->set_kickspeedz(vel.y);
    }
    else {
        command->set_kickspeedz(0);
    }

    command->set_spinner(msg.dribbler);
    command->set_use_angle(msg.use_angle);

    // if no genevastate was given we set it to 3;
    int genevaState = 3;
    if (msg.geneva_state != 0) {
        genevaState = msg.geneva_state-1;
    }
    // angles in degrees
    float angles[] = {20, 10, 0, -10, -20};

    // geneva_angle in radians
    float geneva_angle = 2*M_PI*angles[ msg.geneva_state-1]/360;

    command->set_geneva_angle(geneva_angle);
}

} // robothub
} // rtt