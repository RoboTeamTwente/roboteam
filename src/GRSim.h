#pragma once

#include <array>
#include <QtNetwork>
#include <ros/message_forward.h>
#include <string>
#include <boost/optional.hpp>
#include <chrono>

#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/SlowParam.h"

namespace roboteam_msgs {
    ROS_DECLARE_MESSAGE(RobotCommand);
}

namespace rtt {

class GRSimCommander {
public:
    /**
     * Buffer type which keeps all received robot commands
     */
    using RobotCommandBuffer = std::array<boost::optional<roboteam_msgs::RobotCommand>, 16>;

    /**
     * Clock that always increases no matter what (daylight saving etc.)
     */
    using Clock = std::chrono::steady_clock;

    /**
     * Timepoint type used for time recording
     */
    using TimePoint = std::chrono::time_point<Clock>;
    
    /** 
     * Statistics struct
     */
    struct Stats {
        /**
         * Efficiency is how close we got to the threshold.
         * An efficiency of 100% means we flushed a full buffer.
         * An efficiency of 50% means we flushed a half-filled buffer.
         */
        double averageEfficiency;

        /**
         * The amount of times the buffer was forcibly flushed
         * because of collisions. Should be really low.
         */
        int numForcedFlushes;

        /**
         * The threshold for the buffer to be flushed.
         */
        int threshold;
    };

    /** Maximum amount of double messages to be received
      * before the buffer is forcibly flushed and treshold
      * re-estimated.
      */
    static int const MAX_DROPS = 2;

    /** 
     * Number of entries in the efficiency history
     */
    static int const HISTORY_LEN = 60;

    /**
     * When true, detailed debug trace info is printed.
     */
    static bool const TRACE = false;

    /**
     * When batch is true batching is turned on from the start.
     */
    GRSimCommander(bool batch = false);

    /**
     * Submits a GRSim command for sending to the simulator.
     * If batching is enabled it might not be sent immediately.
     * If batching is disabled it's immediately sent.
     */
    void queueGRSimCommand(const roboteam_msgs::RobotCommand & msg);

    /**
     * Sends a GRSim packet to the simulator using
     * GRSimCommander's UDP socket.
     */
    void sendGRSimPacket(grSim_Packet const & packet);
    /**
     * Sends one single GRSim packet using GRSimCommander's
     * UDP socket.
     */
    void sendGRSimCommand(const roboteam_msgs::RobotCommand & _msg);

    /**
     * Sends multiple GRSim commands in one batch to GRSim.
     */
    void sendMultipleGRSimCommands(const std::vector<roboteam_msgs::RobotCommand> & msgs);

    /**
     * Sends multiple GRSim commands in one batch to GRSim depending on
     * if they're available in the buffer.
     */
    void sendMultipleGRSimCommands(const RobotCommandBuffer & msgs);

    /**
     * Returns true if a command for id is in the buffer.
     */
    bool hasMsgForID(int const id);

    /**
     * Returns the number of messages queued.
     */
    int getMsgsQueued();

    /**
     * Returns the statistics of this GRSimCommander. Also resets
     * statistics counters.
     */
    Stats consumeStatistics();

    /**
     * Returns true if batching is on.
     */
    bool isBatch();

    /**
     * Turns batching on or off.
     */
    void setBatch(bool batch);

private:
    /**
     * Updates the threshold and resets the appropriate variables.
     */
    void updateThreshold();

    /**
     * Flushes the buffer, records statistics and resets appropriate variables.
     */
    void flush();

    /**
     * The socket used for communicating with GRSim.
     */
    QUdpSocket udpsocket;

    /**
     * Used for temporary serialization.
     * If the submitted message is bigger than 1024 bytes
     * a heap allocation is done.
     */
    std::array<char, 1024> packetBuffer;

    /**
     * Stores the batching state.
     */
    bool batch;

    /**
     * For economically getting environment variables.
     */
    SlowParam<std::string> colorParam;
    SlowParam<std::string> grsim_ip;
    SlowParam<int>         grsim_port; 

    // Batch variables
    RobotCommandBuffer robotCommands;
    uint32_t msgReceivedBits;
    uint32_t robotsSeenBits;
    int threshold;
    int drops;
    TimePoint lastBatchTime;

    // Statistics vars
    std::array<double, HISTORY_LEN> efficiency;
    int efficiencyIndex;
    int numForcedFlushes;

} ;

void addRobotCommandToPacket(grSim_Packet & packet, roboteam_msgs::RobotCommand const & msg);

}
