#ifndef ROBOTEAM_ROBOTHUB_GRSIM_H
#define ROBOTEAM_ROBOTHUB_GRSIM_H
#include <array>
#include <QtNetwork>
#include <string>
#include <chrono>

#include "roboteam_proto/RobotCommand.pb.h"
#include "roboteam_proto/grSim_Commands.pb.h"
#include "roboteam_proto/grSim_Packet.pb.h"

namespace rtt {
namespace robothub {

class GRSimCommander {
public:
    /**
     * Buffer type which keeps all received robot commands
     */
    using RobotCommandBuffer = std::array<std::shared_ptr<proto::RobotCommand>, 16>;

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
    explicit GRSimCommander(bool batch = true);

    /**
     * Submits a GRSim command for sending to the simulator.
     * If batching is enabled it might not be sent immediately.
     * If batching is disabled it's immediately sent.
     */
    void queueGRSimCommand(const proto::RobotCommand& msg);

    /**
     * Sends a GRSim packet to the simulator using
     * GRSimCommander's UDP socket.
     */
    void sendGRSimPacket(proto::grSim_Packet const& packet);
    /**
     * Sends one single GRSim packet using GRSimCommander's
     * UDP socket.
     */
    void sendGRSimCommand(const proto::RobotCommand& _msg);

    /**
     * Sends multiple GRSim commands in one batch to GRSim.
     */
    void sendMultipleGRSimCommands(const std::vector<proto::RobotCommand>& msgs);

    /**
     * Sends multiple GRSim commands in one batch to GRSim depending on
     * if they're available in the buffer.
     */
    void sendMultipleGRSimCommands(const RobotCommandBuffer& msgs);

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
    void setColor(bool yellow);
  void setGrsim_ip(const std::string &grsim_ip);
  void setGrsim_port(quint16 grsim_port);

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
    bool isYellow = true;
    std::string grsim_ip = "127.0.0.1";

    quint16 grsim_port = 20011;

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

};

void addRobotCommandToPacket(proto::grSim_Packet& packet, proto::RobotCommand const& msg);

} // robothub
} // rtt

#endif //ROBOTEAM_ROBOTHUB_GRSIM_H

