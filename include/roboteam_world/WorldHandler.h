#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H
#include <roboteam_proto/Publisher.h>
#include <roboteam_proto/Subscriber.h>
#include <roboteam_proto/World.pb.h>
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>

#include <net/robocup_ssl_client.h>
#include <utility>

#include "filters/KalmanFilter.h"
#include "filters/WorldFilter.h"

namespace world {

class WorldHandler {
   private:
    proto::Publisher<proto::World> *world_pub;
    proto::Publisher<proto::SSL_Referee> *ref_pub;
    proto::Publisher<proto::SSL_GeometryData> *geom_pub;

    double lastPacketTime;  // seconds
    std::unique_ptr<WorldFilter> worldFilter;
    RoboCupSSLClient *vision_client;
    RoboCupSSLClient *refbox_client;

   public:
    WorldHandler() = default;

    /*
     * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
     */
    void init();
    void start();
    void handleVisionPackets(proto::SSL_WrapperPacket &vision_packet);
    void handleRefboxPackets(proto::SSL_Referee &ref_packet) const;
    void setupSSLClients();
};

}  // namespace world
#endif