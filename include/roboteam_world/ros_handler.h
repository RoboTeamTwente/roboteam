#pragma once

#include "DetectionFrame.pb.h"
#include "World.pb.h"
#include "roboteam_world/world/world_dummy.h"
#include "roboteam_world/world/filtered_world.h"
#include "kalman/kalmanFilter.h"

namespace rtt {

    class RosHandler {

    private:
//        ros::NodeHandle nh;
//        ros::Subscriber vision_sub;
//        ros::Publisher world_pub;
//        ros::ServiceServer reset_srv;
//        ros::ServiceServer tracker_srv;
        
        WorldBase* world;
        bool kalman;
    public:
        RosHandler() = default;
        void init(WorldBase* _world);
        void kalmanLoop();
        /**
         * Reads the configuration from the parameter server.
         * Updates the configuration of the world and calls a reset.
         */
        void update_config();

        void setKalman(bool on);
        void detection_callback(const roboteam_proto::DetectionFrame msg);
    //    bool reset_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        kalmanFilter KF;
    };

}
