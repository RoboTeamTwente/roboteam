#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"

namespace rtt {

    /// Initiate the world ros_handler
    void RosHandler::init(rtt::WorldBase* _world) {
        world = _world;

        // TODO: sub to geometry
        // Subscribe to the vision_detection input. ROS
      //  vision_sub = nh.subscribe(TOPIC_DETECTION, 1000, &RosHandler::detection_callback, this);

        // Advertise the world output. ROS
       // world_pub = nh.advertise<roboteam_proto::World>(TOPIC_WORLD_STATE, 1);

        // Advertise the reset service.
       // reset_srv = nh.advertiseService(SERVICE_WORLD_RESET, &RosHandler::reset_callback, this);

    }

    void RosHandler::kalmanLoop() {
        // double TICKRATE=1/TIMEDIFF;
        // ros::Rate rate(TICKRATE);
//        while (ros::ok()) {
//            ros::spinOnce();
//            KF.kalmanUpdate();
//            roboteam_proto::World kmWorld = KF.getWorld();
//            world_pub.publish(kmWorld);
//            rate.sleep();
//        }
    }
    void RosHandler::setKalman(bool on) {
        kalman=on;
    }


    /// Callback function for /vision_detection in ros_handler
    void RosHandler::detection_callback(const roboteam_proto::DetectionFrame msg) {
//        if (kalman) {
//            KF.newFrame(msg);
//            return;
//        }
//        world->detection_callback(msg);
//
//        if (auto * fworld = dynamic_cast<FilteredWorld*>(world)) {
//            // Filtered world! Special case that shit
//            if (auto worldOpt = fworld->consumeMsg()) {
//                world_pub.publish(*worldOpt);
//            }
//        } else {
//            // Generic world approach
//            // Where you can never be sure if this message is the new one
//            roboteam_proto::World world_msg = world->as_message();
//
//            world_pub.publish(world_msg);
//        }
    }


//    bool RosHandler::reset_callback(
//            std_srvs::Empty::Request& req,
//            std_srvs::Empty::Response& res) {
//
//        world->reset();
//
//        return true;
//    }


}
