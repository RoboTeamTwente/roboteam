#pragma once

#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(World);
ROS_DECLARE_MESSAGE(WorldBall);
ROS_DECLARE_MESSAGE(GeometryFieldSize);
ROS_DECLARE_MESSAGE(GeometryData);

} // roboteam_msgs

// #include "roboteam_msgs/World.h"
// #include "roboteam_msgs/WorldBall.h"
// #include "roboteam_msgs/GeometryFieldSize.h"
// #include "roboteam_msgs/GeometryData.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"

namespace rtt {

using namespace roboteam_utils;

class LastWorld {
    public:
    static void callback_world_state(const roboteam_msgs::WorldConstPtr& world);
    static void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry);

    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);

    static roboteam_msgs::GeometryFieldSize get_field();
    static void set_field(roboteam_msgs::GeometryFieldSize field);

    static bool have_received_first_world();
    static bool have_received_first_geom();

    static void wait_for_first_messages();

    static Vector2 predictBallPos(double t);

    static Vector2 get_our_goal_center();
    static Vector2 get_their_goal_center();

    private:
    static roboteam_msgs::World lastWorld;
    static roboteam_msgs::GeometryFieldSize field;

    static bool received_first_world;
    static bool received_first_geom;

};

// Can be placed in a main() to set up the wrld and geom callbacks
// DON'T MAKE SUBSCRIBERS PUBLISHER STATIC/GLOBAL! You'll get a weird
// exception that's hard to debug (probably).

class WorldAndGeomCallbackCreator {
public:
    WorldAndGeomCallbackCreator();

private:
    ros::NodeHandle n;
    ros::Subscriber worldSubscriber;
    ros::Subscriber geomSubscriber;

} ;

}
