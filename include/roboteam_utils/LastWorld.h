#pragma once

// TODO: WorldAndGeomCallbackCreator could be facored into it's own file
// Therewith losing the next two includes for every condition/skill/tactic!
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(World);
ROS_DECLARE_MESSAGE(WorldBall);
ROS_DECLARE_MESSAGE(GeometryFieldSize);
ROS_DECLARE_MESSAGE(GeometryData);

} // roboteam_msgs

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"

namespace rtt {

/**
 * \class LastWorld
 * \brief Utility class to keep the last state of the world.
 */
class LastWorld {
    public:
    /**
     * \brief Updates the world state.
     *        To be called on receiving a new world message.
     */
    static void callback_world_state(const roboteam_msgs::WorldConstPtr& world);
    /**
     * \brief Updates the field geometry.
     *        To be called on receiving a new geometry message.
     */
    static void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry);

    static roboteam_msgs::World get();
    static void set(roboteam_msgs::World world);

    static roboteam_msgs::GeometryFieldSize get_field();
    static void set_field(roboteam_msgs::GeometryFieldSize field);

    /**
     * \brief Call to check if a world message has been received.
     *
     * After this returns true, it will never return false again.
     *
     * When starting a node that needs a world message to function
     * this can be used as follows:
     * ```
     * while (!LastWorld::receivedFirstWorldMsg()) {
     *    ros::spinOnce();
     * }
     * ```
     *
     * For convenience one can also use `wait_for_first_messages()`.
     */
    static bool have_received_first_world();
    /**
     * \brief Call to check if a geometry message has been received.
     *
     * After this returns true, it will never return false again.
     */
    static bool have_received_first_geom();

    /**
     * \brief Convenience function that will return when both
     *        the first world and geometry messages have been received.
     *
     * Also returns when `ros::ok()` returns false.
     */
    static void wait_for_first_messages();

    /**
     * \brief Cives the predicted position off the ball in `t` seconds.
     * \param t Number of seconds from now to give the prediction for.
     */
    static Vector2 predictBallPos(double t);

    /**
     * \brief Returns the center of our goals back line.
     */
    static Vector2 get_our_goal_center();
    /**
     * \brief Returns the center of their goals back line.
     */
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
