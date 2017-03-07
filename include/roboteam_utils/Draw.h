#pragma once

#include "ros/ros.h"

#include <vector>

#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_msgs/DebugLine.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/Color.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * \class Draw
 * \brief Used to send draw commands to the field view.
 *
 * Used to send draw commands to the field view.
 * Is only active when the define `RTT_CMAKE_ENABLE_DEBUG_GRAPHICS` is true.
 *
 * Every point or line needs to be supplied a name.
 * When a draw command is sent with the same name,
 * it will override the last command with that name.
 * Names are seperate for points and lines.
 *
 * The color of the drawn objects can be set by calling `setColor()`,
 * after which all new objects will have this color.
 */
class Draw {

public:
    /**
     * \brief Creates the point and line publishers needed for drawing.
     */
    Draw();
    /**
     * \brief Draws a single line.
     * \param name The unique name for this line.
     * \param start The starting point of the line.
     * \param stop The end point of the line.
     */
    void drawLine(std::string name, roboteam_utils::Vector2 start, roboteam_utils::Vector2 stop);
    /**
     * \brief Draws a line consisting of multiple points.
     * \param name The unique name for this line.
     * \param points The points between which to draw the line, in the order supplied.
     */
    void drawLine(std::string name, std::vector<roboteam_utils::Vector2> points);
    /**
     * \brief Removes the line with the supplied name.
     * \param name The name of the line to remove.
     */
    void removeLine(std::string name);
    /**
     * \brief Draws a point.
     * \param name The unique name for this point.
     * \param point The location of the point.
     */
    void drawPoint(std::string name, roboteam_utils::Vector2 point);
    /**
     * \brief Removes the point with the supplied name.
     * \param name The name of the line to remove.
     */
    void removePoint(std::string name);
    /**
     * \brief Change the color for future draw commands.
     */
    void setColor(int r, int g, int b);

private:
    ros::NodeHandle n;
    ros::Publisher debugPub;
    ros::Publisher debugPubPoint;
    roboteam_msgs::Color color;
};

} // rtt
